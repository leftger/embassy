//! Minimal 8-byte-aligned first-fit allocator backing the EWL hooks.
//!
//! The Hantro encoder allocates through four EWL entry points:
//!
//! * `EWLMallocLinear` / `EWLFreeLinear` — contiguous buffers the ASIC reads or
//!   writes (reference frames, bitstream, CABAC context, size tables).
//! * `EWLMallocRefFrm` / `EWLFreeRefFrm` — frame buffers, same requirements.
//! * `EWLmalloc` / `EWLcalloc` / `EWLfree` — CPU-only scratch (instance structs,
//!   ROI maps, temporary CABAC table).
//!
//! All of them are served from one arena supplied by the caller
//! ([`crate::platform::set_pool`]) so there is a single place to place memory in
//! ASIC-visible RAM. Every block is 8-byte aligned, matching EWL's
//! `ALIGNMENT_INCR`.
//!
//! The allocator is not thread safe; the single VENC instance serialises all
//! encoder calls, exactly like ST's reference `ewl_impl.c`.

use core::ptr;

/// Minimum alignment / size granularity, matching EWL's `ALIGNMENT_INCR`.
pub(crate) const ALIGN: usize = 8;

const HEADER: usize = core::mem::size_of::<Header>();
/// Sentinel "no block" offset. The first block lives at offset 0, so `0` cannot
/// be used as NULL.
const NIL: usize = usize::MAX;

#[repr(C)]
struct Header {
    /// Payload size in bytes (excludes this header).
    size: usize,
    /// Absolute address of the next free block's header, or [`NIL`].
    next: usize,
}

struct Pool {
    /// Absolute start address of the arena.
    base: usize,
    /// Absolute end address (exclusive).
    end: usize,
    /// Head of the address-ordered free list (a `Header` address), or [`NIL`].
    free: usize,
    initialized: bool,
}

static mut POOL: Pool = Pool {
    base: 0,
    end: 0,
    free: NIL,
    initialized: false,
};

fn pool() -> &'static mut Pool {
    // SAFETY: the driver guarantees that encoder calls (and therefore pool
    // access) are serialised on a single execution context.
    unsafe { &mut *(&raw mut POOL) }
}

/// True once [`init`] has been called.
pub(crate) fn is_initialized() -> bool {
    pool().initialized
}

/// Initialise the arena. `arena` must live for the rest of the program, because
/// the encoder keeps pointers into it across calls.
pub(crate) fn init(arena: &'static mut [u8]) {
    let p = pool();
    let raw_start = arena.as_mut_ptr() as usize;
    let raw_end = raw_start + arena.len();

    // The first payload must be 8-byte aligned; align the base up and the end
    // down so both stay inside the caller's buffer.
    let base = (raw_start + ALIGN - 1) & !(ALIGN - 1);
    let end = raw_end & !(ALIGN - 1);
    assert!(end >= base + HEADER + ALIGN, "venc pool too small");

    p.base = base;
    p.end = end;
    p.free = base;
    p.initialized = true;

    // SAFETY: `base..end` is inside the caller-provided arena.
    unsafe {
        (base as *mut Header).write(Header {
            size: end - base - HEADER,
            next: NIL,
        })
    };
}

/// Allocate `size` bytes with 8-byte alignment. Returns NULL on exhaustion.
pub(crate) fn alloc(size: usize) -> *mut u8 {
    if size == 0 {
        return ptr::null_mut();
    }
    let p = pool();
    if !p.initialized {
        return ptr::null_mut();
    }

    let need = (size + ALIGN - 1) & !(ALIGN - 1);

    let mut prev = NIL;
    let mut cur = p.free;
    while cur != NIL {
        // SAFETY: every offset on the free list points at a valid `Header`.
        let h = cur as *mut Header;
        let hsize = unsafe { (*h).size };
        if hsize >= need {
            let split = hsize >= need + HEADER + ALIGN;
            if split {
                let rem = cur + HEADER + need;
                unsafe {
                    let next = (*h).next;
                    (rem as *mut Header).write(Header {
                        size: hsize - need - HEADER,
                        next,
                    });
                    (*h).size = need;
                }
                replace(p, prev, cur, rem);
            } else {
                let next = unsafe { (*h).next };
                replace(p, prev, cur, next);
            }
            // SAFETY: `cur` is the block we just took off the free list.
            return (cur + HEADER) as *mut u8;
        }
        prev = cur;
        cur = unsafe { (*h).next };
    }
    ptr::null_mut()
}

/// Replace `old` with `new` in the free list, keeping the address order.
fn replace(p: &mut Pool, prev: usize, old: usize, new: usize) {
    if prev == NIL {
        p.free = new;
    } else {
        // SAFETY: `prev` is a valid free-list header.
        unsafe { (*(prev as *mut Header)).next = new };
    }
    let _ = old;
}

/// Free a block previously returned by [`alloc`].
pub(crate) fn free(ptr: *mut u8) {
    if ptr.is_null() {
        return;
    }
    let p = pool();
    if !p.initialized {
        return;
    }

    let h = (ptr as usize) - HEADER;

    // Find the address-ordered insertion point.
    let mut prev = NIL;
    let mut cur = p.free;
    while cur != NIL && cur < h {
        prev = cur;
        cur = unsafe { (*(cur as *mut Header)).next };
    }

    // SAFETY: `h` is a header written by `alloc`.
    unsafe { (*(h as *mut Header)).next = cur };
    replace(p, prev, h, h);

    // Coalesce forward, then backward, so adjacent free blocks merge.
    coalesce(h);
    if prev != NIL {
        coalesce(prev);
    }
}

/// Merge `h` with the following block if they are contiguous.
fn coalesce(h: usize) {
    let hh = h as *mut Header;
    // SAFETY: `h` is a valid free-list header.
    let next = unsafe { (*hh).next };
    if next == NIL {
        return;
    }
    let h_end = h + HEADER + unsafe { (*hh).size };
    if h_end == next {
        unsafe {
            (*hh).size += HEADER + (*(next as *mut Header)).size;
            (*hh).next = (*(next as *mut Header)).next;
        }
    }
}
