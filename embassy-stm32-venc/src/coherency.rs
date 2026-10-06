//! CPU data-cache maintenance for VENC-visible buffers.
//!
//! The Hantro encoder reads its input pictures and writes its bitstream through
//! the ASIC's own bus masters. When those buffers live in Cortex-M55-cacheable
//! memory the CPU cache must be cleaned before the ASIC reads them and
//! invalidated before the CPU reads results back.
//!
//! The `cortex-m` crate only exposes cache maintenance for ARMv7-M, so the
//! (architecturally identical) ARMv8-M cache-maintenance registers are written
//! directly here, mirroring `LL_ATON_Cache_MCU_*` from the ST examples.

const DCACHE_LINE: u32 = 32;
/// Data cache invalidate by MVA to PoC.
const SCB_DCIMVAC: u32 = 0xE000_EF5C;
/// Data cache clean by MVA to PoC.
const SCB_DCCMVAC: u32 = 0xE000_EF68;
/// Data cache clean and invalidate by MVA to PoC.
const SCB_DCCIMVAC: u32 = 0xE000_EF70;

fn cache_op(op_reg: u32, start: u32, len: u32) {
    if len == 0 {
        return;
    }
    cortex_m::asm::dsb();
    let mut addr = start & !(DCACHE_LINE - 1);
    let end = start.wrapping_add(len);
    while addr < end {
        unsafe { core::ptr::write_volatile(op_reg as *mut u32, addr) };
        addr += DCACHE_LINE;
    }
    cortex_m::asm::dsb();
    cortex_m::asm::isb();
}

/// Clean (write back) `[start, start + len)` so the ASIC sees CPU writes.
pub fn clean_range(start: u32, len: u32) {
    cache_op(SCB_DCCMVAC, start, len);
}

/// Invalidate `[start, start + len)` so the CPU sees ASIC writes. Partially
/// covered cache lines at the edges are cleaned and invalidated instead of
/// plainly invalidated, to avoid discarding neighbouring CPU data.
pub fn invalidate_range(start: u32, len: u32) {
    if len == 0 {
        return;
    }
    if start % DCACHE_LINE != 0 {
        cache_op(SCB_DCCIMVAC, start & !(DCACHE_LINE - 1), 1);
    }
    let end = start + len;
    if end % DCACHE_LINE != 0 {
        cache_op(SCB_DCCIMVAC, end & !(DCACHE_LINE - 1), 1);
    }
    cache_op(SCB_DCIMVAC, start, len);
}

/// Clean and invalidate `[start, start + len)`.
pub fn clean_invalidate_range(start: u32, len: u32) {
    cache_op(SCB_DCCIMVAC, start, len);
}
