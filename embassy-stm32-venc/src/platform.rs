//! EWL (Encoder Wrapper Layer) platform implementation.
//!
//! The Hantro encoder software stack calls into a small OS/platform abstraction
//! — the EWL — for memory, register access, timing and hardware synchronisation.
//! ST ships reference implementations for FreeRTOS, ThreadX and bare-metal
//! (`ewl_impl.c`, `nema_hal_baremetal.c` equivalents). This module is the embassy
//! equivalent: the same twenty entry points, backed by `embassy-time`, a static
//! arena ([`crate::alloc`]) and the `VENC` peripheral registers from
//! `embassy-stm32::pac`.
//!
//! Nothing here uses an RTOS or the ST HAL. The only symbol the prebuilt
//! `libvenc` leaves unresolved is this set; everything else resolves inside the
//! archive.
//!
//! # Threading
//!
//! EWL is a single-instance, non-reentrant interface (ST: "Only one instance is
//! supported"). Calls must be serialised by the driver; `EWLWaitHwRdy` blocks
//! the calling context until the ASIC finishes, so run encoding on its own
//! executor or a blocking thread.

use core::ffi::c_void;

use embassy_stm32::pac;

use crate::alloc;
use crate::ffi::venc;

/// `VENC_REG(x)` index of the interrupt/status register (`VENC_REG(1)`).
const REG_IRQ_STATUS: usize = 1;
/// `VENC_REG(21)` — slice-ready counter lives in the upper half.
const REG_SLICE_READY: usize = 21;
/// Byte offset of the HW-synthesis "fuse 2" register (`BASE_HWFuse2`).
const BASE_HW_FUSE2: u32 = 0x4a0;
/// Byte offset of the instant-input/handshake register (`BASE_HEncInstantInput`).
const BASE_HENC_INSTANT_INPUT: u32 = 0x7c4;

/// Line-buffer-done status write used to acknowledge input line-buffer IRQs.
const LINE_BUFFER_ACK: u32 = 1 << 9;

/// Timeout for a single frame, matching ST's `EWL_TIMEOUT` (ms).
const EWL_TIMEOUT_MS: u64 = 100;

/// Base address of the dedicated VENC SRAM (128 KiB), from the N6 memory map.
pub const VENCRAM_BASE: u32 = 0x2440_0000;
/// Size of the dedicated VENC SRAM.
pub const VENCRAM_SIZE: u32 = 131_072;

/// Opaque EWL instance handed to the encoder and passed back on every call.
///
/// The encoder only stores and forwards the pointer, so a zero-sized singleton
/// is enough: it gives a stable, unique, non-null address without carrying
/// state that the globals below already hold.
#[repr(C)]
pub struct Instance {
    _private: [u8; 0],
}

static mut INSTANCE: Instance = Instance { _private: [] };

fn instance_ptr() -> *const c_void {
    (&raw const INSTANCE).cast()
}

#[inline]
fn reg_read_idx(idx: usize) -> u32 {
    pac::VENC.swreg(idx).read().0
}

#[inline]
fn reg_write_idx(idx: usize, val: u32) {
    pac::VENC.swreg(idx).write_value(pac::venc::regs::Swreg(val));
}

#[inline]
fn reg_read(byte_offset: u32) -> u32 {
    reg_read_idx((byte_offset >> 2) as usize)
}

#[inline]
fn reg_write(byte_offset: u32, val: u32) {
    reg_write_idx((byte_offset >> 2) as usize, val)
}

/// Provide the arena used for all encoder allocations.
///
/// Must be called before creating an encoder. The buffer must live for the rest
/// of the program and be in memory the VENC can access (the ASIC reads and
/// writes most of these buffers directly). ST's default pool size is 1.6 MiB,
/// but the exact requirement depends on resolution and configuration.
pub fn set_pool(pool: &'static mut [u8]) {
    alloc::init(pool);
}

/// Read the encoder ASIC ID (VENC register 0).
pub fn asic_id() -> u32 {
    reg_read(0)
}

/// Human-readable hardware capability snapshot, decoded like
/// `EWLReadAsicConfig`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct HwConfig {
    pub max_encoded_width: u32,
    pub h264: bool,
    pub jpeg: bool,
    pub scaling: bool,
    pub rgb_input: bool,
    pub addr64: bool,
    pub denoise: bool,
    pub instant: bool,
    pub bus_width: u32,
}

/// Decode the ASIC capability registers (VENC 63 and 296).
pub fn hw_config() -> HwConfig {
    let c = reg_read(63 * 4);
    let c2 = reg_read(296 * 4);
    HwConfig {
        max_encoded_width: c & ((1 << 12) - 1),
        h264: (c >> 27) & 1 != 0,
        jpeg: (c >> 25) & 1 != 0,
        rgb_input: (c >> 28) & 1 != 0,
        scaling: (c >> 30) & 1 != 0,
        bus_width: ((c >> 12) & 0xf) * 32,
        addr64: (c2 >> 31) & 1 != 0,
        denoise: (c2 >> 30) & 1 != 0,
        instant: (c2 >> 26) & 1 != 0,
    }
}

// ───────────────────────────── EWL entry points ─────────────────────────────

#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLReadAsicID() -> u32 {
    reg_read(0)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLReadAsicConfig() -> venc::EWLHwConfig_t {
    let c = reg_read(63 * 4);
    let c2 = reg_read(296 * 4);
    let mut cfg: venc::EWLHwConfig_t = unsafe { core::mem::zeroed() };
    cfg.maxEncodedWidth = c & ((1 << 12) - 1);
    cfg.h264Enabled = (c >> 27) & 1;
    cfg.jpegEnabled = (c >> 25) & 1;
    cfg.vp8Enabled = (c >> 26) & 1;
    cfg.vsEnabled = (c >> 24) & 1;
    cfg.rgbEnabled = (c >> 28) & 1;
    cfg.searchAreaSmall = (c >> 29) & 1;
    cfg.scalingEnabled = (c >> 30) & 1;
    cfg.busType = (c >> 20) & 0xf;
    cfg.synthesisLanguage = (c >> 16) & 0xf;
    cfg.busWidth = (c >> 12) & 0xf;
    cfg.addr64Support = (c2 >> 31) & 1;
    cfg.dnfSupport = (c2 >> 30) & 1;
    cfg.rfcSupport = (c2 >> 28) & 3;
    cfg.enhanceSupport = (c2 >> 27) & 1;
    cfg.instantSupport = (c2 >> 26) & 1;
    cfg.svctSupport = (c2 >> 25) & 1;
    cfg.inAxiIdSupport = (c2 >> 24) & 1;
    cfg.inLoopbackSupport = (c2 >> 23) & 1;
    cfg.irqEnhanceSupport = (c2 >> 22) & 1;
    cfg
}

/// Set up the EWL instance. The pool must already be installed with
/// [`set_pool`]; the peripheral clocks and VENCRAM are brought up by the driver.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLInit(param: *mut venc::EWLInitParam_t) -> *const c_void {
    if param.is_null() {
        return core::ptr::null();
    }
    if !alloc::is_initialized() {
        error!("venc: EWLInit called before set_pool()");
        return core::ptr::null();
    }
    let client = unsafe { (*param).clientType };
    if client != venc::EWL_CLIENT_TYPE_H264_ENC && client != venc::EWL_CLIENT_TYPE_JPEG_ENC {
        error!("venc: EWLInit unsupported client type {}", client);
        return core::ptr::null();
    }
    instance_ptr()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLRelease(_inst: *const c_void) -> i32 {
    venc::EWL_OK as i32
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLReserveHw(_inst: *const c_void) -> i32 {
    venc::EWL_OK as i32
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLReleaseHw(_inst: *const c_void) {}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLMallocLinear(_inst: *const c_void, size: u32, info: *mut venc::EWLLinearMem_t) -> i32 {
    if info.is_null() {
        return venc::EWL_ERROR;
    }
    let p = alloc::alloc(size as usize);
    if p.is_null() {
        error!("venc: EWLMallocLinear failed for {} bytes", size);
        return venc::EWL_ERROR;
    }
    unsafe {
        (*info).virtualAddress = p as *mut u32;
        (*info).size = (size + 7) & !7;
        // No MMU: the bus sees the same address as the CPU.
        (*info).busAddress = p as u32;
    }
    venc::EWL_OK as i32
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLFreeLinear(_inst: *const c_void, info: *mut venc::EWLLinearMem_t) {
    if info.is_null() {
        return;
    }
    let ptr = unsafe { (*info).virtualAddress };
    alloc::free(ptr as *mut u8);
    if !info.is_null() {
        unsafe { *info = core::mem::zeroed() };
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLMallocRefFrm(inst: *const c_void, size: u32, info: *mut venc::EWLLinearMem_t) -> i32 {
    unsafe { EWLMallocLinear(inst, size, info) }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLFreeRefFrm(inst: *const c_void, info: *mut venc::EWLLinearMem_t) {
    unsafe { EWLFreeLinear(inst, info) }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLmalloc(n: u32) -> *mut c_void {
    alloc::alloc(n as usize) as *mut c_void
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLcalloc(n: u32, s: u32) -> *mut c_void {
    let total = (n as usize) * (s as usize);
    let p = alloc::alloc(total);
    if !p.is_null() {
        unsafe { core::ptr::write_bytes(p, 0, total) };
    }
    p as *mut c_void
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLfree(p: *mut c_void) {
    alloc::free(p as *mut u8);
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLmemcpy(d: *mut c_void, s: *const c_void, n: u32) -> *mut c_void {
    unsafe { core::ptr::copy_nonoverlapping(s as *const u8, d as *mut u8, n as usize) };
    d
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLmemset(d: *mut c_void, c: i32, n: u32) -> *mut c_void {
    unsafe { core::ptr::write_bytes(d as *mut u8, c as u8, n as usize) };
    d
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLWriteReg(_inst: *const c_void, offset: u32, val: u32) {
    reg_write(offset, val);
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLReadReg(_inst: *const c_void, offset: u32) -> u32 {
    reg_read(offset)
}

/// `EWLWriteReg`, `EWLEnableHW` and `EWLDisableHW` are documented as identical
/// in the IP integration guide, so they share an implementation.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLEnableHW(_inst: *const c_void, offset: u32, val: u32) {
    reg_write(offset, val);
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLDisableHW(_inst: *const c_void, offset: u32, val: u32) {
    reg_write(offset, val);
}

/// The input line buffer lives in the dedicated VENC SRAM.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLGetInputLineBufferBase(_instance: *const c_void, info: *mut venc::EWLLinearMem_t) -> i32 {
    if info.is_null() {
        return venc::EWL_ERROR;
    }
    unsafe {
        (*info).virtualAddress = VENCRAM_BASE as *mut u32;
        (*info).size = VENCRAM_SIZE;
        (*info).busAddress = VENCRAM_BASE;
    }
    venc::EWL_OK as i32
}

/// Block until the ASIC raises a completion/status interrupt.
///
/// Mirrors the polling branch of ST's `ewl_impl.c`: the ASIC status register is
/// polled, status bits are cleared with the correct (write-one vs write-status)
/// convention, and the input line-buffer handshake is acknowledged separately.
/// `embassy-time` provides the timeout, which bounds the wait if the ASIC never
/// signals.
///
/// The wait busy-polls rather than using `wfi`: `embassy-time` only arms its
/// timer interrupt while an alarm is pending, so with nothing scheduled `wfi`
/// would put the core to sleep past the ASIC completion (observed on hardware
/// as a hang on the first `H264EncStrmEncode`). Encoding is a short, bursty
/// busy period, so polling is the right trade-off here.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn EWLWaitHwRdy(_inst: *const c_void, slices_ready: *mut u32) -> i32 {
    let clr_by_write1 = reg_read(BASE_HW_FUSE2) & venc::HWCFGIRQCLEARSUPPORT != 0;
    let start = embassy_time::Instant::now();
    let mut prev_slices = 0u32;

    loop {
        let irq = reg_read_idx(REG_IRQ_STATUS);

        if !slices_ready.is_null() {
            let s = (reg_read_idx(REG_SLICE_READY) >> 16) & 0xFF;
            unsafe { *slices_ready = s };
        }

        let handshake = reg_read(BASE_HENC_INSTANT_INPUT) & (1 << 29) != 0;
        if irq == venc::ASIC_STATUS_LINE_BUFFER_DONE && handshake {
            reg_write_idx(REG_IRQ_STATUS, LINE_BUFFER_ACK);
            continue;
        }

        if irq & venc::ASIC_STATUS_ALL != 0 {
            let base = irq & !(venc::ASIC_STATUS_SLICE_READY | venc::ASIC_IRQ_LINE);
            let clr = if clr_by_write1 {
                venc::ASIC_STATUS_SLICE_READY | venc::ASIC_IRQ_LINE
            } else {
                base
            };
            reg_write_idx(REG_IRQ_STATUS, clr);
            return venc::EWL_HW_WAIT_OK as i32;
        }

        if !slices_ready.is_null() {
            let s = unsafe { *slices_ready };
            if s > prev_slices {
                return venc::EWL_HW_WAIT_OK as i32;
            }
            prev_slices = s;
        }

        if start.elapsed().as_millis() >= EWL_TIMEOUT_MS {
            warn!("venc: EWLWaitHwRdy timeout");
            return venc::EWL_HW_WAIT_TIMEOUT as i32;
        }

        core::hint::spin_loop();
    }
}
