//! Link-only smoke test.
//!
//! Forces the linker to pull `libvenc` out of the archive and resolve every
//! undefined `EWL*` symbol against the Rust platform implementation. It is not
//! meant to run; building the crate as a binary is what exercises the final
//! link step.

#![no_std]
#![no_main]

use embassy_stm32_venc::venc;

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {}
}

static mut POOL: [u8; 0x1000] = [0; 0x1000];

#[unsafe(no_mangle)]
pub extern "C" fn _start() -> ! {
    embassy_stm32_venc::platform::set_pool(unsafe { &mut *(&raw mut POOL) });

    // Reference a symbol that lives in the archive, so the linker must pull
    // objects out of `libvenc.a`.
    let mut cfg = venc::H264EncConfig::default();
    let mut inst: venc::H264EncInst = core::ptr::null();
    unsafe {
        let _ = venc::H264EncInit(&mut cfg, &mut inst);
        let _ = venc::JpegEncInit(core::ptr::null(), &mut inst);
        // Pull the frame path in too, so EWLWaitHwRdy / EWLWriteReg and the
        // rest of the EWL seam are linked and resolved.
        let _ = venc::H264EncStrmEncode(
            inst,
            core::ptr::null(),
            core::ptr::null_mut(),
            None,
            None,
            core::ptr::null_mut(),
        );
    }
    loop {}
}
