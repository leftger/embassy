//! NeoChrom GPU driver using NemaGFX.

use crate::color::Rgba8888;
use crate::error::{Error, InitError};
use crate::ffi::nema_gfx::{
    nema_bind_dst_tex, nema_cl_bind, nema_cl_create, nema_cl_submit, nema_cl_wait, nema_clear, nema_init,
    NEMA_RGBA8888,
};
use crate::framebuffer::FrameBuffer;

/// NeoChrom GPU context.
///
/// Call [`NeoChrom::new`] once after GPU2D clocks are enabled. With the default
/// `stub-gpu2d` feature, a stub HAL is used for link/CI testing.
pub struct NeoChrom {
    _init: (),
}

impl NeoChrom {
    /// Initialize NemaGFX and the platform HAL.
    pub fn new() -> Result<Self, InitError> {
        Self::init_gpu2d()?;

        let status = unsafe { nema_init() };
        if status != 0 {
            return Err(InitError::NemaGfx);
        }

        Ok(Self { _init: () })
    }

    /// Clear `framebuffer` to `color` using the GPU command list path.
    pub fn clear<const W: u32, const H: u32, const N: usize>(
        &mut self,
        framebuffer: &FrameBuffer<W, H, N>,
        color: Rgba8888,
    ) -> Result<(), Error> {
        unsafe {
            nema_bind_dst_tex(framebuffer.phys_addr(), W, H, NEMA_RGBA8888, -1);

            let mut cl = nema_cl_create();
            nema_cl_bind(&mut cl);
            nema_clear(color.bits());
            nema_cl_submit(&mut cl);
            if nema_cl_wait(&mut cl) != 0 {
                return Err(Error::CommandListWait);
            }
        }

        Ok(())
    }

    fn init_gpu2d() -> Result<(), InitError> {
        #[cfg(feature = "stub-gpu2d")]
        {
            nema_gfx_hal::gpu2d_init_stub().map_err(|_| InitError::Gpu2d)
        }
        #[cfg(not(feature = "stub-gpu2d"))]
        {
            // Real GPU2D init must happen in board support before `NeoChrom::new`.
            Ok(())
        }
    }
}
