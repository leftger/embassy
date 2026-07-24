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
    /// Initialize NemaGFX and the platform HAL using the in-tree GPU2D stub.
    ///
    /// Only available with the default `stub-gpu2d` feature (CI / link tests).
    #[cfg(feature = "stub-gpu2d")]
    pub fn new() -> Result<Self, InitError> {
        nema_gfx_hal::gpu2d_init_stub().map_err(|_| InitError::Gpu2d)?;
        Self::finish_init()
    }

    /// Initialize NemaGFX using the real GPU2D peripheral.
    ///
    /// `peri`/`irq` are consumed to enable the GPU2D peripheral clock and
    /// interrupt via [`embassy_stm32::gpu2d::Gpu2d`]. Only available without
    /// the default `stub-gpu2d` feature.
    #[cfg(not(feature = "stub-gpu2d"))]
    pub fn new(
        peri: embassy_stm32::Peri<'static, embassy_stm32::peripherals::GPU2D>,
        irq: impl embassy_stm32::interrupt::typelevel::Binding<
            <embassy_stm32::peripherals::GPU2D as embassy_stm32::gpu2d::Instance>::Interrupt,
            embassy_stm32::gpu2d::InterruptHandler<embassy_stm32::peripherals::GPU2D>,
        > + 'static,
    ) -> Result<Self, InitError> {
        crate::gpu2d_bridge::init(peri, irq);
        Self::finish_init()
    }

    fn finish_init() -> Result<Self, InitError> {
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

    /// Fill a rectangle at (`x`, `y`) with dimensions `w`x`h` using the GPU.
    pub fn fill_rect<const W: u32, const H: u32, const N: usize>(
        &mut self,
        framebuffer: &FrameBuffer<W, H, N>,
        x: i32,
        y: i32,
        w: i32,
        h: i32,
        color: Rgba8888,
    ) -> Result<(), Error> {
        unsafe {
            use crate::ffi::nema_gfx::nema_fill_rect;
            nema_bind_dst_tex(framebuffer.phys_addr(), W, H, NEMA_RGBA8888, -1);

            let mut cl = nema_cl_create();
            nema_cl_bind(&mut cl);
            nema_fill_rect(x, y, w, h, color.bits());
            nema_cl_submit(&mut cl);
            if nema_cl_wait(&mut cl) != 0 {
                return Err(Error::CommandListWait);
            }
        }

        Ok(())
    }

    /// Draw a line from (`x0`, `y0`) to (`x1`, `y1`) using the GPU.
    pub fn draw_line<const W: u32, const H: u32, const N: usize>(
        &mut self,
        framebuffer: &FrameBuffer<W, H, N>,
        x0: i32,
        y0: i32,
        x1: i32,
        y1: i32,
        color: Rgba8888,
    ) -> Result<(), Error> {
        unsafe {
            use crate::ffi::nema_gfx::nema_draw_line;
            nema_bind_dst_tex(framebuffer.phys_addr(), W, H, NEMA_RGBA8888, -1);

            let mut cl = nema_cl_create();
            nema_cl_bind(&mut cl);
            nema_draw_line(x0, y0, x1, y1, color.bits());
            nema_cl_submit(&mut cl);
            if nema_cl_wait(&mut cl) != 0 {
                return Err(Error::CommandListWait);
            }
        }

        Ok(())
    }

    /// Fill a circle at (`cx`, `cy`) with radius `r` using the GPU.
    pub fn fill_circle<const W: u32, const H: u32, const N: usize>(
        &mut self,
        framebuffer: &FrameBuffer<W, H, N>,
        cx: i32,
        cy: i32,
        r: i32,
        color: Rgba8888,
    ) -> Result<(), Error> {
        unsafe {
            use crate::ffi::nema_gfx::nema_fill_circle;
            nema_bind_dst_tex(framebuffer.phys_addr(), W, H, NEMA_RGBA8888, -1);

            let mut cl = nema_cl_create();
            nema_cl_bind(&mut cl);
            nema_fill_circle(cx, cy, r, color.bits());
            nema_cl_submit(&mut cl);
            if nema_cl_wait(&mut cl) != 0 {
                return Err(Error::CommandListWait);
            }
        }

        Ok(())
    }

    /// Blit (copy) source framebuffer `src` into destination framebuffer `dst` at (`dst_x`, `dst_y`).
    pub fn blit<const DW: u32, const DH: u32, const DN: usize, const SW: u32, const SH: u32, const SN: usize>(
        &mut self,
        dst: &FrameBuffer<DW, DH, DN>,
        src: &FrameBuffer<SW, SH, SN>,
        dst_x: i32,
        dst_y: i32,
    ) -> Result<(), Error> {
        unsafe {
            use crate::ffi::nema_gfx::{nema_bind_src_tex, nema_blit, NEMA_TEX_BORDER};
            nema_bind_dst_tex(dst.phys_addr(), DW, DH, NEMA_RGBA8888, -1);
            nema_bind_src_tex(src.phys_addr(), SW, SH, NEMA_RGBA8888, -1, NEMA_TEX_BORDER);

            let mut cl = nema_cl_create();
            nema_cl_bind(&mut cl);
            nema_blit(dst_x, dst_y);
            nema_cl_submit(&mut cl);
            if nema_cl_wait(&mut cl) != 0 {
                return Err(Error::CommandListWait);
            }
        }

        Ok(())
    }
}
