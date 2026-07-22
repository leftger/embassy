//! Framebuffer storage for NeoChrom rendering targets.

use aligned::{A32, Aligned};

use crate::color::Rgba8888;

/// RGBA8888 framebuffer suitable as a NemaGFX destination texture.
///
/// The third const generic `N` must equal `W * H` (number of pixels).
pub struct FrameBuffer<const W: u32, const H: u32, const N: usize> {
    pixels: Aligned<A32, [u32; N]>,
}

impl<const W: u32, const H: u32, const N: usize> FrameBuffer<W, H, N> {
    /// Create a zero-initialized framebuffer.
    pub const fn new() -> Self {
        Self {
            pixels: Aligned([0; N]),
        }
    }

    /// Framebuffer width in pixels.
    #[inline]
    pub const fn width(&self) -> u32 {
        W
    }

    /// Framebuffer height in pixels.
    #[inline]
    pub const fn height(&self) -> u32 {
        H
    }

    /// Physical base address for [`crate::ffi::nema_gfx::nema_bind_dst_tex`].
    #[inline]
    pub fn phys_addr(&self) -> usize {
        self.pixels.as_ptr() as usize
    }

    /// Mutable view of pixel data (native-endian RGBA8888 words).
    #[inline]
    pub fn pixels_mut(&mut self) -> &mut [u32] {
        &mut *self.pixels
    }

    /// Fill the CPU-side buffer without using the GPU (debug / fallback).
    pub fn fill_cpu(&mut self, color: Rgba8888) {
        self.pixels_mut().fill(color.bits());
    }
}

impl<const W: u32, const H: u32, const N: usize> Default for FrameBuffer<W, H, N> {
    fn default() -> Self {
        Self::new()
    }
}
