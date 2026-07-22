//! RGBA color in the format expected by NemaGFX clear/fill APIs.

/// Opaque 32-bit RGBA color (`0xAARRGGBB`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
pub struct Rgba8888(pub u32);

impl Rgba8888 {
    /// Construct from 8-bit RGBA channels.
    #[inline]
    pub const fn new(r: u8, g: u8, b: u8, a: u8) -> Self {
        Self(((a as u32) << 24) | ((r as u32) << 16) | ((g as u32) << 8) | (b as u32))
    }

    /// Solid red.
    pub const RED: Self = Self::new(0xFF, 0x00, 0x00, 0xFF);
    /// Solid green.
    pub const GREEN: Self = Self::new(0x00, 0xFF, 0x00, 0xFF);
    /// Solid blue.
    pub const BLUE: Self = Self::new(0x00, 0x00, 0xFF, 0xFF);
    /// Opaque black.
    pub const BLACK: Self = Self::new(0x00, 0x00, 0x00, 0xFF);
    /// Opaque white.
    pub const WHITE: Self = Self::new(0xFF, 0xFF, 0xFF, 0xFF);

    /// Value passed to [`crate::ffi::nema_gfx::nema_clear`].
    #[inline]
    pub const fn bits(self) -> u32 {
        self.0
    }
}
