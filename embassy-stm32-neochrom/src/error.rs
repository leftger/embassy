//! NeoChrom driver errors.

/// NemaGFX / NeoChrom runtime error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Waiting on a command list failed.
    CommandListWait,
}

/// Initialization failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InitError {
    /// GPU2D HAL initialization failed.
    Gpu2d,
    /// `nema_init()` returned an error.
    NemaGfx,
}
