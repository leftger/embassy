//! Embassy driver for the STM32N6 Video Encoder (VENC).
//!
//! The N6 VENC is a Hantro H.264 + JPEG encoder IP. ST ships the encoder as a
//! software stack (BSD-3-Clause) rather than a prebuilt library, so this crate
//! pairs the `stm32-bindings` FFI (`h264encapi.h` / `jpegencapi.h`) with a Rust
//! implementation of the Encoder Wrapper Layer ([`platform`]) — the OS/platform
//! seam the stack normally fills with FreeRTOS or ThreadX code.
//!
//! # Layout
//!
//! * [`ffi`] — raw bindgen declarations, one-to-one with the C API.
//! * [`platform`] — the twenty EWL entry points (`EWLInit`, `EWLMallocLinear`,
//!   `EWLWaitHwRdy`, …) implemented over `embassy-time`, a static arena and the
//!   `VENC` registers. No RTOS, no ST HAL.
//! * [`coherency`] — Cortex-M55 data-cache maintenance for ASIC-visible buffers.
//! * [`h264`] / [`jpeg`] — safe facades over the two encoders.
//!
//! # Usage
//!
//! ```rust,ignore
//! use embassy_stm32::peripherals::VENC;
//! use embassy_stm32_venc::{Venc, h264};
//!
//! static mut POOL: [u8; 0x190000] = [0; 0x190000];
//!
//! let venc = Venc::new(p.VENC, unsafe { &mut POOL });
//!
//! let mut enc = venc.h264(h264::Config::new(800, 480, 30))?;
//! let mut header = [0u8; 4096];
//! enc.stream_start(&mut header)?;
//! ```
//!
//! # Prerequisites
//!
//! * A pool large enough for the configured resolution (ST's default is
//!   `0x190000`) must be supplied to [`Venc::new`], in memory the ASIC can
//!   reach.
//! * The pool and all input/output buffers must be either non-cacheable or
//!   handled with [`coherency`]. The safe encoder methods already clean/invalidate
//!   the buffers they are given.
//! * `EWLWaitHwRdy` blocks the calling context until the ASIC finishes a frame,
//!   so encode from a dedicated executor or a blocking thread.
//!
//! # Note on the ported stack
//!
//! `Middlewares/Third_Party/VideoEncoder` is compiled unmodified except for
//! `H264TestId.c`, which is replaced by two no-op hooks in the bindings crate's
//! port shim (`stm32-bindings/stm32-bindings-gen/venc_port.c`) so that the
//! library does not pull in `stdio`. `H264EncTestCropping` and
//! `H264EncTestInputLineBuf` therefore do nothing.

#![no_std]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(unused_imports)]

#[macro_use]
mod fmt;

mod alloc;
mod error;

pub mod coherency;
pub mod ffi;
pub mod h264;
pub mod jpeg;
pub mod platform;

pub use error::Error;
pub use ffi::venc;

use embassy_stm32::peripherals;
use embassy_stm32::{Peri, pac};

/// Video Encoder driver.
///
/// Owning a `Venc` means the peripheral is clocked, VENCRAM is allocated to the
/// encoder and the EWL arena is installed. Encoders borrow it, so the pool and
/// peripheral outlive every encoder instance.
pub struct Venc<'d> {
    _peri: Peri<'d, peripherals::VENC>,
}

impl<'d> Venc<'d> {
    /// Bring up the encoder: enable the APB5 clock and reset, turn on the
    /// VENCRAM memory clock, hand VENCRAM to the encoder, and install `pool`
    /// as the EWL arena.
    ///
    /// `pool` must live for the rest of the program.
    pub fn new(peri: Peri<'d, peripherals::VENC>, pool: &'static mut [u8]) -> Self {
        // APB5 clock + reset (VENC is modelled as an RCC peripheral).
        embassy_stm32::rcc::enable_and_reset::<peripherals::VENC>();

        critical_section::with(|_| {
            // VENCRAM is a memory, not a peripheral: its clock lives in MEMENR.
            pac::RCC.memenr().modify(|w| w.set_vencramen(true));
            // Allocate VENCRAM to VENC rather than to the system.
            pac::SYSCFG.vencramcr().modify(|w| w.set_vencram_en(true));
        });

        platform::set_pool(pool);

        Self { _peri: peri }
    }

    /// The encoder ASIC ID (register 0).
    pub fn asic_id(&self) -> u32 {
        platform::asic_id()
    }

    /// Capability snapshot from the ASIC configuration registers.
    pub fn hw_config(&self) -> platform::HwConfig {
        platform::hw_config()
    }

    /// Create an H.264 encoder.
    pub fn h264<'a>(&'a self, cfg: h264::Config) -> Result<h264::Encoder<'a, 'd>, Error> {
        h264::Encoder::new(self, &cfg)
    }

    /// Create a JPEG encoder.
    pub fn jpeg<'a>(&'a self, cfg: jpeg::Config) -> Result<jpeg::Encoder<'a, 'd>, Error> {
        jpeg::Encoder::new(self, &cfg)
    }
}
