//! Raw Hantro encoder bindings re-export.
//!
//! These are the bindgen-generated declarations for the STM32N6 Video Encoder
//! software stack (`h264encapi.h`, `jpegencapi.h`) and the Encoder Wrapper
//! Layer platform types (`ewl.h`). They mirror the C API 1:1 and are `unsafe`;
//! prefer the safe wrappers in [`crate::h264`] and [`crate::jpeg`].

pub use stm32_bindings::venc;
