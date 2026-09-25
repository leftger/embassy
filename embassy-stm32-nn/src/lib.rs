//! # embassy-stm32-nn
//!
//! Embassy companion driver integrating [`embedded-nn`] with the STM32N6 Neural-ART (ATON) NPU.
//!
//! This crate bridges the low-level hardware NPU driver in [`embassy_stm32::npu`] with the
//! [`embedded-nn`] TinyML inference runtime, providing:
//!
//! - **Automatic Cache Coherency**: Safe CPU data cache (D-cache) writebacks and invalidations
//!   around NPU hardware execution boundaries.
//! - **Zero-Copy Tensor Binding**: Relocation of [`embedded_nn::types::TensorView`] and contiguous
//!   slices directly into ST Edge AI Epoch Controller binaries.
//! - **Hybrid Execution Pipeline**: Unified execution of hardware epoch blobs on the ATON NPU
//!   and software fallback layers (Softmax, custom activations, dequantization, recurrent cells)
//!   on the Cortex-M55 CPU.
//! - **Async Model Runner**: Ergonomic [`NpuRunner`] abstraction for seamless async inference.

#![no_std]

pub mod binding;
pub mod coherency;
pub mod epoch;
pub mod error;
pub mod executor;

pub use binding::{BlobSession, bind_input_slice, bind_input_tensor, bind_output_tensor};
pub use coherency::{
    CoherencyState, CoherentBuffer, DCACHE_LINE_SIZE, NPU_ALIGNMENT, check_npu_alignment, clean_invalidate_slice,
    clean_slice, invalidate_slice, is_dcache_aligned,
};
pub use epoch::{ArgMaxEpoch, DequantizeEpoch, HybridEpoch, SoftmaxEpoch, SoftwareKernel};
pub use error::NnError;
pub use executor::{NpuRunner, run_hybrid_pipeline};
