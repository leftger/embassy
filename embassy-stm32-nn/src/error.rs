//! Error definitions for embassy-stm32-nn.

use embassy_stm32::npu;
use embassy_stm32::npu::ecloader;

/// defmt-compatible representation of `embedded_nn::Error`.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum EmbeddedNnError {
    /// Invalid or incompatible argument/dimensions.
    ArgumentError,
    /// Operation is not implemented for the given configuration.
    NoImplementation,
    /// Execution or calculation error.
    Failure,
}

impl From<embedded_nn::Error> for EmbeddedNnError {
    fn from(e: embedded_nn::Error) -> Self {
        match e {
            embedded_nn::Error::ArgumentError => Self::ArgumentError,
            embedded_nn::Error::NoImplementation => Self::NoImplementation,
            embedded_nn::Error::Failure => Self::Failure,
        }
    }
}

/// Comprehensive error type for hybrid NPU and embedded-nn execution.
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum NnError {
    /// Error originating from the low-level ATON NPU peripheral driver.
    Npu(npu::Error),
    /// Error during parsing or relocation of an Epoch Controller binary container.
    Ec(ecloader::EcError),
    /// An error occurred during embedded-nn software kernel execution.
    EmbeddedNn(EmbeddedNnError),
    /// Dimension or shape mismatch between tensors and expected layer parameters.
    DimensionMismatch,
    /// Tensor buffer does not meet NPU memory alignment requirements (minimum 8-byte aligned).
    MisalignedBuffer,
    /// Provided memory arena or buffer is too small for blob or tensor storage.
    BufferTooSmall,
    /// The requested relocation or patch identifier was not found in the container.
    SymbolNotFound,
    /// Execution of a software epoch or callback failed at the specified epoch index.
    SoftwareEpochFailed(usize),
    /// Model or executor has not been initialized.
    NotInitialized,
}

impl From<npu::Error> for NnError {
    #[inline]
    fn from(e: npu::Error) -> Self {
        Self::Npu(e)
    }
}

impl From<ecloader::EcError> for NnError {
    #[inline]
    fn from(e: ecloader::EcError) -> Self {
        Self::Ec(e)
    }
}

impl From<embedded_nn::Error> for NnError {
    #[inline]
    fn from(e: embedded_nn::Error) -> Self {
        Self::EmbeddedNn(e.into())
    }
}

impl From<EmbeddedNnError> for NnError {
    #[inline]
    fn from(e: EmbeddedNnError) -> Self {
        Self::EmbeddedNn(e)
    }
}
