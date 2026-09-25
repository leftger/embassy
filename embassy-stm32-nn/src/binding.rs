//! Dynamic tensor and buffer relocation bindings for ST Edge AI Epoch Controller binaries.

use crate::coherency::{check_npu_alignment, clean_slice};
use crate::error::NnError;
use embassy_stm32::npu::ecloader::{EcBinary, EcError};
use embedded_nn::types::TensorView;

/// Relocates an input tensor into an EC binary blob, validating alignment and cleaning D-cache.
///
/// # Arguments
/// * `binary` - Parsed EC binary container.
/// * `blob` - Writable blob instruction memory in NPU-visible SRAM.
/// * `symbol` - Symbol identifier emitted by Edge AI tooling (e.g. `"_user_io_input_0"`).
/// * `tensor` - Input [`TensorView`] containing quantized sensor/image data.
/// * `prev_base` - Tracks the previous base address relocated into this symbol (initialized to 0).
pub fn bind_input_tensor<T>(
    binary: &EcBinary<'_>,
    blob: &mut [u64],
    symbol: &str,
    tensor: &TensorView<'_, T>,
    prev_base: &mut u32,
) -> Result<(), NnError> {
    bind_input_slice(binary, blob, symbol, tensor.data, prev_base)
}

/// Relocates an input slice into an EC binary blob, validating alignment and cleaning D-cache.
pub fn bind_input_slice<T>(
    binary: &EcBinary<'_>,
    blob: &mut [u64],
    symbol: &str,
    slice: &[T],
    prev_base: &mut u32,
) -> Result<(), NnError> {
    check_npu_alignment(slice)?;
    clean_slice(slice);
    let base = slice.as_ptr() as u32;
    binary.reloc_by_id(blob, symbol, base, prev_base).map_err(|e| match e {
        EcError::NotFound => NnError::SymbolNotFound,
        other => NnError::Ec(other),
    })
}

/// Relocates an output tensor into an EC binary blob, validating alignment.
///
/// Note: Output D-cache invalidation should be performed *after* NPU execution completes.
pub fn bind_output_tensor<T>(
    binary: &EcBinary<'_>,
    blob: &mut [u64],
    symbol: &str,
    slice: &mut [T],
    prev_base: &mut u32,
) -> Result<(), NnError> {
    check_npu_alignment(slice)?;
    let base = slice.as_mut_ptr() as u32;
    binary.reloc_by_id(blob, symbol, base, prev_base).map_err(|e| match e {
        EcError::NotFound => NnError::SymbolNotFound,
        other => NnError::Ec(other),
    })
}

/// A stateful session managing a loaded blob in NPU memory with tracked relocations.
pub struct BlobSession<'a> {
    binary: EcBinary<'a>,
    blob: &'a mut [u64],
    input_base: u32,
    output_base: u32,
}

impl<'a> BlobSession<'a> {
    /// Loads the blob instructions from `binary` into `blob_buffer`.
    pub fn new(binary: EcBinary<'a>, blob_buffer: &'a mut [u64]) -> Result<Self, NnError> {
        check_npu_alignment(blob_buffer)?;
        binary.load_blob(blob_buffer)?;
        Ok(Self {
            binary,
            blob: blob_buffer,
            input_base: 0,
            output_base: 0,
        })
    }

    /// Returns a reference to the loaded blob slice.
    #[inline]
    pub fn blob(&self) -> &[u64] {
        self.blob
    }

    /// Returns a mutable reference to the loaded blob slice.
    #[inline]
    pub fn blob_mut(&mut self) -> &mut [u64] {
        self.blob
    }

    /// Relocate input buffer by symbol ID.
    pub fn set_input<T>(&mut self, symbol: &str, slice: &[T]) -> Result<(), NnError> {
        bind_input_slice(&self.binary, self.blob, symbol, slice, &mut self.input_base)
    }

    /// Relocate output buffer by symbol ID.
    pub fn set_output<T>(&mut self, symbol: &str, slice: &mut [T]) -> Result<(), NnError> {
        bind_output_tensor(&self.binary, self.blob, symbol, slice, &mut self.output_base)
    }

    /// Relocate input buffer by index.
    pub fn set_input_by_index<T>(&mut self, index: usize, slice: &[T]) -> Result<(), NnError> {
        check_npu_alignment(slice)?;
        clean_slice(slice);
        let base = slice.as_ptr() as u32;
        self.binary
            .reloc(self.blob, index, base, &mut self.input_base)
            .map_err(NnError::Ec)
    }

    /// Relocate output buffer by index.
    pub fn set_output_by_index<T>(&mut self, index: usize, slice: &mut [T]) -> Result<(), NnError> {
        check_npu_alignment(slice)?;
        let base = slice.as_mut_ptr() as u32;
        self.binary
            .reloc(self.blob, index, base, &mut self.output_base)
            .map_err(NnError::Ec)
    }
}
