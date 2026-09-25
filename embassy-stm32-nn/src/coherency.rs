//! Cache coherency management for hybrid Cortex-M55 CPU and Neural-ART NPU execution.
//!
//! On the STM32N6, memory buffers shared between the CPU and NPU (such as inputs,
//! activations, and outputs) require explicit synchronization:
//!
//! 1. **CPU -> NPU**: After the CPU fills an input tensor or software epoch intermediate,
//!    its Cortex-M55 data cache (D-cache) must be *cleaned* (written back to SRAM) so the
//!    NPU's AXI masters read valid data.
//! 2. **NPU -> CPU**: After the NPU finishes executing an epoch blob, the D-cache for the
//!    output tensor must be *invalidated* before the CPU reads it, preventing stale cache lines
//!    from masking the newly computed hardware results.

use crate::error::NnError;
use embassy_stm32::npu::cache;

/// Minimum address alignment required by the ATON NPU hardware (8 bytes).
pub const NPU_ALIGNMENT: usize = 8;

/// Cortex-M55 data cache line length in bytes (32 bytes).
pub const DCACHE_LINE_SIZE: usize = 32;

/// Checks that a memory slice satisfies the NPU hardware alignment (8 bytes).
#[inline]
pub fn check_npu_alignment<T>(slice: &[T]) -> Result<(), NnError> {
    let addr = slice.as_ptr() as usize;
    if addr % NPU_ALIGNMENT != 0 {
        return Err(NnError::MisalignedBuffer);
    }
    Ok(())
}

/// Checks that a memory slice satisfies D-cache line alignment (32 bytes).
///
/// While not strictly mandatory for NPU functionality, aligning input/output buffers
/// to 32 bytes prevents false cache-line sharing with adjacent variables during cache invalidation.
#[inline]
pub fn is_dcache_aligned<T>(slice: &[T]) -> bool {
    let addr = slice.as_ptr() as usize;
    let len = slice.len() * core::mem::size_of::<T>();
    addr % DCACHE_LINE_SIZE == 0 && len % DCACHE_LINE_SIZE == 0
}

/// Clean (write back) CPU data cache lines covering `slice` so the NPU reads fresh data.
#[inline]
pub fn clean_slice<T>(slice: &[T]) {
    let start = slice.as_ptr() as u32;
    let len = (slice.len() * core::mem::size_of::<T>()) as u32;
    cache::mcu_clean_range(start, len);
}

/// Invalidate CPU data cache lines covering `slice` so subsequent CPU reads observe NPU results.
#[inline]
pub fn invalidate_slice<T>(slice: &[T]) {
    let start = slice.as_ptr() as u32;
    let len = (slice.len() * core::mem::size_of::<T>()) as u32;
    cache::mcu_invalidate_range(start, len);
}

/// Clean and invalidate CPU data cache lines covering `slice`.
#[inline]
pub fn clean_invalidate_slice<T>(slice: &[T]) {
    let start = slice.as_ptr() as u32;
    let len = (slice.len() * core::mem::size_of::<T>()) as u32;
    cache::mcu_clean_invalidate_range(start, len);
}

/// State tracking for a coherent tensor buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CoherencyState {
    /// Buffer written by CPU; requires clean before NPU access.
    CpuDirty,
    /// Buffer synchronized to SRAM; safe for NPU read.
    Synchronized,
    /// Buffer written by NPU; requires invalidation before CPU read.
    NpuDirty,
}

/// A wrapper around a memory slice providing type-safe cache coherency tracking.
pub struct CoherentBuffer<'a, T> {
    data: &'a mut [T],
    state: CoherencyState,
}

impl<'a, T> CoherentBuffer<'a, T> {
    /// Wraps a mutable slice in a coherent buffer in the `CpuDirty` state.
    pub fn new(data: &'a mut [T]) -> Result<Self, NnError> {
        check_npu_alignment(data)?;
        Ok(Self {
            data,
            state: CoherencyState::CpuDirty,
        })
    }

    /// Prepares the buffer for NPU reading by cleaning CPU D-cache if dirty.
    pub fn prepare_for_npu_read(&mut self) {
        if self.state == CoherencyState::CpuDirty {
            clean_slice(self.data);
            self.state = CoherencyState::Synchronized;
        }
    }

    /// Marks the buffer as being written by the NPU.
    pub fn mark_npu_written(&mut self) {
        self.state = CoherencyState::NpuDirty;
    }

    /// Prepares the buffer for CPU reading by invalidating CPU D-cache if modified by NPU.
    pub fn as_slice(&mut self) -> &[T] {
        if self.state == CoherencyState::NpuDirty {
            invalidate_slice(self.data);
            self.state = CoherencyState::Synchronized;
        }
        self.data
    }

    /// Prepares the buffer for CPU mutation, returning a mutable slice and setting state to `CpuDirty`.
    pub fn as_mut_slice(&mut self) -> &mut [T] {
        if self.state == CoherencyState::NpuDirty {
            invalidate_slice(self.data);
        }
        self.state = CoherencyState::CpuDirty;
        self.data
    }

    /// Returns the raw pointer to the underlying buffer.
    #[inline]
    pub fn as_ptr(&self) -> *const T {
        self.data.as_ptr()
    }

    /// Returns the raw mutable pointer to the underlying buffer.
    #[inline]
    pub fn as_mut_ptr(&mut self) -> *mut T {
        self.data.as_mut_ptr()
    }

    /// Returns the number of elements in the buffer.
    #[inline]
    pub fn len(&self) -> usize {
        self.data.len()
    }

    /// Returns true if the buffer is empty.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }
}
