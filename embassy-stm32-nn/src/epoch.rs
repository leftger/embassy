//! Hybrid epoch definitions bridging NPU hardware epochs with embedded-nn software kernels.

use crate::coherency::invalidate_slice;
use crate::error::NnError;
use embedded_nn::softmax::softmax_s8;
use embedded_nn::support::dequantize_s8_to_f32;

/// Trait representing an executable software epoch running on the Cortex-M55 CPU.
pub trait SoftwareKernel {
    /// Executes the software kernel on CPU.
    fn run(&mut self) -> Result<(), NnError>;
}

impl<F> SoftwareKernel for F
where
    F: FnMut() -> Result<(), NnError>,
{
    #[inline]
    fn run(&mut self) -> Result<(), NnError> {
        (self)()
    }
}

/// A software epoch that applies `embedded-nn` fixed-point Softmax activation.
pub struct SoftmaxEpoch<'a> {
    input: &'a [i8],
    output: &'a mut [i8],
    num_rows: usize,
    row_size: usize,
    mult: i32,
    shift: i32,
    diff_min: i32,
}

impl<'a> SoftmaxEpoch<'a> {
    /// Creates a new Softmax software epoch for a 1D slice of logits.
    pub fn new_1d(input: &'a [i8], output: &'a mut [i8], mult: i32, shift: i32, diff_min: i32) -> Self {
        let row_size = input.len();
        Self {
            input,
            output,
            num_rows: 1,
            row_size,
            mult,
            shift,
            diff_min,
        }
    }

    /// Creates a new Softmax software epoch for multi-row (e.g. batch or sequence) tensors.
    pub fn new_2d(
        input: &'a [i8],
        output: &'a mut [i8],
        num_rows: usize,
        row_size: usize,
        mult: i32,
        shift: i32,
        diff_min: i32,
    ) -> Self {
        Self {
            input,
            output,
            num_rows,
            row_size,
            mult,
            shift,
            diff_min,
        }
    }
}

impl<'a> SoftwareKernel for SoftmaxEpoch<'a> {
    fn run(&mut self) -> Result<(), NnError> {
        if self.input.len() != self.output.len() {
            return Err(NnError::DimensionMismatch);
        }
        softmax_s8(
            self.input,
            self.num_rows,
            self.row_size,
            self.mult,
            self.shift,
            self.diff_min,
            self.output,
        )
        .map_err(NnError::from)
    }
}

/// A software epoch that dequantizes `int8` NPU output logits to `f32` probabilities.
pub struct DequantizeEpoch<'a> {
    input: &'a [i8],
    output: &'a mut [f32],
    scale: f32,
    zero_point: i32,
}

impl<'a> DequantizeEpoch<'a> {
    /// Creates a new dequantization software epoch.
    pub fn new(input: &'a [i8], output: &'a mut [f32], scale: f32, zero_point: i32) -> Self {
        Self {
            input,
            output,
            scale,
            zero_point,
        }
    }
}

impl<'a> SoftwareKernel for DequantizeEpoch<'a> {
    fn run(&mut self) -> Result<(), NnError> {
        if self.input.len() != self.output.len() {
            return Err(NnError::DimensionMismatch);
        }
        for (i, &val) in self.input.iter().enumerate() {
            self.output[i] = dequantize_s8_to_f32(val, self.scale, self.zero_point);
        }
        Ok(())
    }
}

/// A software epoch that calculates the predicted class index (argmax).
pub struct ArgMaxEpoch<'a> {
    logits: &'a [i8],
    best_class: &'a mut usize,
    best_score: &'a mut i8,
}

impl<'a> ArgMaxEpoch<'a> {
    /// Creates a new ArgMax software epoch.
    pub fn new(logits: &'a [i8], best_class: &'a mut usize, best_score: &'a mut i8) -> Self {
        Self {
            logits,
            best_class,
            best_score,
        }
    }
}

impl<'a> SoftwareKernel for ArgMaxEpoch<'a> {
    fn run(&mut self) -> Result<(), NnError> {
        if self.logits.is_empty() {
            return Err(NnError::DimensionMismatch);
        }
        let mut max_idx = 0;
        let mut max_val = self.logits[0];
        for (idx, &val) in self.logits.iter().enumerate().skip(1) {
            if val > max_val {
                max_val = val;
                max_idx = idx;
            }
        }
        *self.best_class = max_idx;
        *self.best_score = max_val;
        Ok(())
    }
}

/// An entry in a hybrid execution pipeline.
pub enum HybridEpoch<'a> {
    /// Hardware epoch blob executed on the ATON NPU.
    Hardware {
        /// Loaded command blob.
        blob: &'a [u64],
        /// Optional output slice to automatically invalidate in CPU D-cache upon completion.
        invalidate_output: Option<&'a [i8]>,
    },
    /// Software fallback epoch executed on the Cortex-M55 CPU.
    Software(&'a mut dyn SoftwareKernel),
}

impl<'a> HybridEpoch<'a> {
    /// Creates a hardware epoch executing `blob`.
    pub fn hw(blob: &'a [u64]) -> Self {
        Self::Hardware {
            blob,
            invalidate_output: None,
        }
    }

    /// Creates a hardware epoch with automatic output D-cache invalidation.
    pub fn hw_with_invalidation(blob: &'a [u64], output: &'a [i8]) -> Self {
        Self::Hardware {
            blob,
            invalidate_output: Some(output),
        }
    }

    /// Creates a software epoch running `kernel`.
    pub fn sw(kernel: &'a mut dyn SoftwareKernel) -> Self {
        Self::Software(kernel)
    }

    /// Executes any post-hardware cache maintenance or software step.
    pub fn handle_post_hw(&self) {
        if let Self::Hardware {
            invalidate_output: Some(out),
            ..
        } = self
        {
            invalidate_slice(out);
        }
    }
}
