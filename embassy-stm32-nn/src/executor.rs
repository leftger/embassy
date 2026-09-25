//! Asynchronous hybrid pipeline executor and model runners.

use crate::binding::BlobSession;
use crate::coherency::invalidate_slice;
use crate::epoch::{HybridEpoch, SoftwareKernel};
use crate::error::NnError;
use embassy_stm32::npu::{Instance, Npu};

/// Executes a schedule of hybrid epochs (NPU hardware and CPU software kernels) in sequence.
pub async fn run_hybrid_pipeline<'d, T: Instance>(
    npu: &mut Npu<'d, T>,
    epochs: &mut [HybridEpoch<'_>],
) -> Result<(), NnError> {
    for (idx, epoch) in epochs.iter_mut().enumerate() {
        match epoch {
            HybridEpoch::Hardware {
                blob,
                invalidate_output,
            } => {
                npu.run_epoch_blob(blob).await.map_err(NnError::Npu)?;
                if let Some(out) = invalidate_output {
                    invalidate_slice(out);
                }
            }
            HybridEpoch::Software(kernel) => {
                kernel.run().map_err(|_| NnError::SoftwareEpochFailed(idx))?;
            }
        }
    }
    Ok(())
}

/// A high-level asynchronous model runner encapsulating an NPU instance and a loaded blob session.
pub struct NpuRunner<'a, 'd, T: Instance> {
    npu: &'a mut Npu<'d, T>,
    session: BlobSession<'a>,
    input_sym: &'static str,
    output_sym: &'static str,
}

impl<'a, 'd, T: Instance> NpuRunner<'a, 'd, T> {
    /// Creates a new `NpuRunner` with default symbol names `"_user_io_input_0"` and `"_user_io_output_0"`.
    pub fn new(npu: &'a mut Npu<'d, T>, session: BlobSession<'a>) -> Self {
        Self {
            npu,
            session,
            input_sym: "_user_io_input_0",
            output_sym: "_user_io_output_0",
        }
    }

    /// Sets custom relocation symbol names for input and output.
    pub fn with_symbols(mut self, input_sym: &'static str, output_sym: &'static str) -> Self {
        self.input_sym = input_sym;
        self.output_sym = output_sym;
        self
    }

    /// Returns a reference to the inner `BlobSession`.
    #[inline]
    pub fn session(&self) -> &BlobSession<'a> {
        &self.session
    }

    /// Returns a mutable reference to the inner `BlobSession`.
    #[inline]
    pub fn session_mut(&mut self) -> &mut BlobSession<'a> {
        &mut self.session
    }

    /// Runs asynchronous inference on `input`, placing hardware results into `output`.
    ///
    /// Automatically manages:
    /// - 8-byte NPU buffer alignment validation
    /// - Symbol relocation into the blob
    /// - Cortex-M55 D-cache clean before NPU execution
    /// - Asynchronous wait for NPU end-of-epoch interrupt
    /// - Cortex-M55 D-cache invalidation after NPU execution
    pub async fn infer(&mut self, input: &[i8], output: &mut [i8]) -> Result<(), NnError> {
        self.session.set_input(self.input_sym, input)?;
        self.session.set_output(self.output_sym, output)?;

        self.npu
            .run_epoch_blob(self.session.blob())
            .await
            .map_err(NnError::Npu)?;

        invalidate_slice(output);
        Ok(())
    }

    /// Runs asynchronous inference followed by an embedded-nn post-processing kernel.
    pub async fn infer_with_postproc<P: SoftwareKernel>(
        &mut self,
        input: &[i8],
        output: &mut [i8],
        postproc: &mut P,
    ) -> Result<(), NnError> {
        self.infer(input, output).await?;
        postproc.run()
    }

    /// Runs asynchronous inference followed by an embedded-nn post-processing closure on `output`.
    pub async fn infer_with_fn<R, F: FnOnce(&[i8]) -> Result<R, NnError>>(
        &mut self,
        input: &[i8],
        output: &mut [i8],
        postproc: F,
    ) -> Result<R, NnError> {
        self.infer(input, output).await?;
        postproc(output)
    }
}
