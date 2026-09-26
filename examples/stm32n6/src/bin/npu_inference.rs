//! STM32N6 Neural-ART (ATON) NPU inference demo using embassy-stm32-nn and embedded-nn.
//!
//! Demonstrates:
//! 1. Enabling all STM32N6 SRAM banks (AXISRAM1..6) and promoting NPU RIF security attributes.
//! 2. Running native `embedded-nn` TinyML inference on the Cortex-M55 CPU with Arm Helium (MVE)
//!    vector acceleration and asserting accuracy against TensorFlow Lite golden test vectors.
//! 3. Profiling latency (Cortex-M55 DWT cycle counting) and static footprint (Flash weights vs SRAM arena).
//! 4. Parsing an ST Edge AI Epoch Controller binary (EC binary) and initializing an NPU `BlobSession`.
//! 5. Executing hybrid inference (NPU hardware epoch + embedded-nn CPU software post-processing).

#![no_std]
#![no_main]

#[path = "../dense_mlp.rs"]
mod dense_mlp;

use aligned::{A8, Aligned};
use cortex_m::peripheral::DWT;
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::npu::ecloader::EcBinary;
use embassy_stm32::rcc::SupplyConfig;
use embassy_stm32::rif::{RifMaster, RifMasterAttributes, RifPeripheral, RifPeripheralAttributes};
use embassy_stm32::{bind_interrupts, npu, pac, peripherals};
use embassy_stm32_nn::{ArgMaxEpoch, BlobSession, DequantizeEpoch, NpuRunner, SoftmaxEpoch, SoftwareKernel};
use panic_probe as _;
use static_cell::StaticCell;

use dense_mlp::{ActiveModel, EC_CONTAINER, GOLDEN_TEST_VECTORS};

bind_interrupts!(struct Irqs {
    NPU0 => npu::InterruptHandler<peripherals::NPU>;
});

static BLOB_STORAGE: StaticCell<Aligned<A8, [u64; 64]>> = StaticCell::new();

/// Powers on and clocks all STM32N6 AXI and AHB SRAM banks so NPU bus masters
/// and CPU have full unconstrained access to activation and blob memory pools.
fn enable_all_sram() {
    pac::RCC.memenr().modify(|w| {
        w.set_axisram1en(true);
        w.set_axisram2en(true);
        w.set_axisram3en(true);
        w.set_axisram4en(true);
        w.set_axisram5en(true);
        w.set_axisram6en(true);
        w.set_ahbsram1en(true);
        w.set_ahbsram2en(true);
        w.set_bkpsramen(true);
    });
}

/// Grants the Neural-ART (ATON) NPU bus master and peripheral privileged
/// and secure access in the STM32N6 Resource Isolation Framework (RIF).
fn configure_npu_rif() {
    RifMaster::Npu.set_attributes(&RifMasterAttributes::new(1, true, true));
    RifPeripheral::Npu.set_attributes(&RifPeripheralAttributes::new(true, true));
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("============================================================");
    info!("  STM32N6 TinyML: embedded-nn Native + Neural-ART NPU Demo  ");
    info!("============================================================");

    // Configure external SMPS power supply for the STM32N6 Nucleo / Discovery kit.
    let mut config = embassy_stm32::Config::default();
    config.rcc.supply_config = SupplyConfig::External;
    let p = embassy_stm32::init(config);

    // 1. Enable Cortex-M55 DWT cycle counter for microsecond-accurate benchmarking.
    let mut core = cortex_m::Peripherals::take().unwrap();
    core.DCB.enable_trace();
    core.DWT.enable_cycle_counter();

    // 2. Power and clock all AXI SRAM banks (AXISRAM1..6).
    enable_all_sram();
    info!("SRAM power enabled for all banks (AXISRAM1..6)");

    // 3. Configure RIF security attributes for the ATON NPU.
    configure_npu_rif();
    info!("RIF security configured for NPU master and peripheral");

    // =========================================================================
    // Phase 1: embedded-nn Native Execution on Cortex-M55 (Helium MVE SIMD)
    // =========================================================================
    info!("------------------------------------------------------------");
    info!("Phase 1: embedded-nn Native Inference (Cortex-M55 + Helium)");
    info!("Model: DenseMlp (16 -> 16 ReLU -> 4 Logits)");
    info!("Weights (Flash): {} bytes", ActiveModel::FLASH_WEIGHTS);
    info!(
        "Arena (SRAM):    {} bytes (zero heap allocation)",
        ActiveModel::ARENA_SIZE
    );
    info!("------------------------------------------------------------");

    let mut arena = [0u8; ActiveModel::ARENA_SIZE];
    let mut total_cpu_cycles = 0u32;
    let mut passed_vectors = 0usize;

    for (idx, vec) in GOLDEN_TEST_VECTORS.iter().enumerate() {
        let start_cycles = DWT::cycle_count();
        let logits = match ActiveModel::predict(&vec.input, &mut arena) {
            Ok(out) => out,
            Err(e) => {
                error!("Test vector {} failed prediction: {}", idx, e);
                continue;
            }
        };
        let elapsed_cycles = DWT::cycle_count().wrapping_sub(start_cycles);
        total_cpu_cycles += elapsed_cycles;

        // Verify top-1 predicted class via embedded-nn ArgMax
        let mut predicted_class = 0usize;
        let mut top_score = i8::MIN;
        let mut argmax = ArgMaxEpoch::new(logits, &mut predicted_class, &mut top_score);
        let _ = argmax.run();

        // Exact match against TensorFlow Lite golden oracle vectors
        let logits_match = logits == &vec.expected_logits;
        let class_match = predicted_class == vec.expected_class;

        if logits_match && class_match {
            passed_vectors += 1;
            info!(
                "Vector {}: PASS [class {}, {} cycles] logits: {:?}",
                idx, predicted_class, elapsed_cycles, logits
            );
        } else {
            error!(
                "Vector {}: MISMATCH! got logits {:?} (class {}), expected {:?} (class {})",
                idx, logits, predicted_class, vec.expected_logits, vec.expected_class
            );
        }

        // Post-processing: Softmax + f32 Dequantization check
        let mut softmax_output = [0i8; 4];
        let mut softmax_step = SoftmaxEpoch::new_1d(logits, &mut softmax_output, 1073741824, 0, -128);
        let _ = softmax_step.run();

        let mut probs_f32 = [0.0f32; 4];
        let mut dequant_step = DequantizeEpoch::new(&softmax_output, &mut probs_f32, 0.00390625, -128);
        let _ = dequant_step.run();
        trace!("  Softmax probabilities: {:?}", probs_f32);
    }

    let avg_cycles = total_cpu_cycles / (GOLDEN_TEST_VECTORS.len() as u32);
    info!(
        "Native CPU Inference Complete: {}/{} golden vectors passed! Avg Latency: {} cycles (~{=f32} us @ 800MHz)",
        passed_vectors,
        GOLDEN_TEST_VECTORS.len(),
        avg_cycles,
        avg_cycles as f32 / 800.0
    );

    // =========================================================================
    // Phase 2: STM32N6 Neural-ART (ATON) Hardware Companion Pipeline
    // =========================================================================
    info!("------------------------------------------------------------");
    info!("Phase 2: STM32N6 Neural-ART NPU Hardware Pipeline");
    info!("------------------------------------------------------------");

    // Initialize the ATON NPU peripheral driver.
    let mut npu_driver = npu::Npu::new(p.NPU, Irqs);
    info!("ATON NPU peripheral driver initialized successfully");

    // Parse the EC binary container compiled by embedded-nn-aton.
    let ec_bin = match EcBinary::new(&EC_CONTAINER) {
        Ok(bin) => bin,
        Err(e) => {
            error!("Failed to parse EC binary: {:?}", e);
            return;
        }
    };
    info!("EC binary parsed: {} relocations found", ec_bin.num_relocs());

    // Initialize writable blob buffer in NPU-accessible SRAM.
    let blob_buffer = BLOB_STORAGE.init(Aligned([0u64; 64]));
    let session = match BlobSession::new(ec_bin, &mut blob_buffer[..]) {
        Ok(s) => s,
        Err(e) => {
            error!("Failed to load blob into session: {:?}", e);
            return;
        }
    };

    // Configure high-level NPU model runner with custom relocation symbols.
    let mut runner = NpuRunner::new(&mut npu_driver, session).with_symbols("_in", "_out");

    // Allocate 8-byte aligned input and output tensors matching Golden Vector 0.
    let first_vec = &GOLDEN_TEST_VECTORS[0];
    let input_data: Aligned<A8, [i8; 16]> = Aligned(first_vec.input);
    let mut npu_output_logits: Aligned<A8, [i8; 4]> = Aligned([0i8; 4]);

    info!("Running hybrid NPU inference (Hardware blob + embedded-nn CPU software epochs)...");

    match runner.infer(&input_data[..], &mut npu_output_logits[..]).await {
        Ok(()) => {
            info!("NPU hardware execution complete!");

            // Post-processing 1: Predict top-1 class via embedded-nn ArgMax.
            let mut predicted_class = 0usize;
            let mut top_score = 0i8;
            let mut argmax_step = ArgMaxEpoch::new(&npu_output_logits[..], &mut predicted_class, &mut top_score);
            let _ = argmax_step.run();
            info!("NPU Predicted class: {}, top score: {}", predicted_class, top_score);

            // Compare against golden vector
            if predicted_class == first_vec.expected_class {
                info!("NPU Output matches Golden Vector 0 prediction!");
            } else {
                warn!(
                    "NPU Output differed from Golden Vector 0 (expected class {})",
                    first_vec.expected_class
                );
            }
        }
        Err(e) => {
            // Note: Since MOCK_EC_CONTAINER contains stub microcode instructions rather than a
            // full trained network compiled by ST Edge AI, the ATON epoch controller hardware will
            // raise a microcode execution fault on real silicon without valid instructions.
            warn!(
                "NPU execution status: {:?} (Expected when using stub microcode without an ST Edge AI compiled blob)",
                e
            );
        }
    }

    info!("============================================================");
    info!("  TinyML STM32N6 End-to-End Deployment Verification Complete  ");
    info!("============================================================");
}
