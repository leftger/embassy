//! STM32N6 Neural-ART (ATON) NPU inference demo using embassy-stm32-nn and embedded-nn.
//!
//! Demonstrates:
//! 1. Enabling the NPU peripheral and routing the ATON interrupt line 0 (NPU0).
//! 2. Parsing an ST Edge AI Epoch Controller binary (EC binary).
//! 3. Initializing a `BlobSession` with dynamic input and output relocations.
//! 4. Running hybrid TinyML inference (NPU hardware epoch + embedded-nn CPU software post-processing).

#![no_std]
#![no_main]

use aligned::{A8, Aligned};
use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::npu::ecloader::{BINARY_MAGIC, BLOB_MAGIC, EcBinary};
use embassy_stm32::rcc::SupplyConfig;
use embassy_stm32::{bind_interrupts, npu, peripherals};
use embassy_stm32_nn::{ArgMaxEpoch, BlobSession, DequantizeEpoch, NpuRunner, SoftmaxEpoch, SoftwareKernel};
use panic_probe as _;
use static_cell::StaticCell;

bind_interrupts!(struct Irqs {
    NPU0 => npu::InterruptHandler<peripherals::NPU>;
});

// A minimal valid EC binary container:
// Header: [magic, reloc_off, patch_off, debug_off, blob_off] (byte offsets)
// Followed by relocations table, strings, and blob section.
#[repr(align(8))]
struct AlignedContainer<const N: usize>([u32; N]);

static MOCK_EC_CONTAINER: AlignedContainer<32> = AlignedContainer([
    // 0..5: Container header
    BINARY_MAGIC, // words[0]: Magic 0xECBF_0050
    20,           // words[1]: reloc_off = byte 20 (word 5)
    0,            // words[2]: patch_off = 0 (none)
    0,            // words[3]: debug_off = 0 (none)
    84,           // words[4]: blob_off = byte 84 (word 21)
    // 5..21: Relocations table (starts at byte 20, word 5)
    2,  // table[0]: 2 relocation entries
    28, // table[1]: reloc 0 id_off (byte 28 relative to table start = byte 48 / word 12)
    1,  // table[2]: reloc 0 num words = 1
    44, // table[3]: reloc 0 list_off (byte 44 relative to table start = byte 64 / word 16)
    36, // table[4]: reloc 1 id_off (byte 36 relative to table start = byte 56 / word 14)
    1,  // table[5]: reloc 1 num words = 1
    48, // table[6]: reloc 1 list_off (byte 48 relative to table start = byte 68 / word 17)
    0,  // word 12 / byte 48: string "_in\0"
    0x006E695F,
    0, // word 14 / byte 56: string "_out\0"
    0x74756F5F,
    0, // word 16 / byte 64: reloc 0 word offset in blob = 0
    0, // word 17 / byte 68: reloc 1 word offset in blob = 1
    1,
    0,
    0, // padding to byte 84 (word 21)
    // 21..32: Blob section (starts at byte 84, word 21)
    BLOB_MAGIC, // words[21]: Magic 0xCA05_7A7A
    4,          // words[22]: instr_words = 4 words
    // Instruction words:
    0x00000000,
    0x00000000,
    0x00000000,
    0x00000000, // padding
    0,
    0,
    0,
    0,
    0,
]);

static BLOB_STORAGE: StaticCell<Aligned<A8, [u64; 64]>> = StaticCell::new();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    info!("Starting STM32N6 NPU + embedded-nn TinyML Demo");

    // Configure external SMPS power supply for the STM32N6 Nucleo / Discovery kit.
    let mut config = embassy_stm32::Config::default();
    config.rcc.supply_config = SupplyConfig::External;
    let p = embassy_stm32::init(config);

    // Initialize the ATON NPU peripheral driver.
    let mut npu_driver = npu::Npu::new(p.NPU, Irqs);
    info!("ATON NPU peripheral initialized successfully");

    // Parse the EC binary container.
    let ec_bin = match EcBinary::from_words(&MOCK_EC_CONTAINER.0) {
        Ok(bin) => bin,
        Err(e) => {
            error!("Failed to parse EC binary: {:?}", e);
            return;
        }
    };
    info!("EC binary parsed: {} relocations found", ec_bin.num_relocs());

    // Initialize writable blob buffer in NPU-accessible RAM.
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

    // Allocate 8-byte aligned input and output tensors.
    let input_data: Aligned<A8, [i8; 16]> = Aligned([10, -5, 20, 15, -30, 4, 8, 12, -1, 3, 7, -12, 18, 22, -6, 0]);
    let mut output_logits: Aligned<A8, [i8; 4]> = Aligned([0i8; 4]);

    info!("Running hybrid inference pipeline (NPU hardware blob + embedded-nn CPU postprocessing)...");

    // Run NPU hardware inference.
    match runner.infer(&input_data[..], &mut output_logits[..]).await {
        Ok(()) => {
            info!("NPU hardware inference complete!");

            // Post-processing 1: Predict top-1 class via embedded-nn ArgMax.
            let mut predicted_class = 0usize;
            let mut top_score = 0i8;
            let mut argmax_step = ArgMaxEpoch::new(&output_logits[..], &mut predicted_class, &mut top_score);
            let _ = argmax_step.run();
            info!("Predicted class: {}, top score: {}", predicted_class, top_score);

            // Post-processing 2: Softmax on logits.
            let mut softmax_output = [0i8; 4];
            let mut softmax_step = SoftmaxEpoch::new_1d(&output_logits[..], &mut softmax_output, 1073741824, 0, -128);
            let _ = softmax_step.run();

            // Post-processing 3: Dequantize probabilities to f32.
            let mut probs_f32 = [0.0f32; 4];
            let mut dequant_step = DequantizeEpoch::new(&softmax_output, &mut probs_f32, 0.00390625, -128);
            let _ = dequant_step.run();
            info!("embedded-nn CPU dequantized probabilities: {:?}", probs_f32);
        }
        Err(e) => {
            // Note: Since MOCK_EC_CONTAINER contains stub instructions rather than a real trained
            // network generated by ST Edge AI, the NPU epoch controller hardware may raise a fault
            // when executed on real silicon without valid microcode.
            warn!("NPU execution status: {:?}", e);
        }
    }

    info!("TinyML NPU inference demo complete.");
}
