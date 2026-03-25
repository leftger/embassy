//! PKA P-256 ECDH Example (High-Level API)
//!
//! Demonstrates P-256 ECDH key agreement using the high-level `pka::p256`
//! module, which wraps the PKA hardware accelerator.
//!
//! Compare with `pka_ecdh.rs` which uses the low-level API directly.
//! This example shows the same key agreement flow in far fewer lines.

#![no_std]
#![no_main]

use defmt::*;
use embassy_stm32::pka::p256::PublicKey;
use embassy_stm32::pka::Pka;
use embassy_stm32::rcc::{
    AHB5Prescaler, AHBPrescaler, APBPrescaler, PllDiv, PllMul, PllPreDiv, PllSource, Sysclk, VoltageScale, mux,
};
use embassy_stm32::rng::Rng;
use embassy_stm32::{Config, bind_interrupts, peripherals};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    PKA => embassy_stm32::pka::InterruptHandler<peripherals::PKA>;
    RNG => embassy_stm32::rng::InterruptHandler<peripherals::RNG>;
});

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let mut config = Config::default();
    config.rcc.pll1 = Some(embassy_stm32::rcc::Pll {
        source: PllSource::HSI,
        prediv: PllPreDiv::DIV1,
        mul: PllMul::MUL30,
        divr: Some(PllDiv::DIV5),
        divq: None,
        divp: Some(PllDiv::DIV30),
        frac: Some(0),
    });
    config.rcc.ahb_pre = AHBPrescaler::DIV1;
    config.rcc.apb1_pre = APBPrescaler::DIV1;
    config.rcc.apb2_pre = APBPrescaler::DIV1;
    config.rcc.apb7_pre = APBPrescaler::DIV1;
    config.rcc.ahb5_pre = AHB5Prescaler::DIV4;
    config.rcc.voltage_scale = VoltageScale::RANGE1;
    config.rcc.sys = Sysclk::PLL1_R;
    config.rcc.mux.rngsel = mux::Rngsel::HSI;

    let p = embassy_stm32::init(config);
    info!("PKA P-256 ECDH Example (High-Level API)");

    let mut pka = Pka::new_blocking(p.PKA, Irqs);
    let mut rng = Rng::new(p.RNG, Irqs);

    // ========== Generate Alice's Key Pair ==========
    info!("=== Generating Alice's Key Pair ===");

    let mut alice_private = [0u8; 32];
    rng.async_fill_bytes(&mut alice_private).await.unwrap();
    alice_private[0] &= 0x7F; // Ensure < n
    alice_private[31] |= 0x01; // Ensure non-zero

    let alice_public = pka.p256_public_key(&alice_private).unwrap();
    info!("Alice public key X: {:02x}", alice_public.x());
    info!("Alice public key Y: {:02x}", alice_public.y());

    // ========== Generate Bob's Key Pair ==========
    info!("=== Generating Bob's Key Pair ===");

    let mut bob_private = [0u8; 32];
    rng.async_fill_bytes(&mut bob_private).await.unwrap();
    bob_private[0] &= 0x7F;
    bob_private[31] |= 0x01;

    let bob_public = pka.p256_public_key(&bob_private).unwrap();
    info!("Bob public key X: {:02x}", bob_public.x());
    info!("Bob public key Y: {:02x}", bob_public.y());

    // ========== Validate Public Keys ==========
    info!("=== Validating Public Keys ===");

    defmt::assert!(pka.p256_validate_key(&bob_public).unwrap(), "Bob's key invalid!");
    info!("Bob's public key is valid (on curve)");

    defmt::assert!(pka.p256_validate_key(&alice_public).unwrap(), "Alice's key invalid!");
    info!("Alice's public key is valid (on curve)");

    // ========== SEC1 Round-Trip ==========
    // Demonstrate encoding → decoding (simulates key exchange over a wire)
    info!("=== SEC1 Encoding Round-Trip ===");

    let bob_sec1 = bob_public.to_sec1_uncompressed();
    info!("Bob SEC1 ({} bytes): {:02x}", bob_sec1.len(), bob_sec1);
    let bob_public_parsed = PublicKey::from_sec1_uncompressed(&bob_sec1).unwrap();

    // ========== Compute Shared Secrets ==========
    info!("=== Computing Shared Secrets ===");

    let alice_shared = pka.p256_shared_secret(&alice_private, &bob_public_parsed).unwrap();
    info!("Alice shared secret: {:02x}", alice_shared.as_bytes());

    let bob_shared = pka.p256_shared_secret(&bob_private, &alice_public).unwrap();
    info!("Bob shared secret:   {:02x}", bob_shared.as_bytes());

    // ========== Verify ==========
    if alice_shared.as_bytes() == bob_shared.as_bytes() {
        info!("SUCCESS: Both parties derived the SAME shared secret!");
    } else {
        error!("FAILURE: Shared secrets do not match!");
    }

    // ========== ST HAL Test Vector ==========
    info!("=== ST HAL ECDH Test Vector ===");

    // Private key from ST HAL PKA_ECCscalarMultiplication example
    let test_private: [u8; 32] = [
        0xfe, 0x22, 0xd2, 0xa5, 0xe9, 0xe8, 0x1f, 0x92, 0xb0, 0xbb, 0x42, 0xc2, 0xfe, 0xde, 0x3e, 0x63, 0xab, 0x0a,
        0xb1, 0x0f, 0x14, 0x9a, 0xa8, 0x3f, 0x76, 0xda, 0x44, 0x69, 0xd3, 0xbe, 0x69, 0x57,
    ];

    // Expected public key (known good from ST HAL)
    let expected_pub_x: [u8; 32] = [
        0xdd, 0x79, 0x95, 0xda, 0x1f, 0xa1, 0xc0, 0x25, 0xf3, 0xe7, 0xaa, 0x6b, 0x62, 0x2c, 0x9d, 0x78, 0x4a, 0x37,
        0x22, 0xdc, 0x8d, 0x64, 0x6b, 0x1b, 0x14, 0xf5, 0xc3, 0xa0, 0x3c, 0xa9, 0x70, 0x19,
    ];
    let expected_pub_y: [u8; 32] = [
        0xc6, 0xd8, 0x7e, 0xb5, 0x78, 0x43, 0xff, 0x15, 0xa0, 0x77, 0x92, 0x55, 0x86, 0x8e, 0x5b, 0xb4, 0x0e, 0xb0,
        0x79, 0xc8, 0xe3, 0x42, 0xca, 0xc4, 0x55, 0xf7, 0x2c, 0xf4, 0x04, 0xb1, 0x99, 0x82,
    ];

    let test_public = pka.p256_public_key(&test_private).unwrap();
    info!("Computed public key X: {:02x}", test_public.x());
    info!("Computed public key Y: {:02x}", test_public.y());

    if test_public.x() == &expected_pub_x && test_public.y() == &expected_pub_y {
        info!("SUCCESS: Public key matches ST HAL expected value!");
    } else {
        error!("Public key does not match expected value");
        error!("Expected X: {:02x}", expected_pub_x);
        error!("Expected Y: {:02x}", expected_pub_y);
    }

    info!("=== P-256 ECDH example complete ===");

    loop {
        cortex_m::asm::wfi();
    }
}
