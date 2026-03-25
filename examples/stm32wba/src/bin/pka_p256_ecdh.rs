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

    // ========== NIST CAVP Test Vector ==========
    info!("=== NIST ECDH Test Vector ===");

    let test_private: [u8; 32] = [
        0xc9, 0xaf, 0xa9, 0xd8, 0x45, 0xba, 0x75, 0x16, 0x6b, 0x5c, 0x21, 0x57, 0x67, 0xb1, 0xd6, 0x93, 0x4e, 0x50,
        0xc3, 0xdb, 0x36, 0xe8, 0x9b, 0x12, 0x7b, 0x8a, 0x62, 0x2b, 0x12, 0x0f, 0x67, 0x21,
    ];

    // Build peer public key from raw coordinates via SEC1
    let mut peer_sec1 = [0u8; 65];
    peer_sec1[0] = 0x04;
    peer_sec1[1..33].copy_from_slice(&[
        0x70, 0x04, 0x0a, 0xcd, 0x89, 0x8e, 0xb2, 0x3d, 0xfa, 0x85, 0x9a, 0x16, 0x53, 0x31, 0x9c, 0xa8, 0xd1, 0xb0,
        0x81, 0xf6, 0x0f, 0x3e, 0x05, 0x97, 0xc7, 0xfd, 0xd6, 0x29, 0x32, 0x4b, 0xe6, 0x2c,
    ]);
    peer_sec1[33..65].copy_from_slice(&[
        0x5f, 0x67, 0x94, 0x7f, 0x9c, 0x63, 0x8f, 0x63, 0xd7, 0xba, 0x35, 0x73, 0xb8, 0xbd, 0xb5, 0x5a, 0x83, 0x62,
        0xb3, 0x9c, 0x23, 0x4e, 0x7d, 0x36, 0x7f, 0xc1, 0xd5, 0xcd, 0x8c, 0x82, 0xc9, 0x25,
    ]);

    let peer_public = PublicKey::from_sec1_uncompressed(&peer_sec1).unwrap();
    defmt::assert!(pka.p256_validate_key(&peer_public).unwrap(), "Test peer key invalid!");

    let test_shared = pka.p256_shared_secret(&test_private, &peer_public).unwrap();
    info!("Test shared secret: {:02x}", test_shared.as_bytes());

    info!("=== P-256 ECDH example complete ===");

    loop {
        cortex_m::asm::wfi();
    }
}
