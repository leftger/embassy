#![no_std]
#![no_main]

//! NeoChrom (NemaGFX) smoke test for the STM32N6570-DK.
//!
//! Initializes the GPU via [`embassy_stm32_neochrom::NeoChrom`] and repeatedly
//! clears a small RGBA8888 framebuffer with the GPU command-list path.
//!
//! With the default `stub-gpu2d` feature the HAL is a link-time stub — the
//! framebuffer may stay zero on hardware until a real STM32Cube GPU2D init is
//! wired in. The example still validates NemaGFX init, linking, and the clear
//! API from an Embassy application.
//!
//! Generate `stm32-bindings` first:
//! ```text
//! cd ../../../stm32-bindings
//! cargo run --release --bin stm32-bindings-gen -- --module nema_gfx
//! ```

use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::rcc::SupplyConfig;
use embassy_stm32::{Config, pac};
use embassy_stm32_neochrom::{FrameBuffer, NeoChrom, Rgba8888};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};

const FB_WIDTH: u32 = 64;
const FB_HEIGHT: u32 = 64;
const FB_PIXELS: usize = (FB_WIDTH * FB_HEIGHT) as usize;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = Config::default();
    // DK uses external SMPS (UM3300 Tab.6); embassy default = internal SMPS hangs init() at VOSRDY.
    config.rcc.supply_config = SupplyConfig::External;
    let _p = embassy_stm32::init(config);

    enable_all_sram();

    info!("stm32n6 neochrom example starting");

    let mut gpu = NeoChrom::new().expect("NeoChrom init failed");
    let fb = FrameBuffer::<FB_WIDTH, FB_HEIGHT, FB_PIXELS>::new();

    let colors = [
        Rgba8888::RED,
        Rgba8888::GREEN,
        Rgba8888::BLUE,
        Rgba8888::WHITE,
        Rgba8888::BLACK,
    ];
    let mut idx = 0usize;

    loop {
        let color = colors[idx];
        gpu.clear(&fb, color).expect("NeoChrom clear failed");
        info!("gpu clear ok, color=0x{:08x}", color.bits());
        idx = (idx + 1) % colors.len();
        Timer::after_secs(1).await;
    }
}

/// Enable run-mode clocks for every AXISRAM bank (same as the LTDC example).
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
