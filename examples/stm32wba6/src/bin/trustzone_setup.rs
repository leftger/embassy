#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::Config;
use embassy_stm32::gtzc::{Peripheral, SramBlock};
use embassy_stm32::sau::{Attribute, Region};
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    // Basic hardware initialization (Secure side)
    let config = Config::default();
    let _p = embassy_stm32::init(config);

    info!("TrustZone Setup Example (Secure Side)");

    // 1. Configure the SAU (Security Attribution Unit)
    // Define memory regions for the Non-Secure world.
    // These addresses are typical for STM32WBA65, but should be adjusted for your specific memory map.
    let regions = [
        // Non-Secure Flash (e.g., second half of Flash)
        Region {
            base_address: 0x0808_0000,
            end_address: 0x080F_FFFF,
            attribute: Attribute::NonSecure,
        },
        // Non-Secure SRAM2 (e.g., all of SRAM2)
        Region {
            base_address: 0x2001_0000,
            end_address: 0x2001_FFFF,
            attribute: Attribute::NonSecure,
        },
        // Non-Secure Callable (NSC) region in Secure Flash for entry points
        Region {
            base_address: 0x0C07_F000,
            end_address: 0x0C07_FFFF,
            attribute: Attribute::NonSecureCallable,
        },
    ];

    unsafe {
        embassy_stm32::sau::init(&regions);
        info!("SAU configured with {} regions", regions.len());
    }

    // 2. Configure the GTZC (Global TrustZone Controller)
    // Assign peripherals and memory blocks to the Non-Secure domain.
    unsafe {
        embassy_stm32::gtzc::init();

        // Assign USART1 to Non-Secure so the NS application can use it for logging
        embassy_stm32::gtzc::set_peripheral_secure(Peripheral::Usart1, false);
        info!("USART1 assigned to Non-Secure");

        // Assign SRAM2 to Non-Secure
        embassy_stm32::gtzc::set_sram_block_secure(SramBlock::Sram2, false);
        info!("SRAM2 assigned to Non-Secure");

        // Enable TZIC to catch illegal access attempts
        embassy_stm32::gtzc::enable_tzic();
        info!("GTZC/TZIC enabled");
    }

    info!("TrustZone configuration complete.");
    info!("In a real scenario, we would now jump to the Non-Secure application.");

    loop {
        embassy_time::Timer::after_secs(1).await;
    }
}
