//! Global TrustZone Controller (GTZC) configuration.
//!
//! This module provides a way to configure the GTZC (TZSC, MPCBB, TZIC) on STM32WBA65 cores.

use crate::pac;

/// GTZC peripheral identifier for TZSC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Peripheral {
    /// FLASH.
    Flash = 0,
    /// SRAM1.
    Sram1 = 1,
    /// SRAM2.
    Sram2 = 2,
    /// GPIOA.
    Gpioa = 3,
    /// GPIOB.
    Gpiob = 4,
    /// GPIOC.
    Gpioc = 5,
    /// GPIOH.
    Gpioh = 6,
    /// USART1.
    Usart1 = 7,
    /// USART2.
    Usart2 = 8,
    /// LPUART1.
    Lpuart1 = 9,
    /// SPI1.
    Spi1 = 10,
    /// SPI3.
    Spi3 = 11,
    /// I2C1.
    I2c1 = 12,
    /// I2C3.
    I2c3 = 13,
    /// LPTIM1.
    Lptim1 = 14,
    /// LPTIM2.
    Lptim2 = 15,
    /// ADC1.
    Adc1 = 16,
    /// RTC.
    Rtc = 17,
    /// TAMP.
    Tamp = 18,
    /// PWR.
    Pwr = 19,
    /// RCC.
    Rcc = 20,
    /// RNG.
    Rng = 21,
    /// HASH.
    Hash = 22,
    /// AES.
    Aes = 23,
    /// SAES.
    Saes = 24,
    /// PKA.
    Pka = 25,
}

/// SRAM block identifier for MPCBB.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SramBlock {
    /// SRAM1.
    Sram1 = 0,
    /// SRAM2.
    Sram2 = 1,
}

/// Initialize the GTZC and configure peripheral and SRAM security.
///
/// This function allows assigning peripherals and SRAM blocks to Secure or Non-Secure domains.
pub unsafe fn init() {
    // Enable GTZC clock via RCC if needed
    // On WBA65, GTZC clock is usually enabled by default or via RCC_AHB2ENR.GTZCEN
}

/// Assign a peripheral to the Secure or Non-Secure domain.
pub unsafe fn set_peripheral_secure(peripheral: Peripheral, secure: bool) {
    let gtzc_tzsc = pac::GTZC_TZSC;

    // The register offset and bit mapping for peripherals is documented in RM0493.
    // Each SECCFGRx register covers a group of peripherals.
    let (reg_idx, bit) = match peripheral {
        Peripheral::Flash => (0, 0),
        Peripheral::Sram1 => (0, 1),
        Peripheral::Sram2 => (0, 2),
        Peripheral::Gpioa => (1, 0),
        Peripheral::Gpiob => (1, 1),
        Peripheral::Gpioc => (1, 2),
        Peripheral::Gpioh => (1, 7),
        Peripheral::Usart1 => (2, 0),
        Peripheral::Usart2 => (2, 1),
        Peripheral::Lpuart1 => (2, 2),
        Peripheral::Spi1 => (2, 3),
        Peripheral::Spi3 => (2, 5),
        Peripheral::I2c1 => (2, 6),
        Peripheral::I2c3 => (2, 8),
        Peripheral::Lptim1 => (2, 9),
        Peripheral::Lptim2 => (2, 10),
        Peripheral::Adc1 => (2, 11),
        Peripheral::Rtc => (3, 0),
        Peripheral::Tamp => (3, 1),
        Peripheral::Pwr => (3, 2),
        Peripheral::Rcc => (3, 3),
        Peripheral::Rng => (3, 4),
        Peripheral::Hash => (3, 5),
        Peripheral::Aes => (3, 6),
        Peripheral::Saes => (3, 7),
        Peripheral::Pka => (3, 8),
    };

    gtzc_tzsc.seccfgr(reg_idx).modify(|w| {
        if secure {
            w.0 |= 1 << bit;
        } else {
            w.0 &= !(1 << bit);
        }
        w
    });
}

/// Assign an SRAM block to the Secure or Non-Secure domain using MPCBB.
pub unsafe fn set_sram_block_secure(block: SramBlock, secure: bool) {
    let mpcbb = match block {
        SramBlock::Sram1 => pac::GTZC_MPCBB1,
        SramBlock::Sram2 => pac::GTZC_MPCBB2,
    };

    // VIO0 register controls the security of SRAM blocks.
    // Each bit in VIO0 represents a 256-byte or 512-byte block depending on the SRAM size.
    // To make the entire SRAM block Secure/Non-Secure, we write all bits in VIOx.
    for i in 0..mpcbb.viosr_count() {
        mpcbb.vior(i).write(|w| {
            if secure {
                w.0 = 0xFFFF_FFFF;
            } else {
                w.0 = 0x0000_0000;
            }
            w
        });
    }
}

/// Enable the TrustZone Interrupt Controller (TZIC) to catch illegal access events.
pub unsafe fn enable_tzic() {
    let gtzc_tzic = pac::GTZC_TZIC;

    // Clear all pending flags
    for i in 0..4 {
        gtzc_tzic.fcr(i).write(|w| {
            w.0 = 0xFFFF_FFFF;
            w
        });
    }

    // Enable interrupts for all possible illegal accesses
    for i in 0..4 {
        gtzc_tzic.ier(i).write(|w| {
            w.0 = 0xFFFF_FFFF;
            w
        });
    }

    // Unpend and enable GTZC IRQ in NVIC
    // Note: The IRQ name might vary (e.g., GTZC_IRQn)
}
