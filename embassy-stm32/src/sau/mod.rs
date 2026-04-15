//! Security Attribution Unit (SAU) configuration.
//!
//! This module provides a way to configure the SAU on Cortex-M33 cores.

use cortex_m::peripheral::SAU;

/// SAU region configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Region {
    /// Base address of the region.
    pub base_address: u32,
    /// End address of the region (inclusive).
    pub end_address: u32,
    /// Security attribute of the region.
    pub attribute: Attribute,
}

/// SAU region security attribute.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Attribute {
    /// Non-Secure.
    NonSecure = 0,
    /// Non-Secure Callable.
    NonSecureCallable = 1,
}

/// Initialize the SAU with the provided regions.
///
/// This function disables the SAU, configures the regions, and then enables it.
/// It also enables the SecureFault exception.
pub unsafe fn init(regions: &[Region]) {
    let mut sau = unsafe { cortex_m::Peripherals::steal().SAU };

    // Disable SAU before configuration
    sau.ctrl.modify(|mut w| {
        w.0 &= !1; // ENABLE = 0
        w
    });

    // Configure regions
    for (i, region) in regions.iter().enumerate() {
        if i >= 8 {
            break; // Standard M33 has 8 regions
        }

        sau.rnr.write(|mut w| {
            w.0 = i as u32;
            w
        });
        sau.rbar.write(|mut w| {
            w.0 = region.base_address & !0x1F;
            w
        });
        sau.rlar.write(|mut w| {
            w.0 = (region.end_address & !0x1F) | ((region.attribute as u32) << 1) | 1; // NSC bit and ENABLE bit
            w
        });
    }

    // Enable SAU and SecureFault
    sau.ctrl.modify(|mut w| {
        w.0 |= 1 | (1 << 1); // ENABLE = 1, ALLNS = 1 (all memory is Non-Secure by default if not matching any region)
        w
    });

    let mut scb = unsafe { cortex_m::Peripherals::steal().SCB };
    // Enable SecureFault
    scb.shcsr.modify(|mut w| {
        w.0 |= 1 << 19; // SECUREFAULTENA
        w
    });
}
