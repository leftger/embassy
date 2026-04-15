//! TrustZone boot utilities.
//!
//! This module provides functions to transition the processor from Secure to Non-Secure state.

use core::arch::asm;

/// Jump to the Non-Secure application.
///
/// This function performs the following steps:
/// 1. Sets the Non-Secure Vector Table Offset Register (VTOR_NS).
/// 2. Sets the Non-Secure Main Stack Pointer (MSP_NS) from the first entry of the NS vector table.
/// 3. Jumps to the Non-Secure Reset Handler (second entry of the NS vector table) using the `BXNS` instruction.
///
/// # Safety
/// This function is extremely unsafe as it transitions the processor state and jumps to an arbitrary address.
/// Ensure that the SAU and GTZC are correctly configured before calling this function, or a SecureFault will occur.
pub unsafe fn jump_to_non_secure(ns_vector_table_address: u32) -> ! {
    let ns_vector_table = ns_vector_table_address as *const u32;

    // 1. Get initial MSP and Reset Handler from the Non-Secure vector table.
    let ns_msp = *ns_vector_table;
    let ns_reset_handler = *ns_vector_table.add(1);

    // 2. Set VTOR_NS (Vector Table Offset Register - Non-Secure).
    // Address: 0xE000ED08 (S) / 0xE002ED08 (NS) - but we use the S alias for NS: 0xE002ED08.
    // However, the standard way in ARMv8-M is to use the MSR instruction.
    asm!(
        "msr vtor_ns, {}",
        in(reg) ns_vector_table_address,
    );

    // 3. Set MSP_NS (Main Stack Pointer - Non-Secure).
    asm!(
        "msr msp_ns, {}",
        in(reg) ns_msp,
    );

    // 4. Prepare for the jump.
    // The target address for BXNS must have the LSB cleared to 0 to indicate a transition to Non-Secure state.
    let target = ns_reset_handler & !1;

    // TODO: Clear R0-R12 and other registers to prevent data leakage from Secure to Non-Secure world.

    // 5. Jump to Non-Secure world.
    asm!(
        "bxns {}",
        in(reg) target,
        options(noreturn),
    );
}
