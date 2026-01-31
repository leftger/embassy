//! BLE Stack Runner for Embassy Integration
//!
//! This module provides the runner that drives the BLE sequencer while
//! integrating properly with the embassy async executor.
//!
//! # Architecture
//!
//! The BLE stack runs in a separate context (with its own stack) managed by
//! the context switching module. The runner:
//!
//! 1. Resumes the sequencer context
//! 2. The sequencer processes pending tasks
//! 3. When idle, the sequencer yields back
//! 4. The runner yields to the embassy executor
//! 5. When woken (by interrupt), repeats from step 1
//!
//! # Usage
//!
//! The runner must be spawned as a separate embassy task:
//!
//! ```no_run
//! use embassy_executor::Spawner;
//! use embassy_stm32_wpan::wba::ble_runner;
//!
//! #[embassy_executor::task]
//! async fn ble_task() {
//!     ble_runner().await
//! }
//!
//! #[embassy_executor::main]
//! async fn main(spawner: Spawner) {
//!     // Initialize BLE stack first...
//!
//!     // Spawn the BLE runner task
//!     spawner.spawn(ble_task()).unwrap();
//!
//!     // Your application logic...
//! }
//! ```

// Note: complete_ble_link_layer_init is now called as part of init_ble_stack()
// in Ble::init(), so we no longer need to call it from the runner.

// BleStack_Process return values
const BLE_SLEEPMODE_RUNNING: u8 = 0;

// External BLE stack process function
#[link(name = "stm32wba_ble_stack_basic")]
unsafe extern "C" {
    /// BLE stack process function - must be called to process BLE events
    fn BleStack_Process() -> u8;
}

// Task IDs for sequencer (matching ST's CFG_TASK_* defines)
const CFG_TASK_BLE_HOST: u32 = 0;  // BleStack_Process task
const TASK_BLE_HOST_MASK: u32 = 1 << CFG_TASK_BLE_HOST;
const TASK_PRIO_BLE_HOST: u32 = 0;  // High priority

/// BleStack_Process background task wrapper
///
/// This matches ST's BleStack_Process_BG function.
/// Per ST: BLE_SLEEPMODE_RUNNING (0) means "has to be executed again"
unsafe extern "C" fn ble_stack_process_bg() {
    #[cfg(feature = "defmt")]
    defmt::trace!("BleStack_Process_BG called");

    let result = BleStack_Process();

    #[cfg(feature = "defmt")]
    defmt::trace!("BleStack_Process returned: {}", result);

    // Per ST's implementation: when result == 0 (BLE_SLEEPMODE_RUNNING),
    // the BLE stack needs to run again. Reschedule the task.
    // This matches ST's BleStackCB_Process() which calls:
    //   UTIL_SEQ_SetTask(1U << CFG_TASK_BLE_HOST, CFG_SEQ_PRIO_0);
    if result == BLE_SLEEPMODE_RUNNING {
        super::util_seq::UTIL_SEQ_SetTask(TASK_BLE_HOST_MASK, TASK_PRIO_BLE_HOST);
    }
}

/// Register BLE stack tasks with the sequencer
///
/// This must be called during initialization to register BleStack_Process
/// as a sequencer task (matching ST's APP_BLE_Init behavior).
pub fn register_ble_tasks() {
    // Register BLE Host stack process task (like ST's APP_BLE_Init does)
    super::util_seq::UTIL_SEQ_RegTask(TASK_BLE_HOST_MASK, 0, Some(ble_stack_process_bg));

    #[cfg(feature = "defmt")]
    defmt::info!("BLE Host task registered with sequencer");
}

/// Schedule the BLE Host task to run
///
/// This triggers BleStack_Process to execute. Should be called after
/// events that require BLE stack processing (e.g., after starting advertising).
pub fn schedule_ble_host_task() {
    super::util_seq::UTIL_SEQ_SetTask(TASK_BLE_HOST_MASK, TASK_PRIO_BLE_HOST);

    #[cfg(feature = "defmt")]
    defmt::trace!("BLE Host task scheduled");
}

/// Pump the BLE stack to process pending work.
///
/// This function schedules the BLE host task and runs the sequencer repeatedly
/// until there's no more work to do. This matches ST's main loop pattern where
/// UTIL_SEQ_Run(UTIL_SEQ_DEFAULT) is called repeatedly in MX_APPE_Process().
///
/// This is safe to call during initialization before the async BLE runner starts,
/// as it directly drives the sequencer instead of relying on the runner task.
pub fn pump_ble_stack() {
    // Schedule the BLE host task to ensure it runs
    schedule_ble_host_task();

    // Run the sequencer until there's no more pending work.
    // Keep running while the sequencer has tasks to execute.
    let mut iterations = 0;
    loop {
        let had_work = super::util_seq::run(super::util_seq::UTIL_SEQ_DEFAULT);

        iterations += 1;

        // If no tasks were executed and no more work is pending, we're done
        if !had_work && !super::util_seq::has_pending_work() {
            break;
        }

        // Safety limit to prevent infinite loop
        if iterations >= 50 {
            #[cfg(feature = "defmt")]
            defmt::warn!("pump_ble_stack: {} iterations, breaking", iterations);
            break;
        }
    }

    #[cfg(feature = "defmt")]
    if iterations > 5 {
        defmt::trace!("pump_ble_stack: completed after {} iterations", iterations);
    }
}

/// Internal function to process BLE stack in the async runner
///
/// This just runs the sequencer once to process any pending tasks.
/// The sequencer will run both BLE host and link layer tasks as needed.
fn process_ble_stack() {
    // Just run the sequencer to process any pending tasks
    // The sequencer will handle calling BleStack_Process via the BLE host task
    super::util_seq::run(super::util_seq::UTIL_SEQ_DEFAULT);
}

/// BLE stack runner function
///
/// This async function drives the BLE stack by continuously running the sequencer.
/// It matches ST's main loop pattern: while(1) { UTIL_SEQ_Run(UTIL_SEQ_DEFAULT); }
///
/// This function should be spawned as a background task to keep the BLE stack processing.
pub async fn ble_runner() -> ! {
    #[cfg(feature = "defmt")]
    defmt::info!("BLE runner started - processing sequencer continuously");

    let mut iteration_count = 0u32;

    loop {
        // Run the sequencer to process any pending BLE tasks
        // This matches ST's pattern: UTIL_SEQ_Run(UTIL_SEQ_DEFAULT) in a while loop
        process_ble_stack();

        iteration_count = iteration_count.wrapping_add(1);

        // Log periodically to show runner is active
        #[cfg(feature = "defmt")]
        if iteration_count % 1000 == 0 {
            defmt::trace!("BLE runner: {} iterations", iteration_count);
        }

        // Yield to allow other async tasks to run
        embassy_futures::yield_now().await;
    }
}

/// Called from radio interrupt handler
///
/// This schedules the link layer task to run
pub fn on_radio_interrupt() {
    schedule_ble_host_task();
}
