//! Platform adaptation layer and sequencer task for STM32WBA BLE Audio (`libble_audio.a`).
//!
//! Provides the `BLE_AUDIO_PLAT_*` C ABI callbacks required by ST's closed-source
//! BLE Audio stack and integrates `BLE_AUDIO_STACK_Task` with the WBA sequencer.

use core::ffi::c_void;

use embassy_time::{Duration, Instant};

use crate::wba::{linklayer_plat, util_seq};

/// Sequencer task ID for `BLE_AUDIO_STACK_Task` (after `CFG_TASK_BLE_HOST = 9`).
pub const CFG_TASK_BLE_AUDIO_STACK: u32 = 10;
pub const TASK_BLE_AUDIO_STACK_MASK: u32 = 1 << CFG_TASK_BLE_AUDIO_STACK;
const TASK_PRIO_BLE_AUDIO_STACK: u32 = 0;

type AudioTimerCallback = unsafe extern "C" fn(*mut c_void);

static mut AUDIO_TIMER_CB: Option<(AudioTimerCallback, *mut c_void)> = None;
static mut AUDIO_TIMER_DEADLINE: Instant = Instant::MAX;

unsafe extern "C" fn ble_audio_stack_task() {
    unsafe { crate::wba::bindings::ble_audio::BLE_AUDIO_STACK_Task() };
}

/// Register the BLE Audio stack background task with the WBA sequencer.
pub(crate) fn init() {
    unsafe {
        AUDIO_TIMER_CB = None;
        AUDIO_TIMER_DEADLINE = Instant::MAX;
    }
    util_seq::UTIL_SEQ_RegTask(TASK_BLE_AUDIO_STACK_MASK, 0, Some(ble_audio_stack_task));
}

/// Return the pending deadline of the BLE Audio one-shot timer, if armed.
pub(crate) fn earliest_timer_deadline() -> Instant {
    unsafe { AUDIO_TIMER_DEADLINE }
}

/// Fire the BLE Audio timer callback if its deadline has elapsed.
pub(crate) fn check_expired_timers() {
    unsafe {
        let deadline = AUDIO_TIMER_DEADLINE;
        if deadline != Instant::MAX && Instant::now() >= deadline {
            AUDIO_TIMER_DEADLINE = Instant::MAX;
            if let Some((cb, param)) = AUDIO_TIMER_CB.take() {
                cb(param);
                util_seq::seq_pend();
            }
        }
    }
}

/// Called by `libble_audio.a` whenever `BLE_AUDIO_STACK_Task()` needs to run.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLE_AUDIO_STACK_NotifyToRun() {
    util_seq::UTIL_SEQ_SetTask(TASK_BLE_AUDIO_STACK_MASK, TASK_PRIO_BLE_AUDIO_STACK);
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLE_AUDIO_PLAT_AesEcbEncrypt(key: *const u8, input: *const u8, output: *mut u8) {
    unsafe { linklayer_plat::BLEPLAT_AesEcbEncrypt(key, input, output) };
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLE_AUDIO_PLAT_AesCmacSetKey(key: *const u8) {
    unsafe { linklayer_plat::BLEPLAT_AesCmacSetKey(key) };
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLE_AUDIO_PLAT_AesCmacCompute(input: *const u8, input_length: u32, output_tag: *mut u8) {
    unsafe { linklayer_plat::BLEPLAT_AesCmacCompute(input, input_length, output_tag) };
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLE_AUDIO_PLAT_RngGet(n: u8, val: *mut u32) {
    unsafe { linklayer_plat::BLEPLAT_RngGet(n, val) };
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLE_AUDIO_PLAT_TimerStart(
    callback_func: *mut c_void,
    param: *mut c_void,
    timeout_ms: u32,
) {
    if callback_func.is_null() {
        return;
    }

    let cb: AudioTimerCallback = unsafe { core::mem::transmute(callback_func) };
    let deadline = Instant::now() + Duration::from_millis(timeout_ms as u64);

    unsafe {
        AUDIO_TIMER_CB = Some((cb, param));
        AUDIO_TIMER_DEADLINE = deadline;
    }
    util_seq::seq_pend();
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLE_AUDIO_PLAT_TimerStop() {
    unsafe {
        AUDIO_TIMER_CB = None;
        AUDIO_TIMER_DEADLINE = Instant::MAX;
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLE_AUDIO_PLAT_TimerGetRemainMs() -> u32 {
    let deadline = unsafe { AUDIO_TIMER_DEADLINE };
    if deadline == Instant::MAX {
        0
    } else {
        deadline
            .saturating_duration_since(Instant::now())
            .as_millis()
            .min(u32::MAX as u64) as u32
    }
}
