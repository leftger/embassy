//! Hardware AudioSync (`RCC->ASCR`), PLL1 fractional tuning, and process IRQ glue
//! for ST's `libcodec_mngr.a`.

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU16, AtomicU32, Ordering};

use embassy_stm32::interrupt;
use embassy_stm32::pac::RCC;
use embassy_stm32::pac::rcc::vals::Assel;
use embassy_stm32::rcc::{pll1_fracn, set_pll1_fracn};

use crate::wba::bindings::ble_audio::{
    CODEC_CLK_Provide_ISO_Captured_Timestamp, CODEC_CLK_trigger_event_notify, CODEC_IRQReq_t,
    CODEC_IRQReq_t_CODEC_IRQ_DISABLE, CODEC_ManagerProcess, CODEC_MaskSrc_t, CODEC_TraceEvnt_t,
};
use crate::wba::linklayer_plat::{nvic_disable, nvic_enable, nvic_set_pending, nvic_set_priority, pack_priority};

const COMP_INTR_NUM: u32 = embassy_stm32::pac::Interrupt::COMP as u32;
const AUDIOSYNC_INTR_NUM: u32 = embassy_stm32::pac::Interrupt::RCC_AUDIOSYNC as u32;

/// Lowest logical priority, matching ST's `CODEC_MNGR_INTR_PRIO = 15`.
const CODEC_MNGR_INTR_PRIO: u32 = 15;
/// Audio timer interrupt priority, matching ST's `TIMER_IT_PRIO = 4`.
const TIMER_IT_PRIO: u32 = 4;

const MAX_TIMER_NB: usize = 7; // MAX_PATH_NB + 1
const AS_AUTORELOAD: u32 = 0x000F_FFFF; // 20-bit counter max
const AS_COMPARE_MAX: u32 = 1_024_000; // ~1 s at 1.024 MHz counter tick
const AS_ACCEPTABLE_WINDOWS: u32 = 100;

/// Expected core clock (MHz) indexed by LE Audio Assigned Numbers sampling frequency index,
/// matching ST's `LC3_EXE_CLOCK_MHZ` table.
const LC3_EXE_CLOCK_MHZ: [f32; 13] = [
    98.304, 0.0, 98.304, 0.0, 98.304, 98.304, 90.3168, 98.304, 0.0, 0.0, 0.0, 0.0, 0.0,
];

/// Clock input for the STM32WBA `RCC->ASCR` AudioSync counter (`RCC->CCIPR2.ASSEL`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum AudioSyncClockSource {
    /// `pll1pclk` (ST CubeWBA default when `SAI1` runs from `PLL1P`).
    Pll1P,
    /// `pll1qclk` (useful when `SAI1` runs from `PLL1Q`, e.g., 49.152 MHz).
    Pll1Q,
}

#[derive(Clone, Copy)]
struct AudioTimerSlot {
    timestamp: u32,
    is_active: bool,
}

impl AudioTimerSlot {
    const fn inactive() -> Self {
        Self {
            timestamp: 0,
            is_active: false,
        }
    }
}

struct AudioTimerState {
    cnt_base: u32,
    is_autoreloading: bool,
    slots: [AudioTimerSlot; MAX_TIMER_NB],
}

impl AudioTimerState {
    const fn new() -> Self {
        Self {
            cnt_base: 0,
            is_autoreloading: false,
            slots: [AudioTimerSlot::inactive(); MAX_TIMER_NB],
        }
    }
}

static mut AUDIO_TIMER_STATE: AudioTimerState = AudioTimerState::new();
static CODEC_IRQ_MASK_REQ: AtomicU8 = AtomicU8::new(0);
static AS_CLK_SOURCE: AtomicU8 = AtomicU8::new(0); // 0 = Pll1P, 1 = Pll1Q
static AS_PRESCALER: AtomicU16 = AtomicU16::new(96);
static AS_HW_CAPTURE: AtomicBool = AtomicBool::new(false);
static AS_CORE_CLK_OVERRIDE_KHZ: AtomicU32 = AtomicU32::new(0);

/// Interrupt handler for `COMP` (software-pended IRQ running `CODEC_ManagerProcess()`).
pub struct CodecProcessInterruptHandler {}

impl embassy_stm32::interrupt::typelevel::Handler<interrupt::typelevel::COMP> for CodecProcessInterruptHandler {
    unsafe fn on_interrupt() {
        CODEC_ManagerProcess();
    }
}

/// Interrupt handler for `RCC_AUDIOSYNC` (compare triggers & Link Layer ISO hardware capture).
pub struct AudioSyncInterruptHandler {}

impl embassy_stm32::interrupt::typelevel::Handler<interrupt::typelevel::RCC_AUDIOSYNC>
    for AudioSyncInterruptHandler
{
    unsafe fn on_interrupt() {
        on_audiosync_irq();
    }
}

/// Apply runtime configuration before initializing `libcodec_mngr.a`.
pub(crate) fn configure(
    clock_source: AudioSyncClockSource,
    timer_prescaler: u16,
    use_hw_capture: bool,
    core_clock_khz: Option<u32>,
) {
    let src_val = match clock_source {
        AudioSyncClockSource::Pll1P => 0,
        AudioSyncClockSource::Pll1Q => 1,
    };
    AS_CLK_SOURCE.store(src_val, Ordering::Release);
    AS_PRESCALER.store(timer_prescaler.clamp(1, 128), Ordering::Release);
    AS_HW_CAPTURE.store(use_hw_capture, Ordering::Release);
    AS_CORE_CLK_OVERRIDE_KHZ.store(core_clock_khz.unwrap_or(0), Ordering::Release);
}

/// Disable AudioSync + Codec process IRQs and reset internal timer state.
pub(crate) fn deinit() {
    critical_section::with(|_| unsafe {
        nvic_disable(AUDIOSYNC_INTR_NUM);
        nvic_disable(COMP_INTR_NUM);
        RCC.ascr().modify(|w| w.set_cen(false));
        RCC.asier().write(|_| {});
        AUDIO_TIMER_STATE = AudioTimerState::new();
        CODEC_IRQ_MASK_REQ.store(0, Ordering::Release);
    });
}

#[inline]
fn selected_assel() -> Assel {
    if AS_CLK_SOURCE.load(Ordering::Acquire) == 1 {
        Assel::Pll1Q
    } else {
        Assel::Pll1P
    }
}

#[inline]
fn is_rcc_as_clocked() -> bool {
    let pll_on = RCC.cr().read().pllon();
    if !pll_on {
        return false;
    }
    let cfgr = RCC.pll1cfgr().read();
    match selected_assel() {
        Assel::Pll1P => cfgr.pllpen(),
        Assel::Pll1Q => cfgr.pllqen(),
    }
}

#[inline]
fn is_rcc_as_running() -> bool {
    is_rcc_as_clocked() && RCC.ascr().read().cen()
}

/// Program `RCC->ASARR`, `RCC->ASCR`, `RCC->ASIER`, and `RCC->ASCOR`, resetting and starting `CEN`.
#[inline]
unsafe fn program_ascr_unlocked(arr: u32, cmp: u32) {
    let prescaler = AS_PRESCALER.load(Ordering::Acquire);
    let psc = prescaler.saturating_sub(1).min(0x7F) as u8;

    RCC.ascr().modify(|w| w.set_cen(false));
    cortex_m::asm::nop();
    cortex_m::asm::nop();
    cortex_m::asm::nop();

    RCC.asarr().write(|w| w.set_ar(arr));
    RCC.ascr().write(|w| {
        w.set_psc(psc);
        w.set_cps(0);
    });
    RCC.asier().write(|w| {
        w.set_coie(true);
        w.set_caie(true);
    });
    RCC.ascor().write(|w| w.set_co(cmp));
    RCC.ascr().modify(|w| w.set_cen(true));
}

#[inline]
unsafe fn host_timestamp_unlocked(st: &AudioTimerState) -> u32 {
    let mut as_cnt_val = if is_rcc_as_running() {
        RCC.ascntr().read().cnt()
    } else {
        0
    };

    if RCC.assr().read().cof() && st.is_autoreloading && as_cnt_val != AS_COMPARE_MAX {
        as_cnt_val = as_cnt_val.wrapping_add(AS_COMPARE_MAX + 1);
    }

    st.cnt_base.wrapping_add(as_cnt_val)
}

unsafe fn init_audio_timer() {
    let assel = selected_assel();
    RCC.ccipr2().modify(|w| w.set_assel(assel));

    critical_section::with(|_| {
        let st = &mut *core::ptr::addr_of_mut!(AUDIO_TIMER_STATE);
        *st = AudioTimerState::new();

        if is_rcc_as_clocked() {
            program_ascr_unlocked(AS_COMPARE_MAX, AS_COMPARE_MAX);
            st.is_autoreloading = true;
            debug!("RCC AudioSync initialized (prescaler={})", AS_PRESCALER.load(Ordering::Acquire));
        } else {
            warn!("RCC AudioSync cannot start: selected PLL1 output is disabled");
        }
    });

    nvic_set_priority(AUDIOSYNC_INTR_NUM, pack_priority(TIMER_IT_PRIO));
    nvic_enable(AUDIOSYNC_INTR_NUM);
}

unsafe fn manage_compare_it() {
    let st = &mut *core::ptr::addr_of_mut!(AUDIO_TIMER_STATE);
    let current_timestamp = st.cnt_base.wrapping_add(RCC.ascor().read().co());
    let mut next_id: Option<u8> = None;
    let mut best_delta = u32::MAX;

    for i in 0..MAX_TIMER_NB {
        if st.slots[i].is_active {
            let delta = st.slots[i].timestamp.wrapping_sub(current_timestamp);
            if delta < AS_ACCEPTABLE_WINDOWS || delta > 0u32.wrapping_sub(AS_ACCEPTABLE_WINDOWS) {
                st.slots[i].is_active = false;
                CODEC_CLK_trigger_event_notify(i as u8);
            } else if delta < best_delta {
                next_id = Some(i as u8);
                best_delta = delta;
            }
        }
    }

    // Ensure ASCNTR advances past COMPARE before inspecting/resetting it.
    if is_rcc_as_running() {
        let mut spins = 0u32;
        while st.cnt_base.wrapping_add(RCC.ascntr().read().cnt()) == current_timestamp {
            spins += 1;
            if spins > 10_000 {
                break;
            }
        }
    }

    if let Some(id) = next_id {
        let ts = st.slots[id as usize].timestamp;
        let _ = CODEC_CLK_RequestTimerEvent(id, ts);
    } else if !st.is_autoreloading {
        critical_section::with(|_| {
            // Wait for one counter tick before reading & resetting CNT.
            if is_rcc_as_running() {
                let prev = RCC.ascntr().read().cnt();
                let mut spins = 0u32;
                while RCC.ascntr().read().cnt() == prev {
                    spins += 1;
                    if spins > 10_000 {
                        break;
                    }
                }
            }
            st.cnt_base = st.cnt_base.wrapping_add(RCC.ascntr().read().cnt() + 1);
            program_ascr_unlocked(AS_COMPARE_MAX, AS_COMPARE_MAX);
            st.is_autoreloading = true;
        });
    } else {
        st.cnt_base = st.cnt_base.wrapping_add(AS_COMPARE_MAX + 1);
    }
}

unsafe fn on_audiosync_irq() {
    let assr = RCC.assr().read();

    if assr.cof() {
        manage_compare_it();
        RCC.assr().modify(|w| w.set_cof(false));
    }

    if assr.caf() {
        if AS_HW_CAPTURE.load(Ordering::Acquire) {
            let captured = RCC.ascar().read().ca() & AS_AUTORELOAD;
            if captured != 0 {
                let base = critical_section::with(|_| (*core::ptr::addr_of!(AUDIO_TIMER_STATE)).cnt_base);
                CODEC_CLK_Provide_ISO_Captured_Timestamp(base.wrapping_add(captured));
            }
        }
        RCC.assr().modify(|w| w.set_caf(false));
    }

    if assr.caef() {
        RCC.assr().modify(|w| w.set_caef(false));
    }
}

// ── ST `libcodec_mngr.a` C ABI Callbacks ─────────────────────────────────────

#[unsafe(no_mangle)]
pub unsafe extern "C" fn CODEC_ProcessInit() {
    nvic_set_priority(COMP_INTR_NUM, pack_priority(CODEC_MNGR_INTR_PRIO));
    nvic_enable(COMP_INTR_NUM);
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn CODEC_ProcessReq() {
    nvic_set_pending(COMP_INTR_NUM);
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn CODEC_ReqIRQState(src: CODEC_MaskSrc_t, req: CODEC_IRQReq_t) {
    critical_section::with(|_| {
        let mut mask = CODEC_IRQ_MASK_REQ.load(Ordering::Relaxed);
        if req == CODEC_IRQReq_t_CODEC_IRQ_DISABLE {
            mask |= src;
            nvic_disable(COMP_INTR_NUM);
        } else {
            mask &= !src;
            if mask == 0 {
                nvic_enable(COMP_INTR_NUM);
            }
        }
        CODEC_IRQ_MASK_REQ.store(mask, Ordering::Release);
    });
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn CODEC_TraceEvnt(_evnt: CODEC_TraceEvnt_t) {}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn CODEC_CLK_Init() -> i32 {
    init_audio_timer();
    0
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn CODEC_CLK_GetHostTimestamp() -> u32 {
    critical_section::with(|_| {
        let st = &*core::ptr::addr_of!(AUDIO_TIMER_STATE);
        host_timestamp_unlocked(st)
    })
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn CODEC_CLK_GetTimerPrescaler() -> u16 {
    AS_PRESCALER.load(Ordering::Acquire)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn CODEC_CLK_GetCoreClock(freq_index: u8) -> f32 {
    let custom_khz = AS_CORE_CLK_OVERRIDE_KHZ.load(Ordering::Acquire);
    if custom_khz != 0 {
        return (custom_khz as f32) / 1000.0;
    }
    LC3_EXE_CLOCK_MHZ.get(freq_index as usize).copied().unwrap_or(0.0)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn CODEC_CLK_GetPLLNfrac() -> u16 {
    pll1_fracn()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn CODEC_CLK_SetPLLNfrac(n_frac: u16) {
    set_pll1_fracn(n_frac);
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn CODEC_CLK_RequestTimerEvent(id: u8, trigger_ts: u32) -> i32 {
    let idx = id as usize;
    if idx >= MAX_TIMER_NB {
        return -1;
    }

    critical_section::with(|_| {
        let st = &mut *core::ptr::addr_of_mut!(AUDIO_TIMER_STATE);
        st.slots[idx].timestamp = trigger_ts;
        st.slots[idx].is_active = true;

        let mut is_closest = true;
        for slot in st.slots.iter() {
            if slot.is_active && slot.timestamp.wrapping_sub(trigger_ts) > 0x8000_0000 {
                is_closest = false;
                break;
            }
        }

        if is_closest && is_rcc_as_running() {
            let now = host_timestamp_unlocked(st);
            let tmp_ascor = trigger_ts.wrapping_sub(now).min(AS_COMPARE_MAX);

            if RCC.ascor().read().co() != tmp_ascor || RCC.asarr().read().ar() != AS_AUTORELOAD {
                st.cnt_base = st.cnt_base.wrapping_add(RCC.ascntr().read().cnt());
                program_ascr_unlocked(AS_AUTORELOAD, tmp_ascor);
            }
            st.is_autoreloading = false;
        }
    });

    0
}
