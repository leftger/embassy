//! Safe Rust facade over ST's `libcodec_mngr.a` (on-chip LC3 + `RCC->ASCR`
//! hardware AudioSync + `PLL1FRACN` clock recovery).
//!
//! Activating [`init`] sets the shared runtime bridge active for streams whose
//! ISO data path is configured as [`DataPathMode::SampleCircularBuffer`] or
//! [`DataPathMode::FrameCircularBuffer`]. Streams configured with
//! [`DataPathMode::HciLegacy`] (`0x00`) continue to bypass Codec Manager and
//! route over the standard HCI ISO data path (`HCI_RX_ISO_DATA_EVENT` /
//! `BLE_SendIsoDataToLinkLayer`).

use core::ffi::c_void;
use core::sync::atomic::{AtomicBool, AtomicUsize, Ordering};

use embassy_stm32::interrupt::typelevel;

pub use crate::wba::codec_if::{AudioSyncClockSource, AudioSyncInterruptHandler, CodecProcessInterruptHandler};
use crate::wba::bindings::ble_audio::{
    AUDIO_CorrectorConfig_t, AUDIO_DeinitializeClockCorrector, AUDIO_InitializeClockCorrector, AUDIO_PLLConfig_t,
    CODEC_DataPathID_t_DATA_PATH_FRAME_CIRC_BUF, CODEC_DataPathID_t_DATA_PATH_HCI_LEGACY,
    CODEC_DataPathID_t_DATA_PATH_SAMPLE_CIRC_BUF, CODEC_DataPathSampleParam_t, CODEC_LC3Config_t, CODEC_MODE_FLOW_CTRL,
    CODEC_MODE_FORCE_TRANSPARENT_CDC_ID, CODEC_MODE_WAIT_SYNC, CODEC_ManagerInit, CODEC_ManagerReset, CODEC_ReceiveData,
    CODEC_RegisterTriggerClbk, CODEC_SendData, CORRECTOR_MODE_BASIC, DATA_PATH_INPUT, DATA_PATH_OUTPUT,
    LC3_DECODER_STACK_SIZE_48KHZ, LC3_DECODER_STRUCT_SIZE_48KHZ, LC3_ENCODER_STACK_SIZE_48KHZ,
    LC3_ENCODER_STRUCT_SIZE_48KHZ, LC3_SESSION_STRUCT_SIZE_48KHZ, codec_status_t_CODEC_STATUS_SUCCESS,
};
use crate::wba::bindings::ble::hci_configure_data_path;
use crate::wba::{ble_codec, codec_if};

/// Number of `u32` words needed to hold one 48 kHz LC3 session state.
pub const fn words_for_bytes(bytes: u32) -> usize {
    (bytes as usize).div_ceil(4)
}

/// Recommended 48 kHz mono sink pool constants matching ST CubeWBA `TMAP_Peripheral`.
pub const DEFAULT_MEDIA_POOL_BYTES: usize = 6144;
pub const LC3_SESSION_48KHZ_WORDS: usize = words_for_bytes(LC3_SESSION_STRUCT_SIZE_48KHZ);
pub const LC3_DECODER_48KHZ_WORDS: usize = words_for_bytes(LC3_DECODER_STRUCT_SIZE_48KHZ);
pub const LC3_ENCODER_48KHZ_WORDS: usize = words_for_bytes(LC3_ENCODER_STRUCT_SIZE_48KHZ);
pub const LC3_MAX_STACK_48KHZ_WORDS: usize = words_for_bytes(
    if LC3_ENCODER_STACK_SIZE_48KHZ > LC3_DECODER_STACK_SIZE_48KHZ {
        LC3_ENCODER_STACK_SIZE_48KHZ
    } else {
        LC3_DECODER_STACK_SIZE_48KHZ
    },
);

static MONO_TO_STEREO_SAMPLES: AtomicUsize = AtomicUsize::new(0);
static INITIALIZED: AtomicBool = AtomicBool::new(false);

/// ISO data path transport mode selectable per stream.
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataPathMode {
    /// HCI ISO Data (`0x00`): controller passes raw SDUs to the application via
    /// `HCI_RX_ISO_DATA_EVENT` / `BLE_SendIsoDataToLinkLayer` (software `liblc3`).
    HciLegacy = CODEC_DataPathID_t_DATA_PATH_HCI_LEGACY,
    /// Vendor sample circular buffer (`0x01`): `libcodec_mngr.a` encodes/decodes LC3
    /// on-chip using `CODEC_ReceiveData` / `CODEC_SendData` and `RCC->ASCR` sync.
    SampleCircularBuffer = CODEC_DataPathID_t_DATA_PATH_SAMPLE_CIRC_BUF,
    /// Vendor frame circular buffer (`0x02`): frame-aligned circular buffering in
    /// `libcodec_mngr.a`.
    FrameCircularBuffer = CODEC_DataPathID_t_DATA_PATH_FRAME_CIRC_BUF,
}

/// Audio data path direction (with respect to the BLE Controller).
#[repr(u8)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum DataPathDirection {
    /// Host -> Controller (TX / Source).
    Input = DATA_PATH_INPUT as u8,
    /// Controller -> Host (RX / Sink).
    Output = DATA_PATH_OUTPUT as u8,
}

/// Errors returned by [`init`] and Codec Manager control calls.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CodecMngrError {
    AlreadyInitialized,
    ClockCorrectorInitFailed,
    CodecManagerInitFailed,
    HciError(u8),
    TriggerRegisterFailed,
}

/// Parameters describing the PLL generating the audio (`SAI`) bit clock so ST's
/// corrector knows the exact fractional step size for drift compensation.
#[derive(Clone, Copy, Debug)]
pub struct AudioPllConfig {
    /// Target audio bus clock frequency in Hz (e.g., `49_152_000` for 48 kHz audio).
    pub target_freq_hz: u32,
    /// VCO input frequency in Hz (`HSE / PLLM`, e.g., `16_000_000.0` for 32 MHz / 2).
    pub vco_input_freq_hz: f32,
    /// Output divider (`PLLQ` or `PLLP`, e.g., `8` for 49.152 MHz or `4` for 98.304 MHz).
    pub output_div: u32,
}

impl Default for AudioPllConfig {
    fn default() -> Self {
        Self {
            // Matches SAI1 driven from PLL1Q = 49.152 MHz with VCO = 393.216 MHz / 8
            // (HSE 32 MHz, PLLM=2 => 16 MHz VCO input).
            target_freq_hz: 49_152_000,
            vco_input_freq_hz: 16_000_000.0,
            output_div: 8,
        }
    }
}

/// `libcodec_mngr.a` clock drift corrector timing parameters.
#[derive(Clone, Copy, Debug)]
pub struct AudioCorrectorConfig {
    /// Sampling window in ms for initial fast frequency alignment (CubeWBA default: 250).
    pub initial_min_sampling_ms: u32,
    /// Sampling window in ms for continuous steady-state drift tracking (CubeWBA default: 500).
    pub min_sampling_ms: u32,
    /// Corrector algorithm mode (`CORRECTOR_MODE_BASIC = 0`).
    pub mode: u8,
}

impl Default for AudioCorrectorConfig {
    fn default() -> Self {
        Self {
            initial_min_sampling_ms: 250,
            min_sampling_ms: 500,
            mode: CORRECTOR_MODE_BASIC as u8,
        }
    }
}

/// Top-level initialization settings for [`init`].
#[derive(Clone, Copy, Debug)]
pub struct CodecMngrConfig {
    /// `RCC->CCIPR2.ASSEL` source feeding the `RCC->ASCR` 20-bit counter.
    pub clock_source: AudioSyncClockSource,
    /// Prescaler applied by `RCC->ASCR.PSC` (`1..=128`) so `AS_clk / psc ≈ 1.024 MHz`.
    pub timer_prescaler: u16,
    /// Enable `ASCR.CAS` hardware ISO capture mode; `false` uses software calibration.
    pub use_hw_capture: bool,
    /// Optional fixed CPU clock in kHz for LC3 MIPS/controller-delay estimation.
    pub core_clock_khz: Option<u32>,
    /// Extra processing margin in us added to min controller delay (CubeWBA default: 2000).
    pub margin_processing_us: u16,
    /// Max RF ISR setup latency in us (CubeWBA default: 1130).
    pub rf_max_setup_time_us: u16,
    /// Enable media-packet flow control backpressure (`CODEC_MODE_FLOW_CTRL`).
    pub flow_control: bool,
    /// Wait for controller ISO sync trigger before starting interface (`CODEC_MODE_WAIT_SYNC`).
    pub wait_sync: bool,
    /// Treat Codec ID as transparent even if set to LC3 (`CODEC_MODE_FORCE_TRANSPARENT_CDC_ID`).
    pub force_transparent_codec_id: bool,
    /// Audio PLL geometry for `PLL1FRACN` adjustments.
    pub pll_config: AudioPllConfig,
    /// Drift corrector timing parameters.
    pub corrector_config: AudioCorrectorConfig,
}

impl Default for CodecMngrConfig {
    fn default() -> Self {
        Self {
            clock_source: AudioSyncClockSource::Pll1P,
            timer_prescaler: 96,
            use_hw_capture: false,
            core_clock_khz: None,
            margin_processing_us: 2000,
            rf_max_setup_time_us: 1130,
            flow_control: true,
            wait_sync: true,
            force_transparent_codec_id: false,
            pll_config: AudioPllConfig::default(),
            corrector_config: AudioCorrectorConfig::default(),
        }
    }
}

/// Static 4-byte-aligned backing memory required by `libcodec_mngr.a` and ST's `liblc3.a`.
///
/// Typical 48 kHz mono sink allocation:
/// `CodecPool<6144, LC3_SESSION_48KHZ_WORDS, LC3_DECODER_48KHZ_WORDS, LC3_MAX_STACK_48KHZ_WORDS>`.
#[repr(C, align(4))]
pub struct CodecPool<
    const MEDIA_POOL_BYTES: usize,
    const SESSION_WORDS: usize,
    const CHANNEL_WORDS: usize,
    const STACK_WORDS: usize,
> {
    pub media_pool: [u8; MEDIA_POOL_BYTES],
    pub lc3_session: [u32; SESSION_WORDS],
    pub lc3_channel: [u32; CHANNEL_WORDS],
    pub lc3_stack: [u32; STACK_WORDS],
}

impl<
    const MEDIA_POOL_BYTES: usize,
    const SESSION_WORDS: usize,
    const CHANNEL_WORDS: usize,
    const STACK_WORDS: usize,
> CodecPool<MEDIA_POOL_BYTES, SESSION_WORDS, CHANNEL_WORDS, STACK_WORDS>
{
    pub const fn new() -> Self {
        Self {
            media_pool: [0; MEDIA_POOL_BYTES],
            lc3_session: [0; SESSION_WORDS],
            lc3_channel: [0; CHANNEL_WORDS],
            lc3_stack: [0; STACK_WORDS],
        }
    }

    fn as_lc3_config(&mut self) -> CODEC_LC3Config_t {
        CODEC_LC3Config_t {
            SessionSize: core::mem::size_of_val(&self.lc3_session) as u32,
            ChannelSize: core::mem::size_of_val(&self.lc3_channel) as u32,
            StackSize: core::mem::size_of_val(&self.lc3_stack) as u32,
            pSessionStart: self.lc3_session.as_mut_ptr().cast::<c_void>(),
            pChannelStart: self.lc3_channel.as_mut_ptr().cast::<c_void>(),
            pStackStart: self.lc3_stack.as_mut_ptr().cast::<c_void>(),
        }
    }
}

/// Initialize ST's `libcodec_mngr.a`, `RCC->ASCR` AudioSync hardware timer, and
/// `PLL1FRACN` clock corrector, and activate codec-managed ISO data paths.
///
/// The caller must bind both `interrupt::typelevel::COMP` -> [`CodecProcessInterruptHandler`]
/// and `interrupt::typelevel::RCC_AUDIOSYNC` -> [`AudioSyncInterruptHandler`].
pub fn init<
    const MEDIA_POOL_BYTES: usize,
    const SESSION_WORDS: usize,
    const CHANNEL_WORDS: usize,
    const STACK_WORDS: usize,
>(
    _irq: impl typelevel::Binding<typelevel::COMP, CodecProcessInterruptHandler>
    + typelevel::Binding<typelevel::RCC_AUDIOSYNC, AudioSyncInterruptHandler>,
    pool: &'static mut CodecPool<MEDIA_POOL_BYTES, SESSION_WORDS, CHANNEL_WORDS, STACK_WORDS>,
    config: CodecMngrConfig,
) -> Result<(), CodecMngrError> {
    if INITIALIZED.swap(true, Ordering::AcqRel) {
        return Err(CodecMngrError::AlreadyInitialized);
    }

    codec_if::configure(
        config.clock_source,
        config.timer_prescaler,
        config.use_hw_capture,
        config.core_clock_khz,
    );

    let mut pll_cfg = AUDIO_PLLConfig_t {
        PLLTargetFreq: config.pll_config.target_freq_hz,
        VCOInputFreq: config.pll_config.vco_input_freq_hz,
        PLLOutputDiv: config.pll_config.output_div,
    };
    let mut corr_cfg = AUDIO_CorrectorConfig_t {
        initialMinSampling: config.corrector_config.initial_min_sampling_ms,
        minSampling: config.corrector_config.min_sampling_ms,
        mode: config.corrector_config.mode,
    };

    let corr_status = unsafe { AUDIO_InitializeClockCorrector(&mut pll_cfg, &mut corr_cfg) };
    if corr_status != codec_status_t_CODEC_STATUS_SUCCESS {
        codec_if::deinit();
        INITIALIZED.store(false, Ordering::Release);
        return Err(CodecMngrError::ClockCorrectorInitFailed);
    }

    let mut codec_mode = 0u8;
    if config.flow_control {
        codec_mode |= CODEC_MODE_FLOW_CTRL as u8;
    }
    if config.wait_sync {
        codec_mode |= CODEC_MODE_WAIT_SYNC as u8;
    }
    if config.force_transparent_codec_id {
        codec_mode |= CODEC_MODE_FORCE_TRANSPARENT_CDC_ID as u8;
    }

    let mut lc3_cfg = pool.as_lc3_config();
    let media_ptr = pool.media_pool.as_mut_ptr();
    let media_len = pool.media_pool.len() as u32;

    let status = unsafe {
        CODEC_ManagerInit(
            media_len,
            media_ptr,
            &mut lc3_cfg,
            config.margin_processing_us,
            config.rf_max_setup_time_us,
            codec_mode,
        )
    };
    if status != codec_status_t_CODEC_STATUS_SUCCESS {
        unsafe { AUDIO_DeinitializeClockCorrector() };
        codec_if::deinit();
        INITIALIZED.store(false, Ordering::Release);
        return Err(CodecMngrError::CodecManagerInitFailed);
    }

    ble_codec::set_active(true);
    Ok(())
}

/// Deinitialize `libcodec_mngr.a`, stop `RCC->ASCR` and `COMP` interrupts, and
/// return all ISO streams to dormant/HCI mode.
pub fn deinit() {
    ble_codec::reset();
    MONO_TO_STEREO_SAMPLES.store(0, Ordering::Release);
    if INITIALIZED.swap(false, Ordering::AcqRel) {
        unsafe {
            CODEC_ManagerReset();
            AUDIO_DeinitializeClockCorrector();
        }
        codec_if::deinit();
    }
}

/// Returns `true` if [`init`] has succeeded and Codec Manager is currently active.
pub fn is_initialized() -> bool {
    INITIALIZED.load(Ordering::Acquire)
}

/// Configure `DATA_PATH_SAMPLE_CIRC_BUF` sample depth and channel decimation
/// via `HCI_CONFIGURE_DATA_PATH`.
pub fn configure_data_path_sample(
    direction: DataPathDirection,
    sample_depth_bits: u8,
    decimation: u8,
) -> Result<(), CodecMngrError> {
    let params = CODEC_DataPathSampleParam_t {
        SampleDepth: sample_depth_bits,
        Decimation: decimation,
    };
    let bytes = [params.SampleDepth, params.Decimation];
    let status = unsafe {
        hci_configure_data_path(
            direction as u8,
            DataPathMode::SampleCircularBuffer as u8,
            bytes.len() as u8,
            bytes.as_ptr(),
        )
    };
    if status == 0 {
        Ok(())
    } else {
        Err(CodecMngrError::HciError(status))
    }
}

/// Register (or clear with `None`) a trigger callback fired when the BLE ISO
/// controller timing reaches the point where the audio peripheral (`SAI`)
/// should start in phase with Controller Delay.
pub fn register_trigger_callback(
    mode: DataPathMode,
    direction: DataPathDirection,
    callback: Option<unsafe extern "C" fn() -> i32>,
) -> Result<(), CodecMngrError> {
    let status = unsafe { CODEC_RegisterTriggerClbk(mode as u8, direction as u8, callback) };
    if status == codec_status_t_CODEC_STATUS_SUCCESS {
        Ok(())
    } else {
        Err(CodecMngrError::TriggerRegisterFailed)
    }
}

/// Enable or disable automatic mono (`channel_nb == 1`) Left-to-Right `i16`
/// sample duplication in [`CODEC_NotifyDataReady`] for interleaved stereo I2S/SAI
/// output (`decimation == 2`). Pass `0` to disable, or `samples_per_frame` (e.g. `480`
/// for 48 kHz / 10 ms) to enable.
pub fn set_mono_to_stereo_samples(samples_per_frame: usize) {
    MONO_TO_STEREO_SAMPLES.store(samples_per_frame, Ordering::Release);
}

/// Pull decoded PCM samples for a sink stream from `libcodec_mngr.a` into `dst`.
///
/// Typically invoked once per half-frame from the `SAI` DMA circular buffer loop
/// (or IRQ).
#[inline]
pub fn receive_data(iso_conn_handle: u16, mode: DataPathMode, dst: &mut [i16]) {
    unsafe {
        CODEC_ReceiveData(iso_conn_handle, mode as u8, dst.as_mut_ptr().cast::<c_void>());
    }
}

/// Push captured PCM samples for a source stream into `libcodec_mngr.a`.
#[inline]
pub fn send_data(iso_conn_handle: u16, mode: DataPathMode, src: &[i16]) {
    unsafe {
        CODEC_SendData(iso_conn_handle, mode as u8, src.as_ptr() as *mut c_void);
    }
}

/// Strong override of ST's weak `CODEC_NotifyDataReady` callback invoked after
/// decoding a sink frame. When [`set_mono_to_stereo_samples`] is non-zero and a
/// stream has a single mono channel (`channel_nb == 1`), duplicates each decoded
/// Left `i16` sample into the adjacent Right slot (`decimation = 2`).
#[unsafe(no_mangle)]
pub unsafe extern "C" fn CODEC_NotifyDataReady(
    _iso_con_hdl: u16,
    pdecoded_data: *mut c_void,
    _channel_idx: u8,
    channel_nb: u8,
) {
    if channel_nb != 1 || pdecoded_data.is_null() {
        return;
    }
    let samples = MONO_TO_STEREO_SAMPLES.load(Ordering::Relaxed);
    if samples == 0 {
        return;
    }
    let pcm = pdecoded_data.cast::<i16>();
    for i in 0..samples {
        unsafe {
            let sample = *pcm.add(2 * i);
            *pcm.add(2 * i + 1) = sample;
        }
    }
}
