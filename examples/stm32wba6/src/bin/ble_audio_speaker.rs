//! BLE LE Audio Unicast Speaker Example (`CAP Acceptor` / `BAP Unicast Server`)
//!
//! Advertises as a Bluetooth LE Audio speaker (`"WBA6_Speaker"`) exposing:
//! - Published Audio Capabilities Service (`PACS` 0x1850): Mono LC3 sink (16/24/48 kHz, 7.5/10 ms)
//! - Audio Stream Control Service (`ASCS` 0x184E): 1 Sink ASE over CIS isochronous channel
//! - Volume Control Service (`VCS` 0x1844) + Common Audio Service (`CAS` 0x1853)
//!
//! Isochronous LC3 audio supports two compile-time selectable modes:
//! - Default (`--features ble-audio`): `HCI_RX_ISO_DATA_EVENT` (`0x00` HCI Legacy
//!   Data Path) decoded with ST's `liblc3.a` in an async Embassy task.
//! - Codec Manager (`--features ble-audio-codec-mngr`): `libcodec_mngr.a`
//!   (`DATA_PATH_SAMPLE_CIRC_BUF` `0x01`), `RCC->ASCR` hardware AudioSync,
//!   live `PLL1FRACN` clock recovery, and controller-triggered SAI playback.

#![no_std]
#![no_main]

#[cfg(not(feature = "ble-audio-codec-mngr"))]
use core::ffi::c_void;
use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU8, Ordering};

use defmt::*;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::sai::{self, Sai};
use embassy_stm32::{Config, bind_interrupts, dma, peripherals, rcc};
use embassy_stm32_wpan::bindings::ble::{
    Adv_Set_t, HCI_ADV_EVENT_PROP_CONNECTABLE, HCI_ADV_TYPE_ADV_IND, aci_gap_adv_set_configuration,
    hci_le_set_extended_advertising_data, hci_le_set_extended_advertising_enable,
};
#[cfg(not(feature = "ble-audio-codec-mngr"))]
use embassy_stm32_wpan::bindings::ble::BLE_STATUS_SUCCESS;
use embassy_stm32_wpan::bindings::ble_audio::{
    ASE_OP_RESP_NO_REASON, ASE_OP_RESP_REASON_CODEC_SPEC_CONF, ASE_OP_RESP_REASON_PRES_DELAY,
    ASE_OP_RESP_SUCCESS, ASE_OP_RESP_UNSUPPORTED_CONF_PARAM, AUDIO_CODING_FORMAT_LC3,
    AUDIO_CONTEXT_CONVERSATIONAL, AUDIO_CONTEXT_MEDIA, AUDIO_CONTEXT_UNSPECIFIED, AUDIO_ROLE_SINK,
    BAP_ASE_Enable_Req_Params_t, BAP_AUDIO_PATH_OUTPUT, BAP_AudioPACRecord_t, BAP_Config_t,
    BAP_FRAMING_UNFRAMED, BAP_PrefQoSConfReq_t, BAP_ROLE_UNICAST_SERVER,
    BAP_SetupAudioDataPathReq_t, BAP_Unicast_Audio_Path_t, BleAudioInit_t, BLE_AUDIO_STACK_Init,
    CAP_BuildAdvPacket, CAP_Config_t, CAP_Init, CAP_NotCode_t_CAP_AUDIO_CLOCK_REQ_EVT,
    CAP_NotCode_t_CAP_LINKUP_COMPLETE_EVT, CAP_NotCode_t_CAP_UNICAST_AUDIO_CONNECTION_DOWN_EVT,
    CAP_NotCode_t_CAP_UNICAST_AUDIO_CONNECTION_UP_EVT,
    CAP_NotCode_t_CAP_UNICAST_AUDIO_DATA_PATH_SETUP_REQ_EVT,
    CAP_NotCode_t_CAP_UNICAST_PREFERRED_QOS_REQ_EVT,
    CAP_NotCode_t_CAP_UNICAST_SERVER_ASE_STATE_EVT,
    CAP_NotCode_t_CAP_UNICAST_SERVER_ENABLE_REQ_EVT, CAP_NotCode_t_CAP_VCP_META_EVT,
    CAP_Notification_Evt_t, CAP_ROLE_ACCEPTOR, CAP_RegisterSnkPACRecord,
    CAP_SetAvailableAudioContexts, CAP_SetSnkAudioLocations, CAP_SetSupportedAudioContexts,
    CAP_Unicast_EnableOpRsp, CAP_Unicast_SetupAudioDataPath, CCP_Config_t,
    CODEC_CAP_AUDIO_CHNL_COUNTS, CODEC_CAP_FRAME_DURATION, CODEC_CAP_OCTETS_PER_CODEC_FRAME,
    CODEC_CAP_SAMPLING_FREQ, CSIP_Config_t, FRAME_DURATION_7_5_MS, FRAME_DURATION_10_MS,
    LE_2M_PHY_PREFERRED, LTV_GetConfiguredFrameDuration, LTV_GetConfiguredFramesBlockPerSdu,
    LTV_GetConfiguredOctetsPerCodecFrame, LTV_GetConfiguredSamplingFrequency, MCP_Config_t,
    METADATA_PREFERRED_AUDIO_CONTEXTS, METADATA_PREFERRED_AUDIO_CONTEXTS_LENGTH, MICP_Config_t,
    SAMPLE_FREQ_16000_HZ, SAMPLE_FREQ_24000_HZ, SAMPLE_FREQ_48000_HZ, SUPPORTED_FRAME_DURATION_7_5_MS,
    SUPPORTED_FRAME_DURATION_10_MS, SUPPORTED_FRAME_DURATION_PREFERRED_10_MS,
    SUPPORTED_SAMPLE_FREQ_16000_HZ, SUPPORTED_SAMPLE_FREQ_24000_HZ, SUPPORTED_SAMPLE_FREQ_48000_HZ,
    VCP_Config_t, VCP_NotCode_t_VCP_RENDERER_UPDATED_VOLUME_STATE_EVT, VCP_Notification_Evt_t,
    VCP_ROLE_RENDERER, VCP_VolumeState_Evt_t,
};
#[cfg(not(feature = "ble-audio-codec-mngr"))]
use embassy_stm32_wpan::bindings::ble_audio::{
    CODEC_DataPathID_t_DATA_PATH_HCI_LEGACY, LC3_Frequency_FREQ_16KHz, LC3_Frequency_FREQ_24KHz,
    LC3_Frequency_FREQ_48KHz, LC3_FrameDuration_FRAME_7MS5, LC3_FrameDuration_FRAME_10MS,
    LC3_SampleDepth_DEPTH_16, lc3_decoder_channel_init, lc3_decoder_process, lc3_session_init,
};
#[cfg(feature = "ble-audio-codec-mngr")]
use embassy_stm32_wpan::codec_mngr::{
    self, AudioCorrectorConfig, AudioPllConfig, CodecMngrConfig, CodecPool, DataPathDirection,
    DataPathMode, LC3_DECODER_48KHZ_WORDS, LC3_MAX_STACK_48KHZ_WORDS, LC3_SESSION_48KHZ_WORDS,
};
#[cfg(feature = "ble-audio-codec-mngr")]
use embassy_stm32_wpan::{AudioSyncClockSource, AudioSyncInterruptHandler, CodecProcessInterruptHandler};
use embassy_stm32_wpan::bluetooth::gap::GapEvent;
use embassy_stm32_wpan::bluetooth::security::{IoCapability, SecurityParams};
use embassy_stm32_wpan::bluetooth::{EventBuffer, HCI};
use embassy_stm32_wpan::{HighInterruptHandler, LowInterruptHandler, Platform, new_platform};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
#[cfg(not(feature = "ble-audio-codec-mngr"))]
use embassy_sync::channel::Channel;
#[cfg(feature = "ble-audio-codec-mngr")]
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Timer};
use panic_probe as _;
use static_cell::StaticCell;

#[cfg(not(feature = "ble-audio-codec-mngr"))]
bind_interrupts!(struct Irqs {
    RADIO => HighInterruptHandler;
    HASH => LowInterruptHandler;
    GPDMA1_CHANNEL2 => dma::InterruptHandler<peripherals::GPDMA1_CH2>;
});

#[cfg(feature = "ble-audio-codec-mngr")]
bind_interrupts!(struct Irqs {
    RADIO => HighInterruptHandler;
    HASH => LowInterruptHandler;
    COMP => CodecProcessInterruptHandler;
    RCC_AUDIOSYNC => AudioSyncInterruptHandler;
    GPDMA1_CHANNEL2 => dma::InterruptHandler<peripherals::GPDMA1_CH2>;
});

#[embassy_executor::task]
async fn ble_runner_task(platform: &'static Platform) {
    platform.run_ble().await
}

// ── Shared Stream & Volume State ─────────────────────────────────────────────

const MAX_LC3_SDU_BYTES: usize = 160;
const SAI_OUTPUT_HZ: u32 = 48_000;
const BAP_MANDATORY_PRES_DELAY_US: u32 = 40_000;
const APP_DELAY_SNK_MIN_US: u32 = 4_000;
const APP_DELAY_SNK_MAX_US: u32 = 40_000;

#[cfg(not(feature = "ble-audio-codec-mngr"))]
#[derive(Clone, Copy)]
struct IsoSduFrame {
    len: u16,
    valid: bool,
    payload: [u8; MAX_LC3_SDU_BYTES],
}

#[cfg(not(feature = "ble-audio-codec-mngr"))]
impl IsoSduFrame {
    const fn empty() -> Self {
        Self {
            len: 0,
            valid: false,
            payload: [0u8; MAX_LC3_SDU_BYTES],
        }
    }
}

#[cfg(not(feature = "ble-audio-codec-mngr"))]
static ISO_RX_QUEUE: Channel<CriticalSectionRawMutex, IsoSduFrame, 8> = Channel::new();

#[cfg(feature = "ble-audio-codec-mngr")]
static CODEC_SYNC_TRIGGER: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[cfg(feature = "ble-audio-codec-mngr")]
static CODEC_POOL: StaticCell<
    CodecPool<6144, LC3_SESSION_48KHZ_WORDS, LC3_DECODER_48KHZ_WORDS, LC3_MAX_STACK_48KHZ_WORDS>,
> = StaticCell::new();

static STREAMING_ACTIVE: AtomicBool = AtomicBool::new(false);
static STREAM_FREQ_HZ: AtomicU16 = AtomicU16::new(48_000);
static STREAM_FRAME_US: AtomicU16 = AtomicU16::new(10_000);
#[cfg(not(feature = "ble-audio-codec-mngr"))]
static STREAM_OCTETS: AtomicU16 = AtomicU16::new(100);
static STREAM_CIS_HANDLE: AtomicU16 = AtomicU16::new(0xFFFF);
#[cfg(not(feature = "ble-audio-codec-mngr"))]
static CODEC_DIRTY: AtomicBool = AtomicBool::new(false);

static VOLUME_SETTING: AtomicU8 = AtomicU8::new(128);
static VOLUME_MUTED: AtomicBool = AtomicBool::new(false);

// ── aligned BLE Audio Static Pools ───────────────────────────────────────────

#[repr(align(4))]
struct AlignedPool<const N: usize>([u32; N]);

#[repr(align(8))]
struct AlignedNvmPool<const N: usize>([u64; N]);

static mut AUDIO_STACK_MEM: AlignedPool<1024> = AlignedPool([0; 1024]); // 4 KB
static mut AUDIO_NVM_MEM: AlignedNvmPool<256> = AlignedNvmPool([0; 256]); // 2 KB
static mut CAP_MEM: AlignedPool<512> = AlignedPool([0; 512]);
static mut PACS_MEM: AlignedPool<512> = AlignedPool([0; 512]);
static mut ASCS_MEM: AlignedPool<512> = AlignedPool([0; 512]);
static mut ISO_MEM: AlignedPool<512> = AlignedPool([0; 512]);
static mut BAP_NVM_MEM: AlignedPool<512> = AlignedPool([0; 512]);
static mut VCP_MEM: AlignedPool<512> = AlignedPool([0; 512]);

// PACS PAC record buffers must stay alive for the lifetime of the service.
static mut SNK_CAP_LTV: [u8; 16] = [0; 16];
static mut SNK_META_LTV: [u8; 4] = [0; 4];
static mut SNK_PAC_RECORD: BAP_AudioPACRecord_t = BAP_AudioPACRecord_t {
    Next: core::ptr::null_mut(),
    Prev: core::ptr::null_mut(),
    Cap: embassy_stm32_wpan::bindings::ble_audio::BAP_AudioCapabilities_t {
        CodecID: embassy_stm32_wpan::bindings::ble_audio::BAP_AudioCodecID_t {
            CodingFormat: 0,
            CompanyID: 0,
            VsCodecID: 0,
        },
        SpecificCapLength: 0,
        pSpecificCap: core::ptr::null_mut(),
        MetadataLength: 0,
        pMetadata: core::ptr::null_mut(),
    },
};

// ── Audio Data Path Transport Hooks ──────────────────────────────────────────

#[cfg(not(feature = "ble-audio-codec-mngr"))]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn HCI_RX_ISO_DATA_EVENT(
    conn_handle: u16,
    flags: u8,
    data_length: u16,
    data: *const u8,
) -> u8 {
    if !STREAMING_ACTIVE.load(Ordering::Acquire) || conn_handle != STREAM_CIS_HANDLE.load(Ordering::Acquire) {
        return BLE_STATUS_SUCCESS as u8;
    }

    if data.is_null() || data_length < 4 {
        return BLE_STATUS_SUCCESS as u8;
    }

    // `flags` carries PB_Flag (bits 0..1) and TS_Flag (bit 2) from `ll_cb_RxIsoData`.
    let pb_flag = flags & 0x03;
    let ts_flag = (flags & 0x04) != 0;
    let bytes = unsafe { core::slice::from_raw_parts(data, data_length as usize) };
    let hdr_offset = if ts_flag { 4usize } else { 0usize };

    if bytes.len() < hdr_offset + 4 {
        return BLE_STATUS_SUCCESS as u8;
    }

    // HCI ISO Data Load header: Packet_Sequence_Number (2B) + SDU_Len/Status (2B).
    let sdu_hdr = u16::from_le_bytes([bytes[hdr_offset + 2], bytes[hdr_offset + 3]]);
    let sdu_len = (sdu_hdr & 0x0FFF) as usize;
    let pkt_status = (sdu_hdr >> 14) & 0x03;
    let payload_slice = &bytes[hdr_offset + 4..];

    let mut frame = IsoSduFrame::empty();
    if pb_flag == 0b10 && pkt_status == 0 && sdu_len > 0 && sdu_len <= MAX_LC3_SDU_BYTES && payload_slice.len() >= sdu_len
    {
        frame.len = sdu_len as u16;
        frame.valid = true;
        frame.payload[..sdu_len].copy_from_slice(&payload_slice[..sdu_len]);
    } else {
        // Trigger LC3 Packet Loss Concealment (PLC) for lost/corrupt SDUs.
        frame.valid = false;
    }

    let _ = ISO_RX_QUEUE.try_send(frame);
    BLE_STATUS_SUCCESS as u8
}

#[cfg(feature = "ble-audio-codec-mngr")]
unsafe extern "C" fn on_codec_sync_trigger() -> i32 {
    CODEC_SYNC_TRIGGER.signal(());
    0
}

// ── BLE Audio Profile Callback (CAP / BAP / VCP Events) ─────────────────────

#[allow(non_upper_case_globals)]
#[unsafe(no_mangle)]
pub unsafe extern "C" fn CAP_Notification(notification: *mut CAP_Notification_Evt_t) {
    if notification.is_null() {
        return;
    }

    let evt = unsafe { &*notification };
    match evt.EvtOpcode {
        CAP_NotCode_t_CAP_LINKUP_COMPLETE_EVT => {
            info!(
                "CAP linkup complete: conn=0x{:04X}, status=0x{:02X}",
                evt.ConnHandle, evt.Status
            );
        }
        CAP_NotCode_t_CAP_UNICAST_SERVER_ASE_STATE_EVT => {
            info!("ASE state update: conn=0x{:04X}", evt.ConnHandle);
        }
        CAP_NotCode_t_CAP_UNICAST_PREFERRED_QOS_REQ_EVT => {
            if !evt.pInfo.is_null() {
                unsafe { handle_preferred_qos_req(&mut *(evt.pInfo as *mut BAP_PrefQoSConfReq_t)) };
            }
        }
        CAP_NotCode_t_CAP_UNICAST_SERVER_ENABLE_REQ_EVT => {
            if !evt.pInfo.is_null() {
                let req = unsafe { &*(evt.pInfo as *const BAP_ASE_Enable_Req_Params_t) };
                let ret = unsafe {
                    CAP_Unicast_EnableOpRsp(
                        evt.ConnHandle,
                        req.ASE_ID,
                        ASE_OP_RESP_SUCCESS as u8,
                        ASE_OP_RESP_NO_REASON as u8,
                    )
                };
                info!("ASE enable req id={} -> rsp status=0x{:02X}", req.ASE_ID, ret);
            }
        }
        CAP_NotCode_t_CAP_UNICAST_AUDIO_DATA_PATH_SETUP_REQ_EVT => {
            if !evt.pInfo.is_null() {
                unsafe { handle_data_path_setup(&*(evt.pInfo as *const BAP_SetupAudioDataPathReq_t)) };
            }
        }
        CAP_NotCode_t_CAP_AUDIO_CLOCK_REQ_EVT => {
            // Audio clock stays synced to PLL1Q = 49.152 MHz for 48 kHz SAI output.
        }
        CAP_NotCode_t_CAP_UNICAST_AUDIO_CONNECTION_UP_EVT => {
            if !evt.pInfo.is_null() {
                let info_evt = unsafe { &*(evt.pInfo as *const BAP_Unicast_Audio_Path_t) };
                if info_evt.AudioPathDirection == BAP_AUDIO_PATH_OUTPUT as u8 && evt.Status == 0 {
                    info!(
                        "Unicast Audio UP: cis=0x{:04X}, delay={} us",
                        info_evt.CIS_ConnHandle, info_evt.ControllerDelay
                    );
                    STREAM_CIS_HANDLE.store(info_evt.CIS_ConnHandle, Ordering::Release);
                    STREAMING_ACTIVE.store(true, Ordering::Release);
                }
            }
        }
        CAP_NotCode_t_CAP_UNICAST_AUDIO_CONNECTION_DOWN_EVT => {
            if !evt.pInfo.is_null() {
                let info_evt = unsafe { &*(evt.pInfo as *const BAP_Unicast_Audio_Path_t) };
                if info_evt.AudioPathDirection == BAP_AUDIO_PATH_OUTPUT as u8 {
                    info!("Unicast Audio DOWN: cis=0x{:04X}", info_evt.CIS_ConnHandle);
                    STREAMING_ACTIVE.store(false, Ordering::Release);
                    STREAM_CIS_HANDLE.store(0xFFFF, Ordering::Release);
                }
            }
        }
        CAP_NotCode_t_CAP_VCP_META_EVT => {
            if !evt.pInfo.is_null() {
                let vcp_evt = unsafe { &*(evt.pInfo as *const VCP_Notification_Evt_t) };
                if vcp_evt.EvtOpcode == VCP_NotCode_t_VCP_RENDERER_UPDATED_VOLUME_STATE_EVT
                    && !vcp_evt.pInfo.is_null()
                {
                    let vol = unsafe { &*(vcp_evt.pInfo as *const VCP_VolumeState_Evt_t) };
                    VOLUME_SETTING.store(vol.VolSetting, Ordering::Release);
                    VOLUME_MUTED.store(vol.Mute != 0, Ordering::Release);
                    info!("VCP volume={} mute={}", vol.VolSetting, vol.Mute);
                }
            }
        }
        other => {
            debug!("CAP event 0x{:02X} status=0x{:02X}", other, evt.Status);
        }
    }
}

unsafe fn handle_preferred_qos_req(req: &mut BAP_PrefQoSConfReq_t) {
    let freq = unsafe {
        LTV_GetConfiguredSamplingFrequency(req.CodecConf.pSpecificConf, req.CodecConf.SpecificConfLength)
    } as u32;
    let frame_dur =
        unsafe { LTV_GetConfiguredFrameDuration(req.CodecConf.pSpecificConf, req.CodecConf.SpecificConfLength) } as u32;
    let octets = unsafe {
        LTV_GetConfiguredOctetsPerCodecFrame(req.CodecConf.pSpecificConf, req.CodecConf.SpecificConfLength)
    };
    let mut blocks =
        unsafe { LTV_GetConfiguredFramesBlockPerSdu(req.CodecConf.pSpecificConf, req.CodecConf.SpecificConfLength) };
    if blocks == 0 {
        blocks = 1;
    }

    let freq_supported = matches!(freq, SAMPLE_FREQ_16000_HZ | SAMPLE_FREQ_24000_HZ | SAMPLE_FREQ_48000_HZ);
    let dur_supported = matches!(frame_dur, FRAME_DURATION_7_5_MS | FRAME_DURATION_10_MS);

    if !freq_supported || !dur_supported || octets < 26 || octets as usize > MAX_LC3_SDU_BYTES {
        unsafe {
            *req.pResp = ASE_OP_RESP_UNSUPPORTED_CONF_PARAM as u8;
            *req.pReason = ASE_OP_RESP_REASON_CODEC_SPEC_CONF as u8;
        }
        return;
    }

    let min_delay = APP_DELAY_SNK_MIN_US + req.SnkControllerDelayMin;
    let max_delay = APP_DELAY_SNK_MAX_US + req.SnkControllerDelayMax;
    if min_delay > BAP_MANDATORY_PRES_DELAY_US || max_delay < BAP_MANDATORY_PRES_DELAY_US {
        unsafe {
            *req.pResp = ASE_OP_RESP_UNSUPPORTED_CONF_PARAM as u8;
            *req.pReason = ASE_OP_RESP_REASON_PRES_DELAY as u8;
        }
        return;
    }

    let rsp = unsafe { &mut *req.pPrefQoSConfRsp };
    rsp.Framing = BAP_FRAMING_UNFRAMED as u8;
    rsp.PrefPHY = LE_2M_PHY_PREFERRED as u8;
    rsp.PrefRetransmissionNumber = 2;
    rsp.MaxTransportLatency = 20;
    rsp.PresentationDelayMin = min_delay;
    rsp.PresentationDelayMax = max_delay;
    rsp.PrefPresentationDelayMin = min_delay;
    rsp.PrefPresentationDelayMax = max_delay;

    unsafe {
        *req.pResp = ASE_OP_RESP_SUCCESS as u8;
        *req.pReason = ASE_OP_RESP_NO_REASON as u8;
    }

    info!(
        "QoS req ASE {}: freq_code={} dur_code={} octets={} blocks={}",
        req.ASE_ID, freq, frame_dur, octets, blocks
    );
}

unsafe fn handle_data_path_setup(req: &BAP_SetupAudioDataPathReq_t) {
    let freq_code = unsafe {
        LTV_GetConfiguredSamplingFrequency(req.CodecConf.pSpecificConf, req.CodecConf.SpecificConfLength)
    } as u32;
    let dur_code =
        unsafe { LTV_GetConfiguredFrameDuration(req.CodecConf.pSpecificConf, req.CodecConf.SpecificConfLength) } as u32;
    let octets = unsafe {
        LTV_GetConfiguredOctetsPerCodecFrame(req.CodecConf.pSpecificConf, req.CodecConf.SpecificConfLength)
    };

    let freq_hz = match freq_code {
        SAMPLE_FREQ_16000_HZ => 16_000u16,
        SAMPLE_FREQ_24000_HZ => 24_000u16,
        _ => 48_000u16,
    };
    let frame_us = if dur_code == FRAME_DURATION_7_5_MS {
        7_500u16
    } else {
        10_000u16
    };

    STREAM_FREQ_HZ.store(freq_hz, Ordering::Release);
    STREAM_FRAME_US.store(frame_us, Ordering::Release);
    #[cfg(not(feature = "ble-audio-codec-mngr"))]
    {
        STREAM_OCTETS.store(octets.max(26), Ordering::Release);
        CODEC_DIRTY.store(true, Ordering::Release);
    }

    let ctrl_delay = req
        .PresentationDelay
        .saturating_sub(APP_DELAY_SNK_MIN_US)
        .clamp(req.ControllerDelayMin, req.ControllerDelayMax);

    #[cfg(feature = "ble-audio-codec-mngr")]
    let data_path_id = DataPathMode::SampleCircularBuffer as u8;
    #[cfg(not(feature = "ble-audio-codec-mngr"))]
    let data_path_id = CODEC_DataPathID_t_DATA_PATH_HCI_LEGACY as u8;

    let ret = unsafe {
        CAP_Unicast_SetupAudioDataPath(
            req.CIS_ConnHandle,
            req.ASE_ID,
            data_path_id,
            ctrl_delay,
            0,
            core::ptr::null(),
        )
    };
    info!(
        "Setup ISO data path (id={}) cis=0x{:04X} ase={} {}Hz/{}us ({}B) -> 0x{:02X}",
        data_path_id, req.CIS_ConnHandle, req.ASE_ID, freq_hz, frame_us, octets, ret
    );
}

// ── LC3 Decoder + SAI Playback Task ──────────────────────────────────────────

#[cfg(not(feature = "ble-audio-codec-mngr"))]
#[repr(align(4))]
struct Lc3Workspace {
    session: [u32; 48],   // LC3_SESSION_STRUCT_SIZE_48kHz = 172 B
    decoder: [u32; 2100], // LC3_DECODER_STRUCT_SIZE_48kHz = 8340 B
    stack: [u32; 1024],   // LC3_DECODER_STACK_SIZE_48kHz = 3908 B
}

#[cfg(not(feature = "ble-audio-codec-mngr"))]
static mut LC3_WORK: Lc3Workspace = Lc3Workspace {
    session: [0; 48],
    decoder: [0; 2100],
    stack: [0; 1024],
};

#[cfg(not(feature = "ble-audio-codec-mngr"))]
fn configure_lc3_decoder(freq_hz: u16, frame_us: u16, octets: u16) -> usize {
    let lc3_freq = match freq_hz {
        16_000 => LC3_Frequency_FREQ_16KHz,
        24_000 => LC3_Frequency_FREQ_24KHz,
        _ => LC3_Frequency_FREQ_48KHz,
    };
    let lc3_dur = if frame_us == 7_500 {
        LC3_FrameDuration_FRAME_7MS5
    } else {
        LC3_FrameDuration_FRAME_10MS
    };

    // Match ST's Nbytes = bitrate * Nms / 8 formula without truncation loss at 7.5 ms.
    let bitrate = if frame_us == 7_500 {
        ((octets as u32) * 3200 + 2) / 3
    } else {
        (octets as u32) * 800
    };

    unsafe {
        let sess = core::ptr::addr_of_mut!(LC3_WORK.session) as *mut c_void;
        let dec = core::ptr::addr_of_mut!(LC3_WORK.decoder) as *mut c_void;
        let st_sess = lc3_session_init(sess, lc3_freq, lc3_dur);
        let st_ch = lc3_decoder_channel_init(sess, dec, bitrate, LC3_SampleDepth_DEPTH_16);
        info!(
            "LC3 decoder init: {}Hz {}us {}bps -> sess={} ch={}",
            freq_hz, frame_us, bitrate, st_sess, st_ch
        );
    }

    (freq_hz as usize * frame_us as usize) / 1_000_000
}

fn scale_and_upsample_to_48k(src: &[i16], src_rate_hz: u16, out: &mut [u16]) -> usize {
    let muted = VOLUME_MUTED.load(Ordering::Acquire);
    let vol = if muted { 0i32 } else { VOLUME_SETTING.load(Ordering::Acquire) as i32 };
    let repeat = (SAI_OUTPUT_HZ / src_rate_hz as u32).max(1) as usize;
    let mut written = 0usize;

    for &s in src {
        // Apply linear VCP volume scaling (255 = full scale).
        let scaled = ((s as i32 * vol) / 255) as i16 as u16;
        for _ in 0..repeat {
            if written < out.len() {
                out[written] = scaled;
                written += 1;
            }
        }
    }
    written
}

#[cfg(not(feature = "ble-audio-codec-mngr"))]
#[embassy_executor::task]
async fn audio_playback_task(mut sai_tx: Sai<'static, u16>) {
    let mut pcm_in = [0i16; 480];
    let mut pcm_out = [0u16; 480];
    let mut started = false;
    let mut src_samples = 480usize;

    loop {
        if !STREAMING_ACTIVE.load(Ordering::Acquire) {
            if started {
                sai_tx.stop();
                sai_tx.clear();
                started = false;
            }
            while ISO_RX_QUEUE.try_receive().is_ok() {}
            Timer::after_millis(10).await;
            continue;
        }

        if CODEC_DIRTY.swap(false, Ordering::AcqRel) {
            src_samples = configure_lc3_decoder(
                STREAM_FREQ_HZ.load(Ordering::Acquire),
                STREAM_FRAME_US.load(Ordering::Acquire),
                STREAM_OCTETS.load(Ordering::Acquire),
            );
        }

        let mut frame = match embassy_time::with_timeout(Duration::from_millis(15), ISO_RX_QUEUE.receive()).await {
            Ok(f) => f,
            Err(_) => IsoSduFrame::empty(), // Run PLC on missed SDU deadline
        };

        let bfi = if frame.valid { 0 } else { 1 };
        let frame_ns = src_samples.min(pcm_in.len());

        unsafe {
            let dec = core::ptr::addr_of_mut!(LC3_WORK.decoder) as *mut c_void;
            let stk = core::ptr::addr_of_mut!(LC3_WORK.stack) as *mut c_void;
            let _ = lc3_decoder_process(
                dec,
                stk,
                frame.payload.as_mut_ptr(),
                pcm_in.as_mut_ptr() as *mut c_void,
                1,
                bfi,
            );
        }

        let freq_hz = STREAM_FREQ_HZ.load(Ordering::Acquire);
        let out_len = scale_and_upsample_to_48k(&pcm_in[..frame_ns], freq_hz, &mut pcm_out);

        if !started {
            let _ = sai_tx.write(&pcm_out[..out_len]).await;
            sai_tx.start();
            started = true;
        } else if let Err(e) = sai_tx.write(&pcm_out[..out_len]).await {
            warn!("SAI write error: {:?}", e);
        }
    }
}

#[cfg(feature = "ble-audio-codec-mngr")]
#[embassy_executor::task]
async fn audio_playback_task(mut sai_tx: Sai<'static, u16>) {
    let mut pcm_in = [0i16; 480];
    let mut pcm_out = [0u16; 480];
    let mut started = false;

    loop {
        if !STREAMING_ACTIVE.load(Ordering::Acquire) {
            if started {
                sai_tx.stop();
                sai_tx.clear();
                started = false;
            }
            CODEC_SYNC_TRIGGER.reset();
            Timer::after_millis(10).await;
            continue;
        }

        if !started {
            if embassy_time::with_timeout(Duration::from_millis(50), CODEC_SYNC_TRIGGER.wait())
                .await
                .is_err()
            {
                continue;
            }
            if !STREAMING_ACTIVE.load(Ordering::Acquire) {
                continue;
            }
        }

        let cis = STREAM_CIS_HANDLE.load(Ordering::Acquire);
        let freq_hz = STREAM_FREQ_HZ.load(Ordering::Acquire);
        let frame_us = STREAM_FRAME_US.load(Ordering::Acquire);
        let frame_ns = ((freq_hz as usize * frame_us as usize) / 1_000_000).min(pcm_in.len());

        codec_mngr::receive_data(cis, DataPathMode::SampleCircularBuffer, &mut pcm_in[..frame_ns]);
        let out_len = scale_and_upsample_to_48k(&pcm_in[..frame_ns], freq_hz, &mut pcm_out);

        if !started {
            let _ = sai_tx.write(&pcm_out[..out_len]).await;
            sai_tx.start();
            started = true;
        } else if let Err(e) = sai_tx.write(&pcm_out[..out_len]).await {
            warn!("SAI write error: {:?}", e);
        }
    }
}

// ── ST BLE Audio Profile Init & Extended Advertising ─────────────────────────

fn init_le_audio_profiles() {
    unsafe {
        let mut init = BleAudioInit_t {
            bleAudioStartRamAddress: core::ptr::addr_of_mut!(AUDIO_STACK_MEM) as *mut u8,
            total_buffer_size: core::mem::size_of::<AlignedPool<1024>>() as u32,
            NumOfLinks: 2,
            NumOfEATTBearersPerLink: 0,
            MaxNumOfBondedDevices: 1,
            bleAudioStartRamAddress_NVM: core::ptr::addr_of_mut!(AUDIO_NVM_MEM) as *mut u64,
            total_buffer_size_NVM: core::mem::size_of::<AlignedNvmPool<256>>() as u32,
        };
        let st = BLE_AUDIO_STACK_Init(&mut init);
        info!("BLE_AUDIO_STACK_Init -> 0x{:02X}", st);

        let cap_cfg = CAP_Config_t {
            Role: CAP_ROLE_ACCEPTOR as u8,
            MaxNumLinks: 2,
            pStartRamAddr: core::ptr::addr_of_mut!(CAP_MEM) as *mut u8,
            RamSize: core::mem::size_of::<AlignedPool<512>>() as u16,
        };

        let mut bap_cfg = BAP_Config_t::default();
        bap_cfg.Role = BAP_ROLE_UNICAST_SERVER as u8;
        bap_cfg.MaxNumBleLinks = 2;
        bap_cfg.MaxNumUSRLinks = 1;
        bap_cfg.PACSSrvConfig.MaxNumSnkPACRecords = 1;
        bap_cfg.PACSSrvConfig.MaxNumSrcPACRecords = 0;
        bap_cfg.PACSSrvConfig.pStartRamAddr = core::ptr::addr_of_mut!(PACS_MEM) as *mut u8;
        bap_cfg.PACSSrvConfig.RamSize = core::mem::size_of::<AlignedPool<512>>() as u16;
        bap_cfg.ASCSSrvConfig.AudioRole = AUDIO_ROLE_SINK as u8;
        bap_cfg.ASCSSrvConfig.MaxNumSnkASEs = 1;
        bap_cfg.ASCSSrvConfig.MaxNumSrcASEs = 0;
        bap_cfg.ASCSSrvConfig.MaxCodecConfSize = 19;
        bap_cfg.ASCSSrvConfig.MaxMetadataLength = 10;
        bap_cfg.ASCSSrvConfig.CachingEn = 0;
        bap_cfg.ASCSSrvConfig.pStartRamAddr = core::ptr::addr_of_mut!(ASCS_MEM) as *mut u8;
        bap_cfg.ASCSSrvConfig.RamSize = core::mem::size_of::<AlignedPool<512>>() as u16;
        bap_cfg.ISOChnlConfig.MaxNumCIG = 1;
        bap_cfg.ISOChnlConfig.MaxNumCISPerCIG = 1;
        bap_cfg.ISOChnlConfig.pStartRamAddr = core::ptr::addr_of_mut!(ISO_MEM) as *mut u8;
        bap_cfg.ISOChnlConfig.RamSize = core::mem::size_of::<AlignedPool<512>>() as u16;
        bap_cfg.NvmMgmtConfig.pStartRamAddr = core::ptr::addr_of_mut!(BAP_NVM_MEM) as *mut u8;
        bap_cfg.NvmMgmtConfig.RamSize = core::mem::size_of::<AlignedPool<512>>() as u16;

        let mut vcp_cfg = VCP_Config_t::default();
        vcp_cfg.Role = VCP_ROLE_RENDERER as u8;
        vcp_cfg.MaxNumBleLinks = 2;
        vcp_cfg.Renderer.InitialVolumeSetting = 128;
        vcp_cfg.Renderer.InitialMuteState = 0;
        vcp_cfg.Renderer.VolumeStepSize = 16;
        vcp_cfg.Renderer.pStartRamAddr = core::ptr::addr_of_mut!(VCP_MEM) as *mut u8;
        vcp_cfg.Renderer.RamSize = core::mem::size_of::<AlignedPool<512>>() as u16;

        let micp_cfg = MICP_Config_t::default();
        let ccp_cfg = CCP_Config_t::default();
        let mcp_cfg = MCP_Config_t::default();
        let csip_cfg = CSIP_Config_t::default();

        let st = CAP_Init(
            &cap_cfg,
            &bap_cfg,
            &vcp_cfg,
            &micp_cfg,
            &ccp_cfg,
            &mcp_cfg,
            &csip_cfg,
        );
        info!("CAP_Init -> 0x{:02X}", st);

        // Register Mono LC3 Sink PAC Record (16/24/48 kHz, 7.5/10 ms, 30..=155 bytes/frame).
        let freq_mask = (SUPPORTED_SAMPLE_FREQ_16000_HZ
            | SUPPORTED_SAMPLE_FREQ_24000_HZ
            | SUPPORTED_SAMPLE_FREQ_48000_HZ) as u16;
        let dur_mask = (SUPPORTED_FRAME_DURATION_7_5_MS
            | SUPPORTED_FRAME_DURATION_10_MS
            | SUPPORTED_FRAME_DURATION_PREFERRED_10_MS) as u8;

        SNK_CAP_LTV = [
            0x03,
            CODEC_CAP_SAMPLING_FREQ as u8,
            freq_mask as u8,
            (freq_mask >> 8) as u8,
            0x02,
            CODEC_CAP_FRAME_DURATION as u8,
            dur_mask,
            0x05,
            CODEC_CAP_OCTETS_PER_CODEC_FRAME as u8,
            30,
            0,
            155,
            0,
            0x02,
            CODEC_CAP_AUDIO_CHNL_COUNTS as u8,
            0x01, // 1 channel supported
        ];
        let ctx_mask = (AUDIO_CONTEXT_UNSPECIFIED | AUDIO_CONTEXT_CONVERSATIONAL | AUDIO_CONTEXT_MEDIA) as u16;
        SNK_META_LTV = [
            METADATA_PREFERRED_AUDIO_CONTEXTS_LENGTH as u8,
            METADATA_PREFERRED_AUDIO_CONTEXTS as u8,
            ctx_mask as u8,
            (ctx_mask >> 8) as u8,
        ];

        SNK_PAC_RECORD.Cap.CodecID.CodingFormat = AUDIO_CODING_FORMAT_LC3 as u8;
        SNK_PAC_RECORD.Cap.SpecificCapLength = 16;
        SNK_PAC_RECORD.Cap.pSpecificCap = core::ptr::addr_of_mut!(SNK_CAP_LTV) as *mut u8;
        SNK_PAC_RECORD.Cap.MetadataLength = 4;
        SNK_PAC_RECORD.Cap.pMetadata = core::ptr::addr_of_mut!(SNK_META_LTV) as *mut u8;

        let mut pac_handle = 0u16;
        let st = CAP_RegisterSnkPACRecord(core::ptr::addr_of_mut!(SNK_PAC_RECORD), &mut pac_handle);
        info!("CAP_RegisterSnkPACRecord (handle={}) -> 0x{:02X}", pac_handle, st);

        let _ = CAP_SetSnkAudioLocations(0x0000_0000); // Mono
        let _ = CAP_SetSupportedAudioContexts(ctx_mask, 0);
        let _ = CAP_SetAvailableAudioContexts(ctx_mask, 0);
    }
}

fn start_le_audio_ext_advertising() {
    let name = b"WBA6_Speaker";
    let mut adv = [0u8; 64];
    // Flags AD type
    adv[0] = 2;
    adv[1] = 0x01;
    adv[2] = 0x06; // LE General Discoverable | BR/EDR Not Supported
    // Complete Local Name AD type
    adv[3] = (1 + name.len()) as u8;
    adv[4] = 0x09;
    adv[5..5 + name.len()].copy_from_slice(name);
    let prefix_len = 5 + name.len();

    let cap_len = unsafe {
        CAP_BuildAdvPacket(
            0, // General Announcement
            core::ptr::null(),
            0,
            adv[prefix_len..].as_mut_ptr(),
            (adv.len() - prefix_len) as u8,
        )
    } as usize;
    let total_len = prefix_len + cap_len;

    let peer_addr = [0u8; 6];
    let adv_set = Adv_Set_t {
        Advertising_Handle: 0,
        Duration: 0,
        Max_Extended_Advertising_Events: 0,
    };

    unsafe {
        let st_cfg = aci_gap_adv_set_configuration(
            HCI_ADV_TYPE_ADV_IND as u8,
            0,
            HCI_ADV_EVENT_PROP_CONNECTABLE as u16,
            0x0080,
            0x00A0,
            0x07,
            0x00, // Public address
            0x00,
            peer_addr.as_ptr(),
            0x00, // No filter list
            0x7F, // Host has no TX power preference
            0,
            0x01, // Secondary PHY 1M
            0,
            0,
        );
        let st_data = hci_le_set_extended_advertising_data(0, 0x03, 0, total_len as u8, adv.as_ptr());
        let st_en = hci_le_set_extended_advertising_enable(1, 1, &adv_set);
        info!(
            "LE Audio Ext Adv started: cfg=0x{:02X} data=0x{:02X} en=0x{:02X} ({}B)",
            st_cfg, st_data, st_en, total_len
        );
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    config.rcc = rcc::Config::new_wpan();
    if let Some(ref mut pll) = config.rcc.pll1 {
        // VCO = 16 MHz * (12 + 2359/8192) = 196.607 MHz -> PLL1Q = 49.152 MHz (48 kHz SAI1)
        pll.frac = Some(2359);
        pll.divq = Some(rcc::PllDiv::Div4);
    }
    config.rcc.mux.sai1sel = rcc::mux::Sai1sel::Pll1Q;

    let p = embassy_stm32::init(config);

    info!("Embassy STM32WBA6 LE Audio Speaker Example");

    // ── SAI1 + MAX98357A Output Setup (48 kHz, 16-bit Mono I2S) ───────────────
    let (sai_a, _sai_b) = sai::split_subblocks(p.SAI1);
    static SAI_DMA_BUF: StaticCell<[u16; 2048]> = StaticCell::new();
    let sai_dma_buf = SAI_DMA_BUF.init([0u16; 2048]);

    let mut sai_cfg = sai::Config::default();
    sai_cfg.mode = sai::Mode::Master;
    sai_cfg.tx_rx = sai::TxRx::Transmitter;
    sai_cfg.stereo_mono = sai::StereoMono::Mono;
    sai_cfg.data_size = sai::DataSize::Data16;
    sai_cfg.bit_order = sai::BitOrder::MsbFirst;
    sai_cfg.slot_size = sai::SlotSize::Channel32;
    sai_cfg.slot_count = sai::word::U4(2);
    sai_cfg.slot_enable = 0b11;
    sai_cfg.first_bit_offset = sai::word::U5(0);
    sai_cfg.frame_sync_polarity = sai::FrameSyncPolarity::ActiveLow;
    sai_cfg.frame_sync_offset = sai::FrameSyncOffset::BeforeFirstBit;
    sai_cfg.frame_length = 32;
    sai_cfg.frame_sync_active_level_length = sai::word::U7(16);
    sai_cfg.fifo_threshold = sai::FifoThreshold::Quarter;
    sai_cfg.master_clock_divider = sai::MasterClockDivider::Div4;

    let sai_tx = Sai::new_asynchronous(sai_a, p.PA7, p.PB14, p.PA8, p.GPDMA1_CH2, Irqs, sai_dma_buf, sai_cfg);
    let _max98357a_sd = Output::new(p.PA1, Level::High, Speed::Low);
    spawner.spawn(audio_playback_task(sai_tx).expect("Failed to spawn audio task"));

    // ── BLE Stack & LE Audio Initialization ───────────────────────────────────
    let (platform, runtime) = new_platform!(12);
    spawner.spawn(ble_runner_task(platform).expect("Failed to spawn BLE runner"));

    let mut ble = HCI::new(platform, runtime, Irqs).await.expect("BLE init failed");

    #[cfg(feature = "ble-audio-codec-mngr")]
    {
        let codec_cfg = CodecMngrConfig {
            clock_source: AudioSyncClockSource::Pll1Q,
            timer_prescaler: 48, // PLL1Q = 49.152 MHz -> 1.024 MHz AudioSync counter
            pll_config: AudioPllConfig {
                target_freq_hz: 49_152_000,
                vco_input_freq_hz: 16_000_000.0,
                output_div: 4,
            },
            corrector_config: AudioCorrectorConfig::default(),
            ..CodecMngrConfig::default()
        };
        codec_mngr::init(Irqs, CODEC_POOL.init(CodecPool::new()), codec_cfg)
            .expect("CodecMngr init failed");
        codec_mngr::configure_data_path_sample(DataPathDirection::Output, 16, 1)
            .expect("Configure DataPath sample failed");
        codec_mngr::register_trigger_callback(
            DataPathMode::SampleCircularBuffer,
            DataPathDirection::Output,
            Some(on_codec_sync_trigger),
        )
        .expect("Register codec trigger failed");
        info!("Codec Manager + RCC AudioSync initialized (SampleCircularBuffer)");
    }

    // LE Audio requires encrypted bonding; configure JustWorks + Bonding so phones can pair.
    let sec_params = SecurityParams::new()
        .with_bonding(true)
        .with_mitm_protection(false)
        .with_io_capability(IoCapability::NoInputNoOutput);
    let mut security = ble.security_manager();
    let _ = security
        .set_authentication_requirements(ble.controller(), sec_params)
        .await;

    init_le_audio_profiles();
    start_le_audio_ext_advertising();

    let mut event_buf = EventBuffer::new();
    loop {
        let event = ble.read_event(&mut event_buf).await;
        if let Some(gap_event) = ble.process_event(&event).await {
            match gap_event {
                GapEvent::Connected(conn) => {
                    info!("Central connected: conn=0x{:04X}", conn.handle.raw());
                }
                GapEvent::Disconnected { handle, reason } => {
                    info!(
                        "Central disconnected: conn=0x{:04X}, reason=0x{:02X}",
                        handle.raw(),
                        reason.as_u8()
                    );
                    STREAMING_ACTIVE.store(false, Ordering::Release);
                    STREAM_CIS_HANDLE.store(0xFFFF, Ordering::Release);
                    start_le_audio_ext_advertising();
                }
                _ => {}
            }
        }
    }
}
