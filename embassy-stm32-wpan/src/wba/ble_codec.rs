//! Bridge between ST's BLE host/link-layer ISO callbacks (`BLECB_*`) and
//! `libcodec_mngr.a` (`CODEC_*` / `AUDIO_*`), with per-stream coexistence for
//! manual `DATA_PATH_HCI_LEGACY` streams.
//!
//! When the `ble-audio-codec-mngr` feature is enabled:
//! - Before `CodecMngr::init()` is called (`!is_active()`), all `BLECB_*` hooks
//!   stay dormant on the HCI path so existing `HCI_RX_ISO_DATA_EVENT` + manual
//!   `liblc3` applications behave identically.
//! - After `CodecMngr::init()` is called (`is_active()`), each CIS/BIS stream can
//!   independently select either `DATA_PATH_HCI_LEGACY` (`0x00`, handled via
//!   `HCI_RX_ISO_DATA_EVENT` / `BLE_SendIsoDataToLinkLayer`) or a codec-managed
//!   data path (`DATA_PATH_SAMPLE_CIRC_BUF` / `DATA_PATH_FRAME_CIRC_BUF`).

use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};

use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use stm32_bindings::bindings::wba_ble_audio::{
    AUDIO_CalibrationClbk, AUDIO_RegisterStream, AUDIO_SyncEventClbk, AUDIO_UnregisterStream, CODEC_ConfigureDataPath,
    CODEC_DataPathID_t_DATA_PATH_HCI_LEGACY, CODEC_ReadLocalSupportedCodecCapabilies,
    CODEC_ReadLocalSupportedCodecsV2, CODEC_ReadLocalSupportedControllerDelay, CODEC_ReceiveMediaPacket,
    CODEC_RemoveIsoDataPath, CODEC_SetupIsoDataPath, CODEC_SetupIsoDataPathCmd_t, DATA_PATH_HCI, DATA_PATH_INPUT,
    DATA_PATH_OUTPUT, MAX_PATH_NB, codec_rcv_status_t_CODEC_RCV_STATUS_BUSY,
    codec_rcv_status_t_CODEC_RCV_STATUS_FAIL, codec_rcv_status_t_CODEC_RCV_STATUS_OK, grp_type_t_GRP_TYPE_BIG,
    grp_type_t_GRP_TYPE_CIG, iso_sdu_fragment_t_FIRST_SDU_FRAG, iso_sdu_fragment_t_FULL_SDU_FRAG,
};
use stm32_bindings::bindings::wba_ble_stack::{
    BLE_STATUS_FAILED, BLE_STATUS_PENDING, BLE_STATUS_SUCCESS, BLE_SendIsoDataToLinkLayer,
    HCI_COMMAND_DISALLOWED_ERR_CODE, HCI_DISCONNECTION_COMPLETE_EVT_CODE, HCI_INVALID_HCI_COMMAND_PARAMETERS_ERR_CODE,
    HCI_LE_BIG_SYNC_ESTABLISHED_SUBEVT_CODE, HCI_LE_BIG_SYNC_LOST_SUBEVT_CODE, HCI_LE_CIS_ESTABLISHED_SUBEVT_CODE,
    HCI_LE_CIS_ESTABLISHED_V2_SUBEVT_CODE, HCI_LE_CREATE_BIG_COMPLETE_SUBEVT_CODE, HCI_LE_META_EVT_CODE,
    HCI_LE_TERMINATE_BIG_COMPLETE_SUBEVT_CODE, HCI_MEMORY_CAPACITY_EXCEEDED_ERR_CODE, HCI_SUCCESS_ERR_CODE,
    HCI_UNKNOWN_HCI_COMMAND_ERR_CODE,
};
use stm32_bindings::bindings::wba_link_layer_ble_full::{
    ble_intf_get_cig_info_st, ble_intf_setup_iso_data_path, iso_sdu_buf_hdr_p, ll_intf_free_iso_sdu,
    ll_intf_get_cig_info, ll_intf_rmv_iso_data_path, ll_intf_set_output_data_path, ll_intf_setup_iso_data_path,
};

const MAX_STREAMS: usize = MAX_PATH_NB as usize;
const MAX_QUEUED_RX_SDUS: usize = 16;

const DIR_MASK_INPUT: u8 = 0x01;
const DIR_MASK_OUTPUT: u8 = 0x02;

static CODEC_ACTIVE: AtomicBool = AtomicBool::new(false);

#[derive(Copy, Clone)]
struct StreamPath {
    conn_handle: u16,
    input_path_id: Option<u8>,
    output_path_id: Option<u8>,
}

impl StreamPath {
    const fn empty() -> Self {
        Self {
            conn_handle: 0,
            input_path_id: None,
            output_path_id: None,
        }
    }

    const fn is_empty(&self) -> bool {
        self.input_path_id.is_none() && self.output_path_id.is_none()
    }
}

#[derive(Copy, Clone)]
struct QueuedRxSdu {
    conn_handle: u16,
    ptr_sdu: iso_sdu_buf_hdr_p,
}

// SAFETY: `iso_sdu_buf_hdr_p` points into Link Layer static ISO SDU pool managed
// exclusively under LL / critical-section synchronization.
unsafe impl Send for QueuedRxSdu {}

struct CodecBridgeState {
    streams: [StreamPath; MAX_STREAMS],
    rx_queue: [Option<QueuedRxSdu>; MAX_QUEUED_RX_SDUS],
}

impl CodecBridgeState {
    const fn new() -> Self {
        Self {
            streams: [StreamPath::empty(); MAX_STREAMS],
            rx_queue: [None; MAX_QUEUED_RX_SDUS],
        }
    }

    fn find_stream(&self, conn_handle: u16) -> Option<&StreamPath> {
        self.streams
            .iter()
            .find(|s| !s.is_empty() && s.conn_handle == conn_handle)
    }

    fn set_stream_path(&mut self, conn_handle: u16, direction: u8, path_id: u8) -> bool {
        if let Some(slot) = self
            .streams
            .iter_mut()
            .find(|s| !s.is_empty() && s.conn_handle == conn_handle)
        {
            match direction as u32 {
                DATA_PATH_INPUT => slot.input_path_id = Some(path_id),
                DATA_PATH_OUTPUT => slot.output_path_id = Some(path_id),
                _ => return false,
            }
            return true;
        }

        if let Some(slot) = self.streams.iter_mut().find(|s| s.is_empty()) {
            slot.conn_handle = conn_handle;
            slot.input_path_id = (direction as u32 == DATA_PATH_INPUT).then_some(path_id);
            slot.output_path_id = (direction as u32 == DATA_PATH_OUTPUT).then_some(path_id);
            return true;
        }

        false
    }

    fn take_removed_codec_dirs(&mut self, conn_handle: u16, dir_mask: u8) -> u8 {
        let mut codec_dir_mask = 0u8;
        if let Some(slot) = self
            .streams
            .iter_mut()
            .find(|s| !s.is_empty() && s.conn_handle == conn_handle)
        {
            if (dir_mask & DIR_MASK_INPUT) != 0 {
                if let Some(id) = slot.input_path_id.take() {
                    if id != DATA_PATH_HCI as u8 {
                        codec_dir_mask |= DIR_MASK_INPUT;
                    }
                }
            }
            if (dir_mask & DIR_MASK_OUTPUT) != 0 {
                if let Some(id) = slot.output_path_id.take() {
                    if id != DATA_PATH_HCI as u8 {
                        codec_dir_mask |= DIR_MASK_OUTPUT;
                    }
                }
            }
        }
        codec_dir_mask
    }

    fn clear_stream(&mut self, conn_handle: u16) {
        for slot in self.streams.iter_mut() {
            if !slot.is_empty() && slot.conn_handle == conn_handle {
                *slot = StreamPath::empty();
            }
        }
    }

    fn push_rx_sdu(&mut self, conn_handle: u16, ptr_sdu: iso_sdu_buf_hdr_p) -> bool {
        if let Some(slot) = self.rx_queue.iter_mut().find(|e| e.is_none()) {
            *slot = Some(QueuedRxSdu { conn_handle, ptr_sdu });
            return true;
        }
        false
    }

    fn push_front_rx_sdu(&mut self, conn_handle: u16, ptr_sdu: iso_sdu_buf_hdr_p) -> bool {
        if self.rx_queue[MAX_QUEUED_RX_SDUS - 1].is_some() {
            return false;
        }
        for i in (1..MAX_QUEUED_RX_SDUS).rev() {
            self.rx_queue[i] = self.rx_queue[i - 1];
        }
        self.rx_queue[0] = Some(QueuedRxSdu { conn_handle, ptr_sdu });
        true
    }

    fn pop_next_rx_sdu(&mut self, conn_handle: u16) -> Option<iso_sdu_buf_hdr_p> {
        let pos = self
            .rx_queue
            .iter()
            .position(|e| matches!(e, Some(item) if item.conn_handle == conn_handle))?;
        let ptr_sdu = self.rx_queue[pos]?.ptr_sdu;
        // Shift later entries left to preserve strict FIFO delivery per handle.
        for i in pos..(MAX_QUEUED_RX_SDUS - 1) {
            self.rx_queue[i] = self.rx_queue[i + 1];
        }
        self.rx_queue[MAX_QUEUED_RX_SDUS - 1] = None;
        Some(ptr_sdu)
    }

    fn drain_handle_rx_sdus<F: FnMut(iso_sdu_buf_hdr_p)>(&mut self, conn_handle: u16, mut f: F) {
        while let Some(ptr_sdu) = self.pop_next_rx_sdu(conn_handle) {
            f(ptr_sdu);
        }
    }

    fn drain_all_rx_sdus<F: FnMut(iso_sdu_buf_hdr_p)>(&mut self, mut f: F) {
        for entry in self.rx_queue.iter_mut() {
            if let Some(item) = entry.take() {
                f(item.ptr_sdu);
            }
        }
    }
}

static BRIDGE_STATE: Mutex<CriticalSectionRawMutex, RefCell<CodecBridgeState>> =
    Mutex::new(RefCell::new(CodecBridgeState::new()));

pub(crate) fn is_active() -> bool {
    CODEC_ACTIVE.load(Ordering::Acquire)
}

pub(crate) fn set_active(active: bool) {
    CODEC_ACTIVE.store(active, Ordering::Release);
}

/// Clear cached per-stream routing and release any buffered ISO SDU packets.
pub(crate) fn reset() {
    set_active(false);
    BRIDGE_STATE.lock(|cell| {
        let mut st = cell.borrow_mut();
        st.drain_all_rx_sdus(|ptr_sdu| unsafe { ll_intf_free_iso_sdu(ptr_sdu) });
        st.streams = [StreamPath::empty(); MAX_STREAMS];
    });
}

/// Process relevant HCI events arriving from the Link Layer so `libcodec_mngr.a`
/// stays synchronized with CIS/BIG establishment and teardown.
pub(crate) fn on_hci_event(packet: &[u8]) {
    if !is_active() || packet.len() < 3 || packet[0] != 0x04 {
        return;
    }

    let evt_code = packet[1] as u32;
    let param_len = packet[2] as usize;
    if packet.len() < 3 + param_len {
        return;
    }
    let params = &packet[3..3 + param_len];

    if evt_code == HCI_DISCONNECTION_COMPLETE_EVT_CODE {
        if params.len() >= 3 && params[0] == HCI_SUCCESS_ERR_CODE as u8 {
            let conn_handle = u16::from_le_bytes([params[1], params[2]]);
            BRIDGE_STATE.lock(|cell| {
                let mut st = cell.borrow_mut();
                st.clear_stream(conn_handle);
                st.drain_handle_rx_sdus(conn_handle, |ptr_sdu| unsafe { ll_intf_free_iso_sdu(ptr_sdu) });
            });
            unsafe {
                AUDIO_UnregisterStream(grp_type_t_GRP_TYPE_CIG, 0, conn_handle);
            }
        }
        return;
    }

    if evt_code != HCI_LE_META_EVT_CODE || params.is_empty() {
        return;
    }

    let subevt = params[0] as u32;
    let sub = &params[1..];

    match subevt {
        HCI_LE_CIS_ESTABLISHED_SUBEVT_CODE | HCI_LE_CIS_ESTABLISHED_V2_SUBEVT_CODE => {
            if sub.len() >= 3 && sub[0] == HCI_SUCCESS_ERR_CODE as u8 {
                let mut cis_handle = u16::from_le_bytes([sub[1], sub[2]]);
                let mut cig_info = ble_intf_get_cig_info_st::default();
                let status = unsafe { ll_intf_get_cig_info(cis_handle, &mut cig_info) };
                if status == BLE_STATUS_SUCCESS {
                    unsafe {
                        AUDIO_RegisterStream(
                            grp_type_t_GRP_TYPE_CIG,
                            cig_info.cig_id,
                            1,
                            &mut cis_handle,
                            cig_info.iso_interval,
                            cig_info.role,
                            cig_info.trsnprt_ltncy_m_to_s,
                            cig_info.trsnprt_ltncy_s_to_m,
                        );
                    }
                }
            }
        }
        HCI_LE_CREATE_BIG_COMPLETE_SUBEVT_CODE => {
            // Layout: Status(1), BIG_Handle(1), BIG_Sync_Delay(3), Transport_Latency(3),
            // PHY(1), NSE(1), BN(1), PTO(1), IRC(1), Max_PDU(2), ISO_Interval(2), Num_BIS(1), Handles(...)
            if sub.len() >= 18 && sub[0] == HCI_SUCCESS_ERR_CODE as u8 {
                let big_handle = sub[1];
                let transport_latency = u32::from_le_bytes([sub[5], sub[6], sub[7], 0]);
                let iso_interval = u16::from_le_bytes([sub[15], sub[16]]);
                let num_bis = sub[17] as usize;
                register_big_handles(big_handle, &sub[18..], num_bis, iso_interval, transport_latency);
            }
        }
        HCI_LE_BIG_SYNC_ESTABLISHED_SUBEVT_CODE => {
            // Layout: Status(1), BIG_Handle(1), Transport_Latency(3),
            // NSE(1), BN(1), PTO(1), IRC(1), Max_PDU(2), ISO_Interval(2), Num_BIS(1), Handles(...)
            if sub.len() >= 14 && sub[0] == HCI_SUCCESS_ERR_CODE as u8 {
                let big_handle = sub[1];
                let transport_latency = u32::from_le_bytes([sub[2], sub[3], sub[4], 0]);
                let iso_interval = u16::from_le_bytes([sub[11], sub[12]]);
                let num_bis = sub[13] as usize;
                register_big_handles(big_handle, &sub[14..], num_bis, iso_interval, transport_latency);
            }
        }
        HCI_LE_TERMINATE_BIG_COMPLETE_SUBEVT_CODE | HCI_LE_BIG_SYNC_LOST_SUBEVT_CODE => {
            if !sub.is_empty() {
                unsafe {
                    AUDIO_UnregisterStream(grp_type_t_GRP_TYPE_BIG, sub[0], 0);
                }
            }
        }
        _ => {}
    }
}

fn register_big_handles(big_handle: u8, handle_bytes: &[u8], num_bis: usize, iso_interval: u16, latency: u32) {
    let available = handle_bytes.len() / 2;
    let count = num_bis.min(available).min(MAX_STREAMS);
    let mut handles = [0u16; MAX_STREAMS];
    for (i, dst) in handles.iter_mut().take(count).enumerate() {
        *dst = u16::from_le_bytes([handle_bytes[2 * i], handle_bytes[2 * i + 1]]);
    }
    unsafe {
        AUDIO_RegisterStream(
            grp_type_t_GRP_TYPE_BIG,
            big_handle,
            count as u8,
            handles.as_mut_ptr(),
            iso_interval,
            0,
            latency,
            0,
        );
    }
}

/// Forward a single received LL ISO SDU buffer into `CODEC_ReceiveMediaPacket`.
/// Returns `Ok(true)` if consumed by Codec Manager, `Ok(false)` if busy, `Err` on failure.
unsafe fn try_deliver_rx_sdu(conn_handle: u16, ptr_sdu: iso_sdu_buf_hdr_p) -> Result<bool, ()> {
    if ptr_sdu.is_null() {
        return Err(());
    }
    let sdu = unsafe { &*ptr_sdu };
    let ts_flag = u8::from(
        sdu.pb_flag == iso_sdu_fragment_t_FIRST_SDU_FRAG || sdu.pb_flag == iso_sdu_fragment_t_FULL_SDU_FRAG,
    );
    let status = unsafe {
        CODEC_ReceiveMediaPacket(
            conn_handle,
            sdu.pb_flag,
            ts_flag,
            sdu.time_stamp,
            sdu.pkt_sqnc_num,
            sdu.pkt_status_flag,
            sdu.iso_sdu_len,
            sdu.ptr_sdu_buffer.cast::<u8>(),
        )
    };
    match status {
        x if x == codec_rcv_status_t_CODEC_RCV_STATUS_OK => Ok(true),
        x if x == codec_rcv_status_t_CODEC_RCV_STATUS_BUSY => Ok(false),
        _ => Err(()),
    }
}

/// Vendor-specific LL output path callback invoked in LL context for streams
/// using a codec-managed RX data path (`DATA_PATH_SAMPLE_CIRC_BUF` / `FRAME_CIRC_BUF`).
unsafe extern "C" fn rx_iso_data_vendor_clbk(ptr_sdu: iso_sdu_buf_hdr_p, conn_handle: u16) {
    if ptr_sdu.is_null() {
        return;
    }

    let consumed_or_err = unsafe { try_deliver_rx_sdu(conn_handle, ptr_sdu) };
    match consumed_or_err {
        Ok(true) => unsafe { ll_intf_free_iso_sdu(ptr_sdu) },
        Ok(false) => {
            let queued = BRIDGE_STATE.lock(|cell| cell.borrow_mut().push_rx_sdu(conn_handle, ptr_sdu));
            if !queued {
                unsafe { ll_intf_free_iso_sdu(ptr_sdu) };
            }
        }
        Err(()) => unsafe { ll_intf_free_iso_sdu(ptr_sdu) },
    }
}

// ============================================================================
// Strong overrides for weak `BLECB_*` hooks in `libstm32wba_ble_stack_full.a`
// ============================================================================

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLECB_ConfigureDataPath(
    data_path_direction: u8,
    data_path_id: u8,
    vendor_specific_config_length: u8,
    vendor_specific_config: *const u8,
) -> u8 {
    if !is_active() {
        return HCI_COMMAND_DISALLOWED_ERR_CODE as u8;
    }
    if data_path_id == CODEC_DataPathID_t_DATA_PATH_HCI_LEGACY {
        return HCI_INVALID_HCI_COMMAND_PARAMETERS_ERR_CODE as u8;
    }
    unsafe {
        CODEC_ConfigureDataPath(
            data_path_direction,
            data_path_id,
            vendor_specific_config_length,
            vendor_specific_config.cast_mut(),
        )
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLECB_ReadLocalSupportedCodecs(
    num_supported_std_codecs: *mut u8,
    std_codec: *mut u8,
    num_supported_vs_codecs: *mut u8,
    vs_codec: *mut u8,
) -> u8 {
    if !is_active() {
        if !num_supported_std_codecs.is_null() {
            unsafe { *num_supported_std_codecs = 0 };
        }
        if !num_supported_vs_codecs.is_null() {
            unsafe { *num_supported_vs_codecs = 0 };
        }
        return HCI_SUCCESS_ERR_CODE as u8;
    }

    let mut std_ptr: *mut u8 = core::ptr::null_mut();
    let mut vs_ptr: *mut u8 = core::ptr::null_mut();
    let status = unsafe {
        CODEC_ReadLocalSupportedCodecsV2(
            num_supported_std_codecs,
            &mut std_ptr,
            num_supported_vs_codecs,
            &mut vs_ptr,
        )
    };
    if status == HCI_SUCCESS_ERR_CODE as u8 {
        if !std_codec.is_null() && !std_ptr.is_null() && !num_supported_std_codecs.is_null() {
            let n = unsafe { *num_supported_std_codecs } as usize;
            unsafe { core::ptr::copy_nonoverlapping(std_ptr, std_codec, n) };
        }
        if !vs_codec.is_null() && !vs_ptr.is_null() && !num_supported_vs_codecs.is_null() {
            let n = (unsafe { *num_supported_vs_codecs } as usize) * 4;
            unsafe { core::ptr::copy_nonoverlapping(vs_ptr, vs_codec, n) };
        }
    }
    status
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLECB_ReadLocalSupportedCodecCapabilities(
    codec_id: *const u8,
    logical_transport_type: u8,
    direction: u8,
    num_codec_capabilities: *mut u8,
    codec_capability: *mut u8,
) -> u8 {
    if !is_active() {
        return HCI_UNKNOWN_HCI_COMMAND_ERR_CODE as u8;
    }

    let mut cap_ptr: *mut u8 = core::ptr::null_mut();
    let status = unsafe {
        CODEC_ReadLocalSupportedCodecCapabilies(
            codec_id.cast_mut(),
            logical_transport_type,
            direction,
            num_codec_capabilities,
            &mut cap_ptr,
        )
    };
    if status == HCI_SUCCESS_ERR_CODE as u8
        && !codec_capability.is_null()
        && !cap_ptr.is_null()
        && !num_codec_capabilities.is_null()
    {
        let num_caps = unsafe { *num_codec_capabilities } as usize;
        // Format: [len_0, bytes_0..., len_1, bytes_1..., ...]
        let mut offset = 0usize;
        for _ in 0..num_caps {
            let item_len = unsafe { *cap_ptr.add(offset) } as usize;
            offset += 1 + item_len;
        }
        unsafe { core::ptr::copy_nonoverlapping(cap_ptr, codec_capability, offset) };
    }
    status
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLECB_ReadLocalSupportedControllerDelay(
    codec_id: *const u8,
    logical_transport_type: u8,
    direction: u8,
    codec_configuration_length: u8,
    codec_configuration: *const u8,
    min_controller_delay: *mut u8,
    max_controller_delay: *mut u8,
) -> u8 {
    if !is_active() {
        // Default 0..4000 us controller delay when no on-chip codec manager is active.
        if !min_controller_delay.is_null() {
            unsafe { core::ptr::copy_nonoverlapping([0u8; 3].as_ptr(), min_controller_delay, 3) };
        }
        if !max_controller_delay.is_null() {
            let max_d = 4000u32.to_le_bytes();
            unsafe { core::ptr::copy_nonoverlapping(max_d.as_ptr(), max_controller_delay, 3) };
        }
        return HCI_SUCCESS_ERR_CODE as u8;
    }

    let mut min_delay = 0u32;
    let mut max_delay = 0u32;
    let status = unsafe {
        CODEC_ReadLocalSupportedControllerDelay(
            codec_id.cast_mut(),
            logical_transport_type,
            direction,
            codec_configuration_length,
            codec_configuration.cast_mut(),
            &mut min_delay,
            &mut max_delay,
        )
    };
    if status == HCI_SUCCESS_ERR_CODE as u8 {
        if !min_controller_delay.is_null() {
            unsafe { core::ptr::copy_nonoverlapping(min_delay.to_le_bytes().as_ptr(), min_controller_delay, 3) };
        }
        if !max_controller_delay.is_null() {
            unsafe { core::ptr::copy_nonoverlapping(max_delay.to_le_bytes().as_ptr(), max_controller_delay, 3) };
        }
    }
    status
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLECB_SetupIsoDataPath(
    conn_handle: u16,
    data_path_direction: u8,
    data_path_id: u8,
    codec_id: *const u8,
    controller_delay: *const u8,
    codec_configuration_length: u8,
    codec_configuration: *const u8,
) -> u8 {
    let mut ll_params = ble_intf_setup_iso_data_path {
        codec_config: codec_configuration.cast_mut(),
        controller_delay: if controller_delay.is_null() {
            0
        } else {
            let mut b = [0u8; 4];
            unsafe { core::ptr::copy_nonoverlapping(controller_delay, b.as_mut_ptr(), 3) };
            u32::from_le_bytes(b)
        },
        codec_id: if codec_id.is_null() {
            [0u8; 5]
        } else {
            let mut c = [0u8; 5];
            unsafe { core::ptr::copy_nonoverlapping(codec_id, c.as_mut_ptr(), 5) };
            c
        },
        data_path_dir: data_path_direction,
        data_path_id,
        codec_config_length: codec_configuration_length,
    };

    // Streams configured with DATA_PATH_HCI (0x00) always stay on the HCI ISO path,
    // even when Codec Manager is active for other streams.
    if data_path_id == DATA_PATH_HCI as u8 {
        let recorded =
            BRIDGE_STATE.lock(|cell| cell.borrow_mut().set_stream_path(conn_handle, data_path_direction, data_path_id));
        if !recorded {
            return HCI_MEMORY_CAPACITY_EXCEEDED_ERR_CODE as u8;
        }
        return unsafe { ll_intf_setup_iso_data_path(conn_handle, &mut ll_params) as u8 };
    }

    if !is_active() {
        return HCI_COMMAND_DISALLOWED_ERR_CODE as u8;
    }

    if codec_configuration_length as usize > 19 {
        return HCI_INVALID_HCI_COMMAND_PARAMETERS_ERR_CODE as u8;
    }

    let mut setup_cmd = CODEC_SetupIsoDataPathCmd_t {
        con_hdle: conn_handle,
        direction: data_path_direction,
        path_ID: data_path_id,
        codec_ID: ll_params.codec_id,
        controller_delay: if controller_delay.is_null() {
            [0u8; 3]
        } else {
            let mut d = [0u8; 3];
            unsafe { core::ptr::copy_nonoverlapping(controller_delay, d.as_mut_ptr(), 3) };
            d
        },
        codec_conf_len: codec_configuration_length,
        codec_conf: [0u8; 19],
    };
    if codec_configuration_length > 0 && !codec_configuration.is_null() {
        unsafe {
            core::ptr::copy_nonoverlapping(
                codec_configuration,
                setup_cmd.codec_conf.as_mut_ptr(),
                codec_configuration_length as usize,
            );
        }
    }

    let recorded =
        BRIDGE_STATE.lock(|cell| cell.borrow_mut().set_stream_path(conn_handle, data_path_direction, data_path_id));
    if !recorded {
        return HCI_MEMORY_CAPACITY_EXCEEDED_ERR_CODE as u8;
    }

    let codec_status = unsafe { CODEC_SetupIsoDataPath((&raw mut setup_cmd).cast::<u8>()) };
    if codec_status != HCI_SUCCESS_ERR_CODE as u8 {
        BRIDGE_STATE.lock(|cell| {
            let dir_mask = if data_path_direction as u32 == DATA_PATH_INPUT {
                DIR_MASK_INPUT
            } else {
                DIR_MASK_OUTPUT
            };
            let _ = cell.borrow_mut().take_removed_codec_dirs(conn_handle, dir_mask);
        });
        return codec_status;
    }

    let ll_status = unsafe { ll_intf_setup_iso_data_path(conn_handle, &mut ll_params) };
    if ll_status == HCI_SUCCESS_ERR_CODE && data_path_direction as u32 == DATA_PATH_OUTPUT {
        unsafe { ll_intf_set_output_data_path(conn_handle, Some(rx_iso_data_vendor_clbk)) };
    }
    ll_status as u8
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLECB_RemoveIsoDataPath(conn_handle: u16, data_path_direction: u8) -> u8 {
    let codec_dir_mask = BRIDGE_STATE.lock(|cell| {
        let mut st = cell.borrow_mut();
        if (data_path_direction & DIR_MASK_OUTPUT) != 0 {
            st.drain_handle_rx_sdus(conn_handle, |ptr_sdu| unsafe { ll_intf_free_iso_sdu(ptr_sdu) });
        }
        st.take_removed_codec_dirs(conn_handle, data_path_direction)
    });

    let ll_status = unsafe { ll_intf_rmv_iso_data_path(conn_handle, data_path_direction) };
    if codec_dir_mask != 0 && is_active() {
        let codec_status = unsafe { CODEC_RemoveIsoDataPath(conn_handle, codec_dir_mask) };
        if ll_status == HCI_SUCCESS_ERR_CODE {
            return codec_status;
        }
    }
    ll_status as u8
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLECB_SyncEvent(
    group_id: u8,
    next_anchor_point: u32,
    time_stamp: u32,
    next_sdu_delivery_timeout: u32,
) {
    if is_active() {
        unsafe { AUDIO_SyncEventClbk(group_id, next_anchor_point, time_stamp, next_sdu_delivery_timeout) };
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLECB_BigTerminateSync(status: u8, big_handle: u8) {
    if status == HCI_SUCCESS_ERR_CODE as u8 && is_active() {
        unsafe { AUDIO_UnregisterStream(grp_type_t_GRP_TYPE_BIG, big_handle, 0) };
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLECB_TerminateBig(status: u8, big_handle: u8) {
    if status == HCI_SUCCESS_ERR_CODE as u8 && is_active() {
        unsafe { AUDIO_UnregisterStream(grp_type_t_GRP_TYPE_BIG, big_handle, 0) };
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLECB_IsoCalibration(timestamp: u32) {
    if is_active() {
        unsafe { AUDIO_CalibrationClbk(timestamp) };
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLECB_IsHciRxIsoDataPathOn(conn_handle: u16) -> u8 {
    if !is_active() {
        return 1;
    }
    let is_hci = BRIDGE_STATE.lock(|cell| {
        let st = cell.borrow();
        match st.find_stream(conn_handle).and_then(|s| s.output_path_id) {
            Some(path_id) => path_id == DATA_PATH_HCI as u8,
            None => true,
        }
    });
    u8::from(is_hci)
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn BLECB_SendIsoData(
    conn_handle: u16,
    pb_flag: u8,
    ts_flag: u8,
    timestamp: u32,
    psn: u16,
    iso_data_load_length: u16,
    total_sdu_length: u16,
    iso_data: *mut u8,
) -> u8 {
    let allow_hci_tx = BRIDGE_STATE.lock(|cell| {
        let st = cell.borrow();
        match st.find_stream(conn_handle).and_then(|s| s.input_path_id) {
            Some(path_id) => path_id == DATA_PATH_HCI as u8,
            None => true,
        }
    });
    if !allow_hci_tx {
        return BLE_STATUS_FAILED as u8;
    }
    unsafe {
        BLE_SendIsoDataToLinkLayer(
            conn_handle,
            pb_flag,
            ts_flag,
            timestamp,
            psn,
            iso_data_load_length,
            total_sdu_length,
            iso_data,
        )
    }
}

// ============================================================================
// Codec Manager media packet callbacks (strong implementations for `libcodec_mngr.a`)
// ============================================================================

#[unsafe(no_mangle)]
pub unsafe extern "C" fn CODEC_CB_ReceiveMediaPacketReady(iso_con_hdl: u16) {
    loop {
        let next = BRIDGE_STATE.lock(|cell| cell.borrow_mut().pop_next_rx_sdu(iso_con_hdl));
        let Some(ptr_sdu) = next else {
            break;
        };

        match unsafe { try_deliver_rx_sdu(iso_con_hdl, ptr_sdu) } {
            Ok(true) | Err(()) => unsafe { ll_intf_free_iso_sdu(ptr_sdu) },
            Ok(false) => {
                // Restore at front to preserve FIFO order until space opens.
                let pushed = BRIDGE_STATE.lock(|cell| cell.borrow_mut().push_front_rx_sdu(iso_con_hdl, ptr_sdu));
                if !pushed {
                    unsafe { ll_intf_free_iso_sdu(ptr_sdu) };
                }
                break;
            }
        }
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn CODEC_CB_SendMediaPacket(
    iso_con_hdl: u16,
    pb_flag: u8,
    ts_flag: u8,
    timestamp: u32,
    psn: u16,
    iso_data_load_length: u16,
    total_sdu_len: u16,
    pdata: *mut u8,
) -> u8 {
    let status = unsafe {
        BLE_SendIsoDataToLinkLayer(
            iso_con_hdl,
            pb_flag,
            ts_flag,
            timestamp,
            psn,
            iso_data_load_length,
            total_sdu_len,
            pdata,
        )
    };
    match status as u32 {
        BLE_STATUS_SUCCESS => codec_rcv_status_t_CODEC_RCV_STATUS_OK,
        BLE_STATUS_PENDING => codec_rcv_status_t_CODEC_RCV_STATUS_BUSY,
        _ => codec_rcv_status_t_CODEC_RCV_STATUS_FAIL,
    }
}
