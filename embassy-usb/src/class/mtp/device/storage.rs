//! High-level [`MtpStorage`] trait and [`MtpRunner`] state machine.

use core::cmp::min;

use super::super::{
    CONTAINER_HEADER_LEN, DEPTH_ALL, DataType, DeviceStatusCode, EventCode, FORMAT_ALL, HANDLE_ALL, HANDLE_ROOT,
    MTP_EXTENSIONS_MICROSOFT, MTP_VENDOR_EXTENSION_ID, MTP_VERSION, ObjectEntry, ObjectFormat, ObjectPropCode,
    OperationCode, PROP_ALL, PTP_STANDARD_VERSION, ResponseBlock, ResponseCode, STORAGE_ALL, StorageInfo, WireError,
    ptp_str_encoded_len,
};
use super::Config;
use super::transport::{DataWriter, MtpTransport, TransportError};
use crate::driver::{Driver, EndpointIn};

/// Maximum number of storage volumes reported by [`MtpRunner`].
const MAX_STORAGES: usize = 8;

/// Maximum number of object handles fetched per batch while streaming.
const HANDLE_BATCH_LEN: usize = 32;

/// Object parent scope filter used by enumeration and creation operations.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParentScope {
    /// All objects in the storage regardless of folder nesting (`0x0000_0000`).
    All,
    /// Only direct children of the storage root (`0xFFFF_FFFF`).
    Root,
    /// Direct children of the given folder object handle.
    Folder(u32),
}

impl ParentScope {
    fn from_query_param(raw: u32) -> Self {
        match raw {
            HANDLE_ALL => Self::All,
            HANDLE_ROOT => Self::Root,
            other => Self::Folder(other),
        }
    }

    fn from_target_param(raw: u32) -> Self {
        if raw == HANDLE_ALL || raw == HANDLE_ROOT {
            Self::Root
        } else {
            Self::Folder(raw)
        }
    }
}

/// Async storage backend for [`MtpClass::run`](super::MtpClass::run).
///
/// Implementing the required read methods exposes a complete read-only MTP device
/// (including `GetDeviceInfo`, `GetStorageIDs`, `GetStorageInfo`, `GetObjectHandles`,
/// `GetObjectInfo`, `GetObject`, `GetPartialObject`, and MTP 1.1 `GetObjectProp*`
/// operations). Override [`Self::is_read_only`] and the optional write methods to
/// allow uploads, deletions, renames, moves, and copies.
pub trait MtpStorage {
    /// Returns `true` if the storage is read-only.
    fn is_read_only(&self) -> bool {
        true
    }

    /// Populates `out` with valid non-zero Storage IDs and returns the count written.
    async fn storage_ids(&mut self, out: &mut [u32]) -> Result<usize, ResponseCode>;

    /// Returns metadata for `storage_id`.
    async fn storage_info(&mut self, storage_id: u32) -> Result<StorageInfo, ResponseCode>;

    /// Counts objects matching `storage_id` (or [`STORAGE_ALL`]), optional `format`, and `parent`.
    async fn num_objects(
        &mut self,
        storage_id: u32,
        format: Option<ObjectFormat>,
        parent: ParentScope,
    ) -> Result<u32, ResponseCode>;

    /// Writes up to `out.len()` object handles starting at `offset`, returning the count written.
    async fn object_handles(
        &mut self,
        storage_id: u32,
        format: Option<ObjectFormat>,
        parent: ParentScope,
        offset: usize,
        out: &mut [u32],
    ) -> Result<usize, ResponseCode>;

    /// Returns metadata for a single object `handle`.
    async fn object_info(&mut self, handle: u32) -> Result<ObjectEntry, ResponseCode>;

    /// Reads object payload bytes starting at `offset` into `buf`, returning bytes read.
    async fn read_object(&mut self, handle: u32, offset: u64, buf: &mut [u8]) -> Result<usize, ResponseCode>;

    /// Prepares to create a file or folder announced by `SendObjectInfo`.
    ///
    /// Returns `(actual_storage_id, actual_parent_handle, new_object_handle)`.
    /// When `info.is_folder()` is `true`, the folder should be created immediately
    /// because the host will not follow with `SendObject`.
    async fn begin_send_object(
        &mut self,
        _storage_id: u32,
        _parent: ParentScope,
        _info: &ObjectEntry,
    ) -> Result<(u32, u32, u32), ResponseCode> {
        Err(ResponseCode::OPERATION_NOT_SUPPORTED)
    }

    /// Writes a chunk of object data for an active `SendObject` at `offset`.
    async fn write_object_data(&mut self, _handle: u32, _offset: u64, _data: &[u8]) -> Result<(), ResponseCode> {
        Err(ResponseCode::OPERATION_NOT_SUPPORTED)
    }

    /// Completes (`ok == true`) or aborts (`ok == false`) an active `SendObject`.
    async fn finish_send_object(&mut self, _handle: u32, _ok: bool) -> Result<(), ResponseCode> {
        Err(ResponseCode::OPERATION_NOT_SUPPORTED)
    }

    /// Deletes an object (and its descendants if it is a folder).
    async fn delete_object(&mut self, _handle: u32, _format: Option<ObjectFormat>) -> Result<(), ResponseCode> {
        Err(ResponseCode::OPERATION_NOT_SUPPORTED)
    }

    /// Renames an existing object (`SetObjectPropValue` for `ObjectFileName` or `Name`).
    async fn rename_object(&mut self, _handle: u32, _new_name: &str) -> Result<(), ResponseCode> {
        Err(ResponseCode::OPERATION_NOT_SUPPORTED)
    }

    /// Moves `handle` to `storage_id` under `new_parent`.
    async fn move_object(
        &mut self,
        _handle: u32,
        _storage_id: u32,
        _new_parent: ParentScope,
    ) -> Result<(), ResponseCode> {
        Err(ResponseCode::OPERATION_NOT_SUPPORTED)
    }

    /// Copies `handle` to `storage_id` under `new_parent`, returning the new object handle.
    async fn copy_object(
        &mut self,
        _handle: u32,
        _storage_id: u32,
        _new_parent: ParentScope,
    ) -> Result<u32, ResponseCode> {
        Err(ResponseCode::OPERATION_NOT_SUPPORTED)
    }
}

const READ_OPS: &[u16] = &[
    OperationCode::GET_DEVICE_INFO.0,
    OperationCode::OPEN_SESSION.0,
    OperationCode::CLOSE_SESSION.0,
    OperationCode::GET_STORAGE_IDS.0,
    OperationCode::GET_STORAGE_INFO.0,
    OperationCode::GET_NUM_OBJECTS.0,
    OperationCode::GET_OBJECT_HANDLES.0,
    OperationCode::GET_OBJECT_INFO.0,
    OperationCode::GET_OBJECT.0,
    OperationCode::GET_PARTIAL_OBJECT.0,
    OperationCode::GET_OBJECT_PROPS_SUPPORTED.0,
    OperationCode::GET_OBJECT_PROP_DESC.0,
    OperationCode::GET_OBJECT_PROP_VALUE.0,
    OperationCode::GET_OBJECT_PROP_LIST.0,
];

pub(super) const RW_OPS: &[u16] = &[
    OperationCode::GET_DEVICE_INFO.0,
    OperationCode::OPEN_SESSION.0,
    OperationCode::CLOSE_SESSION.0,
    OperationCode::GET_STORAGE_IDS.0,
    OperationCode::GET_STORAGE_INFO.0,
    OperationCode::GET_NUM_OBJECTS.0,
    OperationCode::GET_OBJECT_HANDLES.0,
    OperationCode::GET_OBJECT_INFO.0,
    OperationCode::GET_OBJECT.0,
    OperationCode::DELETE_OBJECT.0,
    OperationCode::SEND_OBJECT_INFO.0,
    OperationCode::SEND_OBJECT.0,
    OperationCode::MOVE_OBJECT.0,
    OperationCode::COPY_OBJECT.0,
    OperationCode::GET_PARTIAL_OBJECT.0,
    OperationCode::GET_OBJECT_PROPS_SUPPORTED.0,
    OperationCode::GET_OBJECT_PROP_DESC.0,
    OperationCode::GET_OBJECT_PROP_VALUE.0,
    OperationCode::SET_OBJECT_PROP_VALUE.0,
    OperationCode::GET_OBJECT_PROP_LIST.0,
];

const SUPPORTED_EVENTS: &[u16] = &[
    EventCode::CANCEL_TRANSACTION.0,
    EventCode::OBJECT_ADDED.0,
    EventCode::OBJECT_REMOVED.0,
    EventCode::STORE_ADDED.0,
    EventCode::STORE_REMOVED.0,
    EventCode::OBJECT_INFO_CHANGED.0,
    EventCode::STORAGE_INFO_CHANGED.0,
    EventCode::OBJECT_PROP_CHANGED.0,
];

const SUPPORTED_FORMATS: &[u16] = &[
    ObjectFormat::UNDEFINED.0,
    ObjectFormat::ASSOCIATION.0,
    ObjectFormat::TEXT.0,
    ObjectFormat::HTML.0,
    ObjectFormat::WAV.0,
    ObjectFormat::MP3.0,
    ObjectFormat::EXIF_JPEG.0,
    ObjectFormat::GIF.0,
    ObjectFormat::PNG.0,
    ObjectFormat::TIFF.0,
    ObjectFormat::MP4.0,
    ObjectFormat::XML_DOCUMENT.0,
];

const SUPPORTED_PROPS: &[u16] = &[
    ObjectPropCode::STORAGE_ID.0,
    ObjectPropCode::OBJECT_FORMAT.0,
    ObjectPropCode::PROTECTION_STATUS.0,
    ObjectPropCode::OBJECT_SIZE.0,
    ObjectPropCode::OBJECT_FILE_NAME.0,
    ObjectPropCode::DATE_MODIFIED.0,
    ObjectPropCode::PARENT_OBJECT.0,
    ObjectPropCode::PERSISTENT_UNIQUE_OBJECT_ID.0,
    ObjectPropCode::NAME.0,
];

#[derive(Copy, Clone)]
struct PendingSend {
    handle: u32,
}

/// Runs the MTP command/data/response loop against an [`MtpStorage`].
pub(super) struct MtpRunner<'d> {
    config: Config<'d>,
    session_id: Option<u32>,
    pending_send: Option<PendingSend>,
}

impl<'d> MtpRunner<'d> {
    pub(super) fn new(config: Config<'d>) -> Self {
        Self {
            config,
            session_id: None,
            pending_send: None,
        }
    }

    pub(super) async fn run_connected<D: Driver<'d>, S: MtpStorage>(
        &mut self,
        transport: &mut MtpTransport<'d, D>,
        storage: &mut S,
        buf: &mut [u8],
    ) -> Result<(), TransportError> {
        self.session_id = None;
        self.pending_send = None;

        loop {
            let cmd = match transport.read_command(buf).await {
                Ok(cmd) => cmd,
                Err(TransportError::Reset) => {
                    self.abort_pending_send(storage).await;
                    self.session_id = None;
                    continue;
                }
                Err(TransportError::Endpoint(e)) => return Err(TransportError::Endpoint(e)),
                Err(e) => {
                    warn!("mtp: command read error: {:?}", e);
                    continue;
                }
            };

            transport.control().set_status(DeviceStatusCode::DEVICE_BUSY);
            let resp = match self.dispatch(transport, storage, &cmd, buf).await {
                Ok(resp) => resp,
                Err(TransportError::Cancelled) => {
                    let _ = transport.control().take_cancel(cmd.transaction_id);
                    self.abort_pending_send(storage).await;
                    ResponseBlock::new(ResponseCode::TRANSACTION_CANCELLED, cmd.transaction_id)
                }
                Err(TransportError::Reset) => {
                    self.abort_pending_send(storage).await;
                    self.session_id = None;
                    transport.control().set_status(DeviceStatusCode::OK);
                    continue;
                }
                Err(TransportError::Wire(_)) | Err(TransportError::UnexpectedLength) => {
                    self.abort_pending_send(storage).await;
                    ResponseBlock::new(ResponseCode::INVALID_DATASET, cmd.transaction_id)
                }
                Err(TransportError::BufferTooSmall) => {
                    ResponseBlock::new(ResponseCode::GENERAL_ERROR, cmd.transaction_id)
                }
                Err(TransportError::Endpoint(e)) => {
                    transport.control().set_status(DeviceStatusCode::OK);
                    return Err(TransportError::Endpoint(e));
                }
            };

            transport.control().set_status(DeviceStatusCode::OK);
            transport.send_response(&resp).await?;
        }
    }

    async fn abort_pending_send<S: MtpStorage>(&mut self, storage: &mut S) {
        if let Some(pending) = self.pending_send.take() {
            let _ = storage.finish_send_object(pending.handle, false).await;
        }
    }

    async fn dispatch<D: Driver<'d>, S: MtpStorage>(
        &mut self,
        transport: &mut MtpTransport<'d, D>,
        storage: &mut S,
        cmd: &super::super::CommandBlock,
        buf: &mut [u8],
    ) -> Result<ResponseBlock, TransportError> {
        let tx = cmd.transaction_id;

        if self.session_id.is_none()
            && cmd.operation != OperationCode::GET_DEVICE_INFO
            && cmd.operation != OperationCode::OPEN_SESSION
        {
            if matches!(
                cmd.operation,
                OperationCode::SEND_OBJECT_INFO | OperationCode::SEND_OBJECT | OperationCode::SET_OBJECT_PROP_VALUE
            ) {
                let reader = transport.start_data_out(tx, buf).await?;
                reader.discard().await?;
            }
            return Ok(ResponseBlock::new(ResponseCode::SESSION_NOT_OPEN, tx));
        }

        match cmd.operation {
            OperationCode::GET_DEVICE_INFO => self.get_device_info(transport, storage, tx, buf).await,
            OperationCode::OPEN_SESSION => {
                let sid = cmd.param(0);
                if sid == 0 {
                    Ok(ResponseBlock::new(ResponseCode::INVALID_PARAMETER, tx))
                } else if self.session_id.is_some() {
                    Ok(ResponseBlock::new(ResponseCode::SESSION_ALREADY_OPEN, tx))
                } else {
                    self.session_id = Some(sid);
                    Ok(ResponseBlock::ok(tx))
                }
            }
            OperationCode::CLOSE_SESSION => {
                self.abort_pending_send(storage).await;
                self.session_id = None;
                Ok(ResponseBlock::ok(tx))
            }
            OperationCode::GET_STORAGE_IDS => self.get_storage_ids(transport, storage, tx, buf).await,
            OperationCode::GET_STORAGE_INFO => self.get_storage_info(transport, storage, cmd.param(0), tx, buf).await,
            OperationCode::GET_NUM_OBJECTS => self.get_num_objects(storage, cmd, tx).await,
            OperationCode::GET_OBJECT_HANDLES => self.get_object_handles(transport, storage, cmd, tx, buf).await,
            OperationCode::GET_OBJECT_INFO => self.get_object_info(transport, storage, cmd.param(0), tx, buf).await,
            OperationCode::GET_OBJECT => self.get_object(transport, storage, cmd.param(0), tx, buf).await,
            OperationCode::GET_PARTIAL_OBJECT => {
                self.get_partial_object(
                    transport,
                    storage,
                    cmd.param(0),
                    cmd.param(1) as u64,
                    cmd.param(2),
                    tx,
                    buf,
                )
                .await
            }
            OperationCode::DELETE_OBJECT => {
                let format = optional_format(cmd.param(1));
                Ok(match storage.delete_object(cmd.param(0), format).await {
                    Ok(()) => ResponseBlock::ok(tx),
                    Err(code) => ResponseBlock::new(code, tx),
                })
            }
            OperationCode::SEND_OBJECT_INFO => self.send_object_info(transport, storage, cmd, tx, buf).await,
            OperationCode::SEND_OBJECT => self.send_object(transport, storage, tx, buf).await,
            OperationCode::MOVE_OBJECT => {
                let parent = ParentScope::from_target_param(cmd.param(2));
                Ok(match storage.move_object(cmd.param(0), cmd.param(1), parent).await {
                    Ok(()) => ResponseBlock::ok(tx),
                    Err(code) => ResponseBlock::new(code, tx),
                })
            }
            OperationCode::COPY_OBJECT => {
                let parent = ParentScope::from_target_param(cmd.param(2));
                Ok(match storage.copy_object(cmd.param(0), cmd.param(1), parent).await {
                    Ok(new_handle) => ResponseBlock::with_params(ResponseCode::OK, tx, &[new_handle]),
                    Err(code) => ResponseBlock::new(code, tx),
                })
            }
            OperationCode::GET_OBJECT_PROPS_SUPPORTED => {
                self.get_object_props_supported(transport, cmd.param(0), tx, buf).await
            }
            OperationCode::GET_OBJECT_PROP_DESC => {
                self.get_object_prop_desc(transport, storage, cmd.param(0) as u16, tx, buf)
                    .await
            }
            OperationCode::GET_OBJECT_PROP_VALUE => {
                self.get_object_prop_value(transport, storage, cmd.param(0), cmd.param(1) as u16, tx, buf)
                    .await
            }
            OperationCode::SET_OBJECT_PROP_VALUE => {
                self.set_object_prop_value(transport, storage, cmd.param(0), cmd.param(1) as u16, tx, buf)
                    .await
            }
            OperationCode::GET_OBJECT_PROP_LIST => self.get_object_prop_list(transport, storage, cmd, tx, buf).await,
            OperationCode::RESET_DEVICE => {
                self.abort_pending_send(storage).await;
                self.session_id = None;
                Ok(ResponseBlock::ok(tx))
            }
            _ => Ok(ResponseBlock::new(ResponseCode::OPERATION_NOT_SUPPORTED, tx)),
        }
    }

    async fn get_device_info<D: Driver<'d>, S: MtpStorage>(
        &self,
        transport: &mut MtpTransport<'d, D>,
        storage: &S,
        tx: u32,
        buf: &mut [u8],
    ) -> Result<ResponseBlock, TransportError> {
        let ops = if storage.is_read_only() { READ_OPS } else { RW_OPS };
        let total_len = 2
            + 4
            + 2
            + ptp_str_encoded_len(MTP_EXTENSIONS_MICROSOFT)
            + 2
            + (4 + ops.len() * 2)
            + (4 + SUPPORTED_EVENTS.len() * 2)
            + 4 // DevicePropertiesSupported
            + 4 // CaptureFormats
            + (4 + SUPPORTED_FORMATS.len() * 2)
            + ptp_str_encoded_len(self.config.manufacturer)
            + ptp_str_encoded_len(self.config.model)
            + ptp_str_encoded_len(self.config.device_version)
            + ptp_str_encoded_len(self.config.serial_number);

        let max_pkt = transport.max_packet_size() as usize;
        let mut writer = transport.start_data_in(
            OperationCode::GET_DEVICE_INFO.0,
            tx,
            Some(total_len as u32),
            &mut buf[..max_pkt],
        )?;

        write_u16(&mut writer, PTP_STANDARD_VERSION).await?;
        write_u32(&mut writer, MTP_VENDOR_EXTENSION_ID).await?;
        write_u16(&mut writer, MTP_VERSION).await?;
        write_ptp_str(&mut writer, MTP_EXTENSIONS_MICROSOFT).await?;
        write_u16(&mut writer, 0).await?; // FunctionalMode = Standard
        write_u16_array(&mut writer, ops).await?;
        write_u16_array(&mut writer, SUPPORTED_EVENTS).await?;
        write_u16_array(&mut writer, &[]).await?;
        write_u16_array(&mut writer, &[]).await?;
        write_u16_array(&mut writer, SUPPORTED_FORMATS).await?;
        write_ptp_str(&mut writer, self.config.manufacturer).await?;
        write_ptp_str(&mut writer, self.config.model).await?;
        write_ptp_str(&mut writer, self.config.device_version).await?;
        write_ptp_str(&mut writer, self.config.serial_number).await?;
        writer.finish().await?;

        Ok(ResponseBlock::ok(tx))
    }

    async fn get_storage_ids<D: Driver<'d>, S: MtpStorage>(
        &self,
        transport: &mut MtpTransport<'d, D>,
        storage: &mut S,
        tx: u32,
        buf: &mut [u8],
    ) -> Result<ResponseBlock, TransportError> {
        let mut ids = [0u32; MAX_STORAGES];
        let count = match storage.storage_ids(&mut ids).await {
            Ok(n) => n.min(MAX_STORAGES),
            Err(code) => return Ok(ResponseBlock::new(code, tx)),
        };

        let mut payload = [0u8; 4 + MAX_STORAGES * 4];
        let mut w = super::super::DatasetWriter::new(&mut payload);
        w.write_u32_array(&ids[..count])?;
        let len = w.len();
        transport
            .write_data(OperationCode::GET_STORAGE_IDS.0, tx, &payload[..len], buf)
            .await?;
        Ok(ResponseBlock::ok(tx))
    }

    async fn get_storage_info<D: Driver<'d>, S: MtpStorage>(
        &self,
        transport: &mut MtpTransport<'d, D>,
        storage: &mut S,
        storage_id: u32,
        tx: u32,
        buf: &mut [u8],
    ) -> Result<ResponseBlock, TransportError> {
        let info = match storage.storage_info(storage_id).await {
            Ok(info) => info,
            Err(code) => return Ok(ResponseBlock::new(code, tx)),
        };

        let mut payload = [0u8; 256];
        let mut w = super::super::DatasetWriter::new(&mut payload);
        w.write_u16(info.storage_type as u16)?;
        w.write_u16(info.filesystem_type as u16)?;
        w.write_u16(info.access_capability as u16)?;
        w.write_u64(info.max_capacity)?;
        w.write_u64(info.free_space_bytes)?;
        w.write_u32(info.free_space_objects)?;
        w.write_ptp_str(&info.description)?;
        w.write_ptp_str(&info.volume_id)?;
        let len = w.len();

        transport
            .write_data(OperationCode::GET_STORAGE_INFO.0, tx, &payload[..len], buf)
            .await?;
        Ok(ResponseBlock::ok(tx))
    }

    async fn get_num_objects<S: MtpStorage>(
        &self,
        storage: &mut S,
        cmd: &super::super::CommandBlock,
        tx: u32,
    ) -> Result<ResponseBlock, TransportError> {
        let storage_id = cmd.param(0);
        let format = optional_format(cmd.param(1));
        let parent = ParentScope::from_query_param(cmd.param(2));

        Ok(match storage.num_objects(storage_id, format, parent).await {
            Ok(count) => ResponseBlock::with_params(ResponseCode::OK, tx, &[count]),
            Err(code) => ResponseBlock::new(code, tx),
        })
    }

    async fn get_object_handles<D: Driver<'d>, S: MtpStorage>(
        &self,
        transport: &mut MtpTransport<'d, D>,
        storage: &mut S,
        cmd: &super::super::CommandBlock,
        tx: u32,
        buf: &mut [u8],
    ) -> Result<ResponseBlock, TransportError> {
        let storage_id = cmd.param(0);
        let format = optional_format(cmd.param(1));
        let parent = ParentScope::from_query_param(cmd.param(2));

        let total = match storage.num_objects(storage_id, format, parent).await {
            Ok(n) => n as usize,
            Err(code) => return Ok(ResponseBlock::new(code, tx)),
        };

        let payload_len = 4 + total * 4;
        let max_pkt = transport.max_packet_size() as usize;
        let mut writer = transport.start_data_in(
            OperationCode::GET_OBJECT_HANDLES.0,
            tx,
            Some(payload_len as u32),
            &mut buf[..max_pkt],
        )?;
        write_u32(&mut writer, total as u32).await?;

        let mut offset = 0;
        let mut batch = [0u32; HANDLE_BATCH_LEN];
        while offset < total {
            let want = min(HANDLE_BATCH_LEN, total - offset);
            let got = match storage
                .object_handles(storage_id, format, parent, offset, &mut batch[..want])
                .await
            {
                Ok(0) => break,
                Ok(n) => n.min(want),
                Err(_) => break,
            };
            for &h in &batch[..got] {
                write_u32(&mut writer, h).await?;
            }
            offset += got;
        }
        // Pad with zero handles only if the storage returned fewer than `num_objects`
        // so the declared container byte length still matches.
        while offset < total {
            write_u32(&mut writer, 0).await?;
            offset += 1;
        }
        writer.finish().await?;
        Ok(ResponseBlock::ok(tx))
    }

    async fn get_object_info<D: Driver<'d>, S: MtpStorage>(
        &self,
        transport: &mut MtpTransport<'d, D>,
        storage: &mut S,
        handle: u32,
        tx: u32,
        buf: &mut [u8],
    ) -> Result<ResponseBlock, TransportError> {
        let entry = match storage.object_info(handle).await {
            Ok(e) => e,
            Err(code) => return Ok(ResponseBlock::new(code, tx)),
        };

        let mut payload = [0u8; 512];
        let len = entry.encode_object_info(&mut payload)?;
        transport
            .write_data(OperationCode::GET_OBJECT_INFO.0, tx, &payload[..len], buf)
            .await?;
        Ok(ResponseBlock::ok(tx))
    }

    async fn get_object<D: Driver<'d>, S: MtpStorage>(
        &self,
        transport: &mut MtpTransport<'d, D>,
        storage: &mut S,
        handle: u32,
        tx: u32,
        buf: &mut [u8],
    ) -> Result<ResponseBlock, TransportError> {
        let entry = match storage.object_info(handle).await {
            Ok(e) => e,
            Err(code) => return Ok(ResponseBlock::new(code, tx)),
        };

        let total = entry.size;
        let container_payload_len = if total + (CONTAINER_HEADER_LEN as u64) >= u32::MAX as u64 {
            None
        } else {
            Some(total as u32)
        };
        let sent = self
            .stream_object_range(
                transport,
                storage,
                OperationCode::GET_OBJECT.0,
                handle,
                0,
                total,
                container_payload_len,
                tx,
                buf,
            )
            .await?;
        if sent < total {
            return Ok(ResponseBlock::new(ResponseCode::INCOMPLETE_TRANSFER, tx));
        }
        Ok(ResponseBlock::ok(tx))
    }

    #[allow(clippy::too_many_arguments)]
    async fn get_partial_object<D: Driver<'d>, S: MtpStorage>(
        &self,
        transport: &mut MtpTransport<'d, D>,
        storage: &mut S,
        handle: u32,
        offset: u64,
        max_bytes: u32,
        tx: u32,
        buf: &mut [u8],
    ) -> Result<ResponseBlock, TransportError> {
        let entry = match storage.object_info(handle).await {
            Ok(e) => e,
            Err(code) => return Ok(ResponseBlock::new(code, tx)),
        };

        let avail = entry.size.saturating_sub(offset);
        let to_send = min(avail, max_bytes as u64);
        let sent = self
            .stream_object_range(
                transport,
                storage,
                OperationCode::GET_PARTIAL_OBJECT.0,
                handle,
                offset,
                to_send,
                Some(to_send as u32),
                tx,
                buf,
            )
            .await?;
        Ok(ResponseBlock::with_params(ResponseCode::OK, tx, &[sent as u32]))
    }

    #[allow(clippy::too_many_arguments)]
    async fn stream_object_range<D: Driver<'d>, S: MtpStorage>(
        &self,
        transport: &mut MtpTransport<'d, D>,
        storage: &mut S,
        op_code: u16,
        handle: u32,
        start_offset: u64,
        to_send: u64,
        declared_len: Option<u32>,
        tx: u32,
        buf: &mut [u8],
    ) -> Result<u64, TransportError> {
        let max_pkt = transport.max_packet_size() as usize;
        let (pkt_buf, rest) = buf.split_at_mut(max_pkt);
        let mut stack_chunk = [0u8; 256];
        let chunk_buf = if rest.is_empty() { &mut stack_chunk[..] } else { rest };

        let mut writer = transport.start_data_in(op_code, tx, declared_len, pkt_buf)?;
        let mut sent: u64 = 0;
        while sent < to_send {
            let want = min(chunk_buf.len() as u64, to_send - sent) as usize;
            let n = match storage
                .read_object(handle, start_offset + sent, &mut chunk_buf[..want])
                .await
            {
                Ok(0) => break,
                Ok(n) => min(n, want),
                Err(_) => break,
            };
            writer.write_all(&chunk_buf[..n]).await?;
            sent += n as u64;
        }
        writer.finish().await?;
        Ok(sent)
    }

    async fn send_object_info<D: Driver<'d>, S: MtpStorage>(
        &mut self,
        transport: &mut MtpTransport<'d, D>,
        storage: &mut S,
        cmd: &super::super::CommandBlock,
        tx: u32,
        buf: &mut [u8],
    ) -> Result<ResponseBlock, TransportError> {
        self.abort_pending_send(storage).await;

        let storage_id = cmd.param(0);
        let parent = ParentScope::from_target_param(cmd.param(1));

        let max_pkt = transport.max_packet_size() as usize;
        let mut raw_info = [0u8; 512];
        let reader = transport.start_data_out(tx, &mut buf[..max_pkt]).await?;
        let n = match reader.read_all(&mut raw_info).await {
            Ok(n) => n,
            Err(TransportError::BufferTooSmall) => {
                return Ok(ResponseBlock::new(ResponseCode::INVALID_DATASET, tx));
            }
            Err(e) => return Err(e),
        };

        let entry = match ObjectEntry::decode_object_info(&raw_info[..n]) {
            Ok(e) => e,
            Err(WireError::Truncated | WireError::InvalidString) => {
                return Ok(ResponseBlock::new(ResponseCode::INVALID_DATASET, tx));
            }
            Err(e) => return Err(TransportError::Wire(e)),
        };

        match storage.begin_send_object(storage_id, parent, &entry).await {
            Ok((actual_storage, actual_parent, new_handle)) => {
                if !entry.is_folder() {
                    self.pending_send = Some(PendingSend { handle: new_handle });
                }
                Ok(ResponseBlock::with_params(
                    ResponseCode::OK,
                    tx,
                    &[actual_storage, actual_parent, new_handle],
                ))
            }
            Err(code) => Ok(ResponseBlock::new(code, tx)),
        }
    }

    async fn send_object<D: Driver<'d>, S: MtpStorage>(
        &mut self,
        transport: &mut MtpTransport<'d, D>,
        storage: &mut S,
        tx: u32,
        buf: &mut [u8],
    ) -> Result<ResponseBlock, TransportError> {
        let Some(pending) = self.pending_send.take() else {
            let reader = transport.start_data_out(tx, buf).await?;
            reader.discard().await?;
            return Ok(ResponseBlock::new(ResponseCode::NO_VALID_OBJECT_INFO, tx));
        };

        let max_pkt = transport.max_packet_size() as usize;
        let (pkt_buf, rest) = buf.split_at_mut(max_pkt);
        let mut stack_chunk = [0u8; 256];
        let chunk_buf = if rest.is_empty() { &mut stack_chunk[..] } else { rest };

        let mut reader = match transport.start_data_out(tx, pkt_buf).await {
            Ok(r) => r,
            Err(e) => {
                let _ = storage.finish_send_object(pending.handle, false).await;
                return Err(e);
            }
        };

        let mut offset: u64 = 0;
        let mut write_err: Option<ResponseCode> = None;

        loop {
            let n = match reader.read_chunk(chunk_buf).await {
                Ok(0) => break,
                Ok(n) => n,
                Err(e) => {
                    let _ = storage.finish_send_object(pending.handle, false).await;
                    return Err(e);
                }
            };

            if write_err.is_none()
                && let Err(code) = storage.write_object_data(pending.handle, offset, &chunk_buf[..n]).await
            {
                write_err = Some(code);
            }
            offset += n as u64;
        }

        if let Some(code) = write_err {
            let _ = storage.finish_send_object(pending.handle, false).await;
            return Ok(ResponseBlock::new(code, tx));
        }

        Ok(match storage.finish_send_object(pending.handle, true).await {
            Ok(()) => ResponseBlock::ok(tx),
            Err(code) => ResponseBlock::new(code, tx),
        })
    }

    async fn get_object_props_supported<D: Driver<'d>>(
        &self,
        transport: &mut MtpTransport<'d, D>,
        format_code: u32,
        tx: u32,
        buf: &mut [u8],
    ) -> Result<ResponseBlock, TransportError> {
        let _ = format_code;
        let mut payload = [0u8; 4 + SUPPORTED_PROPS.len() * 2];
        let mut w = super::super::DatasetWriter::new(&mut payload);
        w.write_u16_array(SUPPORTED_PROPS)?;
        let len = w.len();
        transport
            .write_data(OperationCode::GET_OBJECT_PROPS_SUPPORTED.0, tx, &payload[..len], buf)
            .await?;
        Ok(ResponseBlock::ok(tx))
    }

    async fn get_object_prop_desc<D: Driver<'d>, S: MtpStorage>(
        &self,
        transport: &mut MtpTransport<'d, D>,
        storage: &S,
        prop_code: u16,
        tx: u32,
        buf: &mut [u8],
    ) -> Result<ResponseBlock, TransportError> {
        let prop = ObjectPropCode(prop_code);
        let Some((dt, writable)) = prop_meta(prop, storage.is_read_only()) else {
            return Ok(ResponseBlock::new(ResponseCode::OBJECT_PROP_NOT_SUPPORTED, tx));
        };

        let mut payload = [0u8; 32];
        let mut w = super::super::DatasetWriter::new(&mut payload);
        w.write_u16(prop.0)?;
        w.write_u16(dt.0)?;
        w.write_u8(u8::from(writable))?;
        match dt {
            DataType::UINT16 => w.write_u16(0)?,
            DataType::UINT32 => w.write_u32(0)?,
            DataType::UINT64 => w.write_u64(0)?,
            DataType::UINT128 => w.write_u128(0)?,
            DataType::STRING => w.write_u8(0)?,
            _ => {}
        }
        w.write_u32(0)?; // GroupCode
        w.write_u8(0)?; // FormFlag = None
        let len = w.len();
        transport
            .write_data(OperationCode::GET_OBJECT_PROP_DESC.0, tx, &payload[..len], buf)
            .await?;
        Ok(ResponseBlock::ok(tx))
    }

    async fn get_object_prop_value<D: Driver<'d>, S: MtpStorage>(
        &self,
        transport: &mut MtpTransport<'d, D>,
        storage: &mut S,
        handle: u32,
        prop_code: u16,
        tx: u32,
        buf: &mut [u8],
    ) -> Result<ResponseBlock, TransportError> {
        let prop = ObjectPropCode(prop_code);
        if prop_meta(prop, storage.is_read_only()).is_none() {
            return Ok(ResponseBlock::new(ResponseCode::OBJECT_PROP_NOT_SUPPORTED, tx));
        }
        let entry = match storage.object_info(handle).await {
            Ok(e) => e,
            Err(code) => return Ok(ResponseBlock::new(code, tx)),
        };

        let mut payload = [0u8; 300];
        let mut w = super::super::DatasetWriter::new(&mut payload);
        write_prop_value_dataset(&mut w, prop, handle, &entry)?;
        let len = w.len();
        transport
            .write_data(OperationCode::GET_OBJECT_PROP_VALUE.0, tx, &payload[..len], buf)
            .await?;
        Ok(ResponseBlock::ok(tx))
    }

    async fn set_object_prop_value<D: Driver<'d>, S: MtpStorage>(
        &self,
        transport: &mut MtpTransport<'d, D>,
        storage: &mut S,
        handle: u32,
        prop_code: u16,
        tx: u32,
        buf: &mut [u8],
    ) -> Result<ResponseBlock, TransportError> {
        let prop = ObjectPropCode(prop_code);
        let max_pkt = transport.max_packet_size() as usize;
        let mut raw = [0u8; 300];
        let reader = transport.start_data_out(tx, &mut buf[..max_pkt]).await?;
        let n = match reader.read_all(&mut raw).await {
            Ok(n) => n,
            Err(TransportError::BufferTooSmall) => {
                return Ok(ResponseBlock::new(ResponseCode::INVALID_OBJECT_PROP_VALUE, tx));
            }
            Err(e) => return Err(e),
        };

        if prop != ObjectPropCode::OBJECT_FILE_NAME && prop != ObjectPropCode::NAME {
            return Ok(ResponseBlock::new(ResponseCode::OBJECT_PROP_NOT_SUPPORTED, tx));
        }

        let mut dr = super::super::DatasetReader::new(&raw[..n]);
        let new_name: heapless::String<128> = match dr.read_ptp_str() {
            Ok(s) => s,
            Err(_) => return Ok(ResponseBlock::new(ResponseCode::INVALID_OBJECT_PROP_VALUE, tx)),
        };
        if new_name.is_empty() {
            return Ok(ResponseBlock::new(ResponseCode::INVALID_OBJECT_PROP_VALUE, tx));
        }

        Ok(match storage.rename_object(handle, &new_name).await {
            Ok(()) => ResponseBlock::ok(tx),
            Err(code) => ResponseBlock::new(code, tx),
        })
    }

    async fn get_object_prop_list<D: Driver<'d>, S: MtpStorage>(
        &self,
        transport: &mut MtpTransport<'d, D>,
        storage: &mut S,
        cmd: &super::super::CommandBlock,
        tx: u32,
        buf: &mut [u8],
    ) -> Result<ResponseBlock, TransportError> {
        let handle_param = cmd.param(0);
        let format_filter = optional_format(cmd.param(1));
        let prop_param = cmd.param(2);
        let group_param = cmd.param(3);
        let depth = cmd.param(4);

        if prop_param == 0 && group_param != 0 {
            return Ok(ResponseBlock::new(ResponseCode::SPECIFICATION_BY_GROUP_UNSUPPORTED, tx));
        }

        let selected_props: &[u16] = if prop_param == PROP_ALL {
            SUPPORTED_PROPS
        } else {
            let p = ObjectPropCode(prop_param as u16);
            if prop_meta(p, storage.is_read_only()).is_none() {
                return Ok(ResponseBlock::new(ResponseCode::OBJECT_PROP_NOT_SUPPORTED, tx));
            }
            core::slice::from_ref(&SUPPORTED_PROPS[prop_index(p).unwrap()])
        };

        // Determine traversal mode.
        enum Traversal {
            Empty,
            Single(u32),
            Scope(ParentScope),
        }

        let traversal = if handle_param == HANDLE_ALL || handle_param == HANDLE_ROOT {
            match depth {
                0 => {
                    if handle_param == HANDLE_ROOT {
                        Traversal::Empty
                    } else {
                        Traversal::Scope(ParentScope::All)
                    }
                }
                1 => Traversal::Scope(ParentScope::Root),
                DEPTH_ALL => Traversal::Scope(ParentScope::All),
                _ => return Ok(ResponseBlock::new(ResponseCode::SPECIFICATION_BY_DEPTH_UNSUPPORTED, tx)),
            }
        } else {
            match depth {
                0 => Traversal::Single(handle_param),
                1 => Traversal::Scope(ParentScope::Folder(handle_param)),
                _ => return Ok(ResponseBlock::new(ResponseCode::SPECIFICATION_BY_DEPTH_UNSUPPORTED, tx)),
            }
        };

        // Pass 1: compute exact element count and byte length so we can stream without
        // buffering an entire directory listing in RAM.
        let mut elem_count: u32 = 0;
        let mut payload_bytes: u32 = 4; // leading u32 element count

        match traversal {
            Traversal::Empty => {}
            Traversal::Single(h) => {
                let entry = match storage.object_info(h).await {
                    Ok(e) => e,
                    Err(code) => return Ok(ResponseBlock::new(code, tx)),
                };
                if format_filter.is_none_or(|f| f == entry.format) {
                    for &p in selected_props {
                        elem_count += 1;
                        payload_bytes += prop_list_item_len(ObjectPropCode(p), &entry) as u32;
                    }
                }
            }
            Traversal::Scope(scope) => {
                let total = match storage.num_objects(STORAGE_ALL, format_filter, scope).await {
                    Ok(n) => n as usize,
                    Err(code) => return Ok(ResponseBlock::new(code, tx)),
                };
                let mut offset = 0;
                let mut batch = [0u32; HANDLE_BATCH_LEN];
                while offset < total {
                    let want = min(HANDLE_BATCH_LEN, total - offset);
                    let got = match storage
                        .object_handles(STORAGE_ALL, format_filter, scope, offset, &mut batch[..want])
                        .await
                    {
                        Ok(0) => break,
                        Ok(n) => n.min(want),
                        Err(code) => return Ok(ResponseBlock::new(code, tx)),
                    };
                    for &h in &batch[..got] {
                        if let Ok(entry) = storage.object_info(h).await {
                            for &p in selected_props {
                                elem_count += 1;
                                payload_bytes += prop_list_item_len(ObjectPropCode(p), &entry) as u32;
                            }
                        }
                    }
                    offset += got;
                }
            }
        }

        // Pass 2: stream the `ObjectPropList` dataset.
        let max_pkt = transport.max_packet_size() as usize;
        let mut writer = transport.start_data_in(
            OperationCode::GET_OBJECT_PROP_LIST.0,
            tx,
            Some(payload_bytes),
            &mut buf[..max_pkt],
        )?;
        write_u32(&mut writer, elem_count).await?;

        match traversal {
            Traversal::Empty => {}
            Traversal::Single(h) => {
                if let Ok(entry) = storage.object_info(h).await
                    && format_filter.is_none_or(|f| f == entry.format)
                {
                    stream_prop_list_entry(&mut writer, h, &entry, selected_props).await?;
                }
            }
            Traversal::Scope(scope) => {
                let total = storage
                    .num_objects(STORAGE_ALL, format_filter, scope)
                    .await
                    .unwrap_or(0) as usize;
                let mut offset = 0;
                let mut batch = [0u32; HANDLE_BATCH_LEN];
                while offset < total {
                    let want = min(HANDLE_BATCH_LEN, total - offset);
                    let got = match storage
                        .object_handles(STORAGE_ALL, format_filter, scope, offset, &mut batch[..want])
                        .await
                    {
                        Ok(0) => break,
                        Ok(n) => n.min(want),
                        Err(_) => break,
                    };
                    for &h in &batch[..got] {
                        if let Ok(entry) = storage.object_info(h).await {
                            stream_prop_list_entry(&mut writer, h, &entry, selected_props).await?;
                        }
                    }
                    offset += got;
                }
            }
        }

        writer.finish().await?;
        Ok(ResponseBlock::ok(tx))
    }
}

fn optional_format(raw: u32) -> Option<ObjectFormat> {
    if raw == FORMAT_ALL as u32 || raw == 0xFFFF_FFFF {
        None
    } else {
        Some(ObjectFormat(raw as u16))
    }
}

fn prop_index(prop: ObjectPropCode) -> Option<usize> {
    SUPPORTED_PROPS.iter().position(|&p| p == prop.0)
}

fn prop_meta(prop: ObjectPropCode, read_only: bool) -> Option<(DataType, bool)> {
    match prop {
        ObjectPropCode::STORAGE_ID => Some((DataType::UINT32, false)),
        ObjectPropCode::OBJECT_FORMAT => Some((DataType::UINT16, false)),
        ObjectPropCode::PROTECTION_STATUS => Some((DataType::UINT16, false)),
        ObjectPropCode::OBJECT_SIZE => Some((DataType::UINT64, false)),
        ObjectPropCode::OBJECT_FILE_NAME => Some((DataType::STRING, !read_only)),
        ObjectPropCode::DATE_MODIFIED => Some((DataType::STRING, false)),
        ObjectPropCode::PARENT_OBJECT => Some((DataType::UINT32, false)),
        ObjectPropCode::PERSISTENT_UNIQUE_OBJECT_ID => Some((DataType::UINT128, false)),
        ObjectPropCode::NAME => Some((DataType::STRING, !read_only)),
        _ => None,
    }
}

fn persistent_uid(storage_id: u32, handle: u32) -> u128 {
    ((storage_id as u128) << 64) | (handle as u128)
}

fn prop_list_item_len(prop: ObjectPropCode, entry: &ObjectEntry) -> usize {
    // 4 (Handle) + 2 (PropCode) + 2 (DataType) + value
    8 + match prop {
        ObjectPropCode::STORAGE_ID | ObjectPropCode::PARENT_OBJECT => 4,
        ObjectPropCode::OBJECT_FORMAT | ObjectPropCode::PROTECTION_STATUS => 2,
        ObjectPropCode::OBJECT_SIZE => 8,
        ObjectPropCode::PERSISTENT_UNIQUE_OBJECT_ID => 16,
        ObjectPropCode::OBJECT_FILE_NAME | ObjectPropCode::NAME => ptp_str_encoded_len(&entry.filename),
        ObjectPropCode::DATE_MODIFIED => ptp_str_encoded_len(&entry.date_modified),
        _ => 0,
    }
}

fn write_prop_value_dataset(
    w: &mut super::super::DatasetWriter<'_>,
    prop: ObjectPropCode,
    handle: u32,
    entry: &ObjectEntry,
) -> Result<(), WireError> {
    match prop {
        ObjectPropCode::STORAGE_ID => w.write_u32(entry.storage_id),
        ObjectPropCode::OBJECT_FORMAT => w.write_u16(entry.format.0),
        ObjectPropCode::PROTECTION_STATUS => w.write_u16(entry.protection as u16),
        ObjectPropCode::OBJECT_SIZE => w.write_u64(entry.size),
        ObjectPropCode::OBJECT_FILE_NAME | ObjectPropCode::NAME => w.write_ptp_str(&entry.filename),
        ObjectPropCode::DATE_MODIFIED => w.write_ptp_str(&entry.date_modified),
        ObjectPropCode::PARENT_OBJECT => w.write_u32(entry.parent),
        ObjectPropCode::PERSISTENT_UNIQUE_OBJECT_ID => w.write_u128(persistent_uid(entry.storage_id, handle)),
        _ => Ok(()),
    }
}

async fn stream_prop_list_entry<E: EndpointIn>(
    writer: &mut DataWriter<'_, '_, E>,
    handle: u32,
    entry: &ObjectEntry,
    props: &[u16],
) -> Result<(), TransportError> {
    for &p in props {
        let prop = ObjectPropCode(p);
        let (dt, _) = prop_meta(prop, true).expect("prop from SUPPORTED_PROPS");
        write_u32(writer, handle).await?;
        write_u16(writer, prop.0).await?;
        write_u16(writer, dt.0).await?;
        match prop {
            ObjectPropCode::STORAGE_ID => write_u32(writer, entry.storage_id).await?,
            ObjectPropCode::OBJECT_FORMAT => write_u16(writer, entry.format.0).await?,
            ObjectPropCode::PROTECTION_STATUS => write_u16(writer, entry.protection as u16).await?,
            ObjectPropCode::OBJECT_SIZE => write_u64(writer, entry.size).await?,
            ObjectPropCode::OBJECT_FILE_NAME | ObjectPropCode::NAME => write_ptp_str(writer, &entry.filename).await?,
            ObjectPropCode::DATE_MODIFIED => write_ptp_str(writer, &entry.date_modified).await?,
            ObjectPropCode::PARENT_OBJECT => write_u32(writer, entry.parent).await?,
            ObjectPropCode::PERSISTENT_UNIQUE_OBJECT_ID => {
                writer
                    .write_all(&persistent_uid(entry.storage_id, handle).to_le_bytes())
                    .await?
            }
            _ => {}
        }
    }
    Ok(())
}

async fn write_u16<E: EndpointIn>(writer: &mut DataWriter<'_, '_, E>, v: u16) -> Result<(), TransportError> {
    writer.write_all(&v.to_le_bytes()).await
}

async fn write_u32<E: EndpointIn>(writer: &mut DataWriter<'_, '_, E>, v: u32) -> Result<(), TransportError> {
    writer.write_all(&v.to_le_bytes()).await
}

async fn write_u64<E: EndpointIn>(writer: &mut DataWriter<'_, '_, E>, v: u64) -> Result<(), TransportError> {
    writer.write_all(&v.to_le_bytes()).await
}

async fn write_u16_array<E: EndpointIn>(
    writer: &mut DataWriter<'_, '_, E>,
    values: &[u16],
) -> Result<(), TransportError> {
    write_u32(writer, values.len() as u32).await?;
    for &v in values {
        write_u16(writer, v).await?;
    }
    Ok(())
}

async fn write_ptp_str<E: EndpointIn>(writer: &mut DataWriter<'_, '_, E>, s: &str) -> Result<(), TransportError> {
    if s.is_empty() {
        return writer.write_all(&[0]).await;
    }
    let unit_count = s.encode_utf16().take(254).count();
    writer.write_all(&[(unit_count + 1) as u8]).await?;
    for unit in s.encode_utf16().take(254) {
        write_u16(writer, unit).await?;
    }
    write_u16(writer, 0).await
}
