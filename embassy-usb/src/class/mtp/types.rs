//! MTP / PTP (PIMA 15740) wire protocol constants, codes, containers, and datasets.

use heapless::String;

/// USB class code for Still Image Capture (used by PTP and MTP).
pub const USB_CLASS_IMAGE: u8 = 0x06;
/// USB subclass code for Still Image Capture.
pub const IMAGE_SUBCLASS_STILL_IMAGE: u8 = 0x01;
/// USB protocol code for PIMA 15740 (PTP/MTP).
pub const IMAGE_PROTOCOL_PIMA_15740: u8 = 0x01;

// Class-specific EP0 control requests (USB Still Image Capture §5).
/// Cancel an ongoing MTP/PTP transaction (Host-to-Device, 6 bytes).
pub const REQ_CANCEL: u8 = 0x64;
/// Retrieve extended event data (Device-to-Host).
pub const REQ_GET_EXTENDED_EVENT_DATA: u8 = 0x65;
/// Reset the MTP/PTP device state (Host-to-Device, 0 bytes).
pub const REQ_DEVICE_RESET: u8 = 0x66;
/// Query the current device status (Device-to-Host).
pub const REQ_GET_DEVICE_STATUS: u8 = 0x67;

/// Cancellation code expected in a [`REQ_CANCEL`] payload.
pub const CANCEL_CODE: u16 = 0x4001;

/// Size in bytes of every MTP bulk/interrupt container header.
pub const CONTAINER_HEADER_LEN: usize = 12;
/// Maximum size in bytes of an MTP Command or Response container (header + 5 params).
pub const MAX_COMMAND_CONTAINER_LEN: usize = CONTAINER_HEADER_LEN + 5 * 4;
/// Maximum size in bytes of an MTP Event container (header + 3 params).
pub const MAX_EVENT_CONTAINER_LEN: usize = CONTAINER_HEADER_LEN + 3 * 4;
/// Sentinel container length indicating a streaming Data container of >4 GiB or unknown size.
pub const UNKNOWN_CONTAINER_LEN: u32 = 0xFFFF_FFFF;

/// Standard PTP version (`1.00`, encoded as `100`).
pub const PTP_STANDARD_VERSION: u16 = 100;
/// MTP Vendor Extension ID (Microsoft MTP = `6`).
pub const MTP_VENDOR_EXTENSION_ID: u32 = 6;
/// MTP version (`1.00`, encoded as `100`).
pub const MTP_VERSION: u16 = 100;
/// Default MTP extensions string reported in `DeviceInfo`.
pub const MTP_EXTENSIONS_MICROSOFT: &str = "microsoft.com: 1.0; ";

/// Special storage ID matching all storages.
pub const STORAGE_ALL: u32 = 0xFFFF_FFFF;
/// Special parent object handle matching root-level objects only.
pub const HANDLE_ROOT: u32 = 0xFFFF_FFFF;
/// Special object handle matching all objects (flat traversal).
pub const HANDLE_ALL: u32 = 0x0000_0000;
/// Special format code matching all object formats.
pub const FORMAT_ALL: u16 = 0x0000;
/// Special property code matching all object properties.
pub const PROP_ALL: u32 = 0xFFFF_FFFF;
/// Special depth matching the entire subtree in `GetObjectPropList`.
pub const DEPTH_ALL: u32 = 0xFFFF_FFFF;

macro_rules! u16_code_type {
    ($(#[$meta:meta])* $name:ident) => {
        $(#[$meta])*
        #[derive(Copy, Clone, Eq, PartialEq, Ord, PartialOrd, Hash, Debug)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[repr(transparent)]
        pub struct $name(pub u16);

        impl From<u16> for $name {
            #[inline]
            fn from(v: u16) -> Self {
                Self(v)
            }
        }

        impl From<$name> for u16 {
            #[inline]
            fn from(v: $name) -> Self {
                v.0
            }
        }
    };
}

u16_code_type!(
    /// MTP/PTP Operation Code (`wCode` in a Command container).
    OperationCode
);

impl OperationCode {
    /// Retrieve device capabilities, supported operations, and identification strings.
    pub const GET_DEVICE_INFO: Self = Self(0x1001);
    /// Open an MTP/PTP session.
    pub const OPEN_SESSION: Self = Self(0x1002);
    /// Close the active session.
    pub const CLOSE_SESSION: Self = Self(0x1003);
    /// Retrieve the list of valid Storage IDs.
    pub const GET_STORAGE_IDS: Self = Self(0x1004);
    /// Retrieve metadata for a storage volume.
    pub const GET_STORAGE_INFO: Self = Self(0x1005);
    /// Count objects matching a storage/format/parent filter.
    pub const GET_NUM_OBJECTS: Self = Self(0x1006);
    /// List object handles matching a storage/format/parent filter.
    pub const GET_OBJECT_HANDLES: Self = Self(0x1007);
    /// Retrieve metadata for a single object.
    pub const GET_OBJECT_INFO: Self = Self(0x1008);
    /// Read the full contents of an object.
    pub const GET_OBJECT: Self = Self(0x1009);
    /// Read an object thumbnail.
    pub const GET_THUMB: Self = Self(0x100A);
    /// Delete an object (or folder).
    pub const DELETE_OBJECT: Self = Self(0x100B);
    /// Send metadata for a new object prior to [`Self::SEND_OBJECT`].
    pub const SEND_OBJECT_INFO: Self = Self(0x100C);
    /// Upload the payload of the object announced by [`Self::SEND_OBJECT_INFO`].
    pub const SEND_OBJECT: Self = Self(0x100D);
    /// Format a storage volume.
    pub const FORMAT_STORE: Self = Self(0x100F);
    /// Reset the device.
    pub const RESET_DEVICE: Self = Self(0x1010);
    /// Move an object to a new storage/parent.
    pub const MOVE_OBJECT: Self = Self(0x1019);
    /// Copy an object to a new storage/parent.
    pub const COPY_OBJECT: Self = Self(0x101A);
    /// Read a byte range of an object.
    pub const GET_PARTIAL_OBJECT: Self = Self(0x101B);
    /// List object property codes supported for a given format.
    pub const GET_OBJECT_PROPS_SUPPORTED: Self = Self(0x9801);
    /// Describe an object property (type, mutability, default).
    pub const GET_OBJECT_PROP_DESC: Self = Self(0x9802);
    /// Read a single property value for an object.
    pub const GET_OBJECT_PROP_VALUE: Self = Self(0x9803);
    /// Update a single property value for an object.
    pub const SET_OBJECT_PROP_VALUE: Self = Self(0x9804);
    /// Bulk-query object properties for an object or subtree.
    pub const GET_OBJECT_PROP_LIST: Self = Self(0x9805);
    /// Retrieve object reference handles.
    pub const GET_OBJECT_REFERENCES: Self = Self(0x9810);
}

u16_code_type!(
    /// MTP/PTP Response Code (`wCode` in a Response container).
    ResponseCode
);

impl ResponseCode {
    /// Undefined error.
    pub const UNDEFINED: Self = Self(0x2000);
    /// Operation completed successfully.
    pub const OK: Self = Self(0x2001);
    /// Unspecified general failure.
    pub const GENERAL_ERROR: Self = Self(0x2002);
    /// Operation requires an open session.
    pub const SESSION_NOT_OPEN: Self = Self(0x2003);
    /// Invalid transaction ID.
    pub const INVALID_TRANSACTION_ID: Self = Self(0x2004);
    /// Operation is not supported by this device.
    pub const OPERATION_NOT_SUPPORTED: Self = Self(0x2005);
    /// One or more command parameters are not supported.
    pub const PARAMETER_NOT_SUPPORTED: Self = Self(0x2006);
    /// Data transfer terminated early.
    pub const INCOMPLETE_TRANSFER: Self = Self(0x2007);
    /// Unknown or unavailable storage ID.
    pub const INVALID_STORAGE_ID: Self = Self(0x2008);
    /// Unknown object handle.
    pub const INVALID_OBJECT_HANDLE: Self = Self(0x2009);
    /// Device property not supported.
    pub const DEVICE_PROP_NOT_SUPPORTED: Self = Self(0x200A);
    /// Unsupported object format code.
    pub const INVALID_OBJECT_FORMAT_CODE: Self = Self(0x200B);
    /// Storage volume is full.
    pub const STORE_FULL: Self = Self(0x200C);
    /// Object is write-protected.
    pub const OBJECT_WRITE_PROTECTED: Self = Self(0x200D);
    /// Storage volume is read-only.
    pub const STORE_READ_ONLY: Self = Self(0x200E);
    /// Access denied.
    pub const ACCESS_DENIED: Self = Self(0x200F);
    /// Object has no thumbnail.
    pub const NO_THUMBNAIL_PRESENT: Self = Self(0x2010);
    /// Only some objects in a subtree could be deleted.
    pub const PARTIAL_DELETION: Self = Self(0x2012);
    /// Storage volume is not currently available.
    pub const STORE_NOT_AVAILABLE: Self = Self(0x2013);
    /// Filtering by object format is not supported.
    pub const SPECIFICATION_BY_FORMAT_UNSUPPORTED: Self = Self(0x2014);
    /// `SendObject` called without a prior valid `SendObjectInfo`.
    pub const NO_VALID_OBJECT_INFO: Self = Self(0x2015);
    /// Device is busy processing another operation.
    pub const DEVICE_BUSY: Self = Self(0x2019);
    /// Parent object handle is invalid or not an association.
    pub const INVALID_PARENT_OBJECT: Self = Self(0x201A);
    /// Invalid parameter value.
    pub const INVALID_PARAMETER: Self = Self(0x201D);
    /// Session is already open.
    pub const SESSION_ALREADY_OPEN: Self = Self(0x201E);
    /// Transaction was cancelled by the host.
    pub const TRANSACTION_CANCELLED: Self = Self(0x201F);
    /// Destination specification unsupported.
    pub const SPECIFICATION_OF_DESTINATION_UNSUPPORTED: Self = Self(0x2020);
    /// Unknown object property code.
    pub const INVALID_OBJECT_PROP_CODE: Self = Self(0xA801);
    /// Invalid object property format.
    pub const INVALID_OBJECT_PROP_FORMAT: Self = Self(0xA802);
    /// Invalid object property value.
    pub const INVALID_OBJECT_PROP_VALUE: Self = Self(0xA803);
    /// Invalid dataset encoding.
    pub const INVALID_DATASET: Self = Self(0xA806);
    /// Filtering by property group is not supported.
    pub const SPECIFICATION_BY_GROUP_UNSUPPORTED: Self = Self(0xA807);
    /// Requested traversal depth is not supported.
    pub const SPECIFICATION_BY_DEPTH_UNSUPPORTED: Self = Self(0xA808);
    /// Object exceeds maximum supported size.
    pub const OBJECT_TOO_LARGE: Self = Self(0xA809);
    /// Object property not supported for this object format.
    pub const OBJECT_PROP_NOT_SUPPORTED: Self = Self(0xA80A);
}

u16_code_type!(
    /// MTP/PTP Event Code (`wCode` in an Event container on Interrupt-IN).
    EventCode
);

impl EventCode {
    /// Transaction cancellation acknowledgement.
    pub const CANCEL_TRANSACTION: Self = Self(0x4001);
    /// A new object was added (`param0 = object_handle`).
    pub const OBJECT_ADDED: Self = Self(0x4002);
    /// An object was removed (`param0 = object_handle`).
    pub const OBJECT_REMOVED: Self = Self(0x4003);
    /// A storage volume was mounted (`param0 = storage_id`).
    pub const STORE_ADDED: Self = Self(0x4004);
    /// A storage volume was unmounted (`param0 = storage_id`).
    pub const STORE_REMOVED: Self = Self(0x4005);
    /// Device property changed (`param0 = prop_code`).
    pub const DEVICE_PROP_CHANGED: Self = Self(0x4006);
    /// Object metadata changed (`param0 = object_handle`).
    pub const OBJECT_INFO_CHANGED: Self = Self(0x4007);
    /// Device info changed.
    pub const DEVICE_INFO_CHANGED: Self = Self(0x4008);
    /// Storage info changed (`param0 = storage_id`).
    pub const STORAGE_INFO_CHANGED: Self = Self(0x400C);
    /// Object property changed (`param0 = object_handle`, `param1 = prop_code`).
    pub const OBJECT_PROP_CHANGED: Self = Self(0xC801);
}

u16_code_type!(
    /// MTP/PTP Object Format Code.
    ObjectFormat
);

impl ObjectFormat {
    /// Undefined / generic binary file.
    pub const UNDEFINED: Self = Self(0x3000);
    /// Directory / folder association.
    pub const ASSOCIATION: Self = Self(0x3001);
    /// Plain text file (`.txt`).
    pub const TEXT: Self = Self(0x3004);
    /// HTML document (`.html`).
    pub const HTML: Self = Self(0x3005);
    /// WAV audio (`.wav`).
    pub const WAV: Self = Self(0x3008);
    /// MP3 audio (`.mp3`).
    pub const MP3: Self = Self(0x3009);
    /// AVI video (`.avi`).
    pub const AVI: Self = Self(0x300A);
    /// MPEG video (`.mpg`).
    pub const MPEG: Self = Self(0x300B);
    /// JPEG / EXIF image (`.jpg`, `.jpeg`).
    pub const EXIF_JPEG: Self = Self(0x3801);
    /// GIF image (`.gif`).
    pub const GIF: Self = Self(0x3807);
    /// PNG image (`.png`).
    pub const PNG: Self = Self(0x380B);
    /// TIFF image (`.tif`, `.tiff`).
    pub const TIFF: Self = Self(0x380D);
    /// WMA audio (`.wma`).
    pub const WMA: Self = Self(0xB901);
    /// AAC audio (`.aac`).
    pub const AAC: Self = Self(0xB903);
    /// MP4 video (`.mp4`).
    pub const MP4: Self = Self(0xB982);
    /// XML document (`.xml`).
    pub const XML_DOCUMENT: Self = Self(0xBA82);
    /// Firmware image.
    pub const FIRMWARE: Self = Self(0xBB01);

    /// Infers a common [`ObjectFormat`] from a filename extension (case-insensitive).
    pub fn from_filename(name: &str) -> Self {
        let Some((_, ext)) = name.rsplit_once('.') else {
            return Self::UNDEFINED;
        };
        if ext.eq_ignore_ascii_case("txt") || ext.eq_ignore_ascii_case("log") || ext.eq_ignore_ascii_case("csv") {
            Self::TEXT
        } else if ext.eq_ignore_ascii_case("html") || ext.eq_ignore_ascii_case("htm") {
            Self::HTML
        } else if ext.eq_ignore_ascii_case("xml") {
            Self::XML_DOCUMENT
        } else if ext.eq_ignore_ascii_case("wav") {
            Self::WAV
        } else if ext.eq_ignore_ascii_case("mp3") {
            Self::MP3
        } else if ext.eq_ignore_ascii_case("aac") {
            Self::AAC
        } else if ext.eq_ignore_ascii_case("wma") {
            Self::WMA
        } else if ext.eq_ignore_ascii_case("jpg") || ext.eq_ignore_ascii_case("jpeg") {
            Self::EXIF_JPEG
        } else if ext.eq_ignore_ascii_case("png") {
            Self::PNG
        } else if ext.eq_ignore_ascii_case("gif") {
            Self::GIF
        } else if ext.eq_ignore_ascii_case("tif") || ext.eq_ignore_ascii_case("tiff") {
            Self::TIFF
        } else if ext.eq_ignore_ascii_case("mp4") {
            Self::MP4
        } else if ext.eq_ignore_ascii_case("avi") {
            Self::AVI
        } else if ext.eq_ignore_ascii_case("mpg") || ext.eq_ignore_ascii_case("mpeg") {
            Self::MPEG
        } else {
            Self::UNDEFINED
        }
    }
}

u16_code_type!(
    /// MTP Object Property Code.
    ObjectPropCode
);

impl ObjectPropCode {
    /// Storage ID (`UINT32`).
    pub const STORAGE_ID: Self = Self(0xDC01);
    /// Object format (`UINT16`).
    pub const OBJECT_FORMAT: Self = Self(0xDC02);
    /// Protection status (`UINT16`).
    pub const PROTECTION_STATUS: Self = Self(0xDC03);
    /// Object size in bytes (`UINT64`).
    pub const OBJECT_SIZE: Self = Self(0xDC04);
    /// Object file name (`STRING`).
    pub const OBJECT_FILE_NAME: Self = Self(0xDC07);
    /// Date created (`STRING`, ISO-8601 `YYYYMMDDTHHMMSS`).
    pub const DATE_CREATED: Self = Self(0xDC08);
    /// Date modified (`STRING`, ISO-8601 `YYYYMMDDTHHMMSS`).
    pub const DATE_MODIFIED: Self = Self(0xDC09);
    /// Parent object handle (`UINT32`, `0` for root).
    pub const PARENT_OBJECT: Self = Self(0xDC0B);
    /// Persistent unique object identifier (`UINT128`).
    pub const PERSISTENT_UNIQUE_OBJECT_ID: Self = Self(0xDC41);
    /// Display name (`STRING`).
    pub const NAME: Self = Self(0xDC44);
}

u16_code_type!(
    /// MTP/PTP property Data Type code.
    DataType
);

impl DataType {
    /// Undefined type.
    pub const UNDEFINED: Self = Self(0x0000);
    /// Signed 8-bit integer.
    pub const INT8: Self = Self(0x0001);
    /// Unsigned 8-bit integer.
    pub const UINT8: Self = Self(0x0002);
    /// Signed 16-bit integer.
    pub const INT16: Self = Self(0x0003);
    /// Unsigned 16-bit integer.
    pub const UINT16: Self = Self(0x0004);
    /// Signed 32-bit integer.
    pub const INT32: Self = Self(0x0005);
    /// Unsigned 32-bit integer.
    pub const UINT32: Self = Self(0x0006);
    /// Signed 64-bit integer.
    pub const INT64: Self = Self(0x0007);
    /// Unsigned 64-bit integer.
    pub const UINT64: Self = Self(0x0008);
    /// Signed 128-bit integer.
    pub const INT128: Self = Self(0x0009);
    /// Unsigned 128-bit integer.
    pub const UINT128: Self = Self(0x000A);
    /// Array of unsigned 16-bit integers.
    pub const AUINT16: Self = Self(0x4004);
    /// Array of unsigned 32-bit integers.
    pub const AUINT32: Self = Self(0x4006);
    /// PTP UTF-16LE length-prefixed string.
    pub const STRING: Self = Self(0xFFFF);
}

u16_code_type!(
    /// Device status code returned by [`REQ_GET_DEVICE_STATUS`].
    DeviceStatusCode
);

impl DeviceStatusCode {
    /// Device is ready for commands.
    pub const OK: Self = Self(0x2001);
    /// Device is busy processing a request or reset.
    pub const DEVICE_BUSY: Self = Self(0x2019);
    /// Last transaction was cancelled.
    pub const TRANSACTION_CANCELLED: Self = Self(0x201F);
}

/// MTP container type (PIMA 15740 §7.1).
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u16)]
pub enum ContainerType {
    /// Command block (Host -> Device on Bulk-OUT).
    Command = 1,
    /// Data block (either direction on Bulk-IN/OUT).
    Data = 2,
    /// Response block (Device -> Host on Bulk-IN).
    Response = 3,
    /// Event block (Device -> Host on Interrupt-IN).
    Event = 4,
}

impl TryFrom<u16> for ContainerType {
    type Error = u16;

    fn try_from(value: u16) -> Result<Self, u16> {
        match value {
            1 => Ok(Self::Command),
            2 => Ok(Self::Data),
            3 => Ok(Self::Response),
            4 => Ok(Self::Event),
            other => Err(other),
        }
    }
}

/// Decoded 12-byte MTP/PTP container header.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ContainerHeader {
    /// Total container length in bytes, including this 12-byte header (or [`UNKNOWN_CONTAINER_LEN`]).
    pub length: u32,
    /// Container type.
    pub container_type: ContainerType,
    /// Operation, Response, or Event code.
    pub code: u16,
    /// Host-assigned transaction ID.
    pub transaction_id: u32,
}

impl ContainerHeader {
    /// Parses a 12-byte container header from `buf`.
    pub fn parse(buf: &[u8]) -> Result<Self, WireError> {
        if buf.len() < CONTAINER_HEADER_LEN {
            return Err(WireError::Truncated);
        }
        let length = u32::from_le_bytes([buf[0], buf[1], buf[2], buf[3]]);
        let raw_type = u16::from_le_bytes([buf[4], buf[5]]);
        let container_type =
            ContainerType::try_from(raw_type).map_err(|_| WireError::InvalidContainerType(raw_type))?;
        let code = u16::from_le_bytes([buf[6], buf[7]]);
        let transaction_id = u32::from_le_bytes([buf[8], buf[9], buf[10], buf[11]]);
        if length != UNKNOWN_CONTAINER_LEN && (length as usize) < CONTAINER_HEADER_LEN {
            return Err(WireError::InvalidLength(length));
        }
        Ok(Self {
            length,
            container_type,
            code,
            transaction_id,
        })
    }

    /// Encodes this header into the first 12 bytes of `out`.
    pub fn encode(&self, out: &mut [u8]) -> Result<(), WireError> {
        if out.len() < CONTAINER_HEADER_LEN {
            return Err(WireError::BufferTooSmall);
        }
        out[0..4].copy_from_slice(&self.length.to_le_bytes());
        out[4..6].copy_from_slice(&(self.container_type as u16).to_le_bytes());
        out[6..8].copy_from_slice(&self.code.to_le_bytes());
        out[8..12].copy_from_slice(&self.transaction_id.to_le_bytes());
        Ok(())
    }
}

/// Wire encoding/decoding error.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WireError {
    /// Input slice ended unexpectedly.
    Truncated,
    /// Output buffer was too small.
    BufferTooSmall,
    /// Container type was not 1..=4.
    InvalidContainerType(u16),
    /// Container length was invalid (e.g. `< 12` or unaligned parameters).
    InvalidLength(u32),
    /// Container type did not match what was expected.
    UnexpectedContainerType(ContainerType),
    /// Invalid UTF-16 string encoding in dataset.
    InvalidString,
}

/// Decoded MTP Command container.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CommandBlock {
    /// Operation code.
    pub operation: OperationCode,
    /// Transaction ID assigned by the host.
    pub transaction_id: u32,
    /// Up to 5 command parameters (`0..param_count`).
    pub params: [u32; 5],
    /// Number of valid parameters in [`Self::params`] (`0..=5`).
    pub param_count: u8,
}

impl CommandBlock {
    /// Parses a Command container from `buf`.
    pub fn parse(buf: &[u8]) -> Result<Self, WireError> {
        let hdr = ContainerHeader::parse(buf)?;
        if hdr.container_type != ContainerType::Command {
            return Err(WireError::UnexpectedContainerType(hdr.container_type));
        }
        let len = hdr.length as usize;
        if len > MAX_COMMAND_CONTAINER_LEN || !(len - CONTAINER_HEADER_LEN).is_multiple_of(4) {
            return Err(WireError::InvalidLength(hdr.length));
        }
        if buf.len() < len {
            return Err(WireError::Truncated);
        }
        let param_count = (len - CONTAINER_HEADER_LEN) / 4;
        let mut params = [0u32; 5];
        for (i, slot) in params.iter_mut().take(param_count).enumerate() {
            let off = CONTAINER_HEADER_LEN + i * 4;
            *slot = u32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]]);
        }
        Ok(Self {
            operation: OperationCode(hdr.code),
            transaction_id: hdr.transaction_id,
            params,
            param_count: param_count as u8,
        })
    }

    /// Returns parameter `idx`, or `0` if omitted by the host (per PIMA 15740 default parameter rules).
    #[inline]
    pub const fn param(&self, idx: usize) -> u32 {
        if idx < self.param_count as usize && idx < 5 {
            self.params[idx]
        } else {
            0
        }
    }

    /// Returns the populated parameter slice.
    #[inline]
    pub fn params(&self) -> &[u32] {
        &self.params[..self.param_count as usize]
    }
}

/// MTP Response container to send back to the host.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ResponseBlock {
    /// Response status code.
    pub code: ResponseCode,
    /// Transaction ID copied from the matching [`CommandBlock`].
    pub transaction_id: u32,
    /// Up to 5 response parameters.
    pub params: [u32; 5],
    /// Number of valid parameters in [`Self::params`] (`0..=5`).
    pub param_count: u8,
}

impl ResponseBlock {
    /// Creates a response with no parameters.
    pub const fn new(code: ResponseCode, transaction_id: u32) -> Self {
        Self {
            code,
            transaction_id,
            params: [0; 5],
            param_count: 0,
        }
    }

    /// Creates an `OK` response with no parameters.
    pub const fn ok(transaction_id: u32) -> Self {
        Self::new(ResponseCode::OK, transaction_id)
    }

    /// Creates a response with up to 5 parameters (extra parameters beyond 5 are truncated).
    pub fn with_params(code: ResponseCode, transaction_id: u32, params: &[u32]) -> Self {
        let mut out = Self::new(code, transaction_id);
        let count = params.len().min(5);
        out.params[..count].copy_from_slice(&params[..count]);
        out.param_count = count as u8;
        out
    }

    /// Encodes this response into `out` and returns the encoded byte length.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize, WireError> {
        let count = (self.param_count as usize).min(5);
        let total = CONTAINER_HEADER_LEN + count * 4;
        if out.len() < total {
            return Err(WireError::BufferTooSmall);
        }
        ContainerHeader {
            length: total as u32,
            container_type: ContainerType::Response,
            code: self.code.0,
            transaction_id: self.transaction_id,
        }
        .encode(out)?;
        for (i, &p) in self.params[..count].iter().enumerate() {
            let off = CONTAINER_HEADER_LEN + i * 4;
            out[off..off + 4].copy_from_slice(&p.to_le_bytes());
        }
        Ok(total)
    }
}

/// MTP asynchronous Event container sent on the Interrupt-IN endpoint.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct EventBlock {
    /// Event code.
    pub code: EventCode,
    /// Session-wide (`0xFFFF_FFFF`) or transaction-specific transaction ID.
    pub transaction_id: u32,
    /// Up to 3 event parameters.
    pub params: [u32; 3],
    /// Number of valid parameters in [`Self::params`] (`0..=3`).
    pub param_count: u8,
}

impl EventBlock {
    /// Creates an event with `transaction_id = 0xFFFF_FFFF` (all sessions) and up to 3 parameters.
    pub fn new(code: EventCode, params: &[u32]) -> Self {
        Self::with_transaction(code, 0xFFFF_FFFF, params)
    }

    /// Creates an event with an explicit transaction ID and up to 3 parameters.
    pub fn with_transaction(code: EventCode, transaction_id: u32, params: &[u32]) -> Self {
        let mut out_params = [0u32; 3];
        let count = params.len().min(3);
        out_params[..count].copy_from_slice(&params[..count]);
        Self {
            code,
            transaction_id,
            params: out_params,
            param_count: count as u8,
        }
    }

    /// Encodes this event into `out` and returns the encoded byte length.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize, WireError> {
        let count = (self.param_count as usize).min(3);
        let total = CONTAINER_HEADER_LEN + count * 4;
        if out.len() < total {
            return Err(WireError::BufferTooSmall);
        }
        ContainerHeader {
            length: total as u32,
            container_type: ContainerType::Event,
            code: self.code.0,
            transaction_id: self.transaction_id,
        }
        .encode(out)?;
        for (i, &p) in self.params[..count].iter().enumerate() {
            let off = CONTAINER_HEADER_LEN + i * 4;
            out[off..off + 4].copy_from_slice(&p.to_le_bytes());
        }
        Ok(total)
    }
}

/// Storage physical medium type (PIMA 15740 Table 12).
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u16)]
pub enum StorageType {
    /// Undefined storage type.
    Undefined = 0x0000,
    /// Fixed read-only memory.
    FixedRom = 0x0001,
    /// Removable read-only memory.
    RemovableRom = 0x0002,
    /// Fixed read-write memory (e.g. internal flash).
    FixedRam = 0x0003,
    /// Removable read-write memory (e.g. SD card).
    RemovableRam = 0x0004,
}

/// Filesystem hierarchy type reported in `StorageInfo` (PIMA 15740 Table 13).
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u16)]
pub enum FilesystemType {
    /// Undefined filesystem.
    Undefined = 0x0000,
    /// Flat (non-hierarchical) filesystem.
    GenericFlat = 0x0001,
    /// Hierarchical directory tree.
    GenericHierarchical = 0x0002,
    /// DCF (Design rule for Camera File system).
    Dcf = 0x0003,
}

/// Access capability of a storage volume (PIMA 15740 Table 14).
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u16)]
pub enum AccessCapability {
    /// Read and write access.
    ReadWrite = 0x0000,
    /// Read-only; deletion is not allowed.
    ReadOnlyWithoutDelete = 0x0001,
    /// Read-only, except object deletion is allowed.
    ReadOnlyWithDelete = 0x0002,
}

/// Protection status of an object.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u16)]
pub enum ProtectionStatus {
    /// Object can be modified or deleted.
    NoProtection = 0x0000,
    /// Object is read-only.
    ReadOnly = 0x0001,
}

/// Association type for folders/collections (PIMA 15740 §5.3).
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u16)]
pub enum AssociationType {
    /// Not an association (regular file).
    Undefined = 0x0000,
    /// Generic folder / directory.
    GenericFolder = 0x0001,
}

/// Metadata describing a storage volume (`GetStorageInfo`).
#[derive(Clone, Eq, PartialEq, Debug)]
pub struct StorageInfo {
    /// Physical storage type.
    pub storage_type: StorageType,
    /// Filesystem organization.
    pub filesystem_type: FilesystemType,
    /// Read/write capability.
    pub access_capability: AccessCapability,
    /// Total capacity in bytes.
    pub max_capacity: u64,
    /// Available free space in bytes.
    pub free_space_bytes: u64,
    /// Remaining object slots, or `0xFFFF_FFFF` if unknown/unlimited.
    pub free_space_objects: u32,
    /// Human-readable storage description (e.g. `"Internal Storage"`).
    pub description: String<64>,
    /// Volume identifier string (e.g. `"EMBASSY"`).
    pub volume_id: String<32>,
}

impl Default for StorageInfo {
    fn default() -> Self {
        Self {
            storage_type: StorageType::FixedRam,
            filesystem_type: FilesystemType::GenericHierarchical,
            access_capability: AccessCapability::ReadWrite,
            max_capacity: 0,
            free_space_bytes: 0,
            free_space_objects: 0xFFFF_FFFF,
            description: String::new(),
            volume_id: String::new(),
        }
    }
}

/// Metadata describing a single MTP object (file or folder).
#[derive(Clone, Eq, PartialEq, Debug)]
pub struct ObjectEntry {
    /// Storage volume containing this object.
    pub storage_id: u32,
    /// Object format (use [`ObjectFormat::ASSOCIATION`] for folders).
    pub format: ObjectFormat,
    /// Protection status.
    pub protection: ProtectionStatus,
    /// Object size in bytes (`0` for folders).
    pub size: u64,
    /// Parent object handle (`0` for root).
    pub parent: u32,
    /// Association type ([`AssociationType::GenericFolder`] for folders, `Undefined` for files).
    pub association_type: AssociationType,
    /// Filename (without directory path).
    pub filename: String<128>,
    /// Optional creation timestamp in PTP ISO-8601 format (`YYYYMMDDTHHMMSS`).
    pub date_created: String<32>,
    /// Optional modification timestamp in PTP ISO-8601 format (`YYYYMMDDTHHMMSS`).
    pub date_modified: String<32>,
}

impl Default for ObjectEntry {
    fn default() -> Self {
        Self {
            storage_id: 0,
            format: ObjectFormat::UNDEFINED,
            protection: ProtectionStatus::NoProtection,
            size: 0,
            parent: 0,
            association_type: AssociationType::Undefined,
            filename: String::new(),
            date_created: String::new(),
            date_modified: String::new(),
        }
    }
}

impl ObjectEntry {
    /// Creates a file entry with format inferred from `filename`.
    pub fn file(storage_id: u32, parent: u32, filename: &str, size: u64) -> Self {
        let mut name = String::new();
        for c in filename.chars() {
            if name.push(c).is_err() {
                break;
            }
        }
        Self {
            storage_id,
            format: ObjectFormat::from_filename(filename),
            protection: ProtectionStatus::NoProtection,
            size,
            parent,
            association_type: AssociationType::Undefined,
            filename: name,
            date_created: String::new(),
            date_modified: String::new(),
        }
    }

    /// Creates a folder (association) entry.
    pub fn folder(storage_id: u32, parent: u32, name: &str) -> Self {
        let mut fname = String::new();
        for c in name.chars() {
            if fname.push(c).is_err() {
                break;
            }
        }
        Self {
            storage_id,
            format: ObjectFormat::ASSOCIATION,
            protection: ProtectionStatus::NoProtection,
            size: 0,
            parent,
            association_type: AssociationType::GenericFolder,
            filename: fname,
            date_created: String::new(),
            date_modified: String::new(),
        }
    }

    /// Returns `true` if this entry represents a folder/association.
    #[inline]
    pub fn is_folder(&self) -> bool {
        self.format == ObjectFormat::ASSOCIATION || self.association_type == AssociationType::GenericFolder
    }

    /// Encodes the `ObjectInfo` dataset into `out` and returns bytes written.
    pub fn encode_object_info(&self, out: &mut [u8]) -> Result<usize, WireError> {
        let mut w = DatasetWriter::new(out);
        let compressed_size = if self.size >= u32::MAX as u64 {
            u32::MAX
        } else {
            self.size as u32
        };
        w.write_u32(self.storage_id)?;
        w.write_u16(self.format.0)?;
        w.write_u16(self.protection as u16)?;
        w.write_u32(compressed_size)?;
        w.write_u16(0)?; // ThumbFormat
        w.write_u32(0)?; // ThumbCompressedSize
        w.write_u32(0)?; // ThumbPixWidth
        w.write_u32(0)?; // ThumbPixHeight
        w.write_u32(0)?; // ImagePixWidth
        w.write_u32(0)?; // ImagePixHeight
        w.write_u32(0)?; // ImageBitDepth
        w.write_u32(self.parent)?;
        w.write_u16(self.association_type as u16)?;
        w.write_u32(0)?; // AssociationDesc
        w.write_u32(0)?; // SequenceNumber
        w.write_ptp_str(&self.filename)?;
        w.write_ptp_str(&self.date_created)?;
        w.write_ptp_str(&self.date_modified)?;
        w.write_ptp_str("")?; // Keywords
        Ok(w.len())
    }

    /// Decodes an `ObjectInfo` dataset sent by the host in `SendObjectInfo`.
    pub fn decode_object_info(buf: &[u8]) -> Result<Self, WireError> {
        let mut r = DatasetReader::new(buf);
        let storage_id = r.read_u32()?;
        let format = ObjectFormat(r.read_u16()?);
        let raw_prot = r.read_u16()?;
        let protection = if raw_prot == ProtectionStatus::ReadOnly as u16 {
            ProtectionStatus::ReadOnly
        } else {
            ProtectionStatus::NoProtection
        };
        let size = r.read_u32()? as u64;
        let _thumb_format = r.read_u16()?;
        let _thumb_size = r.read_u32()?;
        let _thumb_w = r.read_u32()?;
        let _thumb_h = r.read_u32()?;
        let _img_w = r.read_u32()?;
        let _img_h = r.read_u32()?;
        let _img_bpp = r.read_u32()?;
        let parent = r.read_u32()?;
        let raw_assoc = r.read_u16()?;
        let association_type = if raw_assoc == AssociationType::GenericFolder as u16 {
            AssociationType::GenericFolder
        } else {
            AssociationType::Undefined
        };
        let _assoc_desc = r.read_u32()?;
        let _seq = r.read_u32()?;
        let filename: String<128> = r.read_ptp_str()?;
        let date_created: String<32> = r.read_ptp_str()?;
        let date_modified: String<32> = r.read_ptp_str()?;
        let _keywords: String<32> = r.read_ptp_str().unwrap_or_default();

        Ok(Self {
            storage_id,
            format,
            protection,
            size,
            parent,
            association_type,
            filename,
            date_created,
            date_modified,
        })
    }
}

/// Returns the encoded byte length of a PTP UTF-16LE length-prefixed string.
pub fn ptp_str_encoded_len(s: &str) -> usize {
    if s.is_empty() {
        1
    } else {
        let units = s.encode_utf16().take(254).count();
        1 + (units + 1) * 2
    }
}

/// Bounded little-endian dataset serializer for PIMA 15740 / MTP datasets.
pub struct DatasetWriter<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> DatasetWriter<'a> {
    /// Creates a new writer backed by `buf`.
    pub fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Number of bytes written so far.
    #[inline]
    pub fn len(&self) -> usize {
        self.pos
    }

    /// Returns `true` if no bytes have been written.
    #[inline]
    pub fn is_empty(&self) -> bool {
        self.pos == 0
    }

    /// Returns the written slice.
    #[inline]
    pub fn as_slice(&self) -> &[u8] {
        &self.buf[..self.pos]
    }

    fn alloc(&mut self, n: usize) -> Result<&mut [u8], WireError> {
        let end = self.pos.checked_add(n).ok_or(WireError::BufferTooSmall)?;
        if end > self.buf.len() {
            return Err(WireError::BufferTooSmall);
        }
        let start = self.pos;
        self.pos = end;
        Ok(&mut self.buf[start..end])
    }

    /// Writes a `u8`.
    pub fn write_u8(&mut self, v: u8) -> Result<(), WireError> {
        self.alloc(1)?[0] = v;
        Ok(())
    }

    /// Writes a little-endian `u16`.
    pub fn write_u16(&mut self, v: u16) -> Result<(), WireError> {
        self.alloc(2)?.copy_from_slice(&v.to_le_bytes());
        Ok(())
    }

    /// Writes a little-endian `u32`.
    pub fn write_u32(&mut self, v: u32) -> Result<(), WireError> {
        self.alloc(4)?.copy_from_slice(&v.to_le_bytes());
        Ok(())
    }

    /// Writes a little-endian `u64`.
    pub fn write_u64(&mut self, v: u64) -> Result<(), WireError> {
        self.alloc(8)?.copy_from_slice(&v.to_le_bytes());
        Ok(())
    }

    /// Writes a little-endian `u128`.
    pub fn write_u128(&mut self, v: u128) -> Result<(), WireError> {
        self.alloc(16)?.copy_from_slice(&v.to_le_bytes());
        Ok(())
    }

    /// Writes a PTP `AUINT16` array (`u32` count followed by `u16` elements).
    pub fn write_u16_array(&mut self, values: &[u16]) -> Result<(), WireError> {
        self.write_u32(values.len() as u32)?;
        for &v in values {
            self.write_u16(v)?;
        }
        Ok(())
    }

    /// Writes a PTP `AUINT32` array (`u32` count followed by `u32` elements).
    pub fn write_u32_array(&mut self, values: &[u32]) -> Result<(), WireError> {
        self.write_u32(values.len() as u32)?;
        for &v in values {
            self.write_u32(v)?;
        }
        Ok(())
    }

    /// Writes a PTP UTF-16LE length-prefixed string.
    ///
    /// Empty strings encode as a single `0x00` byte. Non-empty strings encode as
    /// `[num_chars: u8]` (including the trailing NUL) followed by `num_chars` little-endian
    /// UTF-16 code units, truncated to at most 254 UTF-16 code units plus NUL.
    pub fn write_ptp_str(&mut self, s: &str) -> Result<(), WireError> {
        if s.is_empty() {
            return self.write_u8(0);
        }
        let unit_count = s.encode_utf16().take(254).count();
        let total_chars = (unit_count + 1) as u8;
        let needed = 1 + (total_chars as usize) * 2;
        let dst = self.alloc(needed)?;
        dst[0] = total_chars;
        let mut idx = 1;
        for unit in s.encode_utf16().take(254) {
            dst[idx..idx + 2].copy_from_slice(&unit.to_le_bytes());
            idx += 2;
        }
        dst[idx] = 0;
        dst[idx + 1] = 0;
        Ok(())
    }
}

/// Bounded little-endian dataset parser for PIMA 15740 / MTP datasets.
pub struct DatasetReader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> DatasetReader<'a> {
    /// Creates a new reader over `buf`.
    pub fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Remaining unread bytes.
    #[inline]
    pub fn remaining(&self) -> usize {
        self.buf.len().saturating_sub(self.pos)
    }

    fn take(&mut self, n: usize) -> Result<&'a [u8], WireError> {
        let end = self.pos.checked_add(n).ok_or(WireError::Truncated)?;
        if end > self.buf.len() {
            return Err(WireError::Truncated);
        }
        let s = &self.buf[self.pos..end];
        self.pos = end;
        Ok(s)
    }

    /// Reads a `u8`.
    pub fn read_u8(&mut self) -> Result<u8, WireError> {
        Ok(self.take(1)?[0])
    }

    /// Reads a little-endian `u16`.
    pub fn read_u16(&mut self) -> Result<u16, WireError> {
        let b = self.take(2)?;
        Ok(u16::from_le_bytes([b[0], b[1]]))
    }

    /// Reads a little-endian `u32`.
    pub fn read_u32(&mut self) -> Result<u32, WireError> {
        let b = self.take(4)?;
        Ok(u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    }

    /// Reads a PTP UTF-16LE length-prefixed string into a `heapless::String<N>`.
    ///
    /// Safely validates bounds against the remaining buffer before reading.
    pub fn read_ptp_str<const N: usize>(&mut self) -> Result<String<N>, WireError> {
        let count = self.read_u8()? as usize;
        if count == 0 {
            return Ok(String::new());
        }
        let raw = self.take(count * 2)?;
        let mut out = String::new();
        let iter = raw
            .as_chunks::<2>()
            .0
            .iter()
            .map(|c| u16::from_le_bytes(*c))
            .take_while(|&u| u != 0);
        for ch in char::decode_utf16(iter) {
            let c = ch.map_err(|_| WireError::InvalidString)?;
            if out.push(c).is_err() {
                break;
            }
        }
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn command_container_round_trip() {
        let mut raw = [0u8; 24];
        ContainerHeader {
            length: 24,
            container_type: ContainerType::Command,
            code: OperationCode::GET_OBJECT_HANDLES.0,
            transaction_id: 42,
        }
        .encode(&mut raw)
        .unwrap();
        raw[12..16].copy_from_slice(&0x0001_0001u32.to_le_bytes());
        raw[16..20].copy_from_slice(&0u32.to_le_bytes());
        raw[20..24].copy_from_slice(&HANDLE_ROOT.to_le_bytes());

        let cmd = CommandBlock::parse(&raw).unwrap();
        assert_eq!(cmd.operation, OperationCode::GET_OBJECT_HANDLES);
        assert_eq!(cmd.transaction_id, 42);
        assert_eq!(cmd.params(), &[0x0001_0001, 0, HANDLE_ROOT]);
        assert_eq!(cmd.param(4), 0);
    }

    #[test]
    fn response_and_event_encode() {
        let resp = ResponseBlock::with_params(ResponseCode::OK, 7, &[1, 2, 3]);
        let mut buf = [0u8; 32];
        let n = resp.encode(&mut buf).unwrap();
        assert_eq!(n, 24);
        let hdr = ContainerHeader::parse(&buf[..n]).unwrap();
        assert_eq!(hdr.container_type, ContainerType::Response);
        assert_eq!(hdr.code, ResponseCode::OK.0);
        assert_eq!(hdr.transaction_id, 7);

        let evt = EventBlock::new(EventCode::OBJECT_ADDED, &[99]);
        let en = evt.encode(&mut buf).unwrap();
        assert_eq!(en, 16);
        let ehdr = ContainerHeader::parse(&buf[..en]).unwrap();
        assert_eq!(ehdr.container_type, ContainerType::Event);
        assert_eq!(ehdr.code, EventCode::OBJECT_ADDED.0);
        assert_eq!(ehdr.transaction_id, 0xFFFF_FFFF);
    }

    #[test]
    fn ptp_string_round_trip_and_bounds() {
        let mut buf = [0u8; 64];
        let mut w = DatasetWriter::new(&mut buf);
        w.write_ptp_str("").unwrap();
        w.write_ptp_str("hello.txt").unwrap();
        let written = w.len();
        assert_eq!(written, ptp_str_encoded_len("") + ptp_str_encoded_len("hello.txt"));

        let mut r = DatasetReader::new(&buf[..written]);
        let empty: String<32> = r.read_ptp_str().unwrap();
        let hello: String<32> = r.read_ptp_str().unwrap();
        assert_eq!(empty.as_str(), "");
        assert_eq!(hello.as_str(), "hello.txt");
        assert_eq!(r.remaining(), 0);

        // Adversarial oversized string length prefix (CVE-2026-61495 style) must fail cleanly.
        let bad = [255u8, b'a', 0x00];
        let mut bad_r = DatasetReader::new(&bad);
        assert_eq!(bad_r.read_ptp_str::<32>(), Err(WireError::Truncated));
    }

    #[test]
    fn object_info_round_trip() {
        let entry = ObjectEntry::file(0x0001_0001, 5, "photo.JPG", 12345);
        assert_eq!(entry.format, ObjectFormat::EXIF_JPEG);
        let mut buf = [0u8; 256];
        let len = entry.encode_object_info(&mut buf).unwrap();
        let decoded = ObjectEntry::decode_object_info(&buf[..len]).unwrap();
        assert_eq!(decoded.storage_id, 0x0001_0001);
        assert_eq!(decoded.parent, 5);
        assert_eq!(decoded.size, 12345);
        assert_eq!(decoded.format, ObjectFormat::EXIF_JPEG);
        assert_eq!(decoded.filename.as_str(), "photo.JPG");
    }
}
