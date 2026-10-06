//! Encoder error type.

use crate::ffi::venc;

/// Video Encoder error.
///
/// The Hantro stack uses one status enum per encoder; both collapse onto this
/// type. [`Error::FrameReady`] is not a failure — it is the positive status
/// returned by the streaming encode call once a frame has been produced.
#[derive(Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// A required pointer argument was NULL.
    NullArgument,
    /// One of the configuration values was out of range.
    InvalidArgument,
    /// The encoder instance is invalid or already released.
    InstanceError,
    /// The call is not valid in the current encoder state.
    InvalidStatus,
    /// Internal (non-EWL) memory allocation failed.
    MemoryError,
    /// The EWL platform layer reported an error.
    EwlError,
    /// The EWL platform layer could not allocate memory.
    EwlMemoryError,
    /// The ASIC reported a bus error.
    HwBusError,
    /// The ASIC reported a data error.
    HwDataError,
    /// The ASIC reported a reserved/unexpected state.
    HwReserved,
    /// The ASIC was reset while encoding.
    HwReset,
    /// The ASIC did not complete within the timeout.
    HwTimeout,
    /// Encoding a frame succeeded (positive status, not an error).
    FrameReady,
    /// The output bitstream buffer was too small.
    OutputBufferOverflow,
    /// Generic internal error.
    SystemError,
    /// Hypothetical Reference Decoder constraint violation.
    HrdError,
    /// ASIC "fuse" error.
    FuseError,
    /// JPEG restart interval error.
    RestartInterval,
    /// Unrecognised status code.
    Other,
}

macro_rules! map_h264 {
    ($r:expr, $($k:ident => $v:ident),+ $(,)?) => {
        match $r {
            $(venc::$k => Error::$v,)+
            _ => Error::Other,
        }
    };
}

macro_rules! map_jpeg {
    ($r:expr, $($k:ident => $v:ident),+ $(,)?) => {
        match $r {
            $(venc::$k => Error::$v,)+
            _ => Error::Other,
        }
    };
}

impl Error {
    pub(crate) fn from_h264(r: venc::H264EncRet) -> Self {
        map_h264!(r,
            H264EncRet_H264ENC_OK => FrameReady,
            H264EncRet_H264ENC_NULL_ARGUMENT => NullArgument,
            H264EncRet_H264ENC_INVALID_ARGUMENT => InvalidArgument,
            H264EncRet_H264ENC_INSTANCE_ERROR => InstanceError,
            H264EncRet_H264ENC_INVALID_STATUS => InvalidStatus,
            H264EncRet_H264ENC_MEMORY_ERROR => MemoryError,
            H264EncRet_H264ENC_EWL_ERROR => EwlError,
            H264EncRet_H264ENC_EWL_MEMORY_ERROR => EwlMemoryError,
            H264EncRet_H264ENC_HW_BUS_ERROR => HwBusError,
            H264EncRet_H264ENC_HW_DATA_ERROR => HwDataError,
            H264EncRet_H264ENC_HW_RESERVED => HwReserved,
            H264EncRet_H264ENC_HW_RESET => HwReset,
            H264EncRet_H264ENC_HW_TIMEOUT => HwTimeout,
            H264EncRet_H264ENC_FRAME_READY => FrameReady,
            H264EncRet_H264ENC_OUTPUT_BUFFER_OVERFLOW => OutputBufferOverflow,
            H264EncRet_H264ENC_SYSTEM_ERROR => SystemError,
            H264EncRet_H264ENC_HRD_ERROR => HrdError,
            H264EncRet_H264ENC_FUSE_ERROR => FuseError,
        )
    }

    pub(crate) fn from_jpeg(r: venc::JpegEncRet) -> Self {
        map_jpeg!(r,
            JpegEncRet_JPEGENC_OK => FrameReady,
            JpegEncRet_JPEGENC_NULL_ARGUMENT => NullArgument,
            JpegEncRet_JPEGENC_INVALID_ARGUMENT => InvalidArgument,
            JpegEncRet_JPEGENC_INSTANCE_ERROR => InstanceError,
            JpegEncRet_JPEGENC_INVALID_STATUS => InvalidStatus,
            JpegEncRet_JPEGENC_MEMORY_ERROR => MemoryError,
            JpegEncRet_JPEGENC_EWL_ERROR => EwlError,
            JpegEncRet_JPEGENC_EWL_MEMORY_ERROR => EwlMemoryError,
            JpegEncRet_JPEGENC_HW_BUS_ERROR => HwBusError,
            JpegEncRet_JPEGENC_HW_DATA_ERROR => HwDataError,
            JpegEncRet_JPEGENC_HW_RESERVED => HwReserved,
            JpegEncRet_JPEGENC_HW_RESET => HwReset,
            JpegEncRet_JPEGENC_HW_TIMEOUT => HwTimeout,
            JpegEncRet_JPEGENC_FRAME_READY => FrameReady,
            JpegEncRet_JPEGENC_OUTPUT_BUFFER_OVERFLOW => OutputBufferOverflow,
            JpegEncRet_JPEGENC_SYSTEM_ERROR => SystemError,
            JpegEncRet_JPEGENC_RESTART_INTERVAL => RestartInterval,
        )
    }
}

/// Convert an `H264ENC_OK`-style status into `Result<(), Error>`.
pub(crate) fn ok_h264(r: venc::H264EncRet) -> Result<(), Error> {
    if r == venc::H264EncRet_H264ENC_OK {
        Ok(())
    } else {
        Err(Error::from_h264(r))
    }
}

/// Convert a `JPEGENC_OK`-style status into `Result<(), Error>`.
pub(crate) fn ok_jpeg(r: venc::JpegEncRet) -> Result<(), Error> {
    if r == venc::JpegEncRet_JPEGENC_OK {
        Ok(())
    } else {
        Err(Error::from_jpeg(r))
    }
}
