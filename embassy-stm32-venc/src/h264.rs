//! Safe H.264 encoder wrapper.
//!
//! This is a thin, `unsafe`-free facade over the bindgen `H264Enc*` API. It
//! manages the encoder instance lifetime and the `H264EncIn`/`H264EncOut`
//! plumbing so callers only deal with byte slices and configuration.
//!
//! ```ignore
//! let mut enc = venc.h264_config()
//!     .resolution(800, 480)
//!     .frame_rate(30)
//!     .build()?;
//!
//! let mut header = [0u8; 1024];
//! let n = enc.stream_start(&mut header)?;
//!
//! let mut out = [0u8; 800 * 480];
//! let n = enc.encode(&Frame::yuv420_planar(&y, &u, &v), CodingType::Intra, &mut out)?;
//! ```

use crate::Venc;
use crate::error::{Error, ok_h264};
use crate::ffi::venc;

/// Encoded stream framing.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum StreamType {
    /// Annex-B byte stream (start codes).
    ByteStream,
    /// Raw NAL units.
    NalUnits,
}

impl StreamType {
    fn to_ffi(self) -> venc::H264EncStreamType {
        match self {
            StreamType::ByteStream => venc::H264EncStreamType_H264ENC_BYTE_STREAM,
            StreamType::NalUnits => venc::H264EncStreamType_H264ENC_NAL_UNIT_STREAM,
        }
    }
}

/// H.264 level.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[allow(non_camel_case_types)]
pub enum Level {
    L1,
    L1_1,
    L1_2,
    L1_3,
    L2,
    L2_1,
    L2_2,
    L3,
    L3_1,
    L3_2,
    L4,
    L4_1,
    L4_2,
    L5,
    L5_1,
}

impl Level {
    fn to_ffi(self) -> venc::H264EncLevel {
        use venc::*;
        match self {
            Level::L1 => H264EncLevel_H264ENC_LEVEL_1,
            Level::L1_1 => H264EncLevel_H264ENC_LEVEL_1_1,
            Level::L1_2 => H264EncLevel_H264ENC_LEVEL_1_2,
            Level::L1_3 => H264EncLevel_H264ENC_LEVEL_1_3,
            Level::L2 => H264EncLevel_H264ENC_LEVEL_2,
            Level::L2_1 => H264EncLevel_H264ENC_LEVEL_2_1,
            Level::L2_2 => H264EncLevel_H264ENC_LEVEL_2_2,
            Level::L3 => H264EncLevel_H264ENC_LEVEL_3,
            Level::L3_1 => H264EncLevel_H264ENC_LEVEL_3_1,
            Level::L3_2 => H264EncLevel_H264ENC_LEVEL_3_2,
            Level::L4 => H264EncLevel_H264ENC_LEVEL_4,
            Level::L4_1 => H264EncLevel_H264ENC_LEVEL_4_1,
            Level::L4_2 => H264EncLevel_H264ENC_LEVEL_4_2,
            Level::L5 => H264EncLevel_H264ENC_LEVEL_5,
            Level::L5_1 => H264EncLevel_H264ENC_LEVEL_5_1,
        }
    }
}

/// Input picture colour format.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PictureType {
    Yuv420Planar,
    Yuv420SemiPlanar,
    Yuv420SemiPlanarVu,
    Yuv422InterleavedYuyv,
    Yuv422InterleavedUyvy,
    Rgb565,
    Rgb888,
    Bgr565,
    Bgr888,
}

impl PictureType {
    fn to_ffi(self) -> venc::H264EncPictureType {
        use venc::*;
        match self {
            PictureType::Yuv420Planar => H264EncPictureType_H264ENC_YUV420_PLANAR,
            PictureType::Yuv420SemiPlanar => H264EncPictureType_H264ENC_YUV420_SEMIPLANAR,
            PictureType::Yuv420SemiPlanarVu => H264EncPictureType_H264ENC_YUV420_SEMIPLANAR_VU,
            PictureType::Yuv422InterleavedYuyv => H264EncPictureType_H264ENC_YUV422_INTERLEAVED_YUYV,
            PictureType::Yuv422InterleavedUyvy => H264EncPictureType_H264ENC_YUV422_INTERLEAVED_UYVY,
            PictureType::Rgb565 => H264EncPictureType_H264ENC_RGB565,
            PictureType::Rgb888 => H264EncPictureType_H264ENC_RGB888,
            PictureType::Bgr565 => H264EncPictureType_H264ENC_BGR565,
            PictureType::Bgr888 => H264EncPictureType_H264ENC_BGR888,
        }
    }
}

/// Per-picture coding decision.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum CodingType {
    Intra,
    Predicted,
}

impl CodingType {
    fn to_ffi(self) -> venc::H264EncPictureCodingType {
        match self {
            CodingType::Intra => venc::H264EncPictureCodingType_H264ENC_INTRA_FRAME,
            CodingType::Predicted => venc::H264EncPictureCodingType_H264ENC_PREDICTED_FRAME,
        }
    }
}

/// Reference-frame handling for a picture.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum RefMode {
    None,
    Reference,
    Refresh,
    ReferenceAndRefresh,
}

impl RefMode {
    fn to_ffi(self) -> venc::H264EncRefPictureMode {
        use venc::*;
        match self {
            RefMode::None => H264EncRefPictureMode_H264ENC_NO_REFERENCE_NO_REFRESH,
            RefMode::Reference => H264EncRefPictureMode_H264ENC_REFERENCE,
            RefMode::Refresh => H264EncRefPictureMode_H264ENC_REFRESH,
            RefMode::ReferenceAndRefresh => H264EncRefPictureMode_H264ENC_REFERENCE_AND_REFRESH,
        }
    }
}

/// H.264 encoder configuration.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Encoded (output) picture width in pixels, multiple of 4.
    pub width: u32,
    /// Encoded (output) picture height in pixels, multiple of 2.
    pub height: u32,
    /// Input picture width in pixels. Defaults to [`width`](Self::width).
    ///
    /// The encoder validates that the input picture covers the coded frame
    /// (`xOffset + width <= input_width`), so this must be at least `width`.
    /// Set it larger only if the encoder itself should crop/downscale.
    pub input_width: u32,
    /// Input picture height in pixels. Defaults to [`height`](Self::height).
    pub input_height: u32,
    pub frame_rate_num: u32,
    pub frame_rate_denom: u32,
    pub ref_frame_amount: u32,
    pub stream_type: StreamType,
    pub level: Level,
    pub input_type: PictureType,
}

impl Config {
    /// A sensible starting configuration for a `width x height` input at `fps`
    /// with one reference frame and Annex-B byte-stream output.
    pub fn new(width: u32, height: u32, fps: u32) -> Self {
        Self {
            width,
            height,
            input_width: width,
            input_height: height,
            frame_rate_num: fps,
            frame_rate_denom: 1,
            ref_frame_amount: 1,
            stream_type: StreamType::ByteStream,
            level: Level::L2_2,
            input_type: PictureType::Rgb565,
        }
    }
}

/// An input picture. Only the components required by the configured
/// [`PictureType`] need to be provided.
#[derive(Clone, Copy)]
pub struct Frame<'f> {
    /// Luminance plane, or the whole interleaved picture for packed formats.
    pub luma: &'f [u8],
    /// Chrominance planes for planar formats.
    pub chroma: Option<(&'f [u8], &'f [u8])>,
}

impl<'f> Frame<'f> {
    /// Packed/interleaved input (e.g. RGB565): a single buffer.
    pub fn packed(buf: &'f [u8]) -> Self {
        Self {
            luma: buf,
            chroma: None,
        }
    }

    /// Planar YUV420: separate luma, Cb and Cr planes.
    pub fn yuv420_planar(y: &'f [u8], u: &'f [u8], v: &'f [u8]) -> Self {
        Self {
            luma: y,
            chroma: Some((u, v)),
        }
    }
}

/// H.264 encoder instance.
pub struct Encoder<'a, 'd> {
    inst: venc::H264EncInst,
    input_width: u32,
    input_height: u32,
    _venc: &'a Venc<'d>,
}

impl<'a, 'd> Encoder<'a, 'd> {
    pub(crate) fn new(dev: &'a Venc<'d>, cfg: &Config) -> Result<Self, Error> {
        let mut ffi = venc::H264EncConfig::default();
        ffi.streamType = cfg.stream_type.to_ffi();
        ffi.level = cfg.level.to_ffi();
        ffi.width = cfg.width;
        ffi.height = cfg.height;
        ffi.frameRateNum = cfg.frame_rate_num;
        ffi.frameRateDenom = cfg.frame_rate_denom;
        ffi.refFrameAmount = cfg.ref_frame_amount;

        let mut inst: venc::H264EncInst = core::ptr::null();
        let ret = unsafe { venc::H264EncInit(&ffi, &mut inst) };
        ok_h264(ret)?;

        let mut me = Self {
            inst,
            input_width: cfg.input_width,
            input_height: cfg.input_height,
            _venc: dev,
        };
        me.set_pre_processing(cfg.input_type)?;
        Ok(me)
    }

    /// Configure input pre-processing to accept `input` pictures of the size
    /// given by [`Config::input_width`]/[`Config::input_height`].
    ///
    /// `origWidth`/`origHeight` must be filled in: the firmware rejects the
    /// call when the input picture does not cover the coded frame, since
    /// `origWidth == 0` fails its `xOffset + width <= origWidth` check.
    pub fn set_pre_processing(&mut self, input: PictureType) -> Result<(), Error> {
        let mut cfg = venc::H264EncPreProcessingCfg::default();
        ok_h264(unsafe { venc::H264EncGetPreProcessing(self.inst, &mut cfg) })?;
        cfg.inputType = input.to_ffi();
        cfg.origWidth = self.input_width;
        cfg.origHeight = self.input_height;
        ok_h264(unsafe { venc::H264EncSetPreProcessing(self.inst, &cfg) })
    }

    /// Write the stream header (SPS/PPS and, for byte streams, the start code).
    /// Returns the number of bytes written into `out`.
    pub fn stream_start(&mut self, out: &mut [u8]) -> Result<usize, Error> {
        let mut enc_in = self.make_in(out);
        let mut enc_out = venc::H264EncOut::default();
        ok_h264(unsafe { venc::H264EncStrmStart(self.inst, &mut enc_in, &mut enc_out) })?;
        Ok(enc_out.streamSize as usize)
    }

    /// Encode one picture. Returns the number of bytes written into `out`.
    pub fn encode(
        &mut self,
        frame: &Frame<'_>,
        coding: CodingType,
        time_increment: u32,
        out: &mut [u8],
    ) -> Result<usize, Error> {
        let mut enc_in = self.make_in(out);
        enc_in.codingType = coding.to_ffi();
        enc_in.timeIncrement = time_increment;
        enc_in.ipf = RefMode::ReferenceAndRefresh.to_ffi();
        enc_in.ltrf = RefMode::Reference.to_ffi();

        enc_in.busLuma = frame.luma.as_ptr() as venc::ptr_t;
        if let Some((u, v)) = frame.chroma {
            enc_in.busChromaU = u.as_ptr() as venc::ptr_t;
            enc_in.busChromaV = v.as_ptr() as venc::ptr_t;
        }

        // The ASIC reads the input picture through its own bus master.
        crate::coherency::clean_range(frame.luma.as_ptr() as u32, frame.luma.len() as u32);
        if let Some((u, v)) = frame.chroma {
            crate::coherency::clean_range(u.as_ptr() as u32, u.len() as u32);
            crate::coherency::clean_range(v.as_ptr() as u32, v.len() as u32);
        }
        crate::coherency::clean_range(out.as_ptr() as u32, out.len() as u32);

        let mut enc_out = venc::H264EncOut::default();
        let ret =
            unsafe { venc::H264EncStrmEncode(self.inst, &mut enc_in, &mut enc_out, None, None, core::ptr::null_mut()) };
        if ret != venc::H264EncRet_H264ENC_FRAME_READY && ret != venc::H264EncRet_H264ENC_OK {
            return Err(Error::from_h264(ret));
        }

        crate::coherency::invalidate_range(out.as_ptr() as u32, enc_out.streamSize);
        Ok(enc_out.streamSize as usize)
    }

    /// Flush the encoder at the end of the stream. Returns bytes written.
    pub fn stream_end(&mut self, out: &mut [u8]) -> Result<usize, Error> {
        let mut enc_in = self.make_in(out);
        let mut enc_out = venc::H264EncOut::default();
        ok_h264(unsafe { venc::H264EncStrmEnd(self.inst, &mut enc_in, &mut enc_out) })?;
        Ok(enc_out.streamSize as usize)
    }

    fn make_in(&self, out: &mut [u8]) -> venc::H264EncIn {
        let mut enc_in = venc::H264EncIn::default();
        enc_in.pOutBuf = out.as_mut_ptr() as *mut u32;
        enc_in.busOutBuf = out.as_ptr() as usize;
        enc_in.outBufSize = out.len() as u32;
        enc_in
    }
}

impl Drop for Encoder<'_, '_> {
    fn drop(&mut self) {
        unsafe { venc::H264EncRelease(self.inst) };
    }
}
