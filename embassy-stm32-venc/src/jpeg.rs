//! Safe JPEG encoder wrapper.
//!
//! Thin facade over the bindgen `JpegEnc*` API, sharing the same ASIC and EWL
//! platform layer as the H.264 encoder.

use crate::Venc;
use crate::error::{Error, ok_jpeg};
use crate::ffi::venc;
use crate::h264::Frame;

/// Input picture colour format accepted by the JPEG encoder.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PictureType {
    Yuv420Planar,
    Yuv420SemiPlanar,
    Yuv422InterleavedYuyv,
    Yuv422InterleavedUyvy,
    Rgb565,
    Rgb888,
}

impl PictureType {
    fn to_ffi(self) -> venc::JpegEncFrameType {
        use venc::*;
        match self {
            PictureType::Yuv420Planar => JpegEncFrameType_JPEGENC_YUV420_PLANAR,
            PictureType::Yuv420SemiPlanar => JpegEncFrameType_JPEGENC_YUV420_SEMIPLANAR,
            PictureType::Yuv422InterleavedYuyv => JpegEncFrameType_JPEGENC_YUV422_INTERLEAVED_YUYV,
            PictureType::Yuv422InterleavedUyvy => JpegEncFrameType_JPEGENC_YUV422_INTERLEAVED_UYVY,
            PictureType::Rgb565 => JpegEncFrameType_JPEGENC_RGB565,
            PictureType::Rgb888 => JpegEncFrameType_JPEGENC_RGB888,
        }
    }
}

/// Chroma subsampling mode.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Subsampling {
    /// 4:2:0 (2x2)
    Yuv420,
    /// 4:2:2 (2x1)
    Yuv422,
}

impl Subsampling {
    fn to_ffi(self) -> venc::JpegEncCodingMode {
        match self {
            Subsampling::Yuv420 => venc::JpegEncCodingMode_JPEGENC_420_MODE,
            Subsampling::Yuv422 => venc::JpegEncCodingMode_JPEGENC_422_MODE,
        }
    }
}

/// JPEG encoder configuration.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    /// Input picture width in pixels.
    pub input_width: u32,
    /// Input picture height in pixels.
    pub input_height: u32,
    /// Encoded width in pixels (multiple of 16).
    pub coding_width: u32,
    /// Encoded height in pixels (multiple of 16).
    pub coding_height: u32,
    /// Quantisation level, 1 (best quality) .. 10 (smallest).
    pub quality: u32,
    /// Input picture format.
    pub input_type: PictureType,
    /// Chroma subsampling.
    pub subsampling: Subsampling,
}

impl Config {
    /// Configuration for a `width x height` YUV420-planar input at the default
    /// quality.
    pub fn new(width: u32, height: u32) -> Self {
        Self {
            input_width: width,
            input_height: height,
            coding_width: width,
            coding_height: height,
            quality: 5,
            input_type: PictureType::Yuv420Planar,
            subsampling: Subsampling::Yuv420,
        }
    }
}

/// JPEG encoder instance.
pub struct Encoder<'a, 'd> {
    inst: venc::JpegEncInst,
    _venc: &'a Venc<'d>,
}

impl<'a, 'd> Encoder<'a, 'd> {
    pub(crate) fn new(_dev: &'a Venc<'d>, cfg: &Config) -> Result<Self, Error> {
        let mut ffi = venc::JpegEncCfg::default();
        ffi.inputWidth = cfg.input_width;
        ffi.inputHeight = cfg.input_height;
        ffi.codingWidth = cfg.coding_width;
        ffi.codingHeight = cfg.coding_height;
        ffi.qLevel = cfg.quality;
        ffi.frameType = cfg.input_type.to_ffi();
        ffi.codingMode = cfg.subsampling.to_ffi();
        ffi.codingType = venc::JpegEncCodingType_JPEGENC_WHOLE_FRAME;
        ffi.unitsType = venc::JpegEncAppUnitsType_JPEGENC_NO_UNITS;
        ffi.markerType = venc::JpegEncTableMarkerType_JPEGENC_SINGLE_MARKER;
        ffi.rotation = venc::JpegEncPictureRotation_JPEGENC_ROTATE_0;

        let mut inst: venc::JpegEncInst = core::ptr::null();
        let ret = unsafe { venc::JpegEncInit(&ffi, &mut inst) };
        ok_jpeg(ret)?;

        Ok(Self { inst, _venc: _dev })
    }

    /// Encode one picture into `out`. Returns the JPEG byte length.
    pub fn encode(&mut self, frame: &Frame<'_>, out: &mut [u8]) -> Result<usize, Error> {
        let mut enc_in = venc::JpegEncIn::default();
        enc_in.busLum = frame.luma.as_ptr() as usize;
        enc_in.pLum = frame.luma.as_ptr();
        if let Some((u, v)) = frame.chroma {
            enc_in.busCb = u.as_ptr() as usize;
            enc_in.pCb = u.as_ptr();
            enc_in.busCr = v.as_ptr() as usize;
            enc_in.pCr = v.as_ptr();
        }
        enc_in.pOutBuf = out.as_mut_ptr();
        enc_in.busOutBuf = out.as_ptr() as usize;
        enc_in.outBufSize = out.len() as u32;

        crate::coherency::clean_range(frame.luma.as_ptr() as u32, frame.luma.len() as u32);
        if let Some((u, v)) = frame.chroma {
            crate::coherency::clean_range(u.as_ptr() as u32, u.len() as u32);
            crate::coherency::clean_range(v.as_ptr() as u32, v.len() as u32);
        }
        crate::coherency::clean_range(out.as_ptr() as u32, out.len() as u32);

        let mut enc_out = venc::JpegEncOut::default();
        let ret = unsafe { venc::JpegEncEncode(self.inst, &enc_in, &mut enc_out, None, core::ptr::null_mut()) };
        if ret != venc::JpegEncRet_JPEGENC_FRAME_READY && ret != venc::JpegEncRet_JPEGENC_OK {
            return Err(Error::from_jpeg(ret));
        }

        crate::coherency::invalidate_range(out.as_ptr() as u32, enc_out.jfifSize);
        Ok(enc_out.jfifSize as usize)
    }
}

impl Drop for Encoder<'_, '_> {
    fn drop(&mut self) {
        unsafe { venc::JpegEncRelease(self.inst) };
    }
}
