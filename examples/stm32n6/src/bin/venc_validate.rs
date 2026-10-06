#![no_std]
#![no_main]

//! TEMPORARY bring-up harness: validates the VENC H.264 field checks on silicon.
//!
//! Walks each rule in the encoder's `H264CheckCfg`/`SetParameter` (width/height
//! limits and alignment, scaled-output sub-rectangle, macroblocks-per-picture,
//! level vs frame size, frame rate, reference-frame count vs view mode, HW
//! capability gates for RFC and SVCT) and asserts that `H264EncInit` accepts or
//! rejects it as documented.
//!
//! The capability gates are data-driven: the expectation for `refFrameCompress`
//! and `svctLevel` is derived from what `EWLReadAsicConfig` reports, so the run
//! is meaningful on any VENC revision.

use defmt::{info, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_stm32::pac;
use embassy_stm32::rcc::mux::Dcmippsel;
use embassy_stm32::rcc::{CpuClk, IcConfig, Icint, Icsel, Pll, Plldivm, Pllpdiv, Pllsel, SupplyConfig, SysClk};
use embassy_stm32::rif::{RifMaster, RifMasterAttributes, RifPeripheral, RifPeripheralAttributes};
use embassy_stm32::Config;
use embassy_stm32_venc::{Venc, venc};
use panic_probe as _;

const W: u32 = 512;
const H: u32 = 288;

const POOL_BASE: usize = 0x3420_0000;
const POOL_SIZE: usize = 0x180000;
const FRAME_BASE: usize = 0x3410_0000;
const FRAME_SIZE: usize = (W * H * 2) as usize;
const OUT_BASE: usize = 0x3414_8000;
const OUT_SIZE: usize = 0x38000;

/// One `H264EncInit` case. `expect_ok` is the documented outcome.
#[derive(Clone, Copy)]
struct Case {
    name: &'static str,
    stream_type: u8,
    view_mode: u8,
    level: u8,
    width: u32,
    height: u32,
    frame_rate_num: u32,
    frame_rate_denom: u32,
    scaled_width: u32,
    scaled_height: u32,
    ref_frame_amount: u32,
    ref_frame_compress: u32,
    svct_level: u32,
    expect_ok: bool,
}

/// A conforming 512x288 baseline; every case overrides one field.
const BASE: Case = Case {
    name: "baseline 512x288",
    stream_type: venc::H264EncStreamType_H264ENC_BYTE_STREAM,
    view_mode: venc::H264EncViewMode_H264ENC_BASE_VIEW_DOUBLE_BUFFER,
    level: venc::H264EncLevel_H264ENC_LEVEL_2_2,
    width: W,
    height: H,
    frame_rate_num: 30,
    frame_rate_denom: 1,
    scaled_width: 0,
    scaled_height: 0,
    ref_frame_amount: 1,
    ref_frame_compress: 0,
    svct_level: 0,
    expect_ok: true,
};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(rcc_config());
    enable_all_sram();
    promote_masters_to_secure();
    // The VENC sits behind the APB5 bus gate.
    pac::RCC.busensr().write(|w| w.set_apb5ens(true));

    let pool: &'static mut [u8] = unsafe { core::slice::from_raw_parts_mut(POOL_BASE as *mut u8, POOL_SIZE) };
    let venc_dev = Venc::new(p.VENC, pool);

    let hw = venc_dev.hw_config();
    info!(
        "hw: h264={} jpeg={} rgb={} scaling={} max_w={} bus_width={}",
        hw.h264, hw.jpeg, hw.rgb_input, hw.scaling, hw.max_encoded_width, hw.bus_width
    );
    // Capabilities as the encoder itself sees them (via our EWL implementation).
    let caps = unsafe { venc::EWLReadAsicConfig() };
    info!(
        "caps: rfc={} svct={} instant={} scaling={}",
        caps.rfcSupport, caps.svctSupport, caps.instantSupport, caps.scalingEnabled
    );

    let cases: &[Case] = &[
        BASE,
        // streamType
        Case {
            name: "streamType=2 (invalid)",
            stream_type: 2,
            expect_ok: false,
            ..BASE
        },
        // width: [132, 4080], multiple of 4
        Case {
            name: "width=128 (<132)",
            width: 128,
            expect_ok: false,
            ..BASE
        },
        Case {
            name: "width=4084 (>4080)",
            width: 4084,
            expect_ok: false,
            ..BASE
        },
        Case {
            name: "width=514 (not %4)",
            width: 514,
            expect_ok: false,
            ..BASE
        },
        // height: [96, 4080], multiple of 2
        Case {
            name: "height=94 (<96)",
            height: 94,
            expect_ok: false,
            ..BASE
        },
        Case {
            name: "height=289 (odd)",
            height: 289,
            expect_ok: false,
            ..BASE
        },
        // scaled output: must be inside the coded frame, aligned, and smaller
        Case {
            name: "scaledW=520 (>width)",
            scaled_width: 520,
            scaled_height: 288,
            expect_ok: false,
            ..BASE
        },
        Case {
            name: "scaledW=514 (not %4)",
            scaled_width: 514,
            scaled_height: 288,
            expect_ok: false,
            ..BASE
        },
        Case {
            name: "scaledH=290 (>height)",
            scaled_width: 256,
            scaled_height: 290,
            expect_ok: false,
            ..BASE
        },
        Case {
            name: "scaledH=287 (odd)",
            scaled_width: 256,
            scaled_height: 287,
            expect_ok: false,
            ..BASE
        },
        Case {
            name: "scaled=512x288 (no downscale)",
            scaled_width: W,
            scaled_height: H,
            expect_ok: false,
            ..BASE
        },
        Case {
            name: "scaled=256x144 (valid downscale)",
            scaled_width: 256,
            scaled_height: 144,
            expect_ok: true,
            ..BASE
        },
        // level vs macroblocks-per-picture
        Case {
            name: "level 1.1 @ 512x288 (576 MBs > 396)",
            level: venc::H264EncLevel_H264ENC_LEVEL_1_1,
            expect_ok: false,
            ..BASE
        },
        Case {
            name: "level 5.1 @ 512x288",
            level: venc::H264EncLevel_H264ENC_LEVEL_5_1,
            expect_ok: true,
            ..BASE
        },
        Case {
            name: "level=60 (>max 51)",
            level: 60,
            expect_ok: false,
            ..BASE
        },
        // frame rate
        Case {
            name: "frameRateNum=0",
            frame_rate_num: 0,
            expect_ok: false,
            ..BASE
        },
        Case {
            name: "frameRateDenom=0",
            frame_rate_denom: 0,
            expect_ok: false,
            ..BASE
        },
        Case {
            name: "frameRate 30/60 (denom>num)",
            frame_rate_num: 30,
            frame_rate_denom: 60,
            expect_ok: false,
            ..BASE
        },
        Case {
            name: "frameRate 1000/1001 (allowed)",
            frame_rate_num: 1000,
            frame_rate_denom: 1001,
            expect_ok: true,
            ..BASE
        },
        // reference frames [1..3]; >1 requires multi-buffer view mode
        Case {
            name: "refFrameAmount=0",
            ref_frame_amount: 0,
            expect_ok: false,
            ..BASE
        },
        Case {
            name: "refFrameAmount=4 (>3)",
            ref_frame_amount: 4,
            expect_ok: false,
            ..BASE
        },
        Case {
            name: "refFrameAmount=2, double-buffer",
            ref_frame_amount: 2,
            expect_ok: false,
            ..BASE
        },
        Case {
            name: "refFrameAmount=2, multi-buffer",
            ref_frame_amount: 2,
            view_mode: venc::H264EncViewMode_H264ENC_BASE_VIEW_MULTI_BUFFER,
            expect_ok: true,
            ..BASE
        },
        Case {
            name: "refFrameAmount=3, multi-buffer",
            ref_frame_amount: 3,
            view_mode: venc::H264EncViewMode_H264ENC_BASE_VIEW_MULTI_BUFFER,
            expect_ok: true,
            ..BASE
        },
        // HW capability gates
        Case {
            name: "refFrameCompress=1 (needs rfcSupport)",
            ref_frame_compress: 1,
            expect_ok: caps.rfcSupport != 0,
            ..BASE
        },
        Case {
            name: "svctLevel=1 (needs svctSupport)",
            svct_level: 1,
            expect_ok: caps.svctSupport != 0,
            ..BASE
        },
        Case {
            name: "svctLevel=3 (needs svctSupport)",
            svct_level: 3,
            view_mode: venc::H264EncViewMode_H264ENC_BASE_VIEW_MULTI_BUFFER,
            ref_frame_amount: 2,
            expect_ok: caps.svctSupport != 0,
            ..BASE
        },
    ];

    let mut passed = 0u32;
    let mut failed = 0u32;
    for c in cases {
        let mut ffi = venc::H264EncConfig::default();
        ffi.streamType = c.stream_type;
        ffi.viewMode = c.view_mode;
        ffi.level = c.level;
        ffi.width = c.width;
        ffi.height = c.height;
        ffi.frameRateNum = c.frame_rate_num;
        ffi.frameRateDenom = c.frame_rate_denom;
        ffi.scaledWidth = c.scaled_width;
        ffi.scaledHeight = c.scaled_height;
        ffi.refFrameAmount = c.ref_frame_amount;
        ffi.refFrameCompress = c.ref_frame_compress;
        ffi.svctLevel = c.svct_level;

        let mut inst: venc::H264EncInst = core::ptr::null();
        let ret = unsafe { venc::H264EncInit(&ffi, &mut inst) };
        let accepted = ret == venc::H264EncRet_H264ENC_OK;
        if accepted {
            unsafe { venc::H264EncRelease(inst) };
        }

        if accepted == c.expect_ok {
            passed += 1;
            info!("PASS {} accepted={} ret={}", c.name, accepted, ret);
        } else {
            failed += 1;
            warn!("FAIL {} expected_ok={} accepted={} ret={}", c.name, c.expect_ok, accepted, ret);
        }
    }
    info!("validate: {} passed, {} failed", passed, failed);

    // Fields that pass validation must still encode. One probe per interesting
    // non-default configuration.
    encode_probe(
        "byte-stream 1 ref",
        venc::H264EncStreamType_H264ENC_BYTE_STREAM,
        venc::H264EncViewMode_H264ENC_BASE_VIEW_DOUBLE_BUFFER,
        1,
        0,
        0,
        0,
    );
    encode_probe(
        "nal-unit 3 refs multi-buffer",
        venc::H264EncStreamType_H264ENC_NAL_UNIT_STREAM,
        venc::H264EncViewMode_H264ENC_BASE_VIEW_MULTI_BUFFER,
        3,
        0,
        0,
        0,
    );
    encode_probe(
        "byte-stream + svct level 1",
        venc::H264EncStreamType_H264ENC_BYTE_STREAM,
        venc::H264EncViewMode_H264ENC_BASE_VIEW_DOUBLE_BUFFER,
        1,
        0,
        0,
        1,
    );
    encode_probe(
        "byte-stream + single-buffer view",
        venc::H264EncStreamType_H264ENC_BYTE_STREAM,
        venc::H264EncViewMode_H264ENC_BASE_VIEW_SINGLE_BUFFER,
        1,
        0,
        0,
        0,
    );
    // `scaledWidth/Height` passes validation, but this ASIC reports no scaling
    // block, so SetParameter forces the scaled output off. A real downscale
    // would make this frame roughly 4x smaller than the others.
    encode_probe(
        "byte-stream + scaled 256x144",
        venc::H264EncStreamType_H264ENC_BYTE_STREAM,
        venc::H264EncViewMode_H264ENC_BASE_VIEW_DOUBLE_BUFFER,
        1,
        256,
        144,
        0,
    );

    info!("validate: done");
    loop {
        embassy_time::Timer::after_secs(3600).await;
    }
}

/// Init an encoder with `cfg`-ish parameters, feed it one synthetic picture and
/// report whether a non-empty stream came out.
fn encode_probe(
    name: &str,
    stream_type: u8,
    view_mode: u8,
    ref_amount: u32,
    scaled_w: u32,
    scaled_h: u32,
    svct_level: u32,
) {
    let mut ffi = venc::H264EncConfig::default();
    ffi.streamType = stream_type;
    ffi.viewMode = view_mode;
    ffi.level = venc::H264EncLevel_H264ENC_LEVEL_2_2;
    ffi.width = W;
    ffi.height = H;
    ffi.frameRateNum = 30;
    ffi.frameRateDenom = 1;
    ffi.scaledWidth = scaled_w;
    ffi.scaledHeight = scaled_h;
    ffi.refFrameAmount = ref_amount;
    ffi.svctLevel = svct_level;

    let mut inst: venc::H264EncInst = core::ptr::null();
    let ret = unsafe { venc::H264EncInit(&ffi, &mut inst) };
    if ret != venc::H264EncRet_H264ENC_OK {
        warn!("encode {}: init ret={}", name, ret);
        return;
    }

    // RGB565 input, same size as the coded frame.
    let mut pp = venc::H264EncPreProcessingCfg::default();
    unsafe { venc::H264EncGetPreProcessing(inst, &mut pp) };
    pp.inputType = venc::H264EncPictureType_H264ENC_RGB565;
    pp.origWidth = W;
    pp.origHeight = H;
    let ret = unsafe { venc::H264EncSetPreProcessing(inst, &pp) };
    if ret != venc::H264EncRet_H264ENC_OK {
        warn!("encode {}: SetPreProcessing ret={}", name, ret);
        unsafe { venc::H264EncRelease(inst) };
        return;
    }

    // A deterministic gradient so the encoder has real content to code.
    let frame = unsafe { core::slice::from_raw_parts_mut(FRAME_BASE as *mut u8, FRAME_SIZE) };
    for y in 0..H as usize {
        for x in 0..W as usize {
            let v = ((x ^ y) & 0x1f) as u16;
            let px = (v << 11) | (v << 6) | v;
            frame[(y * W as usize + x) * 2] = px as u8;
            frame[(y * W as usize + x) * 2 + 1] = (px >> 8) as u8;
        }
    }
    embassy_stm32_venc::coherency::clean_range(FRAME_BASE as u32, FRAME_SIZE as u32);

    let out = unsafe { core::slice::from_raw_parts_mut(OUT_BASE as *mut u8, OUT_SIZE) };
    embassy_stm32_venc::coherency::clean_range(OUT_BASE as u32, OUT_SIZE as u32);

    // SPS/PPS
    let mut enc_out = venc::H264EncOut::default();
    let mut enc_in = make_in(out);
    let ret = unsafe { venc::H264EncStrmStart(inst, &mut enc_in, &mut enc_out) };
    if ret != venc::H264EncRet_H264ENC_OK {
        warn!("encode {}: StrmStart ret={}", name, ret);
        unsafe { venc::H264EncRelease(inst) };
        return;
    }
    let header = enc_out.streamSize;

    // One IDR frame.
    let mut enc_in = make_in(out);
    enc_in.codingType = venc::H264EncPictureCodingType_H264ENC_INTRA_FRAME;
    enc_in.timeIncrement = 0;
    enc_in.ipf = venc::H264EncRefPictureMode_H264ENC_REFERENCE_AND_REFRESH;
    enc_in.ltrf = venc::H264EncRefPictureMode_H264ENC_REFERENCE;
    enc_in.busLuma = FRAME_BASE as venc::ptr_t;

    let mut enc_out = venc::H264EncOut::default();
    let ret =
        unsafe { venc::H264EncStrmEncode(inst, &mut enc_in, &mut enc_out, None, None, core::ptr::null_mut()) };
    let frame_bytes = enc_out.streamSize;

    let mut enc_in = make_in(out);
    let mut enc_out = venc::H264EncOut::default();
    let end_ret = unsafe { venc::H264EncStrmEnd(inst, &mut enc_in, &mut enc_out) };
    let end_bytes = enc_out.streamSize;

    unsafe { venc::H264EncRelease(inst) };

    if ret == venc::H264EncRet_H264ENC_FRAME_READY || ret == venc::H264EncRet_H264ENC_OK {
        info!(
            "encode {}: header={} frame={} eos={} (eos_ret={})",
            name, header, frame_bytes, end_bytes, end_ret
        );
    } else {
        warn!("encode {}: StrmEncode ret={} frame_bytes={}", name, ret, frame_bytes);
    }
}

fn make_in(out: &mut [u8]) -> venc::H264EncIn {
    let mut enc_in = venc::H264EncIn::default();
    enc_in.pOutBuf = out.as_mut_ptr() as *mut u32;
    enc_in.busOutBuf = out.as_ptr() as usize;
    enc_in.outBufSize = out.len() as u32;
    enc_in
}

// ---- RCC + RIFSC helpers (as in bin/venc_camera_to_sd.rs) ----

fn rcc_config() -> Config {
    let mut config = Config::default();
    config.rcc.supply_config = SupplyConfig::External;
    config.rcc.pll1 = Some(Pll::Oscillator {
        source: Pllsel::Hsi,
        divm: Plldivm::Div4,
        fractional: 0,
        divn: 50,
        divp1: Pllpdiv::Div1,
        divp2: Pllpdiv::Div1,
    });
    config.rcc.ic1 = Some(IcConfig {
        source: Icsel::Pll1,
        divider: Icint::Div1,
    });
    let sys_ic = IcConfig {
        source: Icsel::Pll1,
        divider: Icint::Div4,
    };
    config.rcc.ic2 = Some(sys_ic);
    config.rcc.ic6 = Some(sys_ic);
    config.rcc.ic11 = Some(sys_ic);
    config.rcc.cpu = CpuClk::Ic1;
    config.rcc.sys = SysClk::Ic2;

    config.rcc.ic17 = Some(IcConfig {
        source: Icsel::Pll1,
        divider: Icint::Div3,
    });
    config.rcc.mux.dcmippsel = Dcmippsel::Ic17;

    config.rcc.ic18 = Some(IcConfig {
        source: Icsel::Pll1,
        divider: Icint::Div60,
    });
    config
}

fn enable_all_sram() {
    pac::RCC.memenr().modify(|w| {
        w.set_axisram1en(true);
        w.set_axisram2en(true);
        w.set_axisram3en(true);
        w.set_axisram4en(true);
        w.set_axisram5en(true);
        w.set_axisram6en(true);
        w.set_ahbsram1en(true);
        w.set_ahbsram2en(true);
        w.set_bkpsramen(true);
    });
}

fn promote_masters_to_secure() {
    for rif_master in [RifMaster::Dcmipp, RifMaster::Venc] {
        rif_master.set_attributes(&RifMasterAttributes::new(1, true, true));
    }
    for rif_periph in [RifPeripheral::Dcmipp, RifPeripheral::Venc] {
        rif_periph.set_attributes(&RifPeripheralAttributes::new(true, true));
    }
}
