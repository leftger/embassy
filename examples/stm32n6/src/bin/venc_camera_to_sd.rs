#![no_std]
#![no_main]

//! Camera capture → VENC (H.264 + JPEG) → SD card example for the
//! STM32N6570-DK + MB1854.
//!
//! Streams the IMX335 → CSI → DCMIPP pipeline into RGB565 frames in AXI SRAM
//! and runs them through the on-chip Video Encoder:
//!
//! * **B4 (Tamper, PE0)** — record an H.264 clip (`VID<NNNN>.264`, Annex-B).
//!   Recording stops when B4 is pressed again or after `MAX_FRAMES` frames.
//! * **B2 (PC13)** — write a single JPEG snapshot (`IMG<NNNN>.JPG`).
//!
//! Both encoders share the same ASIC and the same EWL arena; a fresh encoder
//! instance is created per operation and released when it is dropped.
//!
//! # Memory layout
//!
//! The encoder's arena (1.5 MiB) and the two working buffers are placed at
//! fixed AXI-SRAM addresses rather than via the linker, because they are far
//! larger than the 256 KiB `RAM` region the linker script defines and because
//! the VENC's bus master has to see them:
//!
//! ```text
//!   0x3410_0000  FRAME  RGB565 512x288          (294 912 B)  AXISRAM2
//!   0x3414_8000  OUT    per-frame bitstream     (229 376 B)  AXISRAM2
//!   0x3420_0000  POOL   EWL arena               (1 572 864 B) AXISRAM3..6
//! ```
//!
//! # Building
//!
//! Requires the `venc` module of `stm32-bindings` (see `embassy-stm32-venc`'s
//! README). The default `RAM` window (0x341C_0000..0x3420_0000) stays reserved
//! for `.data`/`.bss`, so nothing here collides with the linker script.

#[path = "../imx335.rs"]
mod imx335;

use core::cell::RefCell;
use core::fmt::Write as _;

use defmt::{error, info, unwrap, warn};
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::block_on;
use embassy_futures::select::{Either, select};
use embassy_stm32::csi::{self, Csi, LaneCount};
use embassy_stm32::dcmipp::{
    self, BayerPattern, Dcmipp, DownsizeConfig, InputSource, Pipe1, Pipe1Config, PixelFormat as DcmippPixelFormat,
};
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::i2c::I2c;
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::DCMIPP;
use embassy_stm32::rcc::mux::Dcmippsel;
use embassy_stm32::rcc::{CpuClk, IcConfig, Icint, Icsel, Pll, Plldivm, Pllpdiv, Pllsel, SupplyConfig, SysClk};
use embassy_stm32::rif::{RifMaster, RifMasterAttributes, RifPeripheral, RifPeripheralAttributes};
use embassy_stm32::sdmmc::Sdmmc;
use embassy_stm32::sdmmc::sd::{Addressable, Card, CmdBlock, DataBlock, StorageDevice};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Config, bind_interrupts, interrupt, pac, peripherals};
use embassy_stm32_venc::{Venc, h264, jpeg};
use embassy_time::Timer;
use embedded_sdmmc::{Block, BlockCount, BlockDevice, BlockIdx, Mode, TimeSource, Timestamp, VolumeIdx, VolumeManager};
use panic_probe as _;

use crate::imx335::Imx335;

bind_interrupts!(struct Irqs {
    CSI    => csi::InterruptHandler<peripherals::CSI>;
    DCMIPP => dcmipp::InterruptHandler<peripherals::DCMIPP>;
    EXTI13 => exti::InterruptHandler<interrupt::typelevel::EXTI13>;
    EXTI0  => exti::InterruptHandler<interrupt::typelevel::EXTI0>;
    SDMMC2 => embassy_stm32::sdmmc::InterruptHandler<peripherals::SDMMC2>;
});

const SENSOR_W: u16 = 2592;
const SENSOR_H: u16 = 1944;
const CSI_RATE_MBPS: u32 = 1600;

/// Encoded picture size (both must be EWL-friendly: W%4 == 0, H%2 == 0).
const ENC_W: usize = 512;
const ENC_H: usize = 288;
const ENC_FPS: u32 = 30;
/// Give up after ~10 s so the demo can't fill the card forever. B4 stops earlier.
const MAX_FRAMES: u32 = 300;

/// RGB565 frame buffer the DCMIPP writes and the VENC reads.
const FRAME_BASE: usize = 0x3410_0000;
const FRAME_SIZE: usize = ENC_W * ENC_H * 2;
const FRAME_PITCH: u16 = (ENC_W * 2) as u16;

/// Per-frame H.264/JPEG bitstream buffer.
const OUT_BASE: usize = 0x3414_8000;
const OUT_SIZE: usize = 0x38000;

/// EWL arena. Spans AXISRAM3..6 (contiguous 0x3420_0000..0x343C_0000).
const POOL_BASE: usize = 0x3420_0000;
const POOL_SIZE: usize = 0x180000;

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_stm32::init(rcc_config());
    info!("stm32n6 venc camera->sd starting");

    enable_all_sram();
    promote_masters_to_secure();

    // The VENC lives behind the APB5 bus gate. ST's `LL_VENC_Init` sets this;
    // without it the first VENC register access faults.
    pac::RCC.busensr().write(|w| w.set_apb5ens(true));

    // ---- Camera sensor ----
    let mut i2c_cfg = embassy_stm32::i2c::Config::default();
    i2c_cfg.frequency = Hertz::khz(400);
    let i2c = I2c::new_blocking(p.I2C1, p.PH9, p.PC1, i2c_cfg);
    let pwr_en = Output::new(p.PC8, Level::Low, Speed::Low);
    let nrst = Output::new(p.PD2, Level::Low, Speed::Low);
    let mut cam = Imx335::new(i2c, pwr_en, nrst);
    unwrap!(cam.power_on().await);
    unwrap!(cam.init().await);
    unwrap!(cam.set_gain_db_x10(150));
    unwrap!(cam.set_shutter_lines(2000));
    info!("imx335: ready");

    // ---- CSI + DCMIPP ----
    // The DCMIPP does the downscale and debayer; the VENC gets RGB565 and
    // converts to YUV420 in its own pre-processing block.
    let mut csi = Csi::new(p.CSI, Irqs, csi::Config::new(LaneCount::Two, CSI_RATE_MBPS));

    let dcmipp = Dcmipp::new(p.DCMIPP, Irqs);
    let (_pipe0, mut pipe1, _pipe2) = dcmipp.split();
    let mut p1cfg = Pipe1Config::new(InputSource::Csi, DcmippPixelFormat::Rgb565, FRAME_PITCH);
    p1cfg.demosaic = Some(BayerPattern::Rggb);
    p1cfg.downsize = Some(DownsizeConfig {
        input: (SENSOR_W, SENSOR_H),
        output: (ENC_W as u16, ENC_H as u16),
    });
    pipe1.configure(&p1cfg);

    // ---- Video Encoder ----
    let pool: &'static mut [u8] = unsafe { core::slice::from_raw_parts_mut(POOL_BASE as *mut u8, POOL_SIZE) };
    let venc = Venc::new(p.VENC, pool);
    let hw = venc.hw_config();
    info!(
        "venc: asic id 0x{:08x} h264={} jpeg={} rgb_in={} max_width={}",
        venc.asic_id(),
        hw.h264,
        hw.jpeg,
        hw.rgb_input,
        hw.max_encoded_width
    );
    if !hw.h264 {
        error!("venc: this ASIC reports no H.264 support, aborting");
        loop {
            Timer::after_secs(3600).await;
        }
    }

    // ---- SD card ----
    let mut sd_cfg = embassy_stm32::sdmmc::Config::default();
    sd_cfg.data_transfer_timeout = 200_000_000;
    let mut sd = Sdmmc::new_4bit(p.SDMMC2, p.PC2, p.PC3, p.PC4, p.PC5, p.PC0, p.PE4, Irqs, sd_cfg);
    let mut cmd_block = CmdBlock::new();
    #[allow(deprecated)]
    let mut sd_state = match StorageDevice::new_sd_card(&mut sd, &mut cmd_block, Hertz(24_000_000)).await {
        Ok(storage) => {
            info!("sd: card ready, {} bytes", storage.card().size());
            let block_dev = EmbassyBlockDevice {
                inner: RefCell::new(storage),
            };
            let mut volume_mgr: VolumeManager<_, _> = VolumeManager::new(block_dev, FixedTime);
            let next_idx = scan_next_index(&mut volume_mgr);
            info!("sd: next file index = {}", next_idx);
            Some((volume_mgr, next_idx))
        }
        Err(e) => {
            info!("sd init failed, recording disabled: {:?}", defmt::Debug2Format(&e));
            None
        }
    };

    // ---- Buttons ----
    let mut record_button = ExtiInput::new(p.PE0, p.EXTI0, Pull::Down, Irqs); // B4 = Tamper
    let mut snap_button = ExtiInput::new(p.PC13, p.EXTI13, Pull::Down, Irqs); // B2

    // ---- Streaming ----
    csi.start();
    unwrap!(cam.start_streaming().await);
    info!("streaming; B4 = record H.264, B2 = JPEG snapshot");

    loop {
        match select(record_button.wait_for_rising_edge(), snap_button.wait_for_rising_edge()).await {
            Either::First(()) => {
                let Some((volume_mgr, next_idx)) = sd_state.as_mut() else {
                    info!("record: no SD card, ignored");
                    continue;
                };
                let idx = *next_idx;
                info!("record: VID{:04}.264", idx);
                let bytes = record_clip(volume_mgr, &mut pipe1, &venc, &mut record_button, idx).await;
                if bytes != 0 {
                    info!("record: wrote {} bytes to VID{:04}.264", bytes, idx);
                    *next_idx = next_idx.wrapping_add(1);
                }
            }
            Either::Second(()) => {
                let Some((volume_mgr, next_idx)) = sd_state.as_mut() else {
                    info!("snapshot: no SD card, ignored");
                    continue;
                };
                let idx = *next_idx;
                let bytes = snapshot_jpeg(volume_mgr, &mut pipe1, &venc, idx).await;
                if bytes != 0 {
                    info!("snapshot: wrote {} bytes to IMG{:04}.JPG", bytes, idx);
                    *next_idx = next_idx.wrapping_add(1);
                }
                Timer::after_millis(150).await; // debounce
            }
        }
    }
}

/// Capture frames back-to-back and append the H.264 bitstream to
/// `VID<idx>.264` until `stop` is pressed or `MAX_FRAMES` is reached.
///
/// Returns the number of bytes written, or 0 if the file/encoder could not be
/// set up.
async fn record_clip<'a, 'b>(
    volume_mgr: &mut VolumeManager<EmbassyBlockDevice<'a, 'b>, FixedTime>,
    pipe1: &mut Pipe1<'_, DCMIPP>,
    venc: &Venc<'_>,
    stop: &mut ExtiInput<'static, Async>,
    file_idx: u32,
) -> usize {
    let frame: &'static [u8] = unsafe { core::slice::from_raw_parts(FRAME_BASE as *const u8, FRAME_SIZE) };
    let out: &'static mut [u8] = unsafe { core::slice::from_raw_parts_mut(OUT_BASE as *mut u8, OUT_SIZE) };

    let mut name = heapless::String::<13>::new();
    // 8.3 only: the FAT layer rejects a 4-character `.H264` extension.
    let _ = write!(name, "VID{:04}.264", file_idx);

    let volume = match volume_mgr.open_volume(VolumeIdx(0)) {
        Ok(v) => v,
        Err(e) => {
            error!("record: open_volume: {:?}", defmt::Debug2Format(&e));
            return 0;
        }
    };
    let root = match volume.open_root_dir() {
        Ok(r) => r,
        Err(e) => {
            error!("record: open_root_dir: {:?}", defmt::Debug2Format(&e));
            return 0;
        }
    };
    let file = match root.open_file_in_dir(name.as_str(), Mode::ReadWriteCreateOrTruncate) {
        Ok(f) => f,
        Err(e) => {
            error!("record: create {}: {:?}", name.as_str(), defmt::Debug2Format(&e));
            return 0;
        }
    };

    let cfg = h264::Config::new(ENC_W as u32, ENC_H as u32, ENC_FPS);
    let mut enc = match venc.h264(cfg) {
        Ok(e) => e,
        Err(e) => {
            error!("record: venc h264 init: {:?}", e);
            let _ = file.close();
            let _ = root.close();
            let _ = volume.close();
            return 0;
        }
    };

    let mut total = 0usize;
    let mut frames = 0u32;

    // Everything from here on runs with the file open; a labelled block lets
    // every failure path fall through to the single close at the end.
    let ok = 'clip: {
        // SPS/PPS (and, for byte streams, the first start code).
        match enc.stream_start(out) {
            Ok(n) => {
                if file.write(&out[..n]).is_err() {
                    error!("record: header write failed");
                    break 'clip false;
                }
                total += n;
            }
            Err(e) => {
                error!("record: stream_start: {:?}", e);
                break 'clip false;
            }
        }

        let mut coding = h264::CodingType::Intra;
        let mut time_increment = 0u32;

        loop {
            match select(pipe1.capture(FRAME_BASE as *mut u8), stop.wait_for_rising_edge()).await {
                Either::First(Ok(())) => {
                    let pic = h264::Frame::packed(frame);
                    match enc.encode(&pic, coding, time_increment, out) {
                        Ok(n) => {
                            if file.write(&out[..n]).is_err() {
                                error!("record: frame write failed");
                                break 'clip false;
                            }
                            total += n;
                        }
                        Err(e) => error!("record: encode frame {}: {:?}", frames, e),
                    }
                    frames += 1;
                    if frames.is_multiple_of(ENC_FPS) {
                        info!("record: {} frames, {} bytes", frames, total);
                    }
                    // Only the first frame is an IDR; the rest predict from it.
                    coding = h264::CodingType::Predicted;
                    time_increment = 1;
                    if frames >= MAX_FRAMES {
                        break;
                    }
                }
                Either::First(Err(e)) => warn!("record: capture: {:?}", e),
                Either::Second(()) => break,
            }
        }

        // End-of-sequence NAL.
        if let Ok(n) = enc.stream_end(out) {
            if file.write(&out[..n]).is_ok() {
                total += n;
            }
        }
        true
    };

    drop(enc);
    info!("record: {} frames captured", frames);
    let _ = file.close();
    let _ = root.close();
    let _ = volume.close();
    if ok { total } else { 0 }
}

/// Capture one frame and encode it as a JPEG.
async fn snapshot_jpeg<'a, 'b>(
    volume_mgr: &mut VolumeManager<EmbassyBlockDevice<'a, 'b>, FixedTime>,
    pipe1: &mut Pipe1<'_, DCMIPP>,
    venc: &Venc<'_>,
    file_idx: u32,
) -> usize {
    if let Err(e) = pipe1.capture(FRAME_BASE as *mut u8).await {
        warn!("snapshot: capture: {:?}", e);
        return 0;
    }

    let frame: &'static [u8] = unsafe { core::slice::from_raw_parts(FRAME_BASE as *const u8, FRAME_SIZE) };
    let out: &'static mut [u8] = unsafe { core::slice::from_raw_parts_mut(OUT_BASE as *mut u8, OUT_SIZE) };

    let mut cfg = jpeg::Config::new(ENC_W as u32, ENC_H as u32);
    cfg.input_type = jpeg::PictureType::Rgb565;
    cfg.quality = 6;

    let mut enc = match venc.jpeg(cfg) {
        Ok(e) => e,
        Err(e) => {
            error!("snapshot: venc jpeg init: {:?}", e);
            return 0;
        }
    };

    let n = match enc.encode(&h264::Frame::packed(frame), out) {
        Ok(n) => n,
        Err(e) => {
            error!("snapshot: jpeg encode: {:?}", e);
            return 0;
        }
    };

    let mut name = heapless::String::<13>::new();
    let _ = write!(name, "IMG{:04}.JPG", file_idx);

    let volume = match volume_mgr.open_volume(VolumeIdx(0)) {
        Ok(v) => v,
        Err(e) => {
            error!("snapshot: open_volume: {:?}", defmt::Debug2Format(&e));
            return 0;
        }
    };
    let root = match volume.open_root_dir() {
        Ok(r) => r,
        Err(e) => {
            error!("snapshot: open_root_dir: {:?}", defmt::Debug2Format(&e));
            return 0;
        }
    };
    let file = match root.open_file_in_dir(name.as_str(), Mode::ReadWriteCreateOrTruncate) {
        Ok(f) => f,
        Err(e) => {
            error!("snapshot: create {}: {:?}", name.as_str(), defmt::Debug2Format(&e));
            return 0;
        }
    };

    let ok = file.write(&out[..n]).is_ok();
    let _ = file.close();
    let _ = root.close();
    let _ = volume.close();
    if ok { n } else { 0 }
}

/// Highest `IMG####`/`VID####` index on the card, plus one.
fn scan_next_index<'a, 'b>(volume_mgr: &mut VolumeManager<EmbassyBlockDevice<'a, 'b>, FixedTime>) -> u32 {
    let mut max: i64 = -1;
    let res: Result<(), embedded_sdmmc::Error<embassy_stm32::sdmmc::Error>> = (|| {
        let volume = volume_mgr.open_volume(VolumeIdx(0))?;
        let root = volume.open_root_dir()?;
        root.iterate_dir(|entry| {
            let name = entry.name.base_name();
            if name.len() == 7 && (name.starts_with(b"IMG") || name.starts_with(b"VID")) {
                if let Ok(s) = core::str::from_utf8(&name[3..]) {
                    if let Ok(n) = s.parse::<u32>() {
                        if n as i64 > max {
                            max = n as i64;
                        }
                    }
                }
            }
        })?;
        Ok(())
    })();
    if let Err(e) = res {
        error!("scan_next_index: {:?}", defmt::Debug2Format(&e));
    }
    (max + 1) as u32
}

// ---- BlockDevice glue (same as `bin/camera_to_sd.rs`) ----

struct EmbassyBlockDevice<'a, 'b> {
    inner: RefCell<StorageDevice<'a, 'b, Card>>,
}

impl<'a, 'b> BlockDevice for EmbassyBlockDevice<'a, 'b> {
    type Error = embassy_stm32::sdmmc::Error;

    fn read(&self, blocks: &mut [Block], start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        let mut inner = self.inner.borrow_mut();
        for (i, block) in blocks.iter_mut().enumerate() {
            let mut data = DataBlock([0u32; 128]);
            block_on(inner.read_block(start_block_idx.0 + i as u32, &mut data))?;
            // DataBlock is repr-transparent over [u32; 128] = 512 bytes.
            // SAFETY: same size, properly aligned.
            unsafe {
                core::ptr::copy_nonoverlapping(data.0.as_ptr() as *const u8, block.contents.as_mut_ptr(), 512);
            }
        }
        Ok(())
    }

    fn write(&self, blocks: &[Block], start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        let mut inner = self.inner.borrow_mut();
        for (i, block) in blocks.iter().enumerate() {
            let mut data = DataBlock([0u32; 128]);
            unsafe {
                core::ptr::copy_nonoverlapping(block.contents.as_ptr(), data.0.as_mut_ptr() as *mut u8, 512);
            }
            block_on(inner.write_block(start_block_idx.0 + i as u32, &data))?;
        }
        Ok(())
    }

    fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        let inner = self.inner.borrow();
        let bytes = inner.card().size();
        Ok(BlockCount((bytes / 512) as u32))
    }
}

struct FixedTime;
impl TimeSource for FixedTime {
    fn get_timestamp(&self) -> Timestamp {
        // 2026-01-01 00:00:00 — fine for FAT timestamps in a demo.
        Timestamp {
            year_since_1970: 56,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

// ---- RCC + RIFSC + SRAM helpers ----

fn rcc_config() -> Config {
    let mut config = Config::default();
    // The STM32N6570-DK supplies VCORE from an external SMPS (UM3300 Table 6).
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

/// Grant the bus masters this example uses access to the SRAM regions.
///
/// The VENC reads the frame buffer and its own arena through its AXI master,
/// so it needs the same treatment as the DCMIPP and LTDC. Boards with RIF
/// isolation left at the reset default additionally need the matching RISAF
/// region filters opened for the VENC master compartment.
fn promote_masters_to_secure() {
    for rif_master in [RifMaster::Dcmipp, RifMaster::Venc] {
        rif_master.set_attributes(&RifMasterAttributes::new(1, true, true));
    }

    for rif_periph in [RifPeripheral::Dcmipp, RifPeripheral::Venc] {
        rif_periph.set_attributes(&RifPeripheralAttributes::new(true, true));
    }
}
