#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::time::Hertz;
use embassy_stm32::{peripherals, Config};
use embassy_stm32::sai::{self, Sai};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use embedded_sdmmc::{SdCard, VolumeManager};
use embedded_sdmmc::filesystem::ShortFileName;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::{self, Spi};

// This example plays WAV files from microSD to SAI.

type SdSpiDev = ExclusiveDevice<Spi<'static, embassy_stm32::mode::Async>, Output<'static>, NoDelay>;
type SdCardDev = SdCard<SdSpiDev, embassy_time::Delay>;

#[embassy_executor::task]
async fn sd_stream_task(
    mut sai_tx: Sai<'static, peripherals::SAI1, u16>,
    volume_mgr: &'static mut VolumeManager<SdCardDev, DummyTimesource>,
) {
    info!("SD stream task started");
    // Open first FAT volume and stream WAV files back-to-back
    let raw_vol = match volume_mgr.open_raw_volume(embedded_sdmmc::VolumeIdx(0)) {
        Ok(v) => v,
        Err(e) => {
            warn!("SD open_raw_volume error: {:?}", defmt::Debug2Format(&e));
            return;
        }
    };
    let raw_root = match volume_mgr.open_root_dir(raw_vol) {
        Ok(r) => r,
        Err(e) => {
            warn!("SD open_root_dir error: {:?}", defmt::Debug2Format(&e));
            return;
        }
    };
    // Collect WAV 8.3 names in root
    let mut names: heapless::Vec<ShortFileName, 32> = heapless::Vec::new();
    let _ = volume_mgr.iterate_dir(raw_root, |de| {
        if !de.attributes.is_directory() && de.name.extension() == b"WAV" {
            let _ = names.push(de.name.clone());
        }
    });

    // Helper: parse minimal WAV header
    fn parse_wav_header(h: &[u8]) -> Option<(u32, u16, u16, usize)> {
        if h.len() < 44 { return None; }
        if &h[0..4] != b"RIFF" || &h[8..12] != b"WAVE" { return None; }
        if &h[12..16] != b"fmt " { return None; }
        let fmt_size = u32::from_le_bytes([h[16],h[17],h[18],h[19]]);
        if fmt_size < 16 { return None; }
        let audio_fmt = u16::from_le_bytes([h[20],h[21]]);
        let channels = u16::from_le_bytes([h[22],h[23]]);
        let sample_rate = u32::from_le_bytes([h[24],h[25],h[26],h[27]]);
        let bits = u16::from_le_bytes([h[34],h[35]]);
        // Assume data chunk starts at 44 for PCM
        if audio_fmt != 1 { return None; }
        Some((sample_rate, channels, bits, 44))
    }

    // Simple 44.1k -> 48k linear resampler for interleaved u16 samples (all channels)
    fn resample_441_to_480<'a>(in_samples: &'a [u16], out_buf: &mut heapless::Vec<u16, 1024>, channels: usize) {
        // ratio = 160/147
        let frames_in = in_samples.len() / channels;
        let frames_out = (frames_in as u32 * 160 / 147) as usize;
        out_buf.clear();
        if frames_in < 2 { return; }
        for fo in 0..frames_out {
            let pos_num = fo as u32 * 147;
            let i0 = (pos_num / 160) as usize;
            let frac_num = (pos_num % 160) as u32; // 0..159
            let i1 = (i0 + 1).min(frames_in - 1);
            for ch in 0..channels {
                let s0 = in_samples[i0*channels + ch] as i32;
                let s1 = in_samples[i1*channels + ch] as i32;
                let interp = s0 * (160 - frac_num) as i32 + s1 * frac_num as i32;
                let val = (interp / 160) as i32;
                let _ = out_buf.push(val.clamp(0, 0xFFFF) as u16);
            }
        }
    }

    // Play each WAV file in order
    for name in names.iter() {
        let mut hdr = [0u8; 44];
        let raw_file = match volume_mgr.open_file_in_dir(raw_root, name.clone(), embedded_sdmmc::Mode::ReadOnly) {
            Ok(f) => f,
            Err(_) => continue,
        };
        if volume_mgr.read(raw_file, &mut hdr).unwrap_or(0) < 44 { let _ = volume_mgr.close_file(raw_file); continue; }
        let (sr, ch, bits, _data_off) = match parse_wav_header(&hdr) { Some(v) => v, None => continue };
        if bits != 16 { warn!("Skipping non-16bit WAV"); continue; }
        let channels = ch as usize;

        let mut file_buf = [0u8; 1024];
        let mut in_samples: heapless::Vec<u16, 512> = heapless::Vec::new();
        let mut out_samples: heapless::Vec<u16, 1024> = heapless::Vec::new();

        loop {
            let n = match volume_mgr.read(raw_file, &mut file_buf) { Ok(n) => n, Err(_) => 0 };
            if n == 0 { break; }
            in_samples.clear();
            let mut i = 0;
            while i + 1 < n && in_samples.len() < in_samples.capacity() {
                let s = u16::from_le_bytes([file_buf[i], file_buf[i+1]]);
                let _ = in_samples.push(s);
                i += 2;
            }

            if sr == 44_100 {
                resample_441_to_480(&in_samples, &mut out_samples, channels);
                if channels == 1 {
                    // Duplicate mono to stereo
                    let mut stereo: heapless::Vec<u16, 1024> = heapless::Vec::new();
                    for &s in out_samples.iter() {
                        if stereo.len() + 1 >= stereo.capacity() { break; }
                        let _ = stereo.push(s);
                        let _ = stereo.push(s);
                    }
                    let _ = sai_tx.write(&stereo).await;
                } else {
                    let _ = sai_tx.write(&out_samples).await;
                }
            } else {
                if channels == 1 {
                    // Duplicate mono to stereo
                    let mut stereo: heapless::Vec<u16, 1024> = heapless::Vec::new();
                    for &s in in_samples.iter() {
                        if stereo.len() + 1 >= stereo.capacity() { break; }
                        let _ = stereo.push(s);
                        let _ = stereo.push(s);
                    }
                    let _ = sai_tx.write(&stereo).await;
                } else {
                    let _ = sai_tx.write(&in_samples).await;
                }
            }
        }
        let _ = volume_mgr.close_file(raw_file);
    }
}

struct DummyTimesource();
impl embedded_sdmmc::TimeSource for DummyTimesource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp { year_since_1970: 0, zero_indexed_month: 0, zero_indexed_day: 0, hours: 0, minutes: 0, seconds: 0 }
    }
}

// (USB tasks removed for SD-only example)

/// Feedback value measurement and calculation
///
/// Used for measuring/calculating the number of samples that were received from the host during the
/// `FEEDBACK_REFRESH_PERIOD`.
///
/// Configured in this example with
/// - a refresh period of 8 ms, and
/// - a tick rate of 42 MHz.
///
/// This gives an (ideal) counter value of 336.000 for every update of the `FEEDBACK_SIGNAL`.
// (USB SOF feedback timer interrupt removed)

// If you are trying this and your USB device doesn't connect, the most
// common issues are the RCC config and vbus_detection
//
// See https://embassy.dev/book/#_the_usb_examples_are_not_working_on_my_board_is_there_anything_else_i_need_to_configure
// for more information.
#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::*;
        // Configure PLL1 to generate:
        // - PLL1_R = 96 MHz (system clock)
        // - PLL1_Q = 48 MHz
        // - PLL1_P = 16 MHz (USB OTG HS reference clock)
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,
            prediv: PllPreDiv::DIV1,
            mul: PllMul::MUL30,
            divr: Some(PllDiv::DIV5),  // 96 MHz Sysclk
            divq: Some(PllDiv::DIV10), // 48 MHz
            divp: Some(PllDiv::DIV30), // 16 MHz (USB OTG HS reference)
            frac: Some(0),
        });

        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        config.rcc.apb7_pre = APBPrescaler::DIV1;
        config.rcc.ahb5_pre = AHB5Prescaler::DIV4;

        config.rcc.voltage_scale = VoltageScale::RANGE1;
        config.rcc.mux.otghssel = mux::Otghssel::PLL1_P;
        // Route SAI1 kernel clock from PLL1_Q (48 MHz)
        config.rcc.mux.sai1sel = mux::Sai1sel::PLL1_Q;
        config.rcc.sys = Sysclk::PLL1_R;
    }
    let p = embassy_stm32::init(config);

    info!("Hello World!");

    // Configure all required buffers in a static way.
    // (USB configuration removed; SD-to-SAI only)

    // Initialize SAI1 (I2S) for playback: SCK=PA7, FS=PA8, SD=PB12, DMA=GPDMA1_CH0
    let (sai_a, _sai_b) = sai::split_subblocks(p.SAI1);

    static SAI_DMA_BUF: StaticCell<[u16; 2048]> = StaticCell::new();
    let sai_dma_buf = SAI_DMA_BUF.init([0u16; 2048]);

    let mut sai_cfg = sai::Config::default();
    sai_cfg.mode = sai::Mode::Master;
    sai_cfg.tx_rx = sai::TxRx::Transmitter;
    sai_cfg.stereo_mono = sai::StereoMono::Stereo;
    sai_cfg.data_size = sai::DataSize::Data16;
    sai_cfg.bit_order = sai::BitOrder::MsbFirst;
    // I2S framing compatible with common I2S DACs/codecs (32 bits per channel = 64 BCLK per LRCLK)
    sai_cfg.frame_sync_polarity = sai::FrameSyncPolarity::ActiveHigh;
    sai_cfg.frame_sync_offset = sai::FrameSyncOffset::OnFirstBit;
    sai_cfg.frame_length = 64;
    sai_cfg.frame_sync_active_level_length = sai::word::U7(32);
    sai_cfg.fifo_threshold = sai::FifoThreshold::Quarter;

    let sai_tx = Sai::new_asynchronous(
        sai_a,
        p.PA7,   // SCK  (BCLK)
        p.PB12,  // SD   (data)
        p.PA8,   // FS   (LRC / WS)
        p.GPDMA1_CH0,
        sai_dma_buf,
        sai_cfg,
    );

    // Set up SPI for microSD (SPI1: SCK=PB4, MISO=PB3, MOSI=PA15; CS=PA6). Adjust to your board wiring.
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = Hertz(400_000); // init at <= 400kHz
    let spi = Spi::new(
        p.SPI1,
        p.PB4,  // SCK
        p.PA15, // MOSI
        p.PB3,  // MISO
        p.GPDMA1_CH4,
        p.GPDMA1_CH5,
        spi_cfg,
    );
    let cs = Output::new(p.PA6, Level::High, Speed::VeryHigh);
    let spi_dev: SdSpiDev = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let sd: SdCardDev = SdCard::new(spi_dev, embassy_time::Delay);
    // Give the card some time after power-up and provide at least 80 dummy clocks with CS high.
    embassy_time::Timer::after_millis(50).await;
    // Provide at least 80 dummy clocks with CS high. Use a local buffer to avoid lifetime issues.
    sd.spi(|dev| {
        let dummy = [0xFFu8; 10];
        let _ = dev.bus_mut().write(&dummy);
    });
    info!("SD size {} bytes", sd.num_bytes().unwrap_or(0));

    // Bump SPI after init
    let mut spi_cfg2 = spi::Config::default();
    spi_cfg2.frequency = Hertz(16_000_000);
    let _ = sd.spi(|dev| dev.bus_mut().set_config(&spi_cfg2));

    static VOLUME_MANAGER: StaticCell<VolumeManager<SdCardDev, DummyTimesource>> = StaticCell::new();
    let vol_mgr: &'static mut VolumeManager<SdCardDev, DummyTimesource> =
        VOLUME_MANAGER.init(VolumeManager::new(sd, DummyTimesource()));

    // Start SD streaming task to play WAV files from microSD via SAI.
    spawner.spawn(sd_stream_task(sai_tx, vol_mgr).expect("spawn sd_stream_task"));
}
