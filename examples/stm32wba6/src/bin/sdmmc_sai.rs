#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::sai::{self, Sai};
use embassy_stm32::time::Hertz;
use embassy_stm32::{peripherals, Config};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use embedded_sdmmc::filesystem::ShortFileName;
use embedded_sdmmc::{SdCard, VolumeManager};
use static_cell::StaticCell;
use defmt::*;
use embassy_stm32::spi::{self, Spi};
use {defmt_rtt as _, panic_probe as _};

// This example plays WAV files from microSD to SAI.

type SdSpiDev = ExclusiveDevice<Spi<'static, embassy_stm32::mode::Async>, Output<'static>, NoDelay>;
type SdCardDev = SdCard<SdSpiDev, embassy_time::Delay>;

const USE_LEFT_JUSTIFIED: bool = false; // Set true to try Left-Justified framing

#[embassy_executor::task]
async fn sd_stream_task(
    mut sai_tx: Sai<'static, peripherals::SAI1, u16>,
    volume_mgr: &'static mut VolumeManager<SdCardDev, DummyTimesource>,
) {
    info!("SD stream task started");
    // Periodic log of last sample sent to SAI
    let mut next_log = embassy_time::Instant::now() + embassy_time::Duration::from_secs(1);
    let mut last_sample_written: i16 = 0;
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
    // Print WAV filenames discovered in root (8.3 format)
    for n in names.iter() {
        let base = n.base_name();
        let ext = n.extension();
        let base_end = base.iter().rposition(|&b| b != b' ').map(|i| i + 1).unwrap_or(0);
        let ext_end = ext.iter().rposition(|&b| b != b' ').map(|i| i + 1).unwrap_or(0);
        let base_str = core::str::from_utf8(&base[..base_end]).unwrap_or("?");
        if ext_end > 0 {
            let ext_str = core::str::from_utf8(&ext[..ext_end]).unwrap_or("");
            info!("WAV: {}.{}", base_str, ext_str);
        } else {
            info!("WAV: {}", base_str);
        }
    }

    // Helper: parse minimal (robust) WAV header: returns (sample_rate, channels, bits, data_offset)
    fn parse_wav_header(h: &[u8]) -> Option<(u32, u16, u16, usize)> {
        if h.len() < 44 { return None; }
        if &h[0..4] != b"RIFF" || &h[8..12] != b"WAVE" { return None; }
        let mut idx = 12;
        let mut fmt_found = false;
        let mut data_off = None;
        let mut sample_rate = 0u32;
        let mut channels = 0u16;
        let mut bits = 0u16;
        while idx + 8 <= h.len() {
            let id = &h[idx..idx+4];
            let sz = u32::from_le_bytes([h[idx+4],h[idx+5],h[idx+6],h[idx+7]]) as usize;
            idx += 8;
            if idx + sz > h.len() { break; }
            if id == b"fmt " {
                if sz < 16 { return None; }
                let audio_fmt = u16::from_le_bytes([h[idx],h[idx+1]]);
                channels = u16::from_le_bytes([h[idx+2],h[idx+3]]);
                sample_rate = u32::from_le_bytes([h[idx+4],h[idx+5],h[idx+6],h[idx+7]]);
                bits = u16::from_le_bytes([h[idx+14],h[idx+15]]);
                if audio_fmt != 1 { return None; }
                fmt_found = true;
            } else if id == b"data" {
                data_off = Some(idx);
                break;
            }
            idx += sz;
        }
        if !fmt_found { return None; }
        let doff = data_off?;
        Some((sample_rate, channels, bits, doff))
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
        let mut hdr = [0u8; 256]; // allow bigger header for extra chunks
        let raw_file = match volume_mgr.open_file_in_dir(raw_root, name.clone(), embedded_sdmmc::Mode::ReadOnly) {
            Ok(f) => f,
            Err(_) => continue,
        };
        if volume_mgr.read(raw_file, &mut hdr).unwrap_or(0) < 44 { let _ = volume_mgr.close_file(raw_file); continue; }
        let (sr, ch, bits, data_off) = match parse_wav_header(&hdr) { Some(v) => v, None => { let _=volume_mgr.close_file(raw_file); continue } };
        if bits != 16 { warn!("Skip non-16b"); let _=volume_mgr.close_file(raw_file); continue; }
        if sr != 44_100 && sr != 48_000 { warn!("Skip sr {}", sr); let _=volume_mgr.close_file(raw_file); continue; }
        let channels = ch as usize;

        // Seek to data offset if header larger than buffer's first read
        if data_off > hdr.len() {
            // simplistic: unsupported very large header
            let _ = volume_mgr.close_file(raw_file);
            continue;
        } else if data_off > 256 {
            let _ = volume_mgr.close_file(raw_file);
            continue;
        } else {
            // If data_off > amount already consumed (e.g. >256) we'd need a seek API; not present here.
        }

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
                        if stereo.len() + 2 > stereo.capacity() { break; }
                        let _ = stereo.push(s);
                        let _ = stereo.push(s);
                    }
                    last_sample_written = *stereo.last().unwrap_or(&0) as i16;
                    match sai_tx.write(&stereo).await {
                        Ok(_) => {}
                        Err(e) => warn!("SAI write err {:?}", defmt::Debug2Format(&e)),
                    }
                } else {
                    last_sample_written = *out_samples.last().unwrap_or(&0) as i16;
                    match sai_tx.write(&out_samples).await {
                        Ok(_) => {}
                        Err(e) => warn!("SAI write err {:?}", defmt::Debug2Format(&e)),
                    }
                }
            } else {
                if channels == 1 {
                    // Duplicate mono to stereo
                    let mut stereo: heapless::Vec<u16, 1024> = heapless::Vec::new();
                    for &s in in_samples.iter() {
                        if stereo.len() + 2 > stereo.capacity() { break; }
                        let _ = stereo.push(s);
                        let _ = stereo.push(s);
                    }
                    last_sample_written = *stereo.last().unwrap_or(&0) as i16;
                    match sai_tx.write(&stereo).await {
                        Ok(_) => {}
                        Err(e) => warn!("SAI write err {:?}", defmt::Debug2Format(&e)),
                    }
                } else {
                    last_sample_written = *in_samples.last().unwrap_or(&0) as i16;
                    match sai_tx.write(&in_samples).await {
                        Ok(_) => {}
                        Err(e) => warn!("SAI write err {:?}", defmt::Debug2Format(&e)),
                    }
                }
            }

            // Log approximately once per second
            if embassy_time::Instant::now() >= next_log {
                info!("last sample written {}", last_sample_written);
                next_log += embassy_time::Duration::from_secs(1);
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


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();

    // Added/modified: provide an audio‑friendly SAI clock (49.152 MHz from PLL1_Q)
    {
        use embassy_stm32::rcc::*;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,          // 16 MHz
            prediv: PllPreDiv::DIV1,         // M
            mul: PllMul::MUL12,              // Integer N (12)
            // Fraction gives: 16 * (12 + 2363/8192) ≈ 196.608 MHz VCO
            divq: Some(PllDiv::DIV4),        // 196.608 / 4 = 49.152 MHz -> SAI kernel
            divr: Some(PllDiv::DIV5),        // System clock path (adjust if you need a higher HCLK)
            divp: Some(PllDiv::DIV30),
            frac: Some(2363),
        });
        
        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        config.rcc.apb7_pre = APBPrescaler::DIV1;
        config.rcc.ahb5_pre = AHB5Prescaler::DIV4;

        config.rcc.voltage_scale = VoltageScale::RANGE1;

        config.rcc.mux.sai1sel = mux::Sai1sel::PLL1_Q;
    }

    let p = embassy_stm32::init(config);

    info!("Hello World!");

    // Configure all required buffers in a static way.
    // (USB configuration removed; SD-to-SAI only)

    // Initialize SAI1 (I2S) for playback: SCK=PA7, FS=PA8, SD=PB12, DMA=GPDMA1_CH0
    let (sai_a, _sai_b) = sai::split_subblocks(p.SAI1);

    static SAI_DMA_BUF: StaticCell<[u16; 4096]> = StaticCell::new();
    let sai_dma_buf = SAI_DMA_BUF.init([0u16; 4096]);

    let mut sai_cfg = sai::Config::default();
    sai_cfg.mode = sai::Mode::Master;
    sai_cfg.tx_rx = sai::TxRx::Transmitter;
    sai_cfg.stereo_mono = sai::StereoMono::Stereo;
    sai_cfg.data_size = sai::DataSize::Data16;
    sai_cfg.bit_order = sai::BitOrder::MsbFirst;
    // Explicit slots: 2x32-bit with both L+R enabled
    sai_cfg.slot_size = sai::SlotSize::Channel32;
    sai_cfg.slot_count = sai::word::U4(2);
    sai_cfg.slot_enable = 0b11;
    sai_cfg.first_bit_offset = sai::word::U5(0);
    // Configure framing
    if USE_LEFT_JUSTIFIED {
        // Left-Justified: WS high = left, data MSB on WS edge
        sai_cfg.frame_sync_polarity = sai::FrameSyncPolarity::ActiveHigh;
        sai_cfg.frame_sync_offset = sai::FrameSyncOffset::OnFirstBit;
    } else {
        // I2S: WS low = left, data MSB one BCLK after WS edge
        sai_cfg.frame_sync_polarity = sai::FrameSyncPolarity::ActiveLow;
        sai_cfg.frame_sync_offset = sai::FrameSyncOffset::BeforeFirstBit;
    }
    sai_cfg.frame_length = 64;
    sai_cfg.frame_sync_active_level_length = sai::word::U7(32);
    sai_cfg.fifo_threshold = sai::FifoThreshold::Quarter;

    let sai_tx = Sai::new_asynchronous(
        sai_a,
        p.PA7,   // SCK  (BCLK)
        p.PB14,  // SD   (data)
        p.PA8,   // FS   (LRC / WS)
        p.GPDMA1_CH0,
        sai_dma_buf,
        sai_cfg,
    );

    // (PWM removed)

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
    spi_cfg2.frequency = Hertz(8_000_000);
    let _ = sd.spi(|dev| dev.bus_mut().set_config(&spi_cfg2));

    static VOLUME_MANAGER: StaticCell<VolumeManager<SdCardDev, DummyTimesource>> = StaticCell::new();
    let vol_mgr: &'static mut VolumeManager<SdCardDev, DummyTimesource> =
        VOLUME_MANAGER.init(VolumeManager::new(sd, DummyTimesource()));


    spawner.spawn(unwrap!(sd_stream_task(sai_tx, vol_mgr)));

}
