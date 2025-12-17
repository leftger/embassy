#![no_std]
#![no_main]

use defmt::*;
use embassy_executor::Spawner;
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::sai::{self, Sai};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{Config, peripherals};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use embedded_sdmmc::filesystem::ShortFileName;
use embedded_sdmmc::{SdCard, VolumeManager};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

// This example plays WAV files from microSD to SAI.

// Volume scaling factor (0.0 = silent, 1.0 = full volume)
const VOLUME_SCALE: f32 = 0.5; // 37.5% volume (30% + 25% increase)

type SdSpiDev = ExclusiveDevice<
    Spi<'static, embassy_stm32::mode::Async, embassy_stm32::spi::mode::Master>,
    Output<'static>,
    NoDelay,
>;
type SdCardDev = SdCard<SdSpiDev, embassy_time::Delay>;

const USE_LEFT_JUSTIFIED: bool = false; // Back to I2S - sounded better before
const USE_EMBEDDED_WAV: bool = false; // Back to SD card, but debug the issue

// Embedded 1kHz sine wave WAV file (48kHz, 16-bit, stereo, ~2 seconds)
const EMBEDDED_WAV: &[u8] = &[
    // WAV header (RIFF chunk)
    b'R', b'I', b'F', b'F', // RIFF
    0x24, 0x08, 0x00, 0x00, // File size (2084 bytes)
    b'W', b'A', b'V', b'E', // WAVE
    // Format chunk
    b'f', b'm', b't', b' ', // fmt
    16, 0, 0, 0, // Chunk size (16)
    1, 0, // Audio format (PCM)
    2, 0, // Channels (stereo)
    0x80, 0xBB, 0x00, 0x00, // Sample rate (48000)
    0x00, 0xEE, 0x02, 0x00, // Byte rate (48000 * 2 * 2)
    4, 0, // Block align (2 * 2)
    16, 0, // Bits per sample
    // Data chunk
    b'd', b'a', b't', b'a', // data
    0x00, 0x02, 0x00, 0x00, // Data size (512 bytes = 128 samples * 4 bytes)
];

// Precomputed sine wave lookup table (256 samples per cycle)
const SINE_TABLE_EMBEDDED: [i16; 256] = [
    0, 804, 1608, 2410, 3212, 4011, 4808, 5602, 6393, 7179, 7962, 8739, 9512, 10278, 11039, 11793, 12539, 13279, 14010,
    14732, 15446, 16151, 16846, 17530, 18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594, 23170, 23731, 24279,
    24811, 25329, 25832, 26319, 26790, 27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956, 30273, 30571, 30852,
    31113, 31356, 31580, 31785, 31971, 32137, 32285, 32412, 32520, 32609, 32678, 32728, 32757, 32767, 32757, 32728,
    32678, 32609, 32520, 32412, 32285, 32137, 31971, 31785, 31580, 31356, 31113, 30852, 30571, 30273, 29956, 29621,
    29268, 28898, 28510, 28105, 27683, 27245, 26790, 26319, 25832, 25329, 24811, 24279, 23731, 23170, 22594, 22005,
    21403, 20787, 20159, 19519, 18868, 18204, 17530, 16846, 16151, 15446, 14732, 14010, 13279, 12539, 11793, 11039,
    10278, 9512, 8739, 7962, 7179, 6393, 5602, 4808, 4011, 3212, 2410, 1608, 804, 0, -804, -1608, -2410, -3212, -4011,
    -4808, -5602, -6393, -7179, -7962, -8739, -9512, -10278, -11039, -11793, -12539, -13279, -14010, -14732, -15446,
    -16151, -16846, -17530, -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594, -23170, -23731, -24279,
    -24811, -25329, -25832, -26319, -26790, -27245, -27683, -28105, -28510, -28898, -29268, -29621, -29956, -30273,
    -30571, -30852, -31113, -31356, -31580, -31785, -31971, -32137, -32285, -32412, -32520, -32609, -32678, -32728,
    -32757, -32767, -32757, -32728, -32678, -32609, -32520, -32412, -32285, -32137, -31971, -31785, -31580, -31356,
    -31113, -30852, -30571, -30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683, -27245, -26790, -26319,
    -25832, -25329, -24811, -24279, -23731, -23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868, -18204,
    -17530, -16846, -16151, -15446, -14732, -14010, -13279, -12539, -11793, -11039, -10278, -9512, -8739, -7962, -7179,
    -6393, -5602, -4808, -4011, -3212, -2410, -1608, -804,
];

// Generate 256 samples of 1kHz sine wave (about 5.3ms at 48kHz)
fn generate_sine_samples() -> [u16; 1024] {
    let mut samples = [0u16; 1024];

    for i in 0..512 {
        // Get sine value from lookup table (1kHz at 48kHz = 48 samples per cycle)
        // We want 1000Hz at 48000Hz, so 48 samples per cycle
        // Use table index based on position in cycle
        let table_index = ((i as u32 * 256) / 48) % 256;
        let sine_val = SINE_TABLE_EMBEDDED[table_index as usize];

        // Convert to unsigned with offset (0x8000)
        let unsigned_sample = (sine_val as i32 + 0x8000) as u16;

        // Stereo: same sample for left and right
        samples[i * 2] = unsigned_sample;
        samples[i * 2 + 1] = unsigned_sample;
    }

    samples
}

// Play the embedded WAV data
async fn play_embedded_wav(mut sai_tx: Sai<'static, peripherals::SAI1, u16>) {
    info!("Playing embedded sine wave WAV");

    // Parse the embedded WAV header (simplified - we know the format)
    let _header = &EMBEDDED_WAV[..44]; // WAV header is 44 bytes
    let data_start = 44;
    let data_size = EMBEDDED_WAV.len() - data_start;

    info!(
        "Embedded WAV: {} bytes total, {} bytes audio data",
        EMBEDDED_WAV.len(),
        data_size
    );

    // Generate the sine wave samples
    let sine_samples = generate_sine_samples();

    // Play the sine wave in a loop
    loop {
        // Send samples to SAI (512 stereo samples = 1024 u16 values)
        match sai_tx.write(&sine_samples).await {
            Ok(_) => {
                // Short delay to prevent FIFO overruns
                embassy_time::Timer::after_micros(10).await;
                info!("Embedded WAV SAI write OK");
            }
            Err(e) => warn!("Embedded WAV SAI write err {:?}", defmt::Debug2Format(&e)),
        }

        // Log the first few sample values for debugging
        info!(
            "Embedded WAV samples: L={}, R={}, L={}, R={}",
            sine_samples[0], sine_samples[1], sine_samples[2], sine_samples[3]
        );
    }
}

#[embassy_executor::task]
async fn sd_stream_task(
    mut sai_tx: Sai<'static, peripherals::SAI1, u16>,
    volume_mgr: &'static mut VolumeManager<SdCardDev, DummyTimesource>,
) {
    info!("SD stream task started");
    // Periodic log of last sample sent to SAI
    let mut next_log = embassy_time::Instant::now() + embassy_time::Duration::from_secs(1);
    #[allow(unused_assignments)]
    let mut last_sample_written: i16 = 0;
    if USE_EMBEDDED_WAV {
        info!("Using embedded sine wave WAV data");
        // Use embedded WAV data instead of SD card
        play_embedded_wav(sai_tx).await;
        return;
    }

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
    // Collect WAV and PCM 8.3 names in root
    let mut names: heapless::Vec<ShortFileName, 32> = heapless::Vec::new();
    let _ = volume_mgr.iterate_dir(raw_root, |de| {
        if !de.attributes.is_directory() {
            let ext = de.name.extension();
            if ext == b"WAV" || ext == b"PCM" {
                let _ = names.push(de.name.clone());
            }
        }
    });
    // Print audio filenames discovered in root (8.3 format)
    for n in names.iter() {
        let base = n.base_name();
        let ext = n.extension();
        let base_end = base.iter().rposition(|&b| b != b' ').map(|i| i + 1).unwrap_or(0);
        let ext_end = ext.iter().rposition(|&b| b != b' ').map(|i| i + 1).unwrap_or(0);
        let base_str = core::str::from_utf8(&base[..base_end]).unwrap_or("?");
        if ext_end > 0 {
            let ext_str = core::str::from_utf8(&ext[..ext_end]).unwrap_or("");
            info!("Audio: {}.{} ({})", base_str, ext_str,
                  if ext == b"PCM" { "raw DMA" } else { "WAV" });
        } else {
            info!("Audio: {}", base_str);
        }
    }

    // Helper: parse minimal (robust) WAV header: returns (sample_rate, channels, bits, data_offset)
    fn parse_wav_header(h: &[u8]) -> Option<(u32, u16, u16, usize)> {
        if h.len() < 44 {
            return None;
        }
        if &h[0..4] != b"RIFF" || &h[8..12] != b"WAVE" {
            return None;
        }
        let mut idx = 12;
        let mut fmt_found = false;
        let mut data_off = None;
        let mut sample_rate = 0u32;
        let mut channels = 0u16;
        let mut bits = 0u16;
        let mut chunk_count = 0;
        while idx + 8 <= h.len() {
            let id = &h[idx..idx + 4];
            let sz = u32::from_le_bytes([h[idx + 4], h[idx + 5], h[idx + 6], h[idx + 7]]) as usize;
            chunk_count += 1;
            let chunk_name = core::str::from_utf8(id).unwrap_or("????");
            info!("Chunk {}: {} at idx={}, size={}", chunk_count, chunk_name, idx, sz);
            idx += 8;
            if idx + sz > h.len() {
                if id == b"data" {
                    info!(
                        "Found data chunk but size {} exceeds buffer - data starts at file offset {}",
                        sz, idx
                    );
                    // For data chunks that are too big, we can still use the offset
                    data_off = Some(idx);
                    break;
                } else {
                    info!(
                        "Chunk {}: {} size {} exceeds buffer (idx={}, sz={}, h.len={}) - stopping parse",
                        chunk_count,
                        chunk_name,
                        sz,
                        idx,
                        sz,
                        h.len()
                    );
                    break;
                }
            }
            if id == b"fmt " {
                info!("Found fmt chunk, size: {}", sz);
                if sz < 16 {
                    info!("fmt chunk too small: {}", sz);
                    return None;
                }
                let audio_fmt = u16::from_le_bytes([h[idx], h[idx + 1]]);
                channels = u16::from_le_bytes([h[idx + 2], h[idx + 3]]);
                sample_rate = u32::from_le_bytes([h[idx + 4], h[idx + 5], h[idx + 6], h[idx + 7]]);
                bits = u16::from_le_bytes([h[idx + 14], h[idx + 15]]);
                info!(
                    "fmt: format={}, channels={}, rate={}, bits={}",
                    audio_fmt, channels, sample_rate, bits
                );
                if audio_fmt != 1 {
                    info!("Unsupported audio format: {}", audio_fmt);
                    return None;
                }
                fmt_found = true;
            } else if id == b"data" {
                info!("Found data chunk at offset {}, size: {}", idx, sz);
                data_off = Some(idx);
                break;
            } else {
                let chunk_name = core::str::from_utf8(id).unwrap_or("????");
                info!("Skipping unknown chunk: {} (size: {})", chunk_name, sz);
            }
            idx += sz;
        }
        info!(
            "Parsed {} chunks, fmt_found: {}, data_off: {:?}",
            chunk_count, fmt_found, data_off
        );
        if !fmt_found {
            return None;
        }
        let doff = data_off?;
        Some((sample_rate, channels, bits, doff))
    }

    // Simple 44.1k -> 48k linear resampler for interleaved u16 samples (all channels)
    #[allow(dead_code)]
    fn resample_441_to_480<'a>(in_samples: &'a [u16], out_buf: &mut heapless::Vec<u16, 1024>, channels: usize) {
        // ratio = 160/147
        let frames_in = in_samples.len() / channels;
        let frames_out = (frames_in as u32 * 160 / 147) as usize;
        out_buf.clear();
        if frames_in < 2 {
            return;
        }
        for fo in 0..frames_out {
            let pos_num = fo as u32 * 147;
            let i0 = (pos_num / 160) as usize;
            let frac_num = (pos_num % 160) as u32; // 0..159
            let i1 = (i0 + 1).min(frames_in - 1);
            for ch in 0..channels {
                // Convert unsigned samples back to signed for interpolation
                let s0 = (in_samples[i0 * channels + ch] as i32) - 0x8000;
                let s1 = (in_samples[i1 * channels + ch] as i32) - 0x8000;
                let interp = s0 * (160 - frac_num) as i32 + s1 * frac_num as i32;
                let val = (interp / 160) as i32;
                // Convert back to unsigned with offset
                let unsigned_val = (val + 0x8000).clamp(0, 0xFFFF) as u16;
                let _ = out_buf.push(unsigned_val);
            }
        }
    }

    // Play each WAV file in order
    for name in names.iter() {
        info!("Trying to open file: {:?}", defmt::Debug2Format(name));
        let mut hdr = [0u8; 2048]; // allow bigger header for extra chunks
        let raw_file = match volume_mgr.open_file_in_dir(raw_root, name.clone(), embedded_sdmmc::Mode::ReadOnly) {
            Ok(f) => {
                info!("Successfully opened file");
                f
            }
            Err(e) => {
                warn!("Failed to open file: {:?}", defmt::Debug2Format(&e));
                continue;
            }
        };
        let header_bytes = match volume_mgr.read(raw_file, &mut hdr) {
            Ok(n) => n,
            Err(e) => {
                warn!("Failed to read header: {:?}", defmt::Debug2Format(&e));
                let _ = volume_mgr.close_file(raw_file);
                continue;
            }
        };
        let is_pcm = name.extension() == b"PCM";

        if is_pcm {
            info!("🎵 Raw PCM file detected - direct DMA streaming (no processing!)");
            // PCM files are raw unsigned 16-bit samples, assume 48kHz mono
            // No header parsing needed, start streaming from offset 0
            let sr = 48_000;
            info!("Starting PCM playback: {}Hz mono (raw unsigned 16-bit, DIV4 = 48kHz LRCLK)", sr);

            let mut file_buf = [0u8; 512];
            let mut total_samples_read = 0u32;
            let mut current_file_offset = 0u32;

            // Direct streaming loop - no processing needed!
            loop {
                let n = match volume_mgr.read(raw_file, &mut file_buf) {
                    Ok(n) => n,
                    Err(e) => {
                        warn!("Error reading PCM data: {:?}", defmt::Debug2Format(&e));
                        break;
                    }
                };
                if n == 0 {
                    info!("End of PCM file reached, total samples: {}", total_samples_read);
                    break;
                }
                current_file_offset += n as u32;
                total_samples_read += (n / 2) as u32; // 2 bytes per sample

                // Convert raw bytes to u16 samples with volume scaling
                let mut samples: heapless::Vec<u16, 256> = heapless::Vec::new();
                let mut i = 0;
                while i + 1 < n && samples.len() < samples.capacity() {
                    let sample = u16::from_le_bytes([file_buf[i], file_buf[i + 1]]);
                    // Apply volume scaling
                    let scaled_sample = ((sample as f32) * VOLUME_SCALE) as u16;
                    let _ = samples.push(scaled_sample);
                    i += 2;
                }

                // Send directly to SAI - no processing overhead!
                match sai_tx.write(&samples).await {
                    Ok(_) => info!("PCM DMA write OK: {} samples", samples.len()),
                    Err(e) => warn!("PCM DMA write err {:?}: {} samples", defmt::Debug2Format(&e), samples.len()),
                }

                // Small delay to prevent overruns
                embassy_time::Timer::after_micros(500).await;
            }

            let _ = volume_mgr.close_file(raw_file);
            continue; // Process next file
        } else {
            info!("Read {} header bytes", header_bytes);
            if header_bytes < 44 {
                warn!("Header too short: {} bytes", header_bytes);
                let _ = volume_mgr.close_file(raw_file);
                continue;
            }
            let wav_info = match parse_wav_header(&hdr) {
            Some((sr, ch, bits, data_off)) => {
                info!("Parsed WAV: {}Hz, {}ch, {}bit, data_offset={}", sr, ch, bits, data_off);
                (sr, ch, bits, data_off)
            }
            None => {
                warn!("Failed to parse WAV header");
                // Debug: show first 16 bytes of header
                info!(
                    "Header bytes: {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x} {:02x}",
                    hdr[0],
                    hdr[1],
                    hdr[2],
                    hdr[3],
                    hdr[4],
                    hdr[5],
                    hdr[6],
                    hdr[7],
                    hdr[8],
                    hdr[9],
                    hdr[10],
                    hdr[11],
                    hdr[12],
                    hdr[13],
                    hdr[14],
                    hdr[15]
                );
                let _ = volume_mgr.close_file(raw_file);
                continue;
            }
        };
        let (sr, ch, bits, data_off) = wav_info;
        if bits != 16 {
            warn!("Skip non-16bit file ({} bits)", bits);
            let _ = volume_mgr.close_file(raw_file);
            continue;
        }
        if sr != 44_100 && sr != 48_000 {
            warn!(
                "Skip unsupported sample rate: {} Hz (only 44100 or 48000 supported)",
                sr
            );
            let _ = volume_mgr.close_file(raw_file);
            continue;
        }
        info!("File validation passed: {}Hz, {}ch, {}bit", sr, ch, bits);
        info!("Starting playback of valid WAV file");
        let _channels = ch as usize;

        // Check if we can start streaming from the data offset
        if data_off <= hdr.len() {
            // Data starts within our header buffer
            info!("Data starts at offset {}, within header buffer", data_off);
        } else {
            // Data starts beyond our header buffer - this is normal for large files
            // We'll need to seek to the data offset when we start reading
            info!(
                "Data starts at offset {} (beyond header), will seek when streaming",
                data_off
            );
        }
        info!("Starting WAV playback: {}Hz, {}ch, {}bit", sr, ch, bits);

        let mut file_buf = [0u8; 512];
        let mut in_samples: heapless::Vec<u16, 256> = heapless::Vec::new();
        let mut total_samples_read = 0u32;
        let mut current_file_offset = hdr.len(); // We've read this much already
        let data_offset = data_off; // data_off is the offset

        // Skip data between header and data chunk if necessary
        if current_file_offset < data_offset {
            let skip_bytes = data_offset - current_file_offset;
            info!("Skipping {} bytes to reach data chunk", skip_bytes);
            let mut skip_buf = [0u8; 1024];
            let mut remaining = skip_bytes;
            while remaining > 0 {
                let to_read = remaining.min(1024);
                match volume_mgr.read(raw_file, &mut skip_buf[..to_read]) {
                    Ok(n) if n > 0 => {
                        remaining -= n;
                        current_file_offset += n;
                    }
                    Ok(_) => {
                        warn!("Unexpected end of file while skipping to data");
                        let _ = volume_mgr.close_file(raw_file);
                        return;
                    }
                    Err(e) => {
                        warn!("Error skipping to data: {:?}", defmt::Debug2Format(&e));
                        let _ = volume_mgr.close_file(raw_file);
                        return;
                    }
                }
            }
            info!("Successfully skipped to data offset {}", current_file_offset);
        }

        loop {
            let n = match volume_mgr.read(raw_file, &mut file_buf) {
                Ok(n) => n,
                Err(e) => {
                    warn!("Error reading file data: {:?}", defmt::Debug2Format(&e));
                    0
                }
            };
            if n == 0 {
                info!("End of file reached, total samples: {}", total_samples_read);
                break;
            }
            current_file_offset += n;
            total_samples_read += (n / 2) as u32; // 2 bytes per sample
            in_samples.clear();
            let mut i = 0;
            while i + 1 < n && in_samples.len() < in_samples.capacity() {
                // WAV stores 16-bit PCM as signed little-endian
                let signed_sample = i16::from_le_bytes([file_buf[i], file_buf[i + 1]]);
                // Convert to unsigned with volume scaling
                let unsigned_sample = (signed_sample as i32 + 0x8000) as u16;
                let scaled_sample = ((unsigned_sample as f32) * VOLUME_SCALE) as u16;
                let _ = in_samples.push(scaled_sample);
                i += 2;
            }

            if sr == 44_100 {
                // Play 44.1kHz directly without resampling
                last_sample_written = *in_samples.last().unwrap_or(&0) as i16;
                info!("44.1kHz mono: {} samples (no resampling)", in_samples.len());
                match sai_tx.write(&in_samples).await {
                    Ok(_) => info!("SAI write OK (44.1kHz)"),
                    Err(e) => warn!("SAI write err {:?} (44.1kHz, {} samples)", defmt::Debug2Format(&e), in_samples.len()),
                }
                // Delay to prevent SAI buffer overruns
                embassy_time::Timer::after_micros(500).await;
            } else {
                last_sample_written = ((*in_samples.last().unwrap_or(&0) as i32) - 0x8000) as i16;
                info!("48k mono: {} samples", in_samples.len());
                match sai_tx.write(&in_samples).await {
                    Ok(_) => info!("SAI write OK"),
                    Err(e) => warn!("SAI write err {:?}", defmt::Debug2Format(&e)),
                }
                // Delay to prevent SAI buffer overruns
                embassy_time::Timer::after_micros(500).await;
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

    // If no valid WAV files found, generate a test tone
    info!("No valid WAV files found, generating test tone");
    // Generate a 1kHz sine wave at 48kHz sample rate
    let sample_rate = 48000;
    let frequency = 1000; // 1kHz
    let mut phase_step = 0u16;
    let phase_increment = ((frequency as u32 * 65536) / sample_rate as u32) as u16; // Fixed point phase increment

    // Simple sine lookup table (quarter wave, 64 entries)
    const SINE_TABLE: [i16; 64] = [
        0, 3212, 6393, 9512, 12539, 15446, 18204, 20787, 23170, 25329, 27245, 28898, 30273, 31356, 32137, 32609, 32767,
        32609, 32137, 31356, 30273, 28898, 27245, 25329, 23170, 20787, 18204, 15446, 12539, 9512, 6393, 3212, 0, -3212,
        -6393, -9512, -12539, -15446, -18204, -20787, -23170, -25329, -27245, -28898, -30273, -31356, -32137, -32609,
        -32767, -32609, -32137, -31356, -30273, -28898, -27245, -25329, -23170, -20787, -18204, -15446, -12539, -9512,
        -6393, -3212,
    ];

    let mut samples: heapless::Vec<u16, 1024> = heapless::Vec::new();

    // Generate continuous 1kHz sine wave
    loop {
        samples.clear();

        // Generate 128 samples (about 2.7ms at 48kHz) of sine wave
        for _ in 0..128 {
            // Get sine value from lookup table (0-65535 maps to 0-255 in table)
            let table_index = (phase_step >> 8) as usize & 0xFF;
            let sine_val = if table_index < 64 {
                SINE_TABLE[table_index]
            } else if table_index < 128 {
                SINE_TABLE[127 - table_index]
            } else if table_index < 192 {
                -SINE_TABLE[table_index - 128]
            } else {
                -SINE_TABLE[255 - table_index]
            };

            // Amplify sine wave by 4x, apply volume scaling, convert to unsigned 16-bit (offset by 32768)
            let amplified = (sine_val as i32) * 4;
            let clamped = amplified.clamp(-32768, 32767);
            let scaled_clamped = ((clamped as f32) * VOLUME_SCALE) as i32;
            let sample = (scaled_clamped + 32768) as u16;

            // Mono: single sample
            let _ = samples.push(sample);

            // Update phase (fixed point)
            phase_step = phase_step.wrapping_add(phase_increment);
        }

        last_sample_written = (samples[0] as i32 - 32768) as i16;

        info!("Writing {} sine wave samples", samples.len());
        match sai_tx.write(&samples).await {
            Ok(_) => info!("SAI write OK"),
            Err(e) => warn!("SAI write err {:?}", defmt::Debug2Format(&e)),
        }

        // Log approximately once per second
        if embassy_time::Instant::now() >= next_log {
            info!(
                "sine wave phase step: {}, last sample: {}",
                phase_step, last_sample_written
            );
            next_log += embassy_time::Duration::from_secs(1);
        }
    }
}

struct DummyTimesource();
impl embedded_sdmmc::TimeSource for DummyTimesource {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();

    // Added/modified: provide an audio‑friendly SAI clock (49.152 MHz from PLL1_Q)
    {
        use embassy_stm32::rcc::*;
        config.rcc.pll1 = Some(Pll {
            source: PllSource::HSI,  // 16 MHz
            prediv: PllPreDiv::DIV1, // M
            mul: PllMul::MUL12,      // Integer N (12)
            // Fraction gives: 16 * (12 + 2363/8192) ≈ 196.608 MHz VCO
            divq: Some(PllDiv::DIV4), // 196.608 / 4 = 49.152 MHz -> SAI kernel
            divr: Some(PllDiv::DIV5), // System clock path (adjust if you need a higher HCLK)
            divp: Some(PllDiv::DIV30),
            frac: Some(2363),
        });

        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV1;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        config.rcc.apb7_pre = APBPrescaler::DIV1;
        config.rcc.ahb5_pre = AHB5Prescaler::DIV2;

        config.rcc.voltage_scale = VoltageScale::RANGE1;

        config.rcc.mux.sai1sel = mux::Sai1sel::PLL1_Q;
    }

    let p = embassy_stm32::init(config);

    info!("Hello World!");

    // Configure all required buffers in a static way.
    // (USB configuration removed; SD-to-SAI only)

    // Initialize SAI1 (I2S) for playback: SCK=PA7, FS=PA8, SD=PB14, DMA=GPDMA1_CH4
    let (sai_a, _sai_b) = sai::split_subblocks(p.SAI1);

    static SAI_DMA_BUF: StaticCell<[u16; 4096]> = StaticCell::new();
    let sai_dma_buf = SAI_DMA_BUF.init([0u16; 4096]);

    let mut sai_cfg = sai::Config::default();
    sai_cfg.mode = sai::Mode::Master;
    sai_cfg.tx_rx = sai::TxRx::Transmitter;
    sai_cfg.stereo_mono = sai::StereoMono::Mono;
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
    sai_cfg.frame_length = 32;
    sai_cfg.frame_sync_active_level_length = sai::word::U7(16);
    sai_cfg.fifo_threshold = sai::FifoThreshold::Quarter;
    // DIV4 for proper 48kHz sample rate
    // 49.152 MHz / 4 = 12.288 MHz master clock, LRCLK = 48 kHz
    sai_cfg.master_clock_divider = sai::MasterClockDivider::DIV4;

    let sai_tx = Sai::new_asynchronous(
        sai_a,
        p.PA7,  // SCK  (BCLK)
        p.PB14, // SD   (data) - back to PB14 (PA0 not valid for SAI)
        p.PA8,  // FS   (LRC / WS)
        p.GPDMA1_CH2,
        sai_dma_buf,
        sai_cfg,
    );

    // Configure MAX98357A SD pin (shutdown/mode select) - set HIGH for I2S mode
    // SD = HIGH enables I2S mode, SD = LOW shuts down, SD = Float enables Left-Justified
    let _max98357a_sd = Output::new(p.PA1, Level::High, Speed::Low);
    info!("MAX98357A SD pin (PA1) set to HIGH - I2S mode enabled");

    // SAI transmitters start automatically when writing, no need to call start()

    // (PWM removed)

    // Set up SPI for microSD (SPI1: SCK=PB4, MISO=PB3, MOSI=PA15; CS=PA6). Adjust to your board wiring.
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = Hertz(400_000); // init at <= 400kHz
    let spi = Spi::new(
        p.SPI1,
        p.PB4,  // SCK
        p.PA15, // MOSI
        p.PB3,  // MISO
        p.GPDMA1_CH0,
        p.GPDMA1_CH1,
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
