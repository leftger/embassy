#![no_std]
#![no_main]

use core::cell::{Cell, RefCell};

use defmt::{panic, *};
use embassy_executor::Spawner;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, interrupt, peripherals, timer, usb, Config};
use embassy_stm32::sai::{self, Sai};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::signal::Signal;
use embassy_sync::zerocopy_channel;
use embassy_usb::class::uac1;
use embassy_usb::class::uac1::speaker::{self, Speaker};
use embassy_usb::driver::EndpointError;
use heapless::Vec;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use embassy_time::{Duration, Timer};
use embedded_hal_bus::spi::{ExclusiveDevice, NoDelay};
use embedded_sdmmc::{SdCard, VolumeManager};
use embedded_sdmmc::filesystem::{ToShortFileName, ShortFileName};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::spi::{self, Spi};

bind_interrupts!(struct Irqs {
    USB_OTG_HS => usb::InterruptHandler<peripherals::USB_OTG_HS>;
});

static TIMER: Mutex<CriticalSectionRawMutex, RefCell<Option<timer::low_level::Timer<peripherals::TIM2>>>> =
    Mutex::new(RefCell::new(None));

// A counter signal that is written by the feedback timer, once every `FEEDBACK_REFRESH_PERIOD`.
// At that point, a feedback value is sent to the host.
pub static FEEDBACK_SIGNAL: Signal<CriticalSectionRawMutex, u32> = Signal::new();

// Stereo input
pub const INPUT_CHANNEL_COUNT: usize = 2;

// This example uses a fixed sample rate of 48 kHz.
pub const SAMPLE_RATE_HZ: u32 = 48_000;
pub const FEEDBACK_COUNTER_TICK_RATE: u32 = 31_250_000;

// Use 16-bit samples for broad OS compatibility.
pub const SAMPLE_WIDTH: uac1::SampleWidth = uac1::SampleWidth::Width2Byte;
pub const SAMPLE_WIDTH_BIT: usize = SAMPLE_WIDTH.in_bit();
pub const SAMPLE_SIZE: usize = SAMPLE_WIDTH as usize;
pub const SAMPLE_SIZE_PER_S: usize = (SAMPLE_RATE_HZ as usize) * INPUT_CHANNEL_COUNT * SAMPLE_SIZE;

// High-speed: size per 125 µs microframe (8000 microframes/s).
pub const USB_FRAME_SIZE: usize = SAMPLE_SIZE_PER_S.div_ceil(8000);

// Select front left and right audio channels.
pub const AUDIO_CHANNELS: [uac1::Channel; INPUT_CHANNEL_COUNT] = [uac1::Channel::LeftFront, uac1::Channel::RightFront];

// Factor of two as a margin for feedback (this is an excessive amount)
pub const USB_MAX_PACKET_SIZE: usize = 2 * USB_FRAME_SIZE;
pub const USB_MAX_SAMPLE_COUNT: usize = USB_MAX_PACKET_SIZE / SAMPLE_SIZE;

// The data type that is exchanged via the zero-copy channel (a sample vector).
pub type SampleBlock = Vec<u16, USB_MAX_SAMPLE_COUNT>;

// HS feedback uses 16.16 format (samples per microframe). Refresh every 4 microframes.
pub const FEEDBACK_REFRESH_PERIOD: uac1::FeedbackRefresh = uac1::FeedbackRefresh::Period4Frames;
const FEEDBACK_SHIFT: usize = 16;

const TICKS_PER_SAMPLE: f32 = (FEEDBACK_COUNTER_TICK_RATE as f32) / (SAMPLE_RATE_HZ as f32);

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

/// Sends feedback messages to the host.
async fn feedback_handler<'d, T: usb::Instance + 'd>(
    feedback: &mut speaker::Feedback<'d, usb::Driver<'d, T>>,
    feedback_factor: f32,
) -> Result<(), Disconnected> {
    let mut packet: Vec<u8, 4> = Vec::new();

    // Collects the fractional component of the feedback value that is lost by rounding.
    let mut rest = 0.0_f32;

    loop {
        let counter = FEEDBACK_SIGNAL.wait().await;

        packet.clear();

        let raw_value = counter as f32 * feedback_factor + rest;
        let value = raw_value.round();
        rest = raw_value - value;

        let value = value as u32;

        debug!("Feedback value: {}", value);

        packet.push(value as u8).unwrap();
        packet.push((value >> 8) as u8).unwrap();
        packet.push((value >> 16) as u8).unwrap();
        packet.push((value >> 24) as u8).unwrap();

        feedback.write_packet(&packet).await?;
    }
}

/// Handles streaming of audio data from the host.
async fn stream_handler<'d, T: usb::Instance + 'd>(
    stream: &mut speaker::Stream<'d, usb::Driver<'d, T>>,
    sender: &mut zerocopy_channel::Sender<'static, NoopRawMutex, SampleBlock>,
) -> Result<(), Disconnected> {
    loop {
        let mut usb_data = [0u8; USB_MAX_PACKET_SIZE];
        let data_size = stream.read_packet(&mut usb_data).await?;

        let word_count = data_size / SAMPLE_SIZE;

        if word_count * SAMPLE_SIZE == data_size {
            // Obtain a buffer from the channel
            let samples = sender.send().await;
            samples.clear();

            for w in 0..word_count {
                let byte_offset = w * SAMPLE_SIZE;
                let sample = u16::from_le_bytes(usb_data[byte_offset..byte_offset + SAMPLE_SIZE].try_into().unwrap());

                // Fill the sample buffer with data.
                samples.push(sample).unwrap();
            }

            sender.send_done();
        } else {
            debug!("Invalid USB buffer size of {}, skipped.", data_size);
        }
    }
}

/// Receives audio samples from USB and plays them back via SAI.
#[embassy_executor::task]
async fn audio_playback_task(
    mut usb_audio_receiver: zerocopy_channel::Receiver<'static, NoopRawMutex, SampleBlock>,
    mut sai_tx: Sai<'static, peripherals::SAI1, u16>,
) {
    loop {
        let samples = usb_audio_receiver.receive().await;
        let _ = sai_tx.write(&samples).await;
        usb_audio_receiver.receive_done();
    }
}

type SdSpiDev = ExclusiveDevice<Spi<'static, embassy_stm32::mode::Async>, Output<'static>, NoDelay>;
type SdCardDev = SdCard<SdSpiDev, embassy_time::Delay>;

#[embassy_executor::task]
async fn sd_stream_task(
    mut sai_tx: Sai<'static, peripherals::SAI1, u16>,
    volume_mgr: &'static mut VolumeManager<SdCardDev, DummyTimesource>,
) {
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
                let _ = sai_tx.write(&out_samples).await;
            } else {
                let _ = sai_tx.write(&in_samples).await;
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

/// Receives audio samples from the host.
#[embassy_executor::task]
async fn usb_streaming_task(
    mut stream: speaker::Stream<'static, usb::Driver<'static, peripherals::USB_OTG_HS>>,
    mut sender: zerocopy_channel::Sender<'static, NoopRawMutex, SampleBlock>,
) {
    loop {
        stream.wait_connection().await;
        info!("USB connected.");
        _ = stream_handler(&mut stream, &mut sender).await;
        info!("USB disconnected.");
    }
}

/// Sends sample rate feedback to the host.
///
/// The `feedback_factor` scales the feedback timer's counter value so that the result is the number of samples that
/// this device played back or "consumed" during one SOF period (1 ms) - in 10.14 format.
///
/// Ideally, the `feedback_factor` that is calculated below would be an integer for avoiding numerical errors.
/// This is achieved by having `TICKS_PER_SAMPLE` be a power of two. For audio applications at a sample rate of 48 kHz,
/// 24.576 MHz would be one such option.
#[embassy_executor::task]
async fn usb_feedback_task(mut feedback: speaker::Feedback<'static, usb::Driver<'static, peripherals::USB_OTG_HS>>) {
    let feedback_factor =
        ((1 << FEEDBACK_SHIFT) as f32 / TICKS_PER_SAMPLE) / FEEDBACK_REFRESH_PERIOD.frame_count() as f32;

    loop {
        feedback.wait_connection().await;
        // Bootstrap Windows by sending a few constant 16.16 feedback values (6 samples per HS microframe at 48 kHz).
        let fixed_16_16: u32 = ((SAMPLE_RATE_HZ as u32) << FEEDBACK_SHIFT) / 8000;
        for _ in 0..8 {
            let packet = [
                (fixed_16_16 & 0xFF) as u8,
                ((fixed_16_16 >> 8) & 0xFF) as u8,
                ((fixed_16_16 >> 16) & 0xFF) as u8,
                ((fixed_16_16 >> 24) & 0xFF) as u8,
            ];
            let _ = feedback.write_packet(&packet).await;
        }
        // Fallback: If our SOF-timed counter isn't firing (e.g., missing timer trigger), keep sending
        // constant feedback at the declared refresh period so the host proceeds with streaming.
        let period_us: u32 = (FEEDBACK_REFRESH_PERIOD.frame_count() as u32) * 125;
        loop {
            let packet = [
                (fixed_16_16 & 0xFF) as u8,
                ((fixed_16_16 >> 8) & 0xFF) as u8,
                ((fixed_16_16 >> 16) & 0xFF) as u8,
                ((fixed_16_16 >> 24) & 0xFF) as u8,
            ];
            if feedback.write_packet(&packet).await.is_err() {
                break; // disconnected
            }
            Timer::after(Duration::from_micros(period_us as u64)).await;
        }
    }
}

#[embassy_executor::task]
async fn usb_task(mut usb_device: embassy_usb::UsbDevice<'static, usb::Driver<'static, peripherals::USB_OTG_HS>>) {
    usb_device.run().await;
}

/// Checks for changes on the control monitor of the class.
///
/// In this case, monitor changes of volume or mute state.
#[embassy_executor::task]
async fn usb_control_task(control_monitor: speaker::ControlMonitor<'static>) {
    loop {
        control_monitor.changed().await;

        for channel in AUDIO_CHANNELS {
            let volume = control_monitor.volume(channel).unwrap();
            info!("Volume changed to {} on channel {}.", volume, channel);
        }
    }
}

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
#[interrupt]
fn TIM2() {
    static LAST_TICKS: Mutex<CriticalSectionRawMutex, Cell<u32>> = Mutex::new(Cell::new(0));
    static FRAME_COUNT: Mutex<CriticalSectionRawMutex, Cell<usize>> = Mutex::new(Cell::new(0));

    critical_section::with(|cs| {
        // Read timer counter.
        let timer = TIMER.borrow(cs).borrow().as_ref().unwrap().regs_gp32();

        let status = timer.sr().read();

        const CHANNEL_INDEX: usize = 0;
        if status.ccif(CHANNEL_INDEX) {
            let ticks = timer.ccr(CHANNEL_INDEX).read();

            let frame_count = FRAME_COUNT.borrow(cs);
            let last_ticks = LAST_TICKS.borrow(cs);

            frame_count.set(frame_count.get() + 1);
            if frame_count.get() >= FEEDBACK_REFRESH_PERIOD.frame_count() {
                frame_count.set(0);
                FEEDBACK_SIGNAL.signal(ticks.wrapping_sub(last_ticks.get()));
                last_ticks.set(ticks);
            }
        };

        // Clear trigger interrupt flag.
        timer.sr().modify(|r| r.set_tif(false));
    });
}

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
    debug!("USB packet size is {} byte", USB_MAX_PACKET_SIZE);
    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    let config_descriptor = CONFIG_DESCRIPTOR.init([0; 256]);

    static BOS_DESCRIPTOR: StaticCell<[u8; 32]> = StaticCell::new();
    let bos_descriptor = BOS_DESCRIPTOR.init([0; 32]);

    const CONTROL_BUF_SIZE: usize = 64;
    static CONTROL_BUF: StaticCell<[u8; CONTROL_BUF_SIZE]> = StaticCell::new();
    let control_buf = CONTROL_BUF.init([0; CONTROL_BUF_SIZE]);

    static STATE: StaticCell<speaker::State> = StaticCell::new();
    let state = STATE.init(speaker::State::new());

    // Create the driver, from the HAL, using the internal HS PHY.
    const EP_OUT_BUF_LEN: usize = 64 + USB_MAX_PACKET_SIZE;
    static EP_OUT_BUFFER: StaticCell<[u8; EP_OUT_BUF_LEN]> = StaticCell::new();
    let ep_out_buffer = EP_OUT_BUFFER.init([0u8; EP_OUT_BUF_LEN]);
    let mut usb_cfg = embassy_stm32::usb::Config::default();
    usb_cfg.vbus_detection = false;
    let usb_driver = usb::Driver::new_hs(p.USB_OTG_HS, Irqs, p.PD6, p.PD7, ep_out_buffer, usb_cfg);

    // Basic USB device configuration
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-audio-speaker example");
    config.serial_number = Some("12345678");

    let mut builder = embassy_usb::Builder::new(
        usb_driver,
        config,
        config_descriptor,
        bos_descriptor,
        &mut [], // no msos descriptors
        control_buf,
    );

    // Create the UAC1 Speaker class components
    let (stream, feedback, control_monitor) = Speaker::new(
        &mut builder,
        state,
        USB_MAX_PACKET_SIZE as u16,
        uac1::SampleWidth::Width2Byte,
        &[SAMPLE_RATE_HZ],
        &AUDIO_CHANNELS,
        FEEDBACK_REFRESH_PERIOD,
    );

    // Create the USB device
    let usb_device = builder.build();

    // Establish a zero-copy channel for transferring received audio samples between tasks
    static SAMPLE_BLOCKS: StaticCell<[SampleBlock; 2]> = StaticCell::new();
    let sample_blocks = SAMPLE_BLOCKS.init([Vec::new(), Vec::new()]);

    static CHANNEL: StaticCell<zerocopy_channel::Channel<'_, NoopRawMutex, SampleBlock>> = StaticCell::new();
    let channel = CHANNEL.init(zerocopy_channel::Channel::new(sample_blocks));
    let (sender, _receiver) = channel.split();

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
    sai_cfg.frame_sync_polarity = sai::FrameSyncPolarity::ActiveHigh;
    sai_cfg.frame_sync_offset = sai::FrameSyncOffset::OnFirstBit;
    sai_cfg.frame_length = 64;
    sai_cfg.frame_sync_active_level_length = sai::word::U7(32);
    sai_cfg.fifo_threshold = sai::FifoThreshold::Quarter;

    let sai_tx = Sai::new_asynchronous(
        sai_a,
        p.PA7,   // SCK
        p.PB12,  // SD
        p.PA8,   // FS
        p.GPDMA1_CH0,
        sai_dma_buf,
        sai_cfg,
    );

    // Set up SPI for microSD (SPI1: SCK=PB4, MOSI=PA15, MISO=PB3; CS=PA6). Adjust to your board wiring.
    let mut spi_cfg = spi::Config::default();
    spi_cfg.frequency = Hertz(400_000); // init at <= 400kHz
    let spi = Spi::new(
        p.SPI1,
        p.PB4,
        p.PA15,
        p.PB3,
        p.GPDMA1_CH4,
        p.GPDMA1_CH5,
        spi_cfg,
    );
    let cs = Output::new(p.PA6, Level::High, Speed::VeryHigh);
    let spi_dev: SdSpiDev = ExclusiveDevice::new_no_delay(spi, cs).unwrap();
    let mut sd: SdCardDev = SdCard::new(spi_dev, embassy_time::Delay);
    info!("SD size {} bytes", sd.num_bytes().unwrap_or(0));

    // Bump SPI after init
    let mut spi_cfg2 = spi::Config::default();
    spi_cfg2.frequency = Hertz(16_000_000);
    sd.spi(|dev| dev.bus_mut().set_config(&spi_cfg2));

    static VOLUME_MANAGER: StaticCell<VolumeManager<SdCardDev, DummyTimesource>> = StaticCell::new();
    let vol_mgr: &'static mut VolumeManager<SdCardDev, DummyTimesource> =
        VOLUME_MANAGER.init(VolumeManager::new(sd, DummyTimesource()));

    // Run a timer for counting between SOF interrupts.
    let mut tim = timer::low_level::Timer::new(p.TIM2);
    tim.set_tick_freq(Hertz(FEEDBACK_COUNTER_TICK_RATE));
    // Route USB OTG HS SOF to TIM2: on WBA, use an available ITR. Try ITR2 as generic.
    tim.set_trigger_source(timer::low_level::TriggerSource::ITR2);

    const TIMER_CHANNEL: timer::Channel = timer::Channel::Ch1;
    tim.set_input_ti_selection(TIMER_CHANNEL, timer::low_level::InputTISelection::TRC);
    tim.set_input_capture_prescaler(TIMER_CHANNEL, 0);
    tim.set_input_capture_filter(TIMER_CHANNEL, timer::low_level::FilterValue::FCK_INT_N2);

    // Reset all interrupt flags.
    tim.regs_1ch().sr().write(|r| r.set_uif(false));

    tim.enable_channel(TIMER_CHANNEL, true);
    tim.enable_input_interrupt(TIMER_CHANNEL, true);

    tim.start();

    TIMER.lock(|p| p.borrow_mut().replace(tim));

    // Unmask the TIM2 interrupt.
    unsafe {
        cortex_m::peripheral::NVIC::unmask(interrupt::TIM2);
    }

    // Launch USB audio tasks.
    spawner.spawn(unwrap!(usb_control_task(control_monitor)));
    spawner.spawn(unwrap!(usb_streaming_task(stream, sender)));
    spawner.spawn(unwrap!(usb_feedback_task(feedback)));
    spawner.spawn(unwrap!(usb_task(usb_device)));

    // Start SD streaming task to play AUDIO.WAV from microSD via SAI.
    spawner.spawn(unwrap!(sd_stream_task(sai_tx, vol_mgr)));
}
