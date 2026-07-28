//! ADC Ring Buffered Example - All ADC4 Channels, "Read Latest" Sample
//!
//! This example exercises every channel ADC4 has on the STM32WBA65RI: the three
//! internal channels (VREFINT, VCORE, temperature) plus all ten external analog
//! pins (PA0..PA8, PB9). All 13 channels are continuously scanned back-to-back
//! by circular DMA into a ring buffer.
//!
//! Rather than waiting for the DMA half/full-transfer interrupt like
//! `adc_ring_buffered.rs` does with `read()`, this example calls
//! [`RingBufferedAdc::read_latest`] from a plain 2 Hz software timer. That
//! method never blocks and never reports an overrun: it just hands back the
//! newest complete scan and silently drops anything older, which is exactly
//! what you want when a consumer only cares about "what is the value *right
//! now*" rather than every sample the ADC produced. Since ADC4 scans all 13
//! channels far faster than 2 Hz, this also proves the ring buffer survives
//! being lapped by the producer over and over without ever wedging.
//!
//! Wiring: PA0..PA8 and PB9 are free-floating on a stock NUCLEO-WBA65RI, so
//! expect noisy/floating readings unless you wire a potentiometer or fixed
//! voltage to them. PA8 doubles as the ST-Link Virtual COM Port RX pin (see
//! `ble_serial_com.rs`) - harmless here since this example only logs over RTT,
//! but keep it in mind if you also want a serial console on this board.

#![no_std]
#![no_main]

use defmt::*;
use defmt_rtt as _;
use embassy_stm32::adc::adc4::Calibration;
use embassy_stm32::adc::{Adc, AdcChannel, RingBufferedAdc, adc4};
use embassy_stm32::peripherals::GPDMA1_CH1;
use embassy_stm32::{Config, bind_interrupts, dma};
use embassy_time::{Duration, Ticker};
use panic_probe as _;

/// VREFINT, PA8, PA7, PA6, PA5, PA4, PA3, PA2, PA1, PA0, PB9, VCORE, temperature.
const NUM_CHANNELS: usize = 13;
/// Sample sets buffered before the oldest data would be overwritten.
const SETS_PER_BUFFER: usize = 32;
const DMA_BUF_LEN: usize = NUM_CHANNELS * SETS_PER_BUFFER;

bind_interrupts!(struct Irqs {
    GPDMA1_CHANNEL1 => dma::InterruptHandler<GPDMA1_CH1>;
});

#[embassy_executor::main]
async fn main(_spawner: embassy_executor::Spawner) {
    let p = embassy_stm32::init(Config::default());

    info!("STM32WBA6 ADC4 Ring Buffered Example - all channels, read_latest()");

    let calibration = Calibration::read();
    info!(
        "Calibration: TS_CAL1={} (30C) TS_CAL2={} (130C) VREFINT_CAL={}",
        calibration.ts_cal1, calibration.ts_cal2, calibration.vrefint_cal
    );

    let mut adc = Adc::new_adc4(p.ADC4);
    adc.set_resolution_adc4(adc4::Resolution::Bits12);
    // Averaging disabled: with 2 Hz consumption there is no need to slow the
    // scan down, and read_latest() is meant to tolerate raw, unaveraged data.
    adc.set_averaging_adc4(adc4::Averaging::Disabled);
    let max_count = adc4::resolution_to_max_count(adc4::Resolution::Bits12);

    // Internal channels.
    let mut vrefint = adc.enable_vrefint_adc4(); // channel 0
    let mut vcore = adc.enable_vcore_adc4(); // channel 12
    let mut temperature = adc.enable_temperature_adc4(); // channel 13

    // External pins, named by their Arduino-style header labels on NUCLEO-WBA65RI.
    let mut pa0 = p.PA0; // channel 9
    let mut pa1 = p.PA1; // channel 8
    let mut pa2 = p.PA2; // channel 7
    let mut pa3 = p.PA3; // channel 6
    let mut pa4 = p.PA4; // channel 5
    let mut pa5 = p.PA5; // channel 4
    let mut pa6 = p.PA6; // channel 3
    let mut pa7 = p.PA7; // channel 2
    let mut pa8 = p.PA8; // channel 1
    let mut pb9 = p.PB9; // channel 10

    static mut DMA_BUF: [u16; DMA_BUF_LEN] = [0u16; DMA_BUF_LEN];

    // ADC4's scan sequence must be given in strictly ascending channel-number
    // order, which is why the array below jumps around the pin names above.
    let mut ring_adc: RingBufferedAdc<_> = adc.into_ring_buffered(
        p.GPDMA1_CH1,
        unsafe { &mut *core::ptr::addr_of_mut!(DMA_BUF) },
        Irqs,
        [
            (vrefint.reborrow_adc(), adc4::SampleTime::Cycles795), // ch 0
            (pa8.reborrow_adc(), adc4::SampleTime::Cycles795),     // ch 1
            (pa7.reborrow_adc(), adc4::SampleTime::Cycles795),     // ch 2
            (pa6.reborrow_adc(), adc4::SampleTime::Cycles795),     // ch 3
            (pa5.reborrow_adc(), adc4::SampleTime::Cycles795),     // ch 4
            (pa4.reborrow_adc(), adc4::SampleTime::Cycles795),     // ch 5
            (pa3.reborrow_adc(), adc4::SampleTime::Cycles795),     // ch 6
            (pa2.reborrow_adc(), adc4::SampleTime::Cycles795),     // ch 7
            (pa1.reborrow_adc(), adc4::SampleTime::Cycles795),     // ch 8
            (pa0.reborrow_adc(), adc4::SampleTime::Cycles795),     // ch 9
            (pb9.reborrow_adc(), adc4::SampleTime::Cycles795),     // ch 10
            (vcore.reborrow_adc(), adc4::SampleTime::Cycles795),   // ch 12
            (temperature.reborrow_adc(), adc4::SampleTime::Cycles795), // ch 13
        ]
        .into_iter(),
        None, // free-running: WBA6 has no hw-timer trigger mapping for ADC4 yet
    );

    ring_adc.start();
    info!(
        "Ring buffer running: {} channels, {} sets buffered, reading latest sample twice a second",
        NUM_CHANNELS, SETS_PER_BUFFER
    );

    let mut latest = [0u16; NUM_CHANNELS];
    let mut ticker = Ticker::every(Duration::from_millis(500));

    loop {
        ticker.next().await;

        let n = ring_adc.read_latest(&mut latest);
        if n != NUM_CHANNELS {
            // The producer hasn't completed a full scan yet (only right after start()).
            warn!("read_latest: only {} of {} channels ready, skipping", n, NUM_CHANNELS);
            continue;
        }

        let vdda_mv = calibration.calculate_vdda_mv(latest[0] as u32);
        let temp_mc = calibration.convert_to_millicelsius(latest[12] as u32, latest[0] as u32);
        let vcore_mv = (vdda_mv * latest[11] as u32) / max_count;

        let to_mv = |raw: u16| (vdda_mv * raw as u32) / max_count;

        info!(
            "VDDA={}mV VCORE={}mV Temp={}mC | PA0={}mV PA1={}mV PA2={}mV PA3={}mV PA4={}mV PA5={}mV PA6={}mV PA7={}mV PA8={}mV PB9={}mV",
            vdda_mv,
            vcore_mv,
            temp_mc,
            to_mv(latest[9]),
            to_mv(latest[8]),
            to_mv(latest[7]),
            to_mv(latest[6]),
            to_mv(latest[5]),
            to_mv(latest[4]),
            to_mv(latest[3]),
            to_mv(latest[2]),
            to_mv(latest[1]),
            to_mv(latest[10]),
        );
    }
}
