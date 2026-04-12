use core::future::poll_fn;
use core::task::Poll;

use crate::adc::{Adc, Instance};

/// This enum is passed into `Adc::init_watchdog` to select which channels to monitor.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum WatchdogChannels {
    /// Monitor all channels in the regular sequence.
    All,
    /// Monitor only one channel.
    Single(u8),
}

/// A driver for an ADC analog watchdog.
pub struct AnalogWatchdog<'adc, 'd, T: Instance<Regs = crate::pac::adc::Adc4>> {
    _adc: &'adc mut Adc<'d, T>,
}

impl<'adc, 'd, T: Instance<Regs = crate::pac::adc::Adc4>> AnalogWatchdog<'adc, 'd, T> {
    pub(crate) fn new(adc: &'adc mut Adc<'d, T>) -> Self {
        Self { _adc: adc }
    }

    /// Wait for the watchdog to trigger.
    ///
    /// The watchdog is configured in `Adc::init_watchdog`.
    pub async fn wait_for_trigger(&mut self) {
        self.start_awd();

        poll_fn(|cx| {
            T::state().waker.register(cx.waker());

            if T::regs().isr().read().awd(0) {
                // AWD1 flag is set, interrupt was triggered and disabled in handler
                Poll::Ready(())
            } else {
                Poll::Pending
            }
        })
        .await;
    }

    pub(crate) fn setup_awd(channels: WatchdogChannels, low_threshold: u16, high_threshold: u16) {
        // Configure thresholds
        T::regs().awd1tr().modify(|w| {
            #[cfg(stm32wba)]
            {
                w.set_lt1(low_threshold);
                w.set_ht1(high_threshold);
            }
            #[cfg(stm32u5)]
            {
                w.set_lt3(low_threshold);
                w.set_ht3(high_threshold);
            }
        });

        // Configure AWD1
        T::regs().cfgr1().modify(|w| {
            w.set_awd1en(true);
            #[cfg(stm32wba)]
            {
                use crate::pac::adc::vals::Awd1sgl;
                match channels {
                    WatchdogChannels::Single(ch) => {
                        w.set_awd1sgl(Awd1sgl::SINGLE_CHANNEL);
                        w.set_awd1ch(ch);
                    }
                    WatchdogChannels::All => {
                        w.set_awd1sgl(Awd1sgl::ALL_CHANNELS);
                    }
                }
            }
            #[cfg(stm32u5)]
            {
                match channels {
                    WatchdogChannels::Single(ch) => {
                        w.set_awd1sgl(true);
                        w.set_awd1ch(ch);
                    }
                    WatchdogChannels::All => {
                        w.set_awd1sgl(false);
                    }
                }
            }
        });
    }

    fn start_awd(&mut self) {
        // Clear AWD1 interrupt flag
        T::regs().isr().modify(|w| w.set_awd(0, true));

        // Enable AWD1 interrupt
        T::regs().ier().modify(|w| w.set_awdie(0, true));
    }
}

impl<'adc, 'd, T: Instance<Regs = crate::pac::adc::Adc4>> Drop for AnalogWatchdog<'adc, 'd, T> {
    fn drop(&mut self) {
        // Disable AWD1
        T::regs().cfgr1().modify(|w| w.set_awd1en(false));

        // Disable AWD1 interrupt
        T::regs().ier().modify(|w| w.set_awdie(0, false));
    }
}
