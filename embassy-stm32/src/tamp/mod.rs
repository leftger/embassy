//! Tamper and Backup Registers (TAMP)
#![macro_use]
#![cfg_attr(allow(unused))]

use embassy_hal_internal::Peripheral;
use crate::{peripherals, rcc::RccPeripheral};

/// TAMP peripheral driver.
pub struct Tamp<'d> {
    _inner: Peripheral<'d, peripherals::TAMP>,
}

impl<'d> Tamp<'d> {
    pub fn new(p: impl Peripheral<'d, peripherals::TAMP>) -> Self {
        peripherals::TAMP::enable_and_reset();
        Self { _inner: p.into_ref() }
    }

    pub fn write_backup_register(&mut self, index: usize, value: u32) {
        assert!(index < 32);
        unsafe { &*crate::pac::TAMP::ptr() }
            .bkpr(index)
            .write(|w| w.set_bkp(value));
    }

    pub fn read_backup_register(&self, index: usize) -> u32 {
        assert!(index < 32);
        unsafe { &*crate::pac::TAMP::ptr() }
            .bkpr(index)
            .read()
            .bkp()
    }

    pub fn increment_monotonic_counter(&mut self) {
        unsafe { &*crate::pac::TAMP::ptr() }
            .countr()
            .write(|w| w.set_count(0));
    }

    pub fn read_monotonic_counter(&self) -> u32 {
        unsafe { &*crate::pac::TAMP::ptr() }
            .countr()
            .read()
            .count()
    }

    pub fn enable_ext_interrupt(&mut self, index: usize) {
        assert!(index < 6);
        unsafe { &*crate::pac::TAMP::ptr() }
            .ier()
            .modify(|w| w.set_tampie(index, true));
    }

    pub fn enable_int_interrupt(&mut self, index: usize) {
        assert!(index < 13);
        unsafe { &*crate::pac::TAMP::ptr() }
            .ier()
            .modify(|w| w.set_itampie(index, true));
    }

    pub fn clear_all_flags(&mut self) {
        let r = unsafe { &*crate::pac::TAMP::ptr() };
        r.scr().write(|w| {
            w.set_ctampf(0b111111);
            w.set_citampf(0x1FFF);
        });
    }
}

trait SealedInstance {}

impl SealedInstance for peripherals::TAMP {}

/// TAMP peripheral instance trait.
pub trait Instance: SealedInstance + crate::PeripheralType + RccPeripheral {}

impl Instance for peripherals::TAMP {}