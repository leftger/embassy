use core::fmt;
use core::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Sub, SubAssign};

use super::{GCD_1K, GCD_1M, TICK_HZ};
use crate::GCD_1G;

#[derive(Debug, Default, Copy, Clone, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
/// Represents the difference between two [Instant](struct.Instant.html)s
pub struct Duration {
    pub(crate) ticks: u64,
}

impl Duration {
    /// The smallest value that can be represented by the `Duration` type.
    pub const MIN: Duration = Duration { ticks: u64::MIN };
    /// The largest value that can be represented by the `Duration` type.
    pub const MAX: Duration = Duration { ticks: u64::MAX };

    /// Tick count of the `Duration`.
    pub const fn as_ticks(&self) -> u64 {
        self.ticks
    }

    /// Convert the `Duration` to seconds, rounding down.
    pub const fn as_secs(&self) -> u64 {
        self.ticks / TICK_HZ
    }

    /// Convert the `Duration` to milliseconds, rounding down.
    pub const fn as_millis(&self) -> u64 {
        self.ticks * (1000 / GCD_1K) / (TICK_HZ / GCD_1K)
    }

    /// Convert the `Duration` to microseconds, rounding down.
    pub const fn as_micros(&self) -> u64 {
        self.ticks * (1_000_000 / GCD_1M) / (TICK_HZ / GCD_1M)
    }

    /// Convert the `Duration` to nanoseconds, rounding down.
    pub const fn as_nanos(&self) -> u64 {
        self.ticks * (1_000_000_000 / GCD_1G) / (TICK_HZ / GCD_1G)
    }

    /// Creates a duration from the specified number of clock ticks
    pub const fn from_ticks(ticks: u64) -> Duration {
        Duration { ticks }
    }

    /// Creates a duration from the specified number of seconds, rounding up.
    pub const fn from_secs(secs: u64) -> Duration {
        Duration { ticks: secs * TICK_HZ }
    }

    /// Creates a duration from the specified number of milliseconds, rounding up.
    pub const fn from_millis(millis: u64) -> Duration {
        Duration {
            ticks: u64::div_ceil(millis * (TICK_HZ / GCD_1K), 1000 / GCD_1K),
        }
    }

    /// Creates a duration from the specified number of microseconds, rounding up.
    /// NOTE: Delays this small may be inaccurate.
    pub const fn from_micros(micros: u64) -> Duration {
        Duration {
            ticks: u64::div_ceil(micros * (TICK_HZ / GCD_1M), 1_000_000 / GCD_1M),
        }
    }

    /// Creates a duration from the specified number of nanoseconds, rounding up.
    /// NOTE: Delays this small may be inaccurate.
    pub const fn from_nanos(nanoseconds: u64) -> Duration {
        Duration {
            ticks: u64::div_ceil(nanoseconds * (TICK_HZ / GCD_1G), 1_000_000_000 / GCD_1G),
        }
    }

    /// Creates a duration from the specified number of seconds, rounding down.
    pub const fn from_secs_floor(secs: u64) -> Duration {
        Duration { ticks: secs * TICK_HZ }
    }

    /// Creates a duration from the specified number of milliseconds, rounding down.
    pub const fn from_millis_floor(millis: u64) -> Duration {
        Duration {
            ticks: millis * (TICK_HZ / GCD_1K) / (1000 / GCD_1K),
        }
    }

    /// Creates a duration from the specified number of microseconds, rounding down.
    /// NOTE: Delays this small may be inaccurate.
    pub const fn from_micros_floor(micros: u64) -> Duration {
        Duration {
            ticks: micros * (TICK_HZ / GCD_1M) / (1_000_000 / GCD_1M),
        }
    }

    /// Try to create a duration from the specified number of seconds, rounding up.
    /// Fails if the number of seconds is too large.
    pub const fn try_from_secs(secs: u64) -> Option<Duration> {
        let Some(ticks) = secs.checked_mul(TICK_HZ) else {
            return None;
        };
        Some(Duration { ticks })
    }

    /// Try to create a duration from the specified number of milliseconds, rounding up.
    /// Fails if the number of milliseconds is too large.
    pub const fn try_from_millis(millis: u64) -> Option<Duration> {
        let Some(value) = millis.checked_mul(TICK_HZ / GCD_1K) else {
            return None;
        };
        Some(Duration {
            ticks: u64::div_ceil(value, 1000 / GCD_1K),
        })
    }

    /// Try to create a duration from the specified number of microseconds, rounding up.
    /// Fails if the number of microseconds is too large.
    /// NOTE: Delays this small may be inaccurate.
    pub const fn try_from_micros(micros: u64) -> Option<Duration> {
        let Some(value) = micros.checked_mul(TICK_HZ / GCD_1M) else {
            return None;
        };
        Some(Duration {
            ticks: u64::div_ceil(value, 1_000_000 / GCD_1M),
        })
    }

    /// Try to create a duration from the specified number of nanoseconds, rounding up.
    /// Fails if the number of nanoseconds is too large.
    /// NOTE: Delays this small may be inaccurate.
    pub const fn try_from_nanos(nanoseconds: u64) -> Option<Duration> {
        let Some(value) = nanoseconds.checked_mul(TICK_HZ / GCD_1G) else {
            return None;
        };
        Some(Duration {
            ticks: u64::div_ceil(value, 1_000_000_000 / GCD_1G),
        })
    }

    /// Try to create a duration from the specified number of seconds, rounding down.
    /// Fails if the number of seconds is too large.
    pub const fn try_from_secs_floor(secs: u64) -> Option<Duration> {
        let Some(ticks) = secs.checked_mul(TICK_HZ) else {
            return None;
        };
        Some(Duration { ticks })
    }

    /// Try to create a duration from the specified number of milliseconds, rounding down.
    /// Fails if the number of milliseconds is too large.
    pub const fn try_from_millis_floor(millis: u64) -> Option<Duration> {
        let Some(value) = millis.checked_mul(TICK_HZ / GCD_1K) else {
            return None;
        };
        Some(Duration {
            ticks: value / (1000 / GCD_1K),
        })
    }

    /// Try to create a duration from the specified number of microseconds, rounding down.
    /// Fails if the number of microseconds is too large.
    /// NOTE: Delays this small may be inaccurate.
    pub const fn try_from_micros_floor(micros: u64) -> Option<Duration> {
        let Some(value) = micros.checked_mul(TICK_HZ / GCD_1M) else {
            return None;
        };
        Some(Duration {
            ticks: value / (1_000_000 / GCD_1M),
        })
    }

    /// Creates a duration corresponding to the specified Hz.
    /// NOTE: Giving this function a hz >= the TICK_HZ of your platform will clamp the Duration to 1
    /// tick. Doing so will not deadlock, but will certainly not produce the desired output.
    ///
    /// ## Panics
    ///
    /// Panics if `hz` is zero.
    pub const fn from_hz(hz: u64) -> Duration {
        let ticks = { if hz >= TICK_HZ { 1 } else { (TICK_HZ + hz / 2) / hz } };
        Duration { ticks }
    }

    /// Adds one `Duration` to another, returning a new `Duration` or `None` in the event of an overflow.
    pub fn checked_add(self, rhs: Duration) -> Option<Duration> {
        self.ticks.checked_add(rhs.ticks).map(|ticks| Duration { ticks })
    }

    /// Subtracts one `Duration` from another, returning a new `Duration` or `None` in the event of an overflow.
    pub fn checked_sub(self, rhs: Duration) -> Option<Duration> {
        self.ticks.checked_sub(rhs.ticks).map(|ticks| Duration { ticks })
    }

    /// Multiplies one `Duration` by a scalar `u32`, returning a new `Duration` or `None` in the event of an overflow.
    pub fn checked_mul(self, rhs: u32) -> Option<Duration> {
        self.ticks.checked_mul(rhs as _).map(|ticks| Duration { ticks })
    }

    /// Divides one `Duration` by a scalar `u32`, returning a new `Duration` or `None` in the event of an overflow.
    pub fn checked_div(self, rhs: u32) -> Option<Duration> {
        self.ticks.checked_div(rhs as _).map(|ticks| Duration { ticks })
    }
}

impl Add for Duration {
    type Output = Duration;

    /// Computes `Duration + Duration`. [Read more](Add)
    ///
    /// ## Panics
    ///
    /// Panics if the computed duration overflows.
    fn add(self, rhs: Duration) -> Duration {
        self.checked_add(rhs).expect("overflow when adding durations")
    }
}

impl AddAssign for Duration {
    /// Computes `Duration += Duration`. [Read more](AddAssign)
    ///
    /// ## Panics
    ///
    /// Panics if the computed duration overflows.
    fn add_assign(&mut self, rhs: Duration) {
        *self = *self + rhs;
    }
}

impl Sub for Duration {
    type Output = Duration;

    /// Computes `Duration - Duration`. [Read more](Sub)
    ///
    /// ## Panics
    ///
    /// Panics if the computed duration overflows.
    fn sub(self, rhs: Duration) -> Duration {
        self.checked_sub(rhs).expect("overflow when subtracting durations")
    }
}

impl SubAssign for Duration {
    /// Computes `Duration -= Duration`. [Read more](SubAssign)
    ///
    /// ## Panics
    ///
    /// Panics if the computed duration overflows.
    fn sub_assign(&mut self, rhs: Duration) {
        *self = *self - rhs;
    }
}

impl Mul<u32> for Duration {
    type Output = Duration;

    /// Computes `Duration * u32`. [Read more](Mul)
    ///
    /// ## Panics
    ///
    /// Panics if the computed duration overflows.
    fn mul(self, rhs: u32) -> Duration {
        self.checked_mul(rhs)
            .expect("overflow when multiplying duration by scalar")
    }
}

impl Mul<Duration> for u32 {
    type Output = Duration;

    /// Computes `u32 * Duration`. [Read more](Mul)
    ///
    /// ## Panics
    ///
    /// Panics if the computed duration overflows.
    fn mul(self, rhs: Duration) -> Duration {
        rhs * self
    }
}

impl MulAssign<u32> for Duration {
    /// Computes `Duration *= u32`. [Read more](MulAssign)
    ///
    /// ## Panics
    ///
    /// Panics if the computed duration overflows.
    fn mul_assign(&mut self, rhs: u32) {
        *self = *self * rhs;
    }
}

impl Div<u32> for Duration {
    type Output = Duration;

    /// Computes `Duration / u32`. [Read more](Div)
    ///
    /// ## Panics
    ///
    /// Panics if dividing by zero.
    fn div(self, rhs: u32) -> Duration {
        self.checked_div(rhs)
            .expect("divide by zero error when dividing duration by scalar")
    }
}

impl DivAssign<u32> for Duration {
    /// Computes `Duration /= u32`. [Read more](DivAssign)
    ///
    /// ## Panics
    ///
    /// Panics if dividing by zero.
    fn div_assign(&mut self, rhs: u32) {
        *self = *self / rhs;
    }
}

impl<'a> fmt::Display for Duration {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "{} ticks", self.ticks)
    }
}

impl TryFrom<core::time::Duration> for Duration {
    type Error = <u64 as TryFrom<u128>>::Error;

    /// Converts using [`Duration::from_micros`]. Fails if value can not be represented as u64.
    fn try_from(value: core::time::Duration) -> Result<Self, Self::Error> {
        Ok(Self::from_micros(value.as_micros().try_into()?))
    }
}

impl From<Duration> for core::time::Duration {
    /// Converts using [`Duration::as_micros`].
    fn from(value: Duration) -> Self {
        core::time::Duration::from_micros(value.as_micros())
    }
}

impl core::iter::Sum for Duration {
    fn sum<I>(iter: I) -> Self
    where
        I: Iterator<Item = Duration>,
    {
        Duration::from_ticks(iter.map(|d| d.as_ticks()).sum())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::TICK_HZ;

    #[test]
    fn from_as_roundtrip() {
        assert_eq!(Duration::from_secs(3).as_secs(), 3);
        assert_eq!(Duration::from_millis(1500).as_millis(), 1500);
        assert_eq!(Duration::from_secs(1).as_ticks(), TICK_HZ);
    }

    #[test]
    fn millis_rounds_up_vs_floor() {
        // At 1MHz, 1ms = 1000 ticks exactly; use a value that needs ceil when TICK_HZ divides oddly.
        // from_millis uses div_ceil; from_millis_floor truncates.
        let ceil = Duration::from_millis(1);
        let floor = Duration::from_millis_floor(1);
        assert!(ceil.as_ticks() >= floor.as_ticks());
        assert_eq!(floor.as_millis(), 1);
    }

    #[test]
    fn from_hz() {
        assert_eq!(Duration::from_hz(2).as_ticks(), TICK_HZ / 2);
        assert_eq!(Duration::from_hz(TICK_HZ).as_ticks(), 1);
        assert_eq!(Duration::from_hz(TICK_HZ * 2).as_ticks(), 1);
    }

    #[test]
    #[should_panic]
    fn from_hz_zero_panics() {
        let _ = Duration::from_hz(0);
    }

    #[test]
    fn try_from_overflow() {
        assert!(Duration::try_from_secs(u64::MAX).is_none());
        assert!(Duration::try_from_millis(u64::MAX).is_none());
        assert_eq!(Duration::try_from_secs(2).unwrap().as_secs(), 2);
    }

    #[test]
    fn checked_arith_and_sum() {
        let a = Duration::from_secs(2);
        let b = Duration::from_secs(3);
        assert_eq!(a.checked_add(b).unwrap().as_secs(), 5);
        assert_eq!(b.checked_sub(a).unwrap().as_secs(), 1);
        assert_eq!(a.checked_sub(b), None);
        assert_eq!(a.checked_mul(4).unwrap().as_secs(), 8);
        assert_eq!(b.checked_div(3).unwrap().as_secs(), 1);
        assert_eq!(Duration::MAX.checked_add(Duration::from_ticks(1)), None);

        let sum: Duration = [a, b, Duration::from_secs(1)].into_iter().sum();
        assert_eq!(sum.as_secs(), 6);
    }

    #[test]
    fn core_time_convert() {
        let d = Duration::from_millis(250);
        let core_d: core::time::Duration = d.into();
        assert_eq!(core_d.as_millis(), 250);
        let back = Duration::try_from(core_d).unwrap();
        assert_eq!(back.as_millis(), 250);
    }
}
