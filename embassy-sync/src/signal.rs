//! A synchronization primitive for passing the latest value to a task.
use core::cell::Cell;
use core::future::{Future, poll_fn};
use core::task::{Context, Poll, Waker};

use crate::blocking_mutex::Mutex;
use crate::blocking_mutex::raw::RawMutex;

/// Single-slot signaling primitive for a _single_ consumer.
///
/// This is similar to a [`Channel`](crate::channel::Channel) with a buffer size of 1, except
/// "sending" to it (calling [`Signal::signal`]) when full will overwrite the previous value instead
/// of waiting for the receiver to pop the previous value.
///
/// It is useful for sending data between tasks when the receiver only cares about
/// the latest data, and therefore it's fine to "lose" messages. This is often the case for "state"
/// updates.
///
/// For more advanced use cases, you might want to use [`Channel`](crate::channel::Channel) instead.
/// For multiple consumers, use [`Watch`](crate::watch::Watch) instead.
///
/// Signals are generally declared as `static`s and then borrowed as required.
///
/// ```
/// use embassy_sync::signal::Signal;
/// use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
///
/// enum SomeCommand {
///   On,
///   Off,
/// }
///
/// static SOME_SIGNAL: Signal<CriticalSectionRawMutex, SomeCommand> = Signal::new();
/// ```
pub struct Signal<M, T>
where
    M: RawMutex,
{
    state: Mutex<M, Cell<State<T>>>,
}

#[derive(Debug)]
enum State<T> {
    None,
    Waiting(Waker),
    Signaled(T),
}

impl<M, T> Signal<M, T>
where
    M: RawMutex,
{
    /// Create a new `Signal`.
    pub const fn new() -> Self {
        Self {
            state: Mutex::new(Cell::new(State::None)),
        }
    }
}

impl<M, T> Default for Signal<M, T>
where
    M: RawMutex,
{
    fn default() -> Self {
        Self::new()
    }
}

impl<M, T> Signal<M, T>
where
    M: RawMutex,
{
    /// Mark this Signal as signaled.
    pub fn signal(&self, val: T) {
        self.state.lock(|cell| {
            let state = cell.replace(State::Signaled(val));
            if let State::Waiting(waker) = state {
                waker.wake();
            }
        })
    }

    /// Remove the queued value in this `Signal`, if any.
    pub fn reset(&self) {
        self.try_take();
    }

    /// Poll for state changes of this Signal.
    pub fn poll_wait(&self, cx: &mut Context<'_>) -> Poll<T> {
        self.state.lock(|cell| {
            let state = cell.replace(State::None);
            match state {
                State::None => {
                    cell.set(State::Waiting(cx.waker().clone()));
                    Poll::Pending
                }
                State::Waiting(w) if w.will_wake(cx.waker()) => {
                    cell.set(State::Waiting(w));
                    Poll::Pending
                }
                State::Waiting(w) => {
                    cell.set(State::Waiting(cx.waker().clone()));
                    w.wake();
                    Poll::Pending
                }
                State::Signaled(res) => Poll::Ready(res),
            }
        })
    }

    /// Future that completes when this Signal has been signaled, taking the value out of the signal.
    ///
    /// The returned Future is cancel-safe. No value will be lost even if it isn't polled to completion.
    pub fn wait(&self) -> impl Future<Output = T> + '_ {
        poll_fn(move |cx| self.poll_wait(cx))
    }

    /// non-blocking method to try and take the signal value.
    pub fn try_take(&self) -> Option<T> {
        self.state.lock(|cell| {
            let state = cell.replace(State::None);
            match state {
                State::Signaled(res) => Some(res),
                state => {
                    cell.set(state);
                    None
                }
            }
        })
    }

    /// non-blocking method to check whether this signal has been signaled. This does not clear the signal.  
    pub fn signaled(&self) -> bool {
        self.state.lock(|cell| {
            let state = cell.replace(State::None);

            let res = matches!(state, State::Signaled(_));

            cell.set(state);

            res
        })
    }
}

#[cfg(test)]
mod tests {
    use core::time::Duration;

    use futures_executor::{ThreadPool, block_on};
    use futures_timer::Delay;
    use futures_util::task::SpawnExt;
    use static_cell::StaticCell;

    use super::*;
    use crate::blocking_mutex::raw::{CriticalSectionRawMutex, NoopRawMutex};

    #[test]
    fn try_take_empty() {
        let s = Signal::<NoopRawMutex, u32>::new();
        assert!(!s.signaled());
        assert_eq!(s.try_take(), None);
    }

    #[test]
    fn signal_and_try_take() {
        let s = Signal::<NoopRawMutex, u32>::new();
        s.signal(7);
        assert!(s.signaled());
        assert_eq!(s.try_take(), Some(7));
        assert!(!s.signaled());
        assert_eq!(s.try_take(), None);
    }

    #[test]
    fn signal_overwrites() {
        let s = Signal::<NoopRawMutex, u32>::new();
        s.signal(1);
        s.signal(2);
        assert_eq!(s.try_take(), Some(2));
    }

    #[test]
    fn reset_clears_value() {
        let s = Signal::<NoopRawMutex, u32>::new();
        s.signal(3);
        s.reset();
        assert!(!s.signaled());
        assert_eq!(s.try_take(), None);
    }

    #[test]
    fn wait_after_signal() {
        block_on(async {
            let s = Signal::<NoopRawMutex, u32>::new();
            s.signal(5);
            assert_eq!(s.wait().await, 5);
            assert!(!s.signaled());
        });
    }

    #[futures_test::test]
    async fn wait_then_signal() {
        let executor = ThreadPool::new().unwrap();

        static SIGNAL: StaticCell<Signal<CriticalSectionRawMutex, u32>> = StaticCell::new();
        let s = &*SIGNAL.init(Signal::new());
        let s2 = s;
        assert!(
            executor
                .spawn(async move {
                    Delay::new(Duration::from_millis(10)).await;
                    s2.signal(9);
                })
                .is_ok()
        );
        assert_eq!(s.wait().await, 9);
    }
}
