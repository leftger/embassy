use core::future::Future;
use core::pin::Pin;
use core::ptr;
use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

static VTABLE: RawWakerVTable = RawWakerVTable::new(|_| RawWaker::new(ptr::null(), &VTABLE), |_| {}, |_| {}, |_| {});

/// Run a future to completion using a busy loop.
///
/// This calls `.poll()` on the future in a busy loop, which blocks
/// the current thread at 100% cpu usage until the future is done. The
/// future's `Waker` mechanism is not used.
///
/// You can use this to run multiple futures concurrently with [`join`][crate::join].
///
/// It's suitable for systems with no or limited concurrency and without
/// strict requirements around power consumption. For more complex use
/// cases, prefer using a "real" executor like `embassy-executor`, which
/// supports multiple tasks, and putting the core to sleep when no task
/// needs to do work.
pub fn block_on<F: Future>(mut fut: F) -> F::Output {
    // safety: we don't move the future after this line.
    let mut fut = unsafe { Pin::new_unchecked(&mut fut) };

    let raw_waker = RawWaker::new(ptr::null(), &VTABLE);
    let waker = unsafe { Waker::from_raw(raw_waker) };
    let mut cx = Context::from_waker(&waker);
    loop {
        if let Poll::Ready(res) = fut.as_mut().poll(&mut cx) {
            return res;
        }
    }
}

/// Poll a future once.
pub fn poll_once<F: Future>(mut fut: F) -> Poll<F::Output> {
    // safety: we don't move the future after this line.
    let mut fut = unsafe { Pin::new_unchecked(&mut fut) };

    let raw_waker = RawWaker::new(ptr::null(), &VTABLE);
    let waker = unsafe { Waker::from_raw(raw_waker) };
    let mut cx = Context::from_waker(&waker);

    fut.as_mut().poll(&mut cx)
}

#[cfg(test)]
mod tests {
    use core::future::{pending, ready};
    use core::task::Poll;

    use super::*;
    use crate::yield_now;

    #[test]
    fn block_on_ready() {
        assert_eq!(block_on(ready(42)), 42);
    }

    #[test]
    fn block_on_yield_now() {
        block_on(async {
            yield_now().await;
        });
    }

    #[test]
    fn poll_once_ready() {
        assert_eq!(poll_once(ready(7)), Poll::Ready(7));
    }

    #[test]
    fn poll_once_pending() {
        assert!(matches!(poll_once(pending::<()>()), Poll::Pending));
    }
}
