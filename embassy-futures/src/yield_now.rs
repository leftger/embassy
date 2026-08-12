use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};

/// Yield from the current task once, allowing other tasks to run.
///
/// This can be used to easily and quickly implement simple async primitives
/// without using wakers. The following snippet will wait for a condition to
/// hold, while still allowing other tasks to run concurrently (not monopolizing
/// the executor thread).
///
/// ```rust
/// # use embassy_futures::{block_on, yield_now};
/// # async fn test_fn() {
/// # let mut iter_count: u32 = 0;
/// # let mut some_condition = || { iter_count += 1; iter_count > 10 };
/// while !some_condition() {
///     yield_now().await;
/// }
/// # }
/// # block_on(test_fn());
/// ```
///
/// The downside is this will spin in a busy loop, using 100% of the CPU, while
/// using wakers correctly would allow the CPU to sleep while waiting.
///
/// The internal implementation is: on first poll the future wakes itself and
/// returns `Poll::Pending`. On second poll, it returns `Poll::Ready`.
pub fn yield_now() -> impl Future<Output = ()> {
    YieldNowFuture { yielded: false }
}

#[must_use = "futures do nothing unless you `.await` or poll them"]
struct YieldNowFuture {
    yielded: bool,
}

impl Future for YieldNowFuture {
    type Output = ();
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        if self.yielded {
            Poll::Ready(())
        } else {
            self.yielded = true;
            cx.waker().wake_by_ref();
            Poll::Pending
        }
    }
}

#[cfg(test)]
mod tests {
    use core::pin::pin;
    use core::ptr;
    use core::task::{Context, Poll, RawWaker, RawWakerVTable, Waker};

    use super::*;

    static VTABLE: RawWakerVTable =
        RawWakerVTable::new(|_| RawWaker::new(ptr::null(), &VTABLE), |_| {}, |_| {}, |_| {});

    #[test]
    fn yield_now_pending_then_ready() {
        let mut fut = pin!(yield_now());
        let waker = unsafe { Waker::from_raw(RawWaker::new(ptr::null(), &VTABLE)) };
        let mut cx = Context::from_waker(&waker);
        assert!(matches!(fut.as_mut().poll(&mut cx), Poll::Pending));
        assert!(matches!(fut.as_mut().poll(&mut cx), Poll::Ready(())));
    }

    #[test]
    fn yield_now_under_block_on() {
        crate::block_on(async {
            yield_now().await;
            yield_now().await;
        });
    }
}
