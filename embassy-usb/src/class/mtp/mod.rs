//! USB Media Transfer Protocol (MTP / PTP, PIMA 15740) implementation.
//!
//! MTP exposes files and folders as objects rather than raw disk blocks, allowing
//! the device firmware and the USB host to share storage safely at the same time.
//!
//! Two layers are provided under [`device`]:
//! - **Transport layer** ([`device::MtpClass`], [`device::MtpTransport`], [`device::MtpEventSender`]):
//!   handles the USB Still Image interface descriptors, EP0 class control requests
//!   (`Cancel`, `DeviceReset`, `GetDeviceStatus`), bulk container framing, streaming
//!   data transfers, and interrupt events.
//! - **Storage runner** ([`device::MtpStorage`], [`device::MtpClass::run`]):
//!   an async runner that implements standard PTP/MTP session, storage, object,
//!   and object-property operations on top of a user-supplied [`device::MtpStorage`].

#[cfg(feature = "device")]
pub mod device;
pub mod types;

pub use types::*;
