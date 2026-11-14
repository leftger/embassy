//! Class-specific EP0 control requests for MTP/PTP (USB Still Image Capture §5).

use core::cell::Cell;
use core::mem::MaybeUninit;

use embassy_sync::blocking_mutex::CriticalSectionMutex;

use super::super::{
    CANCEL_CODE, DeviceStatusCode, REQ_CANCEL, REQ_DEVICE_RESET, REQ_GET_DEVICE_STATUS, REQ_GET_EXTENDED_EVENT_DATA,
};
use crate::control::{InResponse, OutResponse, Recipient, Request, RequestType};
use crate::types::InterfaceNumber;
use crate::{Handler, InterfaceHandler};

#[derive(Copy, Clone, Debug)]
struct ControlInner {
    status: DeviceStatusCode,
    cancel_transaction_id: Option<u32>,
    reset_requested: bool,
}

impl ControlInner {
    const fn new() -> Self {
        Self {
            status: DeviceStatusCode::OK,
            cancel_transaction_id: None,
            reset_requested: false,
        }
    }
}

/// Shared state between EP0 control request handlers and the MTP class task.
pub struct ControlShared {
    inner: CriticalSectionMutex<Cell<ControlInner>>,
}

impl ControlShared {
    const fn new() -> Self {
        Self {
            inner: CriticalSectionMutex::new(Cell::new(ControlInner::new())),
        }
    }

    /// Returns the current device status code reported to `GetDeviceStatus` (`0x67`).
    pub fn status(&self) -> DeviceStatusCode {
        self.inner.lock(|c| c.get().status)
    }

    /// Sets the device status code reported to `GetDeviceStatus` (`0x67`).
    pub fn set_status(&self, status: DeviceStatusCode) {
        self.inner.lock(|c| {
            let mut inner = c.get();
            inner.status = status;
            c.set(inner);
        });
    }

    /// Returns `true` if the host requested cancellation of `transaction_id` via EP0.
    pub fn is_cancelled(&self, transaction_id: u32) -> bool {
        self.inner
            .lock(|c| c.get().cancel_transaction_id == Some(transaction_id))
    }

    /// Consumes a pending cancellation for `transaction_id`, returning `true` if it was set.
    pub fn take_cancel(&self, transaction_id: u32) -> bool {
        self.inner.lock(|c| {
            let mut inner = c.get();
            if inner.cancel_transaction_id == Some(transaction_id) {
                inner.cancel_transaction_id = None;
                inner.status = DeviceStatusCode::OK;
                c.set(inner);
                true
            } else {
                false
            }
        })
    }

    /// Consumes any pending `DeviceReset` request, returning `true` if one arrived.
    pub fn take_reset(&self) -> bool {
        self.inner.lock(|c| {
            let mut inner = c.get();
            let was = inner.reset_requested;
            inner.reset_requested = false;
            if was {
                inner.cancel_transaction_id = None;
                inner.status = DeviceStatusCode::OK;
                c.set(inner);
            }
            was
        })
    }
}

/// Internal state for the MTP device class.
pub struct State<'d> {
    pub(super) control: MaybeUninit<Control<'d>>,
    pub(super) shared: ControlShared,
}

impl<'d> Default for State<'d> {
    fn default() -> Self {
        Self::new()
    }
}

impl<'d> State<'d> {
    /// Creates a new `State`.
    pub const fn new() -> Self {
        Self {
            control: MaybeUninit::uninit(),
            shared: ControlShared::new(),
        }
    }
}

pub(super) struct Control<'d> {
    pub(super) interface: InterfaceNumber,
    pub(super) shared: &'d ControlShared,
}

impl<'d> Handler for Control<'d> {
    fn reset(&mut self) {
        self.shared.inner.lock(|c| c.set(ControlInner::new()));
    }

    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        if (req.request_type, req.recipient, req.index)
            != (RequestType::Class, Recipient::Interface, self.interface.0 as u16)
        {
            return None;
        }

        match req.request {
            REQ_CANCEL if req.value == 0 && req.length == 6 && data.len() == 6 => {
                let code = u16::from_le_bytes([data[0], data[1]]);
                let tx_id = u32::from_le_bytes([data[2], data[3], data[4], data[5]]);
                if code != CANCEL_CODE {
                    return Some(OutResponse::Rejected);
                }
                self.shared.inner.lock(|c| {
                    let mut inner = c.get();
                    inner.cancel_transaction_id = Some(tx_id);
                    inner.status = DeviceStatusCode::TRANSACTION_CANCELLED;
                    c.set(inner);
                });
                Some(OutResponse::Accepted)
            }
            REQ_DEVICE_RESET if req.value == 0 && req.length == 0 && data.is_empty() => {
                self.shared.inner.lock(|c| {
                    let mut inner = c.get();
                    inner.reset_requested = true;
                    inner.cancel_transaction_id = None;
                    inner.status = DeviceStatusCode::OK;
                    c.set(inner);
                });
                Some(OutResponse::Accepted)
            }
            _ => Some(OutResponse::Rejected),
        }
    }

    fn control_in<'b>(&'b mut self, req: Request, buf: &'b mut [u8]) -> Option<InResponse<'b>> {
        if (req.request_type, req.recipient, req.index)
            != (RequestType::Class, Recipient::Interface, self.interface.0 as u16)
        {
            return None;
        }

        match req.request {
            REQ_GET_DEVICE_STATUS if req.value == 0 && req.length >= 4 && buf.len() >= 4 => {
                let status = self.shared.status().0;
                buf[0..2].copy_from_slice(&4u16.to_le_bytes());
                buf[2..4].copy_from_slice(&status.to_le_bytes());
                Some(InResponse::Accepted(&buf[..4]))
            }
            REQ_GET_EXTENDED_EVENT_DATA => Some(InResponse::Rejected),
            _ => Some(InResponse::Rejected),
        }
    }
}

impl<'d> InterfaceHandler for Control<'d> {
    fn owns(&self, iface: InterfaceNumber) -> bool {
        iface == self.interface
    }
}

#[cfg(test)]
mod tests {
    use embassy_usb_driver::Direction;

    use super::*;

    #[test]
    fn cancel_and_get_status() {
        let shared = ControlShared::new();
        let mut ctrl = Control {
            interface: InterfaceNumber(0),
            shared: &shared,
        };

        let mut buf = [0u8; 16];
        let status_req = Request {
            direction: Direction::In,
            request_type: RequestType::Class,
            recipient: Recipient::Interface,
            request: REQ_GET_DEVICE_STATUS,
            value: 0,
            index: 0,
            length: 4,
        };
        let resp = ctrl.control_in(status_req, &mut buf);
        assert!(matches!(resp, Some(InResponse::Accepted(b)) if b == [4, 0, 0x01, 0x20]));

        let mut payload = [0u8; 6];
        payload[0..2].copy_from_slice(&CANCEL_CODE.to_le_bytes());
        payload[2..6].copy_from_slice(&0x1234_5678u32.to_le_bytes());
        let cancel_req = Request {
            direction: Direction::Out,
            request_type: RequestType::Class,
            recipient: Recipient::Interface,
            request: REQ_CANCEL,
            value: 0,
            index: 0,
            length: 6,
        };
        assert!(matches!(
            ctrl.control_out(cancel_req, &payload),
            Some(OutResponse::Accepted)
        ));
        assert!(shared.is_cancelled(0x1234_5678));

        let resp = ctrl.control_in(status_req, &mut buf);
        assert!(matches!(resp, Some(InResponse::Accepted(b)) if b == [4, 0, 0x1F, 0x20]));

        assert!(shared.take_cancel(0x1234_5678));
        assert!(!shared.is_cancelled(0x1234_5678));
        assert_eq!(shared.status(), DeviceStatusCode::OK);
    }

    #[test]
    fn device_reset_clears_cancel() {
        let shared = ControlShared::new();
        let mut ctrl = Control {
            interface: InterfaceNumber(1),
            shared: &shared,
        };

        let mut payload = [0u8; 6];
        payload[0..2].copy_from_slice(&CANCEL_CODE.to_le_bytes());
        payload[2..6].copy_from_slice(&99u32.to_le_bytes());
        let cancel_req = Request {
            direction: Direction::Out,
            request_type: RequestType::Class,
            recipient: Recipient::Interface,
            request: REQ_CANCEL,
            value: 0,
            index: 1,
            length: 6,
        };
        ctrl.control_out(cancel_req, &payload);
        assert!(shared.is_cancelled(99));

        let reset_req = Request {
            direction: Direction::Out,
            request_type: RequestType::Class,
            recipient: Recipient::Interface,
            request: REQ_DEVICE_RESET,
            value: 0,
            index: 1,
            length: 0,
        };
        assert!(matches!(ctrl.control_out(reset_req, &[]), Some(OutResponse::Accepted)));
        assert!(shared.take_reset());
        assert!(!shared.is_cancelled(99));
        assert_eq!(shared.status(), DeviceStatusCode::OK);
    }
}
