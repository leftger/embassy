//! USB control data types and request builders.
use core::num::NonZeroU8;

use embassy_usb_driver::Direction;
pub use embassy_usb_driver::host::pipe;
use embassy_usb_driver::host::{HostError, UsbPipe};

use crate::descriptor::{USBDescriptor, descriptor_type};

/// Control request type.
#[repr(u8)]
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum RequestType {
    /// Request is a USB standard request. Usually handled by
    /// [`UsbDevice`](crate::UsbDevice).
    Standard = 0,
    /// Request is intended for a USB class.
    Class = 1,
    /// Request is vendor-specific.
    Vendor = 2,
    /// Reserved.
    Reserved = 3,
}

/// Compatibility alias for [`RequestType`].
pub type ControlType = RequestType;

/// Control request recipient.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Recipient {
    /// Request is intended for the entire device.
    Device = 0,
    /// Request is intended for an interface. Generally, the `index` field of the request specifies
    /// the interface number.
    Interface = 1,
    /// Request is intended for an endpoint. Generally, the `index` field of the request specifies
    /// the endpoint address.
    Endpoint = 2,
    /// None of the above.
    Other = 3,
    /// Reserved.
    Reserved = 4,
}

/// A control request read from or written to a SETUP packet.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Request {
    /// Direction of the request.
    pub direction: Direction,
    /// Type of the request.
    pub request_type: RequestType,
    /// Recipient of the request.
    pub recipient: Recipient,
    /// Request code. The meaning of the value depends on the previous fields.
    pub request: u8,
    /// Request value. The meaning of the value depends on the previous fields.
    pub value: u16,
    /// Request index. The meaning of the value depends on the previous fields.
    pub index: u16,
    /// Length of the DATA stage. For control OUT transfers this is the exact length of the data the
    /// host sent. For control IN transfers this is the maximum length of data the device should
    /// return.
    pub length: u16,
}

/// Alias for [`Request`] when working with host-side SETUP packets.
pub type SetupPacket = Request;

/// HID class descriptor type: Report (HID 1.11 §7.1.1).
const HID_REPORT_DESCRIPTOR_TYPE: u8 = 0x22;

impl Request {
    /// Standard USB control request Get Status
    pub const GET_STATUS: u8 = 0;

    /// Standard USB control request Clear Feature
    pub const CLEAR_FEATURE: u8 = 1;

    /// Standard USB control request Set Feature
    pub const SET_FEATURE: u8 = 3;

    /// Standard USB control request Set Address
    pub const SET_ADDRESS: u8 = 5;

    /// Standard USB control request Get Descriptor
    pub const GET_DESCRIPTOR: u8 = 6;

    /// Standard USB control request Set Descriptor
    pub const SET_DESCRIPTOR: u8 = 7;

    /// Standard USB control request Get Configuration
    pub const GET_CONFIGURATION: u8 = 8;

    /// Standard USB control request Set Configuration
    pub const SET_CONFIGURATION: u8 = 9;

    /// Standard USB control request Get Interface
    pub const GET_INTERFACE: u8 = 10;

    /// Standard USB control request Set Interface
    pub const SET_INTERFACE: u8 = 11;

    /// Standard USB control request Synch Frame
    pub const SYNCH_FRAME: u8 = 12;

    /// Standard USB feature Endpoint Halt for Set/Clear Feature
    pub const FEATURE_ENDPOINT_HALT: u16 = 0;

    /// Standard USB feature Device Remote Wakeup for Set/Clear Feature
    pub const FEATURE_DEVICE_REMOTE_WAKEUP: u16 = 1;

    /// Standard USB feature Device Test Mode for Set Feature
    pub const FEATURE_DEVICE_TEST_MODE: u16 = 2;

    /// Standard USB feature Device Debug Mode for Set Feature
    pub const FEATURE_DEVICE_DEBUG_MODE: u16 = 6;

    /// Parses a USB control request from an 8-byte array.
    pub fn parse(buf: &[u8; 8]) -> Request {
        let rt = buf[0];
        let recipient = rt & 0b11111;

        Request {
            direction: if rt & 0x80 == 0 { Direction::Out } else { Direction::In },
            request_type: match (rt >> 5) & 0b11 {
                0 => RequestType::Standard,
                1 => RequestType::Class,
                2 => RequestType::Vendor,
                _ => RequestType::Reserved,
            },
            recipient: match recipient {
                0 => Recipient::Device,
                1 => Recipient::Interface,
                2 => Recipient::Endpoint,
                3 => Recipient::Other,
                _ => Recipient::Reserved,
            },
            request: buf[1],
            value: u16::from_le_bytes([buf[2], buf[3]]),
            index: u16::from_le_bytes([buf[4], buf[5]]),
            length: u16::from_le_bytes([buf[6], buf[7]]),
        }
    }

    /// Alias for [`Request::parse`].
    pub fn from_bytes(wire: [u8; 8]) -> Self {
        Self::parse(&wire)
    }

    /// Serializes this control request to an 8-byte SETUP packet.
    pub const fn to_bytes(&self) -> [u8; 8] {
        let d = match self.direction {
            Direction::Out => 0,
            Direction::In => 1 << 7,
        };
        let t = (self.request_type as u8) << 5;
        let r = match self.recipient {
            Recipient::Device => 0,
            Recipient::Interface => 1,
            Recipient::Endpoint => 2,
            Recipient::Other => 3,
            Recipient::Reserved => 4,
        };
        let v = self.value.to_le_bytes();
        let i = self.index.to_le_bytes();
        let l = self.length.to_le_bytes();
        [d | t | r, self.request, v[0], v[1], i[0], i[1], l[0], l[1]]
    }

    /// Gets the descriptor type and index from the value field of a GET_DESCRIPTOR request.
    pub const fn descriptor_type_index(&self) -> (u8, u8) {
        ((self.value >> 8) as u8, self.value as u8)
    }

    /// Build a GET_DESCRIPTOR request delivered to the Device recipient.
    pub const fn get_descriptor(class: bool, desc_type: u8, index: u8, max_len: u16) -> Self {
        Self {
            direction: Direction::In,
            request_type: if class { RequestType::Class } else { RequestType::Standard },
            recipient: Recipient::Device,
            request: Self::GET_DESCRIPTOR,
            value: ((desc_type as u16) << 8) | index as u16,
            index: 0,
            length: max_len,
        }
    }

    /// Build a GET_DESCRIPTOR(Device) request.
    pub const fn get_device_descriptor(max_len: u16) -> Self {
        Self::get_descriptor(false, descriptor_type::DEVICE, 0, max_len)
    }

    /// Build a GET_DESCRIPTOR(Configuration) request.
    pub const fn get_config_descriptor(index: u8, max_len: u16) -> Self {
        Self::get_descriptor(false, descriptor_type::CONFIGURATION, index, max_len)
    }

    /// Build a GET_DESCRIPTOR(String) request.
    pub const fn get_string_descriptor(index: u8, lang_id: u16, max_len: u16) -> Self {
        Self {
            direction: Direction::In,
            request_type: RequestType::Standard,
            recipient: Recipient::Device,
            request: Self::GET_DESCRIPTOR,
            value: ((descriptor_type::STRING as u16) << 8) | index as u16,
            index: lang_id,
            length: max_len,
        }
    }

    /// Build a standard GET_DESCRIPTOR request delivered to an Interface recipient.
    pub const fn get_interface_descriptor(desc_type: u8, interface: u16, max_len: u16) -> Self {
        Self {
            direction: Direction::In,
            request_type: RequestType::Standard,
            recipient: Recipient::Interface,
            request: Self::GET_DESCRIPTOR,
            value: (desc_type as u16) << 8,
            index: interface,
            length: max_len,
        }
    }

    /// Build a GET_DESCRIPTOR(HID Report Descriptor) request.
    pub const fn get_hid_report_descriptor(interface: u8, len: u16) -> Self {
        Self::get_interface_descriptor(HID_REPORT_DESCRIPTOR_TYPE, interface as u16, len)
    }

    /// Build a SET_ADDRESS request.
    pub const fn set_address(address: u8) -> Self {
        Self {
            direction: Direction::Out,
            request_type: RequestType::Standard,
            recipient: Recipient::Device,
            request: Self::SET_ADDRESS,
            value: address as u16,
            index: 0,
            length: 0,
        }
    }

    /// Build a SET_CONFIGURATION request.
    pub const fn set_configuration(config_value: u8) -> Self {
        Self {
            direction: Direction::Out,
            request_type: RequestType::Standard,
            recipient: Recipient::Device,
            request: Self::SET_CONFIGURATION,
            value: config_value as u16,
            index: 0,
            length: 0,
        }
    }

    /// Build a GET_CONFIGURATION request.
    pub const fn get_configuration() -> Self {
        Self {
            direction: Direction::In,
            request_type: RequestType::Standard,
            recipient: Recipient::Device,
            request: Self::GET_CONFIGURATION,
            value: 0,
            index: 0,
            length: 1,
        }
    }

    /// Build a class-specific interface request, host-to-device.
    pub const fn class_interface_out(request: u8, value: u16, interface: u16, length: u16) -> Self {
        Self {
            direction: Direction::Out,
            request_type: RequestType::Class,
            recipient: Recipient::Interface,
            request,
            value,
            index: interface,
            length,
        }
    }

    /// Build a class-specific interface request, device-to-host.
    pub const fn class_interface_in(request: u8, value: u16, interface: u16, length: u16) -> Self {
        Self {
            direction: Direction::In,
            request_type: RequestType::Class,
            recipient: Recipient::Interface,
            request,
            value,
            index: interface,
            length,
        }
    }

    /// Build a vendor-specific interface request, host-to-device.
    pub const fn vendor_interface_out(request: u8, value: u16, interface: u16, length: u16) -> Self {
        Self {
            direction: Direction::Out,
            request_type: RequestType::Vendor,
            recipient: Recipient::Interface,
            request,
            value,
            index: interface,
            length,
        }
    }

    /// Build a vendor-specific interface request, device-to-host.
    pub const fn vendor_interface_in(request: u8, value: u16, interface: u16, length: u16) -> Self {
        Self {
            direction: Direction::In,
            request_type: RequestType::Vendor,
            recipient: Recipient::Interface,
            request,
            value,
            index: interface,
            length,
        }
    }
}

/// Extension trait providing higher-level control request methods on a USB control pipe.
pub trait ControlPipeExt<D: pipe::Direction>: UsbPipe<pipe::Control, D> {
    /// Request and parse a fixed-size descriptor.
    async fn request_descriptor<T: USBDescriptor, const SIZE: usize>(
        &mut self,
        index: u8,
        class: bool,
    ) -> Result<T, HostError>
    where
        D: pipe::IsIn,
    {
        let mut buf = [0u8; SIZE];
        let setup = Request::get_descriptor(class, T::DESC_TYPE, index, SIZE as u16);
        self.control_in(&setup.to_bytes(), &mut buf).await?;
        trace!("Descriptor {}: {:?}", core::any::type_name::<T>(), buf);
        T::try_from_bytes(&buf).map_err(|_| HostError::InvalidDescriptor)
    }

    /// Request the raw bytes of a descriptor by type and index.
    async fn request_descriptor_bytes(&mut self, desc_type: u8, index: u8, buf: &mut [u8]) -> Result<usize, HostError>
    where
        D: pipe::IsIn,
    {
        let setup = Request::get_descriptor(false, desc_type, index, buf.len() as u16);
        self.control_in(&setup.to_bytes(), buf)
            .await
            .map_err(HostError::PipeError)
    }

    /// Request the raw bytes of a class-specific interface descriptor.
    async fn interface_request_descriptor_bytes<T: USBDescriptor>(
        &mut self,
        interface_num: u8,
        buf: &mut [u8],
    ) -> Result<usize, HostError>
    where
        D: pipe::IsIn,
    {
        let setup = Request::get_interface_descriptor(T::DESC_TYPE, interface_num as u16, buf.len() as u16);
        self.control_in(&setup.to_bytes(), buf)
            .await
            .map_err(HostError::PipeError)
    }

    /// GET_CONFIGURATION — returns the active configuration value, or `None` if unconfigured.
    async fn active_configuration_value(&mut self) -> Result<Option<NonZeroU8>, HostError>
    where
        D: pipe::IsIn,
    {
        let setup = Request::get_configuration();
        let mut buf = [0u8; 1];
        crate::host::retry_descriptor(async || self.control_in(&setup.to_bytes(), &mut buf).await).await?;
        Ok(NonZeroU8::new(buf[0]))
    }

    /// SET_CONFIGURATION.
    async fn set_configuration(&mut self, config_no: u8) -> Result<(), HostError>
    where
        D: pipe::IsOut,
    {
        let setup = Request::set_configuration(config_no);
        self.control_out(&setup.to_bytes(), &[]).await?;
        Ok(())
    }

    /// SET_ADDRESS — assign the device a new address.
    ///
    /// # Warning
    /// Breaks host channel state; use only during enumeration.
    async fn device_set_address(&mut self, new_addr: u8) -> Result<(), HostError>
    where
        D: pipe::IsOut,
    {
        let setup = Request::set_address(new_addr);
        self.control_out(&setup.to_bytes(), &[]).await?;
        Ok(())
    }

    /// Class + Interface OUT request (no data stage).
    async fn class_request_out(&mut self, request: u8, value: u16, index: u16, buf: &[u8]) -> Result<(), HostError>
    where
        D: pipe::IsOut,
    {
        let setup = Request::class_interface_out(request, value, index, buf.len() as u16);
        self.control_out(&setup.to_bytes(), buf).await?;
        Ok(())
    }
}

impl<D: pipe::Direction, C> ControlPipeExt<D> for C where C: UsbPipe<pipe::Control, D> {}

/// Response for a CONTROL OUT request.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OutResponse {
    /// The request was accepted.
    Accepted,
    /// The request was rejected.
    Rejected,
}

/// Response for a CONTROL IN request.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InResponse<'a> {
    /// The request was accepted. The buffer contains the response data.
    Accepted(&'a [u8]),
    /// The request was rejected.
    Rejected,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrip_setup_packet() {
        let directions = [Direction::In, Direction::Out];
        let control_types = [RequestType::Standard, RequestType::Class, RequestType::Vendor];
        let recipients = [
            Recipient::Device,
            Recipient::Interface,
            Recipient::Endpoint,
            Recipient::Other,
        ];
        for direction in directions {
            for request_type in control_types {
                for recipient in recipients {
                    let setup = Request {
                        direction,
                        request_type,
                        recipient,
                        request: 0x11,
                        value: 0x2233,
                        index: 0x4455,
                        length: 0x6677,
                    };
                    let bytes = setup.to_bytes();
                    assert_eq!(setup, Request::from_bytes(bytes));
                }
            }
        }
    }
}
