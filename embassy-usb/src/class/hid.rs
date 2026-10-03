//! USB HID (Human Interface Device) class implementation.

use core::mem::MaybeUninit;
use core::ops::Range;
use core::sync::atomic::{AtomicUsize, Ordering};

#[cfg(feature = "usbd-hid")]
use usbd_hid::descriptor::AsInputReport;

use embassy_usb_driver::host::{PipeError, UsbHostAllocator, UsbPipe, pipe};
use embassy_usb_driver::{EndpointInfo, EndpointType};

pub use super::hid_report::{ReportDescriptor, ReportField};
use crate::control::{InResponse, OutResponse, Recipient, Request, RequestType, SetupPacket};
use crate::descriptor::ConfigurationDescriptorChain;
use crate::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut};
use crate::host::EnumerationInfo;
use crate::types::InterfaceNumber;
use crate::{Builder, Handler};

const USB_CLASS_HID: u8 = 0x03;

// HID
const HID_DESC_DESCTYPE_HID: u8 = 0x21;
const HID_DESC_DESCTYPE_HID_REPORT: u8 = 0x22;
const HID_DESC_SPEC_1_10: [u8; 2] = [0x10, 0x01];
const HID_DESC_COUNTRY_UNSPEC: u8 = 0x00;

const HID_REQ_SET_IDLE: u8 = 0x0a;
const HID_REQ_GET_IDLE: u8 = 0x02;
const HID_REQ_GET_REPORT: u8 = 0x01;
const HID_REQ_SET_REPORT: u8 = 0x09;
const HID_REQ_GET_PROTOCOL: u8 = 0x03;
const HID_REQ_SET_PROTOCOL: u8 = 0x0b;

/// Get/Set Protocol mapping
/// See (7.2.5 and 7.2.6): <https://www.usb.org/sites/default/files/hid1_11.pdf>
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum HidProtocolMode {
    /// Hid Boot Protocol Mode
    Boot = 0,
    /// Hid Report Protocol Mode
    Report = 1,
}

impl From<u8> for HidProtocolMode {
    fn from(mode: u8) -> HidProtocolMode {
        if mode == HidProtocolMode::Boot as u8 {
            HidProtocolMode::Boot
        } else {
            HidProtocolMode::Report
        }
    }
}

/// USB HID interface subclass values.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum HidSubclass {
    /// No subclass, standard HID device.
    No = 0,
    /// Boot interface subclass, supports BIOS boot protocol.
    Boot = 1,
}

/// USB HID protocol values.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[repr(u8)]
pub enum HidBootProtocol {
    /// No boot protocol.
    None = 0,
    /// Keyboard boot protocol.
    Keyboard = 1,
    /// Mouse boot protocol.
    Mouse = 2,
}

/// Configuration for the HID class.
pub struct Config<'d> {
    /// HID report descriptor.
    pub report_descriptor: &'d [u8],

    /// Handler for control requests.
    pub request_handler: Option<&'d mut dyn RequestHandler>,

    /// Configures how frequently the host should poll for reading/writing HID reports.
    ///
    /// A lower value means better throughput & latency, at the expense
    /// of CPU on the device & bandwidth on the bus. A value of 10 is reasonable for
    /// high performance uses, and a value of 255 is good for best-effort usecases.
    pub poll_ms: u8,

    /// Max packet size for both the IN and OUT endpoints.
    pub max_packet_size: u16,

    /// The HID subclass of this interface
    pub hid_subclass: HidSubclass,

    /// The HID boot protocol of this interface
    pub hid_boot_protocol: HidBootProtocol,
}

/// Report ID
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReportId {
    /// IN report
    In(u8),
    /// OUT report
    Out(u8),
    /// Feature report
    Feature(u8),
}

impl ReportId {
    const fn try_from(value: u16) -> Result<Self, ()> {
        match value >> 8 {
            1 => Ok(ReportId::In(value as u8)),
            2 => Ok(ReportId::Out(value as u8)),
            3 => Ok(ReportId::Feature(value as u8)),
            _ => Err(()),
        }
    }
}

/// Internal state for USB HID.
pub struct State<'d> {
    control: MaybeUninit<Control<'d>>,
    out_report_offset: AtomicUsize,
}

impl<'d> Default for State<'d> {
    fn default() -> Self {
        Self::new()
    }
}

impl<'d> State<'d> {
    /// Create a new `State`.
    pub const fn new() -> Self {
        State {
            control: MaybeUninit::uninit(),
            out_report_offset: AtomicUsize::new(0),
        }
    }
}

/// USB HID reader/writer.
pub struct HidReaderWriter<'d, D: Driver<'d>, const READ_N: usize, const WRITE_N: usize> {
    reader: HidReader<'d, D, READ_N>,
    writer: HidWriter<'d, D, WRITE_N>,
    interface_number: InterfaceNumber,
}

fn build<'d, D: Driver<'d>>(
    builder: &mut Builder<'d, D>,
    state: &'d mut State<'d>,
    config: Config<'d>,
    with_out_endpoint: bool,
) -> (Option<D::EndpointOut>, D::EndpointIn, &'d AtomicUsize, InterfaceNumber) {
    let len = config.report_descriptor.len();

    let mut func = builder.function(USB_CLASS_HID, config.hid_subclass as u8, config.hid_boot_protocol as u8);
    let mut iface = func.interface();
    let if_num = iface.interface_number();
    let mut alt = iface.alt_setting(
        USB_CLASS_HID,
        config.hid_subclass as u8,
        config.hid_boot_protocol as u8,
        None,
    );

    // HID descriptor
    alt.descriptor(
        HID_DESC_DESCTYPE_HID,
        &[
            // HID Class spec version
            HID_DESC_SPEC_1_10[0],
            HID_DESC_SPEC_1_10[1],
            // Country code not supported
            HID_DESC_COUNTRY_UNSPEC,
            // Number of following descriptors
            1,
            // We have a HID report descriptor the host should read
            HID_DESC_DESCTYPE_HID_REPORT,
            // HID report descriptor size,
            (len & 0xFF) as u8,
            (len >> 8 & 0xFF) as u8,
        ],
    );

    let ep_in = alt.endpoint_interrupt_in(None, config.max_packet_size, config.poll_ms);
    let ep_out = if with_out_endpoint {
        Some(alt.endpoint_interrupt_out(None, config.max_packet_size, config.poll_ms))
    } else {
        None
    };

    drop(func);

    let control = state.control.write(Control::new(
        if_num,
        config.report_descriptor,
        config.request_handler,
        &state.out_report_offset,
    ));
    builder.handler(control);

    (ep_out, ep_in, &state.out_report_offset, if_num)
}

impl<'d, D: Driver<'d>, const READ_N: usize, const WRITE_N: usize> HidReaderWriter<'d, D, READ_N, WRITE_N> {
    /// Creates a new `HidReaderWriter`.
    ///
    /// This will allocate one IN and one OUT endpoints. If you only need writing (sending)
    /// HID reports, consider using [`HidWriter::new`] instead, which allocates an IN endpoint only.
    ///
    pub fn new(builder: &mut Builder<'d, D>, state: &'d mut State<'d>, config: Config<'d>) -> Self {
        let (ep_out, ep_in, offset, if_num) = build(builder, state, config, true);

        Self {
            reader: HidReader {
                ep_out: ep_out.unwrap(),
                offset,
            },
            writer: HidWriter { ep_in },
            interface_number: if_num,
        }
    }

    /// Splits into separate readers/writers for input and output reports.
    pub fn split(self) -> (HidReader<'d, D, READ_N>, HidWriter<'d, D, WRITE_N>) {
        (self.reader, self.writer)
    }

    /// Waits for both IN and OUT endpoints to be enabled.
    pub async fn ready(&mut self) {
        self.reader.ready().await;
        self.writer.ready().await;
    }

    /// Writes an input report by serializing the given report structure.
    #[cfg(feature = "usbd-hid")]
    pub async fn write_serialize<IR: AsInputReport>(&mut self, r: &IR) -> Result<(), EndpointError> {
        self.writer.write_serialize(r).await
    }

    /// Writes `report` to its interrupt endpoint.
    pub async fn write(&mut self, report: &[u8]) -> Result<(), EndpointError> {
        self.writer.write(report).await
    }

    /// Reads an output report from the Interrupt Out pipe.
    ///
    /// See [`HidReader::read`].
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ReadError> {
        self.reader.read(buf).await
    }

    /// Get the HID's interface number.
    pub const fn interface_number(&self) -> u8 {
        self.interface_number.0
    }
}

/// USB HID writer.
///
/// You can obtain a `HidWriter` using [`HidReaderWriter::split`].
pub struct HidWriter<'d, D: Driver<'d>, const N: usize> {
    ep_in: D::EndpointIn,
}

/// USB HID reader.
///
/// You can obtain a `HidReader` using [`HidReaderWriter::split`].
pub struct HidReader<'d, D: Driver<'d>, const N: usize> {
    ep_out: D::EndpointOut,
    offset: &'d AtomicUsize,
}

/// Error when reading a HID report.
#[derive(Debug, Clone, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ReadError {
    /// The given buffer was too small to read the received report.
    BufferOverflow,
    /// The endpoint is disabled.
    Disabled,
    /// The report was only partially read. See [`HidReader::read`] for details.
    Sync(Range<usize>),
}

impl From<EndpointError> for ReadError {
    fn from(val: EndpointError) -> Self {
        use EndpointError::{BufferOverflow, Disabled};
        match val {
            BufferOverflow => ReadError::BufferOverflow,
            Disabled => ReadError::Disabled,
        }
    }
}

impl<'d, D: Driver<'d>, const N: usize> HidWriter<'d, D, N> {
    /// Creates a new HidWriter.
    ///
    /// This will allocate one IN endpoint only, so the host won't be able to send
    /// reports to us. If you need that, consider using [`HidReaderWriter::new`] instead.
    ///
    /// poll_ms configures how frequently the host should poll for reading/writing
    /// HID reports. A lower value means better throughput & latency, at the expense
    /// of CPU on the device & bandwidth on the bus. A value of 10 is reasonable for
    /// high performance uses, and a value of 255 is good for best-effort usecases.
    pub fn new(builder: &mut Builder<'d, D>, state: &'d mut State<'d>, config: Config<'d>) -> Self {
        let (ep_out, ep_in, _offset, _) = build(builder, state, config, false);

        assert!(ep_out.is_none());

        Self { ep_in }
    }

    /// Waits for the interrupt in endpoint to be enabled.
    pub async fn ready(&mut self) {
        self.ep_in.wait_enabled().await;
    }

    /// Writes an input report by serializing the given report structure.
    #[cfg(feature = "usbd-hid")]
    pub async fn write_serialize<IR: AsInputReport>(&mut self, r: &IR) -> Result<(), EndpointError> {
        let mut buf: [u8; N] = [0; N];
        let Ok(size) = r.serialize(&mut buf) else {
            return Err(EndpointError::BufferOverflow);
        };
        self.write(&buf[0..size]).await
    }

    /// Writes `report` to its interrupt endpoint.
    pub async fn write(&mut self, report: &[u8]) -> Result<(), EndpointError> {
        assert!(report.len() <= N);
        self.ep_in.write_transfer(report, report.len() < N).await
    }
}

impl<'d, D: Driver<'d>, const N: usize> HidReader<'d, D, N> {
    /// Waits for the interrupt out endpoint to be enabled.
    pub async fn ready(&mut self) {
        self.ep_out.wait_enabled().await;
    }

    /// Delivers output reports from the Interrupt Out pipe to `handler`.
    ///
    /// If `use_report_ids` is true, the first byte of the report will be used as
    /// the `ReportId` value. Otherwise the `ReportId` value will be 0.
    pub async fn run<T: RequestHandler>(mut self, use_report_ids: bool, handler: &mut T) -> ! {
        let offset = self.offset.load(Ordering::Acquire);
        assert!(offset == 0);
        let mut buf = [0; N];
        loop {
            match self.read(&mut buf).await {
                Ok(len) => {
                    let id = if use_report_ids { buf[0] } else { 0 };
                    handler.set_report(ReportId::Out(id), &buf[..len]);
                }
                Err(ReadError::BufferOverflow) => warn!(
                    "Host sent output report larger than the configured maximum output report length ({})",
                    N
                ),
                Err(ReadError::Disabled) => self.ep_out.wait_enabled().await,
                Err(ReadError::Sync(_)) => unreachable!(),
            }
        }
    }

    /// Reads an output report from the Interrupt Out pipe.
    ///
    /// **Note:** Any reports sent from the host over the control pipe will be
    /// passed to [`RequestHandler::set_report()`] for handling. The application
    /// is responsible for ensuring output reports from both pipes are handled
    /// correctly.
    ///
    /// **Note:** If `N` > the maximum packet size of the endpoint (i.e. output
    /// reports may be split across multiple packets) and this method's future
    /// is dropped after some packets have been read, the next call to `read()`
    /// will return a [`ReadError::Sync`]. The range in the sync error
    /// indicates the portion `buf` that was filled by the current call to
    /// `read()`. If the dropped future used the same `buf`, then `buf` will
    /// contain the full report.
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, ReadError> {
        assert!(N != 0);
        assert!(buf.len() >= N);

        // Read packets from the endpoint
        let max_packet_size = usize::from(self.ep_out.info().max_packet_size);
        let starting_offset = self.offset.load(Ordering::Acquire);
        let mut total = starting_offset;
        loop {
            for chunk in buf[starting_offset..N].chunks_mut(max_packet_size) {
                match self.ep_out.read(chunk).await {
                    Ok(size) => {
                        total += size;
                        if size < max_packet_size || total == N {
                            self.offset.store(0, Ordering::Release);
                            break;
                        }
                        self.offset.store(total, Ordering::Release);
                    }
                    Err(err) => {
                        self.offset.store(0, Ordering::Release);
                        return Err(err.into());
                    }
                }
            }

            // Some hosts may send ZLPs even when not required by the HID spec, so we'll loop as long as total == 0.
            if total > 0 {
                break;
            }
        }

        if starting_offset > 0 {
            Err(ReadError::Sync(starting_offset..total))
        } else {
            Ok(total)
        }
    }
}

/// Handler for HID-related control requests.
pub trait RequestHandler {
    /// Reads the value of report `id` into `buf` returning the size.
    ///
    /// Returns `None` if `id` is invalid or no data is available.
    fn get_report(&mut self, id: ReportId, buf: &mut [u8]) -> Option<usize> {
        let _ = (id, buf);
        None
    }

    /// Sets the value of report `id` to `data`.
    fn set_report(&mut self, id: ReportId, data: &[u8]) -> OutResponse {
        let _ = (id, data);
        OutResponse::Rejected
    }

    /// Gets the current hid protocol.
    ///
    /// Returns `Report` protocol by default.
    fn get_protocol(&self) -> HidProtocolMode {
        HidProtocolMode::Report
    }

    /// Sets the current hid protocol to `protocol`.
    ///
    /// Accepts only `Report` protocol by default.
    fn set_protocol(&mut self, protocol: HidProtocolMode) -> OutResponse {
        match protocol {
            HidProtocolMode::Report => OutResponse::Accepted,
            HidProtocolMode::Boot => OutResponse::Rejected,
        }
    }

    /// Get the idle rate for `id`.
    ///
    /// If `id` is `None`, get the idle rate for all reports. Returning `None`
    /// will reject the control request. Any duration at or above 1.024 seconds
    /// or below 4ms will be returned as an indefinite idle rate.
    fn get_idle_ms(&mut self, id: Option<ReportId>) -> Option<u32> {
        let _ = id;
        None
    }

    /// Set the idle rate for `id` to `dur`.
    ///
    /// If `id` is `None`, set the idle rate of all input reports to `dur`. If
    /// an indefinite duration is requested, `dur` will be set to `u32::MAX`.
    fn set_idle_ms(&mut self, id: Option<ReportId>, duration_ms: u32) {
        let _ = (id, duration_ms);
    }
}

struct Control<'d> {
    if_num: InterfaceNumber,
    report_descriptor: &'d [u8],
    request_handler: Option<&'d mut dyn RequestHandler>,
    out_report_offset: &'d AtomicUsize,
    hid_descriptor: [u8; 9],
}

impl<'d> Control<'d> {
    fn new(
        if_num: InterfaceNumber,
        report_descriptor: &'d [u8],
        request_handler: Option<&'d mut dyn RequestHandler>,
        out_report_offset: &'d AtomicUsize,
    ) -> Self {
        Control {
            if_num,
            report_descriptor,
            request_handler,
            out_report_offset,
            hid_descriptor: [
                // Length of buf inclusive of size prefix
                9,
                // Descriptor type
                HID_DESC_DESCTYPE_HID,
                // HID Class spec version
                HID_DESC_SPEC_1_10[0],
                HID_DESC_SPEC_1_10[1],
                // Country code not supported
                HID_DESC_COUNTRY_UNSPEC,
                // Number of following descriptors
                1,
                // We have a HID report descriptor the host should read
                HID_DESC_DESCTYPE_HID_REPORT,
                // HID report descriptor size,
                (report_descriptor.len() & 0xFF) as u8,
                (report_descriptor.len() >> 8 & 0xFF) as u8,
            ],
        }
    }
}

impl<'d> Handler for Control<'d> {
    fn reset(&mut self) {
        self.out_report_offset.store(0, Ordering::Release);
    }

    fn control_out(&mut self, req: Request, data: &[u8]) -> Option<OutResponse> {
        if (req.request_type, req.recipient, req.index)
            != (RequestType::Class, Recipient::Interface, self.if_num.0 as u16)
        {
            return None;
        }

        // This uses a defmt-specific formatter that causes use of the `log`
        // feature to fail to build, so leave it defmt-specific for now.
        #[cfg(feature = "defmt")]
        trace!("HID control_out {:?} {=[u8]:x}", req, data);
        match req.request {
            HID_REQ_SET_IDLE => {
                if let Some(handler) = self.request_handler.as_mut() {
                    let id = req.value as u8;
                    let id = (id != 0).then_some(ReportId::In(id));
                    let dur = u32::from(req.value >> 8);
                    let dur = if dur == 0 { u32::MAX } else { 4 * dur };
                    handler.set_idle_ms(id, dur);
                }
                Some(OutResponse::Accepted)
            }
            HID_REQ_SET_REPORT => match (ReportId::try_from(req.value), self.request_handler.as_mut()) {
                (Ok(id), Some(handler)) => Some(handler.set_report(id, data)),
                _ => Some(OutResponse::Rejected),
            },
            HID_REQ_SET_PROTOCOL => {
                let hid_protocol = HidProtocolMode::from(req.value as u8);
                match (self.request_handler.as_mut(), hid_protocol) {
                    (Some(request_handler), hid_protocol) => Some(request_handler.set_protocol(hid_protocol)),
                    (None, HidProtocolMode::Report) => Some(OutResponse::Accepted),
                    (None, HidProtocolMode::Boot) => {
                        info!("Received request to switch to Boot protocol mode, but it is disabled by default.");
                        Some(OutResponse::Rejected)
                    }
                }
            }
            _ => Some(OutResponse::Rejected),
        }
    }

    fn control_in<'a>(&'a mut self, req: Request, buf: &'a mut [u8]) -> Option<InResponse<'a>> {
        if req.index != self.if_num.0 as u16 {
            return None;
        }

        match (req.request_type, req.recipient) {
            (RequestType::Standard, Recipient::Interface) => match req.request {
                Request::GET_DESCRIPTOR => match (req.value >> 8) as u8 {
                    HID_DESC_DESCTYPE_HID_REPORT => Some(InResponse::Accepted(self.report_descriptor)),
                    HID_DESC_DESCTYPE_HID => Some(InResponse::Accepted(&self.hid_descriptor)),
                    _ => Some(InResponse::Rejected),
                },

                _ => Some(InResponse::Rejected),
            },
            (RequestType::Class, Recipient::Interface) => {
                trace!("HID control_in {:?}", req);
                match req.request {
                    HID_REQ_GET_REPORT => {
                        let size = match ReportId::try_from(req.value) {
                            Ok(id) => self.request_handler.as_mut().and_then(|x| x.get_report(id, buf)),
                            Err(_) => None,
                        };

                        if let Some(size) = size {
                            Some(InResponse::Accepted(&buf[0..size]))
                        } else {
                            Some(InResponse::Rejected)
                        }
                    }
                    HID_REQ_GET_IDLE => {
                        if let Some(handler) = self.request_handler.as_mut() {
                            let id = req.value as u8;
                            let id = (id != 0).then_some(ReportId::In(id));
                            if let Some(dur) = handler.get_idle_ms(id) {
                                let dur = u8::try_from(dur / 4).unwrap_or(0);
                                buf[0] = dur;
                                Some(InResponse::Accepted(&buf[0..1]))
                            } else {
                                Some(InResponse::Rejected)
                            }
                        } else {
                            Some(InResponse::Rejected)
                        }
                    }
                    HID_REQ_GET_PROTOCOL => {
                        if let Some(request_handler) = self.request_handler.as_mut() {
                            buf[0] = request_handler.get_protocol() as u8;
                        } else {
                            // Return `Report` protocol mode by default
                            buf[0] = HidProtocolMode::Report as u8;
                        }
                        Some(InResponse::Accepted(&buf[0..1]))
                    }
                    _ => Some(InResponse::Rejected),
                }
            }
            _ => None,
        }
    }
}

/// Boot protocol.
pub const PROTOCOL_BOOT: u8 = 0;
/// Report protocol.
pub const PROTOCOL_REPORT: u8 = 1;

// ── Boot-protocol report structs ─────────────────────────────────────────────

/// Decoded keyboard report (USB HID boot protocol, 8 bytes).
///
/// All standard USB keyboards support this layout when placed in boot protocol
/// mode via [`HidHost::set_protocol`] with [`PROTOCOL_BOOT`].
#[derive(Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct KeyboardReport {
    /// Modifier keys bitmask.
    ///
    /// Bit 0: Left Ctrl  | Bit 1: Left Shift  | Bit 2: Left Alt  | Bit 3: Left GUI
    /// Bit 4: Right Ctrl | Bit 5: Right Shift | Bit 6: Right Alt | Bit 7: Right GUI
    pub modifiers: u8,
    /// Up to 6 simultaneously pressed key codes (HID usage page 0x07).
    /// A value of 0x00 means "no key"; 0x01 means "rollover error".
    pub keycodes: [u8; 6],
}

impl KeyboardReport {
    /// Parse a boot-protocol keyboard report from an 8-byte buffer.
    /// Returns `None` if the buffer is shorter than 8 bytes.
    pub fn parse(buf: &[u8]) -> Option<Self> {
        if buf.len() < 8 {
            return None;
        }
        Some(Self {
            modifiers: buf[0],
            // buf[1] is reserved
            keycodes: [buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]],
        })
    }

    /// Returns `true` if the given HID key code is currently pressed.
    pub fn is_pressed(&self, keycode: u8) -> bool {
        keycode != 0 && self.keycodes.contains(&keycode)
    }

    /// Returns `true` if Left Ctrl or Right Ctrl is held.
    pub fn ctrl(&self) -> bool {
        self.modifiers & 0x11 != 0
    }
    /// Returns `true` if Left Shift or Right Shift is held.
    pub fn shift(&self) -> bool {
        self.modifiers & 0x22 != 0
    }
    /// Returns `true` if Left Alt or Right Alt is held.
    pub fn alt(&self) -> bool {
        self.modifiers & 0x44 != 0
    }
    /// Returns `true` if Left GUI (Win/Cmd) or Right GUI is held.
    pub fn gui(&self) -> bool {
        self.modifiers & 0x88 != 0
    }
}

/// Mouse button bitmask used in [`MouseReport`].
///
/// Bit 0: left button | Bit 1: right button | Bit 2: middle button
pub type MouseButtons = u8;

/// Decoded mouse report (USB HID boot protocol, 4 bytes).
///
/// All standard USB mice support this layout in boot protocol mode.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MouseReport {
    /// Button state. Use the [`MouseButtons`] constants or check bits directly.
    pub buttons: MouseButtons,
    /// Horizontal movement since last report (signed, positive = right).
    pub x: i8,
    /// Vertical movement since last report (signed, positive = down).
    pub y: i8,
    /// Scroll wheel movement (signed, positive = scroll up / away from user).
    pub wheel: i8,
}

impl MouseReport {
    /// Left mouse button.
    pub const BUTTON_LEFT: MouseButtons = 1 << 0;
    /// Right mouse button.
    pub const BUTTON_RIGHT: MouseButtons = 1 << 1;
    /// Middle mouse button (scroll wheel click).
    pub const BUTTON_MIDDLE: MouseButtons = 1 << 2;

    /// Parse a boot-protocol mouse report from a buffer (minimum 3 bytes; 4 for wheel).
    /// Returns `None` if the buffer is shorter than 3 bytes.
    pub fn parse(buf: &[u8]) -> Option<Self> {
        if buf.len() < 3 {
            return None;
        }
        Some(Self {
            buttons: buf[0],
            x: buf[1] as i8,
            y: buf[2] as i8,
            wheel: if buf.len() >= 4 { buf[3] as i8 } else { 0 },
        })
    }

    /// Returns `true` if the left button is pressed.
    pub fn left(&self) -> bool {
        self.buttons & Self::BUTTON_LEFT != 0
    }
    /// Returns `true` if the right button is pressed.
    pub fn right(&self) -> bool {
        self.buttons & Self::BUTTON_RIGHT != 0
    }
    /// Returns `true` if the middle button is pressed.
    pub fn middle(&self) -> bool {
        self.buttons & Self::BUTTON_MIDDLE != 0
    }
}

/// HID class descriptor type (appears inside the configuration descriptor).
const DESC_HID: u8 = 0x21;
const TRANSFER_INTERRUPT: u8 = 0x03;

/// Information about a HID interface found in a configuration descriptor.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct HidInfo {
    /// HID interface number.
    pub interface_number: u8,
    /// Interrupt IN endpoint address (raw, with direction bit).
    pub interrupt_in_ep: u8,
    /// Interrupt IN max packet size.
    pub interrupt_in_mps: u16,
    /// Interrupt IN polling interval (from endpoint descriptor).
    pub interrupt_in_interval: u8,
    /// Length of the HID Report Descriptor in bytes (from the HID class descriptor).
    /// Pass this to [`HidHost::fetch_report_descriptor`] as the buffer size.
    pub report_descriptor_len: u16,
}

/// Find the first HID interface in a configuration descriptor.
pub fn find_hid(config_desc: &[u8]) -> Option<HidInfo> {
    let cfg = ConfigurationDescriptorChain::try_from_slice(config_desc).ok()?;

    for iface in cfg.iter_interface() {
        if iface.interface_class != USB_CLASS_HID {
            continue;
        }

        // Extract report descriptor length from the HID class descriptor (type 0x21).
        // Layout: bLength, bDescriptorType(0x21), bcdHID(2), bCountryCode,
        //         bNumDescriptors, bDescriptorType(0x22), wDescriptorLength(2)
        let report_desc_len = iface
            .iter_descriptors()
            .find_map(|(_, data)| {
                if data.len() >= 9 && data[1] == DESC_HID {
                    Some(u16::from_le_bytes([data[7], data[8]]))
                } else {
                    None
                }
            })
            .unwrap_or(0);

        let ep = iface
            .iter_endpoints()
            .find(|ep| ep.transfer_type() == TRANSFER_INTERRUPT && ep.is_in())?;

        return Some(HidInfo {
            interface_number: iface.interface_number,
            interrupt_in_ep: ep.endpoint_address,
            interrupt_in_mps: ep.max_packet_size,
            interrupt_in_interval: ep.interval,
            report_descriptor_len: report_desc_len,
        });
    }

    None
}

/// HID host class driver error.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum HidError {
    /// Transfer error.
    Transfer(PipeError),
    /// No matching HID interface found in the device.
    NoInterface,
    /// Failed to allocate a pipe.
    NoPipe,
}

impl From<PipeError> for HidError {
    fn from(e: PipeError) -> Self {
        Self::Transfer(e)
    }
}

impl core::fmt::Display for HidError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Transfer(_e) => write!(f, "Transfer error"),
            Self::NoInterface => write!(f, "No HID interface found"),
            Self::NoPipe => write!(f, "No free pipe"),
        }
    }
}

impl core::error::Error for HidError {}

/// HID host driver.
///
/// Provides report reading and optional class request access to a USB HID device.
pub struct HidHost<'d, A: UsbHostAllocator<'d>> {
    ctrl_ch: A::Pipe<pipe::Control, pipe::InOut>,
    in_ch: A::Pipe<pipe::Interrupt, pipe::In>,
    interface: u8,
    report_descriptor_len: u16,
    _phantom: core::marker::PhantomData<&'d ()>,
}

impl<'d, A: UsbHostAllocator<'d>> HidHost<'d, A> {
    /// Create a new HID host driver.
    ///
    /// Parses the config descriptor to find the HID interface and its interrupt IN endpoint,
    /// then allocates the necessary channels.
    pub fn new(alloc: &A, config_desc: &[u8], enum_info: &EnumerationInfo) -> Result<Self, HidError> {
        let info = find_hid(config_desc).ok_or(HidError::NoInterface)?;

        let ctrl_ep_info = EndpointInfo {
            addr: crate::driver::EndpointAddress::from_parts(0, crate::driver::Direction::In),
            ep_type: EndpointType::Control,
            max_packet_size: enum_info.device_desc.max_packet_size0 as u16,
            interval_ms: 0,
        };

        let in_ep_info = EndpointInfo {
            addr: crate::driver::EndpointAddress::from_parts((info.interrupt_in_ep & 0x0F) as usize, crate::driver::Direction::In),
            ep_type: EndpointType::Interrupt,
            max_packet_size: info.interrupt_in_mps,
            interval_ms: info.interrupt_in_interval,
        };

        let device_address = enum_info.device_address;
        let split = enum_info.split();

        let ctrl_ch = alloc
            .alloc_pipe::<pipe::Control, pipe::InOut>(device_address, &ctrl_ep_info, split)
            .map_err(|_| HidError::NoPipe)?;
        let in_ch = alloc
            .alloc_pipe::<pipe::Interrupt, pipe::In>(device_address, &in_ep_info, split)
            .map_err(|_| HidError::NoPipe)?;

        Ok(Self {
            ctrl_ch,
            in_ch,
            interface: info.interface_number,
            report_descriptor_len: info.report_descriptor_len,
            _phantom: core::marker::PhantomData,
        })
    }

    /// Fetch the HID Report Descriptor from the device into `buf`.
    ///
    /// Returns the descriptor bytes as a slice. Pass the result to
    /// [`ReportDescriptor::parse`] to decode it:
    ///
    /// ```ignore
    /// let mut buf = [0u8; 256];
    /// let desc = hid.fetch_report_descriptor(&mut buf).await?;
    /// let report: ReportDescriptor<32> = ReportDescriptor::parse(desc);
    /// ```
    ///
    /// `buf` should be at least `HidInfo::report_descriptor_len` bytes; any
    /// excess is unused.
    pub async fn fetch_report_descriptor<'a>(&mut self, buf: &'a mut [u8]) -> Result<&'a [u8], HidError> {
        let len = (self.report_descriptor_len as usize).min(buf.len()) as u16;
        let setup = SetupPacket::get_hid_report_descriptor(self.interface, len);
        let n = self
            .ctrl_ch
            .control_in(&setup.to_bytes(), &mut buf[..len as usize])
            .await?;
        Ok(&buf[..n])
    }

    /// Set the idle rate for a report.
    ///
    /// `report_id = 0` applies to all reports. `idle_duration = 0` disables idle repeat.
    ///
    /// Note: HID_REQ_SET_IDLE is optional; some devices STALL this request.
    /// A STALL is treated as success per the HID specification.
    pub async fn set_idle(&mut self, report_id: u8, idle_duration: u8) -> Result<(), HidError> {
        let value = (idle_duration as u16) << 8 | report_id as u16;
        let setup = SetupPacket::class_interface_out(HID_REQ_SET_IDLE, value, self.interface as u16, 0);
        match self.ctrl_ch.control_out(&setup.to_bytes(), &[]).await {
            Ok(_) => Ok(()),
            Err(PipeError::Stall) => Ok(()),
            Err(e) => Err(HidError::Transfer(e)),
        }
    }

    /// Set the protocol (boot or report).
    pub async fn set_protocol(&mut self, protocol: u8) -> Result<(), HidError> {
        let setup = SetupPacket::class_interface_out(HID_REQ_SET_PROTOCOL, protocol as u16, self.interface as u16, 0);
        self.ctrl_ch.control_out(&setup.to_bytes(), &[]).await?;
        Ok(())
    }

    /// Read a raw input report from the interrupt IN endpoint.
    ///
    /// Returns the number of bytes received.
    pub async fn read(&mut self, buf: &mut [u8]) -> Result<usize, HidError> {
        let n = self.in_ch.request_in(buf).await?;
        Ok(n)
    }

    /// Read and parse a boot-protocol keyboard report.
    ///
    /// Call [`HidHost::set_protocol`] with [`PROTOCOL_BOOT`] first.
    /// Returns `None` if the report is malformed (shorter than 8 bytes).
    pub async fn read_keyboard(&mut self) -> Result<Option<KeyboardReport>, HidError> {
        let mut buf = [0u8; 8];
        self.in_ch.request_in(&mut buf).await?;
        Ok(KeyboardReport::parse(&buf))
    }

    /// Read and parse a boot-protocol mouse report.
    ///
    /// Call [`HidHost::set_protocol`] with [`PROTOCOL_BOOT`] first.
    /// Returns `None` if the report is malformed (shorter than 3 bytes).
    pub async fn read_mouse(&mut self) -> Result<Option<MouseReport>, HidError> {
        let mut buf = [0u8; 4];
        // Some mice send only 3 bytes; read up to 4.
        let n = self.in_ch.request_in(&mut buf).await?;
        Ok(MouseReport::parse(&buf[..n]))
    }

    /// Issue a HID_REQ_GET_REPORT control request.
    ///
    /// `report_type`: 1=Input, 2=Output, 3=Feature.
    /// `report_id`: 0 if the device uses a single report.
    ///
    /// Returns the number of bytes received.
    pub async fn get_report(&mut self, report_type: u8, report_id: u8, buf: &mut [u8]) -> Result<usize, HidError> {
        let value = (report_type as u16) << 8 | report_id as u16;
        let setup = SetupPacket::class_interface_in(HID_REQ_GET_REPORT, value, self.interface as u16, buf.len() as u16);
        let n = self.ctrl_ch.control_in(&setup.to_bytes(), buf).await?;
        Ok(n)
    }

    /// Issue a HID_REQ_SET_REPORT control request.
    ///
    /// `report_type`: 1=Input, 2=Output, 3=Feature.
    /// `report_id`: 0 if the device uses a single report.
    /// `buf`: the report body
    pub async fn set_report(&mut self, report_type: u8, report_id: u8, buf: &[u8]) -> Result<(), HidError> {
        let value = (report_type as u16) << 8 | report_id as u16;
        let setup = SetupPacket::class_interface_out(HID_REQ_SET_REPORT, value, self.interface as u16, buf.len() as u16);
        self.ctrl_ch.control_out(&setup.to_bytes(), buf).await?;
        Ok(())
    }
}

#[cfg(test)]
mod test {
    use super::*;

    /// A real HID class descriptor, whose report length (0x0034) sits at bytes 7-8.
    const HID_CONFIG: [u8; 34] = [
        9, 2, 34, 0, 1, 1, 0, 0x80, 50, // configuration
        9, 4, 0, 0, 1, 0x03, 0x01, 0x01, 0, // interface, HID boot keyboard
        9, 0x21, 0x11, 0x01, 0, 1, 0x22, 0x34, 0x00, // HID, report descriptor is 52 bytes
        7, 5, 0x81, 0x03, 8, 0, 10, // endpoint, interrupt IN
    ];

    #[test]
    fn reads_report_descriptor_length() {
        let info = find_hid(&HID_CONFIG).expect("HID interface is found");
        assert_eq!(info.report_descriptor_len, 0x0034);
        assert_eq!(info.interrupt_in_ep, 0x81);
    }
}
