//! MIDI class implementation.

mod packet;
mod descriptors;
mod transport;

pub use packet::*;
pub use descriptors::*;
pub use transport::*;

use core::slice::ChunksExact;
use embassy_usb_driver::host::{PipeError, UsbHostAllocator};
use heapless::Vec;

use crate::descriptor::{SynchronizationType, UsageType};
use crate::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut, EndpointType};
use crate::host::EnumerationInfo;
use crate::types::StringIndex;
use crate::{Builder, Handler};

/// This should be used as `device_class` when building the `UsbDevice`.
pub const USB_AUDIO_CLASS: u8 = 0x01;
/// Audio class code.
pub const USB_CLASS_AUDIO: u8 = USB_AUDIO_CLASS;

const USB_AUDIOCONTROL_SUBCLASS: u8 = 0x01;
const USB_MIDISTREAMING_SUBCLASS: u8 = 0x03;
/// MIDIStreaming subclass code.
pub const USB_SUBCLASS_MIDI_STREAMING: u8 = USB_MIDISTREAMING_SUBCLASS;
const MIDI_IN_JACK_SUBTYPE: u8 = 0x02;
const MIDI_OUT_JACK_SUBTYPE: u8 = 0x03;
const EMBEDDED: u8 = 0x01;
const EXTERNAL: u8 = 0x02;
const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;
const HEADER_SUBTYPE: u8 = 0x01;
const MS_HEADER_SUBTYPE: u8 = 0x01;
const MS_GENERAL: u8 = 0x01;
const PROTOCOL_NONE: u8 = 0x00;
/// MIDI 1.0 protocol code.
pub const USB_MIDI_1_PROTOCOL: u8 = PROTOCOL_NONE;
/// USB-MIDI event packet size.
pub const EVENT_PACKET_SIZE: usize = 4;
const MIDI_IN_SIZE: u8 = 0x06;
const MIDI_OUT_SIZE: u8 = 0x09;

/// Configuration for the MIDI class.
///
/// For the jacks, the field names use the terminology used by the USB specification,
/// which defines the direction from the perspective of the host.
///
/// This struct also implements the `Default` trait with the most common setup:
/// 1 input jack, 1 output jack and a maximum packet size of 64.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct MidiClassConfig<'d> {
    /// Number of jacks for sending data to the host via the IN endpoint.
    /// If set to 0, the IN endpoint will not be allocated.
    pub n_in_jacks: u8,

    /// Number of jacks for receiving data from the host via the OUT endpoint.
    /// If set to 0, the OUT endpoint will not be allocated.
    pub n_out_jacks: u8,

    /// Maximum packet size for the endpoints.
    /// For full-speed devices, the value has to be one of 8, 16, 32 or 64.
    pub max_packet_size: u16,

    /// Name of the MIDIStreaming interface.
    pub interface_name: Option<&'d str>,

    /// Name of each IN jack.
    pub in_jack_names: &'d [Option<&'d str>],

    /// Name of each OUT jack.
    pub out_jack_names: &'d [Option<&'d str>],
}

impl Default for MidiClassConfig<'_> {
    fn default() -> Self {
        Self {
            n_in_jacks: 1,
            n_out_jacks: 1,
            max_packet_size: 64,
            interface_name: None,
            in_jack_names: &[],
            out_jack_names: &[],
        }
    }
}

/// Internal state for a [`MidiClass`].
///
/// Holds names/string-indices for interface and jacks.
#[derive(Default)]
pub struct MidiClassState<'d> {
    first: u8,
    interface_name: Option<&'d str>,
    in_jack_names: &'d [Option<&'d str>],
    out_jack_names: &'d [Option<&'d str>],
}

impl<'d> MidiClassState<'d> {
    /// Creates a new `State`.
    pub const fn new() -> Self {
        MidiClassState {
            first: 0,
            interface_name: None,
            in_jack_names: &[],
            out_jack_names: &[],
        }
    }

    /// The names, in allocation order: interface, IN jacks, OUT jacks.
    fn names(&self) -> impl Iterator<Item = &Option<&'d str>> {
        core::iter::once(&self.interface_name)
            .chain(self.in_jack_names)
            .chain(self.out_jack_names)
    }

    /// The name in slot `n`, if there is one.
    fn name(&self, n: usize) -> Option<&'d str> {
        self.names().nth(n).copied().flatten()
    }

    // returns absolute StringIndex, or None if name is absent (or out of range).
    fn index(&self, n: usize) -> Option<StringIndex> {
        self.name(n).map(|_| StringIndex(self.first + n as u8))
    }

    fn id(&self, n: usize) -> u8 {
        self.index(n).map_or(0, |i| i.0)
    }

    fn interface(&self) -> Option<StringIndex> {
        self.index(0)
    }

    fn in_jack(&self, i: u8) -> u8 {
        self.id(1 + i as usize)
    }

    fn out_jack(&self, i: u8) -> u8 {
        self.id(1 + self.in_jack_names.len() + i as usize)
    }
}

impl Handler for MidiClassState<'_> {
    fn get_string(&mut self, index: StringIndex, _lang_id: u16) -> Option<&str> {
        self.name(index.0.checked_sub(self.first)? as usize)
    }
}

/// Packet level implementation of a USB MIDI device.
///
/// This class can be used directly and it has the least overhead due to directly reading and
/// writing USB packets with no intermediate buffers, but it will not act like a stream-like port.
/// The following constraints must be followed if you use this class directly:
///
/// - `read_packet` must be called with a buffer large enough to hold `max_packet_size` bytes.
/// - `write_packet` must not be called with a buffer larger than `max_packet_size` bytes.
/// - If you write a packet that is exactly `max_packet_size` bytes long, it won't be processed by the
///   host operating system until a subsequent shorter packet is sent. A zero-length packet (ZLP)
///   can be sent if there is no other data to send. This is because USB bulk transactions must be
///   terminated with a short packet, even if the bulk endpoint is used for stream-like data.
pub struct MidiClass<'d, D: Driver<'d>> {
    read_ep: Option<D::EndpointOut>,
    write_ep: Option<D::EndpointIn>,
}

impl<'d, D: Driver<'d>> MidiClass<'d, D> {
    /// Creates a new `MidiClass` with the provided UsbBuilder and configuration.
    ///
    /// The names in `config` are ignored, use [`MidiClass::new_with_names`] if you need to name jacks.
    pub fn new(builder: &mut Builder<'d, D>, config: MidiClassConfig<'d>) -> Self {
        Self::build(builder, config, &MidiClassState::new())
    }

    /// Creates a new `MidiClass` with the provided UsbBuilder that names its interface and jacks.
    pub fn new_with_names(
        builder: &mut Builder<'d, D>,
        state: &'d mut MidiClassState<'d>,
        config: MidiClassConfig<'d>,
    ) -> Self {
        *state = MidiClassState {
            first: builder.string().0,
            interface_name: config.interface_name,
            in_jack_names: config.in_jack_names,
            out_jack_names: config.out_jack_names,
        };

        // string index for interface allocated above, now allocate the jacks.
        for _ in 1..state.names().count() {
            builder.string();
        }

        let class = Self::build(builder, config, state);
        builder.handler(state);
        class
    }

    /// Creates a new `MidiClass` with the provided UsbBuilder and configuration.
    fn build(builder: &mut Builder<'d, D>, config: MidiClassConfig<'d>, names: &MidiClassState<'d>) -> Self {
        let MidiClassConfig {
            n_in_jacks,
            n_out_jacks,
            max_packet_size,
            ..
        } = config;

        // Some sanity checks.
        assert!(
            n_in_jacks != 0 || n_out_jacks != 0,
            "n_in_jacks and n_out_jacks are both 0"
        );
        assert!(
            (n_in_jacks as usize) <= MAX_MIDI_JACKS,
            "n_in_jacks is larger than {}",
            MAX_MIDI_JACKS,
        );
        assert!(
            (n_out_jacks as usize) <= MAX_MIDI_JACKS,
            "n_out_jacks is larger than {}",
            MAX_MIDI_JACKS,
        );

        let mut func = builder.function(USB_AUDIO_CLASS, USB_AUDIOCONTROL_SUBCLASS, PROTOCOL_NONE);

        // Audio control interface
        let mut iface = func.interface();
        let audio_if = iface.interface_number();
        let midi_if = u8::from(audio_if) + 1;
        let mut alt = iface.alt_setting(USB_AUDIO_CLASS, USB_AUDIOCONTROL_SUBCLASS, PROTOCOL_NONE, None);
        alt.descriptor(CS_INTERFACE, &[HEADER_SUBTYPE, 0x00, 0x01, 0x09, 0x00, 0x01, midi_if]);

        // MIDIStreaming interface
        let mut iface = func.interface();
        let mut alt = iface.alt_setting(
            USB_AUDIO_CLASS,
            USB_MIDISTREAMING_SUBCLASS,
            PROTOCOL_NONE,
            names.interface(),
        );

        let midi_streaming_total_length = 7
            + (n_in_jacks + n_out_jacks) as usize * (MIDI_IN_SIZE + MIDI_OUT_SIZE) as usize
            + if n_out_jacks > 0 {
                9 + (4 + n_out_jacks as usize)
            } else {
                0
            }
            + if n_in_jacks > 0 {
                9 + (4 + n_in_jacks as usize)
            } else {
                0
            };
        let (read_ep, write_ep) = alt.descriptors_then_patch(
            |alt| {
                alt.descriptor(
                    CS_INTERFACE,
                    &[
                        MS_HEADER_SUBTYPE,
                        0x00,
                        0x01,
                        (midi_streaming_total_length & 0xFF) as u8,
                        ((midi_streaming_total_length >> 8) & 0xFF) as u8,
                    ],
                );

                // Calculates the index'th embedded midi out jack id
                let out_jack_id_emb = |index| 2 * index + 1;
                // Calculates the index'th external midi in jack id
                let in_jack_id_ext = |index| 2 * index + 2;
                // Calculates the index'th embedded midi in jack id
                let in_jack_id_emb = |index| 2 * n_in_jacks + 2 * index + 1;
                // Calculates the index'th external midi out jack id
                let out_jack_id_ext = |index| 2 * n_in_jacks + 2 * index + 2;

                for i in 0..n_in_jacks {
                    let i_jack = names.in_jack(i);
                    alt.descriptor(
                        CS_INTERFACE,
                        &[
                            MIDI_OUT_JACK_SUBTYPE,
                            EMBEDDED,
                            out_jack_id_emb(i),
                            0x01,
                            in_jack_id_ext(i),
                            0x01,
                            i_jack,
                        ],
                    );
                    alt.descriptor(
                        CS_INTERFACE,
                        &[MIDI_IN_JACK_SUBTYPE, EXTERNAL, in_jack_id_ext(i), i_jack],
                    );
                }

                for i in 0..n_out_jacks {
                    let i_jack = names.out_jack(i);
                    alt.descriptor(
                        CS_INTERFACE,
                        &[MIDI_IN_JACK_SUBTYPE, EMBEDDED, in_jack_id_emb(i), i_jack],
                    );
                    alt.descriptor(
                        CS_INTERFACE,
                        &[
                            MIDI_OUT_JACK_SUBTYPE,
                            EXTERNAL,
                            out_jack_id_ext(i),
                            0x01,
                            in_jack_id_emb(i),
                            0x01,
                            i_jack,
                        ],
                    );
                }

                let mut endpoint_data = [
                    MS_GENERAL, 0, // Number of jacks
                    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // Jack mappings
                ];

                let read_ep = if n_out_jacks > 0 {
                    endpoint_data[1] = n_out_jacks;
                    for i in 0..n_out_jacks {
                        endpoint_data[2 + i as usize] = in_jack_id_emb(i);
                    }
                    let read_ep = alt.endpoint_out(
                        EndpointType::Bulk,
                        None,
                        max_packet_size,
                        0,
                        SynchronizationType::NoSynchronization,
                        UsageType::DataEndpoint,
                        &[0, 0],
                    );
                    alt.descriptor(CS_ENDPOINT, &endpoint_data[0..2 + n_out_jacks as usize]);
                    Some(read_ep)
                } else {
                    None
                };

                let write_ep = if n_in_jacks > 0 {
                    endpoint_data[1] = n_in_jacks;
                    for i in 0..n_in_jacks {
                        endpoint_data[2 + i as usize] = out_jack_id_emb(i);
                    }
                    let write_ep = alt.endpoint_in(
                        EndpointType::Bulk,
                        None,
                        max_packet_size,
                        0,
                        SynchronizationType::NoSynchronization,
                        UsageType::DataEndpoint,
                        &[0, 0],
                    );
                    alt.descriptor(CS_ENDPOINT, &endpoint_data[0..2 + n_in_jacks as usize]);
                    Some(write_ep)
                } else {
                    None
                };

                (read_ep, write_ep)
            },
            |buffer| {
                let len = buffer.len() as u16;
                buffer[5..7].copy_from_slice(&len.to_le_bytes());
            },
        );

        MidiClass { read_ep, write_ep }
    }

    /// Gets the maximum packet size in bytes.
    pub fn max_packet_size(&self) -> u16 {
        // The size is the same for both endpoints.
        if let Some(read_ep) = &self.read_ep {
            read_ep.info().max_packet_size
        } else if let Some(write_ep) = &self.write_ep {
            write_ep.info().max_packet_size
        } else {
            0
        }
    }

    /// Writes a single packet into the IN endpoint.
    pub async fn write_packet(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        let write_ep = self.write_ep.as_mut().ok_or(EndpointError::Disabled)?;
        write_ep.write(data).await
    }

    /// Reads a single packet from the OUT endpoint.
    pub async fn read_packet(&mut self, data: &mut [u8]) -> Result<usize, EndpointError> {
        let read_ep = self.read_ep.as_mut().ok_or(EndpointError::Disabled)?;
        read_ep.read(data).await
    }

    /// Waits for the USB host to enable this interface
    pub async fn wait_connection(&mut self) {
        if let Some(read_ep) = &mut self.read_ep {
            read_ep.wait_enabled().await;
        }
    }

    /// Split the class into a sender and receiver.
    ///
    /// This allows concurrently sending and receiving packets from separate tasks.
    pub fn split(mut self) -> (Option<Sender<'d, D>>, Option<Receiver<'d, D>>) {
        let sender = self.write_ep.take().map(|write_ep| Sender { write_ep });
        let receiver = self.read_ep.take().map(|read_ep| Receiver { read_ep });
        (sender, receiver)
    }
}

/// Midi class packet sender.
///
/// You can obtain a `Sender` with [`MidiClass::split`]
pub struct Sender<'d, D: Driver<'d>> {
    write_ep: D::EndpointIn,
}

impl<'d, D: Driver<'d>> Sender<'d, D> {
    /// Gets the maximum packet size in bytes.
    pub fn max_packet_size(&self) -> u16 {
        // The size is the same for both endpoints.
        self.write_ep.info().max_packet_size
    }

    /// Writes a single packet.
    pub async fn write_packet(&mut self, data: &[u8]) -> Result<(), EndpointError> {
        self.write_ep.write(data).await
    }

    /// Waits for the USB host to enable this interface
    pub async fn wait_connection(&mut self) {
        self.write_ep.wait_enabled().await;
    }
}

/// Midi class packet receiver.
///
/// You can obtain a `Receiver` with [`MidiClass::split`]
pub struct Receiver<'d, D: Driver<'d>> {
    read_ep: D::EndpointOut,
}

impl<'d, D: Driver<'d>> Receiver<'d, D> {
    /// Gets the maximum packet size in bytes.
    pub fn max_packet_size(&self) -> u16 {
        // The size is the same for both endpoints.
        self.read_ep.info().max_packet_size
    }

    /// Reads a single packet.
    pub async fn read_packet(&mut self, data: &mut [u8]) -> Result<usize, EndpointError> {
        self.read_ep.read(data).await
    }

    /// Waits for the USB host to enable this interface
    pub async fn wait_connection(&mut self) {
        self.read_ep.wait_enabled().await;
    }
}

/// Advanced descriptor and endpoint-level USB-MIDI APIs.
pub mod raw {
    pub use super::descriptors::*;
    pub use super::transport::{MidiInputPipe, MidiOutputPipe};
    pub use super::{event_packets, UsbMidiEventPacket};
}

/// One USB-MIDI 1.0 event packet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct UsbMidiEventPacket([u8; EVENT_PACKET_SIZE]);

/// Error constructing a USB-MIDI 1.0 event packet from MIDI bytes.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MidiHostPacketError {
    /// USB-MIDI 1.0 cable numbers are limited to 0 through 15.
    InvalidCable,
    /// The status byte is unsupported or reserved.
    InvalidStatus,
    /// The byte count does not match the MIDI status.
    InvalidLength,
    /// A MIDI data byte has its status bit set.
    InvalidData,
}

impl UsbMidiEventPacket {
    /// Create an event packet from its wire representation.
    pub const fn new(bytes: [u8; EVENT_PACKET_SIZE]) -> Self {
        Self(bytes)
    }

    /// Encode one complete non-SysEx MIDI 1.0 message for a virtual cable.
    pub fn from_midi_bytes(cable: u8, data: &[u8]) -> Result<Self, MidiHostPacketError> {
        if cable > 0x0f {
            return Err(MidiHostPacketError::InvalidCable);
        }
        let status = *data.first().ok_or(MidiHostPacketError::InvalidLength)?;
        let (cin, expected_len) = match status {
            0x80..=0xef => {
                let cin = status >> 4;
                let len = if matches!(cin, 0x0c | 0x0d) { 2 } else { 3 };
                (cin, len)
            }
            0xf1 | 0xf3 => (0x02, 2),
            0xf2 => (0x03, 3),
            0xf6 | 0xf7 => (0x05, 1),
            0xf8 | 0xfa..=0xfc | 0xfe | 0xff => (0x0f, 1),
            _ => return Err(MidiHostPacketError::InvalidStatus),
        };
        if data.len() != expected_len {
            return Err(MidiHostPacketError::InvalidLength);
        }
        if data[1..].iter().any(|byte| byte & 0x80 != 0) {
            return Err(MidiHostPacketError::InvalidData);
        }

        let mut packet = [0; EVENT_PACKET_SIZE];
        packet[0] = cable << 4 | cin;
        packet[1..1 + expected_len].copy_from_slice(data);
        Ok(Self(packet))
    }

    /// Virtual cable number carried by this packet.
    pub const fn cable(&self) -> u8 {
        self.0[0] >> 4
    }

    /// Code Index Number describing the MIDI message carried by this packet.
    pub const fn cin(&self) -> u8 {
        self.0[0] & 0x0f
    }

    /// Number of valid MIDI bytes, or `None` for a reserved CIN.
    pub const fn message_len(&self) -> Option<usize> {
        match self.cin() {
            0x2 | 0x6 | 0xc | 0xd => Some(2),
            0x3 | 0x4 | 0x7 | 0x8..=0xb | 0xe => Some(3),
            0x5 | 0xf => Some(1),
            _ => None,
        }
    }

    /// Valid MIDI bytes, or `None` for a reserved CIN.
    pub fn data(&self) -> Option<&[u8]> {
        self.message_len().map(|len| &self.0[1..1 + len])
    }

    /// Four-byte USB-MIDI wire representation.
    pub const fn as_bytes(&self) -> &[u8; EVENT_PACKET_SIZE] {
        &self.0
    }
}

impl From<[u8; EVENT_PACKET_SIZE]> for UsbMidiEventPacket {
    fn from(bytes: [u8; EVENT_PACKET_SIZE]) -> Self {
        Self::new(bytes)
    }
}

impl From<MidiPacket> for UsbMidiEventPacket {
    fn from(packet: MidiPacket) -> Self {
        Self::new(packet.to_bytes())
    }
}

impl From<UsbMidiEventPacket> for MidiPacket {
    fn from(packet: UsbMidiEventPacket) -> Self {
        Self::new(*packet.as_bytes())
    }
}

/// Iterate over complete USB-MIDI event packets in a transfer.
pub fn event_packets(data: &[u8]) -> Result<impl Iterator<Item = UsbMidiEventPacket> + '_, MidiError> {
    if data.is_empty() || !data.len().is_multiple_of(EVENT_PACKET_SIZE) {
        return Err(MidiError::InvalidPacketLength);
    }

    Ok(data
        .chunks_exact(EVENT_PACKET_SIZE)
        .map(|chunk| UsbMidiEventPacket::new([chunk[0], chunk[1], chunk[2], chunk[3]])))
}

/// USB MIDI host error.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum MidiError {
    /// A USB transfer failed.
    Transfer(PipeError),
    /// No MIDIStreaming interface was found.
    NoInterface,
    /// The requested transfer direction is not exposed by the interface.
    NoEndpoint,
    /// An endpoint was opened using the wrong transfer direction.
    WrongDirection,
    /// The controller could not allocate an endpoint pipe.
    NoPipe,
    /// The MIDI descriptors are malformed or exceed a fixed capacity.
    Descriptor(MidiDescriptorError),
    /// The device uses a topology unsupported by the friendly host API.
    UnsupportedTopology,
    /// A port does not belong to this MIDI device direction.
    InvalidPort,
    /// A MIDI message could not be encoded as an event packet.
    InvalidMessage(MidiHostPacketError),
    /// USB-MIDI transfers must contain complete four-byte event packets.
    InvalidPacketLength,
}

/// Alias for [`MidiError`].
pub type MidiHostError = MidiError;

impl From<PipeError> for MidiError {
    fn from(error: PipeError) -> Self {
        Self::Transfer(error)
    }
}

impl From<MidiDescriptorError> for MidiError {
    fn from(error: MidiDescriptorError) -> Self {
        Self::Descriptor(error)
    }
}

impl From<MidiHostPacketError> for MidiError {
    fn from(error: MidiHostPacketError) -> Self {
        Self::InvalidMessage(error)
    }
}

impl core::fmt::Display for MidiError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Transfer(_) => write!(f, "USB MIDI transfer failed"),
            Self::NoInterface => write!(f, "no USB MIDIStreaming interface found"),
            Self::NoEndpoint => write!(f, "USB MIDI endpoint direction unavailable"),
            Self::WrongDirection => write!(f, "USB MIDI endpoint has the wrong direction"),
            Self::NoPipe => write!(f, "no free USB host pipe"),
            Self::Descriptor(_) => write!(f, "invalid USB MIDI descriptors"),
            Self::UnsupportedTopology => write!(f, "unsupported USB MIDI topology"),
            Self::InvalidPort => write!(f, "USB MIDI port does not belong to this device direction"),
            Self::InvalidMessage(_) => write!(f, "invalid MIDI message"),
            Self::InvalidPacketLength => write!(f, "USB MIDI data is not a multiple of four bytes"),
        }
    }
}

impl core::error::Error for MidiError {}

/// A logical device-to-host MIDI port discovered from a USB cable association.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MidiInputPort {
    device_address: u8,
    interface_number: u8,
    endpoint_address: u8,
    cable: u8,
    jack_id: u8,
}

impl MidiInputPort {
    /// One-based port number suitable for display.
    pub const fn number(&self) -> u8 {
        self.cable + 1
    }

    /// Zero-based USB-MIDI virtual cable number.
    pub const fn cable(&self) -> u8 {
        self.cable
    }

    /// Associated embedded jack identifier.
    pub const fn jack_id(&self) -> u8 {
        self.jack_id
    }

    /// MIDIStreaming interface number.
    pub const fn interface_number(&self) -> u8 {
        self.interface_number
    }
}

/// A logical host-to-device MIDI port discovered from a USB cable association.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MidiOutputPort {
    device_address: u8,
    interface_number: u8,
    endpoint_address: u8,
    cable: u8,
    jack_id: u8,
}

impl MidiOutputPort {
    /// One-based port number suitable for display.
    pub const fn number(&self) -> u8 {
        self.cable + 1
    }

    /// Zero-based USB-MIDI virtual cable number.
    pub const fn cable(&self) -> u8 {
        self.cable
    }

    /// Associated embedded jack identifier.
    pub const fn jack_id(&self) -> u8 {
        self.jack_id
    }

    /// MIDIStreaming interface number.
    pub const fn interface_number(&self) -> u8 {
        self.interface_number
    }
}

/// Friendly host driver for one USB MIDI 1.0 streaming interface.
///
/// Input-only and output-only devices are represented by an empty port slice
/// on the unsupported direction.
///
/// This convenience API supports one alternate-setting-zero MIDIStreaming
/// interface with at most one bulk endpoint per direction. Use [`raw`] for
/// devices with multiple streaming interfaces or same-direction endpoints.
pub struct MidiHost<'d, A: UsbHostAllocator<'d>> {
    sender: MidiHostSender<'d, A>,
    receiver: MidiHostReceiver<'d, A>,
}

impl<'d, A: UsbHostAllocator<'d>> MidiHost<'d, A> {
    /// Discover the MIDI ports and allocate the available bulk pipes.
    pub fn new(alloc: &A, config_desc: &[u8], enum_info: &EnumerationInfo) -> Result<Self, MidiError> {
        let interfaces = parse_midi_interfaces_for_device(&enum_info.device_desc, config_desc)?;
        let (interface, input_endpoint, output_endpoint) = select_streaming_interface(&interfaces)?;

        let receiver = MidiHostReceiver::open(alloc, interface, input_endpoint, enum_info)?;
        let sender = MidiHostSender::open(alloc, interface, output_endpoint, enum_info)?;
        Ok(Self { sender, receiver })
    }

    /// Logical device-to-host MIDI ports.
    pub fn input_ports(&self) -> &[MidiInputPort] {
        self.receiver.ports()
    }

    /// Logical host-to-device MIDI ports.
    pub fn output_ports(&self) -> &[MidiOutputPort] {
        self.sender.ports()
    }

    /// Receive one bulk transfer containing USB-MIDI event packets.
    pub async fn read_transfer(&mut self, buf: &mut [u8]) -> Result<usize, MidiError> {
        self.receiver.read_transfer(buf).await
    }

    /// Send one or more complete USB-MIDI event packets.
    pub async fn write_packet(&mut self, packets: &[u8]) -> Result<(), MidiError> {
        self.sender.write_packet(packets).await
    }

    /// Encode and send one complete non-SysEx MIDI message to a logical port.
    pub async fn send(&mut self, port: MidiOutputPort, message: &[u8]) -> Result<(), MidiError> {
        self.sender.send(port, message).await
    }

    /// Split the class into independently owned sender and receiver halves.
    ///
    /// This allows sending and receiving from separate tasks.
    pub fn split(self) -> (MidiHostSender<'d, A>, MidiHostReceiver<'d, A>) {
        (self.sender, self.receiver)
    }
}

fn select_streaming_interface(
    interfaces: &[MidiStreamingInterface],
) -> Result<
    (
        &MidiStreamingInterface,
        Option<&MidiEndpointDescriptor>,
        Option<&MidiEndpointDescriptor>,
    ),
    MidiError,
> {
    let mut matching = interfaces
        .iter()
        .filter(|interface| interface.alternate_setting == 0 && !interface.endpoints.is_empty());
    let interface = matching.next().ok_or(MidiError::NoInterface)?;
    if matching.next().is_some() {
        return Err(MidiError::UnsupportedTopology);
    }

    let mut input_endpoint = None;
    let mut output_endpoint = None;
    for endpoint in &interface.endpoints {
        let slot = if endpoint.is_in() {
            &mut input_endpoint
        } else {
            &mut output_endpoint
        };
        if slot.replace(endpoint).is_some() {
            return Err(MidiError::UnsupportedTopology);
        }
    }

    Ok((interface, input_endpoint, output_endpoint))
}

/// USB-MIDI host-to-device packet sender.
pub struct MidiHostSender<'d, A: UsbHostAllocator<'d>> {
    pipe: Option<transport::MidiOutputPipe<'d, A>>,
    ports: Vec<MidiOutputPort, MAX_MIDI_JACKS>,
}

/// Alias for [`MidiHostSender`].
pub type HostSender<'d, A> = MidiHostSender<'d, A>;

impl<'d, A: UsbHostAllocator<'d>> MidiHostSender<'d, A> {
    fn open(
        alloc: &A,
        interface: &MidiStreamingInterface,
        endpoint: Option<&MidiEndpointDescriptor>,
        enum_info: &EnumerationInfo,
    ) -> Result<Self, MidiError> {
        let Some(endpoint) = endpoint else {
            return Ok(Self {
                pipe: None,
                ports: Vec::new(),
            });
        };
        let ports = output_ports(interface.interface_number, endpoint, enum_info.device_address)?;
        let pipe = transport::MidiOutputPipe::open(alloc, endpoint, enum_info)?;
        Ok(Self {
            pipe: Some(pipe),
            ports,
        })
    }

    /// Logical ports that can receive MIDI from the host.
    pub fn ports(&self) -> &[MidiOutputPort] {
        &self.ports
    }

    /// Send one or more complete USB-MIDI event packets.
    pub async fn write_packet(&mut self, packets: &[u8]) -> Result<(), MidiError> {
        self.pipe.as_mut().ok_or(MidiError::NoEndpoint)?.write(packets).await
    }

    /// Encode and send one complete non-SysEx MIDI message to a logical port.
    pub async fn send(&mut self, port: MidiOutputPort, message: &[u8]) -> Result<(), MidiError> {
        if !self.ports.contains(&port) {
            return Err(MidiError::InvalidPort);
        }
        let packet = UsbMidiEventPacket::from_midi_bytes(port.cable, message)?;
        self.write_packet(packet.as_bytes()).await
    }
}

/// USB-MIDI device-to-host packet receiver.
pub struct MidiHostReceiver<'d, A: UsbHostAllocator<'d>> {
    pipe: Option<transport::MidiInputPipe<'d, A>>,
    ports: Vec<MidiInputPort, MAX_MIDI_JACKS>,
}

/// Alias for [`MidiHostReceiver`].
pub type HostReceiver<'d, A> = MidiHostReceiver<'d, A>;

impl<'d, A: UsbHostAllocator<'d>> MidiHostReceiver<'d, A> {
    fn open(
        alloc: &A,
        interface: &MidiStreamingInterface,
        endpoint: Option<&MidiEndpointDescriptor>,
        enum_info: &EnumerationInfo,
    ) -> Result<Self, MidiError> {
        let Some(endpoint) = endpoint else {
            return Ok(Self {
                pipe: None,
                ports: Vec::new(),
            });
        };
        let ports = input_ports(interface.interface_number, endpoint, enum_info.device_address)?;
        let pipe = transport::MidiInputPipe::open(alloc, endpoint, enum_info)?;
        Ok(Self {
            pipe: Some(pipe),
            ports,
        })
    }

    /// Logical ports that can send MIDI to the host.
    pub fn ports(&self) -> &[MidiInputPort] {
        &self.ports
    }

    /// Receive one bulk transfer containing complete USB-MIDI event packets.
    pub async fn read_transfer(&mut self, buf: &mut [u8]) -> Result<usize, MidiError> {
        self.pipe.as_mut().ok_or(MidiError::NoEndpoint)?.read(buf).await
    }

    /// Receive one transfer and map each event packet to its logical input port.
    pub async fn receive<'a>(&'a mut self, buffer: &'a mut [u8]) -> Result<ReceivedMidiPackets<'a>, MidiError> {
        let length = self.read_transfer(buffer).await?;
        for chunk in buffer[..length].chunks_exact(EVENT_PACKET_SIZE) {
            let cable = chunk[0] >> 4;
            if !self.ports.iter().any(|port| port.cable == cable) {
                return Err(MidiError::InvalidPort);
            }
        }
        Ok(ReceivedMidiPackets {
            chunks: buffer[..length].chunks_exact(EVENT_PACKET_SIZE),
            ports: &self.ports,
        })
    }
}

/// One received USB-MIDI event packet and its logical input port.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ReceivedMidiPacket {
    port: MidiInputPort,
    packet: UsbMidiEventPacket,
}

impl ReceivedMidiPacket {
    /// Logical input port selected by the packet's virtual cable number.
    pub const fn port(&self) -> MidiInputPort {
        self.port
    }

    /// USB-MIDI event packet.
    pub const fn packet(&self) -> UsbMidiEventPacket {
        self.packet
    }

    /// Valid MIDI bytes, or `None` for a reserved CIN.
    pub fn data(&self) -> Option<&[u8]> {
        self.packet.data()
    }
}

/// Iterator over the logical MIDI packets in one received USB transfer.
pub struct ReceivedMidiPackets<'a> {
    chunks: ChunksExact<'a, u8>,
    ports: &'a [MidiInputPort],
}

impl Iterator for ReceivedMidiPackets<'_> {
    type Item = ReceivedMidiPacket;

    fn next(&mut self) -> Option<Self::Item> {
        loop {
            let chunk = self.chunks.next()?;
            let packet = UsbMidiEventPacket::new([chunk[0], chunk[1], chunk[2], chunk[3]]);
            if packet.data().is_none() {
                continue;
            }
            let port = self.ports.iter().find(|port| port.cable == packet.cable()).copied()?;
            return Some(ReceivedMidiPacket { port, packet });
        }
    }
}

fn input_ports(
    interface_number: u8,
    endpoint: &MidiEndpointDescriptor,
    device_address: u8,
) -> Result<Vec<MidiInputPort, MAX_MIDI_JACKS>, MidiError> {
    let mut ports = Vec::new();
    for (cable, &jack_id) in cable_jacks(endpoint).enumerate() {
        ports
            .push(MidiInputPort {
                device_address,
                interface_number,
                endpoint_address: endpoint.address,
                cable: cable as u8,
                jack_id,
            })
            .map_err(|_| MidiDescriptorError::Capacity)?;
    }
    Ok(ports)
}

fn output_ports(
    interface_number: u8,
    endpoint: &MidiEndpointDescriptor,
    device_address: u8,
) -> Result<Vec<MidiOutputPort, MAX_MIDI_JACKS>, MidiError> {
    let mut ports = Vec::new();
    for (cable, &jack_id) in cable_jacks(endpoint).enumerate() {
        ports
            .push(MidiOutputPort {
                device_address,
                interface_number,
                endpoint_address: endpoint.address,
                cable: cable as u8,
                jack_id,
            })
            .map_err(|_| MidiDescriptorError::Capacity)?;
    }
    Ok(ports)
}

fn cable_jacks(endpoint: &MidiEndpointDescriptor) -> impl Iterator<Item = &u8> {
    static IMPLICIT_CABLE: [u8; 1] = [0];
    if endpoint.jack_ids.is_empty() {
        IMPLICIT_CABLE.iter()
    } else {
        endpoint.jack_ids.iter()
    }
}

/// USB-MIDI host driver types and re-exports.
pub mod host {
    pub use super::{
        event_packets, raw, HostReceiver, HostSender, MidiDescriptorError, MidiEndpointDescriptor, MidiError,
        MidiHost, MidiHostPacketError as MidiPacketError, MidiHostReceiver as Receiver, MidiHostSender as Sender,
        MidiInputPort, MidiOutputPort, MidiStreamingHeader, MidiStreamingInterface, ReceivedMidiPacket,
        ReceivedMidiPackets, UsbMidiEventPacket, EVENT_PACKET_SIZE, MAX_MIDI_ENDPOINTS, MAX_MIDI_INTERFACES,
        MAX_MIDI_JACKS, MAX_MIDI_JACK_SOURCES,
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_event_packet_fields_and_lengths() {
        let note_on = UsbMidiEventPacket::new([0x39, 0x90, 60, 100]);
        assert_eq!(note_on.cable(), 3);
        assert_eq!(note_on.cin(), 9);
        assert_eq!(note_on.data(), Some(&[0x90, 60, 100][..]));

        let program_change = UsbMidiEventPacket::new([0x0c, 0xc0, 12, 0]);
        assert_eq!(program_change.data(), Some(&[0xc0, 12][..]));

        let clock = UsbMidiEventPacket::new([0x0f, 0xf8, 0, 0]);
        assert_eq!(clock.data(), Some(&[0xf8][..]));

        let reserved = UsbMidiEventPacket::new([0x00, 0, 0, 0]);
        assert_eq!(reserved.data(), None);
    }

    #[test]
    fn iterates_complete_event_packets() {
        let data = [0x09, 0x90, 60, 100, 0x08, 0x80, 60, 0];
        let packets: heapless::Vec<_, 2> = event_packets(&data).unwrap().collect();
        assert_eq!(packets.len(), 2);
        assert_eq!(packets[0].data(), Some(&[0x90, 60, 100][..]));
        assert_eq!(packets[1].data(), Some(&[0x80, 60, 0][..]));
        assert!(event_packets(&data[..7]).is_err());
        assert!(event_packets(&[]).is_err());
    }

    #[test]
    fn encodes_channel_voice_event_packets() {
        let messages: &[(&[u8], u8)] = &[
            (&[0x80, 60, 0], 0x08),
            (&[0x90, 60, 100], 0x09),
            (&[0xa0, 60, 50], 0x0a),
            (&[0xb0, 74, 90], 0x0b),
            (&[0xc0, 12], 0x0c),
            (&[0xd0, 77], 0x0d),
            (&[0xe0, 0, 64], 0x0e),
        ];
        for &(message, cin) in messages {
            let packet = UsbMidiEventPacket::from_midi_bytes(3, message).unwrap();
            assert_eq!(packet.cable(), 3);
            assert_eq!(packet.cin(), cin);
            assert_eq!(packet.data(), Some(message));
        }
    }

    #[test]
    fn encodes_system_common_and_realtime_event_packets() {
        let messages: &[(&[u8], u8)] = &[
            (&[0xf1, 1], 0x02),
            (&[0xf2, 1, 2], 0x03),
            (&[0xf3, 3], 0x02),
            (&[0xf6], 0x05),
            (&[0xf7], 0x05),
            (&[0xf8], 0x0f),
            (&[0xfa], 0x0f),
            (&[0xfb], 0x0f),
            (&[0xfc], 0x0f),
            (&[0xfe], 0x0f),
            (&[0xff], 0x0f),
        ];
        for &(message, cin) in messages {
            let packet = UsbMidiEventPacket::from_midi_bytes(15, message).unwrap();
            assert_eq!(packet.cin(), cin);
            assert_eq!(packet.data(), Some(message));
            for &padding in &packet.as_bytes()[1 + message.len()..] {
                assert_eq!(padding, 0);
            }
        }
    }

    #[test]
    fn rejects_invalid_midi_event_packet_inputs() {
        assert_eq!(
            UsbMidiEventPacket::from_midi_bytes(16, &[0x90, 60, 100]),
            Err(MidiHostPacketError::InvalidCable)
        );
        for status in [0x00, 0x7f, 0xf0, 0xf4, 0xf5, 0xf9, 0xfd] {
            assert_eq!(
                UsbMidiEventPacket::from_midi_bytes(0, &[status]),
                Err(MidiHostPacketError::InvalidStatus)
            );
        }
        for message in [&[][..], &[0x90, 60][..], &[0xc0, 12, 0][..], &[0xf8, 0][..]] {
            assert_eq!(
                UsbMidiEventPacket::from_midi_bytes(0, message),
                Err(MidiHostPacketError::InvalidLength)
            );
        }
        assert_eq!(
            UsbMidiEventPacket::from_midi_bytes(0, &[0x90, 0x80, 0]),
            Err(MidiHostPacketError::InvalidData)
        );
    }

    #[test]
    fn maps_endpoint_cables_to_typed_logical_ports() {
        let endpoint = MidiEndpointDescriptor {
            address: 0x81,
            max_packet_size: 64,
            jack_ids: Vec::from_slice(&[3, 7]).unwrap(),
        };
        let ports = input_ports(2, &endpoint, 5).unwrap();

        assert_eq!(ports.len(), 2);
        assert_eq!(ports[0].number(), 1);
        assert_eq!(ports[0].cable(), 0);
        assert_eq!(ports[0].jack_id(), 3);
        assert_eq!(ports[1].number(), 2);
        assert_eq!(ports[1].cable(), 1);
        assert_eq!(ports[1].jack_id(), 7);
        assert_eq!(ports[1].interface_number(), 2);
    }

    #[test]
    fn maps_received_cables_to_input_ports() {
        let endpoint = MidiEndpointDescriptor {
            address: 0x81,
            max_packet_size: 64,
            jack_ids: Vec::from_slice(&[3, 7]).unwrap(),
        };
        let ports = input_ports(2, &endpoint, 5).unwrap();
        let transfer = [0, 0, 0, 0, 0x19, 0x90, 60, 100, 0, 0, 0, 0];
        let mut packets = ReceivedMidiPackets {
            chunks: transfer.chunks_exact(EVENT_PACKET_SIZE),
            ports: &ports,
        };

        let received = packets.next().unwrap();
        assert_eq!(received.port(), ports[1]);
        assert_eq!(received.data(), Some(&[0x90, 60, 100][..]));
        assert!(packets.next().is_none());
    }

    #[test]
    fn supplies_one_implicit_port_without_jack_associations() {
        let endpoint = MidiEndpointDescriptor {
            address: 0x02,
            max_packet_size: 64,
            jack_ids: Vec::new(),
        };
        let ports = output_ports(1, &endpoint, 4).unwrap();

        assert_eq!(ports.len(), 1);
        assert_eq!(ports[0].cable(), 0);
        assert_eq!(ports[0].jack_id(), 0);
    }

    fn interface(alternate_setting: u8, endpoints: &[MidiEndpointDescriptor]) -> MidiStreamingInterface {
        MidiStreamingInterface {
            interface_number: 1,
            alternate_setting,
            header: None,
            in_jacks: Vec::new(),
            out_jacks: Vec::new(),
            endpoints: Vec::from_slice(endpoints).unwrap(),
        }
    }

    fn endpoint(address: u8) -> MidiEndpointDescriptor {
        MidiEndpointDescriptor {
            address,
            max_packet_size: 64,
            jack_ids: Vec::new(),
        }
    }

    #[test]
    fn selects_input_output_and_duplex_topologies() {
        let input = endpoint(0x81);
        let output = endpoint(0x02);

        let interfaces = [interface(0, core::slice::from_ref(&input))];
        let (_, selected_input, selected_output) = select_streaming_interface(&interfaces).unwrap();
        assert_eq!(selected_input.unwrap().address, input.address);
        assert!(selected_output.is_none());

        let interfaces = [interface(0, core::slice::from_ref(&output))];
        let (_, selected_input, selected_output) = select_streaming_interface(&interfaces).unwrap();
        assert!(selected_input.is_none());
        assert_eq!(selected_output.unwrap().address, output.address);

        let interfaces = [interface(0, &[input.clone(), output.clone()])];
        let (_, selected_input, selected_output) = select_streaming_interface(&interfaces).unwrap();
        assert_eq!(selected_input.unwrap().address, input.address);
        assert_eq!(selected_output.unwrap().address, output.address);
    }

    #[test]
    fn rejects_missing_and_ambiguous_topologies() {
        assert!(matches!(select_streaming_interface(&[]), Err(MidiError::NoInterface)));

        let input = endpoint(0x81);
        let output = endpoint(0x02);
        let interfaces = [interface(1, core::slice::from_ref(&input))];
        assert!(matches!(
            select_streaming_interface(&interfaces),
            Err(MidiError::NoInterface)
        ));

        let interfaces = [
            interface(0, core::slice::from_ref(&input)),
            interface(0, core::slice::from_ref(&output)),
        ];
        assert!(matches!(
            select_streaming_interface(&interfaces),
            Err(MidiError::UnsupportedTopology)
        ));

        let interfaces = [interface(0, &[input.clone(), input])];
        assert!(matches!(
            select_streaming_interface(&interfaces),
            Err(MidiError::UnsupportedTopology)
        ));
    }
}
