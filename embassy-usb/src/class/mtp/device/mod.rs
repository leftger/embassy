//! USB MTP/PTP device class implementation.

use core::cmp::max;

use super::{EventBlock, IMAGE_PROTOCOL_PIMA_15740, IMAGE_SUBCLASS_STILL_IMAGE, USB_CLASS_IMAGE};
use crate::driver::{Driver, Endpoint};
use crate::types::InterfaceNumber;
use crate::{Builder, EndpointOptions, msos};

pub mod control;
pub mod storage;
pub mod transport;

pub use control::State;
pub use storage::{MtpStorage, ParentScope};
pub use transport::{DataReader, DataWriter, MtpEventSender, MtpTransport, TransportError};

/// Configuration for [`MtpClass`].
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Config<'d> {
    /// Manufacturer string reported in `GetDeviceInfo`.
    pub manufacturer: &'d str,
    /// Model string reported in `GetDeviceInfo`.
    pub model: &'d str,
    /// Firmware/device version string reported in `GetDeviceInfo`.
    pub device_version: &'d str,
    /// Serial number string reported in `GetDeviceInfo`.
    pub serial_number: &'d str,
    /// Optional USB interface string descriptor.
    pub interface_name: Option<&'d str>,
    /// Polling interval in milliseconds for the Interrupt-IN event endpoint.
    pub interrupt_poll_ms: u8,
    /// Emit Microsoft OS 2.0 `"MTP"` compatible ID descriptor for Windows composite devices.
    ///
    /// When enabled, `Builder` must be configured with a non-empty `msos_descriptor_buf`.
    pub msos_descriptor: bool,
    /// Options for the Bulk-OUT endpoint.
    pub out_endpoint_options: EndpointOptions,
    /// Options for the Bulk-IN endpoint.
    pub in_endpoint_options: EndpointOptions,
}

impl<'d> Config<'d> {
    /// Creates a new `Config` with default metadata and endpoint options.
    pub const fn new() -> Self {
        Self {
            manufacturer: "Embassy",
            model: "MTP Device",
            device_version: "1.0",
            serial_number: "00000001",
            interface_name: Some("MTP"),
            interrupt_poll_ms: 16,
            msos_descriptor: false,
            out_endpoint_options: EndpointOptions::new(),
            in_endpoint_options: EndpointOptions::new(),
        }
    }
}

impl<'d> Default for Config<'d> {
    fn default() -> Self {
        Self::new()
    }
}

/// USB Media Transfer Protocol (MTP / PTP) device class.
pub struct MtpClass<'d, D: Driver<'d>> {
    transport: MtpTransport<'d, D>,
    events: MtpEventSender<'d, D>,
    interface: InterfaceNumber,
    config: Config<'d>,
}

impl<'d, D: Driver<'d>> MtpClass<'d, D> {
    /// Creates a new `MtpClass` and registers its Still Image interface and control handler.
    pub fn new(builder: &mut Builder<'d, D>, state: &'d mut State<'d>, config: Config<'d>) -> Self {
        let iface_string = config.interface_name.map(|_| builder.string());

        if config.msos_descriptor && builder.msos_writer().is_empty() {
            builder.msos_descriptor(msos::windows_version::WIN8_1, 0x02);
        }

        let mut func = builder.function(USB_CLASS_IMAGE, IMAGE_SUBCLASS_STILL_IMAGE, IMAGE_PROTOCOL_PIMA_15740);

        if config.msos_descriptor {
            func.msos_feature(msos::CompatibleIdFeatureDescriptor::new("MTP", ""));
        }

        let mut iface = func.interface();
        let interface = iface.interface_number();
        let mut alt = iface.alt_setting(
            USB_CLASS_IMAGE,
            IMAGE_SUBCLASS_STILL_IMAGE,
            IMAGE_PROTOCOL_PIMA_15740,
            iface_string,
        );

        let int_ep = alt.endpoint_interrupt_in(None, 16, config.interrupt_poll_ms);
        let read_ep = alt.endpoint_bulk_out(None, config.out_endpoint_options);
        let write_ep = alt.endpoint_bulk_in(None, config.in_endpoint_options);

        drop(func);

        let control = state.control.write(control::Control {
            interface,
            shared: &state.shared,
        });
        builder.interface_handler(control);

        Self {
            transport: MtpTransport {
                read_ep,
                write_ep,
                shared: &state.shared,
            },
            events: MtpEventSender { int_ep },
            interface,
            config,
        }
    }

    /// Returns the Bulk endpoint `max_packet_size` in bytes.
    pub fn max_packet_size(&self) -> u16 {
        self.transport.max_packet_size()
    }

    /// Returns the assigned USB interface number.
    pub fn interface_number(&self) -> InterfaceNumber {
        self.interface
    }

    /// Waits until the USB host enables this interface.
    pub async fn wait_connection(&mut self) {
        self.transport.wait_connection().await;
    }

    /// Splits this class into the bulk transport and an independent event sender.
    ///
    /// Useful when you want one task running [`MtpStorage`] or low-level command loops
    /// while another task emits asynchronous [`EventBlock`]s.
    pub fn split(self) -> (MtpTransport<'d, D>, MtpEventSender<'d, D>) {
        (self.transport, self.events)
    }

    /// Returns a mutable reference to the underlying [`MtpTransport`].
    pub fn transport_mut(&mut self) -> &mut MtpTransport<'d, D> {
        &mut self.transport
    }

    /// Returns a mutable reference to the underlying [`MtpEventSender`].
    pub fn events_mut(&mut self) -> &mut MtpEventSender<'d, D> {
        &mut self.events
    }

    /// Sends an asynchronous MTP event on the Interrupt-IN endpoint.
    pub async fn send_event(&mut self, event: &EventBlock) -> Result<(), crate::driver::EndpointError> {
        self.events.send_event(event).await
    }

    /// Runs the high-level MTP storage server forever, handling reconnects automatically.
    ///
    /// `buf` is used for endpoint packet framing and streaming object chunks; it must be
    /// at least `max(max_packet_size, 64)` bytes long (512+ bytes recommended for good throughput).
    pub async fn run<S: MtpStorage>(&mut self, storage: &mut S, buf: &mut [u8]) -> ! {
        let min_len = max(self.transport.read_ep.info().max_packet_size as usize, 64);
        assert!(
            buf.len() >= min_len,
            "mtp buffer must be at least max(max_packet_size, 64)"
        );

        let mut runner = storage::MtpRunner::new(self.config);
        loop {
            self.wait_connection().await;
            info!("mtp: connected");

            let _ = runner.run_connected(&mut self.transport, storage, buf).await;

            info!("mtp: disconnected");
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use core::cell::{Cell, RefCell};
    use std::boxed::Box;
    use std::rc::Rc;
    use std::vec;
    use std::vec::Vec;

    use embassy_futures::block_on;
    use embassy_usb_driver::{
        Bus, ControlPipe, Direction, Driver, Endpoint, EndpointAddress, EndpointAllocError, EndpointConfig,
        EndpointError, EndpointIn, EndpointInfo, EndpointOut, EndpointType, Event, Unsupported,
    };

    use super::*;
    use crate::class::mtp::{
        AccessCapability, CANCEL_CODE, CONTAINER_HEADER_LEN, CommandBlock, ContainerHeader, ContainerType,
        DatasetReader, EventBlock, EventCode, FilesystemType, HANDLE_ALL, HANDLE_ROOT, MTP_EXTENSIONS_MICROSOFT,
        MTP_VENDOR_EXTENSION_ID, MTP_VERSION, ObjectEntry, ObjectFormat, ObjectPropCode, OperationCode,
        PTP_STANDARD_VERSION, REQ_CANCEL, ResponseCode, STORAGE_ALL, StorageInfo, StorageType,
    };
    use crate::control::{OutResponse, Recipient, Request, RequestType};
    use crate::{Builder, Handler};

    #[derive(Default)]
    struct MockEpState {
        out_packets: RefCell<Vec<Vec<u8>>>,
        in_packets: RefCell<Vec<Vec<u8>>>,
        int_packets: RefCell<Vec<Vec<u8>>>,
        fail_after_empty: Cell<bool>,
    }

    struct MockEp {
        info: EndpointInfo,
        state: Rc<MockEpState>,
        is_int: bool,
    }

    impl Endpoint for MockEp {
        fn info(&self) -> &EndpointInfo {
            &self.info
        }

        async fn wait_enabled(&mut self) {}

        fn set_stall(&mut self, _: bool) -> Result<(), Unsupported> {
            Ok(())
        }

        fn is_stalled(&self) -> Result<bool, Unsupported> {
            Ok(false)
        }
    }

    impl EndpointOut for MockEp {
        async fn read(&mut self, buf: &mut [u8]) -> Result<usize, EndpointError> {
            let mut q = self.state.out_packets.borrow_mut();
            if q.is_empty() {
                if self.state.fail_after_empty.get() {
                    return Err(EndpointError::Disabled);
                }
                panic!("mock OUT queue empty");
            }
            let pkt = q.remove(0);
            assert!(pkt.len() <= buf.len());
            buf[..pkt.len()].copy_from_slice(&pkt);
            Ok(pkt.len())
        }
    }

    impl EndpointIn for MockEp {
        async fn write(&mut self, buf: &[u8]) -> Result<(), EndpointError> {
            if self.is_int {
                self.state.int_packets.borrow_mut().push(buf.to_vec());
            } else {
                self.state.in_packets.borrow_mut().push(buf.to_vec());
            }
            Ok(())
        }
    }

    struct Unused;
    impl Endpoint for Unused {
        fn info(&self) -> &EndpointInfo {
            unreachable!()
        }
        async fn wait_enabled(&mut self) {
            unreachable!()
        }
        fn set_stall(&mut self, _: bool) -> Result<(), Unsupported> {
            unreachable!()
        }
        fn is_stalled(&self) -> Result<bool, Unsupported> {
            unreachable!()
        }
    }
    impl EndpointOut for Unused {
        async fn read(&mut self, _: &mut [u8]) -> Result<usize, EndpointError> {
            unreachable!()
        }
    }
    impl EndpointIn for Unused {
        async fn write(&mut self, _: &[u8]) -> Result<(), EndpointError> {
            unreachable!()
        }
    }
    impl Bus for Unused {
        async fn enable(&mut self) {
            unreachable!()
        }
        async fn disable(&mut self) {
            unreachable!()
        }
        async fn poll(&mut self) -> Event {
            unreachable!()
        }
        fn endpoint_set_enabled(&mut self, _: EndpointAddress, _: bool) {
            unreachable!()
        }
        fn endpoint_set_stalled(&mut self, _: EndpointAddress, _: bool) {
            unreachable!()
        }
        fn endpoint_is_stalled(&mut self, _: EndpointAddress) -> bool {
            unreachable!()
        }
        async fn remote_wakeup(&mut self) -> Result<(), Unsupported> {
            unreachable!()
        }
    }
    impl ControlPipe for Unused {
        fn max_packet_size(&self) -> usize {
            unreachable!()
        }
        async fn setup(&mut self) -> [u8; 8] {
            unreachable!()
        }
        async fn data_out(&mut self, _: &mut [u8], _: bool, _: bool) -> Result<usize, EndpointError> {
            unreachable!()
        }
        async fn data_in(&mut self, _: &[u8], _: bool, _: bool) -> Result<(), EndpointError> {
            unreachable!()
        }
        async fn accept(&mut self) {
            unreachable!()
        }
        async fn reject(&mut self) {
            unreachable!()
        }
        async fn accept_set_address(&mut self, _: u8) {
            unreachable!()
        }
    }

    struct MockDriver;
    impl<'d> Driver<'d> for MockDriver {
        type EndpointOut = MockEp;
        type EndpointIn = MockEp;
        type ControlPipe = Unused;
        type Bus = Unused;

        fn alloc_endpoint_out(&mut self, _: &EndpointConfig) -> Result<MockEp, EndpointAllocError> {
            unreachable!()
        }
        fn alloc_endpoint_in(&mut self, _: &EndpointConfig) -> Result<MockEp, EndpointAllocError> {
            unreachable!()
        }
        fn start(self, _: u16) -> (Unused, Unused) {
            unreachable!()
        }
    }

    struct MemItem {
        handle: u32,
        entry: ObjectEntry,
        data: Vec<u8>,
    }

    struct MemStorage {
        items: Vec<MemItem>,
        next_handle: u32,
        aborted_handles: Vec<u32>,
    }

    impl MemStorage {
        fn new() -> Self {
            let mut items = Vec::new();
            items.push(MemItem {
                handle: 1,
                entry: ObjectEntry::folder(0x0001_0001, 0, "docs"),
                data: Vec::new(),
            });
            // Exact 52-byte payload -> 12 + 52 = 64 bytes (exact max_packet_size multiple -> tests ZLP!)
            let exact_52 = vec![b'A'; 52];
            items.push(MemItem {
                handle: 2,
                entry: ObjectEntry::file(0x0001_0001, 1, "readme.txt", exact_52.len() as u64),
                data: exact_52,
            });
            Self {
                items,
                next_handle: 3,
                aborted_handles: Vec::new(),
            }
        }

        fn matches(item: &MemItem, storage_id: u32, format: Option<ObjectFormat>, parent: ParentScope) -> bool {
            if storage_id != STORAGE_ALL && item.entry.storage_id != storage_id {
                return false;
            }
            if let Some(f) = format
                && item.entry.format != f
            {
                return false;
            }
            match parent {
                ParentScope::All => true,
                ParentScope::Root => item.entry.parent == 0,
                ParentScope::Folder(p) => item.entry.parent == p,
            }
        }
    }

    impl MtpStorage for MemStorage {
        fn is_read_only(&self) -> bool {
            false
        }

        async fn storage_ids(&mut self, out: &mut [u32]) -> Result<usize, ResponseCode> {
            out[0] = 0x0001_0001;
            Ok(1)
        }

        async fn storage_info(&mut self, storage_id: u32) -> Result<StorageInfo, ResponseCode> {
            if storage_id != 0x0001_0001 {
                return Err(ResponseCode::INVALID_STORAGE_ID);
            }
            let mut info = StorageInfo {
                storage_type: StorageType::FixedRam,
                filesystem_type: FilesystemType::GenericHierarchical,
                access_capability: AccessCapability::ReadWrite,
                max_capacity: 1024 * 1024,
                free_space_bytes: 512 * 1024,
                free_space_objects: 100,
                ..Default::default()
            };
            let _ = info.description.push_str("Internal");
            let _ = info.volume_id.push_str("EMBASSY");
            Ok(info)
        }

        async fn num_objects(
            &mut self,
            storage_id: u32,
            format: Option<ObjectFormat>,
            parent: ParentScope,
        ) -> Result<u32, ResponseCode> {
            if storage_id != STORAGE_ALL && storage_id != 0x0001_0001 {
                return Err(ResponseCode::INVALID_STORAGE_ID);
            }
            Ok(self
                .items
                .iter()
                .filter(|it| Self::matches(it, storage_id, format, parent))
                .count() as u32)
        }

        async fn object_handles(
            &mut self,
            storage_id: u32,
            format: Option<ObjectFormat>,
            parent: ParentScope,
            offset: usize,
            out: &mut [u32],
        ) -> Result<usize, ResponseCode> {
            let mut n = 0;
            for (slot, item) in out.iter_mut().zip(
                self.items
                    .iter()
                    .filter(|it| Self::matches(it, storage_id, format, parent))
                    .skip(offset),
            ) {
                *slot = item.handle;
                n += 1;
            }
            Ok(n)
        }

        async fn object_info(&mut self, handle: u32) -> Result<ObjectEntry, ResponseCode> {
            self.items
                .iter()
                .find(|it| it.handle == handle)
                .map(|it| it.entry.clone())
                .ok_or(ResponseCode::INVALID_OBJECT_HANDLE)
        }

        async fn read_object(&mut self, handle: u32, offset: u64, buf: &mut [u8]) -> Result<usize, ResponseCode> {
            let item = self
                .items
                .iter()
                .find(|it| it.handle == handle)
                .ok_or(ResponseCode::INVALID_OBJECT_HANDLE)?;
            let off = offset as usize;
            if off >= item.data.len() {
                return Ok(0);
            }
            let n = (item.data.len() - off).min(buf.len());
            buf[..n].copy_from_slice(&item.data[off..off + n]);
            Ok(n)
        }

        async fn begin_send_object(
            &mut self,
            storage_id: u32,
            parent: ParentScope,
            info: &ObjectEntry,
        ) -> Result<(u32, u32, u32), ResponseCode> {
            let sid = if storage_id == 0 { 0x0001_0001 } else { storage_id };
            if sid != 0x0001_0001 {
                return Err(ResponseCode::INVALID_STORAGE_ID);
            }
            let parent_id = match parent {
                ParentScope::Root | ParentScope::All => 0,
                ParentScope::Folder(p) => p,
            };
            let handle = self.next_handle;
            self.next_handle += 1;
            let mut entry = info.clone();
            entry.storage_id = sid;
            entry.parent = parent_id;
            self.items.push(MemItem {
                handle,
                entry,
                data: Vec::new(),
            });
            Ok((sid, parent_id, handle))
        }

        async fn write_object_data(&mut self, handle: u32, offset: u64, data: &[u8]) -> Result<(), ResponseCode> {
            let item = self
                .items
                .iter_mut()
                .find(|it| it.handle == handle)
                .ok_or(ResponseCode::INVALID_OBJECT_HANDLE)?;
            assert_eq!(offset as usize, item.data.len());
            item.data.extend_from_slice(data);
            item.entry.size = item.data.len() as u64;
            Ok(())
        }

        async fn finish_send_object(&mut self, handle: u32, ok: bool) -> Result<(), ResponseCode> {
            if !ok {
                self.aborted_handles.push(handle);
                self.items.retain(|it| it.handle != handle);
            }
            Ok(())
        }

        async fn delete_object(&mut self, handle: u32, _format: Option<ObjectFormat>) -> Result<(), ResponseCode> {
            let before = self.items.len();
            self.items.retain(|it| it.handle != handle);
            if self.items.len() == before {
                Err(ResponseCode::INVALID_OBJECT_HANDLE)
            } else {
                Ok(())
            }
        }

        async fn rename_object(&mut self, handle: u32, new_name: &str) -> Result<(), ResponseCode> {
            let item = self
                .items
                .iter_mut()
                .find(|it| it.handle == handle)
                .ok_or(ResponseCode::INVALID_OBJECT_HANDLE)?;
            item.entry.filename.clear();
            let _ = item.entry.filename.push_str(new_name);
            Ok(())
        }
    }

    fn make_class(state: &Rc<MockEpState>) -> MtpClass<'static, MockDriver> {
        let state_box: &'static mut State<'static> = Box::leak(Box::new(State::new()));
        let bulk_out = MockEp {
            info: EndpointInfo {
                addr: EndpointAddress::from(0x01),
                ep_type: EndpointType::Bulk,
                max_packet_size: 64,
                interval_ms: 0,
                max_burst: 0,
            },
            state: state.clone(),
            is_int: false,
        };
        let bulk_in = MockEp {
            info: EndpointInfo {
                addr: EndpointAddress::from(0x81),
                ep_type: EndpointType::Bulk,
                max_packet_size: 64,
                interval_ms: 0,
                max_burst: 0,
            },
            state: state.clone(),
            is_int: false,
        };
        let int_in = MockEp {
            info: EndpointInfo {
                addr: EndpointAddress::from(0x82),
                ep_type: EndpointType::Interrupt,
                max_packet_size: 16,
                interval_ms: 16,
                max_burst: 0,
            },
            state: state.clone(),
            is_int: true,
        };
        MtpClass {
            transport: MtpTransport {
                read_ep: bulk_out,
                write_ep: bulk_in,
                shared: &state_box.shared,
            },
            events: MtpEventSender { int_ep: int_in },
            interface: InterfaceNumber(0),
            config: Config::new(),
        }
    }

    fn cmd_pkt(op: OperationCode, tx: u32, params: &[u32]) -> Vec<u8> {
        let len = CONTAINER_HEADER_LEN + params.len() * 4;
        let mut buf = vec![0u8; len];
        ContainerHeader {
            length: len as u32,
            container_type: ContainerType::Command,
            code: op.0,
            transaction_id: tx,
        }
        .encode(&mut buf)
        .unwrap();
        for (i, &p) in params.iter().enumerate() {
            buf[12 + i * 4..16 + i * 4].copy_from_slice(&p.to_le_bytes());
        }
        buf
    }

    fn data_pkt(code: u16, tx: u32, payload: &[u8]) -> Vec<Vec<u8>> {
        let mut full = vec![0u8; CONTAINER_HEADER_LEN + payload.len()];
        ContainerHeader {
            length: full.len() as u32,
            container_type: ContainerType::Data,
            code,
            transaction_id: tx,
        }
        .encode(&mut full)
        .unwrap();
        full[CONTAINER_HEADER_LEN..].copy_from_slice(payload);
        full.chunks(64).map(|c| c.to_vec()).collect()
    }

    fn drain_in_containers(pkts: &mut Vec<Vec<u8>>) -> (Vec<u8>, CommandBlock) {
        // Reassemble Data container until short packet (< 64) or ZLP (0)
        let mut data = Vec::new();
        while !pkts.is_empty() {
            let p = pkts.remove(0);
            let is_end = p.len() < 64;
            data.extend_from_slice(&p);
            if is_end {
                break;
            }
        }
        let resp_pkt = pkts.remove(0);
        let hdr = ContainerHeader::parse(&resp_pkt).unwrap();
        assert_eq!(hdr.container_type, ContainerType::Response);
        let mut resp_cmd_shape = resp_pkt.clone();
        resp_cmd_shape[4..6].copy_from_slice(&(ContainerType::Command as u16).to_le_bytes());
        let parsed = CommandBlock::parse(&resp_cmd_shape).unwrap();
        (data, parsed)
    }

    fn take_resp(pkts: &mut Vec<Vec<u8>>) -> (ResponseCode, u32, Vec<u32>) {
        let p = pkts.remove(0);
        let hdr = ContainerHeader::parse(&p).unwrap();
        assert_eq!(hdr.container_type, ContainerType::Response);
        let n_params = (hdr.length as usize - CONTAINER_HEADER_LEN) / 4;
        let mut params = Vec::new();
        for i in 0..n_params {
            params.push(u32::from_le_bytes(p[12 + i * 4..16 + i * 4].try_into().unwrap()));
        }
        (ResponseCode(hdr.code), hdr.transaction_id, params)
    }

    #[test]
    fn end_to_end_session_browse_read_write_delete() {
        let state = Rc::new(MockEpState::default());
        state.fail_after_empty.set(true);

        {
            let mut q = state.out_packets.borrow_mut();
            // 1. GetDeviceInfo without open session
            q.push(cmd_pkt(OperationCode::GET_DEVICE_INFO, 1, &[]));
            // 2. GetStorageIDs before session -> SESSION_NOT_OPEN
            q.push(cmd_pkt(OperationCode::GET_STORAGE_IDS, 2, &[]));
            // 3. OpenSession(1)
            q.push(cmd_pkt(OperationCode::OPEN_SESSION, 3, &[1]));
            // 4. GetStorageIDs
            q.push(cmd_pkt(OperationCode::GET_STORAGE_IDS, 4, &[]));
            // 5. GetObjectHandles(STORAGE_ALL, 0, HANDLE_ROOT)
            q.push(cmd_pkt(
                OperationCode::GET_OBJECT_HANDLES,
                5,
                &[STORAGE_ALL, 0, HANDLE_ROOT],
            ));
            // 6. GetObject(2) -> 52-byte payload + 12-byte header = 64 bytes -> must emit ZLP!
            q.push(cmd_pkt(OperationCode::GET_OBJECT, 6, &[2]));
            // 7. GetPartialObject(2, offset=10, max=16)
            q.push(cmd_pkt(OperationCode::GET_PARTIAL_OBJECT, 7, &[2, 10, 16]));
            // 8. GetObjectPropList(handle=2, format=0, prop=ALL, group=0, depth=0)
            q.push(cmd_pkt(
                OperationCode::GET_OBJECT_PROP_LIST,
                8,
                &[2, 0, 0xFFFF_FFFF, 0, 0],
            ));
            // 9. SendObjectInfo + SendObject
            let new_info = ObjectEntry::file(0, 1, "upload.bin", 5);
            let mut info_buf = [0u8; 256];
            let info_len = new_info.encode_object_info(&mut info_buf).unwrap();
            q.push(cmd_pkt(OperationCode::SEND_OBJECT_INFO, 9, &[0, 1]));
            q.extend(data_pkt(OperationCode::SEND_OBJECT_INFO.0, 9, &info_buf[..info_len]));
            q.push(cmd_pkt(OperationCode::SEND_OBJECT, 10, &[]));
            q.extend(data_pkt(OperationCode::SEND_OBJECT.0, 10, b"12345"));
            // 10. SetObjectPropValue (rename handle 3 -> "renamed.bin")
            let mut name_buf = [0u8; 64];
            let mut nw = super::super::DatasetWriter::new(&mut name_buf);
            nw.write_ptp_str("renamed.bin").unwrap();
            let name_len = nw.len();
            q.push(cmd_pkt(
                OperationCode::SET_OBJECT_PROP_VALUE,
                11,
                &[3, ObjectPropCode::OBJECT_FILE_NAME.0 as u32],
            ));
            q.extend(data_pkt(
                OperationCode::SET_OBJECT_PROP_VALUE.0,
                11,
                &name_buf[..name_len],
            ));
            // 11. DeleteObject(3)
            q.push(cmd_pkt(OperationCode::DELETE_OBJECT, 12, &[3]));
            // 12. CloseSession
            q.push(cmd_pkt(OperationCode::CLOSE_SESSION, 13, &[]));
        }

        let mut class = make_class(&state);
        let mut storage = MemStorage::new();
        let mut runner = storage::MtpRunner::new(Config::new());
        let mut buf = [0u8; 256];
        let res = block_on(runner.run_connected(&mut class.transport, &mut storage, &mut buf));
        assert_eq!(res, Err(TransportError::Endpoint(EndpointError::Disabled)));

        let mut in_pkts = state.in_packets.borrow_mut();

        // 1. GetDeviceInfo
        let (dev_data, resp1) = drain_in_containers(&mut in_pkts);
        assert_eq!(resp1.operation.0, ResponseCode::OK.0);
        let hdr1 = ContainerHeader::parse(&dev_data).unwrap();
        assert_eq!(hdr1.container_type, ContainerType::Data);
        assert_eq!(hdr1.length as usize, dev_data.len());

        // 2. GetStorageIDs before session -> SESSION_NOT_OPEN
        let (code2, _, _) = take_resp(&mut in_pkts);
        assert_eq!(code2, ResponseCode::SESSION_NOT_OPEN);

        // 3. OpenSession -> OK
        let (code3, _, _) = take_resp(&mut in_pkts);
        assert_eq!(code3, ResponseCode::OK);

        // 4. GetStorageIDs
        let (sid_data, resp4) = drain_in_containers(&mut in_pkts);
        assert_eq!(resp4.operation.0, ResponseCode::OK.0);
        let mut dr = DatasetReader::new(&sid_data[12..]);
        assert_eq!(dr.read_u32().unwrap(), 1);
        assert_eq!(dr.read_u32().unwrap(), 0x0001_0001);

        // 5. GetObjectHandles root -> only handle 1 ("docs")
        let (h_data, resp5) = drain_in_containers(&mut in_pkts);
        assert_eq!(resp5.operation.0, ResponseCode::OK.0);
        let mut hdr_r = DatasetReader::new(&h_data[12..]);
        assert_eq!(hdr_r.read_u32().unwrap(), 1);
        assert_eq!(hdr_r.read_u32().unwrap(), 1);

        // 6. GetObject(2) -> verify 64-byte packet + ZLP!
        assert_eq!(in_pkts[0].len(), 64);
        assert_eq!(in_pkts[1].len(), 0); // ZLP!
        let (obj_data, resp6) = drain_in_containers(&mut in_pkts);
        assert_eq!(resp6.operation.0, ResponseCode::OK.0);
        assert_eq!(&obj_data[12..], &[b'A'; 52]);

        // 7. GetPartialObject(2, 10, 16)
        let (part_data, resp7) = drain_in_containers(&mut in_pkts);
        assert_eq!(resp7.operation.0, ResponseCode::OK.0);
        assert_eq!(resp7.params(), &[16]);
        assert_eq!(&part_data[12..], &[b'A'; 16]);

        // 8. GetObjectPropList(2)
        let (plist_data, resp8) = drain_in_containers(&mut in_pkts);
        assert_eq!(resp8.operation.0, ResponseCode::OK.0);
        let mut pr = DatasetReader::new(&plist_data[12..]);
        assert_eq!(pr.read_u32().unwrap(), 9); // 9 properties in SUPPORTED_PROPS

        // 9. SendObjectInfo -> (0x0001_0001, 1, 3), SendObject -> OK
        let (code9, _, params9) = take_resp(&mut in_pkts);
        assert_eq!(code9, ResponseCode::OK);
        assert_eq!(params9, vec![0x0001_0001, 1, 3]);
        let (code10, _, _) = take_resp(&mut in_pkts);
        assert_eq!(code10, ResponseCode::OK);

        // 10. Rename -> OK
        let (code11, _, _) = take_resp(&mut in_pkts);
        assert_eq!(code11, ResponseCode::OK);

        // 11. Delete -> OK
        let (code12, _, _) = take_resp(&mut in_pkts);
        assert_eq!(code12, ResponseCode::OK);
        assert_eq!(storage.items.len(), 2); // back to initial 2 items

        // 12. CloseSession -> OK
        let (code13, _, _) = take_resp(&mut in_pkts);
        assert_eq!(code13, ResponseCode::OK);

        // Event sender test
        block_on(class.send_event(&EventBlock::new(EventCode::OBJECT_ADDED, &[42]))).unwrap();
        let int_pkts = state.int_packets.borrow();
        assert_eq!(int_pkts.len(), 1);
        assert_eq!(int_pkts[0].len(), 16);
    }

    #[test]
    fn device_info_wire_format_and_folder_and_orphan_send() {
        let state = Rc::new(MockEpState::default());
        state.fail_after_empty.set(true);

        {
            let mut q = state.out_packets.borrow_mut();
            q.push(cmd_pkt(OperationCode::GET_DEVICE_INFO, 1, &[]));
            q.push(cmd_pkt(OperationCode::OPEN_SESSION, 2, &[1]));
            // SendObject without SendObjectInfo -> must drain Data-OUT and return NO_VALID_OBJECT_INFO
            q.push(cmd_pkt(OperationCode::SEND_OBJECT, 3, &[]));
            q.extend(data_pkt(OperationCode::SEND_OBJECT.0, 3, b"orphan"));
            // SendObjectInfo for folder -> creates folder immediately, no SendObject needed
            let folder_info = ObjectEntry::folder(0, 0, "photos");
            let mut fbuf = [0u8; 256];
            let flen = folder_info.encode_object_info(&mut fbuf).unwrap();
            q.push(cmd_pkt(OperationCode::SEND_OBJECT_INFO, 4, &[0, HANDLE_ROOT]));
            q.extend(data_pkt(OperationCode::SEND_OBJECT_INFO.0, 4, &fbuf[..flen]));
            // Query handles across all folders -> 1 ("docs"), 2 ("readme.txt"), 3 ("photos")
            q.push(cmd_pkt(
                OperationCode::GET_OBJECT_HANDLES,
                5,
                &[STORAGE_ALL, 0, HANDLE_ALL],
            ));
            // GetObjectPropValue(handle=2, OBJECT_FILE_NAME)
            q.push(cmd_pkt(
                OperationCode::GET_OBJECT_PROP_VALUE,
                6,
                &[2, ObjectPropCode::OBJECT_FILE_NAME.0 as u32],
            ));
        }

        let mut class = make_class(&state);
        let mut storage = MemStorage::new();
        let mut runner = storage::MtpRunner::new(Config::new());
        let mut buf = [0u8; 256];
        let _ = block_on(runner.run_connected(&mut class.transport, &mut storage, &mut buf));

        let mut in_pkts = state.in_packets.borrow_mut();

        // 1. Decode GetDeviceInfo fields
        let (dev_data, resp1) = drain_in_containers(&mut in_pkts);
        assert_eq!(resp1.operation.0, ResponseCode::OK.0);
        let mut dr = DatasetReader::new(&dev_data[12..]);
        assert_eq!(dr.read_u16().unwrap(), PTP_STANDARD_VERSION);
        assert_eq!(dr.read_u32().unwrap(), MTP_VENDOR_EXTENSION_ID);
        assert_eq!(dr.read_u16().unwrap(), MTP_VERSION);
        assert_eq!(dr.read_ptp_str::<64>().unwrap().as_str(), MTP_EXTENSIONS_MICROSOFT);
        assert_eq!(dr.read_u16().unwrap(), 0); // FunctionalMode
        let ops_count = dr.read_u32().unwrap();
        assert_eq!(ops_count as usize, storage::RW_OPS.len());
        for _ in 0..ops_count {
            let _ = dr.read_u16().unwrap();
        }
        let evt_count = dr.read_u32().unwrap();
        for _ in 0..evt_count {
            let _ = dr.read_u16().unwrap();
        }
        assert_eq!(dr.read_u32().unwrap(), 0); // DeviceProps
        assert_eq!(dr.read_u32().unwrap(), 0); // CaptureFormats
        let fmt_count = dr.read_u32().unwrap();
        for _ in 0..fmt_count {
            let _ = dr.read_u16().unwrap();
        }
        assert_eq!(dr.read_ptp_str::<32>().unwrap().as_str(), "Embassy");
        assert_eq!(dr.read_ptp_str::<32>().unwrap().as_str(), "MTP Device");
        assert_eq!(dr.read_ptp_str::<32>().unwrap().as_str(), "1.0");
        assert_eq!(dr.read_ptp_str::<32>().unwrap().as_str(), "00000001");
        assert_eq!(dr.remaining(), 0);

        // 2. OpenSession
        assert_eq!(take_resp(&mut in_pkts).0, ResponseCode::OK);
        // 3. Orphan SendObject
        assert_eq!(take_resp(&mut in_pkts).0, ResponseCode::NO_VALID_OBJECT_INFO);
        // 4. Folder SendObjectInfo -> handle 3
        let (code4, _, params4) = take_resp(&mut in_pkts);
        assert_eq!(code4, ResponseCode::OK);
        assert_eq!(params4, vec![0x0001_0001, 0, 3]);
        // 5. All handles -> [1, 2, 3]
        let (all_h, resp5) = drain_in_containers(&mut in_pkts);
        assert_eq!(resp5.operation.0, ResponseCode::OK.0);
        let mut hr = DatasetReader::new(&all_h[12..]);
        assert_eq!(hr.read_u32().unwrap(), 3);
        assert_eq!(hr.read_u32().unwrap(), 1);
        assert_eq!(hr.read_u32().unwrap(), 2);
        assert_eq!(hr.read_u32().unwrap(), 3);
        // 6. GetObjectPropValue(2, OBJECT_FILE_NAME)
        let (prop_val, resp6) = drain_in_containers(&mut in_pkts);
        assert_eq!(resp6.operation.0, ResponseCode::OK.0);
        let mut pvr = DatasetReader::new(&prop_val[12..]);
        assert_eq!(pvr.read_ptp_str::<32>().unwrap().as_str(), "readme.txt");
    }

    #[test]
    fn cancel_aborts_pending_send_object() {
        let state = Rc::new(MockEpState::default());
        state.fail_after_empty.set(true);

        {
            let mut q = state.out_packets.borrow_mut();
            q.push(cmd_pkt(OperationCode::OPEN_SESSION, 1, &[1]));
            let new_info = ObjectEntry::file(0, 0, "aborted.bin", 10);
            let mut ibuf = [0u8; 256];
            let ilen = new_info.encode_object_info(&mut ibuf).unwrap();
            q.push(cmd_pkt(OperationCode::SEND_OBJECT_INFO, 2, &[0, HANDLE_ROOT]));
            q.extend(data_pkt(OperationCode::SEND_OBJECT_INFO.0, 2, &ibuf[..ilen]));
            // SendObject command arrives, and EP0 cancel was issued for tx 3 before Data-OUT
            q.push(cmd_pkt(OperationCode::SEND_OBJECT, 3, &[]));
        }

        let state_box: &'static mut State<'static> = Box::leak(Box::new(State::new()));
        let mut ctrl = control::Control {
            interface: InterfaceNumber(0),
            shared: &state_box.shared,
        };
        let mut cancel_payload = [0u8; 6];
        cancel_payload[0..2].copy_from_slice(&CANCEL_CODE.to_le_bytes());
        cancel_payload[2..6].copy_from_slice(&3u32.to_le_bytes());
        assert!(matches!(
            ctrl.control_out(
                Request {
                    direction: Direction::Out,
                    request_type: RequestType::Class,
                    recipient: Recipient::Interface,
                    request: REQ_CANCEL,
                    value: 0,
                    index: 0,
                    length: 6,
                },
                &cancel_payload,
            ),
            Some(OutResponse::Accepted)
        ));

        let mut transport = MtpTransport::<MockDriver> {
            read_ep: MockEp {
                info: EndpointInfo {
                    addr: EndpointAddress::from(0x01),
                    ep_type: EndpointType::Bulk,
                    max_packet_size: 64,
                    interval_ms: 0,
                    max_burst: 0,
                },
                state: state.clone(),
                is_int: false,
            },
            write_ep: MockEp {
                info: EndpointInfo {
                    addr: EndpointAddress::from(0x81),
                    ep_type: EndpointType::Bulk,
                    max_packet_size: 64,
                    interval_ms: 0,
                    max_burst: 0,
                },
                state: state.clone(),
                is_int: false,
            },
            shared: &state_box.shared,
        };

        let mut storage = MemStorage::new();
        let mut runner = storage::MtpRunner::new(Config::new());
        let mut buf = [0u8; 256];
        let _ = block_on(runner.run_connected(&mut transport, &mut storage, &mut buf));

        let mut in_pkts = state.in_packets.borrow_mut();
        assert_eq!(take_resp(&mut in_pkts).0, ResponseCode::OK); // OpenSession
        assert_eq!(take_resp(&mut in_pkts).0, ResponseCode::OK); // SendObjectInfo
        assert_eq!(take_resp(&mut in_pkts).0, ResponseCode::TRANSACTION_CANCELLED); // SendObject cancelled
        assert_eq!(storage.aborted_handles, vec![3]);
        assert_eq!(storage.items.len(), 2); // aborted object was cleaned up
    }

    #[test]
    fn builder_emits_still_image_descriptors() {
        struct DummyDriver(Rc<MockEpState>);
        impl<'d> Driver<'d> for DummyDriver {
            type EndpointOut = MockEp;
            type EndpointIn = MockEp;
            type ControlPipe = Unused;
            type Bus = Unused;

            fn alloc_endpoint_out(&mut self, cfg: &EndpointConfig) -> Result<MockEp, EndpointAllocError> {
                assert_eq!(cfg.ep_type, EndpointType::Bulk);
                Ok(MockEp {
                    info: EndpointInfo {
                        addr: cfg.addr.unwrap_or(EndpointAddress::from(0x01)),
                        ep_type: cfg.ep_type,
                        max_packet_size: cfg.max_packet_size,
                        interval_ms: cfg.interval_ms,
                        max_burst: cfg.max_burst,
                    },
                    state: self.0.clone(),
                    is_int: false,
                })
            }
            fn alloc_endpoint_in(&mut self, cfg: &EndpointConfig) -> Result<MockEp, EndpointAllocError> {
                assert!(matches!(cfg.ep_type, EndpointType::Bulk | EndpointType::Interrupt));
                Ok(MockEp {
                    info: EndpointInfo {
                        addr: cfg.addr.unwrap_or(EndpointAddress::from(0x81)),
                        ep_type: cfg.ep_type,
                        max_packet_size: cfg.max_packet_size,
                        interval_ms: cfg.interval_ms,
                        max_burst: cfg.max_burst,
                    },
                    state: self.0.clone(),
                    is_int: cfg.ep_type == EndpointType::Interrupt,
                })
            }
            fn start(self, _: u16) -> (Unused, Unused) {
                (Unused, Unused)
            }
        }

        let mut dev_cfg = crate::Config::new(0x1234, 0x5678);
        dev_cfg.composite_with_iads = false;
        dev_cfg.device_class = 0x00;
        dev_cfg.device_sub_class = 0x00;
        dev_cfg.device_protocol = 0x00;

        let mut config_desc = [0u8; 256];
        let mut bos_desc = [0u8; 128];
        let mut msos_desc = [0u8; 128];
        let mut control_buf = [0u8; 64];

        let mut builder = Builder::new(
            DummyDriver(Rc::new(MockEpState::default())),
            dev_cfg,
            &mut config_desc,
            &mut bos_desc,
            &mut msos_desc,
            &mut control_buf,
        );
        let mut state = State::new();
        let mut mtp_cfg = Config::new();
        mtp_cfg.msos_descriptor = true;
        let _mtp = MtpClass::new(&mut builder, &mut state, mtp_cfg);
        let dev = builder.build();
        drop(dev);

        assert!(
            config_desc
                .windows(3)
                .any(|w| w == [USB_CLASS_IMAGE, IMAGE_SUBCLASS_STILL_IMAGE, IMAGE_PROTOCOL_PIMA_15740])
        );
        assert!(msos_desc.windows(3).any(|w| w == b"MTP"));
    }
}
