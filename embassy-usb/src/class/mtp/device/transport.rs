//! Packetized bulk container transport and streaming Data IN/OUT helpers.

use core::cmp::min;

use super::super::{
    CONTAINER_HEADER_LEN, CommandBlock, ContainerHeader, ContainerType, EventBlock, MAX_COMMAND_CONTAINER_LEN,
    MAX_EVENT_CONTAINER_LEN, ResponseBlock, UNKNOWN_CONTAINER_LEN, WireError,
};
use super::control::ControlShared;
use crate::driver::{Driver, Endpoint, EndpointError, EndpointIn, EndpointOut};

/// Error returned by MTP bulk/interrupt transport operations.
#[derive(Copy, Clone, Eq, PartialEq, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum TransportError {
    /// Underlying USB endpoint error (e.g. disconnected or disabled).
    Endpoint(EndpointError),
    /// Invalid or truncated MTP container framing.
    Wire(WireError),
    /// The active transaction was cancelled by the host via EP0 `Cancel`.
    Cancelled,
    /// The host issued an EP0 `DeviceReset`.
    Reset,
    /// Scratch buffer is smaller than the endpoint `max_packet_size`.
    BufferTooSmall,
    /// Data-OUT transfer ended early or exceeded the declared length.
    UnexpectedLength,
}

impl From<EndpointError> for TransportError {
    fn from(e: EndpointError) -> Self {
        Self::Endpoint(e)
    }
}

impl From<WireError> for TransportError {
    fn from(e: WireError) -> Self {
        Self::Wire(e)
    }
}

/// Low-level MTP bulk transport (Bulk-OUT + Bulk-IN + shared EP0 control state).
pub struct MtpTransport<'d, D: Driver<'d>> {
    pub(super) read_ep: D::EndpointOut,
    pub(super) write_ep: D::EndpointIn,
    pub(super) shared: &'d ControlShared,
}

impl<'d, D: Driver<'d>> MtpTransport<'d, D> {
    /// Returns the Bulk endpoint `max_packet_size`.
    pub fn max_packet_size(&self) -> u16 {
        self.read_ep.info().max_packet_size
    }

    /// Waits until the Bulk endpoints are enabled by the host.
    pub async fn wait_connection(&mut self) {
        self.read_ep.wait_enabled().await;
    }

    /// Returns a reference to the shared EP0 control state.
    pub fn control(&self) -> &'d ControlShared {
        self.shared
    }

    /// Reads the next Command container from Bulk-OUT.
    ///
    /// Ignores zero-length packets that some hosts emit after exact-multiple Bulk-OUT
    /// data transfers. `buf` must be at least `max(max_packet_size, 32)` bytes.
    pub async fn read_command(&mut self, buf: &mut [u8]) -> Result<CommandBlock, TransportError> {
        let max_pkt = self.read_ep.info().max_packet_size as usize;
        if buf.len() < max_pkt || buf.len() < MAX_COMMAND_CONTAINER_LEN {
            return Err(TransportError::BufferTooSmall);
        }

        loop {
            if self.shared.take_reset() {
                return Err(TransportError::Reset);
            }
            let n = self.read_ep.read(&mut buf[..max_pkt]).await?;
            if n == 0 {
                // Skip optional terminating ZLP from an earlier Bulk-OUT transfer.
                continue;
            }
            return Ok(CommandBlock::parse(&buf[..n])?);
        }
    }

    /// Starts a streaming Data-IN container to the host.
    ///
    /// `payload_len` is the number of payload bytes (excluding the 12-byte container
    /// header), or `None` to write [`UNKNOWN_CONTAINER_LEN`] (`0xFFFF_FFFF`).
    /// `packet_buf` must be at least `max_packet_size` bytes long.
    pub fn start_data_in<'a>(
        &'a mut self,
        code: u16,
        transaction_id: u32,
        payload_len: Option<u32>,
        packet_buf: &'a mut [u8],
    ) -> Result<DataWriter<'a, 'd, D::EndpointIn>, TransportError> {
        let max_pkt = self.write_ep.info().max_packet_size as usize;
        if packet_buf.len() < max_pkt || max_pkt < CONTAINER_HEADER_LEN {
            return Err(TransportError::BufferTooSmall);
        }

        let length = match payload_len {
            Some(n) => n
                .checked_add(CONTAINER_HEADER_LEN as u32)
                .unwrap_or(UNKNOWN_CONTAINER_LEN),
            None => UNKNOWN_CONTAINER_LEN,
        };

        ContainerHeader {
            length,
            container_type: ContainerType::Data,
            code,
            transaction_id,
        }
        .encode(packet_buf)?;

        Ok(DataWriter {
            ep: &mut self.write_ep,
            shared: self.shared,
            transaction_id,
            packet_buf,
            max_pkt,
            buf_pos: CONTAINER_HEADER_LEN,
            total_sent: 0,
        })
    }

    /// Sends a complete Data-IN container whose payload fits in `payload`.
    pub async fn write_data(
        &mut self,
        code: u16,
        transaction_id: u32,
        payload: &[u8],
        packet_buf: &mut [u8],
    ) -> Result<(), TransportError> {
        let mut writer = self.start_data_in(code, transaction_id, Some(payload.len() as u32), packet_buf)?;
        writer.write_all(payload).await?;
        writer.finish().await
    }

    /// Starts a streaming Data-OUT container from the host.
    ///
    /// `packet_buf` must be at least `max_packet_size` bytes long.
    pub async fn start_data_out<'a>(
        &'a mut self,
        transaction_id: u32,
        packet_buf: &'a mut [u8],
    ) -> Result<DataReader<'a, 'd, D::EndpointOut>, TransportError> {
        let max_pkt = self.read_ep.info().max_packet_size as usize;
        if packet_buf.len() < max_pkt || max_pkt < CONTAINER_HEADER_LEN {
            return Err(TransportError::BufferTooSmall);
        }

        let n = loop {
            if self.shared.take_reset() {
                return Err(TransportError::Reset);
            }
            if self.shared.is_cancelled(transaction_id) {
                return Err(TransportError::Cancelled);
            }
            let n = self.read_ep.read(&mut packet_buf[..max_pkt]).await?;
            if n == 0 {
                continue;
            }
            break n;
        };

        let hdr = ContainerHeader::parse(&packet_buf[..n])?;
        if hdr.container_type != ContainerType::Data {
            return Err(TransportError::Wire(WireError::UnexpectedContainerType(
                hdr.container_type,
            )));
        }

        let declared_payload_len = if hdr.length == UNKNOWN_CONTAINER_LEN {
            None
        } else {
            Some(hdr.length as usize - CONTAINER_HEADER_LEN)
        };

        let inline = n - CONTAINER_HEADER_LEN;
        let (pkt_end, remaining_after_pkt, eof) = match declared_payload_len {
            Some(total) => {
                let take = min(inline, total);
                let rem = total - take;
                (CONTAINER_HEADER_LEN + take, Some(rem), rem == 0)
            }
            None => (n, None, n < max_pkt),
        };

        Ok(DataReader {
            ep: &mut self.read_ep,
            shared: self.shared,
            transaction_id,
            code: hdr.code,
            declared_payload_len,
            packet_buf,
            max_pkt,
            cursor: CONTAINER_HEADER_LEN,
            pkt_end,
            remaining: remaining_after_pkt,
            eof,
        })
    }

    /// Sends a Response container on Bulk-IN.
    pub async fn send_response(&mut self, resp: &ResponseBlock) -> Result<(), TransportError> {
        let mut buf = [0u8; MAX_COMMAND_CONTAINER_LEN];
        let len = resp.encode(&mut buf)?;
        self.write_ep.write(&buf[..len]).await?;
        Ok(())
    }
}

/// Async interrupt-IN sender for asynchronous MTP events.
pub struct MtpEventSender<'d, D: Driver<'d>> {
    pub(super) int_ep: D::EndpointIn,
}

impl<'d, D: Driver<'d>> MtpEventSender<'d, D> {
    /// Waits until the Interrupt-IN endpoint is enabled.
    pub async fn wait_connection(&mut self) {
        self.int_ep.wait_enabled().await;
    }

    /// Sends an MTP event container on the Interrupt-IN endpoint.
    pub async fn send_event(&mut self, event: &EventBlock) -> Result<(), EndpointError> {
        let mut buf = [0u8; MAX_EVENT_CONTAINER_LEN];
        let len = event.encode(&mut buf).expect("event buffer is MAX_EVENT_CONTAINER_LEN");
        self.int_ep.write(&buf[..len]).await
    }
}

/// Streaming writer for an MTP Data-IN container.
///
/// Packs the 12-byte container header together with the first packet of payload and
/// ensures every intermediate USB packet is a full `max_packet_size`, terminating with
/// a short packet or ZLP in [`Self::finish`].
pub struct DataWriter<'a, 'd, E: EndpointIn> {
    ep: &'a mut E,
    shared: &'d ControlShared,
    transaction_id: u32,
    packet_buf: &'a mut [u8],
    max_pkt: usize,
    buf_pos: usize,
    total_sent: usize,
}

impl<'a, 'd, E: EndpointIn> DataWriter<'a, 'd, E> {
    fn check_abort(&self) -> Result<(), TransportError> {
        if self.shared.take_reset() {
            return Err(TransportError::Reset);
        }
        if self.shared.is_cancelled(self.transaction_id) {
            return Err(TransportError::Cancelled);
        }
        Ok(())
    }

    /// Writes a slice of payload bytes into the Data-IN stream.
    pub async fn write_all(&mut self, mut data: &[u8]) -> Result<(), TransportError> {
        while !data.is_empty() {
            self.check_abort()?;

            if self.buf_pos == 0 && data.len() >= self.max_pkt {
                self.ep.write(&data[..self.max_pkt]).await?;
                self.total_sent += self.max_pkt;
                data = &data[self.max_pkt..];
                continue;
            }

            let take = min(self.max_pkt - self.buf_pos, data.len());
            self.packet_buf[self.buf_pos..self.buf_pos + take].copy_from_slice(&data[..take]);
            self.buf_pos += take;
            data = &data[take..];

            if self.buf_pos == self.max_pkt {
                self.ep.write(&self.packet_buf[..self.max_pkt]).await?;
                self.total_sent += self.max_pkt;
                self.buf_pos = 0;
            }
        }
        Ok(())
    }

    /// Flushes any remaining partial packet or emits a terminating ZLP if the total
    /// container byte count is an exact multiple of `max_packet_size`.
    pub async fn finish(self) -> Result<(), TransportError> {
        self.check_abort()?;
        if self.buf_pos > 0 {
            self.ep.write(&self.packet_buf[..self.buf_pos]).await?;
        } else if self.total_sent > 0 {
            self.ep.write(&[]).await?;
        }
        Ok(())
    }
}

/// Streaming reader for an MTP Data-OUT container.
pub struct DataReader<'a, 'd, E: EndpointOut> {
    ep: &'a mut E,
    shared: &'d ControlShared,
    transaction_id: u32,
    code: u16,
    declared_payload_len: Option<usize>,
    packet_buf: &'a mut [u8],
    max_pkt: usize,
    cursor: usize,
    pkt_end: usize,
    remaining: Option<usize>,
    eof: bool,
}

impl<'a, 'd, E: EndpointOut> DataReader<'a, 'd, E> {
    /// Operation code from the Data container header.
    #[inline]
    pub fn code(&self) -> u16 {
        self.code
    }

    /// Declared payload length in bytes, or `None` if the host sent `0xFFFF_FFFF`.
    #[inline]
    pub fn payload_len(&self) -> Option<usize> {
        self.declared_payload_len
    }

    fn check_abort(&self) -> Result<(), TransportError> {
        if self.shared.take_reset() {
            return Err(TransportError::Reset);
        }
        if self.shared.is_cancelled(self.transaction_id) {
            return Err(TransportError::Cancelled);
        }
        Ok(())
    }

    /// Reads the next chunk of payload into `out`, returning `0` at end-of-container.
    pub async fn read_chunk(&mut self, out: &mut [u8]) -> Result<usize, TransportError> {
        if out.is_empty() {
            return Ok(0);
        }
        self.check_abort()?;

        if self.cursor < self.pkt_end {
            let avail = self.pkt_end - self.cursor;
            let take = min(avail, out.len());
            out[..take].copy_from_slice(&self.packet_buf[self.cursor..self.cursor + take]);
            self.cursor += take;
            return Ok(take);
        }

        if self.eof {
            return Ok(0);
        }

        let n = self.ep.read(&mut self.packet_buf[..self.max_pkt]).await?;
        if n == 0 {
            self.eof = true;
            if self.remaining.is_some_and(|r| r > 0) {
                return Err(TransportError::UnexpectedLength);
            }
            return Ok(0);
        }

        let valid = match self.remaining {
            Some(rem) => {
                if n < self.max_pkt && n < rem {
                    return Err(TransportError::UnexpectedLength);
                }
                let take = min(n, rem);
                let new_rem = rem - take;
                self.remaining = Some(new_rem);
                if new_rem == 0 {
                    self.eof = true;
                }
                take
            }
            None => {
                if n < self.max_pkt {
                    self.eof = true;
                }
                n
            }
        };

        let copy_len = min(valid, out.len());
        out[..copy_len].copy_from_slice(&self.packet_buf[..copy_len]);
        self.cursor = copy_len;
        self.pkt_end = valid;
        Ok(copy_len)
    }

    /// Reads the entire remaining payload into `out`, returning the byte count.
    pub async fn read_all(mut self, out: &mut [u8]) -> Result<usize, TransportError> {
        let mut total = 0;
        loop {
            if total == out.len() {
                // Check whether more payload remains.
                let mut extra = [0u8; 1];
                if self.read_chunk(&mut extra).await? > 0 {
                    return Err(TransportError::BufferTooSmall);
                }
                return Ok(total);
            }
            let n = self.read_chunk(&mut out[total..]).await?;
            if n == 0 {
                return Ok(total);
            }
            total += n;
        }
    }

    /// Discards all remaining payload bytes so Bulk-OUT stays aligned for the next command.
    pub async fn discard(mut self) -> Result<(), TransportError> {
        self.cursor = self.pkt_end;
        while !self.eof {
            self.check_abort()?;
            let n = self.ep.read(&mut self.packet_buf[..self.max_pkt]).await?;
            if n == 0 {
                break;
            }
            match self.remaining {
                Some(rem) => {
                    let take = min(n, rem);
                    let new_rem = rem - take;
                    self.remaining = Some(new_rem);
                    if new_rem == 0 || n < self.max_pkt {
                        break;
                    }
                }
                None => {
                    if n < self.max_pkt {
                        break;
                    }
                }
            }
        }
        Ok(())
    }
}
