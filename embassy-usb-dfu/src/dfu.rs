//! DFU bootloader part of DFU logic
use embassy_boot::{AlignedBuffer, BlockingFirmwareUpdater, FirmwareUpdaterError};
use embassy_usb::class::dfu::consts::{DfuAttributes, Status};
/// Re-export DfuState from embassy-usb for convenience.
pub use embassy_usb::class::dfu::dfu_mode::DfuState as UsbDfuState;
use embassy_usb::class::dfu::dfu_mode::{self, DfuState};
use embassy_usb::driver::Driver;
use embassy_usb::{Builder, FunctionBuilder};
use embedded_storage::nor_flash::{NorFlash, NorFlashErrorKind};

use crate::Reset;

/// Internal handler for USB DFU firmware updates.
///
/// This implements the `embassy_usb::class::dfu::dfu_mode::Handler` trait,
/// providing the firmware write logic using `BlockingFirmwareUpdater`.
pub struct FirmwareHandler<'d, DFU: NorFlash, STATE: NorFlash, RST: Reset, const BLOCK_SIZE: usize> {
    updater: BlockingFirmwareUpdater<'d, DFU, STATE>,
    offset: usize,
    buf: AlignedBuffer<BLOCK_SIZE>,
    reset: RST,

    #[cfg(feature = "_verify")]
    public_key: &'static [u8; 32],
}

impl<'d, DFU: NorFlash, STATE: NorFlash, RST: Reset, const BLOCK_SIZE: usize>
    FirmwareHandler<'d, DFU, STATE, RST, BLOCK_SIZE>
{
    /// Create a new firmware handler.
    pub fn new(
        updater: BlockingFirmwareUpdater<'d, DFU, STATE>,
        reset: RST,
        #[cfg(feature = "_verify")] public_key: &'static [u8; 32],
    ) -> Self {
        Self {
            updater,
            offset: 0,
            buf: AlignedBuffer([0; BLOCK_SIZE]),
            reset,

            #[cfg(feature = "_verify")]
            public_key,
        }
    }
}

fn firmware_error_to_status(e: FirmwareUpdaterError) -> Status {
    match e {
        FirmwareUpdaterError::Flash(e) => match e {
            NorFlashErrorKind::NotAligned => Status::ErrWrite,
            NorFlashErrorKind::OutOfBounds => Status::ErrAddress,
            _ => Status::ErrUnknown,
        },
        FirmwareUpdaterError::Signature(_) => Status::ErrVerify,
        FirmwareUpdaterError::BadState => Status::ErrUnknown,
    }
}

impl<'d, DFU: NorFlash, STATE: NorFlash, RST: Reset, const BLOCK_SIZE: usize> dfu_mode::Handler
    for FirmwareHandler<'d, DFU, STATE, RST, BLOCK_SIZE>
{
    fn start(&mut self) -> Result<(), Status> {
        info!("Download starting");
        self.offset = 0;
        Ok(())
    }

    fn write(&mut self, data: &[u8]) -> Result<(), Status> {
        if data.len() > BLOCK_SIZE {
            error!("USB data len exceeded block size");
            return Err(Status::ErrUnknown);
        }

        // Pad up to the flash write alignment. Only host-provided bytes count toward
        // `offset` (used later for signature length); padding must not leak stale
        // bytes from a previous block into the DFU partition.
        let write_len = data.len().next_multiple_of(DFU::WRITE_SIZE);
        if write_len > BLOCK_SIZE {
            error!("Padded write exceeded block size");
            return Err(Status::ErrUnknown);
        }

        let buf = self.buf.as_mut();
        buf[..data.len()].copy_from_slice(data);
        buf[data.len()..write_len].fill(0);

        debug!(
            "Writing {} bytes (padded from {}) at {}",
            write_len,
            data.len(),
            self.offset
        );
        match self.updater.write_firmware(self.offset, &buf[..write_len]) {
            Ok(_) => {
                self.offset += data.len();
                Ok(())
            }
            Err(e) => {
                error!("Error writing firmware: {:?}", e);
                Err(firmware_error_to_status(e))
            }
        }
    }

    fn finish(&mut self) -> Result<(), Status> {
        debug!("Receiving final transfer");

        #[cfg(feature = "_verify")]
        let update_res: Result<(), FirmwareUpdaterError> = {
            const SIGNATURE_LEN: usize = 64;

            let mut signature = [0; SIGNATURE_LEN];
            let update_len = (self.offset - SIGNATURE_LEN) as u32;

            self.updater.read_dfu(update_len, &mut signature).and_then(|_| {
                self.updater
                    .verify_and_mark_updated(self.public_key, &signature, update_len)
            })
        };

        #[cfg(not(feature = "_verify"))]
        let update_res = self.updater.mark_updated();

        match update_res {
            Ok(_) => {
                info!("Update complete");
                Ok(())
            }
            Err(e) => {
                error!("Error completing update: {}", e);
                Err(firmware_error_to_status(e))
            }
        }
    }

    fn system_reset(&mut self) {
        self.reset.sys_reset()
    }
}

/// Convenience type alias for the DFU state with firmware handler.
pub type State<'d, DFU, STATE, RST, const BLOCK_SIZE: usize> =
    DfuState<FirmwareHandler<'d, DFU, STATE, RST, BLOCK_SIZE>>;

/// Create a new DFU state instance.
///
/// This creates a `DfuState` with a `FirmwareHandler` inside, ready to be
/// used with `usb_dfu`.
pub fn new_state<'d, DFU: NorFlash, STATE: NorFlash, RST: Reset, const BLOCK_SIZE: usize>(
    updater: BlockingFirmwareUpdater<'d, DFU, STATE>,
    attrs: DfuAttributes,
    reset: RST,
    #[cfg(feature = "_verify")] public_key: &'static [u8; 32],
) -> State<'d, DFU, STATE, RST, BLOCK_SIZE> {
    let handler = FirmwareHandler::new(
        updater,
        reset,
        #[cfg(feature = "_verify")]
        public_key,
    );
    DfuState::new(handler, attrs)
}

/// An implementation of the USB DFU 1.1 protocol
///
/// This function will add a DFU interface descriptor to the provided Builder, and register the provided Control as a handler for the USB device
/// The handler is responsive to DFU GetState, GetStatus, Abort, and ClrStatus commands, as well as Download if configured by the user.
///
/// Once the host has initiated a DFU download operation, the chunks sent by the host will be written to the DFU partition.
/// Once the final sync in the manifestation phase has been received, the handler will trigger a system reset to swap the new firmware.
pub fn usb_dfu<'d, D: Driver<'d>, DFU: NorFlash, STATE: NorFlash, RST: Reset, const BLOCK_SIZE: usize>(
    builder: &mut Builder<'d, D>,
    state: &'d mut State<'d, DFU, STATE, RST, BLOCK_SIZE>,
    func_modifier: impl Fn(&mut FunctionBuilder<'_, 'd, D>),
) {
    dfu_mode::usb_dfu(builder, state, BLOCK_SIZE, func_modifier);
}
