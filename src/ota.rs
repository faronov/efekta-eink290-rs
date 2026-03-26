//! OTA firmware writer for nRF52840 internal flash (NVMC).
//!
//! Implements the `FirmwareWriter` trait from zigbee-runtime to write OTA
//! image blocks to the DFU staging slot. Uses embassy-nrf's NVMC driver for
//! page erase and 4-byte-aligned writes.
//!
//! Flash layout (must match bootloader/memory.x):
//! - DFU slot: 0x0007_C000..0x000F_2000 (472 KB)
//! - State:    0x0000_6000..0x0000_7000 (4 KB, bootloader state)

use embassy_nrf::nvmc::Nvmc;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use zigbee_runtime::firmware_writer::{FirmwareError, FirmwareWriter};

/// DFU slot start address in flash.
const DFU_START: u32 = 0x0007_C000;
/// DFU slot size (472 KB = active 468 KB + 1 swap page 4 KB).
const DFU_SIZE: u32 = 472 * 1024;
/// Bootloader state partition start.
const STATE_START: u32 = 0x0000_6000;
/// nRF52840 flash page size.
const PAGE_SIZE: u32 = 4096;

/// embassy-boot 0.4 swap magic: state word filled with 0xF0 triggers swap.
const SWAP_MAGIC: u8 = 0xF0;

/// Firmware writer that writes OTA blocks to the nRF52840 DFU flash slot.
pub struct NrfFirmwareWriter<'d> {
    nvmc: Nvmc<'d>,
    /// Track which pages have been erased to avoid redundant erases.
    last_erased_page: Option<u32>,
}

impl<'d> NrfFirmwareWriter<'d> {
    pub fn new(nvmc: Nvmc<'d>) -> Self {
        Self {
            nvmc,
            last_erased_page: None,
        }
    }

    /// Erase a single 4KB page if not already erased.
    fn ensure_page_erased(&mut self, page_addr: u32) -> Result<(), FirmwareError> {
        if self.last_erased_page == Some(page_addr) {
            return Ok(());
        }
        self.nvmc
            .erase(page_addr, page_addr + PAGE_SIZE)
            .map_err(|_| FirmwareError::EraseFailed)?;
        self.last_erased_page = Some(page_addr);
        Ok(())
    }
}

impl<'d> FirmwareWriter for NrfFirmwareWriter<'d> {
    fn erase_slot(&mut self) -> Result<(), FirmwareError> {
        // Erase entire DFU slot (472 KB = 118 pages)
        let end = DFU_START + DFU_SIZE;
        let mut addr = DFU_START;
        while addr < end {
            self.nvmc
                .erase(addr, addr + PAGE_SIZE)
                .map_err(|_| FirmwareError::EraseFailed)?;
            addr += PAGE_SIZE;
        }
        self.last_erased_page = None;
        Ok(())
    }

    fn write_block(&mut self, offset: u32, data: &[u8]) -> Result<(), FirmwareError> {
        if offset + data.len() as u32 > DFU_SIZE {
            return Err(FirmwareError::OutOfRange);
        }

        let abs_addr = DFU_START + offset;

        // Erase page(s) that this block spans, if needed
        let start_page = abs_addr & !(PAGE_SIZE - 1);
        let end_page = (abs_addr + data.len() as u32 - 1) & !(PAGE_SIZE - 1);
        let mut page = start_page;
        while page <= end_page {
            self.ensure_page_erased(page)?;
            page += PAGE_SIZE;
        }

        // nRF52840 NVMC requires 4-byte aligned writes.
        // OTA blocks are 48 bytes (aligned), but handle edge cases.
        if !data.len().is_multiple_of(4) {
            // Pad to 4-byte alignment
            let mut aligned = [0xFFu8; 64];
            let len = data.len().min(64);
            aligned[..len].copy_from_slice(&data[..len]);
            let aligned_len = (len + 3) & !3;
            self.nvmc
                .write(abs_addr, &aligned[..aligned_len])
                .map_err(|_| FirmwareError::WriteFailed)?;
        } else {
            self.nvmc
                .write(abs_addr, data)
                .map_err(|_| FirmwareError::WriteFailed)?;
        }

        Ok(())
    }

    fn verify(
        &mut self,
        expected_size: u32,
        _expected_hash: Option<&[u8]>,
    ) -> Result<(), FirmwareError> {
        if expected_size > DFU_SIZE {
            return Err(FirmwareError::ImageTooLarge);
        }
        // Basic check: read first 4 bytes to verify non-empty (should be stack pointer)
        let mut buf = [0u8; 4];
        self.nvmc
            .read(DFU_START, &mut buf)
            .map_err(|_| FirmwareError::VerifyFailed)?;
        // Stack pointer should be in RAM range (0x2000_0000..0x2004_0000)
        let sp = u32::from_le_bytes(buf);
        if !(0x2000_0000..=0x2004_0000).contains(&sp) {
            return Err(FirmwareError::VerifyFailed);
        }
        Ok(())
    }

    fn activate(&mut self) -> Result<(), FirmwareError> {
        // Write swap magic to bootloader state partition.
        // embassy-boot expects the state partition to contain SWAP_MAGIC (0x01)
        // to trigger a swap on next boot.
        //
        // First erase the state page, then write the magic byte.
        self.nvmc
            .erase(STATE_START, STATE_START + PAGE_SIZE)
            .map_err(|_| FirmwareError::ActivateFailed)?;

        // Write magic word — all bytes must be SWAP_MAGIC (nRF52840 WRITE_SIZE = 4)
        let magic = [SWAP_MAGIC; 4];
        self.nvmc
            .write(STATE_START, &magic)
            .map_err(|_| FirmwareError::ActivateFailed)?;

        Ok(())
    }

    fn slot_size(&self) -> u32 {
        DFU_SIZE
    }

    fn abort(&mut self) -> Result<(), FirmwareError> {
        // Nothing to do — bootloader will boot the existing active slot
        // since we haven't written swap magic.
        self.last_erased_page = None;
        Ok(())
    }
}
