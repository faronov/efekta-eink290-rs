/* Bootloader memory layout — Efekta E-Ink 290 nRF52840 (1024 KB flash, 256 KB RAM)
 *
 * 0x0000_0000  ┌─────────────────────┐
 *              │  BOOTLOADER (24 KB)  │  embassy-boot-nrf runs here
 * 0x0000_6000  ├─────────────────────┤
 *              │  BOOT STATE (4 KB)   │  swap/revert tracking (1 page)
 * 0x0000_7000  ├─────────────────────┤
 *              │  ACTIVE (468 KB)     │  running firmware (Slot 0)
 * 0x0007_C000  ├─────────────────────┤
 *              │  DFU (472 KB)        │  incoming OTA image (Slot 1)
 *              │  (ACTIVE + 1 page)   │  extra page for swap scratch
 * 0x000F_2000  ├─────────────────────┤
 *              │  NV STORAGE (56 KB)  │  Zigbee NV, calibration data
 * 0x0010_0000  └─────────────────────┘
 *
 * DFU must be ACTIVE size + 1 erase page (4 KB) for swap algorithm.
 * Total: 24 + 4 + 468 + 472 + 56 = 1024 KB ✓
 */

MEMORY
{
    FLASH            : ORIGIN = 0x00000000, LENGTH = 24K
    BOOTLOADER_STATE : ORIGIN = 0x00006000, LENGTH = 4K
    ACTIVE           : ORIGIN = 0x00007000, LENGTH = 468K
    DFU              : ORIGIN = 0x0007C000, LENGTH = 472K
    RAM        (rwx) : ORIGIN = 0x20000000, LENGTH = 32K
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE);

__bootloader_active_start = ORIGIN(ACTIVE);
__bootloader_active_end = ORIGIN(ACTIVE) + LENGTH(ACTIVE);

__bootloader_dfu_start = ORIGIN(DFU);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU);
