/* Application memory layout — Efekta E-Ink 290 nRF52840 (1024 KB flash, 256 KB RAM)
 *
 * Must match bootloader/memory.x partition boundaries exactly!
 *
 * 0x0000_0000  ┌─────────────────────┐
 *              │  BOOTLOADER (24 KB)  │  embassy-boot-nrf
 * 0x0000_6000  ├─────────────────────┤
 *              │  BOOT STATE (4 KB)   │  swap/revert tracking
 * 0x0000_7000  ├─────────────────────┤
 *              │  FLASH/ACTIVE(468KB) │  ← this firmware runs here
 * 0x0007_C000  ├─────────────────────┤
 *              │  DFU (472 KB)        │  OTA staging area
 * 0x000F_2000  ├─────────────────────┤
 *              │  NV STORAGE (56 KB)  │  Zigbee NV, calibration
 * 0x0010_0000  └─────────────────────┘
 */

MEMORY
{
    /* Bootloader occupies 0x0000_0000..0x0000_7000 — app starts after it */
    BOOTLOADER                        : ORIGIN = 0x00000000, LENGTH = 24K
    BOOTLOADER_STATE                  : ORIGIN = 0x00006000, LENGTH = 4K
    FLASH                             : ORIGIN = 0x00007000, LENGTH = 468K
    DFU                               : ORIGIN = 0x0007C000, LENGTH = 472K
    RAM                         (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}

__bootloader_state_start = ORIGIN(BOOTLOADER_STATE);
__bootloader_state_end = ORIGIN(BOOTLOADER_STATE) + LENGTH(BOOTLOADER_STATE);

__bootloader_dfu_start = ORIGIN(DFU);
__bootloader_dfu_end = ORIGIN(DFU) + LENGTH(DFU);
