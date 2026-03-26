//! SSD1680 e-paper display driver — async Embassy-based, 2.9" 128×296 B/W.
//!
//! Drives the GDEM029T94 panel on the Efekta E-Ink 290 board via SPI.
//! Supports full and partial refresh with a 9+1 strategy (every 10th
//! partial update triggers a full LUT refresh to clear ghosting).

use embassy_nrf::{gpio, peripherals, spim};
use embassy_time::{Duration, Instant, Timer};

use defmt::*;

pub const WIDTH: u16 = 128;
pub const HEIGHT: u16 = 296;
pub const BUF_SIZE: usize = (WIDTH as usize * HEIGHT as usize) / 8; // 4736

// SSD1680 commands
const CMD_DRIVER_CONTROL: u8 = 0x01;
const CMD_DATA_MODE: u8 = 0x11;
const CMD_SW_RESET: u8 = 0x12;
const CMD_TEMP_CONTROL: u8 = 0x18;
const CMD_MASTER_ACTIVATE: u8 = 0x20;
const CMD_DISP_CTRL2: u8 = 0x22;
const CMD_WRITE_RAM_BW: u8 = 0x24;
const CMD_WRITE_VCOM: u8 = 0x2C;
const CMD_WRITE_LUT: u8 = 0x32;
const CMD_WRITE_DUMMY: u8 = 0x3A;
const CMD_WRITE_GATELINE: u8 = 0x3B;
const CMD_WRITE_BORDER: u8 = 0x3C;
const CMD_SET_RAM_X_POS: u8 = 0x44;
const CMD_SET_RAM_Y_POS: u8 = 0x45;
const CMD_SET_RAM_X_CNT: u8 = 0x4E;
const CMD_SET_RAM_Y_CNT: u8 = 0x4F;
const CMD_DEEP_SLEEP: u8 = 0x10;

/// Full-refresh LUT — balanced quality, moderate ghosting removal.
static LUT_FULL: [u8; 70] = [
    0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00, // BB
    0x10, 0x60, 0x20, 0x00, 0x00, 0x00, 0x00, // BW
    0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00, // WB
    0x10, 0x60, 0x20, 0x00, 0x00, 0x00, 0x00, // WW
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // VCOM
    0x03, 0x03, 0x00, 0x00, 0x02, // TP0
    0x09, 0x09, 0x00, 0x00, 0x02, // TP1
    0x03, 0x03, 0x00, 0x00, 0x02, // TP2
    0x00, 0x00, 0x00, 0x00, 0x00, // TP3
    0x00, 0x00, 0x00, 0x00, 0x00, // TP4
    0x00, 0x00, 0x00, 0x00, 0x00, // TP5
    0x00, 0x00, 0x00, 0x00, 0x00, // TP6
];

/// Partial-refresh LUT — fast, single-phase, no flicker.
static LUT_PARTIAL: [u8; 70] = [
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // BB
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // BW
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // WB
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // WW
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // VCOM
    0x0A, 0x00, 0x00, 0x00, 0x00, // TP0
    0x00, 0x00, 0x00, 0x00, 0x00, // TP1
    0x00, 0x00, 0x00, 0x00, 0x00, // TP2
    0x00, 0x00, 0x00, 0x00, 0x00, // TP3
    0x00, 0x00, 0x00, 0x00, 0x00, // TP4
    0x00, 0x00, 0x00, 0x00, 0x00, // TP5
    0x00, 0x00, 0x00, 0x00, 0x00, // TP6
];

/// Full-refresh interval for the 9+1 strategy.
const PARTIAL_MAX: u8 = 9;

pub struct Ssd1680<'a> {
    spi: spim::Spim<'a, peripherals::TWISPI1>,
    cs: gpio::Output<'a>,
    dc: gpio::Output<'a>,
    rst: gpio::Output<'a>,
    busy: gpio::Input<'a>,
    partial_count: u8,
}

impl<'a> Ssd1680<'a> {
    /// Create driver, perform hardware reset and full init sequence.
    pub async fn new(
        spi: spim::Spim<'a, peripherals::TWISPI1>,
        cs: gpio::Output<'a>,
        dc: gpio::Output<'a>,
        rst: gpio::Output<'a>,
        busy: gpio::Input<'a>,
    ) -> Self {
        let mut epd = Self {
            spi,
            cs,
            dc,
            rst,
            busy,
            partial_count: 0,
        };
        epd.hw_reset().await;
        epd.init().await;
        epd
    }

    // ── Low-level SPI helpers ────────────────────────────────

    async fn send_command(&mut self, cmd: u8) {
        self.dc.set_low();
        self.cs.set_low();
        let _ = self.spi.write(&[cmd]).await;
        self.cs.set_high();
    }

    async fn send_data(&mut self, data: &[u8]) {
        self.dc.set_high();
        self.cs.set_low();
        let _ = self.spi.write(data).await;
        self.cs.set_high();
    }

    async fn send_command_data(&mut self, cmd: u8, data: &[u8]) {
        self.send_command(cmd).await;
        self.send_data(data).await;
    }

    async fn wait_busy(&mut self) {
        // BUSY HIGH = controller busy. Timeout after 5 seconds to prevent hang.
        let deadline = Instant::now() + Duration::from_secs(5);
        while self.busy.is_high() {
            if Instant::now() >= deadline {
                warn!("SSD1680 BUSY timeout (5s)");
                return;
            }
            Timer::after(Duration::from_millis(5)).await;
        }
    }

    async fn hw_reset(&mut self) {
        self.rst.set_high();
        Timer::after(Duration::from_millis(10)).await;
        self.rst.set_low();
        Timer::after(Duration::from_millis(10)).await;
        self.rst.set_high();
        Timer::after(Duration::from_millis(10)).await;
    }

    // ── Init / config ────────────────────────────────────────

    /// Full initialisation sequence — call after hw_reset or wake.
    pub async fn init(&mut self) {
        // SW reset
        self.send_command(CMD_SW_RESET).await;
        Timer::after(Duration::from_millis(10)).await;
        self.wait_busy().await;

        // Driver output control: HEIGHT-1 = 295 = 0x0127
        self.send_command_data(CMD_DRIVER_CONTROL, &[0x27, 0x01, 0x00])
            .await;

        // Data entry mode: X inc, Y inc
        self.send_command_data(CMD_DATA_MODE, &[0x03]).await;

        // RAM X address range: 0 .. (WIDTH/8 - 1) = 0 .. 15
        self.send_command_data(CMD_SET_RAM_X_POS, &[0x00, 0x0F])
            .await;

        // RAM Y address range: 0 .. HEIGHT-1 = 0 .. 295 (0x0127)
        self.send_command_data(CMD_SET_RAM_Y_POS, &[0x00, 0x00, 0x27, 0x01])
            .await;

        // Border waveform
        self.send_command_data(CMD_WRITE_BORDER, &[0x05]).await;

        // Use internal temperature sensor
        self.send_command_data(CMD_TEMP_CONTROL, &[0x80]).await;

        // VCOM voltage
        self.send_command_data(CMD_WRITE_VCOM, &[0x36]).await;

        // Load full-refresh LUT
        self.load_lut(&LUT_FULL).await;

        // Dummy line period
        self.send_command_data(CMD_WRITE_DUMMY, &[0x17]).await;

        // Gate line width
        self.send_command_data(CMD_WRITE_GATELINE, &[0x08]).await;

        info!("SSD1680 init done");
    }

    async fn load_lut(&mut self, lut: &[u8; 70]) {
        self.send_command(CMD_WRITE_LUT).await;
        self.send_data(lut).await;
    }

    async fn set_ram_pointer(&mut self, x: u8, y: u16) {
        self.send_command_data(CMD_SET_RAM_X_CNT, &[x]).await;
        self.send_command_data(CMD_SET_RAM_Y_CNT, &[y as u8, (y >> 8) as u8])
            .await;
    }

    // ── Public API ───────────────────────────────────────────

    /// Write full frame buffer and trigger display refresh.
    ///
    /// Uses 9+1 strategy: 9 partial refreshes, then 1 full refresh
    /// to clear accumulated ghosting.
    pub async fn display(&mut self, buf: &[u8; BUF_SIZE]) {
        // Choose LUT: every 10th frame is full
        let use_full = self.partial_count >= PARTIAL_MAX;
        if use_full {
            self.load_lut(&LUT_FULL).await;
            self.partial_count = 0;
        } else {
            self.load_lut(&LUT_PARTIAL).await;
            self.partial_count += 1;
        }

        // Set RAM pointer to origin
        self.set_ram_pointer(0, 0).await;

        // Write B/W RAM
        self.send_command(CMD_WRITE_RAM_BW).await;
        self.send_data(buf).await;

        // Display update control 2: load temp + display
        let ctrl2 = if use_full { 0xC7 } else { 0xCF };
        self.send_command_data(CMD_DISP_CTRL2, &[ctrl2]).await;

        // Master activate
        self.send_command(CMD_MASTER_ACTIVATE).await;
        self.wait_busy().await;
    }

    /// Force the next `display()` call to use full-refresh LUT.
    #[allow(dead_code)]
    pub fn force_full_refresh(&mut self) {
        self.partial_count = PARTIAL_MAX;
    }

    /// Enter deep-sleep mode 1 to minimise power consumption.
    pub async fn sleep(&mut self) {
        self.send_command_data(CMD_DEEP_SLEEP, &[0x01]).await;
    }

    /// Wake from deep sleep by hardware-resetting and re-initialising.
    pub async fn wake(&mut self) {
        self.hw_reset().await;
        self.init().await;
    }
}
