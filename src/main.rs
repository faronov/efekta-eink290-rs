//! # Efekta E-Ink 290 Multi-Sensor — Rust/Zigbee-RS Firmware
//!
//! Rust port of the Efekta EINK290 nRF52840 Zigbee multi-sensor.
//! Uses [zigbee-rs](https://github.com/faronov/zigbee-rs) stack
//! instead of NCS/ZBOSS.
//!
//! ## Hardware (Efekta EBYTE2 custom PCB)
//! - nRF52840 SoC
//! - BME280 (I2C 0x77): temperature, humidity, pressure
//! - MAX44009 (I2C 0x4A): ambient light / illuminance
//! - 2.9" e-paper display (SSD1680 via SPI)
//! - Buzzer/speaker (P0.30, PWM)
//! - Button (P0.24, active low, internal pull-up)
//! - LED (P0.02, active low)
//! - 2×AAA NiMH battery
//!
//! ## Pin Map (EBYTE2 variant from variant.h)
//! - I2C: SDA=P0.28, SCL=P0.03
//! - SPI: MOSI=P0.15, SCK=P0.20, CS=P0.22 (MISO=P0.21 NC)
//! - E-paper: RST=P0.17, DC=P0.31, BUSY=P0.13
//! - UART: TX=P0.09, RX=P0.10 (NFC pins, reconfigured as GPIO)
//! - Buzzer: P0.30
//! - LED: P0.02, Button: P0.24
//!
//! ## ZCL Clusters (Endpoint 1, HA Profile 0x0104)
//! - 0x0000 Basic (server)
//! - 0x0003 Identify (server)
//! - 0x0001 Power Configuration (server)
//! - 0x0402 Temperature Measurement (server)
//! - 0x0405 Relative Humidity Measurement (server)
//! - 0x0403 Pressure Measurement (server)
//! - 0x0400 Illuminance Measurement (server)
//!
//! ## Operation (Aqara-style)
//! - Power on → starts idle
//! - Short press → join network (if not joined) / force report (if joined)
//! - Long press (5s) → factory reset + enter pairing mode
//! - LED fast blink → joining network
//! - LED off → normal operation (battery saving)
//! - Sensors read every 60s, values reported via ZCL attribute reporting
//!
//! ## Build variants
//! - Default: release firmware with e-paper display, RTT logging (needs SWD probe)
//! - `--features uart-debug`: UART logging on TX=P0.09 (115200), no e-paper display

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{select3, Either3};
use embassy_nrf::nvmc::Nvmc;
use embassy_nrf::saadc::{self, Saadc};
#[cfg(not(feature = "uart-debug"))]
use embassy_nrf::spim;
use embassy_nrf::twim::{self, Twim};
#[cfg(feature = "uart-debug")]
use embassy_nrf::uarte;
use embassy_nrf::{self as _, bind_interrupts, gpio, peripherals, radio};
use embassy_time::{Duration, Instant, Timer};

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::{StackEvent, TickResult};
use zigbee_runtime::ota::{OtaConfig, OtaManager};
use zigbee_runtime::{ClusterRef, UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::basic::BasicCluster;
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::identify::IdentifyCluster;
use zigbee_zcl::clusters::illuminance::IlluminanceCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::pressure::PressureCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

mod bme280;
#[cfg_attr(feature = "uart-debug", allow(dead_code))]
mod display;
mod max44009;
mod ota;
#[cfg_attr(feature = "uart-debug", allow(dead_code))]
mod paint;
#[cfg_attr(feature = "uart-debug", allow(dead_code))]
mod ssd1680;

const REPORT_INTERVAL_SECS: u64 = 60;
const LONG_PRESS_MS: u64 = 5000;
const BME280_ADDR: u8 = 0x77;
const MAX44009_ADDR: u8 = 0x4A;

/// 2×AAA NiMH battery voltage scaling constants.
/// nRF52840 SAADC with VDD/3 internal channel, 10-bit, 0.6V reference.
/// V_battery = raw * 3 * 0.6 / 1024
const BATTERY_SCALE_MV_NUM: u32 = 1800; // 3 * 600
const BATTERY_SCALE_MV_DEN: u32 = 1024;

/// Piecewise-linear battery percentage for 2×AAA NiMH cells.
/// Input: total voltage in mV (both cells in series).
/// Returns ZCL value: 0..200 (0.5% units, i.e. 200 = 100%).
fn battery_pct_nimh_2s(mv: u32) -> u8 {
    let p: u32 = if mv >= 2700 {
        100 // freshly charged / no load
    } else if mv > 2500 {
        80 + (mv - 2500) * 20 / 200
    } else if mv > 2400 {
        50 + (mv - 2400) * 30 / 100 // flat working plateau
    } else if mv > 2200 {
        10 + (mv - 2200) * 40 / 200
    } else if mv > 2000 {
        (mv - 2000) * 10 / 200 // tail end
    } else {
        0
    };
    (p * 2) as u8 // ZCL: 0..200 (0.5%)
}

#[cfg(not(feature = "uart-debug"))]
bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
    TWISPI1 => spim::InterruptHandler<peripherals::TWISPI1>;
    SAADC => saadc::InterruptHandler;
});

#[cfg(feature = "uart-debug")]
bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
    UARTE1 => uarte::InterruptHandler<peripherals::UARTE1>;
    SAADC => saadc::InterruptHandler;
});

#[derive(defmt::Format, Clone, Copy, PartialEq)]
enum NetworkState {
    Initial,
    Joining,
    Joined,
    JoinFailedSleep,
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    info!("Efekta E-Ink 290 multi-sensor starting (zigbee-rs)…");

    // ── GPIO ────────────────────────────────────────────────
    let mut led = gpio::Output::new(p.P0_02, gpio::Level::High, gpio::OutputDrive::Standard);
    let mut button = gpio::Input::new(p.P0_24, gpio::Pull::Up);

    // ── I2C bus (BME280 + MAX44009) — SDA=P0.28, SCL=P0.03 ──
    let mut i2c_config = twim::Config::default();
    i2c_config.frequency = twim::Frequency::K400;
    let mut i2c = Twim::new(p.TWISPI0, Irqs, p.P0_28, p.P0_03, i2c_config);

    // ── SAADC for battery voltage ───────────────────────────
    let saadc_config = saadc::Config::default();
    let channel_config = saadc::ChannelConfig::single_ended(saadc::VddInput);
    let mut adc = Saadc::new(p.SAADC, Irqs, saadc_config, [channel_config]);

    // ── Buzzer (P0.30) — software bit-bang tone ─────────────
    let mut buzzer = gpio::Output::new(p.P0_30, gpio::Level::Low, gpio::OutputDrive::Standard);

    // ── UART debug mode: TX=P0.09, RX=P0.10 at 115200 ──────
    #[cfg(feature = "uart-debug")]
    let mut uart = {
        let mut config = uarte::Config::default();
        config.baudrate = uarte::Baudrate::BAUD115200;
        // UARTE1 shares hardware with TWISPI1 (no SPI display in this mode)
        uarte::UarteTx::new(p.UARTE1, Irqs, p.P0_09, config)
    };
    #[cfg(feature = "uart-debug")]
    {
        use embedded_io_async::Write;
        let _ = uart.write_all(b"\r\n=== Efekta E-Ink 290 UART debug ===\r\n").await;
        info!("UART debug active on TX=P0.09 115200");
    }

    // ── SPI1 for e-paper display (SSD1680) — only in normal mode ─
    #[cfg(not(feature = "uart-debug"))]
    let mut epd = {
        let mut spi_config = spim::Config::default();
        spi_config.frequency = spim::Frequency::M2;
        // EBYTE2: MOSI=P0.15, SCK=P0.20, MISO=P0.21(NC)
        let spi = spim::Spim::new(p.TWISPI1, Irqs, p.P0_20, p.P0_15, p.P0_21, spi_config);
        let cs = gpio::Output::new(p.P0_22, gpio::Level::High, gpio::OutputDrive::Standard);
        let dc = gpio::Output::new(p.P0_31, gpio::Level::Low, gpio::OutputDrive::Standard);
        let rst = gpio::Output::new(p.P0_17, gpio::Level::High, gpio::OutputDrive::Standard);
        let busy = gpio::Input::new(p.P0_13, gpio::Pull::None);
        ssd1680::Ssd1680::new(spi, cs, dc, rst, busy).await
    };
    #[cfg(not(feature = "uart-debug"))]
    let mut epd_paint = paint::Paint::new();
    #[cfg(not(feature = "uart-debug"))]
    info!("E-paper display ready");

    // ── Initialize sensors ──────────────────────────────────
    let bme280_ok = bme280::init(&mut i2c, BME280_ADDR).await;
    if bme280_ok {
        info!("BME280 ready");
    } else {
        warn!("BME280 not found");
    }

    let max44009_ok = max44009::init(&mut i2c, MAX44009_ADDR).await;
    if max44009_ok {
        info!("MAX44009 ready");
    } else {
        warn!("MAX44009 not found");
    }

    // ── 802.15.4 radio ──────────────────────────────────────
    let radio = radio::ieee802154::Radio::new(p.RADIO, Irqs);
    let mut mac = zigbee_mac::nrf::NrfMac::new(radio);
    mac.set_tx_power(4); // +4 dBm (nRF52840 supports -40 to +8)
    info!("Radio ready (+4 dBm)");

    // ── OTA Manager (flash writer + ZCL cluster) ──────────
    let nvmc = Nvmc::new(p.NVMC);
    let fw_writer = ota::NrfFirmwareWriter::new(nvmc);
    let ota_config = OtaConfig {
        manufacturer_code: 0x1234,
        image_type: 0x0001,
        current_version: env!("OTA_VERSION_U32").parse().unwrap(),
        endpoint: 1,
        block_size: 48,
        auto_accept: true,
    };
    let mut ota_mgr = OtaManager::new(fw_writer, ota_config);

    // ── ZCL cluster instances ───────────────────────────────
    let mut basic_cluster = BasicCluster::new(
        b"Efekta",
        b"EInk290-MultiSensor",
        env!("BUILD_DATE").as_bytes(),
        concat!(env!("CARGO_PKG_VERSION"), "-rs").as_bytes(),
    );
    let mut identify_cluster = IdentifyCluster::new();
    let mut temp_cluster = TemperatureCluster::new(-4000, 12500);
    let mut hum_cluster = HumidityCluster::new(0, 10000);
    let mut press_cluster = PressureCluster::new(3000, 11000); // 300.0–1100.0 hPa
    let mut illum_cluster = IlluminanceCluster::new(1, 50000);
    let mut power_cluster = PowerConfigCluster::new();
    // 2×AAA NiMH: size=AAA(4), quantity=2, rated=1.2V(12), min threshold=2.0V(20)
    power_cluster.set_battery_size(4); // AAA
    power_cluster.set_battery_quantity(2);
    power_cluster.set_battery_rated_voltage(12); // 1.2V per cell
    power_cluster.set_battery_voltage_min_threshold(20); // 2.0V total cutoff

    // ── Zigbee device: endpoint 1 with 8 server clusters ────
    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .manufacturer("Efekta")
        .model("EInk290-MultiSensor")
        .sw_build(concat!(env!("CARGO_PKG_VERSION"), "-rs"))
        .date_code(env!("BUILD_DATE"))
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
            ep.cluster_server(0x0000) // Basic
                .cluster_server(0x0003) // Identify
                .cluster_server(0x0001) // Power Configuration
                .cluster_server(0x0402) // Temperature Measurement
                .cluster_server(0x0405) // Relative Humidity
                .cluster_server(0x0403) // Pressure Measurement
                .cluster_server(0x0400) // Illuminance Measurement
                .cluster_client(0x0019) // OTA Upgrade (client role)
        })
        .build();

    // ── Default reporting thresholds ────────────────────────
    // These are initial defaults; coordinator can override via Configure Reporting (0x06).
    {
        use zigbee_zcl::foundation::reporting::{ReportDirection, ReportingConfig};
        use zigbee_zcl::data_types::{ZclDataType, ZclValue};
        use zigbee_zcl::AttributeId;

        let rpt = device.reporting_mut();

        // Temperature: report on ±0.25°C change (25 centidegrees), 30-900s
        let _ = rpt.configure_for_cluster(1, 0x0402, ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: AttributeId(0x0000),
            data_type: ZclDataType::I16,
            min_interval: 30, max_interval: 900,
            reportable_change: Some(ZclValue::I16(25)),
        });
        // Humidity: report on ±0.5% change (50 centipercent), 30-1200s
        let _ = rpt.configure_for_cluster(1, 0x0405, ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: AttributeId(0x0000),
            data_type: ZclDataType::U16,
            min_interval: 30, max_interval: 1200,
            reportable_change: Some(ZclValue::U16(50)),
        });
        // Pressure: report on ±1 hPa change (1 in hPa units), 60-1800s
        let _ = rpt.configure_for_cluster(1, 0x0403, ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: AttributeId(0x0000),
            data_type: ZclDataType::I16,
            min_interval: 60, max_interval: 1800,
            reportable_change: Some(ZclValue::I16(1)),
        });
        // Illuminance: report on ~50 lux change (5000 ZCL units), 60-300s
        let _ = rpt.configure_for_cluster(1, 0x0400, ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: AttributeId(0x0000),
            data_type: ZclDataType::U16,
            min_interval: 60, max_interval: 300,
            reportable_change: Some(ZclValue::U16(5000)),
        });
        // Battery %: report on ±1% (2 in 0-200 range), 300-3600s
        let _ = rpt.configure_for_cluster(1, 0x0001, ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: AttributeId(0x0021),
            data_type: ZclDataType::U8,
            min_interval: 300, max_interval: 3600,
            reportable_change: Some(ZclValue::U8(2)),
        });
        // Battery voltage: report on ±100mV, 300-3600s
        let _ = rpt.configure_for_cluster(1, 0x0001, ReportingConfig {
            direction: ReportDirection::Send,
            attribute_id: AttributeId(0x0020),
            data_type: ZclDataType::U8,
            min_interval: 300, max_interval: 3600,
            reportable_change: Some(ZclValue::U8(1)),
        });
    }

    let mut net_state = NetworkState::Initial;
    let mut button_press_start: Option<Instant> = None;
    let mut ota_query_countdown: u32 = 0; // seconds until next OTA query
    const OTA_QUERY_INTERVAL: u32 = 86400; // 24 hours

    info!("Ready — short press to join, long press (5s) for factory reset");

    // ── Main event loop ─────────────────────────────────────
    loop {
        match select3(
            device.receive(),
            button.wait_for_any_edge(),
            Timer::after(Duration::from_secs(REPORT_INTERVAL_SECS)),
        )
        .await
        {
            // ── Incoming MAC frame ──────────────────────────
            Either3::First(Ok(indication)) => {
                let mut clusters = [
                    ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut press_cluster },
                    ClusterRef { endpoint: 1, cluster: &mut illum_cluster },
                ];
                if let Some(event) = device.process_incoming(&indication, &mut clusters).await {
                    // Check identify state change after command dispatch
                    if identify_cluster.is_identifying() {
                        buzzer_chirp(&mut buzzer);
                    }

                    // ── OTA: route cluster 0x0019 commands to OtaManager ──
                    if let StackEvent::CommandReceived { cluster_id: 0x0019, command_id, ref payload, .. } = event {
                        if let Some(evt) = ota_mgr.handle_incoming(command_id, payload.as_slice()) {
                            handle_ota_event(&evt);
                        }
                        // Immediately send any OTA response frame (block request, etc.)
                        send_ota_pending(&mut device, &mut ota_mgr).await;
                        // OTA tick for pending block writes → next block request
                        if let Some(evt) = ota_mgr.tick(0) {
                            handle_ota_event(&evt);
                        }
                        send_ota_pending(&mut device, &mut ota_mgr).await;
                    }

                    handle_stack_event(&event, &mut net_state, &mut led, &mut buzzer);
                    // Start OTA check shortly after joining
                    if matches!(event, StackEvent::Joined { .. }) {
                        ota_query_countdown = 30; // query in 30 seconds
                    }
                }
            }
            Either3::First(Err(_)) => warn!("MAC receive error"),

            // ── Button edge ─────────────────────────────────
            Either3::Second(_) => {
                if button.is_low() {
                    button_press_start = Some(Instant::now());
                } else if let Some(start) = button_press_start.take() {
                    let press_ms = start.elapsed().as_millis();

                    if press_ms >= LONG_PRESS_MS {
                        info!("Long press → factory reset + pair");
                        led_flash(&mut led, 5).await;
                        net_state = NetworkState::Joining;
                        device.user_action(UserAction::Toggle);
                        // second toggle re-starts commissioning
                        device.user_action(UserAction::Toggle);
                    } else if press_ms >= 50 {
                        match net_state {
                            NetworkState::Joined => {
                                info!("Short press → force report");
                                led_flash(&mut led, 3).await;
                                let mut disp_data = read_sensors(
                                    &mut i2c,
                                    &mut adc,
                                    &mut temp_cluster,
                                    &mut hum_cluster,
                                    &mut press_cluster,
                                    &mut illum_cluster,
                                    &mut power_cluster,
                                    bme280_ok,
                                    max44009_ok,
                                )
                                .await;
                                // Force send reports for all clusters
                                {
                                    let mut clusters = [
                                        ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                                        ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
                                        ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                                        ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                                        ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                                        ClusterRef { endpoint: 1, cluster: &mut press_cluster },
                                        ClusterRef { endpoint: 1, cluster: &mut illum_cluster },
                                    ];
                                    if let TickResult::Event(ref e) =
                                        device.tick(0, &mut clusters).await
                                    {
                                        handle_stack_event(e, &mut net_state, &mut led, &mut buzzer);
                                    }
                                }

                                // Update e-paper display
                                disp_data.joined = net_state == NetworkState::Joined;
                                #[cfg(not(feature = "uart-debug"))]
                                {
                                    epd.wake().await;
                                    display::draw_dashboard(&mut epd_paint, &disp_data);
                                    epd.display(&epd_paint.buf).await;
                                    epd.sleep().await;
                                }
                                #[cfg(feature = "uart-debug")]
                                uart_log_sensors(&mut uart, &disp_data).await;
                            }
                            NetworkState::Joining => {
                                info!("Already joining…");
                                led_flash(&mut led, 1).await;
                            }
                            _ => {
                                info!("Short press → join");
                                net_state = NetworkState::Joining;
                                led.set_low();
                                device.user_action(UserAction::Toggle);
                            }
                        }
                    }
                    Timer::after(Duration::from_millis(200)).await;
                }

                {
                    let mut clusters = [
                        ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut press_cluster },
                        ClusterRef { endpoint: 1, cluster: &mut illum_cluster },
                    ];
                    if let TickResult::Event(ref e) = device.tick(0, &mut clusters).await {
                        handle_stack_event(e, &mut net_state, &mut led, &mut buzzer);
                    }
                }
            }

            // ── Periodic sensor report ──────────────────────
            Either3::Third(_) => {
                // Tick identify cluster countdown
                let was_identifying = identify_cluster.is_identifying();
                identify_cluster.tick(REPORT_INTERVAL_SECS as u16);
                if identify_cluster.is_identifying() {
                    buzzer_chirp(&mut buzzer);
                    led_flash(&mut led, 2).await;
                } else if was_identifying {
                    info!("Identify complete");
                }

                if device.is_joined() {
                    // Read sensors and update cluster attributes
                    let mut disp_data = read_sensors(
                        &mut i2c,
                        &mut adc,
                        &mut temp_cluster,
                        &mut hum_cluster,
                        &mut press_cluster,
                        &mut illum_cluster,
                        &mut power_cluster,
                        bme280_ok,
                        max44009_ok,
                    )
                    .await;

                    // Tick reporting engine — automatically sends due reports
                    {
                        let mut clusters = [
                            ClusterRef { endpoint: 1, cluster: &mut basic_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut identify_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut power_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut temp_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut hum_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut press_cluster },
                            ClusterRef { endpoint: 1, cluster: &mut illum_cluster },
                        ];
                        if let TickResult::Event(ref e) =
                            device.tick(REPORT_INTERVAL_SECS as u16, &mut clusters).await
                        {
                            handle_stack_event(e, &mut net_state, &mut led, &mut buzzer);
                        }
                    }

                    // Update e-paper display
                    disp_data.joined = true;
                    #[cfg(not(feature = "uart-debug"))]
                    {
                        epd.wake().await;
                        display::draw_dashboard(&mut epd_paint, &disp_data);
                        epd.display(&epd_paint.buf).await;
                        epd.sleep().await;
                    }
                    #[cfg(feature = "uart-debug")]
                    uart_log_sensors(&mut uart, &disp_data).await;

                    // ── OTA: periodic query + tick ───────────────
                    let ota_elapsed = REPORT_INTERVAL_SECS as u16;
                    if let Some(evt) = ota_mgr.tick(ota_elapsed) {
                        handle_ota_event(&evt);
                    }
                    send_ota_pending(&mut device, &mut ota_mgr).await;

                    // Check if it's time to query for new firmware
                    if ota_query_countdown <= REPORT_INTERVAL_SECS as u32 {
                        if let Some(evt) = ota_mgr.start_query() {
                            handle_ota_event(&evt);
                        }
                        send_ota_pending(&mut device, &mut ota_mgr).await;
                        ota_query_countdown = OTA_QUERY_INTERVAL;
                    } else {
                        ota_query_countdown -= REPORT_INTERVAL_SECS as u32;
                    }
                }
            }
        }
    }
}

/// Read all sensors and update ZCL cluster attribute values.
/// Returns display data for the e-paper screen.
#[allow(clippy::too_many_arguments)]
async fn read_sensors(
    i2c: &mut Twim<'_, peripherals::TWISPI0>,
    adc: &mut Saadc<'_, 1>,
    temp_cluster: &mut TemperatureCluster,
    hum_cluster: &mut HumidityCluster,
    press_cluster: &mut PressureCluster,
    illum_cluster: &mut IlluminanceCluster,
    power_cluster: &mut PowerConfigCluster,
    bme280_ok: bool,
    max44009_ok: bool,
) -> display::DisplayData {
    // ── Battery voltage via SAADC (VDD/3 internal) ──────
    let mut buf = [0i16; 1];
    adc.sample(&mut buf).await;
    let raw = buf[0].max(0) as u32;
    let battery_mv = (raw * BATTERY_SCALE_MV_NUM) / BATTERY_SCALE_MV_DEN;

    // ZCL PowerConfig: BatteryVoltage in units of 100mV
    let batt_voltage_zcl = (battery_mv / 100) as u8;
    // Percentage: 2×AAA NiMH piecewise-linear curve
    // Returns ZCL 0..200 (0.5% units)
    let batt_pct = battery_pct_nimh_2s(battery_mv);
    power_cluster.set_battery_voltage(batt_voltage_zcl);
    power_cluster.set_battery_percentage(batt_pct);
    info!("Battery: {}mV ({}%)", battery_mv, batt_pct / 2);

    let mut disp = display::DisplayData {
        temperature_centideg: 0,
        humidity_centipct: 0,
        pressure_hpa: 0,
        lux: 0,
        battery_pct: batt_pct / 2,
        battery_mv: battery_mv as u16,
        joined: false, // caller sets this
        pressure_history: [0u16; 24],
        pressure_history_len: 0,
        time_hour: 0,
        time_minute: 0,
        date_day: 0,
        date_month: 0,
        prev_temperature_centideg: 0, // TODO: persist across cycles
        colon_on: true,               // TODO: toggle each cycle
    };

    // ── BME280: temperature, humidity, pressure ─────────
    if bme280_ok {
        if let Some(data) = bme280::read(i2c, BME280_ADDR).await {
            info!(
                "T={}.{:02}°C H={}.{:02}% P={}hPa",
                data.temperature_centideg / 100,
                (data.temperature_centideg % 100).unsigned_abs(),
                data.humidity_centipct / 100,
                data.humidity_centipct % 100,
                data.pressure_hpa,
            );
            temp_cluster.set_temperature(data.temperature_centideg);
            hum_cluster.set_humidity(data.humidity_centipct);
            press_cluster.set_pressure(data.pressure_hpa as i16);

            disp.temperature_centideg = data.temperature_centideg;
            disp.humidity_centipct = data.humidity_centipct;
            disp.pressure_hpa = data.pressure_hpa;
        } else {
            warn!("BME280 read failed");
        }
    }

    // ── MAX44009: illuminance ───────────────────────────
    if max44009_ok {
        if let Some(lux) = max44009::read_lux(i2c, MAX44009_ADDR).await {
            // ZCL illuminance: 10000 * log10(lux) + 1, approx via log2
            let illum_zcl = if lux == 0 {
                0u16
            } else {
                let log2 = 31 - (lux as u32).leading_zeros();
                let log10_x10000 = log2 * 3010;
                (log10_x10000 + 1) as u16
            };
            info!("{} lux → ZCL {}", lux, illum_zcl);
            illum_cluster.set_illuminance(illum_zcl);
            disp.lux = lux;
        } else {
            warn!("MAX44009 read failed");
        }
    }

    disp
}

/// Send any queued OTA frame to the coordinator.
async fn send_ota_pending<M: zigbee_mac::MacDriver, F: zigbee_runtime::firmware_writer::FirmwareWriter>(
    device: &mut ZigbeeDevice<M>,
    ota_mgr: &mut OtaManager<F>,
) {
    while let Some(frame) = ota_mgr.take_pending_frame() {
        let _ = device
            .send_zcl_frame(
                zigbee_types::ShortAddress(0x0000), // coordinator
                frame.endpoint,
                frame.endpoint,
                frame.cluster_id,
                &frame.zcl_data,
            )
            .await;
    }
}

/// Look up the attribute store (read-only) for a given cluster ID.
fn handle_stack_event(
    event: &StackEvent,
    net_state: &mut NetworkState,
    led: &mut gpio::Output<'_>,
    buzzer: &mut gpio::Output<'_>,
) {
    match event {
        StackEvent::Joined {
            short_address,
            channel,
            pan_id,
        } => {
            info!(
                "Joined! 0x{:04X} ch={} pan=0x{:04X}",
                short_address, channel, pan_id
            );
            *net_state = NetworkState::Joined;
            led.set_high(); // LED off (active low)
            buzzer_chirp(buzzer);
        }
        StackEvent::Left => {
            info!("Left network");
            *net_state = NetworkState::Initial;
        }
        StackEvent::CommissioningComplete { success } => {
            if *success {
                info!("Commissioning OK");
                *net_state = NetworkState::Joined;
            } else {
                warn!("Commissioning failed");
                *net_state = NetworkState::JoinFailedSleep;
            }
            led.set_high();
        }
        StackEvent::ReportSent => info!("Report sent"),
        _ => info!("Stack event"),
    }
}

/// Handle OTA-related stack events from the OtaManager.
fn handle_ota_event(event: &StackEvent) {
    match event {
        StackEvent::OtaProgress { percent } => {
            info!("OTA: {}% downloaded", percent);
        }
        StackEvent::OtaComplete => {
            info!("OTA: complete — rebooting!");
            cortex_m::peripheral::SCB::sys_reset();
        }
        StackEvent::OtaFailed => {
            error!("OTA: failed");
        }
        _ => {}
    }
}

async fn led_flash(led: &mut gpio::Output<'_>, count: u8) {
    for i in 0..count {
        led.set_low();
        Timer::after(Duration::from_millis(100)).await;
        led.set_high();
        if i < count - 1 {
            Timer::after(Duration::from_millis(100)).await;
        }
    }
}

/// Buzzer "join confirmed" chirp: 400 Hz for 10 ms, then 200 Hz for 5 ms.
/// Uses busy-wait bit-bang, matching the original Arduino firmware.
fn buzzer_chirp(pin: &mut gpio::Output<'_>) {
    buzzer_tone(pin, 400, 10);
    buzzer_tone(pin, 200, 5);
}

/// Bit-bang a square wave on the buzzer pin.
/// `freq_hz`: tone frequency, `duration_ms`: how long to play.
/// Uses cortex-m busy-wait at 64 MHz (nRF52840 HFCLK).
fn buzzer_tone(pin: &mut gpio::Output<'_>, freq_hz: u32, duration_ms: u32) {
    let half_period_us = 500_000 / freq_hz;
    // 64 MHz → 64 cycles per µs
    let half_period_cycles = half_period_us * 64;
    let total_cycles = duration_ms * 1000 * 64;
    let mut elapsed = 0u32;
    while elapsed < total_cycles {
        pin.set_high();
        cortex_m::asm::delay(half_period_cycles);
        pin.set_low();
        cortex_m::asm::delay(half_period_cycles);
        elapsed += half_period_cycles * 2;
    }
}

/// UART debug: print sensor readings as plain text over serial.
/// Format is human-readable ASCII at 115200 baud.
#[cfg(feature = "uart-debug")]
async fn uart_log_sensors(
    uart: &mut uarte::UarteTx<'_, peripherals::UARTE1>,
    data: &display::DisplayData,
) {
    use embedded_io_async::Write;
    let mut buf = [0u8; 200];
    let neg = data.temperature_centideg < 0;
    let abs_t = if neg {
        (-(data.temperature_centideg as i32)) as u16
    } else {
        data.temperature_centideg as u16
    };
    let len = fmt_uart(
        &mut buf,
        neg,
        abs_t / 100,
        (abs_t % 100) / 10,
        data.humidity_centipct / 100,
        (data.humidity_centipct % 100) / 10,
        data.pressure_hpa,
        data.lux,
        data.battery_mv,
        data.battery_pct,
        data.joined,
    );
    let _ = uart.write_all(&buf[..len]).await;
}

/// Format sensor data into a plain text line (no alloc, no core::fmt).
/// Returns number of bytes written.
#[cfg(feature = "uart-debug")]
#[allow(clippy::too_many_arguments)]
fn fmt_uart(
    buf: &mut [u8],
    neg: bool,
    t_int: u16,
    t_frac: u16,
    h_int: u16,
    h_frac: u16,
    press: u16,
    lux: u16,
    bat_mv: u16,
    bat_pct: u8,
    joined: bool,
) -> usize {
    let mut pos = 0usize;
    pos = put_str(buf, pos, b"T=");
    if neg {
        pos = put_byte(buf, pos, b'-');
    }
    pos = put_u16(buf, pos, t_int);
    pos = put_byte(buf, pos, b'.');
    pos = put_u16(buf, pos, t_frac);
    pos = put_str(buf, pos, b"C H=");
    pos = put_u16(buf, pos, h_int);
    pos = put_byte(buf, pos, b'.');
    pos = put_u16(buf, pos, h_frac);
    pos = put_str(buf, pos, b"% P=");
    pos = put_u16(buf, pos, press);
    pos = put_str(buf, pos, b"hPa L=");
    pos = put_u16(buf, pos, lux);
    pos = put_str(buf, pos, b"lux B=");
    pos = put_u16(buf, pos, bat_mv);
    pos = put_str(buf, pos, b"mV(");
    pos = put_u16(buf, pos, bat_pct as u16);
    pos = put_str(buf, pos, b"%) ZB=");
    pos = put_str(buf, pos, if joined { b"joined" } else { b"offline" });
    pos = put_str(buf, pos, b"\r\n");
    pos
}

#[cfg(feature = "uart-debug")]
fn put_byte(buf: &mut [u8], pos: usize, b: u8) -> usize {
    if pos < buf.len() {
        buf[pos] = b;
        pos + 1
    } else {
        pos
    }
}

#[cfg(feature = "uart-debug")]
fn put_str(buf: &mut [u8], mut pos: usize, s: &[u8]) -> usize {
    for &b in s {
        if pos < buf.len() {
            buf[pos] = b;
            pos += 1;
        }
    }
    pos
}

#[cfg(feature = "uart-debug")]
fn put_u16(buf: &mut [u8], pos: usize, mut v: u16) -> usize {
    if v == 0 {
        return put_byte(buf, pos, b'0');
    }
    let mut d = [0u8; 5];
    let mut n = 0usize;
    while v > 0 {
        d[n] = b'0' + (v % 10) as u8;
        v /= 10;
        n += 1;
    }
    let mut p = pos;
    while n > 0 {
        n -= 1;
        p = put_byte(buf, p, d[n]);
    }
    p
}
