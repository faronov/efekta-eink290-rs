//! # Efekta E-Ink 290 Multi-Sensor — Rust/Zigbee-RS Firmware
//!
//! Rust port of the Efekta EINK290 nRF52840 Zigbee multi-sensor.
//! Uses [zigbee-rs](https://github.com/faronov/zigbee-rs) stack
//! instead of NCS/ZBOSS.
//!
//! ## Hardware (Efekta custom PCB)
//! - nRF52840 SoC
//! - BME280 (I2C 0x77): temperature, humidity, pressure
//! - MAX44009 (I2C 0x4A): ambient light / illuminance
//! - 2.9" e-paper display (SSD1680 or UC8151 via SPI) — future
//! - Button (P0.24, active low, internal pull-up)
//! - LED (P0.02, active low)
//! - CR2032 battery
//!
//! ## Pin Map (from Efekta DTS)
//! - I2C0 SDA: P0.30
//! - I2C0 SCL: P0.31
//! - LED0: P0.02 (active low)
//! - Button0: P0.24 (active low, pull-up)
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

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_futures::select::{select3, Either3};
use embassy_nrf::saadc::{self, Saadc};
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{self as _, bind_interrupts, gpio, peripherals, radio};
use embassy_time::{Duration, Instant, Timer};

use defmt::*;
use {defmt_rtt as _, panic_probe as _};

use zigbee_aps::PROFILE_HOME_AUTOMATION;
use zigbee_nwk::DeviceType;
use zigbee_runtime::event_loop::{StackEvent, TickResult};
use zigbee_runtime::{UserAction, ZigbeeDevice};
use zigbee_zcl::clusters::humidity::HumidityCluster;
use zigbee_zcl::clusters::illuminance::IlluminanceCluster;
use zigbee_zcl::clusters::power_config::PowerConfigCluster;
use zigbee_zcl::clusters::pressure::PressureCluster;
use zigbee_zcl::clusters::temperature::TemperatureCluster;

mod bme280;
mod max44009;

const REPORT_INTERVAL_SECS: u64 = 60;
const LONG_PRESS_MS: u64 = 5000;
const BME280_ADDR: u8 = 0x77;
const MAX44009_ADDR: u8 = 0x4A;

/// CR2032 battery voltage divider constants.
/// nRF52840 SAADC with VDD/3 internal channel, 10-bit, 0.6V reference.
/// V_battery = raw * 3 * 0.6 / 1024
const BATTERY_SCALE_MV_NUM: u32 = 1800; // 3 * 600
const BATTERY_SCALE_MV_DEN: u32 = 1024;

bind_interrupts!(struct Irqs {
    RADIO => radio::InterruptHandler<peripherals::RADIO>;
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
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

    // ── I2C bus (BME280 + MAX44009) ─────────────────────────
    let mut i2c_config = twim::Config::default();
    i2c_config.frequency = twim::Frequency::K400;
    let mut i2c = Twim::new(p.TWISPI0, Irqs, p.P0_30, p.P0_31, i2c_config);

    // ── SAADC for battery voltage ───────────────────────────
    let saadc_config = saadc::Config::default();
    let channel_config = saadc::ChannelConfig::single_ended(saadc::VddInput);
    let mut adc = Saadc::new(p.SAADC, Irqs, saadc_config, [channel_config]);

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
    let mac = zigbee_mac::nrf::NrfMac::new(radio);
    info!("Radio ready");

    // ── ZCL cluster instances ───────────────────────────────
    let mut temp_cluster = TemperatureCluster::new(-4000, 12500);
    let mut hum_cluster = HumidityCluster::new(0, 10000);
    let mut press_cluster = PressureCluster::new(3000, 11000); // 300.0–1100.0 hPa
    let mut illum_cluster = IlluminanceCluster::new(1, 50000);
    let mut power_cluster = PowerConfigCluster::new();

    // ── Zigbee device: endpoint 1 with 7 server clusters ────
    let mut device = ZigbeeDevice::builder(mac)
        .device_type(DeviceType::EndDevice)
        .manufacturer("Efekta")
        .model("EInk290-MultiSensor")
        .sw_build("0.1.0-rs")
        .channels(zigbee_types::ChannelMask::ALL_2_4GHZ)
        .endpoint(1, PROFILE_HOME_AUTOMATION, 0x0302, |ep| {
            ep.cluster_server(0x0000) // Basic
                .cluster_server(0x0003) // Identify
                .cluster_server(0x0001) // Power Configuration
                .cluster_server(0x0402) // Temperature Measurement
                .cluster_server(0x0405) // Relative Humidity
                .cluster_server(0x0403) // Pressure Measurement
                .cluster_server(0x0400) // Illuminance Measurement
        })
        .build();

    let mut net_state = NetworkState::Initial;
    let mut button_press_start: Option<Instant> = None;

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
                if let Some(event) = device.process_incoming(&indication) {
                    handle_stack_event(&event, &mut net_state, &mut led);
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
                                report_sensors(
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

                if let TickResult::Event(ref e) = device.tick(0).await {
                    handle_stack_event(e, &mut net_state, &mut led);
                }
            }

            // ── Periodic sensor report ──────────────────────
            Either3::Third(_) => {
                if device.is_joined() {
                    report_sensors(
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
                }
                if let TickResult::Event(ref e) = device.tick(REPORT_INTERVAL_SECS as u16).await {
                    handle_stack_event(e, &mut net_state, &mut led);
                }
            }
        }
    }
}

/// Read all sensors and update ZCL cluster attribute values.
#[allow(clippy::too_many_arguments)]
async fn report_sensors(
    i2c: &mut Twim<'_, peripherals::TWISPI0>,
    adc: &mut Saadc<'_, 1>,
    temp_cluster: &mut TemperatureCluster,
    hum_cluster: &mut HumidityCluster,
    press_cluster: &mut PressureCluster,
    illum_cluster: &mut IlluminanceCluster,
    power_cluster: &mut PowerConfigCluster,
    bme280_ok: bool,
    max44009_ok: bool,
) {
    // ── Battery voltage via SAADC (VDD/3 internal) ──────
    let mut buf = [0i16; 1];
    adc.sample(&mut buf).await;
    let raw = buf[0].max(0) as u32;
    let battery_mv = (raw * BATTERY_SCALE_MV_NUM) / BATTERY_SCALE_MV_DEN;

    // ZCL PowerConfig: BatteryVoltage in units of 100mV
    let batt_voltage_zcl = (battery_mv / 100) as u8;
    // Percentage: CR2032 range ~2.0V (empty) to 3.0V (full)
    let batt_pct = if battery_mv >= 3000 {
        200u8 // 100% in ZCL half-percent units
    } else if battery_mv <= 2000 {
        0u8
    } else {
        ((battery_mv - 2000) * 200 / 1000) as u8
    };
    power_cluster.set_battery_voltage(batt_voltage_zcl);
    power_cluster.set_battery_percentage(batt_pct);
    info!("Battery: {}mV ({}%)", battery_mv, batt_pct / 2);

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
        } else {
            warn!("MAX44009 read failed");
        }
    }
}

fn handle_stack_event(
    event: &StackEvent,
    net_state: &mut NetworkState,
    led: &mut gpio::Output<'_>,
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
