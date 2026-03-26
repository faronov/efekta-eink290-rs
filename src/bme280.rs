//! BME280 I2C driver — minimal no_std, forced-mode for battery devices.
//!
//! Implements the Bosch BME280 compensation formulas from the datasheet.
//! Uses forced mode (single-shot) to minimize power consumption.

use embassy_nrf::peripherals;
use embassy_nrf::twim::Twim;
use embassy_time::{Duration, Timer};

use defmt::*;

const REG_ID: u8 = 0xD0;
const REG_CTRL_HUM: u8 = 0xF2;
const REG_STATUS: u8 = 0xF3;
const REG_CTRL_MEAS: u8 = 0xF4;
const REG_CONFIG: u8 = 0xF5;
const REG_DATA_START: u8 = 0xF7;
const REG_CALIB_00: u8 = 0x88;
const REG_CALIB_26: u8 = 0xE1;

const BME280_CHIP_ID: u8 = 0x60;

struct Calibration {
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
    dig_p1: u16,
    dig_p2: i16,
    dig_p3: i16,
    dig_p4: i16,
    dig_p5: i16,
    dig_p6: i16,
    dig_p7: i16,
    dig_p8: i16,
    dig_p9: i16,
    dig_h1: u8,
    dig_h2: i16,
    dig_h3: u8,
    dig_h4: i16,
    dig_h5: i16,
    dig_h6: i8,
}

static mut CALIB: Option<Calibration> = None;

pub struct Bme280Data {
    /// Temperature in centidegrees (2350 = 23.50°C)
    pub temperature_centideg: i16,
    /// Humidity in centipercent (6500 = 65.00%)
    pub humidity_centipct: u16,
    /// Pressure in hPa (integer)
    pub pressure_hpa: u16,
}

pub async fn init(i2c: &mut Twim<'_, peripherals::TWISPI0>, addr: u8) -> bool {
    let mut id = [0u8];
    if i2c.write_read(addr, &[REG_ID], &mut id).await.is_err() {
        return false;
    }
    if id[0] != BME280_CHIP_ID {
        warn!("BME280: chip ID 0x{:02X} != 0x60", id[0]);
        return false;
    }

    let mut calib_tp = [0u8; 26];
    if i2c
        .write_read(addr, &[REG_CALIB_00], &mut calib_tp)
        .await
        .is_err()
    {
        return false;
    }

    let mut calib_h = [0u8; 7];
    if i2c
        .write_read(addr, &[REG_CALIB_26], &mut calib_h)
        .await
        .is_err()
    {
        return false;
    }

    let mut h1 = [0u8];
    if i2c.write_read(addr, &[0xA1], &mut h1).await.is_err() {
        return false;
    }

    let cal = Calibration {
        dig_t1: u16::from_le_bytes([calib_tp[0], calib_tp[1]]),
        dig_t2: i16::from_le_bytes([calib_tp[2], calib_tp[3]]),
        dig_t3: i16::from_le_bytes([calib_tp[4], calib_tp[5]]),
        dig_p1: u16::from_le_bytes([calib_tp[6], calib_tp[7]]),
        dig_p2: i16::from_le_bytes([calib_tp[8], calib_tp[9]]),
        dig_p3: i16::from_le_bytes([calib_tp[10], calib_tp[11]]),
        dig_p4: i16::from_le_bytes([calib_tp[12], calib_tp[13]]),
        dig_p5: i16::from_le_bytes([calib_tp[14], calib_tp[15]]),
        dig_p6: i16::from_le_bytes([calib_tp[16], calib_tp[17]]),
        dig_p7: i16::from_le_bytes([calib_tp[18], calib_tp[19]]),
        dig_p8: i16::from_le_bytes([calib_tp[20], calib_tp[21]]),
        dig_p9: i16::from_le_bytes([calib_tp[22], calib_tp[23]]),
        dig_h1: h1[0],
        dig_h2: i16::from_le_bytes([calib_h[0], calib_h[1]]),
        dig_h3: calib_h[2],
        dig_h4: ((calib_h[3] as i16) << 4) | ((calib_h[4] & 0x0F) as i16),
        dig_h5: ((calib_h[5] as i16) << 4) | ((calib_h[4] >> 4) as i16),
        dig_h6: calib_h[6] as i8,
    };

    unsafe {
        CALIB = Some(cal);
    }

    let _ = i2c.write(addr, &[REG_CTRL_HUM, 0b100]).await; // humidity x8
    let _ = i2c.write(addr, &[REG_CONFIG, 0b1000_0000]).await; // standby 500ms, filter off

    true
}

pub async fn read(i2c: &mut Twim<'_, peripherals::TWISPI0>, addr: u8) -> Option<Bme280Data> {
    let cal = unsafe { (*core::ptr::addr_of!(CALIB)).as_ref()? };

    // Trigger forced mode: temp x8, press x8
    if i2c
        .write(addr, &[REG_CTRL_MEAS, 0b1001_0001])
        .await
        .is_err()
    {
        return None;
    }

    Timer::after(Duration::from_millis(50)).await;

    let mut bme_ready = false;
    for _ in 0..10 {
        let mut status = [0u8];
        if i2c
            .write_read(addr, &[REG_STATUS], &mut status)
            .await
            .is_ok()
            && status[0] & 0x08 == 0
        {
            bme_ready = true;
            break;
        }
        Timer::after(Duration::from_millis(10)).await;
    }
    if !bme_ready {
        defmt::warn!("BME280 busy timeout (150ms)");
        return None;
    }

    let mut raw = [0u8; 8];
    if i2c
        .write_read(addr, &[REG_DATA_START], &mut raw)
        .await
        .is_err()
    {
        return None;
    }

    let adc_p = ((raw[0] as i32) << 12) | ((raw[1] as i32) << 4) | ((raw[2] as i32) >> 4);
    let adc_t = ((raw[3] as i32) << 12) | ((raw[4] as i32) << 4) | ((raw[5] as i32) >> 4);
    let adc_h = ((raw[6] as i32) << 8) | (raw[7] as i32);

    let (t_fine, temp) = compensate_temperature(cal, adc_t);
    let press_pa = compensate_pressure(cal, adc_p, t_fine);
    let hum = compensate_humidity(cal, adc_h, t_fine);

    Some(Bme280Data {
        temperature_centideg: temp as i16,
        humidity_centipct: hum as u16,
        pressure_hpa: (press_pa / 100) as u16,
    })
}

fn compensate_temperature(cal: &Calibration, adc_t: i32) -> (i32, i32) {
    let var1 = (((adc_t >> 3) - ((cal.dig_t1 as i32) << 1)) * (cal.dig_t2 as i32)) >> 11;
    let var2 = (((((adc_t >> 4) - (cal.dig_t1 as i32)) * ((adc_t >> 4) - (cal.dig_t1 as i32)))
        >> 12)
        * (cal.dig_t3 as i32))
        >> 14;
    let t_fine = var1 + var2;
    let temp = (t_fine * 5 + 128) >> 8;
    (t_fine, temp)
}

fn compensate_pressure(cal: &Calibration, adc_p: i32, t_fine: i32) -> i32 {
    let mut var1 = (t_fine as i64) - 128000;
    let mut var2 = var1 * var1 * (cal.dig_p6 as i64);
    var2 += (var1 * (cal.dig_p5 as i64)) << 17;
    var2 += (cal.dig_p4 as i64) << 35;
    var1 = ((var1 * var1 * (cal.dig_p3 as i64)) >> 8) + ((var1 * (cal.dig_p2 as i64)) << 12);
    var1 = (((1i64 << 47) + var1) * (cal.dig_p1 as i64)) >> 33;
    if var1 == 0 {
        return 0;
    }
    let mut p: i64 = 1048576 - adc_p as i64;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((cal.dig_p9 as i64) * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((cal.dig_p8 as i64) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((cal.dig_p7 as i64) << 4);
    (p / 256) as i32
}

fn compensate_humidity(cal: &Calibration, adc_h: i32, t_fine: i32) -> i32 {
    let mut var = t_fine - 76800i32;
    if var == 0 {
        return 0;
    }
    var = ((((adc_h << 14) - ((cal.dig_h4 as i32) << 20) - ((cal.dig_h5 as i32) * var)) + 16384)
        >> 15)
        * (((((((var * (cal.dig_h6 as i32)) >> 10)
            * (((var * (cal.dig_h3 as i32)) >> 11) + 32768))
            >> 10)
            + 2097152)
            * (cal.dig_h2 as i32)
            + 8192)
            >> 14);
    var -= ((((var >> 15) * (var >> 15)) >> 7) * (cal.dig_h1 as i32)) >> 4;
    var = var.clamp(0, 419430400);
    let hum_1024 = var >> 12;
    ((hum_1024 * 100) / 1024).clamp(0, 10000)
}
