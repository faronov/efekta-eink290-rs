//! MAX44009 ambient light sensor I2C driver — minimal no_std.
//!
//! Auto-ranging lux sensor. Returns illuminance from two registers.

use embassy_nrf::peripherals;
use embassy_nrf::twim::Twim;

use defmt::*;

const REG_INT_STATUS: u8 = 0x00;
const REG_INT_ENABLE: u8 = 0x01;
const REG_CONFIG: u8 = 0x02;
const REG_LUX_HIGH: u8 = 0x03;
const REG_LUX_LOW: u8 = 0x04;

pub async fn init(i2c: &mut Twim<'_, peripherals::TWISPI0>, addr: u8) -> bool {
    let mut buf = [0u8];
    if i2c
        .write_read(addr, &[REG_INT_STATUS], &mut buf)
        .await
        .is_err()
    {
        warn!("MAX44009 not found at 0x{:02X}", addr);
        return false;
    }

    // Continuous mode, auto-range
    if i2c.write(addr, &[REG_CONFIG, 0b1000_0000]).await.is_err() {
        return false;
    }

    let _ = i2c.write(addr, &[REG_INT_ENABLE, 0x00]).await;
    true
}

pub async fn read_lux(i2c: &mut Twim<'_, peripherals::TWISPI0>, addr: u8) -> Option<u16> {
    let mut high = [0u8];
    if i2c
        .write_read(addr, &[REG_LUX_HIGH], &mut high)
        .await
        .is_err()
    {
        return None;
    }

    let mut low = [0u8];
    if i2c
        .write_read(addr, &[REG_LUX_LOW], &mut low)
        .await
        .is_err()
    {
        return None;
    }

    let exponent = (high[0] >> 4) as u32;
    let mantissa = (((high[0] & 0x0F) as u32) << 4) | ((low[0] & 0x0F) as u32);
    let lux = ((mantissa << exponent) * 45) / 1000;

    Some(lux as u16)
}
