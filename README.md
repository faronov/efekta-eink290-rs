# efekta-eink290-rs

Rust firmware for the **Efekta E-Ink 290 nRF52840** multi-sensor, powered by [zigbee-rs](https://github.com/faronov/zigbee-rs).

This is a Rust port of the [original C/Zephyr/ZBOSS firmware](https://github.com/faronov/efekta_eink290_nrf52840).

## Hardware

| Component | Details |
|-----------|---------|
| MCU | nRF52840 (custom Efekta PCB) |
| Sensors | BME280 (temp/humidity/pressure, I2C 0x77) + MAX44009 (lux, I2C 0x4A) |
| Display | 2.9" e-paper (SSD1680/UC8151) — *future* |
| Button | P0.24 (active low) |
| LED | P0.02 (active low) |
| Power | CR2032 battery |

## Zigbee Clusters (Endpoint 1)

- Basic (0x0000)
- Identify (0x0003)
- Power Configuration (0x0001)
- Temperature Measurement (0x0402)
- Relative Humidity (0x0405)
- Pressure Measurement (0x0403)
- Illuminance Measurement (0x0400)

## Operation (Aqara-style)

- **Short press** → Join network (idle) / Force report (joined)
- **Long press (5s)** → Factory reset + pairing mode
- Sensors read every 60s when joined

## Building

```bash
# Install probe-rs
cargo install probe-rs-tools

# Build release firmware
cargo build --release

# Flash and run (with probe-rs + J-Link/DAPLink)
cargo run --release
```

## Dependencies

This firmware uses [zigbee-rs](https://github.com/faronov/zigbee-rs) as a git dependency — a pure Rust Zigbee PRO R22 stack. No ZBOSS binary blob, no Zephyr, no NCS.

## Status

🚧 **Work in progress** — the Zigbee stack is functional but some features are still being developed:
- [x] BME280 driver (temperature, humidity, pressure)
- [x] MAX44009 driver (illuminance)
- [x] Button handling (short/long press)
- [x] LED patterns (Aqara-style)
- [x] ZCL attribute reporting
- [ ] E-paper display (SSD1680/UC8151)
- [ ] Deep sleep between reports
- [ ] NVS persistence across reboots
- [ ] Battery voltage monitoring

## License

MIT OR Apache-2.0
