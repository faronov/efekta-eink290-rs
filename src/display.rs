//! Sensor dashboard layout for the 128×296 e-paper display.
//!
//! Draws temperature, humidity, pressure, lux, battery and Zigbee
//! status in a simple portrait layout using the 5×7 bitmap font.

use crate::paint::{self, Paint, BLACK, WHITE};

/// Sensor + status data for a single display update.
pub struct DisplayData {
    pub temperature_centideg: i16,
    pub humidity_centipct: u16,
    pub pressure_hpa: u16,
    pub lux: u16,
    pub battery_pct: u8,
    pub joined: bool,
}

/// Draw the full sensor dashboard onto the paint buffer.
pub fn draw_dashboard(paint: &mut Paint, data: &DisplayData) {
    paint.clear(WHITE);

    let font = &paint::FONT_5X7;

    // ── Status bar (y=4) ──────────────────────────────────
    let mut tmp = StrBuf::<16>::new();
    tmp.push_str("Bat:");
    fmt_u16(&mut tmp, data.battery_pct as u16);
    tmp.push_str("%");
    paint.draw_str(4, 4, tmp.as_str(), font, BLACK);

    if data.joined {
        paint.draw_str(90, 4, "ZB:OK", font, BLACK);
    } else {
        paint.draw_str(90, 4, "ZB:--", font, BLACK);
    }

    // separator
    paint.hline(0, 16, 128, BLACK);

    // ── Temperature (y=24, large-ish — draw 2× scaled) ────
    let mut tmp = StrBuf::<16>::new();
    let sign = if data.temperature_centideg < 0 {
        tmp.push_str("-");
        true
    } else {
        false
    };
    let abs_val = if sign {
        (-(data.temperature_centideg as i32)) as u16
    } else {
        data.temperature_centideg as u16
    };
    fmt_u16(&mut tmp, abs_val / 100);
    tmp.push_str(".");
    let frac = (abs_val % 100) / 10;
    fmt_u16(&mut tmp, frac);
    tmp.push_str(" C");
    draw_scaled(paint, 8, 24, tmp.as_str(), font, BLACK, 2);

    // separator
    paint.hline(0, 48, 128, BLACK);

    // ── Humidity (y=56) ───────────────────────────────────
    let mut tmp = StrBuf::<16>::new();
    tmp.push_str("H: ");
    fmt_u16(&mut tmp, data.humidity_centipct / 100);
    tmp.push_str(".");
    fmt_u16(&mut tmp, (data.humidity_centipct % 100) / 10);
    tmp.push_str(" %");
    draw_scaled(paint, 8, 56, tmp.as_str(), font, BLACK, 2);

    // separator
    paint.hline(0, 80, 128, BLACK);

    // ── Pressure (y=88) ───────────────────────────────────
    let mut tmp = StrBuf::<16>::new();
    tmp.push_str("P: ");
    fmt_u16(&mut tmp, data.pressure_hpa);
    tmp.push_str(" hPa");
    draw_scaled(paint, 8, 88, tmp.as_str(), font, BLACK, 2);

    // separator
    paint.hline(0, 112, 128, BLACK);

    // ── Lux (y=120) ───────────────────────────────────────
    let mut tmp = StrBuf::<16>::new();
    tmp.push_str("L: ");
    fmt_u16(&mut tmp, data.lux);
    tmp.push_str(" lux");
    draw_scaled(paint, 8, 120, tmp.as_str(), font, BLACK, 2);
}

// ── Helpers ──────────────────────────────────────────────────

/// Draw text at 2× scale for larger headings (no_std, no alloc).
fn draw_scaled(
    paint: &mut Paint,
    x: u16,
    y: u16,
    s: &str,
    font: &paint::Font,
    color: u8,
    scale: u16,
) {
    let mut cx = x;
    for ch in s.bytes() {
        if ch < font.first_char || ch > font.last_char {
            cx += (font.width + 1) * scale;
            continue;
        }
        let offset = (ch - font.first_char) as usize * font.width as usize;
        for col in 0..font.width {
            let bits = font.data[offset + col as usize];
            for row in 0..font.height {
                let c = if bits & (1 << row) != 0 {
                    color
                } else {
                    color ^ 1
                };
                for sy in 0..scale {
                    for sx in 0..scale {
                        paint.pixel(cx + col * scale + sx, y + row * scale + sy, c);
                    }
                }
            }
        }
        cx += (font.width + 1) * scale;
    }
}

/// Tiny stack-allocated string buffer (no alloc needed).
struct StrBuf<const N: usize> {
    buf: [u8; N],
    len: usize,
}

impl<const N: usize> StrBuf<N> {
    fn new() -> Self {
        Self {
            buf: [0; N],
            len: 0,
        }
    }

    fn push_str(&mut self, s: &str) {
        for &b in s.as_bytes() {
            if self.len < N {
                self.buf[self.len] = b;
                self.len += 1;
            }
        }
    }

    fn push_byte(&mut self, b: u8) {
        if self.len < N {
            self.buf[self.len] = b;
            self.len += 1;
        }
    }

    fn as_str(&self) -> &str {
        // Safety: we only ever push valid ASCII bytes.
        unsafe { core::str::from_utf8_unchecked(&self.buf[..self.len]) }
    }
}

/// Format a u16 as decimal ASCII into a StrBuf (no_std).
fn fmt_u16<const N: usize>(buf: &mut StrBuf<N>, mut val: u16) {
    if val == 0 {
        buf.push_byte(b'0');
        return;
    }
    let mut digits = [0u8; 5];
    let mut i = 0;
    while val > 0 {
        digits[i] = b'0' + (val % 10) as u8;
        val /= 10;
        i += 1;
    }
    while i > 0 {
        i -= 1;
        buf.push_byte(digits[i]);
    }
}
