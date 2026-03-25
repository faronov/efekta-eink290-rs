//! Sensor dashboard layout for the 2.9" e-paper display.
//!
//! Landscape orientation (296×128 logical, 90° rotation).
//! **Inverted mode** (black background, white text) matching the
//! Efekta design language from the reference devices.

use crate::paint::{self, Paint, BLACK, WHITE};

/// Sensor + status data for a single display update.
pub struct DisplayData {
    pub temperature_centideg: i16,
    pub humidity_centipct: u16,
    pub pressure_hpa: u16,
    pub lux: u16,
    pub battery_pct: u8,
    pub battery_mv: u16,
    pub joined: bool,
    /// Rolling pressure history (hPa, oldest first). 0 = no data.
    pub pressure_history: [u16; 24],
    pub pressure_history_len: u8,
    /// Time from ZCL Time cluster (0 = unavailable).
    pub time_hour: u8,
    pub time_minute: u8,
    pub date_day: u8,
    pub date_month: u8,
    /// Previous temperature (centideg) for trend arrow direction.
    pub prev_temperature_centideg: i16,
    /// Toggle for colon blink (firmware flips each refresh cycle).
    pub colon_on: bool,
}

/// Draw the full sensor dashboard onto the paint buffer (landscape, inverted).
///
/// Layout (296×128, white-on-black):
/// ```text
/// ┌──════════════════════════════════════════════┐  y=0
/// │ [BAT5] 2.94V 85%     25 Mar 17:15 (Z)       │  y:2-10 header
/// ├══════════════════════════════════════════════╡  y=13+15 double-line
/// │       ▲ 23.5 °C                              │  y:17-56 hero (24×40)
/// ├══════════════════════════════════════════════╡  y=59+61 double-line
/// │💧Humidity  │⊕Press hPa │☀Light lux           │  y:63-69 icons+labels
/// │ 45.2% OK▲  │ 1013▼     │  1250▲              │  y:73-86 values+comfort
/// │            │ +2         │                     │  y:88 3h delta
/// ├══════════════════════════════════════════════╡  y=96+98 double-line
/// │      Pressure 24h ☁              1021        │  y:100 label+weather+max
/// │  ┊▐█▌▐██▌▐█▌▐███▌▐██▌▐█▌▐██▌▐███▌          │  y:109-127 bars+ticks
/// │                                   1005       │  y:120 min
/// └──────────────────────────────────────────────┘  y=128
/// ```
pub fn draw_dashboard(paint: &mut Paint, data: &DisplayData) {
    // Inverted: black background, white foreground
    paint.clear(BLACK);

    let font = &paint::FONT_5X7;
    let lw = paint.width(); // 296
    let fg = WHITE;

    // Temperature trend: compare with previous
    let temp_up = data.temperature_centideg >= data.prev_temperature_centideg;

    // ── Header (y=2) ────────────────────────────────────
    draw_battery_icon(paint, 4, 2, data.battery_pct, fg);

    // Battery LOW indicator when <10%, otherwise voltage+percentage
    if data.battery_pct < 10 {
        paint.draw_str(26, 3, "LOW", font, fg);
    } else {
        let mut bv = StrBuf::<12>::new();
        fmt_u16(&mut bv, data.battery_mv / 1000);
        bv.push_str(".");
        fmt_u16(&mut bv, (data.battery_mv % 1000) / 10);
        bv.push_str("V ");
        fmt_u16(&mut bv, data.battery_pct as u16);
        bv.push_str("%");
        paint.draw_str(26, 3, bv.as_str(), font, fg);
    }

    // Date/time with colon blink + Zigbee icon on the right
    if data.date_month > 0 && data.date_month <= 12 {
        let mut dt = StrBuf::<12>::new();
        fmt_u16(&mut dt, data.date_day as u16);
        dt.push_str(" ");
        dt.push_str(month_abbr(data.date_month));
        dt.push_str(" ");
        if data.time_hour < 10 {
            dt.push_str("0");
        }
        fmt_u16(&mut dt, data.time_hour as u16);
        // Colon blink: alternate ":" vs " " each refresh
        if data.colon_on {
            dt.push_str(":");
        } else {
            dt.push_str(" ");
        }
        if data.time_minute < 10 {
            dt.push_str("0");
        }
        fmt_u16(&mut dt, data.time_minute as u16);
        // Position date/time dynamically: right-aligned before ZB icon
        let dt_len = dt.as_str().len() as u16;
        paint.draw_str(lw - 13 - 4 - dt_len * 6, 3, dt.as_str(), font, fg);
    }
    draw_zigbee_icon(paint, lw - 13, 2, data.joined, fg);

    // Double-line separator
    draw_double_line(paint, 13, 0, lw, fg);

    // ── Hero temperature (24×40 large 7-segment font) ──
    let mut tmp = StrBuf::<12>::new();
    let neg = data.temperature_centideg < 0;
    if neg {
        tmp.push_str("-");
    }
    let abs_val = if neg {
        (-(data.temperature_centideg as i32)) as u16
    } else {
        data.temperature_centideg as u16
    };
    fmt_u16(&mut tmp, abs_val / 100);
    tmp.push_str(".");
    fmt_u16(&mut tmp, (abs_val % 100) / 10);
    let temp_text = tmp.as_str();

    let large_w = large_text_width(temp_text);
    let deg_w = 9u16;
    let c_w = 18u16;
    let total_w = 7 + 6 + large_w + deg_w + 2 + c_w;
    let tx = (lw - total_w) / 2;
    let hero_y = 17u16; // shifted for double-line

    // Thick trend arrow linked to real data
    draw_trend_arrow(paint, tx, hero_y + 8, 24, temp_up, fg);
    let end_x = draw_large_text(paint, tx + 7 + 6, hero_y, temp_text, fg);
    draw_degree_symbol(paint, end_x, hero_y, 3);
    draw_scaled(paint, end_x + deg_w + 2, hero_y, "C", font, fg, 3);

    // Double-line separator
    draw_double_line(paint, 59, 0, lw, fg);

    // ── Three-column secondary readings ────────────────
    let col_w = lw / 3;
    paint.vline(col_w, 60, 37, fg);
    paint.vline(col_w * 2, 60, 37, fg);

    // Compute 3h pressure trend
    let count = data.pressure_history_len.min(24) as usize;
    let p_trend_3h: i16 = if count >= 4 {
        let p3h_ago = data.pressure_history[count - 4];
        if p3h_ago > 0 {
            data.pressure_hpa as i16 - p3h_ago as i16
        } else {
            0
        }
    } else {
        0
    };

    // Column 1: Humidity — centered icon + label, centered value + trend + comfort
    let c1_mid = col_w / 2;
    let h_label_w = 11u16 + 8 * 6;
    let h_label_x = c1_mid.saturating_sub(h_label_w / 2);
    draw_droplet_icon(paint, h_label_x, 63, fg);
    paint.draw_str(h_label_x + 11, 63, "Humidity", font, fg);
    let mut tmp = StrBuf::<12>::new();
    fmt_u16(&mut tmp, data.humidity_centipct / 100);
    tmp.push_str(".");
    fmt_u16(&mut tmp, (data.humidity_centipct % 100) / 10);
    tmp.push_str("%");
    let val_w = tmp.as_str().len() as u16 * 12 + 7;
    let val_x = c1_mid.saturating_sub(val_w / 2);
    draw_scaled(paint, val_x, 73, tmp.as_str(), font, fg, 2);
    draw_small_arrow(
        paint,
        val_x + tmp.as_str().len() as u16 * 12 + 2,
        76,
        data.humidity_centipct >= 4520,
        fg,
    );
    // Comfort zone label
    let comfort = comfort_label(data.humidity_centipct);
    let comfort_len = comfort.len() as u16;
    paint.draw_str(
        c1_mid.saturating_sub(comfort_len * 3),
        88,
        comfort,
        font,
        fg,
    );

    // Column 2: Pressure — centered icon + label, centered value + trend + 3h delta
    let c2_mid = col_w + col_w / 2;
    let p_label_w = 11u16 + 9 * 6;
    let p_label_x = c2_mid.saturating_sub(p_label_w / 2);
    draw_barometer_icon(paint, p_label_x, 63, fg);
    paint.draw_str(p_label_x + 11, 63, "Press hPa", font, fg);
    let mut tmp = StrBuf::<12>::new();
    fmt_u16(&mut tmp, data.pressure_hpa);
    let val_w = tmp.as_str().len() as u16 * 12 + 7;
    let val_x = c2_mid.saturating_sub(val_w / 2);
    draw_scaled(paint, val_x, 73, tmp.as_str(), font, fg, 2);
    draw_small_arrow(
        paint,
        val_x + tmp.as_str().len() as u16 * 12 + 2,
        76,
        p_trend_3h >= 0,
        fg,
    );
    // 3h delta text
    let mut delta_buf = StrBuf::<6>::new();
    if p_trend_3h >= 0 {
        delta_buf.push_str("+");
    }
    fmt_i16(&mut delta_buf, p_trend_3h);
    let delta_len = delta_buf.as_str().len() as u16;
    paint.draw_str(
        c2_mid.saturating_sub(delta_len * 3),
        88,
        delta_buf.as_str(),
        font,
        fg,
    );

    // Column 3: Light — centered icon + label, centered value + trend
    let c3_mid = col_w * 2 + col_w / 2;
    let l_label_w = 11u16 + 9 * 6;
    let l_label_x = c3_mid.saturating_sub(l_label_w / 2);
    draw_sun_icon(paint, l_label_x, 63, fg);
    paint.draw_str(l_label_x + 11, 63, "Light lux", font, fg);
    let mut tmp = StrBuf::<12>::new();
    fmt_u16(&mut tmp, data.lux);
    let val_w = tmp.as_str().len() as u16 * 12 + 7;
    let val_x = c3_mid.saturating_sub(val_w / 2);
    draw_scaled(paint, val_x, 73, tmp.as_str(), font, fg, 2);
    draw_small_arrow(
        paint,
        val_x + tmp.as_str().len() as u16 * 12 + 2,
        76,
        true,
        fg,
    );

    // Double-line separator
    draw_double_line(paint, 96, 0, lw, fg);

    // ── Pressure bar chart (y=96-127) with min/max ─────────
    // Centered label + weather forecast icon
    let label = "Pressure 24h";
    let label_w = label.len() as u16 * 6;
    let label_x = (lw - label_w - 10) / 2;
    paint.draw_str(label_x, 100, label, font, fg);
    draw_weather_icon(paint, label_x + label_w + 4, 100, p_trend_3h, fg);

    let graph_x = 4u16;
    let graph_y = 109u16;
    let graph_w = lw - 8;
    let graph_h = 18u16;

    if count >= 2 {
        // Min/max labels on right side
        let mut lo = u16::MAX;
        let mut hi = 0u16;
        for i in 0..count {
            let v = data.pressure_history[i];
            if v > 0 {
                lo = lo.min(v);
                hi = hi.max(v);
            }
        }
        if hi > lo {
            let mut tmp = StrBuf::<6>::new();
            fmt_u16(&mut tmp, hi);
            let hi_len = tmp.as_str().len() as u16;
            paint.draw_str(lw - hi_len * 6 - 4, 100, tmp.as_str(), font, fg);

            let mut tmp = StrBuf::<6>::new();
            fmt_u16(&mut tmp, lo);
            let lo_len = tmp.as_str().len() as u16;
            paint.draw_str(lw - lo_len * 6 - 4, 120, tmp.as_str(), font, fg);
        }

        // Left-side tick marks (top, mid, bottom)
        if graph_x >= 2 {
            paint.pixel(graph_x - 1, graph_y, fg);
            paint.pixel(graph_x - 2, graph_y, fg);
            paint.pixel(graph_x - 1, graph_y + graph_h / 2, fg);
            paint.pixel(graph_x - 2, graph_y + graph_h / 2, fg);
            paint.pixel(graph_x - 1, graph_y + graph_h - 1, fg);
            paint.pixel(graph_x - 2, graph_y + graph_h - 1, fg);
        }

        // Draw bars with reduced width
        let chart_w = graph_w - 30;
        draw_sparkline(paint, graph_x, graph_y, chart_w, graph_h, data, fg);
    } else {
        paint.draw_str(graph_x, graph_y + 5, "Collecting...", font, fg);
    }
}

// ── Helpers ──────────────────────────────────────────────────

/// Calculate the pixel width of text rendered with the large 7-segment font.
fn large_text_width(s: &str) -> u16 {
    let mut w = 0u16;
    for ch in s.bytes() {
        w += match ch {
            b'0'..=b'9' => paint::LARGE_DIGIT_W + paint::LARGE_DIGIT_GAP,
            b'.' => 4 + paint::LARGE_DIGIT_GAP,
            b'-' => 16 + paint::LARGE_DIGIT_GAP,
            _ => 12 + paint::LARGE_DIGIT_GAP,
        };
    }
    w
}

/// Draw text using large 7-segment digits (+ proportional dot/minus).
/// Returns the X position after the last character (including trailing gap).
fn draw_large_text(paint: &mut Paint, x: u16, y: u16, s: &str, color: u8) -> u16 {
    let gap = paint::LARGE_DIGIT_GAP;
    let h = paint::LARGE_DIGIT_H;
    let mut cx = x;
    for ch in s.bytes() {
        match ch {
            b'0'..=b'9' => {
                paint.draw_large_digit(cx, y, ch - b'0', color);
                cx += paint::LARGE_DIGIT_W + gap;
            }
            b'.' => {
                // 4×8 dot at bottom, background above
                paint.fill_rect(cx, y, 4, h - 8, color ^ 1);
                paint.fill_rect(cx, y + h - 8, 4, 8, color);
                cx += 4 + gap;
            }
            b'-' => {
                // 16×8 bar at middle, background above/below
                paint.fill_rect(cx, y, 16, 16, color ^ 1);
                paint.fill_rect(cx, y + 16, 16, 8, color);
                paint.fill_rect(cx, y + 24, 16, 16, color ^ 1);
                cx += 16 + gap;
            }
            _ => {
                paint.fill_rect(cx, y, 12, h, color ^ 1);
                cx += 12 + gap;
            }
        }
    }
    cx
}

/// Draw pressure bar chart (auto-scaled vertical bars).
fn draw_sparkline(
    paint: &mut Paint,
    x: u16,
    y: u16,
    w: u16,
    h: u16,
    data: &DisplayData,
    color: u8,
) {
    let count = data.pressure_history_len.min(24) as usize;
    if count < 2 {
        return;
    }
    // Find min/max for auto-scaling
    let mut lo = u16::MAX;
    let mut hi = 0u16;
    for i in 0..count {
        let v = data.pressure_history[i];
        if v > 0 {
            lo = lo.min(v);
            hi = hi.max(v);
        }
    }
    if hi <= lo {
        hi = lo + 1;
    }
    let range = (hi - lo) as u32;

    // Bar width and gap: fit `count` bars in `w` pixels
    let bar_total = w / count as u16; // total space per bar
    let bar_w = bar_total.saturating_sub(1).max(1); // bar width (1px gap)

    for i in 0..count {
        let bx = x + i as u16 * bar_total;
        let val = data.pressure_history[i].max(lo).min(hi);
        // Minimum bar height of 1px even for lowest value
        let bar_h = 1 + ((val - lo) as u32 * (h - 2) as u32 / range) as u16;
        let by = y + h - bar_h;
        paint.fill_rect(bx, by, bar_w, bar_h, color);
    }
}

fn month_abbr(m: u8) -> &'static str {
    match m {
        1 => "Jan",
        2 => "Feb",
        3 => "Mar",
        4 => "Apr",
        5 => "May",
        6 => "Jun",
        7 => "Jul",
        8 => "Aug",
        9 => "Sep",
        10 => "Oct",
        11 => "Nov",
        12 => "Dec",
        _ => "---",
    }
}

/// Graphical battery icon (20×9 pixels): outline + 5 fill segments + terminal nub.
fn draw_battery_icon(paint: &mut Paint, x: u16, y: u16, pct: u8, color: u8) {
    // Outline body 18×9
    paint.hline(x, y, 18, color);
    paint.hline(x, y + 8, 18, color);
    paint.vline(x, y, 9, color);
    paint.vline(x + 17, y, 9, color);
    // Terminal nub
    paint.fill_rect(x + 18, y + 2, 2, 5, color);
    // Fill segments (up to 5 based on percentage)
    let segs = if pct > 80 {
        5u16
    } else if pct > 60 {
        4
    } else if pct > 40 {
        3
    } else if pct > 20 {
        2
    } else if pct > 5 {
        1
    } else {
        0
    };
    for i in 0..segs {
        paint.fill_rect(x + 2 + i * 3 + i, y + 2, 2, 5, color);
    }
}

/// Zigbee icon (11×9 pixels): circle with Z = joined, circle with X = offline.
fn draw_zigbee_icon(paint: &mut Paint, x: u16, y: u16, joined: bool, color: u8) {
    #[rustfmt::skip]
    static CIRCLE: [[u8; 11]; 9] = [
        [0,0,0,1,1,1,1,1,0,0,0],
        [0,1,1,0,0,0,0,0,1,1,0],
        [1,0,0,0,0,0,0,0,0,0,1],
        [1,0,0,0,0,0,0,0,0,0,1],
        [1,0,0,0,0,0,0,0,0,0,1],
        [1,0,0,0,0,0,0,0,0,0,1],
        [1,0,0,0,0,0,0,0,0,0,1],
        [0,1,1,0,0,0,0,0,1,1,0],
        [0,0,0,1,1,1,1,1,0,0,0],
    ];
    #[rustfmt::skip]
    static Z_LETTER: [[u8; 11]; 9] = [
        [0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,1,1,1,1,1,0,0,0],
        [0,0,0,0,0,0,1,0,0,0,0],
        [0,0,0,0,0,1,0,0,0,0,0],
        [0,0,0,0,1,0,0,0,0,0,0],
        [0,0,0,1,1,1,1,1,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0],
        [0,0,0,0,0,0,0,0,0,0,0],
    ];
    for row in 0..9u16 {
        for col in 0..11u16 {
            if CIRCLE[row as usize][col as usize] == 1 {
                paint.pixel(x + col, y + row, color);
            }
            if joined && Z_LETTER[row as usize][col as usize] == 1 {
                paint.pixel(x + col, y + row, color);
            }
        }
    }
    if !joined {
        for i in 1..8u16 {
            paint.pixel(x + 1 + i, y + i, color);
            paint.pixel(x + 9 - i, y + i, color);
        }
    }
}

/// Small 5×7 trend arrow for column values.
fn draw_small_arrow(paint: &mut Paint, x: u16, y: u16, up: bool, color: u8) {
    if up {
        paint.pixel(x + 2, y, color);
        for dx in 1..=3 {
            paint.pixel(x + dx, y + 1, color);
        }
        for dx in 0..=4 {
            paint.pixel(x + dx, y + 2, color);
        }
        for dy in 3..7 {
            for dx in 1..=3 {
                paint.pixel(x + dx, y + dy, color);
            }
        }
    } else {
        for dy in 0..4 {
            for dx in 1..=3 {
                paint.pixel(x + dx, y + dy, color);
            }
        }
        for dx in 0..=4 {
            paint.pixel(x + dx, y + 4, color);
        }
        for dx in 1..=3 {
            paint.pixel(x + dx, y + 5, color);
        }
        paint.pixel(x + 2, y + 6, color);
    }
}

/// Thick trend arrow (7px wide × h px tall, 3px shaft + wide arrowhead).
/// `up` = true for up arrow, false for down arrow.
fn draw_trend_arrow(paint: &mut Paint, x: u16, y: u16, h: u16, up: bool, color: u8) {
    if up {
        // 3px wide shaft
        for dy in 5..h {
            paint.pixel(x + 2, y + dy, color);
            paint.pixel(x + 3, y + dy, color);
            paint.pixel(x + 4, y + dy, color);
        }
        // Wide arrowhead (up)
        paint.pixel(x + 3, y, color);
        for dx in 2..=4 {
            paint.pixel(x + dx, y + 1, color);
        }
        for dx in 1..=5 {
            paint.pixel(x + dx, y + 2, color);
        }
        for dx in 0..=6 {
            paint.pixel(x + dx, y + 3, color);
        }
    } else {
        // 3px wide shaft
        for dy in 0..h.saturating_sub(5) {
            paint.pixel(x + 2, y + dy, color);
            paint.pixel(x + 3, y + dy, color);
            paint.pixel(x + 4, y + dy, color);
        }
        // Wide arrowhead (down)
        let tip = h.saturating_sub(1);
        for dx in 0..=6 {
            paint.pixel(x + dx, y + tip - 3, color);
        }
        for dx in 1..=5 {
            paint.pixel(x + dx, y + tip - 2, color);
        }
        for dx in 2..=4 {
            paint.pixel(x + dx, y + tip - 1, color);
        }
        paint.pixel(x + 3, y + tip, color);
    }
}

/// Draw a degree symbol (tiny hollow square) using the given color.
fn draw_degree_symbol(paint: &mut Paint, x: u16, y: u16, scale: u16) {
    let fg = WHITE;
    let bg = BLACK;
    let sz = 3 * scale;
    // Horizontal bars
    for dx in 0..sz {
        for s in 0..scale {
            paint.pixel(x + dx, y + s, fg);
            paint.pixel(x + dx, y + 2 * scale + s, fg);
        }
    }
    // Vertical bars
    for dy in 0..sz {
        for s in 0..scale {
            paint.pixel(x + s, y + dy, fg);
            paint.pixel(x + 2 * scale + s, y + dy, fg);
        }
    }
    // Clear inside
    for dy in scale..2 * scale {
        for dx in scale..2 * scale {
            paint.pixel(x + dx, y + dy, bg);
        }
    }
}

/// Water droplet icon (9×9 pixels) for humidity.
fn draw_droplet_icon(paint: &mut Paint, x: u16, y: u16, color: u8) {
    #[rustfmt::skip]
    static DROPLET: [[u8; 9]; 9] = [
        [0,0,0,0,1,0,0,0,0],
        [0,0,0,1,0,1,0,0,0],
        [0,0,1,0,0,0,1,0,0],
        [0,0,1,0,0,0,1,0,0],
        [0,1,0,0,0,0,0,1,0],
        [0,1,0,0,0,0,0,1,0],
        [0,1,0,0,1,0,0,1,0],
        [0,0,1,0,0,0,1,0,0],
        [0,0,0,1,1,1,0,0,0],
    ];
    for row in 0..9u16 {
        for col in 0..9u16 {
            if DROPLET[row as usize][col as usize] == 1 {
                paint.pixel(x + col, y + row, color);
            }
        }
    }
}

/// Barometer icon (9×9 pixels) for atmospheric pressure — dial gauge with needle.
fn draw_barometer_icon(paint: &mut Paint, x: u16, y: u16, color: u8) {
    #[rustfmt::skip]
    static BARO: [[u8; 9]; 9] = [
        [0,0,1,1,1,1,1,0,0],
        [0,1,0,0,1,0,0,1,0],
        [1,0,0,0,0,0,1,0,1],
        [1,0,0,0,0,1,0,0,1],
        [1,0,0,0,1,0,0,0,1],
        [1,0,0,0,0,0,0,0,1],
        [1,0,0,0,0,0,0,0,1],
        [0,1,0,0,0,0,0,1,0],
        [0,0,1,1,1,1,1,0,0],
    ];
    for row in 0..9u16 {
        for col in 0..9u16 {
            if BARO[row as usize][col as usize] == 1 {
                paint.pixel(x + col, y + row, color);
            }
        }
    }
}

/// Sun icon (9×9 pixels) for light/lux.
fn draw_sun_icon(paint: &mut Paint, x: u16, y: u16, color: u8) {
    #[rustfmt::skip]
    static SUN: [[u8; 9]; 9] = [
        [0,0,0,0,1,0,0,0,0],
        [1,0,0,0,1,0,0,0,1],
        [0,1,0,1,1,1,0,1,0],
        [0,0,1,1,1,1,1,0,0],
        [1,1,1,1,1,1,1,1,1],
        [0,0,1,1,1,1,1,0,0],
        [0,1,0,1,1,1,0,1,0],
        [1,0,0,0,1,0,0,0,1],
        [0,0,0,0,1,0,0,0,0],
    ];
    for row in 0..9u16 {
        for col in 0..9u16 {
            if SUN[row as usize][col as usize] == 1 {
                paint.pixel(x + col, y + row, color);
            }
        }
    }
}

/// Draw text at Nx scale (no_std, no alloc).
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

/// Double-line separator (two 1px lines 2px apart for bolder section dividers).
fn draw_double_line(paint: &mut Paint, y: u16, x0: u16, w: u16, color: u8) {
    paint.hline(x0, y, w, color);
    paint.hline(x0, y + 2, w, color);
}

/// Weather forecast icon (7×7 pixels) based on 3h pressure trend.
/// Sun = rising (trend ≥ 3), Cloud = stable (-2 ≤ trend < 3), Rain = falling (trend < -2).
fn draw_weather_icon(paint: &mut Paint, x: u16, y: u16, trend_3h: i16, color: u8) {
    if trend_3h >= 3 {
        // Sun: simple radiating icon
        #[rustfmt::skip]
        static SUN7: [[u8; 7]; 7] = [
            [0,0,0,1,0,0,0],
            [1,0,0,1,0,0,1],
            [0,1,1,1,1,1,0],
            [1,1,1,1,1,1,1],
            [0,1,1,1,1,1,0],
            [1,0,0,1,0,0,1],
            [0,0,0,1,0,0,0],
        ];
        for row in 0..7u16 {
            for col in 0..7u16 {
                if SUN7[row as usize][col as usize] == 1 {
                    paint.pixel(x + col, y + row, color);
                }
            }
        }
    } else if trend_3h >= -2 {
        // Cloud
        #[rustfmt::skip]
        static CLOUD: [[u8; 7]; 7] = [
            [0,0,0,0,0,0,0],
            [0,0,1,1,0,0,0],
            [0,1,0,0,1,0,0],
            [1,0,0,0,0,1,0],
            [1,0,0,0,0,1,1],
            [0,1,1,1,1,1,0],
            [0,0,0,0,0,0,0],
        ];
        for row in 0..7u16 {
            for col in 0..7u16 {
                if CLOUD[row as usize][col as usize] == 1 {
                    paint.pixel(x + col, y + row, color);
                }
            }
        }
    } else {
        // Rain: cloud + drops
        #[rustfmt::skip]
        static RAIN: [[u8; 7]; 7] = [
            [0,0,1,1,0,0,0],
            [0,1,0,0,1,0,0],
            [1,0,0,0,0,1,1],
            [0,1,1,1,1,1,0],
            [0,1,0,1,0,1,0],
            [0,0,1,0,1,0,0],
            [0,0,0,0,0,0,0],
        ];
        for row in 0..7u16 {
            for col in 0..7u16 {
                if RAIN[row as usize][col as usize] == 1 {
                    paint.pixel(x + col, y + row, color);
                }
            }
        }
    }
}

/// Comfort zone label based on humidity: DRY (<30%), OK (30-60%), WET (>60%).
fn comfort_label(humidity_centipct: u16) -> &'static str {
    let pct = humidity_centipct / 100;
    if pct < 30 {
        "DRY"
    } else if pct <= 60 {
        "OK"
    } else {
        "WET"
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

/// Format an i16 as decimal ASCII (with sign for negatives) into a StrBuf (no_std).
fn fmt_i16<const N: usize>(buf: &mut StrBuf<N>, val: i16) {
    if val < 0 {
        buf.push_str("-");
        fmt_u16(buf, (-(val as i32)) as u16);
    } else {
        fmt_u16(buf, val as u16);
    }
}
