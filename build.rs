fn main() {
    println!("cargo:rustc-link-arg-bins=--noinhibit-exec");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");

    // OTA version: "major.minor.patch" → (major << 24) | (minor << 16) | patch
    // Example: "0.1.0" → 0x00010000, "1.2.3" → 0x01020003
    let version = env!("CARGO_PKG_VERSION");
    let parts: Vec<u32> = version.split('.').map(|s| s.parse().unwrap()).collect();
    let (major, minor, patch) = (parts[0], parts[1], parts[2]);
    let ota_u32 = (major << 24) | (minor << 16) | patch;
    println!("cargo:rustc-env=OTA_VERSION_U32={ota_u32}");
    println!("cargo:rustc-env=OTA_VERSION_HEX=0x{ota_u32:08X}");

    // Build date for Basic cluster DateCode attribute (YYYYMMDD format)
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_secs();
    // Simple days-since-epoch to YYYYMMDD (no chrono dependency needed)
    let days = (now / 86400) as i64;
    let (y, m, d) = days_to_ymd(days);
    println!("cargo:rustc-env=BUILD_DATE={y:04}{m:02}{d:02}");
}

/// Convert days since Unix epoch to (year, month, day).
fn days_to_ymd(days: i64) -> (i64, i64, i64) {
    // Civil calendar algorithm from Howard Hinnant
    let z = days + 719468;
    let era = z.div_euclid(146097);
    let doe = z.rem_euclid(146097);
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    (y, m, d)
}
