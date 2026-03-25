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
}
