#!/usr/bin/env python3
"""Generate a Zigbee OTA (.zigbee) file from a firmware binary.

Wraps a raw firmware .bin in the standard Zigbee OTA Upgrade Image format
(ZCL spec §11.4, Table 11-2) so coordinators like Zigbee2MQTT can serve it.

Usage:
    python3 make_ota.py <input.bin> <output.zigbee> [options]

Options:
    --manufacturer  Manufacturer code (default: 0x1234)
    --image-type    Image type ID    (default: 0x0001)
    --version       File version     (default: 0x00000100 = 0.1.0)
    --string        Header string    (max 32 chars)
"""

import argparse
import struct

OTA_MAGIC = 0x0BEEF11E
OTA_HEADER_VERSION = 0x0100
OTA_HEADER_SIZE = 56  # fixed, no optional fields
ZIGBEE_STACK_VERSION = 0x0002  # Zigbee PRO
SUB_ELEMENT_TAG_UPGRADE_IMAGE = 0x0000


def make_ota(
    firmware: bytes,
    manufacturer: int,
    image_type: int,
    file_version: int,
    header_string: str,
) -> bytes:
    hs = header_string.encode("utf-8")[:32]
    hs_padded = hs + b"\x00" * (32 - len(hs))

    # Sub-element: tag(2) + length(4) + payload
    sub_element_header = struct.pack("<HI", SUB_ELEMENT_TAG_UPGRADE_IMAGE, len(firmware))
    total_size = OTA_HEADER_SIZE + len(sub_element_header) + len(firmware)

    header = struct.pack(
        "<IHHHHHIH32sI",
        OTA_MAGIC,
        OTA_HEADER_VERSION,
        OTA_HEADER_SIZE,
        0x0000,              # field control (no optional fields)
        manufacturer,
        image_type,
        file_version,
        ZIGBEE_STACK_VERSION,
        hs_padded,
        total_size,
    )

    assert len(header) == OTA_HEADER_SIZE
    return header + sub_element_header + firmware


def main():
    parser = argparse.ArgumentParser(description="Generate Zigbee OTA image")
    parser.add_argument("input", help="Input firmware .bin file")
    parser.add_argument("output", help="Output .zigbee file")
    parser.add_argument("--manufacturer", type=lambda x: int(x, 0), default=0x1234,
                        help="Manufacturer code (hex, default: 0x1234)")
    parser.add_argument("--image-type", type=lambda x: int(x, 0), default=0x0001,
                        help="Image type (hex, default: 0x0001)")
    parser.add_argument("--version", type=lambda x: int(x, 0), default=0x00000100,
                        help="File version (hex, default: 0x00000100)")
    parser.add_argument("--string", default="Efekta EInk290 nRF52840",
                        help="Header string (max 32 chars)")
    args = parser.parse_args()

    with open(args.input, "rb") as f:
        firmware = f.read()

    ota = make_ota(firmware, args.manufacturer, args.image_type, args.version, args.string)

    with open(args.output, "wb") as f:
        f.write(ota)

    print(f"OTA image: {len(ota)} bytes (header={OTA_HEADER_SIZE}, "
          f"firmware={len(firmware)}, sub_element_hdr=6)")
    print(f"  Manufacturer: 0x{args.manufacturer:04X}")
    print(f"  Image type:   0x{args.image_type:04X}")
    print(f"  File version: 0x{args.version:08X}")
    print(f"  Header:       {args.string!r}")


if __name__ == "__main__":
    main()
