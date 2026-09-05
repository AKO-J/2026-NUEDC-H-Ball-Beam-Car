#!/usr/bin/env python3
"""Convert a contiguous raw flash binary into TI-TXT format for MSPM0 BSL."""

from __future__ import annotations

import argparse
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, type=Path, help="Raw binary from objcopy")
    parser.add_argument("--output", required=True, type=Path, help="TI-TXT output path")
    parser.add_argument("--address", default="0x0", help="Start address (default: 0x0)")
    args = parser.parse_args()

    address = int(args.address, 0)
    data = args.input.read_bytes()
    if not data:
        parser.error("input binary is empty")
    if address < 0 or address + len(data) > 0x20000:
        parser.error("image is outside MSPM0G3507 MAIN flash")

    lines = [f"@{address:04X}"]
    lines.extend(data[offset : offset + 16].hex().upper() for offset in range(0, len(data), 16))
    lines.append("q")
    args.output.write_text("\n".join(lines) + "\n", encoding="ascii")
    print(f"Wrote {args.output}: {len(data)} bytes at 0x{address:08X}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
