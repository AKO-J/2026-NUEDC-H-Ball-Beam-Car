#!/usr/bin/env python3
"""Non-destructive MSPM0 ROM-BSL UART probe.

This is intentionally limited to the Connection and Get Device ID commands.
It cannot erase, unlock, reset, or program the device.

The packets follow TI's MSPM0 BSL_GUI_source_code/BSL_pack.py implementation
for the MSPM0L/G family.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

try:
    import serial
    import serial.tools.list_ports
except ImportError as exc:
    raise SystemExit(
        "Missing pyserial. Run with PYTHONPATH=/private/tmp/mspm0-bsl-deps"
    ) from exc


def crc32_mspm0(data: bytes) -> int:
    """CRC used by TI's MSPM0L/G BSL host implementation."""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    return crc & 0xFFFFFFFF


def packet(command: int) -> bytes:
    core = bytes((command,))
    return b"\x80\x01\x00" + core + struct.pack("<I", crc32_mspm0(core))


CONNECTION = packet(0x12)
GET_DEVICE_ID = packet(0x19)


def list_ports() -> list[str]:
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports detected.")
        return []
    for port in ports:
        print(f"{port.device}: {port.description}")
    return [port.device for port in ports]


def read_exact(port: serial.Serial, size: int) -> bytes:
    data = bytearray()
    while len(data) < size:
        chunk = port.read(size - len(data))
        if not chunk:
            break
        data.extend(chunk)
    return bytes(data)


def probe(port_name: str, wait_seconds: float) -> int:
    deadline = time.monotonic() + wait_seconds
    with serial.Serial(
        port_name,
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.25,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    ) as port:
        print(f"Opened {port_name} at 9600 8N1. Waiting up to {wait_seconds:g}s for BSL...")
        while time.monotonic() < deadline:
            port.reset_input_buffer()
            port.write(CONNECTION)
            port.flush()
            ack = read_exact(port, 1)
            if ack == b"\x00":
                # This is the non-destructive readiness query used by TI's BSL GUI.
                port.write(b"\xbb")
                port.flush()
                ready = read_exact(port, 1)
                if ready != b"\x51":
                    time.sleep(0.15)
                    continue
                port.write(GET_DEVICE_ID)
                port.flush()
                response = read_exact(port, 33)
                if response:
                    print("BSL connected. Get Device ID response:", response.hex())
                    return 0
            time.sleep(0.15)
    print("No BSL response. No erase or write was attempted.", file=sys.stderr)
    return 2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="List detected serial ports and exit")
    parser.add_argument("--port", help="Serial port, for example /dev/cu.usbserial-XXXX")
    parser.add_argument("--wait", type=float, default=12, help="Handshake wait window in seconds (default: 12)")
    args = parser.parse_args()

    if args.list:
        list_ports()
        return 0
    if not args.port:
        parser.error("--port is required unless --list is used")
    if args.wait <= 0:
        parser.error("--wait must be positive")
    return probe(args.port, args.wait)


if __name__ == "__main__":
    raise SystemExit(main())
