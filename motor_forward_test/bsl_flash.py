#!/usr/bin/env python3
"""Program an MSPM0L/G ROM BSL UART target from a TI-TXT image.

This utility is intentionally guarded by --yes because it mass-erases MAIN
flash before programming.  It uses the UART packet format implemented in
TI's MSPM0 BSL GUI source.
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError as exc:
    raise SystemExit("Missing pyserial. Run with PYTHONPATH=/private/tmp/mspm0-bsl-deps") from exc


ACK_OK = b"\x00"
READY = b"\x51"
CMD_CONNECTION = 0x12
CMD_GET_DEVICE_ID = 0x19
CMD_PASSWORD = 0x21
CMD_MASS_ERASE = 0x15
CMD_PROGRAM = 0x20
CMD_START_APP = 0x40
DEFAULT_PASSWORD = b"\xFF" * 32


def crc32_mspm0(data: bytes) -> int:
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    return crc & 0xFFFFFFFF


def command_packet(command: int, data: bytes = b"") -> bytes:
    core = bytes((command,)) + data
    return b"\x80" + struct.pack("<H", len(core)) + core + struct.pack("<I", crc32_mspm0(core))


def program_packet(address: int, data: bytes) -> bytes:
    if not 1 <= len(data) <= 128:
        raise ValueError("program blocks must contain 1..128 bytes")
    return command_packet(CMD_PROGRAM, struct.pack("<I", address) + data)


def parse_ti_txt(path: Path) -> list[tuple[int, bytes]]:
    segments: list[tuple[int, bytes]] = []
    address: int | None = None
    buffer = bytearray()

    def flush() -> None:
        nonlocal buffer
        if address is not None and buffer:
            segments.append((address, bytes(buffer)))
        buffer = bytearray()

    for line_no, raw_line in enumerate(path.read_text(encoding="ascii").splitlines(), start=1):
        line = raw_line.strip()
        if not line:
            continue
        if line.lower() == "q":
            break
        if line.startswith("@"):
            flush()
            try:
                address = int(line[1:], 16)
            except ValueError as exc:
                raise ValueError(f"line {line_no}: invalid address") from exc
            continue
        if address is None:
            raise ValueError(f"line {line_no}: data appears before an address")
        try:
            chunk = bytes.fromhex(line)
        except ValueError as exc:
            raise ValueError(f"line {line_no}: invalid hex data") from exc
        buffer.extend(chunk)
    flush()

    if not segments:
        raise ValueError("TI-TXT contains no data")
    for start, data in segments:
        if start < 0 or start + len(data) > 0x20000:
            raise ValueError("image is outside MSPM0G3507 MAIN flash")
    return segments


def prepare_blocks(segments: list[tuple[int, bytes]]) -> list[tuple[int, bytes]]:
    """Split into BSL blocks and write the reset vector last, like TI's GUI."""
    memory: dict[int, int] = {}
    for start, data in segments:
        for offset, byte in enumerate(data):
            address = start + offset
            if address in memory and memory[address] != byte:
                raise ValueError(f"overlapping data differs at 0x{address:08X}")
            memory[address] = byte
    if not all(address in memory for address in range(16)):
        raise ValueError("image must include the 16-byte reset vector at 0x00000000")

    deferred_vector = bytes(memory.pop(address) for address in range(16))
    blocks: list[tuple[int, bytes]] = []
    addresses = sorted(memory)
    index = 0
    while index < len(addresses):
        start = addresses[index]
        block = bytearray((memory[start],))
        index += 1
        while index < len(addresses):
            next_address = addresses[index]
            if next_address != start + len(block) or len(block) == 128:
                break
            block.append(memory[next_address])
            index += 1
        blocks.append((start, bytes(block)))
    blocks.append((0, deferred_vector))
    return blocks


def read_exact(port: serial.Serial, size: int) -> bytes:
    result = bytearray()
    while len(result) < size:
        chunk = port.read(size - len(result))
        if not chunk:
            break
        result.extend(chunk)
    return bytes(result)


def exchange(port: serial.Serial, packet: bytes, name: str, expect_status: bool = True) -> bytes:
    port.reset_input_buffer()
    port.write(packet)
    port.flush()
    ack = read_exact(port, 1)
    if ack != ACK_OK:
        raise RuntimeError(f"{name}: BSL acknowledgement {ack.hex() or 'timeout'}, expected 00")
    if not expect_status:
        return b""
    response = read_exact(port, 9)
    # The BSL response is a 9-byte UART packet.  The operation status is
    # byte 5 (index 4), matching TI's BSL GUI check of response_hex[8:10].
    if len(response) != 9 or response[4] != 0:
        raise RuntimeError(f"{name}: BSL response {response.hex() or 'timeout'}")
    return response


def connect_bsl(port: serial.Serial, wait_seconds: float) -> bytes:
    deadline = time.monotonic() + wait_seconds
    while time.monotonic() < deadline:
        port.reset_input_buffer()
        port.write(command_packet(CMD_CONNECTION))
        port.flush()
        if read_exact(port, 1) == ACK_OK:
            port.write(b"\xBB")
            port.flush()
            if read_exact(port, 1) == READY:
                port.write(command_packet(CMD_GET_DEVICE_ID))
                port.flush()
                response = read_exact(port, 33)
                if response:
                    return response
        time.sleep(0.15)
    raise RuntimeError("No BSL response before timeout")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="e.g. /dev/cu.usbserial-XXXX")
    parser.add_argument("--image", required=True, type=Path, help="TI-TXT firmware image")
    parser.add_argument("--wait", type=float, default=20, help="BSL invoke wait time in seconds")
    parser.add_argument("--response-timeout", type=float, default=2.0,
                        help="UART acknowledgement timeout per BSL operation in seconds")
    parser.add_argument("--dry-run", action="store_true", help="Validate image and print block count only")
    parser.add_argument("--yes", action="store_true", help="Confirm MAIN flash erase and programming")
    args = parser.parse_args()

    blocks = prepare_blocks(parse_ti_txt(args.image))
    image_size = sum(len(data) for _, data in blocks)
    print(f"Validated {args.image}: {image_size} bytes in {len(blocks)} BSL blocks")
    if args.dry_run:
        return 0
    if not args.yes:
        parser.error("refusing to erase/program without --yes")
    if args.wait <= 0:
        parser.error("--wait must be positive")

    with serial.Serial(args.port, 9600, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE,
                       timeout=args.response_timeout, xonxoff=False, rtscts=False, dsrdtr=False) as port:
        print("Waiting for ROM BSL hardware invoke...")
        device_id = connect_bsl(port, args.wait)
        print("BSL connected. Device ID:", device_id.hex())
        exchange(port, command_packet(CMD_PASSWORD, DEFAULT_PASSWORD), "password")
        print("Password accepted. Erasing MAIN flash...")
        exchange(port, command_packet(CMD_MASS_ERASE), "mass erase")
        for number, (address, data) in enumerate(blocks, start=1):
            exchange(port, program_packet(address, data), f"program block {number}/{len(blocks)}")
        # TI's BSL GUI waits only for this acknowledgement: the target may
        # immediately leave BSL, so a complete status packet is not reliable.
        exchange(port, command_packet(CMD_START_APP), "start application", expect_status=False)
    print("Programming and BSL verification completed successfully.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"BSL flash failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
