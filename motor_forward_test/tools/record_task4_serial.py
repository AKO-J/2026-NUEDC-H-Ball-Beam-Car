#!/usr/bin/env python3
"""Record the task-4 AB baseline telemetry and immutable source metadata."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import signal
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports

COLUMNS = (
    "pc_ms,frame,k230_ms,ball_x_px,ball_error_cm_x100,confidence,vision_age_ms,"
    "ball_velocity_px_s,beam_cmd_mdeg,beam_target_pos,beam_estimated_pos,step_freq,"
    "left_encoder_count,right_encoder_count,left_speed_counts_s,right_speed_counts_s,"
    "accel_x_raw,accel_y_raw,accel_z_raw,left_pwm,right_pwm,phase,safety_state"
).split(",")
PHASES = {"DISARMED", "READY", "RAMP_UP", "CRUISE", "RAMP_DOWN", "STOPPED"}
ROOT = Path(__file__).resolve().parents[1]
SOURCES = (
    "tests/bringup/task4_baseline_test.c",
    "tests/bringup/ball_beam_vision_control_test.c",
    "task4_baseline_profile.c", "task4_baseline_profile.h",
    "ball_beam_controller.c", "ball_beam_controller.h",
    "beam_calibration.c", "beam_calibration.h", "motor_driver.c",
    "motor_driver.h", "encoder.c", "encoder.h", "icm42688.c", "icm42688.h",
    "diagnostic_uart.c", "diagnostic_uart.h", "k230/steel_ball_uart_yolo.py",
)
stop_requested = False


def stop(_signum: int, _frame: object) -> None:
    global stop_requested
    stop_requested = True


def valid_row(text: str) -> list[str] | None:
    try:
        row = next(csv.reader([text]), None)
        if row is None or len(row) != len(COLUMNS) or row[-2] not in PHASES:
            return None
        for value in row[:-2]:
            int(value)
        return row
    except (csv.Error, ValueError):
        return None


def choose_port(requested: str, baud: int) -> str:
    if requested != "auto":
        return requested
    candidates = [p.device for p in list_ports.comports()
                  if "xds110" in ((p.description or "") + (p.manufacturer or "")).lower()]
    for name in candidates:
        try:
            with serial.Serial(name, baud, timeout=0.25) as device:
                if any(valid_row(device.readline().decode("ascii", "replace").strip())
                       for _ in range(3)):
                    return name
        except serial.SerialException:
            pass
    interface_one = [name for name in candidates if name.endswith("1")]
    if len(interface_one) == 1:
        return interface_one[0]
    raise SystemExit("无法唯一识别第四题 XDS110 日志串口，请用 --port 指定")


def snapshot() -> dict[str, object]:
    hashes = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
              for name in SOURCES if (ROOT / name).is_file()}
    canonical = json.dumps(hashes, sort_keys=True, separators=(",", ":"))
    return {
        "schema": "task4_ab_baseline/v1",
        "configuration_version": hashlib.sha256(canonical.encode()).hexdigest()[:16],
        "source_sha256": hashes,
        "profile": {"ramp_up_ms": 1500, "cruise_ms": 2000,
                    "ramp_down_ms": 1500, "cruise_pwm_percent": 22},
        "pid_tuning": "frozen_no_automatic_changes",
        "imu_missing_sentinel": -2147483648,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="记录第四题低速直线 AB CSV v1")
    parser.add_argument("--port", default="auto")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--logs", type=Path, default=Path("logs/task4"))
    args = parser.parse_args()
    port = choose_port(args.port, args.baud)
    args.logs.mkdir(parents=True, exist_ok=True)
    stem = datetime.now().strftime("task4_%Y%m%d_%H%M%S")
    output = args.logs / f"{stem}.csv"
    metadata_path = args.logs / f"{stem}.meta.json"
    metadata = {**snapshot(), "csv": output.name, "serial_port": port,
                "baud": args.baud,
                "started_at": datetime.now().astimezone().isoformat(timespec="seconds")}
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2,
                                        sort_keys=True) + "\n", encoding="utf-8")
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    accepted = rejected = 0
    try:
        with serial.Serial(port, args.baud, timeout=0.2) as device, \
                output.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle, lineterminator="\n")
            writer.writerow(COLUMNS)
            handle.flush()
            while not stop_requested:
                raw = device.readline()
                if not raw:
                    continue
                row = valid_row(raw.decode("ascii", "replace").strip())
                if row is None:
                    rejected += 1
                    continue
                writer.writerow(row)
                handle.flush()
                accepted += 1
    finally:
        metadata.update({"stopped_at": datetime.now().astimezone().isoformat(timespec="seconds"),
                         "accepted_rows": accepted, "rejected_rows": rejected})
        metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2,
                                            sort_keys=True) + "\n", encoding="utf-8")
        if output.exists():
            with output.open("a", encoding="utf-8") as handle:
                handle.flush(); os.fsync(handle.fileno())
    print(f"第四题记录已保存: {output}（有效 {accepted}，忽略 {rejected}）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
