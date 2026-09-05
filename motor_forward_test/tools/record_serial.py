#!/usr/bin/env python3
"""Record MSPM0 diagnostic CSV from the XDS110 backchannel into logs/."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import re
import signal
import sys
import uuid
from datetime import datetime
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError as error:  # pragma: no cover - depends on host setup
    raise SystemExit(
        "缺少 pyserial；请先运行: python3 -m pip install -r tools/requirements.txt"
    ) from error


CSV_COLUMNS = (
    "pc_ms,frame,k230_ms,x,confidence,age_ms,target_x,v,e,cmd,"
    "target_motor_pos,estimated_motor_pos,step_freq,state"
).split(",")
STOP_REQUESTED = False
VALID_STATES = {
    "ACCEL", "BRAKE", "HOLD", "LOST",
    "WAIT_LEVEL", "JOG_LEFT", "JOG_RIGHT", "JOG_STOP",
    "PRESET_LIFT", "RETURN_LEVEL", "PRESET_REFUSED", "ZERO_ACCEPTED",
    "LEVEL_READY", "ACTIVE", "LIMIT_FAULT",
    "TARGET_REJECTED_UNREFERENCED", "TARGET_REJECTED_LIMIT",
    "TARGET_REJECTED_BUSY", "TARGET_REJECTED_FAULT",
}
PROJECT_ROOT = Path(__file__).resolve().parents[1]
TEST_RUN_COLUMNS = (
    "test_run_id,session_run_id,source_csv,configuration_version,"
    "test_plan_id,test_case_id,release_position_cm,"
    "start_wall_time,end_wall_time,start_row_index,end_row_index,"
    "start_pc_ms,end_pc_ms,start_x,target_x,start_motor_pos,"
    "end_x,end_motor_pos,end_state"
).split(",")
VERSIONED_SOURCES = (
    "ball_beam_controller.c",
    "ball_beam_controller.h",
    "beam_calibration.c",
    "beam_calibration.h",
    "stepper_beam.c",
    "stepper_beam.h",
    "tests/bringup/ball_beam_vision_control_test.c",
    "k230/steel_ball_uart_yolo.py",
)


def request_stop(_signum: int, _frame: object) -> None:
    global STOP_REQUESTED
    STOP_REQUESTED = True


def choose_port(requested: str, baud: int) -> str:
    if requested != "auto":
        return requested
    ports = list(list_ports.comports())
    preferred = [
        item.device
        for item in ports
        if any(token in ((item.description or "") + " " +
                         (item.manufacturer or "")).lower()
               for token in ("xds110", "ti xds", "texas instruments"))
    ]
    if len(preferred) == 1:
        return preferred[0]
    if len(preferred) > 1:
        # XDS110 exposes several CDC interfaces.  Ask each candidate for a
        # short sample and select the one that is already producing our CSV.
        # This avoids hard-coding a board serial number or a macOS tty name.
        for device_name in preferred:
            try:
                with serial.Serial(device_name, baud, timeout=0.25) as device:
                    for _ in range(3):
                        line = device.readline().decode("ascii", errors="replace").strip()
                        if valid_row(line) is not None:
                            return device_name
            except serial.SerialException:
                continue
        # The XDS110 application UART is normally CDC interface 1; retain a
        # deterministic fallback when the target has not started transmitting.
        interface_one = [name for name in preferred if name.endswith("1")]
        if len(interface_one) == 1:
            return interface_one[0]
    candidates = [
        item.device for item in ports
        if item.device.startswith(("/dev/cu.usb", "/dev/tty.usb", "COM"))
    ]
    if len(candidates) == 1:
        return candidates[0]
    discovered = ", ".join(item.device for item in ports) or "无"
    raise SystemExit(
        "无法唯一识别 XDS110 串口，请用 --port 指定。已发现: " + discovered
    )


def valid_row(text: str) -> list[str] | None:
    # A USB reset/noisy UART can occasionally splice a line terminator into
    # a read.  Treat that as one rejected telemetry frame, never as a reason
    # to abandon an otherwise valid test recording.
    try:
        row = next(csv.reader([text]), None)
    except csv.Error:
        return None
    if row is None or len(row) != len(CSV_COLUMNS):
        return None
    if row[-1] not in VALID_STATES:
        return None
    try:
        for value in row[:-1]:
            int(value)
    except ValueError:
        return None
    return row


def source_snapshot() -> dict[str, object]:
    """Capture the exact controller/configuration source set for this run.

    This is intentionally evidence only: it never writes firmware values and it
    keeps tuning locked until hardware safety validation is complete.
    """
    source_hashes: dict[str, str] = {}
    extracted: dict[str, str] = {}
    for relative_name in VERSIONED_SOURCES:
        path = PROJECT_ROOT / relative_name
        if not path.is_file():
            continue
        data = path.read_bytes()
        source_hashes[relative_name] = hashlib.sha256(data).hexdigest()
        if path.suffix in {".c", ".h"}:
            text = data.decode("utf-8", errors="replace")
            # Keep human-readable evidence of the active controller gains,
            # estimator constants and angle/travel constraints in addition to
            # the authoritative whole-file hashes.  Enum settings in headers
            # are intentionally included too (the PD gains live there).
            names = (r"BALL_BEAM_[A-Z0-9_]+|BALL_CONTROL_SIGN|"
                     r"ALPHA_[A-Z0-9_]+|BETA_[A-Z0-9_]+|VISION_LOOKAHEAD_MS|"
                     r"BEAM_CAL_[A-Z0-9_]+|TASK3_[A-Z0-9_]+|"
                     r"WAIT_LEVEL_[A-Z0-9_]+|LOWER_STOP_[A-Z0-9_]+")
            for name, value in re.findall(
                    rf"^\s*#define\s+({names})\s+([^\s/]+)", text,
                    flags=re.MULTILINE):
                extracted[name] = value
            for name, value in re.findall(
                    rf"\b({names})\s*=\s*([^,\n/]+)", text):
                extracted[name] = value.strip()
    canonical = json.dumps(source_hashes, sort_keys=True, separators=(",", ":"))
    return {
        "configuration_version": hashlib.sha256(canonical.encode("utf-8")).hexdigest()[:16],
        "source_sha256": source_hashes,
        "extracted_compile_time_values": extracted,
        "tuning_mode": "locked_diagnostic_only",
    }


def write_metadata(path: Path, metadata: dict[str, object]) -> None:
    path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8")


def append_test_run(path: Path, record: dict[str, str | int]) -> None:
    """Append one completed ACTIVE interval to the durable test ledger."""
    new_file = not path.exists()
    with path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=TEST_RUN_COLUMNS,
                                lineterminator="\n")
        if new_file:
            writer.writeheader()
        writer.writerow(record)
        handle.flush()
        os.fsync(handle.fileno())


def choose_test_ledger(logs: Path) -> Path:
    """Never append a wider schema into an old CSV header.

    Existing test evidence is preserved verbatim.  When an earlier recorder
    created ``test_runs.csv`` with fewer columns, new records go to v2.
    """
    legacy = logs / "test_runs.csv"
    if not legacy.exists():
        return legacy
    try:
        with legacy.open(newline="", encoding="utf-8") as handle:
            header = next(csv.reader(handle), [])
    except OSError:
        return logs / "test_runs_v2.csv"
    return legacy if header == TEST_RUN_COLUMNS else logs / "test_runs_v2.csv"


def load_test_context(path: Path) -> dict[str, str]:
    """Read an optional operator-controlled test label without trusting it for safety."""
    try:
        context = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}
    if not isinstance(context, dict) or context.get("state") != "armed_for_operator_start":
        return {}
    required = ("test_plan_id", "test_case_id", "release_position_cm")
    if any(not isinstance(context.get(key), (str, int, float)) for key in required):
        return {}
    return {key: str(context[key]) for key in required}


def test_run_start(session_run_id: str, source_csv: str,
                   configuration_version: str, row_index: int,
                   row: list[str], test_context: dict[str, str]) -> dict[str, str | int]:
    """Create one immutable test identity at the first ACTIVE telemetry row.

    Row indices are one-based data-row indices in ``source_csv``.  They make
    the test-to-CSV relation robust even when a target reset makes pc_ms wrap.
    """
    return {
        "test_run_id": "test_" + uuid.uuid4().hex[:12],
        "session_run_id": session_run_id,
        "source_csv": source_csv,
        "configuration_version": configuration_version,
        "test_plan_id": test_context.get("test_plan_id", ""),
        "test_case_id": test_context.get("test_case_id", ""),
        "release_position_cm": test_context.get("release_position_cm", ""),
        "start_wall_time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "start_row_index": row_index,
        "start_pc_ms": int(row[0]),
        "start_x": int(row[3]),
        "target_x": int(row[6]),
        "start_motor_pos": int(row[11]),
    }


def test_run_finish(record: dict[str, str | int], row_index: int,
                    row: list[str]) -> dict[str, str | int]:
    completed = dict(record)
    completed.update({
        "end_wall_time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "end_row_index": row_index,
        "end_pc_ms": int(row[0]),
        "end_x": int(row[3]),
        "end_motor_pos": int(row[11]),
        "end_state": row[-1],
    })
    return completed


def main() -> int:
    parser = argparse.ArgumentParser(
        description="保存 TI XDS110 诊断 CSV；Ctrl-C 自动刷新并关闭文件。"
    )
    parser.add_argument("--port", default="auto", help="串口路径，默认自动识别 XDS110")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--logs", type=Path, default=Path("logs"))
    parser.add_argument("--context-file", type=Path,
                        help="可选的放球测试计划上下文 JSON；只用于日志标签，不会触发电机")
    args = parser.parse_args()

    port = choose_port(args.port, args.baud)
    args.logs.mkdir(parents=True, exist_ok=True)
    output = args.logs / datetime.now().strftime("run_%Y%m%d_%H%M%S.csv")
    metadata_output = output.with_suffix(".meta.json")
    test_ledger = choose_test_ledger(args.logs)
    context_file = args.context_file or (args.logs / "active_test_context.json")
    session_run_id = "session_" + uuid.uuid4().hex[:12]
    started_at = datetime.now().astimezone().isoformat(timespec="seconds")
    metadata: dict[str, object] = {
        "schema": "ball_beam_test_run/v1",
        "session_run_id": session_run_id,
        "csv": output.name,
        "started_at": started_at,
        "serial_port": port,
        "baud": args.baud,
        **source_snapshot(),
    }
    write_metadata(metadata_output, metadata)
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    accepted = 0
    rejected = 0
    active_test: dict[str, str | int] | None = None
    last_accepted_row: list[str] | None = None
    print(f"开始记录: {port} @ {args.baud} -> {output}", flush=True)
    print(f"测试台账: {test_ledger}", flush=True)
    try:
        with serial.Serial(port, args.baud, timeout=0.2) as device, \
                output.open("w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle, lineterminator="\n")
            writer.writerow(CSV_COLUMNS)
            handle.flush()
            previous_state = None
            while not STOP_REQUESTED:
                try:
                    raw = device.readline()
                except serial.SerialException as error:
                    # Flash/reset temporarily removes the XDS110 CDC device.
                    # Preserve the CSV and finish cleanly instead of hiding
                    # the valid run behind a Python traceback.
                    print(f"串口已断开，保存当前记录: {error}", flush=True)
                    break
                if not raw:
                    continue
                row = valid_row(raw.decode("ascii", errors="replace").strip())
                if row is None:
                    rejected += 1
                    continue
                writer.writerow(row)
                handle.flush()
                accepted += 1
                state = row[-1]
                if (state == "ACTIVE") and (active_test is None):
                    active_test = test_run_start(
                        session_run_id, output.name,
                        str(metadata["configuration_version"]), accepted, row,
                        load_test_context(context_file))
                    print(
                        "测试开始: "
                        f"{active_test['test_run_id']}  "
                        f"x0={active_test['start_x']}  "
                        f"target={active_test['target_x']}",
                        flush=True,
                    )
                elif (state != "ACTIVE") and (active_test is not None):
                    completed = test_run_finish(active_test, accepted, row)
                    append_test_run(test_ledger, completed)
                    print(
                        "测试结束: "
                        f"{completed['test_run_id']}  "
                        f"end={completed['end_state']}",
                        flush=True,
                    )
                    active_test = None
                if state != previous_state:
                    print(
                        "状态: "
                        f"{state}  x={row[3]}  cmd={row[9]}  "
                        f"target={row[10]}  pos={row[11]}",
                        flush=True,
                    )
                    previous_state = state
                last_accepted_row = row
    finally:
        # ``with`` has closed the serial port and CSV handle.  fsync is useful
        # when Ctrl-C follows a test failure or an emergency stop.
        if output.exists():
            with output.open("a", encoding="utf-8") as handle:
                handle.flush()
                os.fsync(handle.fileno())
        metadata.update({
            "stopped_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "accepted_rows": accepted,
            "rejected_rows": rejected,
        })
        if (active_test is not None) and (last_accepted_row is not None):
            completed = test_run_finish(active_test, accepted, last_accepted_row)
            completed["end_state"] = "RECORDER_STOPPED"
            append_test_run(test_ledger, completed)
        write_metadata(metadata_output, metadata)
        print(
            f"记录停止: {output}（有效 {accepted} 行，忽略 {rejected} 行）",
            flush=True,
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
