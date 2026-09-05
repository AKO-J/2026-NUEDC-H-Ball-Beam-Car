#!/usr/bin/env python3
"""Interactive, read-only evidence collection for vision safety checks.

This program opens only the XDS110 diagnostic UART (the same 115200-baud
output consumed by record_serial.py).  It never opens the K230 control UART,
never transmits bytes, and never writes controller/PID source files.  Each
Enter-delimited operator check is saved as a separate CSV and JSON evidence
pair under logs/.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import queue
import signal
import sys
import threading
import time
import uuid
from collections import Counter
from datetime import datetime
from pathlib import Path

from record_serial import CSV_COLUMNS, choose_port, valid_row


CASES = (
    (
        "vision_stable",
        "视觉连续性",
        "让球保持在 ROI 内静止或缓慢移动至少 10 秒；不要按 START 进入自动控制。",
        "期望：frame 与 k230_ms 单调递增，age_ms 小于 150，且没有 LOST。",
    ),
    (
        "ball_occlusion",
        "遮挡丢球",
        "球在画面中时，用手/不透明物完全遮挡镜头或球至少 2 秒，然后移开。",
        "期望：出现 LOST；若此前为 ACTIVE，step_freq 应归零且离开 ACTIVE。",
    ),
    (
        "ball_removed",
        "移走小球",
        "球在画面中时，将球移出 ROI 至少 2 秒，然后放回。",
        "期望：出现 LOST；不得继续使用旧位置驱动机构。",
    ),
    (
        "uart_timeout",
        "通信超时",
        "仅在机构已安全停稳时，断开 K230→MSPM0 的 TX 线至少 1 秒后重新接回。",
        "期望：数据年龄超过 150 ms 后进入 LOST；重新接线不应自动恢复 ACTIVE。",
    ),
)
STOP_REQUESTED = False


def request_stop(_signum: int, _frame: object) -> None:
    global STOP_REQUESTED
    STOP_REQUESTED = True


def now_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


def write_json(path: Path, value: dict[str, object]) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8")


def summarize(rows: list[list[str]], rejected_lines: int) -> dict[str, object]:
    """Produce evidence facts, not a pass/fail or any tuning decision."""
    if not rows:
        return {"valid_rows": 0, "rejected_pc_lines": rejected_lines}
    fields = [dict(zip(CSV_COLUMNS, row)) for row in rows]
    frames = [int(row["frame"]) for row in fields]
    stamps = [int(row["k230_ms"]) for row in fields]
    ages = [int(row["age_ms"]) for row in fields if int(row["age_ms"]) >= 0]
    states = [row["state"] for row in fields]
    frame_backtracks = sum(later < earlier for earlier, later in zip(frames, frames[1:]))
    stamp_backtracks = sum(later < earlier for earlier, later in zip(stamps, stamps[1:]))
    # TI logs only every 50 ms and deliberately keeps the newest K230 frame.
    # Therefore a frame delta of two can be perfectly normal, not a UART loss.
    # Keep it as an observation for review; only TI RX counters can prove a
    # receiver-byte overflow or parser rejection.
    frame_sequence_skips = sum(max(0, later - earlier - 1)
                               for earlier, later in zip(frames, frames[1:])
                               if later >= earlier)
    lost_indices = [index for index, state in enumerate(states) if state == "LOST"]
    lost_step_nonzero = [index for index in lost_indices if int(fields[index]["step_freq"]) != 0]
    first_lost = lost_indices[0] if lost_indices else None
    return {
        "valid_rows": len(rows),
        "rejected_pc_lines": rejected_lines,
        "pc_duration_ms": int(fields[-1]["pc_ms"]) - int(fields[0]["pc_ms"]),
        "state_counts": dict(Counter(states)),
        "lost_seen": bool(lost_indices),
        "first_lost_row": (first_lost + 1) if first_lost is not None else None,
        "nonzero_step_freq_while_lost_rows": [index + 1 for index in lost_step_nonzero],
        "max_age_ms": max(ages) if ages else None,
        "mean_age_ms": (sum(ages) / len(ages)) if ages else None,
        "frame_backtracks": frame_backtracks,
        "k230_time_backtracks": stamp_backtracks,
        "frame_sequence_skips_at_50ms_log_samples": frame_sequence_skips,
        "warning": ("帧号跳过只表示 50 ms 日志采样期间 TI 选择了较新的 K230 帧；"
                    "它不等价于丢帧，也不等价于 TI RX 丢字节计数。"),
    }


class Reader(threading.Thread):
    def __init__(self, device: object, output: queue.Queue[tuple[float, list[str] | None]]) -> None:
        super().__init__(daemon=True)
        self.device = device
        self.output = output
        self.rejected = 0
        self.running = True

    def run(self) -> None:
        while self.running:
            try:
                raw = self.device.readline()
            except Exception as error:  # serial exception type differs by platform
                print(f"\n串口读取已停止: {error}", flush=True)
                self.running = False
                return
            if not raw:
                continue
            row = valid_row(raw.decode("ascii", errors="replace").strip())
            if row is None:
                self.rejected += 1
            else:
                self.output.put((time.monotonic(), row))


def drain(samples: queue.Queue[tuple[float, list[str] | None]], destination: list[list[str]]) -> None:
    while True:
        try:
            _received_at, row = samples.get_nowait()
        except queue.Empty:
            return
        if row is not None:
            destination.append(row)


def wait_for_enter(result: queue.Queue[bool]) -> None:
    """Keep stdin blocking off the telemetry-display path."""
    try:
        input()
    except (EOFError, KeyboardInterrupt):
        pass
    result.put(True)


def collect_case(case: tuple[str, str, str, str], samples: queue.Queue[tuple[float, list[str] | None]],
                 reader: Reader, output_dir: Path) -> Path | None:
    case_id, title, instruction, expected = case
    print("\n" + "=" * 66)
    print(f"[{title}]\n操作：{instruction}\n观察：{expected}")
    try:
        input("准备好后按 Enter 开始记录；此工具不会发送任何命令：")
    except (EOFError, KeyboardInterrupt):
        return None
    if not reader.running:
        print("串口已经断开或被其他程序占用；本项没有开始，也不会写入空证据。")
        return None
    drain(samples, [])  # discard prior-case backlog at the explicit operator boundary
    rows: list[list[str]] = []
    rejected_start = reader.rejected
    started_at = now_iso()
    print("正在记录。完成动作后按 Enter 结束；终端每秒显示最新状态。")
    enter_result: queue.Queue[bool] = queue.Queue(maxsize=1)
    threading.Thread(target=wait_for_enter, args=(enter_result,), daemon=True).start()
    last_display = 0.0
    while not STOP_REQUESTED:
        drain(samples, rows)
        if not reader.running:
            print("串口在本项中断开；本项作废，不写入证据文件。")
            return None
        if not enter_result.empty():
            break
        monotonic_now = time.monotonic()
        if rows and (monotonic_now - last_display >= 1.0):
            latest = rows[-1]
            print(f"状态 {latest[-1]}  frame={latest[1]}  age={latest[5]}ms  "
                  f"step_freq={latest[12]}")
            last_display = monotonic_now
        time.sleep(0.05)
    drain(samples, rows)
    ended_at = now_iso()
    stem = f"{case_id}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    csv_path = output_dir / f"{stem}.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(CSV_COLUMNS)
        writer.writerows(rows)
        handle.flush()
        os.fsync(handle.fileno())
    summary = summarize(rows, reader.rejected - rejected_start)
    evidence_path = output_dir / f"{stem}.json"
    write_json(evidence_path, {
        "schema": "ball_beam_vision_safety_case/v1",
        "case_id": case_id,
        "title": title,
        "operator_instruction": instruction,
        "expected_observation": expected,
        "started_at": started_at,
        "ended_at": ended_at,
        "source_csv": str(csv_path.resolve()),
        "summary": summary,
        "tuning_eligible": False,
        "safety_notice": "This file is diagnostic evidence only. It cannot unlock or modify PID.",
    })
    print(f"已保存：{csv_path}\n证据摘要：{evidence_path}")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return evidence_path


def main() -> int:
    parser = argparse.ArgumentParser(description="交互式视觉/丢球安全证据采集；只读 TI 诊断串口。")
    parser.add_argument("--port", default="auto", help="XDS110 诊断串口，默认自动识别")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--logs", type=Path, default=Path("logs"))
    parser.add_argument("--cases", default=",".join(case[0] for case in CASES),
                        help="逗号分隔的项目 ID；默认全部")
    args = parser.parse_args()
    selected = {value.strip() for value in args.cases.split(",") if value.strip()}
    unknown = selected - {case[0] for case in CASES}
    if unknown:
        parser.error("未知项目: " + ", ".join(sorted(unknown)))
    cases = [case for case in CASES if case[0] in selected]
    if not cases:
        parser.error("至少选择一个项目")
    output_dir = args.logs / ("vision_safety_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    output_dir.mkdir(parents=True, exist_ok=False)
    port = choose_port(args.port, args.baud)
    try:
        import serial
    except ImportError as error:  # pragma: no cover
        raise SystemExit("缺少 pyserial；请安装 tools/requirements.txt") from error
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    with serial.Serial(port, args.baud, timeout=0.2) as device:
        samples: queue.Queue[tuple[float, list[str] | None]] = queue.Queue()
        reader = Reader(device, samples)
        reader.start()
        print(f"只读连接 {port} @ {args.baud}。输出目录：{output_dir}")
        try:
            _received_at, first_row = samples.get(timeout=2.0)
        except queue.Empty:
            reader.running = False
            raise SystemExit(
                "2 秒内未收到有效 TI CSV；请确认固件正在输出、串口未被 record_serial.py 占用，"
                "然后重试。"
            )
        samples.put((_received_at, first_row))
        evidence_files: list[str] = []
        try:
            for case in cases:
                if STOP_REQUESTED:
                    break
                result = collect_case(case, samples, reader, output_dir)
                if result is None:
                    print("本项未完成，停止后续项目。")
                    break
                evidence_files.append(str(result.resolve()))
        finally:
            reader.running = False
        write_json(output_dir / "session.json", {
            "schema": "ball_beam_vision_safety_session/v1",
            "created_at": now_iso(), "serial_port": port, "baud": args.baud,
            "evidence_files": evidence_files, "tuning_eligible": False,
        })
    print("\n会话结束。请人工复核每个 JSON 与 CSV；不能据此自动调 PID。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
