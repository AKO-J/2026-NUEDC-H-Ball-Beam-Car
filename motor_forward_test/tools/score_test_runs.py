#!/usr/bin/env python3
"""Score completed ACTIVE intervals from logs/test_runs.csv.

This command is deliberately diagnostic-only.  It emits evidence linked by
test_run_id, source CSV and configuration version; it never writes firmware or
changes PID values.
"""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

from analyze_latest import closed_loop_metrics, estimate_dropped_frames, load_rows


def choose_test_ledger(logs: Path) -> Path:
    """Prefer the newest compatible ledger without rewriting older evidence."""
    required = {"test_run_id", "source_csv", "configuration_version",
                "start_row_index", "end_row_index", "end_state"}
    candidates = [logs / "test_runs_v2.csv", logs / "test_runs.csv"]
    for ledger in candidates:
        if not ledger.is_file():
            continue
        with ledger.open(newline="", encoding="utf-8") as handle:
            header = set(next(csv.reader(handle), []))
        if required <= header:
            return ledger
    return logs / "test_runs.csv"


def completed_records(ledger: Path) -> list[dict[str, str]]:
    if not ledger.is_file():
        raise SystemExit(f"没有测试台账: {ledger}；请用新的 record_serial.py 重新开始记录。")
    with ledger.open(newline="", encoding="utf-8") as handle:
        records = list(csv.DictReader(handle))
    required = {"test_run_id", "source_csv", "configuration_version",
                "start_row_index", "end_row_index", "end_state"}
    if not records:
        return []
    missing = required - set(records[0])
    if missing:
        raise SystemExit("测试台账格式过旧，需重启记录器后生成新测试：缺少 " + ", ".join(sorted(missing)))
    return [record for record in records if record["end_row_index"]]


def slice_rows(rows: list[dict[str, int | str]], record: dict[str, str]) -> list[dict[str, int | str]]:
    start, end = int(record["start_row_index"]), int(record["end_row_index"])
    if start < 1 or end < start or end > len(rows):
        raise ValueError(f"CSV 行范围无效: {start}..{end}，文件有 {len(rows)} 行")
    return rows[start - 1:end]


def score_record(logs: Path, output: Path, record: dict[str, str]) -> Path:
    source = logs / record["source_csv"]
    if not source.is_file():
        raise ValueError(f"原始 CSV 不存在: {source}")
    selected = slice_rows(load_rows(source), record)
    metrics = closed_loop_metrics(selected)
    states = [str(row["state"]) for row in selected]
    score = {
        "schema": "ball_beam_test_score/v2",
        "test_run_id": record["test_run_id"],
        "session_run_id": record["session_run_id"],
        "source_csv": str(source),
        "configuration_version": record["configuration_version"],
        "test_plan_id": record.get("test_plan_id") or None,
        "test_case_id": record.get("test_case_id") or None,
        "release_position_cm": (float(record["release_position_cm"])
                                if record.get("release_position_cm") else None),
        "row_range": {"start": int(record["start_row_index"]), "end": int(record["end_row_index"])},
        "start": {"wall_time": record["start_wall_time"], "pc_ms": int(record["start_pc_ms"]),
                  "x": int(record["start_x"]), "target_x": int(record["target_x"]),
                  "motor_pos": int(record["start_motor_pos"])},
        "end": {"wall_time": record["end_wall_time"], "pc_ms": int(record["end_pc_ms"]),
                "x": int(record["end_x"]), "motor_pos": int(record["end_motor_pos"]),
                "state": record["end_state"]},
        "telemetry": {
            "samples": len(selected),
            "estimated_dropped_frames": estimate_dropped_frames(selected),
            "lost_samples": sum(state == "LOST" for state in states),
        },
        "metrics": metrics,
        "tuning_eligible": False,
        "tuning_lock_reason": "Safety validation is incomplete; this score is diagnostic evidence only and cannot modify PID.",
    }
    output.mkdir(parents=True, exist_ok=True)
    destination = output / f"{record['test_run_id']}_score.json"
    destination.write_text(json.dumps(score, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                           encoding="utf-8")
    return destination


def main() -> int:
    parser = argparse.ArgumentParser(description="按 test_run_id 评分闭环测试（仅诊断，PID 锁定）")
    parser.add_argument("--logs", type=Path, default=Path("logs"))
    parser.add_argument("--output", type=Path, default=Path("analysis"))
    parser.add_argument("--test-run-id", help="仅评分这个测试 ID；默认评分所有完成测试")
    args = parser.parse_args()
    ledger = choose_test_ledger(args.logs)
    records = completed_records(ledger)
    print(f"使用测试台账: {ledger}")
    if args.test_run_id:
        records = [record for record in records if record["test_run_id"] == args.test_run_id]
        if not records:
            raise SystemExit(f"台账中没有完成的测试: {args.test_run_id}")
    if not records:
        print("没有已完成测试；尚未产生 ACTIVE→非 ACTIVE 的独立测试记录。")
        return 0
    failures = 0
    for record in records:
        try:
            destination = score_record(args.logs, args.output, record)
            print(f"评分记录: {destination}（PID 调参锁定）")
        except (OSError, ValueError) as error:
            failures += 1
            print(f"无法评分 {record['test_run_id']}: {error}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
