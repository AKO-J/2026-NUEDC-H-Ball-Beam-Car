#!/usr/bin/env python3
"""Operator-authorized release-position test sequence for the ball-beam rig.

This helper never opens the control UART and never commands a motor.  It only
creates a durable test plan and an optional context label consumed by the
separate CSV recorder after the operator presses the physical START button.
"""

from __future__ import annotations

import argparse
import csv
import json
import uuid
from datetime import datetime
from pathlib import Path

from score_test_runs import choose_test_ledger


def parse_positions(text: str) -> list[float]:
    try:
        values = [float(value.strip()) for value in text.split(",")]
    except ValueError as error:
        raise argparse.ArgumentTypeError("位置必须是逗号分隔的厘米数，例如 -1,-2,-3") from error
    if not values or any(abs(value) > 100 for value in values):
        raise argparse.ArgumentTypeError("至少给出一个合理的放球位置（单位 cm，范围 ±100）")
    return values


def write_json(path: Path, content: dict[str, object]) -> None:
    path.write_text(json.dumps(content, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8")


def matching_completed_run(ledger: Path, plan_id: str, case_id: str) -> str | None:
    if not ledger.is_file():
        return None
    with ledger.open(newline="", encoding="utf-8") as handle:
        for row in reversed(list(csv.DictReader(handle))):
            if row.get("test_plan_id") == plan_id and row.get("test_case_id") == case_id:
                return row.get("test_run_id") or None
    return None


def main() -> int:
    parser = argparse.ArgumentParser(
        description="按固定放球位置编排测试；只等待实体 START，不发送任何电机命令。"
    )
    parser.add_argument("--positions-cm", type=parse_positions, default=parse_positions("-1,-2,-3"),
                        help="放球位置序列，单位 cm；默认 -1,-2,-3")
    parser.add_argument("--logs", type=Path, default=Path("logs"))
    args = parser.parse_args()
    args.logs.mkdir(parents=True, exist_ok=True)
    plan_id = "release_plan_" + uuid.uuid4().hex[:12]
    created_at = datetime.now().astimezone().isoformat(timespec="seconds")
    plan = {
        "schema": "ball_beam_release_sequence/v1",
        "test_plan_id": plan_id,
        "created_at": created_at,
        "positions_cm": args.positions_cm,
        "safety": [
            "No UART motor command is sent by this tool.",
            "Each run requires the physical START button.",
            "Use the existing horizontal-return control after every run.",
            "PID/configuration values are not modified.",
        ],
    }
    plan_path = args.logs / f"{plan_id}.json"
    context_path = args.logs / "active_test_context.json"
    write_json(plan_path, plan)
    print(f"测试计划: {plan_path}")
    print("请确认记录器已在另一个终端运行。Ctrl-C 可安全结束本编排；不会影响已写入的测试记录。")
    try:
        for index, position in enumerate(args.positions_cm, start=1):
            case_id = f"release_{index:02d}_{position:g}cm"
            context = {
                "schema": "ball_beam_active_test_context/v1",
                "state": "armed_for_operator_start",
                "test_plan_id": plan_id,
                "test_case_id": case_id,
                "release_position_cm": position,
                "armed_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            }
            write_json(context_path, context)
            input(f"\n第 {index}/{len(args.positions_cm)} 轮：请将球放在 {position:g} cm；准备好后按 Enter。")
            print("现在仅可由你按实体 START 开始。本轮结束后，请用现有安全流程回到水平。")
            input("确认本轮已经停止并回到水平后按 Enter，继续下一轮。")
            run_id = matching_completed_run(choose_test_ledger(args.logs), plan_id, case_id)
            if run_id:
                print(f"已关联测试记录: {run_id}")
            else:
                print("警告：尚未在台账中看到本轮记录；请检查记录器仍在运行，再继续。")
        write_json(context_path, {"schema": "ball_beam_active_test_context/v1", "state": "idle"})
        print("序列完成。运行 tools/score_test_runs.py 生成逐轮评分；PID 仍锁定。")
    except KeyboardInterrupt:
        write_json(context_path, {"schema": "ball_beam_active_test_context/v1", "state": "idle"})
        print("\n序列已取消；未发送电机命令，PID 未修改。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
