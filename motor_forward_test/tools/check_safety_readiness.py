#!/usr/bin/env python3
"""Maintain an evidence-backed manual safety-validation checklist.

This tool is intentionally unable to unlock or edit PID configuration.  It
only records which hardware checks a human has attested with a local evidence
file, so diagnostics can clearly distinguish missing proof from a pass.
"""

from __future__ import annotations

import argparse
import hashlib
import json
from datetime import datetime
from pathlib import Path


GATES = {
    "motor_execution": "电机执行：ACTIVE 时观察到真实步进运动，停止后脉冲归零。",
    "level_zero": "水平零点：人工确认水平、保存零点、回水平动作均正确。",
    "travel_limits": "限位：两个软行程端点和机械限位/急停按硬件规程验证。",
    "ball_loss": "丢球保护：遮挡/移走小球进入 LOST 且停止脉冲。",
    "log_credibility": "日志可信度：测试 ID、CSV、版本、评分关联且视觉周期/丢帧可信。",
}
SCHEMA = "ball_beam_safety_validation/v1"


def load(path: Path) -> dict[str, object]:
    if not path.is_file():
        return {"schema": SCHEMA, "validations": {}}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as error:
        raise SystemExit(f"安全验证台账不是有效 JSON: {path}") from error
    if data.get("schema") != SCHEMA or not isinstance(data.get("validations"), dict):
        raise SystemExit(f"安全验证台账格式不匹配: {path}")
    return data


def evidence_digest(path: Path) -> str:
    if not path.is_file():
        raise SystemExit(f"证据文件不存在: {path}")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def status(data: dict[str, object]) -> int:
    validations = data["validations"]
    assert isinstance(validations, dict)
    for key, description in GATES.items():
        entry = validations.get(key)
        if isinstance(entry, dict):
            print(f"PASS-RECORDED  {key}: {entry.get('evidence_path')}")
        else:
            print(f"MISSING        {key}: {description}")
    complete = all(isinstance(validations.get(key), dict) for key in GATES)
    print("\n状态: " + ("证据已记录，仍需人工审查；PID 自动修改仍禁用。" if complete
                           else "安全证据未齐全；仅允许诊断，不允许 PID 修改。"))
    return 0 if complete else 2


def main() -> int:
    parser = argparse.ArgumentParser(description="显示/记录闭环平台安全验证证据（不会解锁 PID）")
    parser.add_argument("--logs", type=Path, default=Path("logs"))
    parser.add_argument("--confirm", choices=sorted(GATES), help="记录一个人工验证项目")
    parser.add_argument("--evidence-file", type=Path, help="确认时必填：对应 CSV、评分或验证说明文件")
    parser.add_argument("--note", help="简短人工观察说明；不能代替 evidence-file")
    args = parser.parse_args()
    if args.confirm and not args.evidence_file:
        parser.error("--confirm 必须同时提供 --evidence-file")
    if args.evidence_file and not args.confirm:
        parser.error("--evidence-file 只能与 --confirm 一起使用")
    args.logs.mkdir(parents=True, exist_ok=True)
    ledger = args.logs / "safety_validation.json"
    data = load(ledger)
    if args.confirm:
        validations = data["validations"]
        assert isinstance(validations, dict)
        evidence = args.evidence_file.resolve()
        validations[args.confirm] = {
            "recorded_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "evidence_path": str(evidence),
            "evidence_sha256": evidence_digest(evidence),
            "note": args.note or "",
        }
        ledger.write_text(json.dumps(data, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                          encoding="utf-8")
        print(f"已记录 {args.confirm} 的证据: {ledger}")
    return status(data)


if __name__ == "__main__":
    raise SystemExit(main())
