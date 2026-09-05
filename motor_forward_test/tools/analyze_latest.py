#!/usr/bin/env python3
"""Plot and summarize the newest closed-loop diagnostic CSV in logs/."""

from __future__ import annotations

import argparse
import csv
import json
import statistics
from pathlib import Path

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
except ImportError as error:  # pragma: no cover - depends on host setup
    raise SystemExit(
        "缺少 matplotlib；请先运行: python3 -m pip install -r tools/requirements.txt"
    ) from error


REQUIRED_COLUMNS = (
    "pc_ms,frame,k230_ms,x,confidence,age_ms,target_x,v,e,cmd,"
    "target_motor_pos,estimated_motor_pos,step_freq,state"
).split(",")
STATE_VALUE = {
    "LIMIT_FAULT": 0, "LOST": 1, "WAIT_LEVEL": 2, "JOG_LEFT": 3,
    "JOG_RIGHT": 4, "JOG_STOP": 5, "PRESET_LIFT": 6,
    "PRESET_REFUSED": 7, "ZERO_ACCEPTED": 8, "LEVEL_READY": 9,
    "RETURN_LEVEL": 10, "ACTIVE": 11,
    "HOLD": 10, "BRAKE": 11, "ACCEL": 12,
    "TARGET_REJECTED_UNREFERENCED": 13,
    "TARGET_REJECTED_LIMIT": 14,
    "TARGET_REJECTED_BUSY": 15,
    "TARGET_REJECTED_FAULT": 16,
}
SETTLE_TOLERANCE_PX = 5
SETTLE_DWELL_MS = 1000
OSCILLATION_DEADBAND_PX = 5


def newest_csv(logs: Path) -> Path:
    candidates = [path for path in logs.glob("run_*.csv") if path.is_file()]
    if not candidates:
        raise SystemExit(f"没有找到 {logs}/run_*.csv")
    return max(candidates, key=lambda path: path.stat().st_mtime)


def load_rows(path: Path) -> list[dict[str, int | str]]:
    with path.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames != REQUIRED_COLUMNS:
            raise SystemExit(f"CSV 表头不匹配: {path}")
        rows: list[dict[str, int | str]] = []
        for line_number, row in enumerate(reader, start=2):
            try:
                parsed: dict[str, int | str] = {
                    key: (row[key] if key == "state" else int(row[key]))
                    for key in REQUIRED_COLUMNS
                }
            except (KeyError, TypeError, ValueError) as error:
                raise SystemExit(f"第 {line_number} 行不是有效诊断数据") from error
            if parsed["state"] not in STATE_VALUE:
                raise SystemExit(f"第 {line_number} 行状态无效: {parsed['state']}")
            rows.append(parsed)
    if not rows:
        raise SystemExit(f"日志没有数据行: {path}")
    return rows


def positive_deltas(values: list[int]) -> list[int]:
    return [later - earlier for earlier, later in zip(values, values[1:]) if later > earlier]


def estimate_dropped_frames(rows: list[dict[str, int | str]]) -> int:
    frames = [int(row["frame"]) for row in rows]
    k230_ms = [int(row["k230_ms"]) for row in rows]
    frame_deltas = positive_deltas(frames)
    time_deltas = positive_deltas(k230_ms)
    if not frame_deltas or not time_deltas:
        return 0
    periods = [dt / df for dt, df in zip(
        [b - a for a, b in zip(k230_ms, k230_ms[1:])],
        [b - a for a, b in zip(frames, frames[1:])]
    ) if dt > 0 and df > 0]
    if not periods:
        return 0
    frame_period_ms = sorted(periods)[len(periods) // 2]
    missing = 0
    for dt, observed in zip(
            [b - a for a, b in zip(k230_ms, k230_ms[1:])],
            [b - a for a, b in zip(frames, frames[1:])]):
        if dt <= 0 or observed <= 0:
            continue
        expected = max(1, round(dt / frame_period_ms))
        missing += max(0, observed - expected)
    return missing


def value_or_na(value: int | float | None, digits: int = 3) -> str:
    return "N/A" if value is None else f"{value:.{digits}f}"


def vision_intervals(rows: list[dict[str, int | str]]) -> list[int]:
    """One interval per new K230 frame; repeated TI log rows are ignored."""
    samples: list[tuple[int, int]] = []
    last_frame: int | None = None
    for row in rows:
        if str(row["state"]) == "LOST":
            continue
        frame, stamp = int(row["frame"]), int(row["k230_ms"])
        if last_frame is None or frame != last_frame:
            samples.append((frame, stamp))
            last_frame = frame
    return [later_stamp - earlier_stamp for (_, earlier_stamp), (_, later_stamp)
            in zip(samples, samples[1:]) if later_stamp > earlier_stamp]


def count_oscillations(active: list[dict[str, int | str]]) -> int:
    """Count meaningful error sign reversals, suppressing ±deadband noise."""
    sign = 0
    reversals = 0
    for row in active:
        error = int(row["e"])
        next_sign = 1 if error > OSCILLATION_DEADBAND_PX else -1 if error < -OSCILLATION_DEADBAND_PX else 0
        if next_sign and sign and next_sign != sign:
            reversals += 1
        if next_sign:
            sign = next_sign
    return reversals


def first_setpoint_transition(active: list[dict[str, int | str]]) -> int | None:
    for index, (previous, current) in enumerate(zip(active, active[1:]), start=1):
        if int(current["target_x"]) != int(previous["target_x"]):
            return index
    return None


def response_start(active: list[dict[str, int | str]]) -> tuple[int, int, int, str] | None:
    """Identify either a target step or an operator release toward a fixed target.

    Release-position tests deliberately keep the controller target constant.
    Their overshoot and settling measures must be referenced to the initial
    ball offset rather than incorrectly reported as N/A.
    """
    transition = first_setpoint_transition(active)
    if transition is not None:
        return (transition, int(active[transition - 1]["target_x"]),
                int(active[transition]["target_x"]), "target_step")
    if not active:
        return None
    initial_x, target = int(active[0]["x"]), int(active[0]["target_x"])
    if abs(initial_x - target) > SETTLE_TOLERANCE_PX:
        return (0, initial_x, target, "release_to_fixed_target")
    return None


def closed_loop_metrics(rows: list[dict[str, int | str]]) -> dict[str, object]:
    active = [row for row in rows if str(row["state"]) == "ACTIVE"]
    fault_states = {"LOST", "LIMIT_FAULT", "TARGET_REJECTED_UNREFERENCED",
                    "TARGET_REJECTED_LIMIT", "TARGET_REJECTED_BUSY", "TARGET_REJECTED_FAULT"}
    faults = [str(row["state"]) for row in rows if str(row["state"]) in fault_states]
    intervals = vision_intervals(rows)
    active_duration_ms = ((int(active[-1]["pc_ms"]) - int(active[0]["pc_ms"]))
                          if len(active) > 1 else 0)
    iae = 0.0
    for previous, current in zip(active, active[1:]):
        dt_s = (int(current["pc_ms"]) - int(previous["pc_ms"])) / 1000.0
        if 0 < dt_s <= 0.25:
            iae += abs(int(current["e"])) * dt_s
    motor_moved = any(abs(int(row["step_freq"])) > 0 for row in active)
    response = response_start(active)
    step_metrics: dict[str, object] = {
        "setpoint_transition": "not_present",
        "maximum_overshoot_px": None,
        "settling_time_s": None,
        "steady_state_error_px": None,
    }
    if response is not None:
        start_index, initial_x, target, response_kind = response
        test = active[start_index:]
        direction = 1 if target > initial_x else -1
        overshoots = [direction * (int(row["x"]) - target) for row in test]
        step_metrics["setpoint_transition"] = {
            "kind": response_kind,
            "at_pc_ms": int(test[0]["pc_ms"]), "from_px": initial_x, "to_px": target,
        }
        step_metrics["maximum_overshoot_px"] = max(0, max(overshoots))
        for index, row in enumerate(test):
            until = int(row["pc_ms"]) + SETTLE_DWELL_MS
            window = [candidate for candidate in test[index:] if int(candidate["pc_ms"]) <= until]
            if window and int(window[-1]["pc_ms"]) >= until and all(abs(int(candidate["e"])) <= SETTLE_TOLERANCE_PX for candidate in window):
                step_metrics["settling_time_s"] = (int(row["pc_ms"]) - int(test[0]["pc_ms"])) / 1000.0
                break
        final_window_start = int(test[-1]["pc_ms"]) - 2000
        final_window = [row for row in test if int(row["pc_ms"]) >= final_window_start]
        if final_window:
            step_metrics["steady_state_error_px"] = sum(int(row["e"]) for row in final_window) / len(final_window)
    credible = bool(active) and active_duration_ms >= 3000 and motor_moved and not faults
    return {
        "active_samples": len(active),
        "active_duration_s": active_duration_ms / 1000.0,
        "motor_execution_observed": motor_moved,
        "fault_states": sorted(set(faults)),
        "position_iae_px_s": iae if active else None,
        "control_output_peak_mdeg": max((abs(int(row["cmd"])) for row in active), default=None),
        "oscillation_reversals": count_oscillations(active) if active else None,
        "vision_mean_period_ms": statistics.fmean(intervals) if intervals else None,
        "vision_max_interval_ms": max(intervals) if intervals else None,
        "vision_min_interval_ms": min(intervals) if intervals else None,
        "tuning_eligible": False,
        "test_score_eligible": credible and response is not None,
        "test_score_reason": ("TUNING_LOCKED: requires manual safety verification; no PID changes are permitted"
                              if credible else "diagnostic only: needs >=3 s ACTIVE motor execution with no safety fault"),
        **step_metrics,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="分析 logs/ 中最新的闭环 CSV")
    parser.add_argument("--logs", type=Path, default=Path("logs"))
    parser.add_argument("--output", type=Path, default=Path("analysis"))
    args = parser.parse_args()

    source = newest_csv(args.logs)
    rows = load_rows(source)
    args.output.mkdir(parents=True, exist_ok=True)
    stem = source.stem
    plot_path = args.output / f"{stem}_analysis.png"
    summary_path = args.output / f"{stem}_summary.txt"
    score_path = args.output / f"{stem}_score.json"

    t0 = int(rows[0]["pc_ms"])
    time_s = [(int(row["pc_ms"]) - t0) / 1000.0 for row in rows]
    get = lambda name: [int(row[name]) for row in rows]
    states = [str(row["state"]) for row in rows]
    pc_delta = [None] + [later - earlier for earlier, later in zip(get("pc_ms"), get("pc_ms")[1:])]
    k230_delta = [None] + [later - earlier for earlier, later in zip(get("k230_ms"), get("k230_ms")[1:])]
    dropped_frames = estimate_dropped_frames(rows)
    lost_samples = sum(state == "LOST" for state in states)
    metrics = closed_loop_metrics(rows)
    metadata_path = source.with_suffix(".meta.json")
    metadata: dict[str, object] = {}
    if metadata_path.is_file():
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            metadata = {"metadata_error": "invalid JSON"}

    figure, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True, constrained_layout=True)
    axes[0].plot(time_s, get("x"), label="ball x", color="#1f77b4")
    axes[0].plot(time_s, get("target_x"), label="target x", color="#d62728", linestyle="--")
    axes[0].set_ylabel("pixel offset")
    axes[0].set_title(f"Ball closed-loop diagnostic — {source.name}")
    axes[0].legend(loc="best")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(time_s, get("v"), color="#9467bd", label="estimated velocity")
    axes[1].axhline(0, color="black", linewidth=0.7)
    axes[1].set_ylabel("px/s")
    axes[1].set_title("Estimated ball velocity")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(time_s, get("cmd"), color="#17becf", label="command (mdeg)")
    state_axis = axes[2].twinx()
    state_axis.step(time_s, [STATE_VALUE[state] for state in states], where="post", color="#333333", alpha=0.75, label="state")
    state_axis.set_ylim(-0.3, max(STATE_VALUE.values()) + 0.3)
    state_axis.set_yticks(list(STATE_VALUE.values()), list(STATE_VALUE.keys()))
    axes[2].set_ylabel("beam command (mdeg)")
    axes[2].set_title("Control output and state")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(time_s, k230_delta, label="K230 frame interval", color="#2ca02c")
    axes[3].plot(time_s, get("age_ms"), label="data age", color="#ff7f0e")
    axes[3].plot(time_s, pc_delta, label="TI log interval", color="#7f7f7f", linestyle=":")
    axes[3].set_xlabel("time since first log row (s)")
    axes[3].set_ylabel("milliseconds")
    axes[3].set_title("Frame interval and data age")
    axes[3].legend(loc="best")
    axes[3].grid(True, alpha=0.3)
    figure.savefig(plot_path, dpi=160)
    plt.close(figure)

    duration_s = time_s[-1] if len(time_s) > 1 else 0.0
    age_values = [value for value in get("age_ms") if value >= 0]
    summary = [
        f"source_csv={source}",
        f"rows={len(rows)}",
        f"duration_s={duration_s:.3f}",
        f"lost_samples={lost_samples}",
        f"lost_ratio={lost_samples / len(rows):.4f}",
        f"estimated_dropped_frames={dropped_frames}",
        f"max_data_age_ms={max(age_values) if age_values else -1}",
        f"mean_data_age_ms={sum(age_values) / len(age_values):.2f}" if age_values else "mean_data_age_ms=-1",
        f"configuration_version={metadata.get('configuration_version', 'unknown (record predates metadata)')}",
        f"active_samples={metrics['active_samples']}",
        f"active_duration_s={value_or_na(float(metrics['active_duration_s']))}",
        f"motor_execution_observed={metrics['motor_execution_observed']}",
        f"fault_states={','.join(metrics['fault_states']) or 'none'}",
        f"maximum_overshoot_px={value_or_na(metrics['maximum_overshoot_px'])}",
        f"settling_time_s={value_or_na(metrics['settling_time_s'])}",
        f"steady_state_error_px={value_or_na(metrics['steady_state_error_px'])}",
        f"oscillation_reversals={metrics['oscillation_reversals'] if metrics['oscillation_reversals'] is not None else 'N/A'}",
        f"position_iae_px_s={value_or_na(metrics['position_iae_px_s'])}",
        f"control_output_peak_mdeg={metrics['control_output_peak_mdeg'] if metrics['control_output_peak_mdeg'] is not None else 'N/A'}",
        f"vision_mean_period_ms={value_or_na(metrics['vision_mean_period_ms'], 2)}",
        f"vision_max_interval_ms={metrics['vision_max_interval_ms'] if metrics['vision_max_interval_ms'] is not None else 'N/A'}",
        f"test_score_eligible={metrics['test_score_eligible']}",
        f"tuning_eligible=false",
        f"test_score_reason={metrics['test_score_reason']}",
        f"plot={plot_path}",
    ]
    summary_path.write_text("\n".join(summary) + "\n", encoding="utf-8")
    score = {
        "schema": "ball_beam_test_score/v1",
        "source_csv": str(source),
        "metadata": str(metadata_path) if metadata_path.is_file() else None,
        "configuration_version": metadata.get("configuration_version"),
        "estimated_dropped_frames": dropped_frames,
        "lost_ratio": lost_samples / len(rows),
        "max_data_age_ms": max(age_values) if age_values else None,
        "mean_data_age_ms": (sum(age_values) / len(age_values)) if age_values else None,
        "metrics": metrics,
        "tuning_eligible": False,
        "tuning_lock_reason": "Hardware safety validation has not been recorded; automatic PID editing is disabled.",
    }
    score_path.write_text(json.dumps(score, ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                          encoding="utf-8")
    print(f"分析完成: {plot_path}")
    print(f"统计完成: {summary_path}")
    print(f"评分记录: {score_path}（调参锁定）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
