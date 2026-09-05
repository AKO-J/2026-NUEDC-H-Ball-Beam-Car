#!/usr/bin/env python3
"""Plot and summarize a task-4 low-speed AB baseline CSV."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from record_task4_serial import COLUMNS


def newest(logs: Path) -> Path:
    paths = list(logs.glob("task4_*.csv"))
    if not paths:
        raise SystemExit(f"没有找到 {logs}/task4_*.csv")
    return max(paths, key=lambda p: p.stat().st_mtime)


def main() -> int:
    parser = argparse.ArgumentParser(description="分析第四题低速直线 AB 基线")
    parser.add_argument("csv", nargs="?", type=Path)
    parser.add_argument("--logs", type=Path, default=Path("logs/task4"))
    parser.add_argument("--output", type=Path, default=Path("analysis/task4"))
    args = parser.parse_args()
    source = args.csv or newest(args.logs)
    with source.open(newline="", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        if reader.fieldnames != COLUMNS:
            raise SystemExit(f"CSV v1 表头不匹配: {source}")
        rows = list(reader)
    if not rows:
        raise SystemExit("CSV 没有数据行")
    numeric = {name: [int(row[name]) for row in rows] for name in COLUMNS[:-2]}
    t0 = numeric["pc_ms"][0]
    time_s = [(x - t0) / 1000 for x in numeric["pc_ms"]]
    phases = [row["phase"] for row in rows]
    imu_missing = sum(x == -2147483648 for x in numeric["accel_x_raw"])
    valid_indices = [i for i, row in enumerate(rows)
                     if int(row["confidence"]) >= 320 and
                     int(row["vision_age_ms"]) >= 0]
    valid_errors = [numeric["ball_error_cm_x100"][i] for i in valid_indices]
    max_abs_error = (max(abs(x) for x in valid_errors) / 100
                     if valid_errors else None)
    within = (sum(abs(x) <= 100 for x in valid_errors) / len(valid_errors)
              if valid_errors else None)
    left_distance_cm = abs(numeric["left_encoder_count"][-1]) * 100 / 6445
    right_distance_cm = abs(numeric["right_encoder_count"][-1]) * 100 / 6267
    average_distance_cm = (left_distance_cm + right_distance_cm) / 2
    args.output.mkdir(parents=True, exist_ok=True)
    plot = args.output / f"{source.stem}_analysis.png"
    summary = args.output / f"{source.stem}_summary.txt"
    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True,
                             constrained_layout=True)
    axes[0].plot(time_s, [x / 100 for x in numeric["ball_error_cm_x100"]])
    axes[0].axhspan(-1, 1, color="green", alpha=.12); axes[0].set_ylabel("ball error (cm)")
    axes[1].plot(time_s, numeric["left_speed_counts_s"], label="left")
    axes[1].plot(time_s, numeric["right_speed_counts_s"], label="right"); axes[1].legend()
    axes[1].set_ylabel("encoder counts/s")
    axes[2].plot(time_s, numeric["accel_x_raw"], label="ax")
    axes[2].plot(time_s, numeric["accel_y_raw"], label="ay", alpha=.7)
    axes[2].plot(time_s, numeric["accel_z_raw"], label="az", alpha=.7); axes[2].legend()
    axes[2].set_ylabel("IMU raw")
    axes[3].plot(time_s, numeric["beam_cmd_mdeg"], label="beam mdeg")
    axes[3].step(time_s, numeric["left_pwm"], where="post", label="wheel PWM")
    axes[3].legend(); axes[3].set_xlabel("time (s)")
    for axis in axes: axis.grid(True, alpha=.25)
    fig.suptitle(f"Task 4 AB baseline — {source.name}")
    fig.savefig(plot, dpi=160); plt.close(fig)
    phase_order = []
    for phase in phases:
        if not phase_order or phase != phase_order[-1]: phase_order.append(phase)
    summary_lines = [
        f"source_csv={source}", f"rows={len(rows)}",
        f"duration_s={time_s[-1]:.3f}",
        f"phase_sequence={','.join(phase_order)}",
        f"valid_vision_samples={len(valid_indices)}",
        (f"max_abs_ball_error_cm={max_abs_error:.2f}"
         if max_abs_error is not None else "max_abs_ball_error_cm=N/A"),
        (f"within_1cm_ratio={within:.4f}"
         if within is not None else "within_1cm_ratio=N/A"),
        f"left_distance_cm={left_distance_cm:.2f}",
        f"right_distance_cm={right_distance_cm:.2f}",
        f"average_forward_distance_cm={average_distance_cm:.2f}",
        f"imu_missing_samples={imu_missing}",
        f"safety_states={','.join(sorted(set(row['safety_state'] for row in rows)))}",
        "pid_tuning_eligible=false",
    ]
    summary.write_text("\n".join(summary_lines) + "\n", encoding="utf-8")
    print(f"分析完成: {plot}\n摘要完成: {summary}（PID 调参保持锁定）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
