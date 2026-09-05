#!/usr/bin/env python3
"""Analyze one joint-control CSV and generate the required plot and JSON."""
from __future__ import annotations
import argparse, csv, json
from pathlib import Path
import matplotlib.pyplot as plt

def main() -> int:
    p=argparse.ArgumentParser(); p.add_argument("csv",type=Path); p.add_argument("--output-dir",type=Path,default=Path("analysis/joint")); a=p.parse_args()
    with a.csv.open(newline="") as f: rows=list(csv.DictReader(f))
    if not rows: raise SystemExit("CSV没有有效数据")
    numeric={k:[int(r[k]) for r in rows] for k in rows[0] if k not in ("state","safety_state")}
    active=[i for i,r in enumerate(rows) if int(r["run_id"])>0 and r["state"] in {"RAMP_UP","CRUISE","RAMP_DOWN","STOPPED"}]
    ready=[i for i,r in enumerate(rows) if r["state"]=="READY" and int(r["vision_age_ms"])>=0]
    ball=[abs(numeric["ball_error_cm_x100"][i]) for i in active]
    ready_ball=[abs(numeric["ball_error_cm_x100"][i]) for i in ready]
    faults=[numeric["fault_flags"][i] for i in active]
    ready_tail=ready[-20:]
    summary={"schema":"joint_line_balance_analysis/v1","csv":a.csv.name,
             "active_samples":len(active),"run_ids":sorted({numeric["run_id"][i] for i in active}),
             "max_abs_ball_error_cm":max(ball,default=0)/100,
             "ball_within_1cm":bool(active) and max(ball,default=101)<=100,
             "vision_stale_samples":sum(numeric["vision_age_ms"][i]>150 for i in active),
             "fault_samples":sum(v!=0 for v in faults),
             "pwm_saturation_samples":sum(max(numeric["left_pwm"][i],numeric["right_pwm"][i])>=45 for i in active),
             "ready_samples":len(ready),
             "ready_max_abs_ball_error_cm":max(ready_ball,default=0)/100,
             "ready_final_1s_max_abs_ball_error_cm":max((abs(numeric["ball_error_cm_x100"][i]) for i in ready_tail),default=0)/100,
             "ready_final_1s_within_1cm":bool(ready_tail) and all(abs(numeric["ball_error_cm_x100"][i])<=100 for i in ready_tail),
             "ready_pd_min_mdeg":min((numeric["beam_ball_pd_mdeg"][i] for i in ready),default=0),
             "ready_pd_max_mdeg":max((numeric["beam_ball_pd_mdeg"][i] for i in ready),default=0),
             "ready_vision_stale_samples":sum(numeric["vision_age_ms"][i]>150 for i in ready),
             "ready_fault_samples":sum(numeric["fault_flags"][i]!=0 for i in ready)}
    a.output_dir.mkdir(parents=True,exist_ok=True); stem=a.csv.stem
    (a.output_dir/f"{stem}_summary.json").write_text(json.dumps(summary,ensure_ascii=False,indent=2)+"\n")
    t=[]; offset=0; previous=numeric["mcu_ms"][0]
    for value in numeric["mcu_ms"]:
        if value < previous: offset += previous
        t.append((offset+value-numeric["mcu_ms"][0])/1000)
        previous=value
    plot_indices=active if active else ready if ready else list(range(len(rows)))
    plot_t=[t[i]-t[plot_indices[0]] for i in plot_indices]
    fig,axes=plt.subplots(4,1,figsize=(12,12),sharex=True)
    axes[0].plot(plot_t,[numeric["line_error"][i] for i in plot_indices],label="line error"); axes[0].plot(plot_t,[numeric["line_correction"][i] for i in plot_indices],label="correction"); axes[0].legend(); axes[0].grid()
    for side in ("left","right"):
        axes[1].plot(plot_t,[numeric[f"{side}_speed_ref_cm_s_x100"][i]/100 for i in plot_indices],label=f"{side} ref")
        axes[1].plot(plot_t,[numeric[f"{side}_speed_cm_s_x100"][i]/100 for i in plot_indices],label=f"{side} actual")
    axes[1].legend(ncol=2); axes[1].set_ylabel("cm/s"); axes[1].grid()
    axes[2].plot(plot_t,[numeric["vehicle_accel_cm_s2_x100"][i]/100 for i in plot_indices],label="accel cm/s2"); axes[2].plot(plot_t,[numeric["beam_target_mdeg"][i]/1000 for i in plot_indices],label="beam deg"); axes[2].legend(); axes[2].grid()
    axes[3].plot(plot_t,[numeric["ball_error_cm_x100"][i]/100 for i in plot_indices],label="ball cm"); axes[3].axhline(1,color="r"); axes[3].axhline(-1,color="r"); axes[3].legend(); axes[3].grid(); axes[3].set_xlabel("s")
    fig.tight_layout(); fig.savefig(a.output_dir/f"{stem}_analysis.png",dpi=160); plt.close(fig)
    print(json.dumps(summary,ensure_ascii=False)); return 0
if __name__ == "__main__": raise SystemExit(main())
