#!/usr/bin/env python3
"""Score at least five completed joint-control runs without tuning anything."""
from __future__ import annotations
import argparse, csv, json
from pathlib import Path

def score(path: Path) -> list[dict[str,object]]:
    with path.open(newline="") as f: rows=list(csv.DictReader(f))
    results=[]
    for run_id in sorted({int(r["run_id"]) for r in rows if int(r["run_id"])>0}):
        run=[r for r in rows if int(r["run_id"])==run_id and r["state"] in {"RAMP_UP","CRUISE","RAMP_DOWN","STOPPED","FAULT"}]
        maximum=max((abs(int(r["ball_error_cm_x100"])) for r in run),default=100000)
        results.append({"csv":path.name,"run_id":run_id,"samples":len(run),
                        "max_abs_ball_error_cm":maximum/100,
                        "stopped":any(r["state"]=="STOPPED" for r in run),
                        "fault_free":all(int(r["fault_flags"])==0 for r in run),
                        "passed":bool(run) and maximum<=100 and any(r["state"]=="STOPPED" for r in run) and all(int(r["fault_flags"])==0 for r in run)})
    return results

def main() -> int:
    p=argparse.ArgumentParser(); p.add_argument("paths",nargs="+",type=Path); p.add_argument("--output",type=Path,default=Path("analysis/joint/score.json")); a=p.parse_args()
    runs=[r for path in a.paths for r in score(path)]; consecutive=0
    for run in runs: consecutive=consecutive+1 if run["passed"] else 0
    result={"schema":"joint_line_balance_score/v1","runs":runs,
            "passed_runs":sum(bool(r["passed"]) for r in runs),
            "consecutive_passes_at_end":consecutive,"accepted":consecutive>=5}
    a.output.parent.mkdir(parents=True,exist_ok=True); a.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n")
    print(json.dumps(result,ensure_ascii=False)); return 0 if result["accepted"] else 2
if __name__ == "__main__": raise SystemExit(main())
