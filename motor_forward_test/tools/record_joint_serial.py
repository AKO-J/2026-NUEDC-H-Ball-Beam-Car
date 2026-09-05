#!/usr/bin/env python3
"""Record joint straight-line telemetry plus immutable build metadata."""
from __future__ import annotations

import argparse, csv, hashlib, json, os, signal
from datetime import datetime
from pathlib import Path
import serial
from serial.tools import list_ports

COLUMNS = """mcu_ms,run_id,state,safety_state,ir_raw_mask,line_error,line_error_rate,line_correction,base_speed_ref_cm_s_x100,base_accel_ref_cm_s2_x100,left_speed_ref_cm_s_x100,right_speed_ref_cm_s_x100,left_speed_cm_s_x100,right_speed_cm_s_x100,left_speed_error_x100,right_speed_error_x100,left_pwm,right_pwm,left_encoder,right_encoder,vehicle_speed_cm_s_x100,vehicle_accel_cm_s2_x100,imu_ax_raw,imu_ay_raw,imu_az_raw,kff_milli,beam_ff_mdeg,beam_ball_pd_mdeg,beam_target_mdeg,stepper_target_pos,stepper_actual_pos,step_freq,vision_frame,vision_age_ms,ball_target_cm_x100,ball_error_cm_x100,ball_velocity_cm_s_x100,fault_flags""".split(",")
STATES = {"WAIT_LEVEL","READY","RAMP_UP","CRUISE","RAMP_DOWN","STOPPED","FAULT"}
ROOT = Path(__file__).resolve().parents[1]
SOURCES = ["tests/bringup/joint_line_balance_test.c","joint_line_balance_control.c",
           "joint_line_balance_control.h","ball_beam_controller.c",
           "ball_beam_controller.h","diagnostic_uart.c","diagnostic_uart.h",
           "motor_driver.c","encoder.c","icm42688.c","ir_line_sensor.c",
           "stepper_beam.c","beam_calibration.c","vision_uart.c"]
stop_requested = False

def stop(*_: object) -> None:
    global stop_requested
    stop_requested = True

def valid(text: str) -> list[str] | None:
    try:
        row = next(csv.reader([text]))
        if len(row) != len(COLUMNS) or row[2] not in STATES: return None
        for i, value in enumerate(row):
            if i not in (2, 3): int(value)
        return row
    except (ValueError, csv.Error, StopIteration): return None

def choose_port(requested: str, baud: int) -> str:
    if requested != "auto": return requested
    ports = [p.device for p in list_ports.comports()
             if "xds110" in ((p.description or "") + (p.manufacturer or "")).lower()]
    for port in ports:
        try:
            with serial.Serial(port, baud, timeout=.25) as dev:
                if any(valid(dev.readline().decode("ascii","replace").strip()) for _ in range(4)):
                    return port
        except serial.SerialException: pass
    raise SystemExit("无法识别联合控制日志串口，请用 --port 指定")

def main() -> int:
    parser=argparse.ArgumentParser(); parser.add_argument("--port",default="auto")
    parser.add_argument("--baud",type=int,default=115200)
    parser.add_argument("--logs",type=Path,default=Path("logs/joint")); args=parser.parse_args()
    args.logs.mkdir(parents=True,exist_ok=True); port=choose_port(args.port,args.baud)
    stem=datetime.now().strftime("joint_%Y%m%d_%H%M%S")
    output=args.logs/f"{stem}.csv"; meta=args.logs/f"{stem}.meta.json"
    hashes={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in SOURCES}
    metadata={"schema":"joint_line_balance/v2","csv":output.name,"serial_port":port,
              "baud":args.baud,"started_at":datetime.now().astimezone().isoformat(timespec="seconds"),
              "source_sha256":hashes,"tuning_status":"unverified_no_automatic_changes",
              "vision_role":"task3_pd_plus_acceleration_feedforward",
              "ball_tolerance_cm":1.0}
    meta.write_text(json.dumps(metadata,ensure_ascii=False,indent=2,sort_keys=True)+"\n")
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop); accepted=rejected=0
    serial_error = None
    try:
        with serial.Serial(port,args.baud,timeout=.2) as dev, output.open("w",newline="") as handle:
            writer=csv.writer(handle,lineterminator="\n"); writer.writerow(COLUMNS); handle.flush()
            while not stop_requested:
                row=valid(dev.readline().decode("ascii","replace").strip())
                if row is None: rejected+=1; continue
                writer.writerow(row); handle.flush(); accepted+=1
    except serial.SerialException as error:
        serial_error = str(error)
    finally:
        metadata.update(stopped_at=datetime.now().astimezone().isoformat(timespec="seconds"),
                        accepted_rows=accepted,rejected_rows=rejected)
        if serial_error is not None: metadata["serial_error"] = serial_error
        meta.write_text(json.dumps(metadata,ensure_ascii=False,indent=2,sort_keys=True)+"\n")
        if output.exists():
            with output.open("a") as handle: handle.flush(); os.fsync(handle.fileno())
    print(f"已保存 {output}，有效 {accepted} 行，忽略 {rejected} 行")
    return 0
if __name__ == "__main__": raise SystemExit(main())
