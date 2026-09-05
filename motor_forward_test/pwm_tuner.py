#!/usr/bin/python3
"""Visual tuner for the active 2024 H-task-2 firmware.

This utility deliberately edits only named firmware settings and then lets the
user build them for flashing in CCS. It does not claim to be a live tuner:
live adjustment needs a UART protocol in the firmware and a confirmed serial
connection.

Run: python3 pwm_tuner.py
"""

from __future__ import annotations

import os
import queue
import re
import subprocess
import threading
from dataclasses import dataclass
from functools import partial
from pathlib import Path
from tkinter import messagebox

# macOS bundled Tk displays native buttons reliably, but on this system it may
# fail to draw Label, Scale, and Canvas widgets.  Keep the whole UI button-only.
os.environ.setdefault("TK_SILENCE_DEPRECATION", "1")

import tkinter as tk


PROJECT_DIR = Path(__file__).resolve().parent
SOURCE_FILE = PROJECT_DIR / "h_track_pwm_config.h"
@dataclass(frozen=True)
class Parameter:
    key: str
    label: str
    define: str
    default: int
    minimum: int
    maximum: int
    suffix: str = "U"


# These are every active fixed-table PWM value. The heading target and old
# preview PD / speed PI are intentionally absent from H task 2.
PARAMETERS = (
    Parameter("center_left", "直线左轮 PWM", "H_TRACK_CENTER_LEFT_DUTY",
              17, 0, 100),
    Parameter("center_right", "直线右轮 PWM", "H_TRACK_CENTER_RIGHT_DUTY",
              19, 0, 100),
    Parameter("single_inner", "X4/X5 单灯内轮 PWM", "H_TRACK_SINGLE_INNER_DUTY",
              10, 0, 100),
    Parameter("single_outer", "X4/X5 单灯外轮 PWM", "H_TRACK_SINGLE_OUTER_DUTY",
              33, 0, 100),
    Parameter("inner_pair_inner", "X3/X4、X5/X6 内轮 PWM",
              "H_TRACK_INNER_PAIR_INNER_DUTY", 10, 0, 100),
    Parameter("inner_pair_outer", "X3/X4、X5/X6 外轮 PWM",
              "H_TRACK_INNER_PAIR_OUTER_DUTY", 43, 0, 100),
    Parameter("middle_pair_inner", "X2/X3、X6/X7 内轮 PWM",
              "H_TRACK_MIDDLE_PAIR_INNER_DUTY", 5, 0, 100),
    Parameter("middle_pair_outer", "X2/X3、X6/X7 外轮 PWM",
              "H_TRACK_MIDDLE_PAIR_OUTER_DUTY", 51, 0, 100),
    Parameter("outer_pair_inner", "X1/X2、X7/X8 内轮 PWM",
              "H_TRACK_OUTER_PAIR_INNER_DUTY", 7, 0, 100),
    Parameter("outer_pair_outer", "X1/X2、X7/X8 外轮 PWM",
              "H_TRACK_OUTER_PAIR_OUTER_DUTY", 61, 0, 100),
    Parameter("extreme_inner", "X1/X8 单灯内轮 PWM",
              "H_TRACK_EXTREME_INNER_DUTY", 0, 0, 100),
    Parameter("extreme_outer", "X1/X8 单灯外轮 PWM",
              "H_TRACK_EXTREME_OUTER_DUTY", 80, 0, 100),
)
PARAMETER_BY_KEY = {parameter.key: parameter for parameter in PARAMETERS}


def read_source_values() -> dict[str, int]:
    """Read exactly the named settings shown by this version of the UI."""
    source = SOURCE_FILE.read_text(encoding="utf-8")
    values: dict[str, int] = {}
    for parameter in PARAMETERS:
        match = re.search(
            rf"^\s*#define\s+{re.escape(parameter.define)}\s+(\d+)"
            rf"{re.escape(parameter.suffix)}\b",
            source,
            flags=re.MULTILINE,
        )
        values[parameter.key] = (
            int(match.group(1)) if match else parameter.default
        )
    return values


def write_source_values(values: dict[str, int]) -> None:
    """Replace each displayed define once, never touch other source text."""
    source = SOURCE_FILE.read_text(encoding="utf-8")
    for parameter in PARAMETERS:
        value = values[parameter.key]
        pattern = (
            rf"(^\s*#define\s+{re.escape(parameter.define)}\s+)\d+"
            rf"{re.escape(parameter.suffix)}\b"
        )
        source, count = re.subn(
            pattern,
            rf"\g<1>{value}{parameter.suffix}",
            source,
            count=1,
            flags=re.MULTILINE,
        )
        if count != 1:
            raise RuntimeError(f"未找到 {parameter.define}")
    SOURCE_FILE.write_text(source, encoding="utf-8")


class PwmTuner(tk.Frame):
    def __init__(self, master: tk.Tk) -> None:
        super().__init__(master, padx=16, pady=16)
        self.master = master
        self.events: queue.Queue[tuple[str, str]] = queue.Queue()
        self.busy = False
        current = read_source_values()
        self.values = {
            parameter.key: tk.IntVar(value=current[parameter.key])
            for parameter in PARAMETERS
        }
        self.status = tk.StringVar(
            value="已读取任务 2 活动固定表；这不是车端实时遥测"
        )
        self.value_buttons: dict[str, tk.Button] = {}
        self._build_ui()
        for value in self.values.values():
            value.trace_add("write", self._refresh_preview)
        self._refresh_preview()
        self.after(80, self._drain_events)

    def _build_ui(self) -> None:
        self.grid(sticky="nsew")
        self.master.columnconfigure(0, weight=1)
        self.master.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)

        for row, parameter in enumerate(PARAMETERS):
            line = tk.Frame(self)
            line.grid(row=row, column=0, sticky="w", pady=4)
            tk.Button(
                line, text=parameter.label, width=24,
                command=partial(self._select_parameter, parameter.key),
            ).grid(row=0, column=0, padx=(0, 8))
            tk.Button(
                line, text="−5", width=5,
                command=partial(self._change_value, parameter.key, -5),
            ).grid(row=0, column=1)
            tk.Button(
                line, text="−1", width=5,
                command=partial(self._change_value, parameter.key, -1),
            ).grid(row=0, column=2, padx=(4, 4))
            value_button = tk.Button(
                line, width=8,
                command=partial(self._select_parameter, parameter.key),
            )
            value_button.grid(row=0, column=3)
            self.value_buttons[parameter.key] = value_button
            tk.Button(
                line, text="+1", width=5,
                command=partial(self._change_value, parameter.key, 1),
            ).grid(row=0, column=4, padx=(4, 4))
            tk.Button(
                line, text="+5", width=5,
                command=partial(self._change_value, parameter.key, 5),
            ).grid(row=0, column=5)

        info_row = len(PARAMETERS)
        self.straight_info_button = tk.Button(self, anchor="w", width=76)
        self.straight_info_button.grid(
            row=info_row, column=0, sticky="w", pady=(10, 0)
        )
        self.turn_info_button = tk.Button(self, anchor="w", width=76)
        self.turn_info_button.grid(row=info_row + 1, column=0, sticky="w", pady=(4, 0))
        self.fine_info_button = tk.Button(self, anchor="w", width=76)
        self.fine_info_button.grid(row=info_row + 2, column=0, sticky="w", pady=(4, 0))
        self.recovery_info_button = tk.Button(self, anchor="w", width=76)
        self.recovery_info_button.grid(row=info_row + 3, column=0, sticky="w", pady=(4, 0))
        self.pi_info_button = tk.Button(self, anchor="w", width=76)
        self.pi_info_button.grid(row=info_row + 4, column=0, sticky="w", pady=(4, 0))
        self.mapping_info_button = tk.Button(self, anchor="w", width=76)
        self.mapping_info_button.grid(row=info_row + 5, column=0, sticky="w", pady=(4, 0))

        buttons = tk.Frame(self)
        buttons.grid(row=info_row + 6, column=0, sticky="ew", pady=(16, 0))
        self.read_button = tk.Button(buttons, text="重新读取", command=self._load_values)
        self.save_button = tk.Button(buttons, text="保存到主程序", command=self._save_values)
        self.build_button = tk.Button(buttons, text="构建", command=self._build)
        self.flash_button = tk.Button(
            buttons, text="保存、构建并准备烧录", command=self._save_build_prepare_flash
        )
        self.read_button.grid(row=0, column=0, padx=(0, 8))
        self.save_button.grid(row=0, column=1, padx=(0, 8))
        self.build_button.grid(row=0, column=2, padx=(0, 8))
        self.flash_button.grid(row=0, column=3)

        self.status_button = tk.Button(self, anchor="w", width=76)
        self.status_button.grid(row=info_row + 7, column=0, sticky="w", pady=(10, 0))

    def _current_values(self) -> dict[str, int]:
        return {
            parameter.key: max(
                parameter.minimum,
                min(parameter.maximum, self.values[parameter.key].get()),
            )
            for parameter in PARAMETERS
        }

    def _refresh_preview(self, *_args: object) -> None:
        values = self._current_values()
        for parameter in PARAMETERS:
            display = (
                f"{values[parameter.key]}%"
                if "PWM" in parameter.label else str(values[parameter.key])
            )
            self.value_buttons[parameter.key].configure(text=display)

        self.straight_info_button.configure(
            text=("X4+X5 居中／白底航向死区：左轮 "
                  f"{values['center_left']}% ／ 右轮 "
                  f"{values['center_right']}%")
        )
        self.turn_info_button.configure(
            text=("X4 或 X5：内 "
                  f"{values['single_inner']}% ／ 外 {values['single_outer']}%"
                  " ｜ X3+X4 或 X5+X6：内 "
                  f"{values['inner_pair_inner']}% ／ 外 "
                  f"{values['inner_pair_outer']}%")
        )
        self.fine_info_button.configure(
            text=("X2+X3 或 X6+X7：内 "
                  f"{values['middle_pair_inner']}% ／ 外 "
                  f"{values['middle_pair_outer']}%"
                  " ｜ X1+X2 或 X7+X8：内 "
                  f"{values['outer_pair_inner']}% ／ 外 "
                  f"{values['outer_pair_outer']}%")
        )
        self.recovery_info_button.configure(
            text=("X1 或 X8：内 "
                  f"{values['extreme_inner']}% ／ 外 "
                  f"{values['extreme_outer']}%（最大固定差速）")
        )
        self.pi_info_button.configure(
            text="弯道：固定掩码查表；不使用预瞄 PD、均速 PI、斜坡或弯中陀螺仪。"
        )
        self.mapping_info_button.configure(
            text="白底连续 500 ms 后才启用航向保持；保存、构建后，在 CCS 中烧录才会生效。"
        )
        self.status_button.configure(text=self.status.get())

    def _change_value(self, key: str, delta: int) -> None:
        parameter = PARAMETER_BY_KEY[key]
        value = max(
            parameter.minimum,
            min(parameter.maximum, self.values[key].get() + delta),
        )
        self.values[key].set(value)
        self.status.set(
            f"已修改：{parameter.label} = {value}（尚未保存、构建或烧录）"
        )
        self._refresh_preview()

    def _select_parameter(self, key: str) -> None:
        parameter = PARAMETER_BY_KEY[key]
        suffix = "%" if "PWM" in parameter.label else ""
        self.status.set(
            f"当前选择：{parameter.label} = {self.values[key].get()}{suffix}；"
            "请点击本行 −5、−1、+1 或 +5 调节"
        )
        self._refresh_preview()

    def _load_values(self) -> None:
        try:
            current = read_source_values()
        except OSError as error:
            messagebox.showerror("读取失败", str(error))
            return
        for key, value in current.items():
            self.values[key].set(value)
        self.status.set("已从 h_track_pwm_config.h 读取活动任务 2 配置（非车端实时遥测）")
        self._refresh_preview()

    def _save_values(self) -> bool:
        values = self._current_values()
        duty_pairs = (
            ("single_inner", "single_outer"),
            ("inner_pair_inner", "inner_pair_outer"),
            ("middle_pair_inner", "middle_pair_outer"),
            ("outer_pair_inner", "outer_pair_outer"),
            ("extreme_inner", "extreme_outer"),
        )
        if any(values[inner] > values[outer] for inner, outer in duty_pairs):
            messagebox.showerror("参数不合理", "各模式的内侧 PWM 不能大于外侧 PWM。")
            return False
        try:
            write_source_values(values)
        except (OSError, RuntimeError) as error:
            messagebox.showerror("保存失败", str(error))
            return False
        self.status.set("参数已保存到 h_track_pwm_config.h；仍需构建并烧录才会生效")
        self._refresh_preview()
        return True

    def _build(self) -> None:
        self._run_commands("构建", [["make", "-B", "line-follow"]])

    def _save_build_prepare_flash(self) -> None:
        if not self._save_values():
            return
        self._run_commands(
            "保存、构建并准备 CCS 烧录",
            [["make", "-B", "line-follow"]],
        )

    def _run_commands(self, title: str, commands: list[list[str]]) -> None:
        if self.busy:
            return
        self.busy = True
        self.status.set(f"正在{title}…")
        self._refresh_preview()
        for button in (self.read_button, self.save_button, self.build_button, self.flash_button):
            button.configure(state="disabled")

        def worker() -> None:
            try:
                for command in commands:
                    process = subprocess.run(
                        command,
                        cwd=PROJECT_DIR,
                        env=os.environ.copy(),
                        text=True,
                        stdout=subprocess.PIPE,
                        stderr=subprocess.STDOUT,
                    )
                    if process.returncode != 0:
                        raise RuntimeError(f"命令失败，退出码 {process.returncode}")
            except Exception as error:
                self.events.put(("error", str(error)))
            else:
                if title == "保存、构建并准备 CCS 烧录":
                    self.events.put((
                        "done",
                        "构建完成：在 CCS 选中工程后执行 Run → Flash Project，"
                        "使用板载 XDS110 写入开发板。",
                    ))
                else:
                    self.events.put(("done", f"{title}完成"))

        threading.Thread(target=worker, daemon=True).start()

    def _drain_events(self) -> None:
        try:
            while True:
                kind, message = self.events.get_nowait()
                if kind == "error":
                    self.status.set(message)
                    self._refresh_preview()
                    messagebox.showerror("执行失败", message)
                    self._finish_task()
                elif kind == "done":
                    self.status.set(message)
                    self._refresh_preview()
                    self._finish_task()
        except queue.Empty:
            pass
        self.after(80, self._drain_events)

    def _finish_task(self) -> None:
        self.busy = False
        for button in (self.read_button, self.save_button, self.build_button, self.flash_button):
            button.configure(state="normal")


def main() -> None:
    root = tk.Tk()
    root.title("2024 H 题任务 2 固定掩码 PWM 调参")
    root.minsize(1040, 780)
    PwmTuner(root)
    root.mainloop()


if __name__ == "__main__":
    main()
