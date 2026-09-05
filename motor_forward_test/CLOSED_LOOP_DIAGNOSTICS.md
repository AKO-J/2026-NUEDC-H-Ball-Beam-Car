# 最简闭环诊断模式

K230 处于无手机/无网络预览的 headless 模式，只经原有 UART2 TX（IO5）向
TI UART3 RX（PA13）发送：

```text
B,frame,k230_ms,x,confidence,lost
```

TI 不改变 PID、UART3 接线、X42S 引脚或现有行程限位。它每 50 ms 通过
LaunchPad 原有 XDS110 backchannel UART0（PA10 TX，115200-8N1）输出一条
无表头 CSV：

```text
pc_ms,frame,k230_ms,x,confidence,age_ms,target_x,v,e,cmd,target_motor_pos,estimated_motor_pos,step_freq,state
```

`age_ms` 是 TI 端自最近一包有效 UART3 数据到当前日志时刻的年龄，因此不依赖
K230 与 TI 上电时钟是否同步。普通诊断固件启动后先输出 `WAIT_LEVEL`，不会
自动控制小球。调平时输出 `JOG_LEFT`、`JOG_RIGHT` 与 `JOG_STOP`；成功建立零点
后依次输出 `ZERO_ACCEPTED`、`ACTIVE`。K230 无效时立即输出 `LOST`，触及行程
保护时输出 `LIMIT_FAULT`，目标被现有安全检查拒绝时输出
`TARGET_REJECTED_*` 原因。旧的 `ACCEL`、`BRAKE`、`HOLD` 保留给 task-3 构建。

## 安全调平与启动

1. 上电后确认日志状态为 `WAIT_LEVEL`。此时闭环未启用。
2. 若机构已经在**断电后人工停靠的最低机械点**再上电，按一次 `PB0` 按键可执行
   旧版的有限预置抬升 `POS 0 -> -565`（实测 X42S 63.6°），日志为 `PRESET_LIFT`。它仅在软件位置为
   0、未保存零点且电机停止时接受；否则输出 `PRESET_REFUSED` 并立即停止。没有
   实体低点限位时，PB0 不是寻零，绝不能在未知位置或重新上电后未人工停低点时使用。
3. 按住 `UP`：正向、80 pulse/s 的 `JOG_RIGHT`；按住 `DOWN`：负向、80 pulse/s
   的 `JOG_LEFT`。松开按键（或同时按两键）立即停止为 `JOG_STOP`。该微调使用
   与闭环相同的 ±132 步保护范围，不能与自动闭环同时运行。
4. 目视确认梁实际水平后，松开按键并按一次 `START`。固件调用
   `StepperBeam_acceptManualReference()` 和
   `StepperBeam_configureTravelLimits(level-132, level+132)`；只有两者成功才进入
   `ZERO_ACCEPTED`，随后进入 `ACTIVE`。
5. 在 `ACTIVE` 中再次按 `START`、PB0 被误按、K230 丢球/超时或行程保护触发，都会立即停止
   STEP 脉冲并离开自动控制。`LOST` 或 `LIMIT_FAULT` 后须重新通过 `WAIT_LEVEL`
   调平并确认零点，才可再次进入 `ACTIVE`。

首次在电脑上运行工具前，在项目内创建虚拟环境并安装依赖（不会修改 Homebrew
管理的系统 Python）：

```sh
cd motor_forward_test
python3 -m venv .venv
. .venv/bin/activate
python -m pip install -r tools/requirements.txt
```

烧录完 TI 固件后，在另一终端开始记录；`--port auto` 会优先识别 XDS110，无法
唯一识别时改用 `--port /dev/cu.usbmodem...`。按 `Ctrl-C` 会刷新并关闭 CSV：

```sh
.venv/bin/python tools/record_serial.py --port auto
```

记录结束后分析最新的一次：

```sh
.venv/bin/python tools/analyze_latest.py
```

对每个已完成的 `ACTIVE` 测试段单独评分（推荐作为 PID 的后续人工评分依据）：

```sh
.venv/bin/python tools/score_test_runs.py
```

若要按固定放球位置进行可重复测试，先启动记录器，再在另一终端运行：

```sh
.venv/bin/python tools/run_release_sequence.py --positions-cm -1,-2,-3
```

它逐轮提示放球位置，但**不打开串口、不发送任何电机命令**；每轮仍必须由操作者按实体
`START` 开始，并用既有安全流程回到水平。记录器会把该轮的计划 ID、案例 ID 和放球位置
自动写入台账，供逐轮评分关联。

如果目录内保留了早期、列较少的 `test_runs.csv`，新版记录器会保留它不动，并把新记录
写入 `test_runs_v2.csv`；评分工具会自动选择兼容的新台账，避免旧记录发生错列。

曲线 PNG 与丢帧/年龄统计文本位于 `analysis/`，原始记录位于
`logs/run_YYYYMMDD_HHMMSS.csv`。

记录器在每次进入 `ACTIVE` 时生成 `test_run_id`，并在退出 `ACTIVE` 时把一条完成
记录追加到 `logs/test_runs.csv`。每条记录包含原始 CSV 名称、配置版本、CSV 精确行范围、
起点/目标、开始结束时间和结束状态；`score_test_runs.py` 把同一个 `test_run_id`、CSV
片段和配置版本写进 `analysis/test_*_score.json`。这三者构成可追溯链，避免跨多轮复位或
暂停的混合日志被当成一次测试评分。

## 可重复性与调参锁

每次由 `record_serial.py` 新建的记录会同时生成
`logs/run_YYYYMMDD_HHMMSS.meta.json`。其中保存串口设置、开始/结束时间、控制器、
校准、步进、普通闭环和 K230 源文件的 SHA-256，以及由这些哈希计算的
`configuration_version`。它只记录证据，绝不会更改固件参数。

`analyze_latest.py` 还会生成同名的 `analysis/*_score.json`。它记录：

- 过冲、稳定时间和稳态误差（目标阶跃，或从指定放球位置释放到固定目标时计算）；
- 有效 `ACTIVE` 时段的位置 IAE、控制输出峰值和误差反向次数；
- 视觉平均/最大帧周期、丢帧估计、数据年龄和丢球比例；
- 配置版本、是否观察到实际步进执行和所有安全故障状态。

既没有目标阶跃、也没有明显偏离固定目标的放球起点、没有至少 3 秒 `ACTIVE` 电机执行，
或出现 `LOST`/限位/目标拒绝时，
对应指标或评分会明确写为 `N/A` / `diagnostic only`，不能据此修改 PID。
所有评分文件目前固定为 `tuning_eligible=false`；自动 PID 修改尚未实现，也不会被
日志或脚本自动解锁。

在进入后续半自动调参前，必须人工、分次验证并保留对应日志：

1. `WAIT_LEVEL` 下两个方向微调、松手立即停止，以及两个软行程端点均可停止；
2. 重新调平后 `START` 能建立零点；再次按 `START` 能从 `ACTIVE` 停止；
3. 遮挡/移走小球能进入 `LOST` 并停止脉冲；
4. 以球在中心附近的受控测试观察到 `ACTIVE`、实际 `step_freq` 和可信视觉数据；
5. 现有急停和机械限位按硬件规程验证。

这些人工安全证据未完成前，工具只用于诊断，不允许搜索、写入或自动调整 PID。

可用下列命令查看尚缺的安全验证项；它不会解锁 PID：

```sh
.venv/bin/python tools/check_safety_readiness.py
```

### 交互式视觉与丢球证据采集

在**另一个记录器没有占用 XDS110 诊断串口**时，可运行下列只读交互工具。它会依次
显示“视觉连续性、遮挡丢球、移走小球、通信超时”的人工操作提示。每项按一次 Enter 开始
记录、完成物理动作后再按一次 Enter 结束；它只读取 TI 的 115200 诊断 UART，绝不向 K230、
MSPM0 或电机发送字节。

```sh
.venv/bin/python tools/verify_vision_safety.py --port auto
```

每项都会写入独立 CSV 和 JSON 到 `logs/vision_safety_*/`，其中保存可观察的 `LOST`、
`step_freq`、数据年龄、帧号/时间戳倒退和主机侧拒绝行。由于 TI 每 50 ms 只记录最新视觉帧，
相邻日志帧号跳过不等于视觉或 UART 丢帧；必须结合 TI RX 接收/拒绝/丢字节计数审查。JSON
是供人工审查的证据摘要，不是自动通过判定，也不能解锁 PID。需要单独运行某项时，例如：

```sh
.venv/bin/python tools/verify_vision_safety.py --cases ball_occlusion,ball_removed
```

每完成一项硬件验证，必须用对应的真实 CSV、评分 JSON 或人工验证说明作为证据文件记录，
例如：

```sh
.venv/bin/python tools/check_safety_readiness.py \
  --confirm log_credibility --evidence-file analysis/test_xxx_score.json \
  --note "人工复核 CSV 行范围、版本哈希与评分记录一致"
```

证据保存在 `logs/safety_validation.json` 并带 SHA-256；该台账仅供人工审查，绝不自动
解锁 PID 修改权限。

## 放球定位显示与视觉标定

当前 K230 源码的最新记录是：视觉中心 `x=148 px`，`+1 cm` 为 `x=160 px`，
因此当前暂用 `1 cm = 12.0 px`。2026-08-01 已把 ROI 从 `0,50,288,68` 收紧为
`0,52,288,48`，以排除杆下方电路；虽然 X 坐标原点不变，YOLO 输入的纵横比已经改变，
所以在此 ROI 上重新核对 `0 cm`、`+1 cm` 以及 `±5 cm` 后，才可将该换算视为有效。

```text
1 cm = 12.0 px
offset_mm = (ball_x_px - 148) × 10 / 12.0
```

放球中心容差设为 `±2.0 mm`，等效 `±2.4 px`。由于球心像素为整数，程序以
`|offset_px| <= 2.4` 判定 `READY`（也就是整数偏移不超过 `2 px`）。

在 K230 的 `steel_ball_uart_yolo.py` 中把 `ENABLE_LCD_DEBUG = True` 后，LCD 会显示：

- 红线：`x=148 px` 的视觉中心；两条绿线：`±2 mm` 放球带；
- `BALL X`：球心原始像素读数；`dX`：相对中心的像素与毫米偏移；
- `READY` 或 `MOVE BALL`：是否落在放球容差内。

相机、支架、ROI、分辨率或镜像任一项变更后，旧换算即失效。把球依次放到相距已知
`D_mm` 的两点，记下 LCD 的 `BALL X` 为 `x1`、`x2`，然后更新：

```text
PIXELS_PER_CM = abs(x2 - x1) * 10 / D_mm
CONTROL_CENTER_X = 中心物理位置对应的 BALL X
```
