# SMARTCAR 系统与代码架构

更新时间：2026-08-06

## 控制链路

```text
K230 摄像头
  -> 检测钢球 X 坐标
  -> UART2 TX(IO5), 921600 baud
  -> MSPM0 UART3 RX(PA13)
       -> BallBeamController / α-β估计
       -> X42S STEP/DIR/EN 调整摆杆
       -> LF04 四路输入进行循迹
       -> TB6612 控制左右车轮
       -> 编码器与 ICM42688 诊断
       -> OLED 和按键完成现场交互
       -> XDS110 UART 输出 CSV 日志
```

正式控制闭环在车上完成；场外设备只用于接收、记录和分析，不应向车发送比赛控制指令。

## 核心硬件连接

| 功能 | 引脚/连接 |
| --- | --- |
| K230 坐标 TX | K230 IO5 → MSPM0 PA13，双方共地 |
| X42S | STEP=PB13，DIR=PB12，EN=PB6 |
| LF04 O1/O2/O3/O4 | PB18 / PA24 / PA17 / PA12 |
| 左/右电机 PWM | PB4 / PB1 |
| 左编码器 A/B | PB2 / PB24 |
| 右编码器 A/B | PB9 / PA27 |
| OLED SCL/SDA | PB19 / PA15 |
| S1/S2/S3/S4/S5 | PB7 / PB8 / PB15 / PB0 / PB20，低电平有效 |

12 V 只能进入允许的电机电源端，不能进入 MCU 或信号脚。完整物理排针表在清理完成前仍以 `motor_forward_test/H_TASK_1_40引脚总表.md` 为核对来源。

## 核心源码

| 作用 | 文件 |
| --- | --- |
| 统一选择入口 | `competition_selector_main.c` |
| 三任务接口 | `competition_entries.h` |
| 第二题 | `h_track_task2.c`、`h_track_controller.c`、`h_track_finish.c` |
| 第三题/闭环首测 | `tests/bringup/ball_beam_vision_control_test.c` |
| 第4/5/6问联合入口 | `tests/bringup/joint_line_balance_test.c` |
| 联合纯控制逻辑 | `joint_line_balance_control.c` |
| 小球控制器 | `ball_beam_controller.c` |
| 摆杆驱动/标定 | `stepper_beam.c`、`beam_calibration.c`、`beam_level_reference.c` |
| K230 视觉 | `k230/steel_ball_uart_yolo.py`、`k230/ball_roi_config.py` |
| TI 视觉接收 | `vision_uart.c`、`vision_ball_protocol.c` |
| 日志分析/评分 | `tools/analyze_latest.py`、`tools/score_test_runs.py`、`tools/check_safety_readiness.py` |
| 构建与主机测试 | `Makefile`、`tests/host/` |

## 统一固件行为

- S2 选择第二题，S3 选择第三题，S4 选择第4/5/6问，S1 确认，S5 保留。
- 选择确认后必须等待 S1 稳定释放，避免进入任务时误触发任务内部启动。
- 每个任务保留自己的状态机；统一层只选择和调用入口。
- 停止后要切换任务时，当前设计要求人工复位回到选择页。
- 运行中 S1 急停优先于 OLED、视觉处理和普通按键逻辑。

## 当前关键参数

### 第三题

- 目标：`0 → +5 → -5 cm`，总时长不超过 5 s，两端误差不超过 1 cm。
- 当前通过参数：正向巡航 `+800 mdeg`、正端提前折返 `+12 px`、负向制动 `+565 mdeg`。
- 可追溯通过样本：`run_20260801_123833`，总时长 1.75 s，正端峰值 +66 px，负端停止后 -77..-56 px，无丢球。

### 第4/5/6问联合控制

- 小球控制：Kp=11 mdeg/px，Kd=1 mdeg/(px/s)，I=0，方向 `+1`，PD 限幅 ±400 mdeg。
- 摆杆总角度限制：实测标定范围 `-2221..+1777 mdeg`，不得外推。
- 车轮参考：`CRUISE_SPEED_X100=500`、`ACCEL_LIMIT_X100=100`、`JERK_LIMIT_X100=60`。
- 启动固定预倾 60 mdeg；计划前馈在启动后 0.6 s 开始，`KFF_MILLI=1000`、`KFF_SIGN=+1`。
- PWM：750 ms 内升到左4%/右5%，保持到 1.0 s，再跨到左5%/右6%。
- 无里程自动停车；运行中 S1 急停。

源码注释仍把巡航值和 `KFF_SIGN` 标为需要验证，因此这些值是“当前冻结候选/交接值”，不能仅因写在文档中就称为统一固件实机验证通过。

## K230 标定

| 物理位置 | 完整画面 X | 发给 TI 的偏移 |
| --- | ---: | ---: |
| -5 cm | 84 px | -65 px |
| 0 cm | 149 px | 0 px |
| +5 cm | 208 px | +59 px |

负侧约 13.0 px/cm，正侧约 11.8 px/cm，不应强行合并为统一比例。修改视觉脚本、ROI、镜头位置或安装姿态后必须重做三点核对。
