# H 题当前工程入口

本目录只保留当前 H 题会使用的正式代码、可复用驱动和少量安全首测。旧八路灰度、旧正方形循迹和一次性电机实验均已保存在 [`archive/`](archive/README.md)，不再有 Make 构建目标，避免误烧录。例外是仍由当前 `stepper_beam.c` 驱动的 X42S 摆杆首测；它只构建为独立的安全点动镜像。

## 当前可用固件

| 用途 | 命令 | 是否驱动电机 |
| --- | --- | --- |
| LF04 四路读数与 OLED 首测 | `make ir-line-sensor-read-test` | 否 |
| ICM42688 I2C 通信检查 | `make icm42688-test` | 否 |
| 编码器直线距离标定 | `make encoder-distance-calibration-test` | 按下 `PB7` 后低速运行至左右各 6000 脉冲 |
| X42S 摆杆手动水平零点/点动首测 | `make stepper-jog-test` | 仅在实体按键解锁后输出脉冲 |
| H 题任务 2 正式 LF04 循迹 | `make line-follow` | 按下 `PB7` 启动键后才会驱动 |

正式镜像是 `build/line_follow_test.out`。上电先显示编码器整圈停车参数；启动键 `PB7` 连到 GND 并稳定按下 30 ms 后才进入循迹。LF04 用 `3.3 V`，信号固定为 `O1/O2/O3/O4 = PB18(25)/PA24(27)/PA17(28)/PA12(32)`，黑线为低电平。

当前 A 点横线不能稳定检出，因此 LF04 只负责循迹，停车直接使用编码器里程。两次直线实测得到左轮 `64.45 脉冲/cm`、右轮 `62.67 脉冲/cm`；程序分别换算两轮距离后取平均。按图示 `1.5 m` 直线和 `0.5 m` 半径计算，A→A 中心线目标是 `614.16 cm`（平均约 `39035` 脉冲），最后 `30 cm` 降速并短刹。四路编码器现在使用 GPIO 双边沿中断计数，避免巡线速度下因 OLED/IMU 通信阻塞主循环而漏脉冲。IMU 只保留为诊断，不参与正式停车判定。停车距离在 [`h_track_pwm_config.h`](h_track_pwm_config.h) 中标定，仍需用多次实车 A 点偏差收敛到 `2 cm` 内。

## 构建、校验、烧录

```sh
make check-host
make encoder-distance-calibration-test
make line-follow
./flash_xds110.sh --yes build/line_follow_test.out
```

直线标定时，构建并烧录独立镜像：

```sh
make encoder-distance-calibration-test
./flash_xds110.sh --yes build/encoder_distance_calibration_test.out
```

它上电保持停车，按下并释放 `PB7` 后才低速前进；左右轮分别到达 6000 脉冲后短刹并自动停止，运行中再次按 `PB7` 可中止。量取起点到同一车体参考点的实际距离，并记录 OLED 的 `ENC左/右` 或 CCS 中的 `g_encoder_cal_left_count`、`g_encoder_cal_right_count`。

摆杆仅调试时，构建并烧录独立镜像：

```sh
make stepper-jog-test
./flash_xds110.sh --yes build/stepper_jog_test.out
```

它使用 `PB13/PB12/PB6` 驱动 X42S 的 `Stp/Dir/En`，不使用机械限位开关；上电保持锁定，长按 START 会停止脉冲。接线和现场顺序见
[X42S_空载点动测试.md](X42S_空载点动测试.md)。

白杆首次实测得到的“摆杆角度 → 电机脉冲”查表、零点方法和适用范围见
[X42S_白杆首次标定结果_2026-07-31.md](X42S_白杆首次标定结果_2026-07-31.md)。后续写摆杆或小球控制前先读取该表；不可将 X42S 屏幕电机轴角度直接当作白杆角度。

最后一条只擦写并校验，不自动运行。烧录电机相关镜像前，必须断开 TB6612 的 `VM` 或将车轮悬空。

CCS 用户可导入以下 `.projectspec` 后直接使用 **Build → Debug**：

- `ticlang/lf04_ir_read_test_LP_MSPM0G3507_nortos_ticlang.projectspec`：安全 LF04/OLED 首测。
- `ticlang/h_task2_line_follow_LP_MSPM0G3507_nortos_ticlang.projectspec`：正式任务 2 循迹。

## 目录

| 位置 | 内容 |
| --- | --- |
| 根目录 | 正式任务 2 应用、编码器距离终点检测器、LF04/电机/编码器/OLED/ICM 驱动。 |
| `tests/bringup/` | 只读 LF04/ICM 首测和 PB7 启动的编码器直线距离标定程序。 |
| `tests/host/` | 可在电脑上运行的控制器/映射单元测试。 |
| `archive/` | 保留的历史程序和文档，不参与当前构建。 |

完整接线见 [H_TASK_1_40引脚总表.md](H_TASK_1_40引脚总表.md)，当前任务约束见 [H_TASK_AI_HANDOFF.md](H_TASK_AI_HANDOFF.md)。

2026-08-01 的编码器距离与 A 点停车校准安排见
[明日任务_2026-08-01.md](明日任务_2026-08-01.md)。
