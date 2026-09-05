# SMARTCAR 项目说明

## 最终目标

利用视觉检测摆杆内钢球的位置，通过步进电机连续微调摆杆，最终实现安全、可复现实验数据分析与控制参数调整。

## 工作要求

1. 修改代码前，先阅读本文件、`PROJECT_STATUS.md` 和 `CURRENT_TASK.md`。
2. 不得凭空假设硬件状态；明确区分已验证事实、推测和下一步实验。
3. 每次闭环实验必须保存 CSV、运行参数（或对应的 `*.meta.json`）和分析曲线。
4. 修改控制参数时，一次只改变少量变量；没有通过安全验证时，不得自动改写 PID 参数。
5. 完成任务后更新 `PROJECT_STATUS.md` 和 `EXPERIMENTS.md`；切换任务时更新 `CURRENT_TASK.md`。
6. 不删除已经验证有效的功能。历史代码移入既有 `motor_forward_test/archive/`，不得重新纳入默认构建。
7. 电机相关固件上电前保持安全锁定；烧录前断开 TB6612 的 `VM` 或使车轮悬空。
8. `PROJECT_STATUS.md` 只写当前有效状态，`CURRENT_TASK.md` 只保留一个任务，`EXPERIMENTS.md` 只做关键实验索引；详细过程放入 `archive/project_memory/` 或专题交接文档。
9. 构建通过、烧录成功、芯片校验成功和实物验收通过必须分别表述，不得互相替代。

## 代码与验证入口

- 当前工程：`motor_forward_test/`
- 小球闭环首测：`tests/bringup/ball_beam_vision_control_test.c`
- 控制器：`ball_beam_controller.c`；步进接口：`stepper_beam.c`
- K230 视觉与 UART：`k230/steel_ball_uart_yolo.py`、`vision_uart.c`
- 日志分析：`tools/analyze_latest.py`；评分/安全检查：`tools/score_test_runs.py`、`tools/check_safety_readiness.py`
- 主机侧回归测试：在 `motor_forward_test/` 运行 `make check-host`
- 统一比赛固件：`competition_selector_main.c`，构建目标 `make competition-unified`

## 交接规则

结束一个长对话前，只将仍有效的结论写入状态文件；无效尝试和重复讨论不进入项目记忆。新对话必须先读取上述三份根目录文件、当前相关代码，以及与当前任务直接相关的最新 CSV、`*.meta.json` 和分析图，再提出方案。不要为了“完整”把历史过程重新复制回状态文件。
