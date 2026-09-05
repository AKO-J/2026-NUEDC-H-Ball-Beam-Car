# SMARTCAR 现场操作与安全

更新时间：2026-08-06

## 不可跳过的安全门

烧录或首次运行电机相关固件前：

- 断开 TB6612 `VM`，或确保左右车轮可靠悬空。
- 取下钢球；手、线材和工具离开摆杆运动范围。
- 确认 X42S 未顶机械限位，步进输出仍处于安全锁定。
- 确认 MSPM0、K230、驱动器和传感器共地。
- 核对要烧录的映像名和 SHA-256。
- 烧录完成只代表写入；调试器未自动启动时需人工复位，但不能把 RESET 当作 START。

任何异常优先断开 `VM`。未完成架空验证前禁止落地。

## 构建与烧录

```sh
cd motor_forward_test
make -B competition-unified check-host
make flash-competition-unified
```

期望映像：`build/competition_unified.out`。必须分别保存：构建返回结果、主机测试结果、烧录器写入结果、`Program verification successful` 输出和实际映像哈希。

常用独立目标仍可用于隔离诊断：

```sh
make -B line-follow check-host
make -B ball-beam-task3-test check-host
make -B joint-line-balance-test check-host
```

不要从 `archive/` 构建或烧录历史程序。

## 统一固件架空验收

1. 上电/复位，确认停留在选择页，车轮和步进均不动作。
2. 分别复位测试 S2、S3、S4；每次只应选择对应功能。
3. S1 确认后保持按住片刻，验证释放前不会触发任务内部启动。
4. 按各任务自己的 OLED 流程推进；不跳过标定。
5. 在执行器允许动作后短时运行，并立即测试 S1 急停。
6. 检查急停同时停止车轮、停止步进并禁用步进输出。
7. 记录每个入口的通过/失败，不用一个入口的结果代替另一个。

## 联合模式流程

1. 第一次 S1：解锁调平，车轮保持停止。
2. 第二次 S1：从下机械端软件 POS=0 相对抬升约 `-565` 步。
3. S2/S3：按住人工微调水平。
4. 第三次 S1：保存杆水平和当前视觉目标，进入 READY。
5. 第四次 S1：启动预倾、PWM 预升和持续循迹。
6. 运行中再次按 S1：急停。

K230 缺失可改变状态指示和使视觉控制失效，但按现有交接不阻止第四次 S1；因此操作者必须在启动前主动确认视觉新鲜、球在安全位置。推荐 `vision_age_ms < 150 ms`，启动前球误差在 ±1 cm 内。

## 日志与分析

第三题/通用日志：

```sh
.venv/bin/python tools/record_serial.py --port /dev/cu.usbmodemMG3500011
.venv/bin/python tools/analyze_latest.py --logs logs --output analysis
```

联合日志：

```sh
.venv/bin/python tools/record_joint_serial.py --port /dev/cu.usbmodemMG3500011
.venv/bin/python tools/analyze_joint.py logs/joint/joint_YYYYMMDD_HHMMSS.csv
```

端口名以现场实际枚举为准。每次闭环实验必须同时保留 CSV、同名 meta、分析图/摘要、固件哈希、安全条件、人工干预和本次唯一变量。

## 验收用语

| 层级 | 允许表述 |
| --- | --- |
| 编译/链接 | “构建通过” |
| 电脑单元测试 | “主机回归通过” |
| XDS110 写入 | “烧录成功” |
| 芯片读回/校验 | “芯片内 verification 成功” |
| 车轮悬空/VM断开检查 | “架空流程通过” |
| 实际赛道动作和指标 | “实物/落地验收通过” |

后一层不能由前一层推导。用户口头确认可以记录，但必须明确写成“用户现场确认”，不能冒充 CSV 分析结论。
