# SMARTCAR 证据保留与删减策略

更新时间：2026-09-05

## 清理原则

先做可恢复归档，再做删除。大文件、重复交付包和训练中间产物最占空间，但源码体积小、不可替代；不要按文件大小盲删。执行删除前先生成清单、文件数量、总大小和关键文件 SHA-256，并至少保留一份离线完整备份。

## 最小核心保留集

### 必须保留

- `AGENTS.md`、`CORE_DOCS/`、`PROJECT_STATUS.md`、`CURRENT_TASK.md`、`EXPERIMENTS.md`。
- `motor_forward_test/Makefile` 和统一固件实际依赖的 `.c/.h`。
- `competition_selector_main.c`、`competition_entries.h`、三个任务入口及其驱动。
- `motor_forward_test/k230/steel_ball_uart_yolo.py`、ROI 配置和当前 `.kmodel`。
- `motor_forward_test/tests/host/` 与仍用于安全 bring-up 的测试。
- 当前烧录脚本、目标配置和必要工具脚本。
- 已验证样本的 CSV、meta、分析图/摘要；至少保留 `run_20260801_123833` 完整证据链。
- 影响当前参数决策的联合实验：Kp=11、12、14 和架空起转/故障样本对应的原始证据。
- 当前统一映像及哈希；若未来能从干净源码稳定重建，可把旧中间目标移出核心树，但仍建议保留最终比赛映像。
- 排针/接线信息在完全合并并人工核对前，保留 `H_TASK_1_40引脚总表.md`。

### 建议移入冷归档，而非直接删除

- `motor_forward_test/archive/` 和 `archive/project_memory/`。
- 旧交接 Markdown、旧 docx、设计报告草稿和生成脚本。
- 全量历史日志、失败实验和旧分析图；失败记录可能解释为什么参数被否决。
- 数据集源文件、标注集、训练 run、`.pt`、`.onnx` 和转换工具。若未来可能重训，这些应放到独立“训练资产归档”，不和运行工程混放。
- 完整工作区 ZIP、早期工程 ZIP、旧交付包和 `deliverables/` 的展开副本。

### 通常可重建或可删除的候选

以下项目在确认没有唯一内容、且完整备份已验证后，通常可删：

- `__pycache__/`、`*.pyc`、`.DS_Store`。
- `motor_forward_test/build/` 中除最终交付映像外的编译中间文件。
- 训练输出中的批次预览、重复曲线、缓存和可由原始数据+配置重建的中间产物。
- 已有原始 CSV 后生成的重复临时预览，但正式分析结论对应的图/JSON/摘要要留。
- 已安装的软件安装镜像，例如根目录 `UniFlash_9.6.0_macOS.dmg`，前提是安装来源和版本可重新获得。
- `tmp/`、异常生成的单文件目录或明显重复展开包，但必须先检查内容。

## 当前空间热点

2026-08-06 只读盘点显示：`motor_forward_test/` 约 2.9 GB；其中已明确看到数据集、训练 run、日志、分析和构建产物。根目录另有约 301 MB 的 UniFlash 安装镜像、44 MB 的 `tmp/`、17 MB 的 `true` 单文件，以及多个 ZIP/RAR/docx。2.9 GB 中还有环境或训练依赖等未在简表展开的内容，删除前应再按一级目录生成精确清单。

2026-09-05 复核显示整个工作区约 3.2 GB，主要空间来自：`.venv-xanylabeling/` 约 1.1 GB、`.venv-yolo/` 约 1.1 GB、`.venv/` 约 121 MB、`training/` 约 302 MB、根目录 UniFlash DMG 约 301 MB。它们合计约 2.9 GB，绝大部分是环境、安装包或训练资产，而不是最终固件源码。最终轻量归档校验后，可优先清理前三个虚拟环境和 DMG；`training/` 只有在确认不再需要重训或已有离线副本后再清理。

## 结题后的清理优先级

1. 低风险、可重新获得：三个 Python 虚拟环境、`__pycache__`、`.DS_Store`、`tmp/`、UniFlash DMG、构建中间文件。
2. 需确认是否留离线副本：训练环境 wheel/安装器、训练 runs、数据集源文件、旧工作区 ZIP。
3. 不清理：最终源码和映像、当前 K230 模型与配置、接线/安全文档、设计报告、获奖证明、关键 CSV/meta/分析图、最终归档包及哈希。

项目“归档”首先是停止活动开发并固定可复现证据，不等于必须把 3.2 GB 全部压成一个包。把可重建依赖打进归档会浪费空间，也增加以后校验成本。

## 推荐的最终目录形态

```text
SMARTCAR/
├── AGENTS.md
├── CORE_DOCS/
├── PROJECT_STATUS.md
├── CURRENT_TASK.md
├── EXPERIMENTS.md
├── motor_forward_test/
│   ├── Makefile
│   ├── 核心源码与头文件
│   ├── k230/（脚本、配置、当前模型）
│   ├── tests/（host + 必要 bringup）
│   ├── tools/（记录、分析、安全检查）
│   ├── evidence/（少量决定性实验）
│   └── build/competition_unified.out
└── cold_archive/  （最好放到工作区外或离线盘）
```

## 删除前检查表

- 核心源码能否在干净环境构建。
- `make check-host` 是否通过。
- 当前模型、配置、烧录脚本和工具依赖是否齐全。
- 决定性实验是否同时有 CSV、meta 和分析结论。
- 归档是否能解压，哈希是否已保存。
- 是否存在只有旧文档才记录的接线、标定或人工结论。
- 删除清单是否经过人工确认。

## Skills 建议

- 当前 Markdown 收敛不需要额外 skill；保持纯文本最利于版本管理和长期维护。
- 需要交付正式 Word 时使用 `documents` skill；需要生成并视觉检查 PDF 时使用 `pdf` skill。
- 要把大量实验 CSV 汇总为单一台账时使用 `spreadsheets` skill，但原始 CSV 仍需保留。
- 如果以后频繁执行同一套“读状态 → 查最新证据 → 构建 → 安全检查 → 更新四份状态文件”的流程，可以用 `skill-creator` 制作项目专用 skill。该 skill 应只自动化读取、检查和生成报告，不能自动烧录、驱动电机、删除文件或修改 PID。
- 不建议为这次清理安装第三方远程插件；项目核心工作都在本地文件系统，增加连接器不会提高证据可靠性。
