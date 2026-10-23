# A2 报告与提交文件

根据 `a2.pdf` 的题目与分值建立，沿用 A1 的封面、学生信息、页眉、配色和代码样式。Part A 已填写；Part B 包含模型、UCT/MCTS 伪代码、实现说明和实测实验分析，完整数据与测试位于相邻的 `../A2_code/`。

## 文件

- `u8076655_A2_report.tex`：报告主文件；学生信息沿用 A1（Rui Luo / u8076655）。
- `u8076655_A2_report.pdf`：已编译的完整报告。
- `u8076655_A2_scenario_gallery.pdf`：全部 76 个评价配置与老师示例的轨迹、每步耗时图册；不包含调参场景。
- `partB_visualisations.tex`：末尾的 12 个代表性案例，分成三页；完整单图与来源索引位于代码目录的 `results/visualisations/`。
- `partB*.tex`、`tables/`：Part B 正文、算法与实验明细附录（`partB_details.tex`）及自动生成的实测表格。
- `u8076655_A2_code.zip`：单一顶层目录的源码、测试、场景、原始数据和报告源文件。
- `AI_prompts.txt`：本会话原语言 AI 提示词记录，包括用户提供的草稿。
- `a2.pdf`：作业要求。
- `figures/alpha_envelope.pdf`：Q2 上包络矢量图；同目录的 PNG 用于预览。
- `scripts/plot_alpha_envelope.py`：生成该图的脚本，依赖 NumPy 和 Matplotlib。

## 运行 Part B

在工作区的 `../A2_code/` 目录运行；若使用已解压的源码包，则在压缩包的顶层目录运行：

```bash
python run_u8076655.py --config environments/demo_open_2d.yaml
python run_u8076655.py --config environments/demo_four_3d.yaml
python -m unittest discover -s tests -v
```

默认每步规划预算为 9.5 秒；无图形界面即可运行。完整使用说明、可选可视化、实验复现命令和覆盖范围见代码目录的 `README.md`（源码包内为 `../README.md`）。

## 编译

在本目录运行：

```bash
latexmk -pdf -interaction=nonstopmode -halt-on-error u8076655_A2_report.tex
```

保留 PDF、清除编译辅助文件：

```bash
latexmk -c u8076655_A2_report.tex
```

使用与 A1 相同的 pdfLaTeX 工作流。正文模板为英文；中文提示词保存在 UTF-8 文本文件中，无需额外中文字体或宏包。

重新生成 Q2 图后，再运行上述 LaTeX 编译命令：

```bash
python3 scripts/plot_alpha_envelope.py
```

轨迹与耗时图只读取已有 seed-0 记录，不重跑规划器。在代码目录运行 `python scripts/plot_scenario_gallery.py` 可重新生成完整图册和附录中的三张组合图。图中区分默认演示预算与放宽截止时间的固定模拟次数实验；实验耗时不能作为满足演示 10 秒限制的证据。

主要实验固定每步 2,000 次模拟；预算研究和无搜索对照另列。演示同时受 9.5 秒预算与 2,000 次上限约束。正文保留模型、伪代码、实验设计、关键图和统计分析；附录保留支撑结论的均值与置信区间。目录清单、完整调参过程、重复回归/预算明细和未采用方法的探针不再展开于 PDF，相关数据和图表仍保存在源码包中。

## 内容与检查

1. 核对主文件开头的姓名、UID、课程和学期。
2. 当前报告已填入正文；后续修改后重新编译并检查公式、表格和分页。
3. Part A：第 1 题 10 分；第 2 题 (a) 4 分、(b) 6 分；第 3 题 10 分。
4. Part B：MDP 建模 3 分、在线求解器与伪代码 4 分、Python 实现与演示 8 分、实验分析 20 分。模型、伪代码、代码和实验须一致。
5. B4 分别考虑状态空间大小、动作空间大小和环境拥挤程度，说明控制变量；每个场景重复运行并报告均值与 95% 置信区间，讨论各因素和求解器组件的影响。
6. 实测原始数据位于代码目录的 `results/`（源码包内为 `../results/`）；不要用示例数字覆盖它们。重新运行时保留源码版本与实验元数据。
   Part B 正文中的实验数字通过 `\val{...}` 引用 `tables/numbers.tex`（由 `scripts/analyse_results.py` 生成），不要在正文里手改数字；重跑分析后重新编译即可。
7. 后续提示词继续原文追加到 `AI_prompts.txt`；其他工具或独立对话的使用也需按实际补全披露。
8. 源码压缩包内包含报告源文件和文件哈希，可独立解压检查运行。

可检查尚未填写的标记：

```bash
rg -n 'placeholder\{|TODO' u8076655_A2_report.tex
```

## 提交提醒（依据 a2.pdf）

- Canvas 分别提交一份非编程题答案 PDF，以及一个 `.zip` / `.7z` / `.tar.gz` 源码与测试用例压缩包。
- 源码压缩包须有单一顶层文件夹，并保留目录结构；包括报告所用测试场景、运行所需文件和 AI 提示词记录。
- 程序使用 Python，每一步规划最多 10 秒，不得并行化。演示时需要下载已提交的压缩包，在实验室电脑上运行。
- 截止时间：2026 年 10 月 26 日 09:00（Canberra 时间）；宽限期至当日 21:00。宽限期结束后不接受提交。
- 另有现场演示与口试；具体安排见作业说明和课程通知。

此报告沿用学生自建模板；若课程发布更新要求，以课程要求为准。
