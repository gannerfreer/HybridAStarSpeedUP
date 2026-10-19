# tools/sample_report.py

- 作用：E0-6 固定样例报告流程：重建合成样例运行目录并生成带“样例 / 非实验结果”标注的报告，验证“封存 → 清单校验 → 报告”全链路；**不是实验证据**。
- 入口/用法：`python3 tools/sample_report.py [--repo-root PATH]`（无其他参数；不占 worker 名额、不登记、不需要资源监控）。
- 输入/输出：
  - 重建 `artifacts/samples/report_sample/`（重复运行先删除该目录，只允许重建它），经 `rundir.init_run` / `append_metric_row` / `append_resource_row` / `record_event` / `write_task_summary` / `seal_run` 封存。
  - 输出到 `artifacts/reports/report_sample/`：`<run_id>.md`、`<run_id>.csv`、`status.json`，均标注样例。固定 run ID 为 `20260101T000000000000Z_sample_report_deadbeef`。
  - 样例运行目录含 `manifest.json`、`config.json`、`metrics.csv`、`resources.csv`、`events.jsonl`、`junit.xml`、`summary.md`、`artifacts.sha256.json`（封存后不可变）；封存锁为 `artifacts/samples/report_sample.lock`。
- 依赖：CPython 3.8+ 标准库；内部模块 `tools/report_results.py`、`tools/rundir.py`、`tools/runtime.py`、`tools/clock.py`（`FakeClock`）。
- 调用关系：调用 `report_results.generate(..., sample=True)`；不参与正式任务派发。
- 证据与约定：
  - 样例内容固定：4 条 metrics（success / failed / resource_stopped / interrupted 各一）、3 条资源样本、4 用例 JUnit XML（1 失败）、1 条 `resource_stopped` 事件；封存状态 `resource_stopped`（exit_code 143，原因 `resource_stopped:cpu_percent=95.0>90.0`）；manifest 含 `sample: true`。
  - 样例目录未通过清单校验视为内部错误；所有输出显著标注“样例 / 非实验结果”，不得作为算法证据。
  - 在证据链中的位置：E0-6“报告流程”验收环节（合成数据 → 封存 → 校验 → 报告）；真实实验证据仍以 `artifacts/runs/` 与 `Docs/reports/` 为准。
- 退出码：0 成功；65 数据/校验错误（样例重建、封存或报告生成失败）；docstring 另列 70 内部错误（当前实现路径见源码）。
- 源码与测试：`tools/sample_report.py`；`tools/tests/test_report.py`（`test_sample_report_script_flow`）。
- 相关文档：[tools/README.md](../README.md) · [代码组织结构](../../Docs/structure.md)
