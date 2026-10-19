# tools/report_results.py

- 作用：由运行目录（`artifacts/runs/<run_id>/`）只读生成验收/实验报告：Markdown、汇总 CSV 与机器可读 `status.json`；每次生成校验产物清单并检测篡改。
- 入口/用法：

```sh
python3 tools/report_results.py --run-dir artifacts/runs/<run_id>
python3 tools/report_results.py --run-dir artifacts/runs/<run_id> --sample   # 样例模式
```

| 参数 | 说明 | 默认 |
| --- | --- | --- |
| `--run-dir` | 运行目录（必填） | — |
| `--output-dir` | 输出目录 | `artifacts/reports/<run_id>/` |
| `--sample` | 样例模式：所有输出标注“样例 / 非实验结果” | 关闭 |
| `--repo-root` | 仓库根（默认由运行目录推导） | 自动 |

- 输入/输出：只读 `manifest.json`、`metrics.csv`、`resources.csv`、`events.jsonl`、`junit.xml` 与 `artifacts.sha256.json`（经 `rundir.verify_run_dir` 校验）；输出 `<run_id>.md`（按 `Docs/reports/report_template.md` 的六节结构）、`<run_id>.csv`（`metric,value` 行）与 `status.json`（`type=planner_run_status`，含 overview/counts/verification）。不写运行目录；正式报告由集成负责人复制/链接到 `Docs/reports/<component>/`。
- 依赖：CPython 3.8+ 标准库；内部模块 `tools/rundir.py`、`tools/runtime.py`、`tools/clock.py`。
- 调用关系：独立 CLI；被 `tools/sample_report.py` 调用生成样例报告；其输出供 `Docs/reports/STATUS.md` 与组件验收报告使用。
- 证据与约定：
  - 清单校验失败（逐项大小/内容、清单外额外文件、清单哈希与 `manifest.artifact_sha256` 不符）时**仍生成报告**（标注失败），退出码 65。
  - `--sample` 或 manifest 标记 `sample: true` 时 md/csv/status 全部带“样例 / 非实验结果”横幅，不得作为算法正确性或性能证据。
  - 计数来自 metrics 的 `status` 列（无 metrics 时退回 `manifest.status`）；性能摘要为 nearest-rank P50/P95、CPU 总时长、任务树 RSS 峰值（metrics 与 resources 取最大）与系统 CPU/内存峰值。
  - 报告结构固定六节：结论、复现信息、正确性、性能、资源与异常、产物与下一步；“未测量”与原始记录保留，不推导下一步。
- 退出码：0 成功；65 运行目录/清单错误（含校验失败）；2 参数错误（argparse）。
- 源码与测试：`tools/report_results.py`；`tools/tests/test_report.py`。
- 相关文档：[tools/README.md](../README.md) · [代码组织结构](../../Docs/structure.md)
