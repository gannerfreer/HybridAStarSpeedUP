# tools/rundir.py

- 作用：运行目录生命周期库——创建与脚手架、metrics/resources/events 写入、summary、§9 封存与清单校验、中断恢复；无 CLI。
- 入口/用法：无命令行。主要接口：

| 分类 | 接口 |
| --- | --- |
| 创建 | `new_run_id` / `parse_run_id` / `create_run_dir(runs_root, component, retry_of=None, clock=None, run_id=None)`（独占创建，自动重试上限 100 次） |
| 脚手架 | `RunLayout`（固定路径属性 + `ensure_dirs`）、`init_run`（写 config/manifest、CSV 表头、空日志与事件文件） |
| 写入 | `record_event`、`append_metric_row` / `append_resource_row` / `append_csv_row` / `read_csv_rows`、`update_manifest` |
| 封存 | `write_task_summary`、`compute_artifact_manifest`、`seal_run(run_dir, final_fields, clock=None, lock_path=None, lock_timeout=30.0)`、`verify_run_dir`、`recover_interrupted_run(...)` |
| 辅助 | `parse_junit_counts`、`make_source_archive`、`read_events`、`csv_text`、`write_text_atomic` |

- 输入/输出：写 `artifacts/runs/<run_id>/` 固定结构（manifest.json、config.json、inputs/、metrics.csv、resources.csv、events.jsonl、stdout.log、stderr.log、junit.xml、outputs/、plots/、summary.md、artifacts.sha256.json、source_untracked.tar.gz）；`METRIC_COLUMNS`（18 列）/`RESOURCE_COLUMNS`（7 列）为固定表头。
- 依赖：标准库 csv/io/json/os/tarfile/pathlib/xml.etree.ElementTree；内部 `tools/clock.py`、`tools/locking.py`、`tools/runtime.py`。
- 调用关系：被 `task_runner.py`（全生命周期）、`resource_monitor.py`（中断恢复、事件与资源行写入）、`report_results.py` / `sample_report.py`（读取与封存）及测试调用；自身调用 runtime/clock/locking。
- 证据与约定：
  - run_id 形如 `YYYYMMDDTHHMMSSffffffZ_<component>_<8位随机>`；目录已存在且显式指定 run_id 时抛 `RunDirError`；
  - 清单按 POSIX 相对路径字典序排列，排除 manifest 与清单自身及 `*.tmp`；`manifest.artifact_sha256` = 清单文件原始字节 SHA256；
  - 封存顺序为 summary→清单→manifest（`seal_run` 在锁内完成），之后目录不可变；
  - `recover_interrupted_run` 返回 `missing`/`missing_manifest`/`already_sealed`/`sealed`（锁超时为 `locked`）；`verify_run_dir` 逐项校验大小/哈希并拒绝清单外文件。
- 事件格式：`events.jsonl` 每行一条 JSON（time/task_id/run_id/component/event/details），由 `runtime.append_jsonl` 追加。
- summary.md 由 `write_task_summary` 生成（字段表 + 计数 + 备注）；封存时若缺失会补写最小摘要。
- 源码与测试：`tools/rundir.py`；`tools/tests/test_rundir.py`。
- 相关文档：[tools/README.md](../README.md) · [代码组织结构](../../Docs/structure.md)
