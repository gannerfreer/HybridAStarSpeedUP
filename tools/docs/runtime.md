# tools/runtime.py

- 作用：E0 工具共享的运行时基础设施——路径解析、原子写入与哈希、进程身份/终止、`/proc` 资源读数，以及全局默认常量与退出码定义。
- 入口/用法：无 CLI。主要接口：

| 分类 | 接口 |
| --- | --- |
| 路径 | `repo_root_from`、`git_common_dir`、`runtime_dir`、`locks_dir`、`evidence_lock_path`、`live_dir`、`runs_dir`、`safe_name` |
| 写入/读取 | `atomic_write_bytes/json/text`（同目录 `.tmp` + `os.replace`，返回 SHA256）、`read_json`、`sha256_file/bytes`、`append_jsonl`（可选共享锁，超时默认 30 秒）、`list_relative_files`（跳过符号链接）、`now_iso` |
| 进程 | `pid_alive`、`process_start_ticks`（`/proc/<pid>/stat` 第 22 字段）、`pid_identity_matches`、`process_group_alive`（僵尸视为已退出）、`stop_process_tree` |
| 资源 | `read_meminfo`、`read_mem_percent`、`read_cpu_times`、`process_group_rss_bytes` |

- `stop_process_tree(pid, pgid, expected_start_ticks=None, clock=None, grace=None, on_event=None, on_tick=None)` 返回 `already_exited`/`pid_reused`/`own_group`/`sigterm`/`sigkill`/`sigkill_sent`；先 SIGTERM，`STOP_GRACE_S=5.0` 秒后 SIGKILL。
- 关键常量：`SAMPLE_INTERVAL_S=2.0`、`STATUS_MAX_AGE_S=10.0`、`THRESHOLD_PERCENT=90.0`、`RESUME_CONSECUTIVE_SAMPLES=3`、`STOP_GRACE_S=5.0`、`MONITOR_HEARTBEAT_MAX_AGE_S=6.0`、`DEFAULT_WORKER_CAP=4`、`DEFAULT_MAX_PARALLEL=4`、`TEMP_SUFFIX=".tmp"`。
- 退出码：`EXIT_OK=0`、`EXIT_USAGE=2`、`EXIT_DATA=65`、`EXIT_UNAVAILABLE=69`、`EXIT_SOFTWARE=70`、`EXIT_TEMPFAIL=75`。
- 输入/输出：计算 `planner-runtime/`、`artifacts/live`、`artifacts/runs` 路径；原子写 JSON/文本文件（临时名含 pid，`fsync` 尽力而为）；读取 `/proc`（stat/meminfo/statm）。
- 依赖：标准库 hashlib/json/os/re/subprocess/pathlib；内部 `tools/clock.py`、`tools/locking.py`；外部命令 `git rev-parse --git-common-dir`（失败时回退 `<repo>/.git`）。
- 调用关系：被 `registry.py`、`rundir.py`、`fixtures.py`、`task_runner.py`、`resource_monitor.py`、`report_results.py`、`sample_report.py` 调用；不反向依赖上层模块。
- 证据与约定：所有 JSON 状态原子写入；终止前核对进程启动标识防止 PID 复用误杀；拒绝终止自身进程组（`own_group`）；退出码常量是入口与报告层的共同语义来源。
- 辅助细节：`safe_name` 把任意文本清理为 `[A-Za-z0-9_.-]` 片段（run ID 与输入文件名用）；`list_relative_files` 按目录/文件名排序返回普通文件。
- 平台约定：仅 Linux（依赖 `/proc`、进程组信号与 flock），与 E0 平台锁定一致。
- 源码与测试：`tools/runtime.py`；`tools/tests/test_process.py`（进程身份/终止）、`tools/tests/test_monitor.py`（`/proc` 读数消费方）。
- 相关文档：[tools/README.md](../README.md) · [代码组织结构](../../Docs/structure.md)
