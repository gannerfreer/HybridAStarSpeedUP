# tools/resource_monitor.py

- 作用：E0 资源监控长驻进程：每 2 秒采样系统 CPU/内存与任务树 RSS，执行 90% 阈值处置（暂停派发、终止最新任务），发布心跳与派发状态，并在启动时恢复残留登记。
- 入口/用法：

```sh
python3 tools/resource_monitor.py            # 前台长驻；由监控 agent 看护
python3 tools/resource_monitor.py --once     # 冒烟自检：采样一轮后退出
```

| 参数 | 说明 | 默认 |
| --- | --- | --- |
| `--repo-root` | 仓库根目录（默认由脚本位置推导） | 自动 |
| `--interval` | 采样间隔秒（正式运行固定 2.0，仅诊断调整） | 2.0 |
| `--status-interval` | `status.json` 最大写入间隔秒 | 10.0 |
| `--stop-grace` | SIGTERM 到 SIGKILL 的宽限秒 | 5.0 |
| `--once` | 只采样一轮后退出（等价 `--max-cycles 1`） | 关闭 |
| `--max-cycles` | 采样轮数上限（测试/诊断） | 不限 |

- 输入/输出：
  - 读 `planner-runtime/registry.json`（活动任务的 PID、进程组、进程启动标识、run 目录）与 `planner-runtime/dispatch_state.json`（重启后恢复暂停状态）。
  - 写 `planner-runtime/monitor_heartbeat.json`（`status`/`updated_at`/`updated_monotonic`/`interval_s`；调度器在缺失、`stopped` 或超 6 秒未更新时拒绝派发）；写回 `dispatch_state.json`（`paused`/`reason`/`since`/`consecutive_ok_samples`）；阈值处置写 `planner-runtime/stop_reasons/<run_id>.json`。
  - 写 `artifacts/live/status.json`（至多每 10 秒；暂停/恢复状态变化立即写）与 `artifacts/live/events.jsonl`（异常事件立即记录，`planner-runtime/locks/live-events.lock` 保护）。
  - 向每个活动运行目录追加 `resources.csv`：`time_utc, monotonic_elapsed_s, cpu_percent, mem_percent, task_tree_rss_bytes, active_tasks, build_task_id`。
  - 单实例锁 `planner-runtime/monitor.lock`；被占用时第二个监控进程退出码 69。
- 依赖：CPython 3.8+ 标准库（`/proc`、POSIX 信号）；内部模块 `tools/runtime.py`、`tools/registry.py`、`tools/rundir.py`、`tools/locking.py`、`tools/clock.py`。
- 调用关系：由监控 agent 按 AGENTS.md 启动，是任务派发的前置；`tools/task_runner.py` 派发前读心跳与 `dispatch_state.json`，运行中的受管任务每 1 秒复查心跳、失效时终止自身子进程并记录 `monitor_lost`。阈值处置后本进程等待 supervisor 释放登记（`seal_wait` 默认 10 秒），超时则自行按中断封存并释放登记——supervisor 负责正常收尾，monitor 负责兜底。
- 证据与约定：
  - CPU 口径：`/proc/stat` 聚合计算，忙时 = 总增量 − 空闲增量（首轮建立基线返回 0）；内存口径 `(MemTotal − MemAvailable) / MemTotal`；任务树 RSS 按登记进程组聚合，为采样估计（见 README §8 已知限制）。
  - 严格 `> 90.0` 触发（恰好 90.0 不触发）；触发即暂停派发并终止**最新启动**的本仓库任务：先 SIGTERM 进程组，5 秒后 SIGKILL，终止前核对 `/proc/<pid>/stat` 进程启动标识防 PID 复用误杀，运行标记 `resource_stopped`。无本仓库任务时只暂停并报警 `dispatch_paused_no_task`，不终止外部进程；连续 3 次采样 ≤90% 才恢复派发（`dispatch_resumed`）。
  - 启动恢复：核对残留登记——仍存活的遗留任务按中断终止并封存，未完成运行标记 `failed`/`interrupted`；另扫描 `artifacts/runs/` 中 `status=running` 且未封存（无 `artifact_sha256`）的目录并恢复。
  - 正式入口不暴露模拟开关；单测通过构造器注入时钟/采样器，假读数不可能作用于真实监控或正式任务。
- 源码与测试：`tools/resource_monitor.py`；`tools/tests/test_monitor.py`。
- 相关文档：[tools/README.md](../README.md) · [代码组织结构](../../Docs/structure.md)
