# tools/registry.py

- 作用：`planner-runtime/` 下的共享活动登记、worker 名额排队、编译锁与心跳/派发状态/停止原因读写；无 CLI，是供入口与监控复用的库模块。
- 入口/用法：无命令行。主要接口：
  - `Registry(runtime_dir_path, worker_cap=DEFAULT_WORKER_CAP, dispatch_state_path=None)`：`reserve(...)`、`release(task_id)`、`update_process(task_id, pid, pgid, start_ticks)`、`read()/tasks()/count_workers()`、`compile_lock()`、`read_dispatch_state()`、`clear_tasks()`；
  - 模块级：`heartbeat_path` / `read_heartbeat` / `heartbeat_state(max_age=6.0)`、`write_dispatch_state`、`stop_reason_path` / `write_stop_reason` / `read_stop_reason` / `clear_stop_reason`、`newest_task(tasks)`。
- 关键参数：

| 参数 | 说明 | 默认 |
| --- | --- | --- |
| `Registry(worker_cap)` | 全局 worker 名额上限；≤0 抛 `RegistryError` | `4` |
| `reserve(kind, component, workers, run_id, run_dir, task_id)` | 原子预留；`kind` ∈ `build/test/run/benchmark/exec` | — |
| `reserve(queue_timeout, poll_interval)` | 排队等待上限 / 轮询间隔 | `3600.0` / `0.5` |
| `LOCK_TIMEOUT_S`（登记文件锁获取超时） | 常量 | `30.0` |
| `heartbeat_state(max_age)` | 心跳新鲜度上限（秒） | `6.0` |

- 输入/输出：读写 `planner-runtime/registry.json`（`schema_version/type/updated_at/next_seq/tasks`；条目含 task_id、kind、component、workers、run_id、run_dir、pid、pgid、process_start_ticks、started_at、started_monotonic、seq）、`locks/registry.lock`、`build.lock`、`stop_reasons/<run_id>.json`；只读 `dispatch_state.json`（监控写）与 `monitor_heartbeat.json`；全部原子写入。
- 依赖：标准库 os/pathlib；内部 `tools/clock.py`、`tools/locking.py`（FileLock）、`tools/runtime.py`（原子 JSON、默认值、时间格式）。
- 调用关系：被 `task_runner.py`（登记、编译锁、心跳门禁、停止原因）与 `resource_monitor.py`（发布派发状态、写停止原因、`newest_task` 选停、启动恢复）调用；自身调用 runtime/locking/clock。
- 证据与约定：
  - worker 名额只统计 `test`/`run`/`exec`（`SLOT_KINDS`）；`benchmark` 需无其他登记的独占窗口（`EXCLUSIVE_KINDS`）；`build` 只持全局唯一 `build.lock`；
  - `heartbeat_state` 返回 `None`（正常）/`monitor_missing`/`monitor_stopped`/`monitor_lost`；
  - 异常语义：`DispatchPaused`（监控暂停）、`QueueTimeout`（排队/锁超时）、`ExclusiveWindowBusy`、`GateFailure`（外部门禁）、`RegistryError`；`Reservation` 释放幂等且可作上下文管理器。
- 源码与测试：`tools/registry.py`；`tools/tests/test_registry.py`。
- 相关文档：[tools/README.md](../README.md) · [代码组织结构](../../Docs/structure.md)
