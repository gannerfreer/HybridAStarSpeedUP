# tools/locking.py

- 作用：基于 `fcntl.flock` 的跨进程文件锁封装，提供共享/独占两种模式与非阻塞重试，供 `planner-runtime/` 各状态文件使用。
- 入口/用法：无 CLI。`FileLock(path, timeout=None, poll_interval=0.1, shared=False)`：

| 成员 | 说明 | 默认 |
| --- | --- | --- |
| `acquire(timeout=None)` | 非阻塞重试获取；成功 True、超时 False；已持有再获取抛 RuntimeError | 用构造时的 timeout |
| `release()` | 幂等释放（LOCK_UN + close） | — |
| `unlink()` | 释放并尽力删除锁文件（仅用于运行结束后的证据锁清理） | — |
| 上下文管理器 | 获取失败抛 `LockTimeoutError` | — |
| `shared` | `True`=LOCK_SH（证据写入方共享），`False`=LOCK_EX（封存方独占） | `False` |
| `poll_interval` | 轮询间隔，下限 0.01 秒 | `0.1` |

- 输入/输出：在 `path` 所在目录创建/打开锁文件（`O_RDWR|O_CREAT`，权限 0644），不写入任何内容；锁通过 flock 施加于文件描述符。
- 依赖：标准库 errno/fcntl/os/time/pathlib；无内部模块依赖（依赖树叶子）。
- 调用关系：被 `registry.py`（`locks/registry.lock`、`build.lock`）、`runtime.py`（`append_jsonl` 事件流共享锁）、`rundir.py`（封存/恢复锁）、`resource_monitor.py`（live-events 锁、证据锁共享持有）使用；`task_runner.py` 经 registry/rundir 间接使用。
- 证据与约定：`LOCK_SH`/`LOCK_EX` + `LOCK_NB` 轮询；仅 EACCES/EAGAIN 视为“暂不可得”，其他 errno 直接抛出；显式 `acquire()` 以返回值表达超时，上下文管理器以 `LockTimeoutError` 表达超时；`unlink` 忽略删除失败。
- 关键细节：`timeout=None` 表示无限等待；`timeout=0` 等效单次尝试；锁文件父目录在 `acquire` 时自动创建。
- 文件位置约定：锁文件位于共享 Git 目录的 `planner-runtime/` 下，多个 worktree 共用同一把锁。
- 兼容性：仅支持 Linux/POSIX（`fcntl.flock`、errno），与 E0 平台约定一致。
- 源码与测试：`tools/locking.py`；`tools/tests/test_locking.py`。
- 相关文档：[tools/README.md](../README.md) · [代码组织结构](../../Docs/structure.md)
