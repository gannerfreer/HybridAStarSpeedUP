"""共享任务登记、worker 名额、编译锁与心跳状态。

约定（§5.3）：

- 所有状态位于共享 ``planner-runtime/``（`git rev-parse --git-common-dir`）。
- 登记文件写入受 ``registry.lock`` 独占文件锁保护；读取使用共享锁。
- worker 名额只统计 ``test``、``run`` 与 ``exec`` 任务；``build`` 使用全局唯一编译锁；
  ``benchmark`` 需要独占工作负载窗口（无其他登记任务）。
- 派发状态（暂停/恢复）由资源监控写入，任务入口只读。
- 本模块只使用 CPython 3.8+ 标准库。
"""

import os
from pathlib import Path

from tools import clock as clock_mod
from tools import locking
from tools import runtime

DEFAULT_WORKER_CAP = runtime.DEFAULT_WORKER_CAP
KINDS = ("build", "test", "run", "benchmark", "exec")
SLOT_KINDS = ("test", "run", "exec")
EXCLUSIVE_KINDS = ("benchmark",)

REGISTRY_SCHEMA_VERSION = 1
REGISTRY_FILE = "registry.json"
LOCK_TIMEOUT_S = 30.0


class DispatchError(RuntimeError):
    """登记/派发层错误基类。"""


class DispatchPaused(DispatchError):
    """资源监控已暂停派发。"""


class QueueTimeout(DispatchError):
    """等待名额、锁或独占窗口超时。"""


class ExclusiveWindowBusy(DispatchError):
    """存在其他活动任务，独占窗口不可用。"""


class GateFailure(DispatchError):
    """外部门（如心跳）判定不允许继续等待。"""


class RegistryError(DispatchError):
    pass


class Reservation(object):
    """名额预留句柄；释放幂等，可作上下文管理器。"""

    def __init__(self, registry, entry):
        self.registry = registry
        self.entry = dict(entry)
        self.released = False

    @property
    def task_id(self):
        return self.entry["task_id"]

    def release(self):
        if self.released:
            return
        self.registry.release(self.task_id)
        self.released = True

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.release()
        return False


def default_task_id(kind, component):
    return "%s:%s" % (kind, component or "unknown")


class Registry(object):
    """``planner-runtime/`` 下的活动登记与锁。"""

    def __init__(self, runtime_dir_path, worker_cap=DEFAULT_WORKER_CAP,
                 dispatch_state_path=None):
        self.runtime_dir = Path(runtime_dir_path)
        self.worker_cap = int(worker_cap)
        if self.worker_cap <= 0:
            raise RegistryError("worker_cap 必须为正整数")
        self.path = self.runtime_dir / REGISTRY_FILE
        self.lock_path = self.runtime_dir / "locks" / "registry.lock"
        self.compile_lock_path = self.runtime_dir / "build.lock"
        if dispatch_state_path is None:
            self.dispatch_state_path = self.runtime_dir / "dispatch_state.json"
        else:
            self.dispatch_state_path = Path(dispatch_state_path)
        self.lock_path.parent.mkdir(parents=True, exist_ok=True)

    # -- 读取 --------------------------------------------------------------

    def _empty(self):
        return {
            "schema_version": REGISTRY_SCHEMA_VERSION,
            "type": "planner_runtime_registry",
            "updated_at": None,
            "next_seq": 1,
            "tasks": [],
        }

    def _load_unlocked(self):
        data = runtime.read_json(self.path, default=None)
        if not isinstance(data, dict):
            return self._empty()
        tasks = data.get("tasks")
        if not isinstance(tasks, list):
            data["tasks"] = []
        if not isinstance(data.get("next_seq"), int):
            data["next_seq"] = len(data.get("tasks", [])) + 1
        return data

    def read(self):
        with locking.FileLock(self.lock_path, timeout=LOCK_TIMEOUT_S, shared=True):
            return self._load_unlocked()

    def tasks(self):
        return [dict(item) for item in self.read().get("tasks", [])
                if isinstance(item, dict)]

    def has_task(self, task_id):
        return any(item.get("task_id") == task_id for item in self.read().get("tasks", [])
                   if isinstance(item, dict))

    def count_workers(self):
        return self._count_workers(self.read())

    @staticmethod
    def _count_workers(data):
        total = 0
        for item in data.get("tasks", []):
            if not isinstance(item, dict):
                continue
            try:
                total += int(item.get("workers", 0))
            except (TypeError, ValueError):
                continue
        return total

    # -- 派发状态与心跳 ----------------------------------------------------

    def read_dispatch_state(self):
        data = runtime.read_json(self.dispatch_state_path, default=None)
        if not isinstance(data, dict):
            return {
                "schema_version": REGISTRY_SCHEMA_VERSION,
                "type": "planner_dispatch_state",
                "paused": False,
                "reason": None,
                "consecutive_ok_samples": 0,
            }
        data.setdefault("paused", False)
        data.setdefault("consecutive_ok_samples", 0)
        return data

    # -- 预留与释放 --------------------------------------------------------

    def _blocking_reason(self, data, kind, workers, dispatch_state):
        if dispatch_state.get("paused"):
            return "paused"
        tasks = [item for item in data.get("tasks", []) if isinstance(item, dict)]
        if kind == "benchmark":
            if tasks:
                return "exclusive_window"
            return None
        if any(item.get("kind") in EXCLUSIVE_KINDS for item in tasks):
            return "benchmark_active"
        active = self._count_workers(data)
        if active + int(workers) > self.worker_cap:
            return "worker_cap"
        return None

    def reserve(self, kind, component=None, workers=0, run_id=None, run_dir=None,
                task_id=None, queue_timeout=3600.0, poll_interval=0.5,
                clock=None, on_wait=None, gate=None):
        """原子预留任务名额。

        - ``test``/``run``/``exec`` 预占 ``workers`` 个名额，超过 cap 时排队等待；
        - ``build`` 默认不占名额（仅全局编译锁）；npm 构建通过 ``workers>0`` 占名额；
        - ``benchmark`` 需要没有其他登记任务的独占窗口。
        参数 ``gate`` 是可调用对象：返回字符串表示禁止继续（抛出 GateFailure）。
        """
        if kind not in KINDS:
            raise RegistryError("未知任务类型: %r" % (kind,))
        clk = clock if clock is not None else clock_mod.Clock()
        try:
            workers = int(workers)
        except (TypeError, ValueError):
            raise RegistryError("workers 必须为整数")
        if workers < 0:
            raise RegistryError("workers 不能为负")
        if kind in SLOT_KINDS and workers < 1:
            raise RegistryError("%s 任务至少占用 1 个 worker 名额" % kind)
        task_id = task_id or default_task_id(kind, component)
        deadline = None
        if queue_timeout and float(queue_timeout) > 0:
            deadline = clk.monotonic() + float(queue_timeout)
        reason = None
        while True:
            if gate is not None:
                gate_reason = gate()
                if gate_reason:
                    raise GateFailure(gate_reason)
            with locking.FileLock(self.lock_path, timeout=LOCK_TIMEOUT_S):
                data = self._load_unlocked()
                state = self.read_dispatch_state()
                reason = self._blocking_reason(data, kind, workers, state)
                if reason is None:
                    for existing in data.get("tasks", []):
                        if (isinstance(existing, dict) and
                                existing.get("task_id") == task_id):
                            raise RegistryError("task_id 已存在: %s" % task_id)
                    entry_workers = workers
                    seq = int(data.get("next_seq", 1))
                    entry = {
                        "task_id": task_id,
                        "kind": kind,
                        "component": component,
                        "workers": entry_workers,
                        "run_id": run_id,
                        "run_dir": str(run_dir) if run_dir is not None else None,
                        "pid": None,
                        "pgid": None,
                        "process_start_ticks": None,
                        "started_at": runtime.now_iso(clk),
                        "started_monotonic": clk.monotonic(),
                        "seq": seq,
                    }
                    data["tasks"].append(entry)
                    data["next_seq"] = seq + 1
                    data["updated_at"] = runtime.now_iso(clk)
                    runtime.atomic_write_json(self.path, data)
                    return Reservation(self, entry)
            if reason == "paused":
                raise DispatchPaused("资源监控已暂停派发: %s" % (
                    self.read_dispatch_state().get("reason") or "资源超限"))
            if deadline is not None and clk.monotonic() >= deadline:
                raise QueueTimeout("等待 '%s' 超时（已有任务占用名额/独占窗口）" % reason)
            if on_wait is not None:
                on_wait(reason)
            clk.sleep(poll_interval)

    def update_process(self, task_id, pid, pgid, start_ticks):
        with locking.FileLock(self.lock_path, timeout=LOCK_TIMEOUT_S):
            data = self._load_unlocked()
            changed = False
            for item in data.get("tasks", []):
                if isinstance(item, dict) and item.get("task_id") == task_id:
                    item["pid"] = int(pid) if pid is not None else None
                    item["pgid"] = int(pgid) if pgid is not None else None
                    item["process_start_ticks"] = (
                        int(start_ticks) if start_ticks is not None else None)
                    changed = True
                    break
            if changed:
                data["updated_at"] = runtime.now_iso(clock_mod.Clock())
                runtime.atomic_write_json(self.path, data)
            return changed

    def release(self, task_id):
        with locking.FileLock(self.lock_path, timeout=LOCK_TIMEOUT_S):
            data = self._load_unlocked()
            tasks = data.get("tasks", [])
            remaining = [item for item in tasks
                         if not (isinstance(item, dict) and item.get("task_id") == task_id)]
            if len(remaining) == len(tasks):
                return False
            data["tasks"] = remaining
            data["updated_at"] = runtime.now_iso(clock_mod.Clock())
            runtime.atomic_write_json(self.path, data)
            return True

    def clear_tasks(self):
        """清空登记（仅测试与显式恢复使用）。"""
        with locking.FileLock(self.lock_path, timeout=LOCK_TIMEOUT_S):
            data = self._load_unlocked()
            data["tasks"] = []
            data["updated_at"] = runtime.now_iso(clock_mod.Clock())
            runtime.atomic_write_json(self.path, data)

    # -- 锁 ----------------------------------------------------------------

    def compile_lock(self, poll_interval=0.1):
        return locking.FileLock(self.compile_lock_path, poll_interval=poll_interval)


# ---------------------------------------------------------------------------
# 心跳、派发状态、停止原因（模块级函数，监控与调度器共用）
# ---------------------------------------------------------------------------

def heartbeat_path(runtime_dir_path):
    return Path(runtime_dir_path) / "monitor_heartbeat.json"


def read_heartbeat(runtime_dir_path):
    data = runtime.read_json(heartbeat_path(runtime_dir_path), default=None)
    return data if isinstance(data, dict) else None


def heartbeat_state(runtime_dir_path, clock, max_age=runtime.MONITOR_HEARTBEAT_MAX_AGE_S):
    """返回 None（正常）、monitor_missing、monitor_stopped 或 monitor_lost。"""
    data = read_heartbeat(runtime_dir_path)
    if not isinstance(data, dict):
        return "monitor_missing"
    if data.get("status") == "stopped":
        return "monitor_stopped"
    updated = data.get("updated_monotonic")
    if isinstance(updated, (int, float)):
        age = clock.monotonic() - float(updated)
    else:
        # 回退：用 UTC 时间估算（跨进程可比）
        stamp = data.get("updated_at")
        try:
            parsed = clock_mod.parse_utc_iso(stamp)
            age = (clock.utc_now() - parsed).total_seconds()
        except (TypeError, ValueError):
            return "monitor_missing"
    if age > float(max_age) + 1e-9:
        return "monitor_lost"
    return None


def write_dispatch_state(runtime_dir_path, state, clock=None):
    clk = clock if clock is not None else clock_mod.Clock()
    payload = dict(state)
    payload["schema_version"] = REGISTRY_SCHEMA_VERSION
    payload["type"] = "planner_dispatch_state"
    payload["updated_at"] = runtime.now_iso(clk)
    runtime.atomic_write_json(Path(runtime_dir_path) / "dispatch_state.json", payload)
    return payload


def stop_reason_path(runtime_dir_path, run_id):
    return Path(runtime_dir_path) / "stop_reasons" / ("%s.json" % run_id)


def write_stop_reason(runtime_dir_path, run_id, payload):
    data = dict(payload)
    data["schema_version"] = REGISTRY_SCHEMA_VERSION
    data["type"] = "planner_stop_reason"
    data["run_id"] = run_id
    runtime.atomic_write_json(stop_reason_path(runtime_dir_path, run_id), data)
    return data


def read_stop_reason(runtime_dir_path, run_id):
    data = runtime.read_json(stop_reason_path(runtime_dir_path, run_id), default=None)
    return data if isinstance(data, dict) else None


def clear_stop_reason(runtime_dir_path, run_id):
    try:
        os.unlink(str(stop_reason_path(runtime_dir_path, run_id)))
    except OSError:
        pass


def newest_task(tasks):
    """最新启动的登记任务；以 started_monotonic 为准，seq 作为平局裁决。"""
    candidates = [item for item in tasks if isinstance(item, dict)]
    if not candidates:
        return None

    def sort_key(item):
        started = item.get("started_monotonic")
        if not isinstance(started, (int, float)):
            started = -1.0
        seq = item.get("seq")
        if not isinstance(seq, int):
            seq = 0
        return (float(started), seq)

    return max(candidates, key=sort_key)
