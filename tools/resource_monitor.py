#!/usr/bin/env python3
"""E0 资源监控：2 秒采样、阈值处置、心跳与派发状态。

与 §5.3 对齐：

- CPU 用 ``/proc/stat`` 聚合计算，内存用 ``(MemTotal - MemAvailable) / MemTotal``，
  任务树 RSS 按登记进程组聚合（采样估计）。
- 写 ``artifacts/live/status.json``（至多每 10 秒）与 ``artifacts/live/events.jsonl``
  （异常事件立即记录）；活动运行目录追加 ``resources.csv``。
- CPU 或内存 > 90%：暂停派发，终止最新启动的本仓库任务（先 SIGTERM 进程组，
  5 秒后 SIGKILL，终止前核对进程启动标识），标记 ``resource_stopped``；
  连续 3 次采样 <= 90% 才恢复派发；没有可停任务时只暂停并报警。
- 心跳文件定在 ``planner-runtime/monitor_heartbeat.json``（跨 worktree 共享，
  见 tools/README.md）；心跳 > 6 秒未更新时调度器停止派发并终止受管任务。
- 启动时核对残留登记：未完成运行标记 ``failed`` / 原因 ``interrupted``，
  遗留仍存活的任务按中断终止后封存。
- 单测通过构造器注入时钟与采样器；正式入口不暴露模拟开关，模拟读数不可能
  作用于真实监控或正式任务。

本模块只使用 CPython 3.8+ 标准库，平台为 Linux。
"""

import argparse
import os
import signal
import sys

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools import clock as clock_mod
from tools import locking
from tools import registry as registry_mod
from tools import rundir as rundir_mod
from tools import runtime


class SystemSampler(object):
    """正式采样器：只读 /proc，不执行任何仓库代码。"""

    def __init__(self):
        self._cpu_total = None
        self._cpu_idle = None

    def cpu_percent(self):
        current = runtime.read_cpu_times()
        if current is None:
            return 0.0
        total, idle = current
        if self._cpu_total is None:
            self._cpu_total, self._cpu_idle = total, idle
            return 0.0
        total_delta = total - self._cpu_total
        idle_delta = idle - self._cpu_idle
        self._cpu_total, self._cpu_idle = total, idle
        if total_delta <= 0:
            return 0.0
        busy = max(0, total_delta - max(0, idle_delta))
        return 100.0 * busy / float(total_delta)

    def mem_percent(self):
        value = runtime.read_mem_percent()
        return None if value is None else float(value)

    def sample(self, tasks):
        rss = {}
        for entry in tasks:
            pgid = entry.get("pgid")
            if pgid is None:
                continue
            rss[entry.get("task_id")] = runtime.process_group_rss_bytes(pgid)
        return {
            "cpu_percent": self.cpu_percent(),
            "mem_percent": self.mem_percent(),
            "task_rss": rss,
        }


class ResourceMonitor(object):
    """采样与阈值处置核心；时钟、采样器、终止器均可注入用于单测。"""

    def __init__(self, repo_root, runtime_dir_path=None, live_dir_path=None,
                 runs_root=None, clock=None, sampler=None, stopper=None,
                 interval=runtime.SAMPLE_INTERVAL_S,
                 threshold=runtime.THRESHOLD_PERCENT,
                 resume_samples=runtime.RESUME_CONSECUTIVE_SAMPLES,
                 stop_grace=runtime.STOP_GRACE_S,
                 heartbeat_max_age=runtime.MONITOR_HEARTBEAT_MAX_AGE_S,
                 status_interval=runtime.STATUS_MAX_AGE_S,
                 seal_wait=10.0, monitor_pid=None):
        self.repo_root = os.fspath(repo_root)
        self.runtime_dir = (os.fspath(runtime.runtime_dir(self.repo_root))
                            if runtime_dir_path is None else os.fspath(runtime_dir_path))
        self.live_dir = (os.fspath(runtime.live_dir(self.repo_root))
                         if live_dir_path is None else os.fspath(live_dir_path))
        self.runs_root = (os.fspath(runtime.runs_dir(self.repo_root))
                          if runs_root is None else os.fspath(runs_root))
        self.clock = clock if clock is not None else clock_mod.Clock()
        self.sampler = sampler if sampler is not None else SystemSampler()
        self.stopper = stopper if stopper is not None else self._default_stopper
        self.interval = float(interval)
        self.threshold = float(threshold)
        self.resume_samples = int(resume_samples)
        self.stop_grace = float(stop_grace)
        self.heartbeat_max_age = float(heartbeat_max_age)
        self.status_interval = float(status_interval)
        self.seal_wait = float(seal_wait)
        self.monitor_pid = int(monitor_pid) if monitor_pid is not None else os.getpid()

        self.registry = registry_mod.Registry(self.runtime_dir)
        self.heartbeat_path = registry_mod.heartbeat_path(self.runtime_dir)
        self.live_events_path = os.path.join(self.live_dir, "events.jsonl")
        self.live_events_lock = os.path.join(
            runtime.locks_dir(self.runtime_dir), "live-events.lock")
        self.status_path = os.path.join(self.live_dir, "status.json")

        self.paused = False
        self.pause_reason = None
        self.pause_since = None
        self.consecutive_ok = 0
        self.stop_count = 0
        self.last_sample = None
        self.last_status = None
        self.last_status_write = None
        self._alerted_no_task = False
        self._stop_requested = False
        self._sample_fail_count = 0

    # -- 基础设施 ----------------------------------------------------------

    def _default_stopper(self, entry, clock, grace):
        return runtime.stop_process_tree(
            entry.get("pid"), entry.get("pgid"), entry.get("process_start_ticks"),
            clock=clock, grace=grace, on_tick=self.write_heartbeat)

    def request_stop(self):
        self._stop_requested = True

    def write_heartbeat(self, status="running"):
        payload = {
            "schema_version": runtime.SCHEMA_VERSION,
            "type": "monitor_heartbeat",
            "pid": self.monitor_pid,
            "status": status,
            "updated_at": runtime.now_iso(self.clock),
            "updated_monotonic": self.clock.monotonic(),
            "interval_s": self.interval,
        }
        runtime.atomic_write_json(self.heartbeat_path, payload)
        return payload

    def live_event(self, event, task_id=None, run_id=None, details=None):
        record = {
            "time": runtime.now_iso(self.clock),
            "source": "resource_monitor",
            "task_id": task_id,
            "run_id": run_id,
            "event": event,
            "details": dict(details) if details else {},
        }
        try:
            runtime.append_jsonl(self.live_events_path, record,
                                 lock_path=self.live_events_lock)
        except (OSError, TimeoutError):
            pass
        return record

    def restore_dispatch_state(self):
        state = self.registry.read_dispatch_state()
        self.paused = bool(state.get("paused"))
        self.pause_reason = state.get("reason")
        self.pause_since = state.get("since")
        try:
            self.consecutive_ok = int(state.get("consecutive_ok_samples", 0))
        except (TypeError, ValueError):
            self.consecutive_ok = 0

    def persist_dispatch_state(self):
        state = {
            "paused": self.paused,
            "reason": self.pause_reason,
            "since": self.pause_since,
            "consecutive_ok_samples": self.consecutive_ok,
        }
        registry_mod.write_dispatch_state(self.runtime_dir, state, clock=self.clock)

    # -- 采样 --------------------------------------------------------------

    def sample_once(self):
        tasks = self.registry.tasks()
        raw = self.sampler.sample(tasks)
        sample = {
            "time": runtime.now_iso(self.clock),
            "monotonic": self.clock.monotonic(),
            "cpu_percent": _as_float(raw.get("cpu_percent")),
            "mem_percent": _as_float(raw.get("mem_percent")),
            "task_rss": dict(raw.get("task_rss") or {}),
            "tasks": tasks,
        }
        self.last_sample = sample
        self._handle_sample(sample)
        self._append_resources(sample)
        self._write_status(sample)
        self.write_heartbeat()
        return sample

    def _handle_sample(self, sample):
        over_cpu = sample["cpu_percent"] > self.threshold
        over_mem = sample["mem_percent"] > self.threshold
        if over_cpu or over_mem:
            self._handle_over(sample, over_cpu, over_mem)
        else:
            self._handle_ok()

    def _handle_over(self, sample, over_cpu, over_mem):
        metrics = []
        if over_cpu:
            metrics.append(("cpu_percent", sample["cpu_percent"]))
        if over_mem:
            metrics.append(("mem_percent", sample["mem_percent"]))
        metric, value = max(metrics, key=lambda item: item[1]) if metrics else ("cpu_percent", 0.0)
        if not self.paused:
            self.paused = True
            self.pause_since = runtime.now_iso(self.clock)
            self.consecutive_ok = 0
            self.pause_reason = "resource_threshold:%s=%.3f>%.1f" % (
                metric, value, self.threshold)
            self.live_event("dispatch_paused", details={
                "reason": self.pause_reason,
                "cpu_percent": sample["cpu_percent"],
                "mem_percent": sample["mem_percent"],
                "threshold": self.threshold,
            })
            self.persist_dispatch_state()
        else:
            self.consecutive_ok = 0
            self.persist_dispatch_state()
        candidates = [item for item in sample["tasks"] if item.get("pid") is not None]
        if candidates:
            self._alerted_no_task = False
            target = registry_mod.newest_task(candidates)
            if target is not None:
                self._stop_task(target, sample, metric, value)
        else:
            if not self._alerted_no_task:
                self._alerted_no_task = True
                self.live_event("dispatch_paused_no_task", details={
                    "reason": self.pause_reason,
                    "cpu_percent": sample["cpu_percent"],
                    "mem_percent": sample["mem_percent"],
                    "threshold": self.threshold,
                    "message": "没有本仓库登记任务可停止，仅暂停派发并报警",
                })

    def _handle_ok(self):
        if not self.paused:
            return
        self.consecutive_ok += 1
        if self.consecutive_ok >= self.resume_samples:
            previous = self.pause_reason
            self.paused = False
            self.pause_reason = None
            self.pause_since = None
            self.consecutive_ok = 0
            self.live_event("dispatch_resumed", details={"previous_reason": previous})
            self.persist_dispatch_state()
        else:
            self.live_event("dispatch_recovery_progress", details={
                "consecutive_ok_samples": self.consecutive_ok,
                "required": self.resume_samples,
            })
            self.persist_dispatch_state()

    # -- 终止 --------------------------------------------------------------

    def _stop_task(self, entry, sample, metric, value):
        task_id = entry.get("task_id")
        run_id = entry.get("run_id")
        run_dir = entry.get("run_dir")
        details = {
            "metric": metric,
            "value": float(value),
            "threshold": float(self.threshold),
            "pid": entry.get("pid"),
            "pgid": entry.get("pgid"),
            "run_dir": run_dir,
        }
        self.live_event("resource_stop_requested", task_id=task_id, run_id=run_id,
                        details=details)
        self.write_heartbeat()
        failure_reason = "resource_stopped:%s=%.3f>%.1f" % (metric, value, self.threshold)
        if run_id:
            registry_mod.write_stop_reason(self.runtime_dir, run_id, {
                "task_id": task_id,
                "status": "resource_stopped",
                "failure_reason": failure_reason,
                "reason": "resource_stopped",
                "metric": metric,
                "value": float(value),
                "threshold": float(self.threshold),
                "stopped_at": runtime.now_iso(self.clock),
                "pid": entry.get("pid"),
                "pgid": entry.get("pgid"),
            })
        outcome = self.stopper(entry, self.clock, self.stop_grace)
        self.stop_count += 1
        self.live_event("resource_stopped", task_id=task_id, run_id=run_id, details={
            "outcome": outcome,
            "metric": metric,
            "value": float(value),
            "threshold": float(self.threshold),
        })
        self._append_run_event(entry, "resource_stopped", {
            "outcome": outcome,
            "metric": metric,
            "value": float(value),
            "threshold": float(self.threshold),
            "failure_reason": failure_reason,
        })
        self._wait_or_recover(entry, failure_reason)
        self.write_heartbeat()

    def _wait_or_recover(self, entry, failure_reason, status="resource_stopped"):
        task_id = entry.get("task_id")
        deadline = self.clock.monotonic() + self.seal_wait
        while self.clock.monotonic() < deadline:
            if not self.registry.has_task(task_id):
                return "supervisor_released"
            self.write_heartbeat()
            self.clock.sleep(0.2)
        if not self.registry.has_task(task_id):
            return "supervisor_released"
        run_dir = entry.get("run_dir")
        run_id = entry.get("run_id")
        if run_dir and run_id:
            rundir_mod.recover_interrupted_run(
                run_dir, clock=self.clock, status=status,
                reason=failure_reason,
                lock_path=runtime.evidence_lock_path(self.runtime_dir, run_id),
                extra_fields={"exit_code": entry.get("exit_code")})
        self.registry.release(task_id)
        self.live_event("resource_stopped_recovered", task_id=task_id, run_id=run_id,
                        details={"reason": failure_reason, "status": status})
        return "recovered"

    # -- 运行目录与实时状态 -------------------------------------------------

    def _append_run_event(self, entry, event, details):
        run_id = entry.get("run_id")
        run_dir = entry.get("run_dir")
        if not run_id or not run_dir:
            return
        lock = locking.FileLock(
            runtime.evidence_lock_path(self.runtime_dir, run_id), shared=True)
        if not lock.acquire(timeout=0.5):
            return
        try:
            if not self.registry.has_task(entry.get("task_id")):
                return
            layout = rundir_mod.RunLayout(run_dir)
            rundir_mod.record_event(layout, self.clock, event,
                                    task_id=entry.get("task_id"),
                                    component=entry.get("component"),
                                    details=details)
        except (OSError, rundir_mod.RunDirError):
            pass
        finally:
            lock.release()

    def _append_resources(self, sample):
        tasks = sample["tasks"]
        build_task_id = ""
        for item in tasks:
            if item.get("kind") == "build":
                build_task_id = item.get("task_id") or ""
                break
        for entry in tasks:
            run_id = entry.get("run_id")
            run_dir = entry.get("run_dir")
            if not run_id or not run_dir:
                continue
            lock = locking.FileLock(
                runtime.evidence_lock_path(self.runtime_dir, run_id), shared=True)
            if not lock.acquire(timeout=0.5):
                continue
            try:
                if not self.registry.has_task(entry.get("task_id")):
                    continue
                started = entry.get("started_monotonic")
                try:
                    elapsed = sample["monotonic"] - float(started)
                except (TypeError, ValueError):
                    elapsed = 0.0
                rss = sample["task_rss"].get(entry.get("task_id"), "")
                row = {
                    "time_utc": sample["time"],
                    "monotonic_elapsed_s": "%.3f" % max(0.0, elapsed),
                    "cpu_percent": "%.3f" % sample["cpu_percent"],
                    "mem_percent": "%.3f" % sample["mem_percent"],
                    "task_tree_rss_bytes": rss,
                    "active_tasks": len(tasks),
                    "build_task_id": build_task_id,
                }
                layout = rundir_mod.RunLayout(run_dir)
                rundir_mod.append_resource_row(layout, row)
            except (OSError, rundir_mod.RunDirError):
                continue
            finally:
                lock.release()

    def _write_status(self, sample, force=False):
        now = sample["monotonic"]
        state_changed = (self.last_status is None or
                         self.last_status.get("dispatch", {}).get("paused") != self.paused)
        if not force and not state_changed:
            if (self.last_status_write is not None and
                    now - self.last_status_write < self.status_interval):
                return self.last_status
        task_rows = []
        for entry in sample["tasks"]:
            started = entry.get("started_monotonic")
            try:
                age = sample["monotonic"] - float(started)
            except (TypeError, ValueError):
                age = None
            task_rows.append({
                "task_id": entry.get("task_id"),
                "kind": entry.get("kind"),
                "component": entry.get("component"),
                "run_id": entry.get("run_id"),
                "pid": entry.get("pid"),
                "pgid": entry.get("pgid"),
                "workers": entry.get("workers"),
                "started_at": entry.get("started_at"),
                "age_s": None if age is None else round(max(0.0, age), 3),
                "rss_bytes": sample["task_rss"].get(entry.get("task_id")),
            })
        status = {
            "schema_version": runtime.SCHEMA_VERSION,
            "type": "resource_status",
            "updated_at": sample["time"],
            "monitor": {
                "pid": self.monitor_pid,
                "interval_s": self.interval,
                "heartbeat": "running",
            },
            "dispatch": {
                "paused": self.paused,
                "reason": self.pause_reason,
                "since": self.pause_since,
                "consecutive_ok_samples": self.consecutive_ok,
            },
            "system": {
                "cpu_percent": round(sample["cpu_percent"], 3),
                "mem_percent": round(sample["mem_percent"], 3),
                "thresholds": {"cpu_percent": self.threshold, "mem_percent": self.threshold},
            },
            "tasks": task_rows,
            "counters": {"stopped_tasks": self.stop_count},
        }
        runtime.atomic_write_json(self.status_path, status)
        self.last_status = status
        self.last_status_write = now
        return status

    # -- 启动恢复 ----------------------------------------------------------

    def startup_recovery(self):
        self.live_event("monitor_startup", details={
            "runtime_dir": self.runtime_dir,
            "repo_root": self.repo_root,
        })
        active_ids = set()
        for entry in self.registry.tasks():
            active_ids.add(entry.get("run_id"))
            self._recover_entry(entry)
        self._scan_unsealed_runs(active_ids)

    def _recover_entry(self, entry):
        task_id = entry.get("task_id")
        run_id = entry.get("run_id")
        run_dir = entry.get("run_dir")
        pid = entry.get("pid")
        alive = pid is not None and runtime.pid_identity_matches(
            pid, entry.get("process_start_ticks"))
        if alive:
            self.live_event("leftover_task_terminating", task_id=task_id, run_id=run_id,
                            details={"pid": pid, "pgid": entry.get("pgid"),
                                     "reason": "interrupted"})
            if run_id:
                registry_mod.write_stop_reason(self.runtime_dir, run_id, {
                    "task_id": task_id,
                    "status": "failed",
                    "failure_reason": "interrupted",
                    "reason": "interrupted",
                    "restored_at": runtime.now_iso(self.clock),
                    "pid": pid,
                    "pgid": entry.get("pgid"),
                })
            self.stopper(entry, self.clock, self.stop_grace)
            self._wait_or_recover(entry, "interrupted", status="failed")
            return
        if run_dir:
            rundir_mod.recover_interrupted_run(
                run_dir, clock=self.clock, status="failed", reason="interrupted",
                lock_path=runtime.evidence_lock_path(self.runtime_dir, run_id)
                if run_id else None)
        self.registry.release(task_id)
        self.live_event("leftover_task_recovered", task_id=task_id, run_id=run_id,
                        details={"reason": "interrupted", "pid": pid})

    def _scan_unsealed_runs(self, active_ids):
        if not os.path.isdir(self.runs_root):
            return
        try:
            names = sorted(os.listdir(self.runs_root))
        except OSError:
            return
        for name in names:
            run_dir = os.path.join(self.runs_root, name)
            manifest_path = os.path.join(run_dir, "manifest.json")
            if not os.path.isfile(manifest_path):
                continue
            if name in active_ids:
                continue
            manifest = runtime.read_json(manifest_path, default=None)
            if not isinstance(manifest, dict):
                continue
            if manifest.get("artifact_sha256"):
                continue
            if manifest.get("status") != "running":
                continue
            result = rundir_mod.recover_interrupted_run(
                run_dir, clock=self.clock, status="failed", reason="interrupted",
                lock_path=runtime.evidence_lock_path(self.runtime_dir, name))
            self.live_event("stale_run_recovered", run_id=name,
                            details={"result": result, "reason": "interrupted"})

    # -- 主循环 ------------------------------------------------------------

    def run(self, max_cycles=None):
        self.restore_dispatch_state()
        self.write_heartbeat("running")
        cycles = 0
        while not self._stop_requested:
            try:
                self.sample_once()
                self._sample_fail_count = 0
            except Exception as exc:  # noqa: BLE001 - 监控必须继续存活
                self._sample_fail_count += 1
                if self._sample_fail_count == 1 or self._sample_fail_count % 30 == 0:
                    self.live_event("sample_failed", details={
                        "error": str(exc),
                        "consecutive_failures": self._sample_fail_count,
                    })
            cycles += 1
            if max_cycles is not None and cycles >= int(max_cycles):
                break
            self._sleep_interval()
        self.write_heartbeat("stopped")
        self.live_event("monitor_stopped", details={"cycles": cycles})
        return cycles

    def _sleep_interval(self):
        remaining = self.interval
        step = 0.2
        while remaining > 0 and not self._stop_requested:
            chunk = min(step, remaining)
            self.clock.sleep(chunk)
            remaining -= chunk


def _as_float(value):
    if value is None:
        return 0.0
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def build_parser():
    parser = argparse.ArgumentParser(
        prog="resource_monitor.py",
        description="E0 资源监控（2 秒采样、90% 阈值处置、心跳与恢复）")
    parser.add_argument("--repo-root", default=None,
                        help="仓库根目录（默认由脚本位置推导）")
    parser.add_argument("--interval", type=float, default=runtime.SAMPLE_INTERVAL_S,
                        help="采样间隔秒（正式运行固定 2.0，仅诊断时调整）")
    parser.add_argument("--status-interval", type=float, default=runtime.STATUS_MAX_AGE_S,
                        help="status.json 最大写入间隔秒（默认 10.0）")
    parser.add_argument("--stop-grace", type=float, default=runtime.STOP_GRACE_S,
                        help="SIGTERM 到 SIGKILL 的宽限秒（默认 5.0）")
    parser.add_argument("--once", action="store_true", help="只采样一轮后退出（冒烟自检）")
    parser.add_argument("--max-cycles", type=int, default=None, help="采样轮数上限（测试/诊断）")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    repo_root = args.repo_root or runtime.repo_root_from(__file__)
    monitor = ResourceMonitor(
        repo_root,
        interval=args.interval,
        stop_grace=args.stop_grace,
        status_interval=args.status_interval,
    )
    monitor_lock = locking.FileLock(
        os.path.join(runtime.runtime_dir(repo_root), "monitor.lock"))
    if not monitor_lock.acquire(timeout=0.5):
        sys.stderr.write("已有资源监控在运行（monitor.lock 被占用）；退出。\n")
        return runtime.EXIT_UNAVAILABLE
    try:
        def _handle_signal(signum, frame):  # noqa: ARG001
            monitor.request_stop()

        signal.signal(signal.SIGTERM, _handle_signal)
        signal.signal(signal.SIGINT, _handle_signal)
        monitor.startup_recovery()
        max_cycles = 1 if args.once else args.max_cycles
        monitor.run(max_cycles=max_cycles)
    finally:
        monitor_lock.release()
    return runtime.EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
