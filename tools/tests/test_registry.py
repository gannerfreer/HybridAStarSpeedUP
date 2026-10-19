"""共享登记与名额测试：原子分配/释放、排队、暂停、独占窗口、心跳、异常清理。

全部使用临时目录中的隔离注册表；不触碰真实 planner-runtime/。
"""

import os
import select
import subprocess
import sys
import tempfile
import threading
import time
import unittest

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from tools import clock as clock_mod
from tools import registry as registry_mod
from tools import runtime


class RegistryTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.rt = os.path.join(self.tmp.name, "planner-runtime")
        self.reg = registry_mod.Registry(self.rt, worker_cap=4)

    def tearDown(self):
        self.tmp.cleanup()

    def test_reserve_release_count(self):
        with self.reg.reserve("test", component="a", workers=2, task_id="t1") as res:
            self.assertEqual(res.entry["workers"], 2)
            self.assertEqual(self.reg.count_workers(), 2)
            self.assertTrue(self.reg.has_task("t1"))
        self.assertEqual(self.reg.count_workers(), 0)
        self.assertFalse(self.reg.has_task("t1"))

    def test_worker_cap_queues_then_succeeds(self):
        holder = self.reg.reserve("test", component="a", workers=3, task_id="t1")
        try:
            released = {"done": False}

            def release_later():
                time.sleep(0.3)
                holder.release()
                released["done"] = True

            thread = threading.Thread(target=release_later, daemon=True)
            thread.start()
            reservation = self.reg.reserve("test", component="b", workers=2,
                                           task_id="t2", queue_timeout=5.0,
                                           poll_interval=0.05)
            self.assertEqual(reservation.entry["workers"], 2)
            self.assertTrue(released["done"])
            reservation.release()
        finally:
            if not holder.released:
                holder.release()

    def test_worker_cap_queue_timeout(self):
        holder = self.reg.reserve("test", component="a", workers=4, task_id="t1")
        try:
            with self.assertRaises(registry_mod.QueueTimeout):
                self.reg.reserve("test", component="b", workers=1, task_id="t2",
                                 queue_timeout=0.3, poll_interval=0.05)
        finally:
            holder.release()

    def test_paused_dispatch_rejected(self):
        registry_mod.write_dispatch_state(self.rt, {
            "paused": True, "reason": "resource_threshold:cpu_percent=99>90"})
        with self.assertRaises(registry_mod.DispatchPaused):
            self.reg.reserve("test", component="a", workers=1, task_id="t1")
        self.assertEqual(self.reg.read_dispatch_state()["paused"], True)

    def test_exception_cleanup_releases_slot(self):
        with self.assertRaises(ValueError):
            with self.reg.reserve("test", component="a", workers=1, task_id="t1"):
                raise ValueError("boom")
        self.assertEqual(self.reg.count_workers(), 0)
        self.assertEqual(self.reg.tasks(), [])

    def test_benchmark_exclusive_window(self):
        holder = self.reg.reserve("test", component="a", workers=1, task_id="t1")
        try:
            with self.assertRaises(registry_mod.QueueTimeout):
                self.reg.reserve("benchmark", component="b", task_id="t2",
                                 queue_timeout=0.2, poll_interval=0.05)
        finally:
            holder.release()
        bench = self.reg.reserve("benchmark", component="b", task_id="t2",
                                 queue_timeout=1.0)
        try:
            with self.assertRaises(registry_mod.QueueTimeout):
                self.reg.reserve("test", component="c", workers=1, task_id="t3",
                                 queue_timeout=0.2, poll_interval=0.05)
        finally:
            bench.release()

    def test_duplicate_task_id_rejected(self):
        holder = self.reg.reserve("test", component="a", workers=1, task_id="dup")
        try:
            with self.assertRaises(registry_mod.RegistryError):
                self.reg.reserve("test", component="b", workers=1, task_id="dup",
                                 queue_timeout=0.2)
        finally:
            holder.release()

    def test_exec_kind_counts_workers(self):
        res = self.reg.reserve("exec", component="adhoc", workers=2, task_id="x1")
        try:
            self.assertEqual(self.reg.count_workers(), 2)
            self.assertTrue(self.reg.has_task("x1"))
        finally:
            res.release()
        self.assertEqual(self.reg.count_workers(), 0)

    def test_build_with_workers_counts_toward_cap(self):
        res = self.reg.reserve("build", component="web", workers=1, task_id="b1")
        try:
            self.assertEqual(self.reg.count_workers(), 1)
            with self.assertRaises(registry_mod.QueueTimeout):
                self.reg.reserve("test", component="x", workers=4, task_id="t1",
                                 queue_timeout=0.2, poll_interval=0.05)
        finally:
            res.release()
        self.assertEqual(self.reg.count_workers(), 0)

    def test_zero_worker_build_does_not_consume_cap(self):
        res = self.reg.reserve("build", component="cmake-comp", workers=0,
                               task_id="b2")
        try:
            self.assertEqual(self.reg.count_workers(), 0)
        finally:
            res.release()

    def test_update_process_roundtrip(self):
        res = self.reg.reserve("test", component="a", workers=1, task_id="t1")
        try:
            self.assertTrue(self.reg.update_process("t1", 1234, 1234, 999))
            entry = self.reg.tasks()[0]
            self.assertEqual(entry["pid"], 1234)
            self.assertEqual(entry["pgid"], 1234)
            self.assertEqual(entry["process_start_ticks"], 999)
            self.assertFalse(self.reg.update_process("missing", 1, 1, 1))
        finally:
            res.release()

    def test_newest_task_selection(self):
        tasks = [
            {"task_id": "a", "started_monotonic": 1.0, "seq": 5},
            {"task_id": "b", "started_monotonic": 3.0, "seq": 1},
            {"task_id": "c", "started_monotonic": 3.0, "seq": 2},
            {"task_id": "bad", "started_monotonic": None, "seq": None},
        ]
        self.assertEqual(registry_mod.newest_task(tasks)["task_id"], "c")
        self.assertIsNone(registry_mod.newest_task([]))

    def test_compile_lock_mutual_exclusion(self):
        first = self.reg.compile_lock()
        self.assertTrue(first.acquire(timeout=1.0))
        second = self.reg.compile_lock()
        try:
            self.assertFalse(second.acquire(timeout=0.2))
        finally:
            first.release()
        self.assertTrue(second.acquire(timeout=1.0))
        second.release()

    def test_compile_lock_multiprocess_exclusion(self):
        script = (
            "import sys, time\n"
            "sys.path.insert(0, sys.argv[2])\n"
            "from tools.locking import FileLock\n"
            "lock = FileLock(sys.argv[1])\n"
            "if not lock.acquire(timeout=5.0):\n"
            "    sys.exit(2)\n"
            "print('locked', flush=True)\n"
            "time.sleep(30)\n"
        )
        proc = subprocess.Popen(
            [sys.executable, "-c", script, str(self.reg.compile_lock_path), _REPO_ROOT],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        try:
            ready, _, _ = select.select([proc.stdout], [], [], 10)
            self.assertTrue(ready, "子进程未在 10 秒内获取编译锁")
            self.assertEqual(proc.stdout.readline().strip(), "locked")
            contender = self.reg.compile_lock()
            self.assertFalse(contender.acquire(timeout=0.3),
                             "编译锁应跨进程互斥")
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)
        winner = self.reg.compile_lock()
        self.assertTrue(winner.acquire(timeout=2.0))
        winner.release()

    def test_heartbeat_states(self):
        clk = clock_mod.FakeClock(monotonic=1000.0)
        self.assertEqual(registry_mod.heartbeat_state(self.rt, clk), "monitor_missing")
        runtime.atomic_write_json(registry_mod.heartbeat_path(self.rt), {
            "status": "running", "updated_monotonic": 1000.0})
        self.assertIsNone(registry_mod.heartbeat_state(self.rt, clk))
        clk.advance(6.0)
        self.assertIsNone(registry_mod.heartbeat_state(self.rt, clk),
                          "恰好 6 秒仍视为新鲜")
        clk.advance(0.001)
        self.assertEqual(registry_mod.heartbeat_state(self.rt, clk), "monitor_lost")
        runtime.atomic_write_json(registry_mod.heartbeat_path(self.rt), {
            "status": "stopped", "updated_monotonic": clk.monotonic()})
        self.assertEqual(registry_mod.heartbeat_state(self.rt, clk), "monitor_stopped")

    def test_stop_reason_roundtrip(self):
        registry_mod.write_stop_reason(self.rt, "run-1", {
            "status": "resource_stopped", "failure_reason": "cpu"})
        payload = registry_mod.read_stop_reason(self.rt, "run-1")
        self.assertEqual(payload["status"], "resource_stopped")
        self.assertEqual(payload["run_id"], "run-1")
        registry_mod.clear_stop_reason(self.rt, "run-1")
        self.assertIsNone(registry_mod.read_stop_reason(self.rt, "run-1"))

    def test_gate_failure_while_waiting(self):
        holder = self.reg.reserve("test", component="a", workers=4, task_id="t1")
        try:
            with self.assertRaises(registry_mod.GateFailure):
                self.reg.reserve("test", component="b", workers=1, task_id="t2",
                                 queue_timeout=5.0, poll_interval=0.01,
                                 gate=lambda: "monitor_lost")
        finally:
            holder.release()


if __name__ == "__main__":
    unittest.main()
