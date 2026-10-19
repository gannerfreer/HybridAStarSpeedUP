"""资源监控测试：阈值边界、最新任务停止、3 次恢复、心跳、启动恢复、资源行。

全部通过构造器注入 FakeClock/假采样器/假终止器，使用临时目录中的隔离注册表；
真实监控与正式任务不受影响（正式入口不暴露注入开关）。
"""

import os
import sys
import tempfile
import unittest
from pathlib import Path

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from tools import clock as clock_mod
from tools import registry as registry_mod
from tools import resource_monitor as monitor_mod
from tools import rundir
from tools import runtime


class FakeSampler(object):
    def __init__(self):
        self.cpu = 0.0
        self.mem = 0.0
        self.rss = {}

    def sample(self, tasks):
        return {
            "cpu_percent": self.cpu,
            "mem_percent": self.mem,
            "task_rss": dict(self.rss),
        }


class MonitorTestBase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name) / "repo"
        self.rt = Path(self.tmp.name) / "planner-runtime"
        self.clk = clock_mod.FakeClock(monotonic=1000.0)
        self.sampler = FakeSampler()
        self.stopped = []

        def stopper(entry, clock, grace):
            self.stopped.append(dict(entry))
            return "sigterm"

        self.monitor = monitor_mod.ResourceMonitor(
            str(self.root),
            runtime_dir_path=str(self.rt),
            live_dir_path=str(self.root / "artifacts" / "live"),
            runs_root=str(self.root / "artifacts" / "runs"),
            clock=self.clk,
            sampler=self.sampler,
            stopper=stopper,
            interval=2.0,
            status_interval=10.0,
            seal_wait=1.0,
        )
        self.reg = self.monitor.registry

    def tearDown(self):
        self.tmp.cleanup()

    def _make_run(self, run_id, component="demo"):
        run_dir = self.root / "artifacts" / "runs" / run_id
        layout = rundir.RunLayout(run_dir).ensure_dirs()
        runtime.atomic_write_json(layout.manifest, {
            "schema_version": 1,
            "type": "planner_run_manifest",
            "run_id": run_id,
            "component": component,
            "status": "running",
            "artifact_sha256": None,
        })
        return run_dir

    def _live_events(self):
        return rundir.read_events(Path(self.monitor.live_events_path))


class ThresholdTest(MonitorTestBase):
    def test_90_0_does_not_trigger(self):
        self.sampler.cpu = 90.0
        self.sampler.mem = 90.0
        self.monitor.sample_once()
        self.assertFalse(self.monitor.paused)
        self.assertIsNone(self.reg.read_dispatch_state().get("reason"))

    def test_90_1_triggers_and_pauses_dispatch(self):
        self.sampler.cpu = 90.1
        self.monitor.sample_once()
        self.assertTrue(self.monitor.paused)
        state = self.reg.read_dispatch_state()
        self.assertTrue(state["paused"])
        self.assertIn("cpu_percent", state["reason"])
        events = [item["event"] for item in self._live_events()]
        self.assertIn("dispatch_paused", events)

    def test_memory_threshold_triggers(self):
        self.sampler.mem = 95.5
        self.monitor.sample_once()
        self.assertTrue(self.monitor.paused)
        self.assertIn("mem_percent", self.reg.read_dispatch_state()["reason"])

    def test_resume_after_three_ok_samples(self):
        self.sampler.cpu = 95.0
        self.monitor.sample_once()
        self.assertTrue(self.monitor.paused)
        self.sampler.cpu = 89.9
        self.monitor.sample_once()
        self.assertTrue(self.monitor.paused, "第 1 次低样本不恢复")
        self.sampler.cpu = 80.0
        self.monitor.sample_once()
        self.assertTrue(self.monitor.paused, "第 2 次低样本不恢复")
        self.sampler.cpu = 90.0
        self.monitor.sample_once()
        self.assertFalse(self.monitor.paused, "连续第 3 次 <= 90 恢复派发")
        events = [item["event"] for item in self._live_events()]
        self.assertIn("dispatch_resumed", events)

    def test_over_threshold_without_tasks_pauses_only(self):
        self.sampler.mem = 99.0
        self.monitor.sample_once()
        self.assertTrue(self.monitor.paused)
        self.assertEqual(self.stopped, [])
        events = [item["event"] for item in self._live_events()]
        self.assertEqual(events.count("dispatch_paused_no_task"), 1)
        self.monitor.sample_once()
        events = [item["event"] for item in self._live_events()]
        self.assertEqual(events.count("dispatch_paused_no_task"), 1,
                         "无任务可停的报警不重复刷屏")


class StopNewestTaskTest(MonitorTestBase):
    def test_stops_newest_and_recovers_run(self):
        old_dir = self._make_run("r-old")
        new_dir = self._make_run("r-new")
        old = self.reg.reserve("test", component="a", workers=1, run_id="r-old",
                               run_dir=str(old_dir), task_id="r-old")
        new = self.reg.reserve("test", component="b", workers=1, run_id="r-new",
                               run_dir=str(new_dir), task_id="r-new")
        try:
            self.reg.update_process("r-old", 111111, 111111, 1)
            self.reg.update_process("r-new", 222222, 222222, 1)
            self.sampler.cpu = 96.0
            self.monitor.sample_once()
            self.assertEqual(len(self.stopped), 1)
            self.assertEqual(self.stopped[0]["task_id"], "r-new")
            self.assertFalse(self.reg.has_task("r-new"))
            self.assertTrue(self.reg.has_task("r-old"))
            manifest = runtime.read_json(new_dir / "manifest.json")
            self.assertEqual(manifest["status"], "resource_stopped")
            self.assertIsNotNone(manifest["artifact_sha256"])
            stop_events = [item for item in self._live_events()
                           if item["event"] == "resource_stopped"]
            self.assertEqual(len(stop_events), 1)
            self.assertEqual(stop_events[0]["run_id"], "r-new")
            reason = runtime.read_json(
                registry_mod.stop_reason_path(str(self.rt), "r-new"))
            self.assertEqual(reason["reason"], "resource_stopped")
        finally:
            if not old.released:
                old.release()
            if not new.released:
                new.release()

    def test_stops_remaining_on_next_sample(self):
        first_dir = self._make_run("r-1")
        second_dir = self._make_run("r-2")
        first = self.reg.reserve("test", component="a", workers=1, run_id="r-1",
                                 run_dir=str(first_dir), task_id="r-1")
        second = self.reg.reserve("test", component="b", workers=1, run_id="r-2",
                                  run_dir=str(second_dir), task_id="r-2")
        try:
            self.reg.update_process("r-1", 111, 111, 1)
            self.reg.update_process("r-2", 222, 222, 1)
            self.sampler.mem = 91.0
            self.monitor.sample_once()
            self.assertEqual(self.stopped[0]["task_id"], "r-2")
            self.monitor.sample_once()
            self.assertEqual(self.stopped[1]["task_id"], "r-1")
            self.assertEqual(self.reg.tasks(), [])
        finally:
            if not first.released:
                first.release()
            if not second.released:
                second.release()


class HeartbeatTest(MonitorTestBase):
    def test_heartbeat_freshness_boundary(self):
        self.monitor.write_heartbeat()
        self.assertIsNone(registry_mod.heartbeat_state(str(self.rt), self.clk))
        self.clk.advance(6.0)
        self.assertIsNone(registry_mod.heartbeat_state(str(self.rt), self.clk))
        self.clk.advance(0.001)
        self.assertEqual(registry_mod.heartbeat_state(str(self.rt), self.clk),
                         "monitor_lost")

    def test_heartbeat_stopped_after_run_exit(self):
        self.monitor.run(max_cycles=1)
        state = registry_mod.heartbeat_state(str(self.rt), self.clk)
        self.assertEqual(state, "monitor_stopped")
        payload = registry_mod.read_heartbeat(str(self.rt))
        self.assertEqual(payload["status"], "stopped")


class StartupRecoveryTest(MonitorTestBase):
    def test_dead_and_unstarted_leftovers_marked_interrupted(self):
        dead_dir = self._make_run("r-dead")
        none_dir = self._make_run("r-none")
        self.reg.reserve("test", component="a", workers=1, run_id="r-dead",
                         run_dir=str(dead_dir), task_id="r-dead")
        self.reg.reserve("test", component="b", workers=1, run_id="r-none",
                         run_dir=str(none_dir), task_id="r-none")
        self.reg.update_process("r-dead", 987654321, 987654321, 1)
        self.monitor.startup_recovery()
        self.assertEqual(self.reg.tasks(), [])
        self.assertEqual(self.stopped, [], "已死进程不应再收到信号")
        for run_dir in (dead_dir, none_dir):
            manifest = runtime.read_json(run_dir / "manifest.json")
            self.assertEqual(manifest["status"], "failed")
            self.assertEqual(manifest["failure_reason"], "interrupted")
            self.assertIsNotNone(manifest["artifact_sha256"])

    def test_alive_leftover_terminated_and_marked_interrupted(self):
        run_dir = self._make_run("r-alive")
        reservation = self.reg.reserve(
            "test", component="a", workers=1, run_id="r-alive",
            run_dir=str(run_dir), task_id="r-alive")
        try:
            self.reg.update_process("r-alive", os.getpid(), 999999998,
                                    runtime.process_start_ticks(os.getpid()))
            self.monitor.startup_recovery()
            self.assertEqual(len(self.stopped), 1)
            self.assertEqual(self.stopped[0]["task_id"], "r-alive")
            self.assertFalse(self.reg.has_task("r-alive"))
            manifest = runtime.read_json(run_dir / "manifest.json")
            self.assertEqual(manifest["status"], "failed")
            self.assertEqual(manifest["failure_reason"], "interrupted")
        finally:
            reservation.release()

    def test_stale_unsealed_run_scanned(self):
        stale_dir = self._make_run("r-stale")
        self.monitor.startup_recovery()
        manifest = runtime.read_json(stale_dir / "manifest.json")
        self.assertEqual(manifest["status"], "failed")
        self.assertEqual(manifest["failure_reason"], "interrupted")
        events = [item["event"] for item in self._live_events()]
        self.assertIn("stale_run_recovered", events)


class ResourceRowsTest(MonitorTestBase):
    def test_resources_csv_written_for_registered_run(self):
        run_dir = self._make_run("r-csv")
        reservation = self.reg.reserve("test", component="a", workers=1,
                                       run_id="r-csv", run_dir=str(run_dir),
                                       task_id="r-csv")
        try:
            self.reg.update_process("r-csv", 1234, 4321, 1)
            self.sampler.cpu = 10.0
            self.sampler.mem = 20.0
            self.sampler.rss = {"r-csv": 123456}
            self.monitor.sample_once()
            rows = rundir.read_csv_rows(run_dir / "resources.csv")
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["cpu_percent"], "10.000")
            self.assertEqual(rows[0]["mem_percent"], "20.000")
            self.assertEqual(rows[0]["task_tree_rss_bytes"], "123456")
            self.assertEqual(rows[0]["active_tasks"], "1")
        finally:
            reservation.release()

    def test_status_json_and_heartbeat_written(self):
        self.monitor.sample_once()
        status = runtime.read_json(self.monitor.status_path)
        self.assertEqual(status["type"], "resource_status")
        self.assertIn("dispatch", status)
        self.assertIn("thresholds", status["system"])
        heartbeat = registry_mod.read_heartbeat(str(self.rt))
        self.assertEqual(heartbeat["status"], "running")

    def test_isolated_paths_do_not_touch_real_runtime(self):
        self.assertTrue(str(self.monitor.runtime_dir).startswith(self.tmp.name))
        self.assertTrue(str(self.monitor.live_dir).startswith(self.tmp.name))
        self.assertTrue(str(self.monitor.runs_root).startswith(self.tmp.name))

    def test_default_constructor_paths_are_json_serializable(self):
        """回归：CLI 默认路径分支的 PosixPath 曾导致 startup_recovery 崩溃。"""
        monitor = monitor_mod.ResourceMonitor(str(self.root))
        self.assertIsInstance(monitor.runtime_dir, str)
        self.assertIsInstance(monitor.live_dir, str)
        self.assertIsInstance(monitor.runs_root, str)
        monitor.startup_recovery()
        events = rundir.read_events(Path(monitor.live_events_path))
        startup = [item for item in events if item["event"] == "monitor_startup"]
        self.assertEqual(len(startup), 1)
        self.assertIsInstance(startup[0]["details"]["runtime_dir"], str)
        self.assertIsInstance(startup[0]["details"]["repo_root"], str)


if __name__ == "__main__":
    unittest.main()
