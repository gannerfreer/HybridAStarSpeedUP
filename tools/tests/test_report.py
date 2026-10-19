"""report_results 测试：样例标注、成功/失败/中断分列、状态数据、篡改检测。"""

import os
import sys
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from tools import clock as clock_mod
from tools import report_results
from tools import rundir
from tools import runtime

RUN_ID = "20260102T030405123456Z_demo_deadbeef"


class ReportTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.clk = clock_mod.FakeClock(
            utc=datetime(2026, 1, 2, 3, 4, 5, 123456, tzinfo=timezone.utc))

    def tearDown(self):
        self.tmp.cleanup()

    def _make_sealed_run(self, run_id=RUN_ID):
        run_dir = self.root / "artifacts" / "runs" / run_id
        layout = rundir.RunLayout(run_dir).ensure_dirs()
        manifest = {
            "schema_version": 1,
            "type": "planner_run_manifest",
            "run_id": run_id,
            "task_id": "test:demo",
            "component": "demo",
            "backend": "none",
            "task_kind": "test",
            "command_argv": [["ctest", "-L", "demo"]],
            "cwd": str(self.root),
            "started_at": "2026-01-02T03:04:05.123Z",
            "finished_at": None,
            "status": "running",
            "exit_code": None,
            "git_commit": "deadbeef",
            "dirty": True,
            "dependency_versions": {"gtest": {"version": "1.15.2", "status": "verified"}},
            "hardware": {"machine": "x86_64", "platform": "Linux", "python": "3.11"},
            "build_profile": "release",
            "config_sha256": None,
            "input_sha256": {"tests/fixtures/v1": "abc"},
            "seed": 0,
            "worker_count": 1,
            "artifact_sha256": None,
            "retry_of": None,
            "failure_reason": None,
        }
        manifest = rundir.init_run(layout, manifest, {"schema_version": 1},
                                   clock=self.clk)
        cases = [
            ("case_ok", "success"),
            ("case_fail", "failed"),
            ("case_stop", "resource_stopped"),
            ("case_interrupt", "interrupted"),
        ]
        for case_id, status in cases:
            rundir.append_metric_row(layout, {
                "run_id": run_id,
                "case_id": case_id,
                "repeat_id": "0",
                "cache_mode": "cold",
                "seed": "0",
                "status": status,
                "failure_reason": "" if status == "success" else status,
                "wall_ms": "10.5",
                "cpu_ms": "5.0",
                "peak_rss_bytes": "1024",
                "valid": "true" if status == "success" else "false",
            })
        rundir.append_resource_row(layout, {
            "time_utc": "2026-01-02T03:04:06.123Z",
            "monotonic_elapsed_s": "1.000",
            "cpu_percent": "12.500",
            "mem_percent": "30.000",
            "task_tree_rss_bytes": "2048",
            "active_tasks": "1",
            "build_task_id": "",
        })
        layout.junit.write_text(
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            '<testsuites name="t" tests="2" failures="1" errors="0" skipped="0" time="0.5">'
            '<testsuite name="t" tests="2" failures="1" errors="0" skipped="0" time="0.5"/>'
            '</testsuites>\n',
            encoding="utf-8")
        rundir.record_event(layout, self.clk, "resource_stopped",
                            task_id="test:demo", component="demo",
                            details={"metric": "cpu_percent", "value": 95.0})
        rundir.write_task_summary(layout, manifest)
        rundir.seal_run(run_dir, {
            "status": "resource_stopped",
            "exit_code": 143,
            "failure_reason": "resource_stopped:cpu_percent=95.0>90.0",
        }, clock=self.clk)
        return run_dir

    def test_sample_report_marks_sample_and_counts(self):
        run_dir = self._make_sealed_run()
        out = self.root / "reports" / "sample"
        result = report_results.generate(run_dir, output_dir=out, sample=True)
        self.assertTrue(result["verification"]["ok"], result["verification"]["problems"])
        md = Path(result["report"]).read_text(encoding="utf-8")
        self.assertIn(report_results.SAMPLE_BANNER, md)
        csv_text = Path(result["csv"]).read_text(encoding="utf-8")
        self.assertIn("sample_marker", csv_text)
        self.assertIn(report_results.SAMPLE_BANNER, csv_text)
        status = runtime.read_json(result["status"], strict=True)
        self.assertTrue(status["sample"])
        counts = status["counts"]
        self.assertEqual(counts["success"], 1)
        self.assertEqual(counts["failed"], 1)
        self.assertEqual(counts["resource_stopped"], 1)
        self.assertEqual(counts["interrupted"], 1)
        self.assertEqual(counts["total"], 4)

    def test_non_sample_report_has_no_banner(self):
        run_dir = self._make_sealed_run()
        out = self.root / "reports" / "normal"
        result = report_results.generate(run_dir, output_dir=out, sample=False)
        md = Path(result["report"]).read_text(encoding="utf-8")
        self.assertNotIn(report_results.SAMPLE_BANNER, md)
        status = runtime.read_json(result["status"], strict=True)
        self.assertFalse(status["sample"])

    def test_main_detects_tampering(self):
        run_dir = self._make_sealed_run()
        good_out = self.root / "reports" / "good"
        self.assertEqual(report_results.main([
            "--run-dir", str(run_dir), "--output-dir", str(good_out)]), 0)
        with open(run_dir / "metrics.csv", "a", encoding="utf-8") as handle:
            handle.write("tampered,row\n")
        bad_out = self.root / "reports" / "bad"
        code = report_results.main([
            "--run-dir", str(run_dir), "--output-dir", str(bad_out)])
        self.assertEqual(code, runtime.EXIT_DATA)
        status = runtime.read_json(bad_out / "status.json", strict=True)
        self.assertFalse(status["verification"]["ok"])
        self.assertTrue(status["verification"]["problems"])

    def test_missing_run_dir_is_data_error(self):
        code = report_results.main([
            "--run-dir", str(self.root / "nope"),
            "--output-dir", str(self.root / "reports" / "x")])
        self.assertEqual(code, runtime.EXIT_DATA)

    def test_sample_report_script_flow(self):
        from tools import sample_report
        code = sample_report.main(["--repo-root", str(self.root)])
        self.assertEqual(code, 0)
        run_dir = self.root / "artifacts" / "samples" / "report_sample"
        self.assertTrue((run_dir / "manifest.json").is_file())
        manifest = runtime.read_json(run_dir / "manifest.json")
        self.assertTrue(manifest.get("sample"))
        self.assertIsNotNone(manifest.get("artifact_sha256"))
        verification = rundir.verify_run_dir(run_dir)
        self.assertTrue(verification["ok"], verification["problems"])
        report_md = (self.root / "artifacts" / "reports" / "report_sample"
                     / ("%s.md" % sample_report.SAMPLE_RUN_ID))
        self.assertTrue(report_md.is_file())
        self.assertIn(report_results.SAMPLE_BANNER,
                      report_md.read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()
