"""运行目录测试：唯一创建、封存、清单校验（篡改/缺失/额外文件）、中断恢复、CSV。"""

import os
import sys
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path
from xml.etree import ElementTree

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from tools import clock as clock_mod
from tools import rundir
from tools import runtime


class RunDirTestBase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.runs = self.root / "artifacts" / "runs"
        self.clk = clock_mod.FakeClock(utc=datetime(2026, 1, 2, 3, 4, 5, 123456,
                                                    tzinfo=timezone.utc))

    def tearDown(self):
        self.tmp.cleanup()

    def _make_run(self, run_id="20260102T030405123456Z_demo_deadbeef"):
        run_dir = self.runs / run_id
        layout = rundir.RunLayout(run_dir).ensure_dirs()
        manifest = {
            "schema_version": 1,
            "type": "planner_run_manifest",
            "run_id": run_id,
            "task_id": "test:demo",
            "component": "demo",
            "status": "running",
            "exit_code": None,
            "artifact_sha256": None,
        }
        manifest = rundir.init_run(layout, manifest, {"schema_version": 1},
                                   clock=self.clk)
        return run_dir, layout, manifest


class RunIdTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.runs = Path(self.tmp.name) / "runs"

    def tearDown(self):
        self.tmp.cleanup()

    def test_run_id_format(self):
        clk = clock_mod.FakeClock(utc=datetime(2026, 1, 2, 3, 4, 5, 123456,
                                               tzinfo=timezone.utc))
        run_id = rundir.new_run_id("rs", clock=clk, random_hex="deadbeef")
        self.assertEqual(run_id, "20260102T030405123456Z_rs_deadbeef")
        parsed = rundir.parse_run_id(run_id)
        self.assertEqual(parsed["component"], "rs")
        self.assertEqual(parsed["random"], "deadbeef")

    def test_create_run_dir_exclusive(self):
        rid, path = rundir.create_run_dir(self.runs, "demo",
                                          clock=clock_mod.FakeClock(),
                                          run_id="fixed-run-id")
        self.assertTrue(path.is_dir())
        self.assertEqual(rid, "fixed-run-id")
        with self.assertRaises(rundir.RunDirError):
            rundir.create_run_dir(path.parent, "demo", run_id="fixed-run-id")


class SealVerifyTest(RunDirTestBase):
    def _add_evidence(self, layout):
        (layout.path / "a.txt").write_text("alpha", encoding="utf-8")
        (layout.path / "sub").mkdir()
        (layout.path / "sub" / "b.bin").write_bytes(b"beta")
        (layout.path / "outputs").mkdir(exist_ok=True)
        (layout.path / "outputs" / "o.json").write_text("{}", encoding="utf-8")

    def test_seal_and_verify_roundtrip(self):
        run_dir, layout, _ = self._make_run()
        self._add_evidence(layout)
        lock_path = self.root / "locks" / "evidence.lock"
        artifact_sha = rundir.seal_run(run_dir, {"status": "success", "exit_code": 0},
                                       clock=self.clk, lock_path=lock_path)
        result = rundir.verify_run_dir(run_dir)
        self.assertTrue(result["ok"], result["problems"])
        self.assertEqual(result["artifact_sha256"], artifact_sha)
        manifest = runtime.read_json(layout.manifest)
        self.assertEqual(manifest["artifact_sha256"], artifact_sha)
        self.assertEqual(manifest["status"], "success")
        self.assertIsNotNone(manifest["finished_at"])

        items = runtime.read_json(layout.artifacts_manifest, strict=True)
        self.assertIsInstance(items, list)
        paths = [item["path"] for item in items]
        self.assertEqual(paths, sorted(paths))
        self.assertIn("a.txt", paths)
        self.assertIn("sub/b.bin", paths)
        self.assertIn("summary.md", paths)
        self.assertNotIn("manifest.json", paths)
        self.assertNotIn("artifacts.sha256.json", paths)
        for item in items:
            full = run_dir / item["path"]
            self.assertEqual(item["size_bytes"], full.stat().st_size)
            self.assertEqual(item["sha256"], runtime.sha256_file(full))
        self.assertEqual(artifact_sha, runtime.sha256_file(layout.artifacts_manifest))

    def test_verify_rejects_tampered_file(self):
        run_dir, layout, _ = self._make_run()
        self._add_evidence(layout)
        rundir.seal_run(run_dir, {"status": "success", "exit_code": 0}, clock=self.clk)
        with open(run_dir / "a.txt", "a", encoding="utf-8") as handle:
            handle.write("tampered")
        result = rundir.verify_run_dir(run_dir)
        self.assertFalse(result["ok"])
        joined = "\n".join(result["problems"])
        self.assertIn("a.txt", joined)

    def test_verify_rejects_missing_file(self):
        run_dir, layout, _ = self._make_run()
        self._add_evidence(layout)
        rundir.seal_run(run_dir, {"status": "success", "exit_code": 0}, clock=self.clk)
        os.unlink(str(run_dir / "a.txt"))
        result = rundir.verify_run_dir(run_dir)
        self.assertFalse(result["ok"])
        self.assertIn("清单文件缺失: a.txt", result["problems"])

    def test_verify_rejects_extra_file(self):
        run_dir, layout, _ = self._make_run()
        self._add_evidence(layout)
        rundir.seal_run(run_dir, {"status": "success", "exit_code": 0}, clock=self.clk)
        (run_dir / "extra.txt").write_text("extra", encoding="utf-8")
        result = rundir.verify_run_dir(run_dir)
        self.assertFalse(result["ok"])
        self.assertIn("存在清单外额外文件: extra.txt", result["problems"])

    def test_verify_rejects_manifest_hash_mismatch(self):
        run_dir, layout, _ = self._make_run()
        self._add_evidence(layout)
        rundir.seal_run(run_dir, {"status": "success", "exit_code": 0}, clock=self.clk)
        manifest = runtime.read_json(layout.manifest)
        manifest["artifact_sha256"] = "0" * 64
        runtime.atomic_write_json(layout.manifest, manifest)
        result = rundir.verify_run_dir(run_dir)
        self.assertFalse(result["ok"])
        joined = "\n".join(result["problems"])
        self.assertIn("artifact_sha256", joined)

    def test_temp_files_excluded_from_manifest(self):
        run_dir, layout, _ = self._make_run()
        self._add_evidence(layout)
        (run_dir / "leftover.tmp").write_text("partial", encoding="utf-8")
        rundir.seal_run(run_dir, {"status": "success", "exit_code": 0}, clock=self.clk)
        items = runtime.read_json(layout.artifacts_manifest, strict=True)
        paths = [item["path"] for item in items]
        self.assertNotIn("leftover.tmp", paths)
        self.assertTrue(rundir.verify_run_dir(run_dir)["ok"])


class RecoveryTest(RunDirTestBase):
    def test_recover_interrupted_run(self):
        run_dir, layout, _ = self._make_run()
        (layout.path / "partial.txt").write_text("partial", encoding="utf-8")
        result = rundir.recover_interrupted_run(run_dir, clock=self.clk)
        self.assertEqual(result, "sealed")
        manifest = runtime.read_json(layout.manifest)
        self.assertEqual(manifest["status"], "failed")
        self.assertEqual(manifest["failure_reason"], "interrupted")
        self.assertIsNotNone(manifest["artifact_sha256"])
        self.assertTrue(rundir.verify_run_dir(run_dir)["ok"])
        self.assertEqual(rundir.recover_interrupted_run(run_dir, clock=self.clk),
                         "already_sealed")

    def test_recover_missing_manifest(self):
        run_dir = self.runs / "no-manifest"
        run_dir.mkdir(parents=True)
        self.assertEqual(rundir.recover_interrupted_run(run_dir, clock=self.clk),
                         "missing_manifest")


class CsvAndJunitTest(RunDirTestBase):
    def test_header_only_and_rows(self):
        run_dir, layout, _ = self._make_run()
        path = layout.metrics
        text = path.read_text(encoding="utf-8")
        self.assertTrue(text.startswith("run_id,case_id"))
        self.assertEqual(len(text.strip().splitlines()), 1, "初始 CSV 只有表头")
        rundir.append_metric_row(layout, {"run_id": "r1", "case_id": "c1",
                                          "status": "success", "wall_ms": "1.5"})
        rows = rundir.read_csv_rows(path)
        self.assertEqual(len(rows), 1)
        self.assertEqual(rows[0]["case_id"], "c1")
        self.assertEqual(rows[0]["failure_reason"], "", "缺失列留空而不是 0")

    def test_parse_junit_counts(self):
        run_dir, layout, _ = self._make_run()
        xml = (
            '<?xml version="1.0" encoding="UTF-8"?>\n'
            '<testsuites name="t" tests="3" failures="1" errors="0" skipped="1" time="1.5">'
            '<testsuite name="t" tests="3" failures="1" errors="0" skipped="1" time="1.5"/>'
            '</testsuites>\n'
        )
        layout.junit.write_text(xml, encoding="utf-8")
        counts = rundir.parse_junit_counts(layout.junit)
        self.assertEqual(counts["tests"], 3)
        self.assertEqual(counts["failures"], 1)
        self.assertEqual(counts["skipped"], 1)
        ElementTree.parse(str(layout.junit))
        self.assertIsNone(rundir.parse_junit_counts(run_dir / "missing.xml"))

    def test_events_are_jsonl(self):
        run_dir, layout, _ = self._make_run()
        rundir.record_event(layout, self.clk, "started", task_id="t1",
                            component="demo", details={"a": 1})
        events = rundir.read_events(layout)
        self.assertEqual(events[0]["event"], "started")
        self.assertEqual(events[0]["details"], {"a": 1})


if __name__ == "__main__":
    unittest.main()
