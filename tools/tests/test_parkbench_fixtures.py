"""parkbench_fixtures（v9 ParkBench 夹具生成器）测试：解析/清洗/画布/预检并入/
检查模式/输入拒绝。仅标准库；用合成 zip 与临时仓库树。"""

import json
import os
import sys
import tempfile
import types
import unittest
import zipfile
from pathlib import Path

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from tools import parkbench_fixtures as pf


def frame(start, target, segments):
    return {
        "Frames": {
            "0": {
                "PlanningRequest": {
                    "m_startPosture": {"m_pose": list(start)},
                    "m_targetArea": {"m_targetPosture": {"m_pose": list(target)}},
                },
                "NfmAggregatedPolygonObjects": [
                    {
                        "nfmPolygonObjectNodes": [
                            {"m_x": a[0], "m_y": a[1]} for a in segment
                        ]
                    }
                    for segment in segments
                ],
            }
        }
    }


class ParkbenchFixturesTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        # 合成 v8 源夹具树。
        source = self.root / "tests" / "fixtures" / "v8"
        (source / "paths").mkdir(parents=True)
        (source / "vehicles").mkdir(parents=True)
        (source / "paths" / "cases.json").write_text('{"cases": []}\n', encoding="utf-8")
        (source / "vehicles" / "default.json").write_text('{"id": "d"}\n', encoding="utf-8")
        (source / "index.json").write_text(json.dumps({
            "schema_version": 1, "type": "fixture_index", "fixture_version": "v8",
            "revision_of": "v7", "revision_summary": "t", "revision_history": [],
            "sets": [{"id": "paths", "description": "paths"},
                     {"id": "vehicles", "description": "vehicles"}],
            "notes": [],
        }), encoding="utf-8")
        (source / "README.md").write_text("v8 readme\n", encoding="utf-8")

        # 合成 ParkBench zip：两个场景（第二个用 m_targetAreas 变体）。
        # 起点 (0,0,0)，目标 (20,0,0)；目标框 x∈[-1.075,3.975]（+0.05 余量后）。
        # a.json 的线段：远距离群 / 完整穿越目标框 / 一端在目标框内 / 正常线段。
        target = (20.0, 0.0, 0.0)
        segments = [
            [(5000.0, 0.0), (5000.0, 10.0)],       # 离群：丢弃
            [(19.0, -3.0), (19.0, 3.0)],           # 纵向穿越目标框：2 片段
            [(19.5, 0.5), (25.0, 0.5)],            # 一端在框内：1 片段
            [(30.0, -5.0), (35.0, -5.0)],          # 正常：1 个四边形
        ]
        document_a = frame((0.0, 0.0, 0.0), target, segments)
        document_b = frame((1.0, -6.0, 0.3), (24.0, 4.0, -1.0), [
            [(20.0, 8.0), (28.0, 8.0)],
        ])
        del document_b["Frames"]["0"]["PlanningRequest"]["m_targetArea"]
        document_b["Frames"]["0"]["PlanningRequest"]["m_targetAreas"] = {
            "m_targetPosture": [{"m_pose": [24.0, 4.0, -1.0]}]
        }
        self.zip_path = self.root / "source.zip"
        with zipfile.ZipFile(self.zip_path, "w") as archive:
            archive.writestr(pf.ZIP_INNER_ROOT + "a.json",
                             json.dumps(document_a))
            archive.writestr(pf.ZIP_INNER_ROOT + "b.json",
                             json.dumps(document_b))
        self.output = self.root / "tests" / "fixtures" / "v9"

    def tearDown(self):
        self.tmp.cleanup()

    def generate(self, expected_cases=2, precheck_json=None):
        original = pf.EXPECTED_CASES
        pf.EXPECTED_CASES = expected_cases
        try:
            args = types.SimpleNamespace(
                repo_root=str(self.root),
                source_zip=str(self.zip_path),
                expected_sha256="",
                source_version="v8",
                output_dir=str(self.output),
                force=True,
                precheck_json=precheck_json,
            )
            return pf.generate(args)
        finally:
            pf.EXPECTED_CASES = original

    def load_cases(self):
        return json.loads(
            (self.output / "paths" / "cases_bench_parkbench.json").read_text(
                encoding="utf-8"))

    def test_generate_cleans_and_builds(self):
        self.assertEqual(self.generate(), 0)
        document = self.load_cases()
        self.assertEqual(len(document["cases"]), 2)
        self.assertEqual(document["cases"][0]["id"], "pb-001")
        self.assertEqual(document["cases"][1]["id"], "pb-002")
        self.assertEqual(document["cases"][1]["goal"]["x_m"], 24.0)
        per_case = document["self_check"]["cleaning_audit"]["per_case"]
        first = per_case[0]
        self.assertEqual(first["segments"], 4)
        self.assertEqual(first["outliers"], 1)
        self.assertEqual(first["target_removed"], 0)
        self.assertEqual(first["target_partial"], 2)
        self.assertEqual(first["start_removed"], 0)
        self.assertEqual(first["start_partial"], 0)
        self.assertEqual(first["quads"], 4)
        vehicle = json.loads(
            (self.output / "vehicles" / "parkbench_vehicle.json").read_text(
                encoding="utf-8"))
        self.assertAlmostEqual(vehicle["wheelbase_m"], 3.0)
        self.assertAlmostEqual(vehicle["max_steer_rad"], pf.VEHICLE["max_steer_rad"])
        readme = (self.output / "README.md").read_text(encoding="utf-8")
        self.assertIn("CC-BY-NC-4.0", readme)
        index = json.loads((self.output / "index.json").read_text(encoding="utf-8"))
        self.assertEqual(index["fixture_version"], "v9")

    def test_no_quad_overlaps_start_or_goal(self):
        self.assertEqual(self.generate(), 0)
        document = self.load_cases()
        for case in document["cases"]:
            for label in ("start", "goal"):
                pose = case[label]
                box = pf.footprint_corners(
                    (pose["x_m"], pose["y_m"], pose["yaw_rad"]), pf.VEHICLE)
                for obstacle in case["obstacles"]:
                    quad = [(v["x_m"], v["y_m"]) for v in obstacle["vertices"]]
                    self.assertFalse(pf.convex_polygons_overlap(quad, box),
                                     "%s %s %s" % (case["id"], label, obstacle["id"]))

    def test_canvas_contains_all_vertices(self):
        self.assertEqual(self.generate(), 0)
        document = self.load_cases()
        for case in document["cases"]:
            grid = case["grid"]
            for obstacle in case["obstacles"]:
                for vertex in obstacle["vertices"]:
                    x = (vertex["x_m"] - grid["origin"]["x_m"]) / pf.RESOLUTION_M
                    y = (vertex["y_m"] - grid["origin"]["y_m"]) / pf.RESOLUTION_M
                    self.assertGreaterEqual(x, -1e-6)
                    self.assertLessEqual(x, grid["width"] + 1e-6)
                    self.assertGreaterEqual(y, -1e-6)
                    self.assertLessEqual(y, grid["height"] + 1e-6)

    def test_precheck_merged_into_self_check(self):
        precheck = {"performed_by": "unit-test", "valid_cases": 2,
                    "invalid_cases": [], "cases": {}}
        path = self.root / "precheck.json"
        path.write_text(json.dumps(precheck), encoding="utf-8")
        self.assertEqual(self.generate(precheck_json=str(path)), 0)
        document = self.load_cases()
        self.assertEqual(
            document["self_check"]["collision_precheck"]["valid_cases"], 2)
        readme = (self.output / "README.md").read_text(encoding="utf-8")
        self.assertIn("碰撞预检", readme)

    def test_check_mode_roundtrip(self):
        self.assertEqual(self.generate(), 0)
        original = pf.EXPECTED_CASES
        pf.EXPECTED_CASES = 2
        try:
            args = types.SimpleNamespace(repo_root=str(self.root),
                                         output_dir=str(self.output))
            self.assertEqual(pf.check(args), 0)
        finally:
            pf.EXPECTED_CASES = original

    def test_check_rejects_goal_overlap(self):
        self.assertEqual(self.generate(), 0)
        path = self.output / "paths" / "cases_bench_parkbench.json"
        document = json.loads(path.read_text(encoding="utf-8"))
        goal = document["cases"][0]["goal"]
        document["cases"][0]["obstacles"].append({
            "id": "bad",
            "type": "convex_polygon",
            "vertices": [
                {"x_m": goal["x_m"] - 0.1, "y_m": goal["y_m"] - 0.1},
                {"x_m": goal["x_m"] + 0.1, "y_m": goal["y_m"] - 0.1},
                {"x_m": goal["x_m"] + 0.1, "y_m": goal["y_m"] + 0.1},
                {"x_m": goal["x_m"] - 0.1, "y_m": goal["y_m"] + 0.1},
            ],
        })
        path.write_text(json.dumps(document), encoding="utf-8")
        original = pf.EXPECTED_CASES
        pf.EXPECTED_CASES = 2
        try:
            args = types.SimpleNamespace(repo_root=str(self.root),
                                         output_dir=str(self.output))
            self.assertEqual(pf.check(args), 1)
        finally:
            pf.EXPECTED_CASES = original

    def test_generate_rejects_wrong_case_count(self):
        with self.assertRaises(pf.ParkbenchFixtureError):
            self.generate(expected_cases=3)


if __name__ == "__main__":
    unittest.main()
