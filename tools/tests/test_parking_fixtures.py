"""parking_fixtures（v8 停车夹具生成器）测试：生成/复制完整性/画布裕量/
预检并入/检查模式/输入拒绝。仅标准库。"""

import json
import os
import sys
import tempfile
import types
import unittest
from pathlib import Path

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from tools import parking_fixtures as pf


def make_neutral_document(case_count=115):
    cases = []
    for index in range(1, case_count + 1):
        x = -10.0 + (index % 20) * 0.5
        y = -10.0 + ((index * 7) % 20) * 0.5
        cases.append(
            {
                "index": index,
                "source": "CaseNo_%d.mat" % index,
                "boundary": {
                    "x0_m": x,
                    "y0_m": y,
                    "theta0_rad": 0.5,
                    "xf_m": x + 5.0,
                    "yf_m": y + 3.0,
                    "thetaf_rad": -0.25,
                },
                "obstacles": [
                    {
                        "id": "poly_0",
                        "vertices": [
                            {"x_m": 0.0, "y_m": 0.0},
                            {"x_m": 0.0, "y_m": 1.0},
                            {"x_m": 1.0, "y_m": 1.0},
                            {"x_m": 1.0, "y_m": 0.0},
                        ],
                    }
                ],
            }
        )
    return {
        "schema_version": 1,
        "type": "parking_cases_export",
        "source": "unit-test",
        "canvas": {"x_min_m": -20.0, "x_max_m": 20.0, "y_min_m": -20.0, "y_max_m": 20.0},
        "vehicle": {
            "wheelbase_m": 2.8,
            "front_overhang_m": 0.96,
            "rear_overhang_m": 0.929,
            "width_m": 1.942,
            "max_steer_rad": 0.75,
            "safety_margin_m": 0.0,
        },
        "case_count": case_count,
        "cases": cases,
    }


class ParkingFixturesTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        source = self.root / "tests" / "fixtures" / "v7"
        (source / "paths").mkdir(parents=True)
        (source / "vehicles").mkdir(parents=True)
        (source / "map").mkdir(parents=True)
        (source / "playback").mkdir(parents=True)
        (source / "paths" / "cases.json").write_text('{"cases": []}\n', encoding="utf-8")
        (source / "vehicles" / "default.json").write_text('{"id": "default"}\n', encoding="utf-8")
        (source / "map" / "README.md").write_text("nested readme\n", encoding="utf-8")
        (source / "playback" / "index.json").write_text('{"nested": 1}\n', encoding="utf-8")
        (source / "index.json").write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "type": "fixture_index",
                    "fixture_version": "v7",
                    "revision_of": "v6",
                    "revision_summary": "test",
                    "revision_history": [],
                    "sets": [{"id": "paths", "description": "paths"}],
                    "notes": [],
                }
            ),
            encoding="utf-8",
        )
        (source / "README.md").write_text("v7 readme\n", encoding="utf-8")
        self.source = source
        self.input_path = self.root / "parking_cases.json"
        self.input_path.write_text(
            json.dumps(make_neutral_document()), encoding="utf-8"
        )
        self.output = self.root / "tests" / "fixtures" / "v8"

    def tearDown(self):
        self.tmp.cleanup()

    def generate(self, precheck_json=None, margin=0.1, input_path=None):
        args = types.SimpleNamespace(
            repo_root=str(self.root),
            input=str(input_path or self.input_path),
            source_version="v7",
            output_dir=str(self.output),
            force=True,
            precheck_json=precheck_json,
            canvas_margin=margin,
        )
        return pf.generate(args)

    def test_generate_builds_expected_tree(self):
        self.assertEqual(self.generate(), 0)
        cases = json.loads(
            (self.output / "paths" / "cases_bench_parking.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(len(cases["cases"]), 115)
        self.assertEqual(cases["cases"][0]["id"], "case_1")
        self.assertEqual(cases["cases"][0]["class"], "parking")
        self.assertEqual(cases["cases"][0]["expected"]["status"], "success")
        vehicle = json.loads(
            (self.output / "vehicles" / "parking_bench.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(vehicle["id"], "parking_bench_vehicle")
        self.assertEqual(vehicle["safety_margin_m"], 0.0)
        index = json.loads((self.output / "index.json").read_text(encoding="utf-8"))
        self.assertEqual(index["fixture_version"], "v8")
        self.assertEqual(index["revision_of"], "v7")

    def test_generate_copies_nested_v7_files(self):
        self.assertEqual(self.generate(), 0)
        self.assertTrue((self.output / "map" / "README.md").is_file())
        self.assertTrue((self.output / "playback" / "index.json").is_file())
        self.assertTrue((self.output / "paths" / "cases.json").is_file())
        self.assertFalse((self.output / pf.MANIFEST_NAME).exists())

    def test_canvas_margin_extends_grid(self):
        self.assertEqual(self.generate(margin=0.1), 0)
        cases = json.loads(
            (self.output / "paths" / "cases_bench_parking.json").read_text(
                encoding="utf-8"
            )
        )
        grid = cases["cases"][0]["grid"]
        self.assertAlmostEqual(grid["origin"]["x_m"], -20.1)
        self.assertEqual(grid["width"], 402)
        self.assertEqual(grid["height"], 402)

    def test_precheck_is_merged(self):
        precheck = {
            "performed_by": "unit-test",
            "valid_cases": 115,
            "invalid_cases": [],
            "cases": {},
        }
        precheck_path = self.root / "precheck.json"
        precheck_path.write_text(json.dumps(precheck), encoding="utf-8")
        self.assertEqual(self.generate(precheck_json=str(precheck_path)), 0)
        cases = json.loads(
            (self.output / "paths" / "cases_bench_parking.json").read_text(
                encoding="utf-8"
            )
        )
        self.assertEqual(
            cases["self_check"]["collision_precheck"]["valid_cases"], 115
        )
        readme = (self.output / "README.md").read_text(encoding="utf-8")
        self.assertIn("正式碰撞检查器起终点预检", readme)

    def test_check_mode_accepts_generated_tree(self):
        self.assertEqual(self.generate(), 0)
        args = types.SimpleNamespace(
            repo_root=str(self.root), output_dir=str(self.output)
        )
        self.assertEqual(pf.check(args), 0)

    def test_check_mode_rejects_wrong_case_count(self):
        self.assertEqual(self.generate(), 0)
        cases_path = self.output / "paths" / "cases_bench_parking.json"
        document = json.loads(cases_path.read_text(encoding="utf-8"))
        document["cases"] = document["cases"][:-1]
        cases_path.write_text(json.dumps(document), encoding="utf-8")
        args = types.SimpleNamespace(
            repo_root=str(self.root), output_dir=str(self.output)
        )
        self.assertEqual(pf.check(args), 1)

    def test_generate_rejects_wrong_case_count(self):
        bad_input = self.root / "bad.json"
        bad_input.write_text(
            json.dumps(make_neutral_document(case_count=3)), encoding="utf-8"
        )
        with self.assertRaises(pf.ParkingFixtureError):
            self.generate(input_path=bad_input)


if __name__ == "__main__":
    unittest.main()
