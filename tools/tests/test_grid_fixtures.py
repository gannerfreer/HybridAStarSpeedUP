"""v6 栅格夹具生成器测试：splitmix64 已知向量、阈值映射、手算用例计数、
base64 编解码、旋转原点坐标、障碍叠加、降级与 scene_id 规范序列化、
生成确定性（临时目录，不写仓库）。"""

import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from tools import grid_fixtures as gf  # noqa: E402


class Splitmix64Test(unittest.TestCase):
    """splitmix64 已知向量（独立于文档的参考实现输出）。"""

    KNOWN = {
        0: 16294208416658607535,
        1: 10451216379200822465,
        2: 10905525725756348110,
        3: 2092789425003139053,
        0x9E3779B97F4A7C15: 7960286522194355700,
    }

    def test_known_vectors(self):
        for seed, expected in self.KNOWN.items():
            self.assertEqual(gf.splitmix64(seed), expected, seed)

    def test_output_range_and_mask(self):
        for seed in range(200):
            value = gf.splitmix64(seed)
            self.assertGreaterEqual(value, 0)
            self.assertLess(value, 1 << 64)

    def test_threshold_mapping(self):
        self.assertEqual(gf.threshold_state(0), 0)
        self.assertEqual(gf.threshold_state(81), 0)
        self.assertEqual(gf.threshold_state(82), 1)
        self.assertEqual(gf.threshold_state(93), 1)
        self.assertEqual(gf.threshold_state(94), 2)
        self.assertEqual(gf.threshold_state(99), 2)
        # h % 100 的映射
        self.assertEqual(gf.threshold_state(100 + 0), 0)
        self.assertEqual(gf.threshold_state(100 + 82), 1)
        self.assertEqual(gf.threshold_state(100 + 94), 2)

    def test_display_spot_check_vectors(self):
        vectors = (
            (20261012, 256, 0, 0, 10310797173907234146, 46, 0),
            (20261012, 256, 1, 0, 17824297617258956416, 16, 0),
            (20261012, 256, 255, 255, 13021325191633500146, 46, 0),
            (20261013, 512, 0, 0, 8467169762938477137, 37, 0),
            (20261013, 512, 1, 0, 9373545611912741094, 94, 2),
            (20261013, 512, 511, 511, 12186592873910028583, 83, 1),
            (20261014, 1024, 0, 0, 4154154545692503677, 77, 0),
            (20261014, 1024, 1, 0, 15893866491313827054, 54, 0),
            (20261014, 1024, 1023, 1023, 17846917371812991894, 94, 2),
        )
        for seed, width, row, col, expected_h, expected_v, expected_state in vectors:
            h = gf.display_cell_hash(seed, width, row, col)
            self.assertEqual(h, expected_h)
            self.assertEqual(h % 100, expected_v)
            self.assertEqual(gf.threshold_state(h), expected_state)


class Base64Test(unittest.TestCase):
    def test_roundtrip(self):
        states = bytes([0, 1, 2, 0, 1, 2, 0, 0, 1])
        encoded = gf.encode_cells(states)
        self.assertEqual(gf.decode_cells(encoded, 3, 3), list(states))

    def test_snapshot_small_vector(self):
        self.assertEqual(gf.decode_cells("AAEAAAAC", 3, 2), [0, 1, 0, 0, 0, 2])

    def test_protocol_example_vector(self):
        self.assertEqual(gf.decode_cells("AAEAAAAA", 3, 2), [0, 1, 0, 0, 0, 0])

    def test_rejects_length_mismatch(self):
        with self.assertRaises(gf.FixtureError):
            gf.decode_cells("AAEAAAAC", 2, 2)

    def test_rejects_invalid_state_value(self):
        # 三字节 [0,1,3] -> base64 "AAED"
        with self.assertRaises(gf.FixtureError):
            gf.decode_cells("AAED", 3, 1)

    def test_rejects_bad_alphabet_and_padding(self):
        with self.assertRaises(gf.FixtureError):
            gf.decode_cells("AAE!AAAA", 6, 1)
        with self.assertRaises(gf.FixtureError):
            gf.decode_cells("AAE", 2, 1)
        with self.assertRaises(gf.FixtureError):
            gf.decode_cells("AAE=AAAA", 6, 1)

    def test_rejects_noncanonical_padding(self):
        # "AB==" 与 "AA==" 解码相同字节，但 AB== 的填充位非零，不是规范编码
        with self.assertRaises(gf.FixtureError):
            gf.decode_cells("AB==", 1, 1)

    def test_all_states_roundtrip(self):
        states = bytes(range(3)) * 341
        encoded = gf.encode_cells(states)
        self.assertEqual(gf.decode_cells(encoded, 33, 31), list(states))


class RotatedOriginTest(unittest.TestCase):
    def test_g1_centers(self):
        origin = {"x_m": 10.0, "y_m": -5.0, "yaw_rad": math.pi / 2.0}
        expected = {
            (0, 0): (9.75, -4.75),
            (1, 2): (9.25, -3.75),
            (3, 3): (8.25, -3.25),
        }
        for (row, col), center in expected.items():
            actual = gf.cell_center(origin, 0.5, row, col)
            self.assertAlmostEqual(actual[0], center[0], places=9)
            self.assertAlmostEqual(actual[1], center[1], places=9)

    def test_g1_cell_polygon(self):
        origin = {"x_m": 10.0, "y_m": -5.0, "yaw_rad": math.pi / 2.0}
        corners = gf.cell_corners(origin, 0.5, 0, 0)
        expected = [(10.0, -5.0), (10.0, -4.5), (9.5, -4.5), (9.5, -5.0)]
        for actual, wanted in zip(corners, expected):
            self.assertAlmostEqual(actual[0], wanted[0], places=9)
            self.assertAlmostEqual(actual[1], wanted[1], places=9)

    def test_g2_centers(self):
        origin = {"x_m": -3.0, "y_m": 2.0, "yaw_rad": -math.pi / 2.0}
        expected = {
            (0, 0): (-2.75, 1.75),
            (0, 3): (-2.75, 0.25),
            (2, 1): (-1.75, 1.25),
            (3, 0): (-1.25, 1.75),
            (3, 3): (-1.25, 0.25),
        }
        for (row, col), center in expected.items():
            actual = gf.cell_center(origin, 0.5, row, col)
            self.assertAlmostEqual(actual[0], center[0], places=9)
            self.assertAlmostEqual(actual[1], center[1], places=9)

    def test_world_to_cell_roundtrip_rotated(self):
        origin = {"x_m": 10.0, "y_m": -5.0, "yaw_rad": math.pi / 2.0}
        for row in range(4):
            for col in range(4):
                x, y = gf.cell_center(origin, 0.5, row, col)
                self.assertEqual(gf.world_to_cell(origin, 0.5, x, y), (row, col))


class HandFixtureRulesTest(unittest.TestCase):
    def test_g8_counts(self):
        counts = gf.g8_expected_counts()
        self.assertEqual(counts, {0: 3026, 1: 1008, 2: 62})
        self.assertEqual(sum(counts.values()), 4096)

    def test_g8_rule_precedence(self):
        width = 64
        cells = []
        for row in range(width):
            for col in range(width):
                if row == 0 or row == width - 1 or col == 0 or col == width - 1:
                    cells.append(1)
                elif row == col:
                    cells.append(2)
                elif (row + col) % 5 == 0:
                    cells.append(1)
                else:
                    cells.append(0)
        spots = {(0, 0): 1, (1, 1): 2, (1, 2): 0, (1, 4): 1, (2, 3): 1,
                 (31, 31): 2, (31, 32): 0, (62, 62): 2, (62, 60): 0, (63, 63): 1}
        for (row, col), state in spots.items():
            self.assertEqual(cells[row * width + col], state, (row, col))

    def test_edge_cells_in_bounds(self):
        for row, col in ((-1, 0), (0, -1), (4, 0), (0, 4)):
            self.assertFalse(gf.in_bounds(4, 4, row, col))
        self.assertTrue(gf.in_bounds(4, 4, 0, 0))
        self.assertTrue(gf.in_bounds(4, 4, 3, 3))

    def test_obstacle_overlay_rasterization(self):
        cells = gf.rasterize_obstacles(
            gf.G5_OBSTACLES, 6, 4, 1.0, {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0})
        expected = [
            0, 0, 0, 0, 1, 1,
            0, 1, 1, 0, 0, 0,
            0, 1, 1, 0, 0, 1,
            0, 1, 0, 0, 0, 0,
        ]
        self.assertEqual(cells, expected)
        self.assertEqual(sum(cells), 8)
        self.assertNotIn(2, cells)

    def test_coord_conversion_inverse(self):
        origin = {"x_m": 1.5, "y_m": -2.0, "yaw_rad": 0.0}
        cases = {
            (1.5, -2.0): (0, 0),
            (1.7499, -1.7501): (0, 0),
            (1.75, -1.75): (1, 1),
            (2.9999, -0.7501): (4, 5),
        }
        for point, expected in cases.items():
            self.assertEqual(gf.world_to_cell(origin, 0.25, point[0], point[1]), expected)
        self.assertEqual(gf.world_to_cell(origin, 0.25, 3.0, -0.75), (5, 6))
        self.assertFalse(gf.in_bounds(6, 5, 5, 6))

    def test_degrade_cases(self):
        result = gf.degrade_resolution(
            {"min_x_m": 0.0, "min_y_m": 0.0, "max_x_m": 1600.0, "max_y_m": 1600.0}, 0.1)
        self.assertEqual(result["effective_resolution_m"], 0.4)
        self.assertTrue(result["degraded"])
        self.assertEqual((result["width"], result["height"], result["cells"]), (4000, 4000, 16000000))
        self.assertEqual(result["doublings"], 2)

        result = gf.degrade_resolution(
            {"min_x_m": 0.0, "min_y_m": 0.0, "max_x_m": 2000.0, "max_y_m": 2000.0}, 0.1)
        self.assertEqual(result["effective_resolution_m"], 0.8)
        self.assertEqual((result["width"], result["height"], result["cells"]), (2500, 2500, 6250000))
        self.assertEqual(result["doublings"], 3)

    def test_degrade_guard_exhaustion(self):
        result = gf.degrade_resolution(
            {"min_x_m": 0.0, "min_y_m": 0.0, "max_x_m": 2e12, "max_y_m": 2e12}, 0.1)
        self.assertIsNone(result)

    def test_degrade_no_degradation(self):
        result = gf.degrade_resolution(
            {"min_x_m": 0.0, "min_y_m": 0.0, "max_x_m": 20.0, "max_y_m": 20.0}, 0.1)
        self.assertFalse(result["degraded"])
        self.assertEqual(result["doublings"], 0)
        self.assertEqual((result["width"], result["height"]), (200, 200))


class SceneIdTest(unittest.TestCase):
    def test_protocol_example_conformance(self):
        # 生成器自检：规范序列化与协议 §5.3 冻结示例一致（含 SHA-256）
        self.assertEqual(gf.verify_scene_id_example(), gf.SCENE_ID_EXAMPLE_SHA256)

    def test_fixed_format(self):
        self.assertEqual(gf.format_fixed(0.2), "0.200000000")
        self.assertEqual(gf.format_fixed(0.0), "0.000000000")
        self.assertEqual(gf.format_fixed(-0.0), "0.000000000")
        self.assertEqual(gf.format_fixed(1.5), "1.500000000")
        self.assertEqual(gf.format_fixed(-1e-9), "-0.000000001")

    def test_quantization_half_away_from_zero(self):
        self.assertEqual(gf.quantize_fixed(1e-9 * 0.5), 1)
        self.assertEqual(gf.quantize_fixed(-1e-9 * 0.5), -1)

    def test_scene_id_differs_with_obstacles(self):
        window = {"min_x_m": 0.0, "min_y_m": 0.0, "max_x_m": 6.0, "max_y_m": 6.0}
        origin = {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0}
        first = gf.scene_id_of("m", "v", window, 0.1, 60, 60, origin, [])
        second = gf.scene_id_of("m", "v", window, 0.1, 60, 60, origin, [
            {"id": "obs-1", "type": "rectangle", "x_min_m": 1.0, "y_min_m": 1.0,
             "x_max_m": 2.0, "y_max_m": 2.0},
        ])
        self.assertNotEqual(first, second)
        self.assertEqual(len(first), 64)


class GenerationTest(unittest.TestCase):
    """在临时目录生成与复验；不写仓库。"""

    def _generate(self, root):
        return gf.generate("v6", root, _REPO_ROOT)

    def test_generation_and_validation(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "v6"
            summary = self._generate(root)
            self.assertTrue(summary["validated"])
            self.assertEqual(summary["grid_files"], 14)
            for name in ("README.md", "index.json", "grids/minimal.json",
                         "grids/protocol_cases.json", "grids/display_1024.json",
                         "grids/display_spec.json"):
                self.assertTrue((root / name).is_file(), name)
            self.assertFalse((root / gf.MANIFEST_NAME).exists())
            problems = gf.compare_tree(root, gf.build_documents(_REPO_ROOT), _REPO_ROOT)
            self.assertEqual(problems, [])

    def test_generation_is_deterministic(self):
        with tempfile.TemporaryDirectory() as first, tempfile.TemporaryDirectory() as second:
            self._generate(Path(first) / "v6")
            self._generate(Path(second) / "v6")
            documents_first = sorted(
                (path.relative_to(first).as_posix(), path.read_bytes())
                for path in Path(first).rglob("*") if path.is_file())
            documents_second = sorted(
                (path.relative_to(second).as_posix(), path.read_bytes())
                for path in Path(second).rglob("*") if path.is_file())
            self.assertEqual(documents_first, documents_second)

    def test_minimal_is_byte_identical_to_v5(self):
        v5_minimal = Path(_REPO_ROOT) / "tests" / "fixtures" / "v5" / "grids" / "minimal.json"
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "v6"
            self._generate(root)
            self.assertEqual((root / "grids" / "minimal.json").read_bytes(),
                             v5_minimal.read_bytes())

    def test_compare_tree_detects_tampering(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp) / "v6"
            self._generate(root)
            target = root / "grids" / "edge_cells.json"
            payload = json.loads(target.read_text(encoding="utf-8"))
            payload["cells"][0] = 0
            target.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                              encoding="utf-8")
            problems = gf.compare_tree(root, gf.build_documents(_REPO_ROOT), _REPO_ROOT)
            self.assertTrue(any("edge_cells" in problem for problem in problems), problems)

    def test_check_reports_missing_repo_tree(self):
        with tempfile.TemporaryDirectory() as tmp:
            ok, problems = gf.check("v6", tmp)
            self.assertFalse(ok)
            self.assertTrue(problems)

    def test_rejects_unknown_version(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(gf.FixtureError):
                gf.generate("v7", Path(tmp) / "v7", _REPO_ROOT)


if __name__ == "__main__":
    unittest.main()
