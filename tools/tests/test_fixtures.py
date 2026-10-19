"""夹具 freeze/verify 测试：字典序、字段、缺失/篡改/额外文件拒绝、幂等性。"""

import os
import sys
import tempfile
import unittest
from pathlib import Path

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from tools import fixtures
from tools import runtime


class FixtureTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name) / "fixtures" / "v1"
        (self.root / "sub").mkdir(parents=True)
        (self.root / "a.json").write_text('{"a": 1}\n', encoding="utf-8")
        (self.root / "sub" / "b.bin").write_bytes(b"binary-data")
        (self.root / "z.txt").write_text("z", encoding="utf-8")

    def tearDown(self):
        self.tmp.cleanup()

    def test_freeze_fields_and_order(self):
        items = fixtures.freeze(self.root)
        self.assertEqual([item["path"] for item in items],
                         ["a.json", "sub/b.bin", "z.txt"])
        manifest = runtime.read_json(fixtures.manifest_path(self.root), strict=True)
        self.assertIsInstance(manifest, list)
        for item in manifest:
            full = self.root / item["path"]
            self.assertEqual(item["size_bytes"], full.stat().st_size)
            self.assertEqual(item["sha256"], runtime.sha256_file(full))
        self.assertNotIn(fixtures.MANIFEST_NAME,
                         [item["path"] for item in manifest])

    def test_freeze_is_idempotent(self):
        fixtures.freeze(self.root)
        first = fixtures.manifest_path(self.root).read_bytes()
        fixtures.freeze(self.root)
        second = fixtures.manifest_path(self.root).read_bytes()
        self.assertEqual(first, second)

    def test_verify_accepts_frozen_tree(self):
        fixtures.freeze(self.root)
        result = fixtures.verify(self.root)
        self.assertTrue(result["ok"], result["problems"])
        self.assertEqual(result["checked"], 3)

    def test_verify_rejects_tampered_file(self):
        fixtures.freeze(self.root)
        with open(self.root / "z.txt", "a", encoding="utf-8") as handle:
            handle.write("tampered")
        result = fixtures.verify(self.root)
        self.assertFalse(result["ok"])
        self.assertIn("z.txt", "\n".join(result["problems"]))

    def test_verify_rejects_missing_file(self):
        fixtures.freeze(self.root)
        os.unlink(str(self.root / "a.json"))
        result = fixtures.verify(self.root)
        self.assertFalse(result["ok"])
        self.assertIn("清单文件缺失: a.json", result["problems"])

    def test_verify_rejects_extra_file(self):
        fixtures.freeze(self.root)
        (self.root / "new.txt").write_text("new", encoding="utf-8")
        result = fixtures.verify(self.root)
        self.assertFalse(result["ok"])
        self.assertIn("存在清单外额外文件: new.txt", result["problems"])

    def test_verify_manifest_itself_not_extra(self):
        fixtures.freeze(self.root)
        result = fixtures.verify(self.root)
        joined = "\n".join(result["problems"])
        self.assertNotIn(fixtures.MANIFEST_NAME, joined)

    def test_verify_missing_manifest_and_root(self):
        result = fixtures.verify(self.root)
        self.assertFalse(result["ok"])
        with self.assertRaises(fixtures.FixtureError):
            fixtures.freeze(self.root / "does-not-exist")


if __name__ == "__main__":
    unittest.main()
