"""junit_runner 测试：标准库发现、JUnit XML 结构、退出码。"""

import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

JUNIT_RUNNER = os.path.join(_REPO_ROOT, "tools", "junit_runner.py")

PASSING_TEST = (
    "import unittest\n"
    "\n"
    "class Passing(unittest.TestCase):\n"
    "    def test_pass(self):\n"
    "        self.assertEqual(2 + 2, 4)\n"
)

FAILING_TEST = (
    "import unittest\n"
    "\n"
    "class Failing(unittest.TestCase):\n"
    "    def test_fail(self):\n"
    "        self.assertEqual(1, 2)\n"
)


class JUnitRunnerTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)

    def tearDown(self):
        self.tmp.cleanup()

    def _run(self, tests_dir, xml_path):
        return subprocess.run(
            [sys.executable, JUNIT_RUNNER, "--start-dir", str(tests_dir),
             "--pattern", "test_*.py", "--junit", str(xml_path)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=120)

    def test_passing_suite_exit_zero_and_xml(self):
        tests = self.root / "tests"
        tests.mkdir()
        (tests / "test_pass.py").write_text(PASSING_TEST, encoding="utf-8")
        xml = self.root / "out" / "junit.xml"
        proc = self._run(tests, xml)
        self.assertEqual(proc.returncode, 0, proc.stderr.decode("utf-8", "replace"))
        self.assertTrue(xml.is_file())
        root = ElementTree.parse(str(xml)).getroot()
        self.assertEqual(root.get("tests"), "1")
        self.assertEqual(root.get("failures"), "0")
        cases = root.findall(".//testcase")
        self.assertEqual(len(cases), 1)
        self.assertIn("test_pass", cases[0].get("name"))

    def test_failure_exit_one_and_xml_records_failure(self):
        tests = self.root / "tests"
        tests.mkdir()
        (tests / "test_pass.py").write_text(PASSING_TEST, encoding="utf-8")
        (tests / "test_fail.py").write_text(FAILING_TEST, encoding="utf-8")
        xml = self.root / "junit.xml"
        proc = self._run(tests, xml)
        self.assertEqual(proc.returncode, 1)
        root = ElementTree.parse(str(xml)).getroot()
        self.assertEqual(root.get("tests"), "2")
        self.assertEqual(root.get("failures"), "1")
        failures = root.findall(".//failure")
        self.assertEqual(len(failures), 1)

    def test_empty_suite_fails_loudly(self):
        tests = self.root / "empty"
        tests.mkdir()
        xml = self.root / "empty.xml"
        proc = self._run(tests, xml)
        self.assertEqual(proc.returncode, 1)
        self.assertTrue(xml.is_file())


if __name__ == "__main__":
    unittest.main()
