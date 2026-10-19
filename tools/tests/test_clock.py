"""时钟抽象与 FakeClock 的单元测试（无子进程）。"""

import os
import sys
import unittest
from datetime import datetime, timezone

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from tools import clock as clock_mod


class ClockHelpersTest(unittest.TestCase):
    def test_utc_iso_millisecond_format(self):
        moment = datetime(2026, 1, 2, 3, 4, 5, 123456, tzinfo=timezone.utc)
        self.assertEqual(clock_mod.utc_iso(moment), "2026-01-02T03:04:05.123Z")

    def test_parse_utc_iso_roundtrip(self):
        text = "2026-01-02T03:04:05.123Z"
        parsed = clock_mod.parse_utc_iso(text)
        self.assertEqual(parsed.microsecond, 123000)
        self.assertEqual(clock_mod.utc_iso(parsed), text)

    def test_run_id_timestamp_is_microsecond_precision(self):
        moment = datetime(2026, 10, 9, 22, 30, 1, 999999, tzinfo=timezone.utc)
        stamp = clock_mod.run_id_timestamp(moment)
        self.assertEqual(stamp, "20261009T223001999999Z")
        self.assertRegex(stamp, r"^\d{8}T\d{12}Z$")


class FakeClockTest(unittest.TestCase):
    def test_advance_and_sleep(self):
        fake = clock_mod.FakeClock(monotonic=1000.0)
        before = fake.utc_now()
        fake.sleep(2.5)
        self.assertAlmostEqual(fake.monotonic(), 1002.5, places=6)
        self.assertEqual((fake.utc_now() - before).total_seconds(), 2.5)

    def test_negative_advance_rejected(self):
        fake = clock_mod.FakeClock()
        with self.assertRaises(ValueError):
            fake.advance(-1.0)


if __name__ == "__main__":
    unittest.main()
