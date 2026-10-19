"""进程身份、终止升级与子进程清理测试。

只使用受控短生命周期子进程；宽限时间缩短以保持测试快速。
"""

import os
import select
import subprocess
import sys
import time
import unittest
from unittest import mock

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from tools import clock as clock_mod
from tools import runtime


def _wait_ready(proc, timeout=10.0):
    ready, _, _ = select.select([proc.stdout], [], [], timeout)
    if not ready:
        return None
    return proc.stdout.readline().decode("utf-8", "replace").strip()


def _stop_quietly(proc):
    if proc.poll() is None:
        proc.kill()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            pass
    try:
        proc.stdout.close()
    except OSError:
        pass


def _pid_gone_or_zombie(pid):
    """僵尸进程视为已清理（父进程可能不立即回收）。"""
    if not runtime.pid_alive(pid):
        return True
    try:
        with open("/proc/%d/stat" % int(pid), "r", encoding="utf-8",
                  errors="replace") as handle:
            data = handle.read()
    except OSError:
        return True
    close = data.rfind(")")
    if close < 0:
        return True
    fields = data[close + 2:].split()
    return bool(fields) and fields[0] in ("Z", "X", "x")


class ProcessIdentityTest(unittest.TestCase):
    def test_start_ticks_and_identity(self):
        ticks = runtime.process_start_ticks(os.getpid())
        self.assertIsNotNone(ticks)
        self.assertTrue(runtime.pid_identity_matches(os.getpid(), ticks))
        self.assertFalse(runtime.pid_identity_matches(os.getpid(), ticks + 1))
        self.assertFalse(runtime.pid_identity_matches(987654321, 1))

    def test_pid_reuse_protection_blocks_kill(self):
        ticks = runtime.process_start_ticks(os.getpid())
        with mock.patch("tools.runtime.os.killpg") as killpg:
            result = runtime.stop_process_tree(
                os.getpid(), 999999999, ticks + 1, clock=clock_mod.FakeClock())
            self.assertEqual(result, "pid_reused")
            killpg.assert_not_called()

    def test_own_process_group_refused(self):
        result = runtime.stop_process_tree(
            os.getpid(), os.getpgrp(), runtime.process_start_ticks(os.getpid()),
            clock=clock_mod.FakeClock())
        self.assertEqual(result, "own_group")

    def test_nonexistent_group_already_exited(self):
        result = runtime.stop_process_tree(
            987654321, 987654321, None, clock=clock_mod.FakeClock())
        self.assertEqual(result, "already_exited")


class TerminationTest(unittest.TestCase):
    def test_sigterm_terminates(self):
        script = (
            "import signal, time\n"
            "signal.signal(signal.SIGTERM, signal.SIG_DFL)\n"
            "print('ready', flush=True)\n"
            "time.sleep(60)\n"
        )
        proc = subprocess.Popen([sys.executable, "-c", script],
                                stdout=subprocess.PIPE, start_new_session=True)
        try:
            self.assertEqual(_wait_ready(proc), "ready")
            ticks = runtime.process_start_ticks(proc.pid)
            result = runtime.stop_process_tree(
                proc.pid, os.getpgid(proc.pid), ticks,
                clock=clock_mod.Clock(), grace=2.0)
            proc.wait(timeout=5)
            self.assertEqual(result, "sigterm")
            self.assertIsNotNone(proc.returncode)
        finally:
            _stop_quietly(proc)

    def test_sigkill_escalation_after_ignored_sigterm(self):
        script = (
            "import signal, time\n"
            "signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
            "print('ready', flush=True)\n"
            "time.sleep(60)\n"
        )
        proc = subprocess.Popen([sys.executable, "-c", script],
                                stdout=subprocess.PIPE, start_new_session=True)
        try:
            self.assertEqual(_wait_ready(proc), "ready")
            ticks = runtime.process_start_ticks(proc.pid)
            started = time.monotonic()
            result = runtime.stop_process_tree(
                proc.pid, os.getpgid(proc.pid), ticks,
                clock=clock_mod.Clock(), grace=0.5)
            proc.wait(timeout=5)
            elapsed = time.monotonic() - started
            self.assertEqual(result, "sigkill")
            self.assertLess(elapsed, 5.0)
            self.assertLess(proc.returncode, 0)
        finally:
            _stop_quietly(proc)

    def test_descendant_cleanup_with_process_group(self):
        script = (
            "import signal, subprocess, sys, time\n"
            "signal.signal(signal.SIGTERM, signal.SIG_IGN)\n"
            "child = subprocess.Popen([sys.executable, '-c',"
            " 'import time; time.sleep(60)'])\n"
            "print(child.pid, flush=True)\n"
            "time.sleep(60)\n"
        )
        proc = subprocess.Popen([sys.executable, "-c", script],
                                stdout=subprocess.PIPE, start_new_session=True)
        try:
            line = _wait_ready(proc)
            self.assertIsNotNone(line)
            grandchild = int(line)
            self.assertTrue(runtime.pid_alive(grandchild))
            ticks = runtime.process_start_ticks(proc.pid)
            result = runtime.stop_process_tree(
                proc.pid, os.getpgid(proc.pid), ticks,
                clock=clock_mod.Clock(), grace=0.5)
            proc.wait(timeout=5)
            self.assertEqual(result, "sigkill")
            deadline = time.monotonic() + 5.0
            while not _pid_gone_or_zombie(grandchild) and time.monotonic() < deadline:
                time.sleep(0.05)
            self.assertTrue(_pid_gone_or_zombie(grandchild),
                            "同一进程组的孙子进程应被清理")
        finally:
            _stop_quietly(proc)


if __name__ == "__main__":
    unittest.main()
