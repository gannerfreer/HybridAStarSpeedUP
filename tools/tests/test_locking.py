"""FileLock 行为测试：同进程互斥、超时、多进程互斥、清理。

多进程测试使用受控短生命周期子进程，仅为验证 flock 跨进程互斥。
"""

import os
import select
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from tools.locking import FileLock, LockTimeoutError


class FileLockTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name) / "locks" / "test.lock"

    def tearDown(self):
        self.tmp.cleanup()

    def test_acquire_release_and_conflict(self):
        first = FileLock(self.path)
        self.assertTrue(first.acquire(timeout=1.0))
        second = FileLock(self.path)
        self.assertFalse(second.acquire(timeout=0.2))
        first.release()
        self.assertTrue(second.acquire(timeout=1.0))
        second.release()

    def test_context_manager_raises_on_timeout(self):
        holder = FileLock(self.path)
        self.assertTrue(holder.acquire(timeout=1.0))
        try:
            with self.assertRaises(LockTimeoutError):
                with FileLock(self.path, timeout=0.1):
                    pass
        finally:
            holder.release()

    def test_unlink_releases_and_removes(self):
        lock = FileLock(self.path)
        self.assertTrue(lock.acquire(timeout=1.0))
        lock.unlink()
        self.assertFalse(self.path.exists())

    def test_multiprocess_mutual_exclusion(self):
        script = (
            "import sys, time\n"
            "sys.path.insert(0, sys.argv[2])\n"
            "from tools.locking import FileLock\n"
            "lock = FileLock(sys.argv[1])\n"
            "if not lock.acquire(timeout=5.0):\n"
            "    sys.exit(2)\n"
            "print('locked', flush=True)\n"
            "time.sleep(30)\n"
        )
        proc = subprocess.Popen(
            [sys.executable, "-c", script, str(self.path), _REPO_ROOT],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )
        try:
            ready, _, _ = select.select([proc.stdout], [], [], 10)
            self.assertTrue(ready, "子进程未在 10 秒内获取锁")
            self.assertEqual(proc.stdout.readline().strip(), "locked")
            contender = FileLock(self.path)
            self.assertFalse(contender.acquire(timeout=0.3),
                             "子进程持锁期间不应能获取同一把锁")
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)
        winner = FileLock(self.path)
        self.assertTrue(winner.acquire(timeout=2.0))
        winner.release()


if __name__ == "__main__":
    unittest.main()
