"""基于 fcntl.flock 的跨进程文件锁。

- 锁文件位于公共 Git 目录的 planner-runtime/ 下，多个 worktree 共用同一把锁。
- FileLock(shared=True) 供证据写入方使用（共享），封存方使用默认独占锁。
"""

import errno
import fcntl
import os
import time
from pathlib import Path


class LockTimeoutError(RuntimeError):
    """上下文管理器获取锁失败时抛出。"""


class FileLock:
    """非阻塞重试的 flock 包装；acquire() 返回 True/False。"""

    def __init__(self, path, timeout=None, poll_interval=0.1, shared=False):
        self.path = Path(path)
        self.timeout = timeout
        self.poll_interval = max(0.01, float(poll_interval))
        self.shared = bool(shared)
        self._fd = None

    @property
    def acquired(self):
        return self._fd is not None

    def acquire(self, timeout=None):
        if self._fd is not None:
            raise RuntimeError("锁已获取: %s" % self.path)
        effective = self.timeout if timeout is None else timeout
        deadline = None if effective is None else time.monotonic() + float(effective)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        fd = os.open(str(self.path), os.O_RDWR | os.O_CREAT, 0o644)
        mode = fcntl.LOCK_SH if self.shared else fcntl.LOCK_EX
        try:
            while True:
                try:
                    fcntl.flock(fd, mode | fcntl.LOCK_NB)
                    self._fd = fd
                    return True
                except OSError as exc:
                    if exc.errno not in (errno.EACCES, errno.EAGAIN):
                        raise
                    if deadline is not None and time.monotonic() >= deadline:
                        os.close(fd)
                        return False
                    time.sleep(self.poll_interval)
        except BaseException:
            os.close(fd)
            raise

    def release(self):
        if self._fd is None:
            return
        try:
            fcntl.flock(self._fd, fcntl.LOCK_UN)
        finally:
            os.close(self._fd)
            self._fd = None

    def unlink(self):
        """释放并尽力删除锁文件（仅用于运行结束后的证据锁清理）。"""
        self.release()
        try:
            os.unlink(str(self.path))
        except OSError:
            pass

    def __enter__(self):
        if not self.acquire():
            raise LockTimeoutError("获取锁超时: %s" % self.path)
        return self

    def __exit__(self, exc_type, exc, tb):
        self.release()
        return False
