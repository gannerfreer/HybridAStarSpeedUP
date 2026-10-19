"""E0 工具共享的运行时路径、原子写入与进程控制辅助。

与 ``Docs/component_development_plan.md`` §5.3、§9 对齐：

- 共享运行目录位于 ``git rev-parse --git-common-dir`` 下的 ``planner-runtime/``，
  多个 worktree 共用编译锁、活动登记、监控心跳与派发状态。
- 所有 JSON 状态文件使用“同目录临时文件 + os.replace”原子写入。
- 终止任务前必须核对进程启动标识（/proc/<pid>/stat 第 22 字段 starttime），
  避免 PID 复用误杀。
- 本模块只使用 CPython 3.8+ 标准库，平台为 Linux。
"""

import hashlib
import json
import os
import re
import subprocess
from pathlib import Path

from tools import clock as clock_mod
from tools import locking

SCHEMA_VERSION = 1

# §5.3 / §7.2 固定默认值
SAMPLE_INTERVAL_S = 2.0
STATUS_MAX_AGE_S = 10.0
THRESHOLD_PERCENT = 90.0
RESUME_CONSECUTIVE_SAMPLES = 3
STOP_GRACE_S = 5.0
MONITOR_HEARTBEAT_MAX_AGE_S = 6.0
DEFAULT_WORKER_CAP = 4
DEFAULT_MAX_PARALLEL = 4
TEMP_SUFFIX = ".tmp"

# 退出码约定（子进程真实退出码原样转发；基础设施拒绝使用以下码）
EXIT_OK = 0
EXIT_USAGE = 2
EXIT_DATA = 65
EXIT_UNAVAILABLE = 69
EXIT_SOFTWARE = 70
EXIT_TEMPFAIL = 75


# ---------------------------------------------------------------------------
# 路径
# ---------------------------------------------------------------------------

def repo_root_from(start):
    """从文件或目录向上寻找包含 ``.git`` 的仓库根目录。"""
    candidate = Path(start).resolve()
    if candidate.is_file():
        candidate = candidate.parent
    for path in [candidate] + list(candidate.parents):
        if (path / ".git").exists():
            return path
    return candidate


def git_common_dir(repo_root):
    """定位共享 Git 目录；命令失败时回退到 ``<repo>/.git``。"""
    try:
        proc = subprocess.run(
            ["git", "rev-parse", "--git-common-dir"],
            cwd=str(repo_root),
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            check=True,
            timeout=30,
        )
        text = proc.stdout.decode("utf-8", "replace").strip()
        if text:
            path = Path(text)
            if not path.is_absolute():
                path = (Path(repo_root) / path).resolve()
            return path
    except (OSError, subprocess.SubprocessError):
        pass
    return Path(repo_root) / ".git"


def runtime_dir(repo_root):
    """共享运行目录（登记、锁、心跳、派发状态）。"""
    return git_common_dir(repo_root) / "planner-runtime"


def locks_dir(runtime_dir_path):
    return Path(runtime_dir_path) / "locks"


def evidence_lock_path(runtime_dir_path, run_id):
    """单次运行证据写入锁：监控共享持有，封存方独占持有。"""
    return locks_dir(runtime_dir_path) / ("evidence-%s.lock" % run_id)


def live_dir(repo_root):
    return Path(repo_root) / "artifacts" / "live"


def runs_dir(repo_root):
    return Path(repo_root) / "artifacts" / "runs"


def safe_name(text, fallback="task"):
    """把任意文本清理成适合放进文件名的片段。"""
    cleaned = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(text)).strip("._-")
    return cleaned or fallback


# ---------------------------------------------------------------------------
# 原子写入与哈希
# ---------------------------------------------------------------------------

def dump_json_bytes(obj):
    text = json.dumps(obj, ensure_ascii=False, indent=2, sort_keys=False) + "\n"
    return text.encode("utf-8")


def atomic_write_bytes(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name("%s.%d%s" % (path.name, os.getpid(), TEMP_SUFFIX))
    with open(str(tmp), "wb") as handle:
        handle.write(data)
        handle.flush()
        try:
            os.fsync(handle.fileno())
        except OSError:
            pass
    os.replace(str(tmp), str(path))


def atomic_write_json(path, obj):
    """原子写 JSON；返回写入字节的 SHA256（便于 config_sha256）。"""
    data = dump_json_bytes(obj)
    atomic_write_bytes(path, data)
    return hashlib.sha256(data).hexdigest()


def atomic_write_text(path, text):
    data = text.encode("utf-8")
    atomic_write_bytes(path, data)
    return hashlib.sha256(data).hexdigest()


def read_json(path, default=None, strict=False):
    try:
        with open(str(path), "r", encoding="utf-8") as handle:
            return json.load(handle)
    except FileNotFoundError:
        return default
    except ValueError:
        if strict:
            raise
        return default


def sha256_file(path, chunk_size=1024 * 1024):
    digest = hashlib.sha256()
    with open(str(path), "rb") as handle:
        while True:
            chunk = handle.read(chunk_size)
            if not chunk:
                break
            digest.update(chunk)
    return digest.hexdigest()


def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def append_jsonl(path, record, lock_path=None, timeout=30.0):
    """追加一行 JSON；可选共享 fcntl 锁（§8.1 事件流共享锁）。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    line = json.dumps(record, ensure_ascii=False, sort_keys=False) + "\n"
    lock = None
    if lock_path is not None:
        lock = locking.FileLock(lock_path, shared=True)
        if not lock.acquire(timeout=timeout):
            raise TimeoutError("JSONL 追加锁超时: %s" % lock_path)
    try:
        with open(str(path), "a", encoding="utf-8") as handle:
            handle.write(line)
            handle.flush()
    finally:
        if lock is not None:
            lock.release()


def now_iso(clk):
    return clock_mod.utc_iso(clk.utc_now())


def list_relative_files(root):
    """递归列出普通文件（跳过符号链接），返回绝对 Path 列表。"""
    root = Path(root)
    result = []
    if not root.is_dir():
        return result
    for dirpath, dirnames, filenames in os.walk(str(root)):
        dirnames.sort()
        for name in sorted(filenames):
            full = Path(dirpath) / name
            if full.is_symlink():
                continue
            if full.is_file():
                result.append(full)
    return result


# ---------------------------------------------------------------------------
# 进程身份与终止
# ---------------------------------------------------------------------------

def pid_alive(pid):
    try:
        os.kill(int(pid), 0)
        return True
    except (OSError, ValueError, TypeError):
        return False


def process_start_ticks(pid):
    """读取 /proc/<pid>/stat 第 22 字段（starttime，开机后时钟节拍）。"""
    try:
        with open("/proc/%d/stat" % int(pid), "r", encoding="utf-8", errors="replace") as handle:
            data = handle.read()
    except (OSError, ValueError, TypeError):
        return None
    close = data.rfind(")")
    if close < 0:
        return None
    fields = data[close + 2:].split()
    if len(fields) < 20:
        return None
    try:
        return int(fields[19])
    except (TypeError, ValueError):
        return None


def pid_identity_matches(pid, expected_start_ticks):
    """进程存在且启动标识一致。expected 为 None 时只要求进程存在。"""
    if expected_start_ticks is None:
        return pid_alive(pid)
    current = process_start_ticks(pid)
    if current is None:
        return False
    try:
        return current == int(expected_start_ticks)
    except (TypeError, ValueError):
        return False


def process_group_alive(pgid):
    """进程组中是否还有非僵尸成员（僵尸视为已退出，避免误判 SIGTERM 未生效）。"""
    try:
        pgid = int(pgid)
    except (TypeError, ValueError):
        return False
    try:
        entries = os.listdir("/proc")
    except OSError:
        entries = None
    if entries is None:
        try:
            os.killpg(pgid, 0)
            return True
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        except (OSError, ValueError, TypeError):
            return False
    for entry in entries:
        if not entry.isdigit():
            continue
        try:
            with open("/proc/%s/stat" % entry, "r", encoding="utf-8",
                      errors="replace") as handle:
                data = handle.read()
        except OSError:
            continue
        close = data.rfind(")")
        if close < 0:
            continue
        fields = data[close + 2:].split()
        if len(fields) < 3:
            continue
        state = fields[0]
        if state in ("Z", "X", "x"):
            continue
        try:
            member_pgrp = int(fields[2])
        except (TypeError, ValueError):
            continue
        if member_pgrp == pgid:
            return True
    return False


def _signal_group(pgid, sig, on_event):
    try:
        os.killpg(int(pgid), sig)
        return True
    except ProcessLookupError:
        return False
    except OSError as exc:
        if on_event is not None:
            on_event("signal_failed", {"pgid": int(pgid), "signal": int(sig), "error": str(exc)})
        return False


def stop_process_tree(pid, pgid, expected_start_ticks=None, clock=None,
                      grace=None, on_event=None, on_tick=None):
    """先 SIGTERM 进程组，grace 秒后仍未退出则 SIGKILL。

    返回：already_exited / pid_reused / own_group / sigterm / sigkill / sigkill_sent。
    终止前核对进程启动标识，防止 PID 复用后误杀无关进程。
    ``on_tick`` 在等待循环中调用，供监控刷新心跳。
    """
    clk = clock if clock is not None else clock_mod.Clock()
    wait_grace = STOP_GRACE_S if grace is None else float(grace)
    try:
        pid = int(pid)
        pgid = int(pgid)
    except (TypeError, ValueError):
        return "already_exited"
    if pgid <= 1:
        return "already_exited"
    if pgid == os.getpgrp():
        if on_event is not None:
            on_event("refused_own_process_group", {"pid": pid, "pgid": pgid})
        return "own_group"
    if pid_alive(pid) and not pid_identity_matches(pid, expected_start_ticks):
        if on_event is not None:
            on_event("refused_pid_reuse", {"pid": pid, "pgid": pgid,
                                           "expected_start_ticks": expected_start_ticks,
                                           "current_start_ticks": process_start_ticks(pid)})
        return "pid_reused"
    if not process_group_alive(pgid):
        return "already_exited"
    if on_event is not None:
        on_event("signal_sent", {"pgid": pgid, "signal": "SIGTERM", "kind": "process_tree"})
    if not _signal_group(pgid, 15, on_event):
        return "already_exited"
    deadline = clk.monotonic() + wait_grace
    while clk.monotonic() < deadline:
        if not process_group_alive(pgid):
            return "sigterm"
        if on_tick is not None:
            on_tick()
        clk.sleep(0.1)
    if pid_alive(pid) and not pid_identity_matches(pid, expected_start_ticks):
        return "pid_reused"
    if on_event is not None:
        on_event("signal_sent", {"pgid": pgid, "signal": "SIGKILL", "kind": "process_tree"})
    if not _signal_group(pgid, 9, on_event):
        return "sigkill"
    deadline = clk.monotonic() + 1.0
    while clk.monotonic() < deadline:
        if not process_group_alive(pgid):
            return "sigkill"
        if on_tick is not None:
            on_tick()
        clk.sleep(0.05)
    return "sigkill_sent"


# ---------------------------------------------------------------------------
# CPU/内存读取（正式监控使用；单测注入假读数，不触碰这里）
# ---------------------------------------------------------------------------

def read_meminfo():
    """返回 /proc/meminfo 的 {key: int 字节}，解析失败返回 {}。"""
    try:
        with open("/proc/meminfo", "r", encoding="utf-8", errors="replace") as handle:
            text = handle.read()
    except OSError:
        return {}
    info = {}
    for line in text.splitlines():
        if ":" not in line:
            continue
        key, value = line.split(":", 1)
        parts = value.strip().split()
        if parts:
            try:
                info[key.strip()] = int(parts[0]) * 1024
            except ValueError:
                continue
    return info


def read_mem_percent():
    info = read_meminfo()
    total = info.get("MemTotal")
    available = info.get("MemAvailable")
    if not total or available is None:
        return None
    used = total - available
    return 100.0 * used / float(total)


def read_cpu_times():
    """读取 /proc/stat 聚合行，返回 (total_ticks, idle_ticks)。"""
    try:
        with open("/proc/stat", "r", encoding="utf-8", errors="replace") as handle:
            first = handle.readline()
    except OSError:
        return None
    parts = first.split()
    if not parts or parts[0] != "cpu":
        return None
    values = []
    for item in parts[1:]:
        try:
            values.append(int(item))
        except ValueError:
            return None
    if len(values) < 4:
        return None
    total = sum(values)
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return total, idle


def process_group_rss_bytes(pgid):
    """按进程组聚合 RSS（页 -> 字节），用于任务树采样估计。"""
    page_size = os.sysconf("SC_PAGE_SIZE")
    total = 0
    try:
        entries = os.listdir("/proc")
    except OSError:
        return 0
    for entry in entries:
        if not entry.isdigit():
            continue
        pid = int(entry)
        try:
            with open("/proc/%d/stat" % pid, "r", encoding="utf-8", errors="replace") as handle:
                data = handle.read()
            close = data.rfind(")")
            if close < 0:
                continue
            fields = data[close + 2:].split()
            if len(fields) < 3:
                continue
            pgrp = int(fields[2])
            if pgrp != int(pgid):
                continue
            with open("/proc/%d/statm" % pid, "r", encoding="utf-8", errors="replace") as handle:
                statm = handle.read().split()
            if len(statm) >= 2:
                total += int(statm[1]) * page_size
        except (OSError, ValueError):
            continue
    return total
