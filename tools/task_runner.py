#!/usr/bin/env python3
"""E0 统一任务入口：build / test / benchmark / run / exec / fixtures。

与 §5.1 命令约定、§5.3 资源调度、§9 证据保存对齐：

- ``build``：全局唯一编译锁（跨 worktree），配置 + 编译，``--parallel`` 最多 4；
- ``test``：按 ``tools/components.json`` 分派 ctest 或标准库 unittest，
  预占 worker 名额（ctest/unittest 并行度受全局 4 名额约束），生成 JUnit XML；
- ``benchmark``：等待编译/测试/服务退出后登记独占窗口，监控继续采样；
- ``run``：长驻服务任务，占名额、登记与回收；
- ``exec``：一次性任务（验收命令等），占 worker 名额、走监控门禁、按 §9 建运行
  目录并封存、退出码原样转发；命令支持 {repo_root}/{build_dir}/{run_dir}/{profile}
  占位符；
- ``fixtures freeze|verify``：生成/校验 ``tests/fixtures/<version>/`` 清单。

每个任务创建 ``artifacts/runs/<run_id>/``，按 §9 封存；退出码转发子进程真实
返回码（信号死亡按 128+signal 折算），基础设施拒绝使用 65/69/70/75。

本模块只使用 CPython 3.8+ 标准库，平台为 Linux。
"""

import argparse
import os
import platform
import re
import selectors
import shutil
import socket
import subprocess
import sys
import tarfile
import traceback
from pathlib import Path

import resource as resource_mod

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools import clock as clock_mod
from tools import fixtures as fixtures_mod
from tools import registry as registry_mod
from tools import rundir as rundir_mod
from tools import runtime

DEFAULT_THREAD_ENV = {
    "OMP_NUM_THREADS": "1",
    "OPENBLAS_NUM_THREADS": "1",
    "MKL_NUM_THREADS": "1",
}

UNTRACKED_SOURCE_EXTENSIONS = (
    ".py", ".pyi", ".c", ".cc", ".cpp", ".cxx",
    ".h", ".hh", ".hpp", ".hxx", ".cmake", ".json",
    ".ts", ".tsx", ".js", ".mjs", ".cjs", ".css", ".html",
    ".sh", ".toml", ".yaml", ".yml",
)
UNTRACKED_SOURCE_NAMES = ("CMakeLists.txt",)
UNTRACKED_EXCLUDED_DIRS = (
    ".git", ".cache", "__pycache__", "node_modules", "dist", "build",
    "artifacts", "reports", "coverage",
)
UNTRACKED_MAX_BYTES = 5 * 1024 * 1024
INPUT_COPY_MAX_BYTES = 10 * 1024 * 1024
QUEUE_POLL_S = 0.5
CHILD_POLL_S = 0.2
HEARTBEAT_RECHECK_S = 1.0
BENCHMARK_METRICS_NAME = "benchmark_metrics.csv"
RUN_ID_PATTERN = re.compile(r"^\d{8}T\d{12}Z_[A-Za-z0-9_.-]+_[0-9a-f]{8}$")


class ConfigError(RuntimeError):
    """参数、组件注册表或命令模板错误。"""


class ChildOutcome(object):
    def __init__(self):
        self.returncode = None
        self.monitor_lost = False
        self.error = None
        self.start_ticks = None
        self.pgid = None
        self.step = None


# ---------------------------------------------------------------------------
# 组件注册表与命令构造
# ---------------------------------------------------------------------------

def load_components(repo_root):
    path = Path(repo_root) / "tools" / "components.json"
    try:
        data = runtime.read_json(path, default=None, strict=True)
    except ValueError as exc:
        raise ConfigError("components.json 不是有效 JSON: %s" % exc)
    if not isinstance(data, dict) or not isinstance(data.get("components"), dict):
        raise ConfigError("components.json 缺失或结构错误: %s" % path)
    return data


def resolve_component(components, name):
    entry = (components.get("components") or {}).get(name)
    if not isinstance(entry, dict):
        raise ConfigError("未注册组件 '%s'（见 tools/components.json）" % name)
    return entry


def normalize_parallel(value):
    try:
        value = int(value)
    except (TypeError, ValueError):
        raise ConfigError("--parallel 必须为整数")
    if value < 1:
        raise ConfigError("--parallel 至少为 1")
    if value > runtime.DEFAULT_MAX_PARALLEL:
        sys.stderr.write("警告: --parallel %d 超过全局上限，按 %d 执行\n"
                         % (value, runtime.DEFAULT_MAX_PARALLEL))
        return runtime.DEFAULT_MAX_PARALLEL
    return value


def normalize_workers(value, cap):
    try:
        value = int(value)
    except (TypeError, ValueError):
        raise ConfigError("--workers 必须为整数")
    if value < 1:
        raise ConfigError("--workers 至少为 1")
    if value > cap:
        raise ConfigError("--workers %d 超过全局上限 %d；请拆分任务或排队运行"
                          % (value, cap))
    return value


def normalize_retry_of(value):
    """校验 ``--retry-of <run_id>``；None 表示非重试。"""
    if value is None or value == "":
        return None
    text = str(value).strip()
    if not RUN_ID_PATTERN.match(text):
        raise ConfigError(
            "--retry-of 需要形如 YYYYMMDDTHHMMSSffffffZ_<component>_<8位小写十六进制> "
            "的 run ID: %r" % value)
    return text


def build_dir_for(args, repo_root, profile):
    override = getattr(args, "build_dir", None)
    if override:
        path = Path(override)
        return path if path.is_absolute() else (Path(repo_root) / path)
    return Path(repo_root) / "build" / profile


def compose_build_commands(repo_root, build_dir, profile, parallel, entry):
    cmake = entry.get("cmake") or {}
    options = cmake.get("options") or {}
    build_type = "Release" if profile == "release" else "Debug"
    configure = [
        "cmake", "-S", str(repo_root), "-B", str(build_dir),
        "-DCMAKE_BUILD_TYPE=%s" % build_type,
    ]
    for key in sorted(options):
        configure.append("-D%s=%s" % (key, options[key]))
    compile_command = ["cmake", "--build", str(build_dir),
                       "--parallel", str(parallel)]
    return [configure, compile_command]


def compose_test_commands(repo_root, build_dir, component, test_spec, workers,
                          layout, ctest_junit_supported):
    runner = (test_spec or {}).get("runner")
    if runner == "ctest":
        label = test_spec.get("label")
        if not label:
            raise ConfigError("组件 %s 的 test.label 缺失" % component)
        command = ["ctest", "-L", str(label), "--output-on-failure",
                   "--no-tests=error", "--parallel", str(workers)]
        if ctest_junit_supported:
            command += ["--output-junit", str(layout.junit)]
        return [command]
    if runner == "unittest":
        start_dir = test_spec.get("start_dir", "tools/tests")
        pattern = test_spec.get("pattern", "test_*.py")
        runner_path = Path(repo_root) / "tools" / "junit_runner.py"
        if not runner_path.is_file():
            raise ConfigError("缺少 tools/junit_runner.py")
        command = [
            sys.executable, str(runner_path),
            "--start-dir", str(Path(repo_root) / start_dir),
            "--pattern", pattern,
            "--junit", str(layout.junit),
        ]
        return [command]
    raise ConfigError("组件 %s 未配置可用的 test.runner（ctest/unittest）" % component)


def compose_template_command(template, repo_root, build_dir, run_dir, profile=None):
    if not isinstance(template, list) or not template:
        raise ConfigError("命令模板必须是非空字符串数组")
    mapping = {
        "{repo_root}": str(repo_root),
        "{build_dir}": str(build_dir),
        "{run_dir}": str(run_dir),
        "{profile}": str(profile or "release"),
    }
    result = []
    for raw in template:
        text = str(raw)
        for key, value in mapping.items():
            text = text.replace(key, value)
        if "{" in text or "}" in text:
            raise ConfigError("命令模板含未知占位符: %s" % text)
        result.append(text)
    return result


def normalize_exec_command(raw):
    """校验 ``exec -- <command> [args...]``；剥离开头的 ``--``。"""
    command = [str(part) for part in (raw or [])]
    while command and command[0] == "--":
        command.pop(0)
    if not command:
        raise ConfigError("exec 需要 '-- <command> [args...]'（命令不得为空）")
    return command


def expand_placeholders(command, mapping):
    """按 mapping 替换命令中的已知占位符；未知花括号保持原样（不做模板报错）。"""
    result = []
    for raw in command:
        text = str(raw)
        for key, value in mapping.items():
            text = text.replace(key, value)
        result.append(text)
    return result


def placeholder_mapping(repo_root, build_dir, run_dir, profile=None,
                        component_dir=None):
    mapping = {
        "{repo_root}": str(repo_root),
        "{build_dir}": str(build_dir),
        "{run_dir}": str(run_dir),
        "{profile}": str(profile or "release"),
    }
    if component_dir is not None:
        mapping["{dir}"] = str(component_dir)
        mapping["{component_dir}"] = str(component_dir)
    return mapping


def expand_exec_placeholders(command, repo_root, build_dir, run_dir, profile=None):
    """替换 exec 命令中的已知占位符；未知花括号保持原样。"""
    return expand_placeholders(
        command, placeholder_mapping(repo_root, build_dir, run_dir, profile))


def expand_component_command(command, repo_root, build_dir, run_dir, profile=None,
                             component_dir=None):
    """替换组件命令占位符（含 ``{dir}``/``{component_dir}``）。"""
    if not isinstance(command, list) or not command:
        raise ConfigError("命令必须是非空字符串数组")
    return expand_placeholders(
        command, placeholder_mapping(repo_root, build_dir, run_dir, profile,
                                     component_dir))


def npm_component_dir(entry, repo_root):
    rel = entry.get("dir")
    if not rel:
        raise ConfigError("npm 组件缺少 dir 字段")
    path = Path(repo_root) / str(rel)
    if not path.is_dir():
        raise ConfigError("npm 组件目录不存在: %s（由组件代理交付后可用）" % path)
    return path


def component_source_dir(entry, repo_root, component):
    """组件源目录：条目含 ``dir`` 时以其为准，否则按约定 ``Src/<id>``。"""
    rel = entry.get("dir") or os.path.join("Src", str(component))
    path = Path(repo_root) / str(rel)
    if not path.is_dir():
        raise ConfigError("组件源目录不存在: %s" % path)
    return path


def touch_component_sources(component_dir):
    """刷新组件目录内所有普通文件的 mtime（内容不变）；返回文件数。"""
    count = 0
    for full in runtime.list_relative_files(component_dir):
        try:
            os.utime(str(full), None)
            count += 1
        except OSError:
            continue
    return count


def compose_npm_build_commands(entry, component_dir, repo_root, build_dir,
                               run_dir, profile=None):
    """npm 构建序列：node_modules 缺失时先 install（有 lockfile 且配置了
    install_ci 时用 ci），再 build。"""
    build_command = entry.get("build")
    if not isinstance(build_command, list) or not build_command:
        raise ConfigError("npm 组件缺少 build 命令")
    commands = []
    node_modules = component_dir / "node_modules"
    lockfile = component_dir / "package-lock.json"
    if not node_modules.exists():
        if lockfile.is_file() and entry.get("install_ci"):
            install = entry.get("install_ci")
        else:
            install = entry.get("install")
        if not isinstance(install, list) or not install:
            raise ConfigError("npm 组件缺少 install 命令（node_modules 缺失时必需）")
        commands.append(expand_component_command(
            install, repo_root, build_dir, run_dir, profile, component_dir))
    commands.append(expand_component_command(
        build_command, repo_root, build_dir, run_dir, profile, component_dir))
    return commands


def compose_npm_test_commands(entry, component_dir, repo_root, build_dir,
                              run_dir, profile=None):
    test_command = entry.get("test")
    if not isinstance(test_command, list) or not test_command:
        raise ConfigError("npm 组件缺少 test 命令")
    return [expand_component_command(
        test_command, repo_root, build_dir, run_dir, profile, component_dir)]


def compose_npm_service_commands(entry, component_dir, repo_root, build_dir,
                                 run_dir, profile=None):
    service = entry.get("service") or {}
    command = service.get("command")
    if not isinstance(command, list) or not command:
        raise ConfigError("npm 组件缺少 service.command")
    return [expand_component_command(
        command, repo_root, build_dir, run_dir, profile, component_dir)]


def ctest_supports_junit():
    try:
        proc = subprocess.run(["ctest", "--help"], stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT, timeout=30)
    except (OSError, subprocess.SubprocessError):
        return False
    return "--output-junit" in proc.stdout.decode("utf-8", "replace")


def resolve_commands(spec, layout, build_dir):
    commands = spec.get("commands")
    if callable(commands):
        commands = commands(layout, build_dir)
    if not isinstance(commands, list) or not commands:
        raise ConfigError("没有可执行的命令（检查 tools/components.json）")
    for command in commands:
        if not isinstance(command, list) or not command:
            raise ConfigError("命令必须是字符串数组: %r" % (command,))
    return commands


def build_child_env(overrides, no_default):
    env = dict(os.environ)
    if not no_default:
        for key, value in DEFAULT_THREAD_ENV.items():
            env[key] = value
    for item in overrides or []:
        if "=" not in item:
            raise ConfigError("--thread-env 需要 KEY=VALUE 形式: %s" % item)
        key, value = item.split("=", 1)
        env[key.strip()] = value
    return env


def inject_context_env(env, run_id, run_dir, task_id, component, profile, build_dir):
    """向子进程注入 PLANNER_* 运行上下文（组件基准/服务按契约读取）。"""
    result = dict(env)
    result.update({
        "PLANNER_RUN_ID": str(run_id),
        "PLANNER_RUN_DIR": str(run_dir),
        "PLANNER_TASK_ID": str(task_id),
        "PLANNER_COMPONENT": str(component),
        "PLANNER_PROFILE": str(profile),
        "PLANNER_BUILD_DIR": str(build_dir),
    })
    return result


# ---------------------------------------------------------------------------
# 仓库事实与证据快照
# ---------------------------------------------------------------------------

def git_run(repo_root, argv, binary=False):
    try:
        proc = subprocess.run(argv, cwd=str(repo_root), stdout=subprocess.PIPE,
                              stderr=subprocess.DEVNULL, timeout=60)
    except (OSError, subprocess.SubprocessError):
        return None
    if proc.returncode != 0:
        return None
    if binary:
        return proc.stdout
    return proc.stdout.decode("utf-8", "replace")


def git_facts(repo_root):
    commit = git_run(repo_root, ["git", "rev-parse", "HEAD"])
    porcelain = git_run(repo_root, ["git", "status", "--porcelain"]) or ""
    return {
        "commit": commit.strip() if commit else None,
        "porcelain_dirty": bool(porcelain.strip()),
    }


def dependency_versions(repo_root):
    lock = runtime.read_json(Path(repo_root) / "dependencies.lock.json", default=None)
    if not isinstance(lock, dict):
        return {}
    deps = lock.get("dependencies")
    if not isinstance(deps, dict):
        return {}
    result = {}
    for name in sorted(deps):
        item = deps[name]
        if not isinstance(item, dict):
            continue
        result[name] = {
            "version": item.get("version"),
            "commit": item.get("commit"),
            "status": item.get("status"),
        }
    return result


def hardware_facts():
    meminfo = runtime.read_meminfo()
    return {
        "platform": platform.platform(),
        "machine": platform.machine(),
        "cpu_count": os.cpu_count(),
        "mem_total_bytes": meminfo.get("MemTotal"),
        "python": platform.python_version(),
        "hostname": socket.gethostname(),
    }


def _is_untracked_snapshot_candidate(rel):
    """未跟踪源快照策展：收录源码/脚本/前端类型，排除派生目录。"""
    parts = Path(rel).parts
    if not parts:
        return False
    if any(part in UNTRACKED_EXCLUDED_DIRS for part in parts[:-1]):
        return False
    name = parts[-1]
    if name in UNTRACKED_SOURCE_NAMES:
        return True
    return Path(rel).suffix.lower() in UNTRACKED_SOURCE_EXTENSIONS


def filter_untracked_candidates(relatives):
    return [rel for rel in relatives if _is_untracked_snapshot_candidate(rel)]


def capture_source_snapshot(repo_root, layout):
    """保存 source.patch 与 source_untracked.tar.gz（存在时）。"""
    result = {
        "patch_bytes": 0,
        "untracked_files": 0,
        "untracked_archive": False,
        "dirty": False,
    }
    patch = git_run(repo_root, ["git", "diff", "--binary", "HEAD"], binary=True)
    patch_path = layout.child("source.patch")
    if patch is None:
        runtime.atomic_write_bytes(patch_path, b"")
        result["patch_error"] = "git diff 失败"
    else:
        runtime.atomic_write_bytes(patch_path, patch)
        result["patch_bytes"] = len(patch)
    untracked_text = git_run(repo_root, ["git", "ls-files", "--others",
                                         "--exclude-standard"]) or ""
    untracked = [line.strip() for line in untracked_text.splitlines() if line.strip()]
    selected = []
    for rel in filter_untracked_candidates(untracked):
        full = Path(repo_root) / rel
        try:
            if full.stat().st_size > UNTRACKED_MAX_BYTES:
                continue
        except OSError:
            continue
        selected.append(rel)
    if selected:
        count = rundir_mod.make_source_archive(layout, repo_root, selected)
        result["untracked_files"] = count
        result["untracked_archive"] = count > 0
    result["dirty"] = bool(result["patch_bytes"]) or result["untracked_files"] > 0
    return result


def capture_inputs(inputs, repo_root, layout):
    """复制小输入并记录 SHA256；大输入按 §9 记录外部归档引用。"""
    sha = {}
    for raw in inputs or []:
        path = Path(raw)
        if not path.is_absolute():
            path = Path(repo_root) / path
        if not path.exists():
            raise ConfigError("输入不存在: %s" % raw)
        if path.is_file():
            digest = runtime.sha256_file(path)
            size = path.stat().st_size
            if size <= INPUT_COPY_MAX_BYTES:
                target = layout.inputs / runtime.safe_name(path.name)
                shutil.copyfile(str(path), str(target))
                sha["inputs/%s" % target.name] = digest
            else:
                # §9：>10 MiB 使用外部归档引用；原路径记录在 config.inputs
                sha["external/%s" % runtime.safe_name(path.name)] = digest
        elif path.is_dir():
            archive_path = layout.inputs / (runtime.safe_name(path.name) + ".tar.gz")
            with tarfile.open(str(archive_path), "w:gz") as archive:
                for full in runtime.list_relative_files(path):
                    rel = full.relative_to(path).as_posix()
                    archive.add(str(full), arcname=rel)
            sha["inputs/%s" % archive_path.name] = runtime.sha256_file(archive_path)
        else:
            raise ConfigError("不支持的输入类型: %s" % raw)
    if sha:
        runtime.atomic_write_json(layout.inputs / "input_sha256.json", sha)
    return sha


# ---------------------------------------------------------------------------
# 子进程执行
# ---------------------------------------------------------------------------

def _write_chunk(target, chunk):
    target.write(chunk)
    target.flush()


def _tee(stream_name, chunk, echo):
    if echo:
        try:
            stream = sys.stdout.buffer if stream_name == "stdout" else sys.stderr.buffer
            stream.write(chunk)
            stream.flush()
        except (OSError, ValueError):
            pass


def _read_chunk(fileobj):
    try:
        return os.read(fileobj.fileno(), 65536)
    except OSError:
        return b""


def _drain(selector, echo, out_f, err_f):
    idle = 0
    while idle < 2:
        events = selector.select(timeout=0.5)
        if not events:
            idle += 1
            continue
        idle = 0
        for key, _ in events:
            chunk = _read_chunk(key.fileobj)
            name, stream_file = key.data
            if chunk:
                _write_chunk(stream_file, chunk)
                _tee(name, chunk, echo)
            else:
                try:
                    selector.unregister(key.fileobj)
                except (KeyError, ValueError):
                    pass


def run_child(command, cwd, env, layout, clk, on_spawn=None, heartbeat_check=None,
              step_index=0, echo=True):
    """在独立进程组中运行一个命令，输出同时写日志与终端。"""
    outcome = ChildOutcome()
    outcome.step = step_index
    with open(str(layout.stdout), "ab") as out_f, open(str(layout.stderr), "ab") as err_f:
        proc = subprocess.Popen(
            command, cwd=cwd, env=env,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            start_new_session=True,
        )
        try:
            pgid = os.getpgid(proc.pid)
        except OSError:
            pgid = proc.pid
        outcome.pgid = pgid
        outcome.start_ticks = runtime.process_start_ticks(proc.pid)
        selector = selectors.DefaultSelector()
        last_heartbeat_check = clk.monotonic()
        try:
            selector.register(proc.stdout, selectors.EVENT_READ, ("stdout", out_f))
            selector.register(proc.stderr, selectors.EVENT_READ, ("stderr", err_f))
            if on_spawn is not None:
                on_spawn(proc, pgid, outcome.start_ticks)
            while True:
                for key, _ in selector.select(timeout=CHILD_POLL_S):
                    chunk = _read_chunk(key.fileobj)
                    name, stream_file = key.data
                    if chunk:
                        _write_chunk(stream_file, chunk)
                        _tee(name, chunk, echo)
                    else:
                        try:
                            selector.unregister(key.fileobj)
                        except (KeyError, ValueError):
                            pass
                returncode = proc.poll()
                if returncode is not None:
                    _drain(selector, echo, out_f, err_f)
                    outcome.returncode = returncode
                    return outcome
                if heartbeat_check is not None:
                    if clk.monotonic() - last_heartbeat_check >= HEARTBEAT_RECHECK_S:
                        last_heartbeat_check = clk.monotonic()
                        reason = heartbeat_check()
                        if reason:
                            outcome.monitor_lost = True
                            outcome.error = reason
                            runtime.stop_process_tree(proc.pid, pgid, outcome.start_ticks,
                                                      clock=clk)
                            try:
                                proc.wait(timeout=10)
                            except subprocess.TimeoutExpired:
                                pass
                            outcome.returncode = proc.returncode
                            return outcome
        except BaseException:
            if proc.poll() is None:
                runtime.stop_process_tree(proc.pid, pgid, outcome.start_ticks, clock=clk)
                try:
                    proc.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    pass
            raise
        finally:
            selector.close()
            try:
                proc.stdout.close()
                proc.stderr.close()
            except OSError:
                pass


def exit_code_from_returncode(returncode):
    if returncode is None:
        return runtime.EXIT_SOFTWARE
    returncode = int(returncode)
    if returncode < 0:
        return 128 + min(127, -returncode)
    return returncode


def _monitor_reason(detail):
    detail = str(detail) if detail is not None else ""
    if not detail or detail == "monitor_lost":
        return "monitor_lost"
    return "monitor_lost:%s" % detail


def acquire_with_gate(lock, clk, timeout, gate=None, on_wait=None, poll=QUEUE_POLL_S):
    deadline = None
    if timeout and float(timeout) > 0:
        deadline = clk.monotonic() + float(timeout)
    while True:
        if lock.acquire(timeout=poll):
            return True
        if gate is not None:
            reason = gate()
            if reason:
                raise registry_mod.GateFailure(reason)
        if deadline is not None and clk.monotonic() >= deadline:
            return False
        if on_wait is not None:
            on_wait()


# ---------------------------------------------------------------------------
# 任务生命周期
# ---------------------------------------------------------------------------

def base_manifest(args, kind, task_id, run_id, component, repo_root, profile,
                  workers, clk):
    git = git_facts(repo_root)
    return {
        "schema_version": runtime.SCHEMA_VERSION,
        "type": "planner_run_manifest",
        "run_id": run_id,
        "task_id": task_id,
        "component": component,
        "backend": getattr(args, "backend", None) or "default",
        "task_kind": kind,
        "command_argv": None,
        "cwd": str(repo_root),
        "started_at": runtime.now_iso(clk),
        "finished_at": None,
        "status": "running",
        "exit_code": None,
        "git_commit": git.get("commit"),
        "dirty": None,
        "dependency_versions": dependency_versions(repo_root),
        "hardware": hardware_facts(),
        "build_profile": profile,
        "config_sha256": None,
        "input_sha256": {},
        "seed": getattr(args, "seed", None),
        "worker_count": workers,
        "artifact_sha256": None,
        "retry_of": getattr(args, "retry_of", None),
        "failure_reason": None,
        "monitor_required": not bool(getattr(args, "allow_unmonitored", False)),
    }


def compose_config(args, repo_root, kind, component, entry, profile, build_dir,
                   workers, exclusive, commands, gate_enabled):
    return {
        "schema_version": runtime.SCHEMA_VERSION,
        "task_kind": kind,
        "component": component,
        "component_registered": bool(entry),
        "component_spec": entry,
        "profile": profile,
        "build_dir": str(build_dir),
        "workers": workers,
        "parallel": getattr(args, "parallel", None),
        "suite": getattr(args, "suite", None),
        "exclusive_window": bool(exclusive),
        "queue_timeout_s": float(getattr(args, "queue_timeout", 3600.0) or 0.0),
        "command": commands,
        "cwd": str(repo_root),
        "environment_overrides": list(getattr(args, "thread_env", None) or []),
        "no_default_thread_env": bool(getattr(args, "no_default_thread_env", False)),
        "default_thread_env": dict(DEFAULT_THREAD_ENV),
        "monitor_gate_enabled": bool(gate_enabled),
        "inputs": list(getattr(args, "input", None) or []),
        "seed": getattr(args, "seed", None),
        "retry_of": getattr(args, "retry_of", None),
        "defaults": {
            "worker_cap": runtime.DEFAULT_WORKER_CAP,
            "max_parallel": runtime.DEFAULT_MAX_PARALLEL,
            "sample_interval_s": runtime.SAMPLE_INTERVAL_S,
            "threshold_percent": runtime.THRESHOLD_PERCENT,
            "monitor_heartbeat_max_age_s": runtime.MONITOR_HEARTBEAT_MAX_AGE_S,
        },
    }


def peak_rss_from_resources(layout):
    values = []
    for row in rundir_mod.read_csv_rows(layout.resources):
        raw = row.get("task_tree_rss_bytes")
        if raw is None or raw == "":
            continue
        try:
            values.append(int(float(raw)))
        except (TypeError, ValueError):
            continue
    return max(values) if values else None


def merge_benchmark_metrics(layout, event=None):
    """把基准二进制的 ``outputs/benchmark_metrics.csv`` 并入运行目录 metrics.csv。

    契约见 tools/README.md“基准输出契约”：组件按 §9 固定列逐 trial 写出，
    失败/中断记录同样保留；缺失文件不阻塞任务，只记录事件。
    """
    source = layout.outputs / BENCHMARK_METRICS_NAME
    if not source.is_file():
        if event is not None:
            event("benchmark_metrics_missing", {"expected": str(source)})
        return {"merged": 0, "missing": True}
    rows = rundir_mod.read_csv_rows(source)
    merged = 0
    for row in rows:
        if not isinstance(row, dict):
            continue
        rundir_mod.append_metric_row(layout, row)
        merged += 1
    if event is not None:
        event("benchmark_metrics_merged", {"rows": merged, "source": str(source)})
    return {"merged": merged, "missing": False}


def collect_component_junit(layout, source, event=None):
    """把组件测试命令产出的 JUnit XML 收进运行目录（尽力而为）。

    - 命令已直接写 ``{run_dir}/junit.xml`` 时保留原文件；
    - 配置了 ``test_junit``（相对组件 dir）且文件存在时复制；
    - 都不存在时如实记录 ``junit_missing``，不伪造。
    """
    if layout.junit.exists():
        if event is not None:
            event("junit_present", {"path": str(layout.junit)})
        return "already_present"
    source = Path(source)
    if source.is_file():
        shutil.copyfile(str(source), str(layout.junit))
        if event is not None:
            event("junit_collected", {"source": str(source)})
        return "copied"
    if event is not None:
        event("junit_missing", {"expected": str(source)})
    return "missing"


def summary_counts(layout, status):
    counts = {"task_status": status}
    rows = rundir_mod.read_csv_rows(layout.metrics)
    counts["metrics_rows"] = len(rows)
    junit = rundir_mod.parse_junit_counts(layout.junit)
    if junit:
        counts["junit_tests"] = junit["tests"]
        counts["junit_failures"] = junit["failures"]
        counts["junit_errors"] = junit["errors"]
        counts["junit_skipped"] = junit["skipped"]
    return counts


def summary_notes(snapshot, spec):
    notes = []
    if snapshot:
        notes.append("source.patch 字节数: %s" % snapshot.get("patch_bytes"))
        notes.append("未跟踪源文件快照数: %s" % snapshot.get("untracked_files"))
        if snapshot.get("patch_error"):
            notes.append("source.patch 生成异常: %s" % snapshot["patch_error"])
    if spec.get("exclusive"):
        notes.append("本任务在独占工作负载窗口内运行。")
    return notes


def execute_task(args, kind, repo_root, components, spec):
    clk = clock_mod.Clock()
    repo_root = Path(repo_root)
    runtime_dir_path = runtime.runtime_dir(repo_root)
    runs_root = runtime.runs_dir(repo_root)
    allow_unmonitored = bool(getattr(args, "allow_unmonitored", False))
    queue_timeout = float(getattr(args, "queue_timeout", 3600.0) or 0.0)
    profile = getattr(args, "profile", "release") or "release"
    build_dir = build_dir_for(args, repo_root, profile)
    component = args.component
    entry = spec["entry"]
    workers = int(spec.get("workers") or 0)
    task_id = "%s:%s" % (kind, component)

    gate_reason = None if allow_unmonitored else registry_mod.heartbeat_state(
        runtime_dir_path, clk)
    if gate_reason:
        sys.stderr.write(
            "拒绝派发（%s）：资源监控心跳缺失或失效。先启动 "
            "python3 tools/resource_monitor.py；E0 引导期可用 --allow-unmonitored "
            "做无负载自检。\n" % gate_reason)
        return runtime.EXIT_UNAVAILABLE

    reg = registry_mod.Registry(
        runtime_dir_path,
        worker_cap=int(components.get("worker_cap", runtime.DEFAULT_WORKER_CAP)))
    state = reg.read_dispatch_state()
    if state.get("paused"):
        sys.stderr.write("拒绝派发：资源监控已暂停派发（%s）；恢复后以新 run ID 重试。\n"
                         % (state.get("reason") or "阈值超限"))
        return runtime.EXIT_TEMPFAIL

    try:
        run_id, run_dir = rundir_mod.create_run_dir(
            runs_root, component, retry_of=getattr(args, "retry_of", None), clock=clk)
    except rundir_mod.RunDirError as exc:
        sys.stderr.write("创建运行目录失败: %s\n" % exc)
        return runtime.EXIT_TEMPFAIL

    layout = rundir_mod.RunLayout(run_dir).ensure_dirs()
    started_mono = clk.monotonic()
    manifest = base_manifest(args, kind, task_id, run_id, component, repo_root,
                             profile, workers, clk)
    runtime.atomic_write_json(layout.manifest, manifest)

    snapshot = None
    input_sha = {}
    cpu_ms = None
    steps_report = []
    status = "failed"
    failure_reason = None
    exit_code = runtime.EXIT_SOFTWARE
    reservation = None
    compile_lock = None

    def gate():
        if allow_unmonitored:
            return None
        return registry_mod.heartbeat_state(runtime_dir_path, clk)

    def event(name, details=None):
        try:
            rundir_mod.record_event(layout, clk, name, task_id=task_id,
                                    component=component, details=details)
        except (OSError, ValueError):
            pass

    wait_stamps = {}

    def wait_event(key, name, details=None):
        now = clk.monotonic()
        last = wait_stamps.get(key)
        if last is None or now - last >= 10.0:
            wait_stamps[key] = now
            event(name, details)

    def on_wait(reason):
        wait_event("queued", "queued", {"reason": reason, "workers": workers})
        current = gate()
        if current:
            raise registry_mod.GateFailure(current)

    try:
        snapshot = capture_source_snapshot(repo_root, layout)
        env = build_child_env(getattr(args, "thread_env", None),
                              bool(getattr(args, "no_default_thread_env", False)))
        env = inject_context_env(env, run_id, run_dir, task_id, component,
                                 profile, build_dir)
        input_sha = capture_inputs(getattr(args, "input", None) or [], repo_root, layout)
        git = git_facts(repo_root)
        commands = resolve_commands(spec, layout, build_dir)
        config = compose_config(args, repo_root, kind, component, entry, profile,
                                build_dir, workers, spec.get("exclusive", False),
                                commands, gate_enabled=not allow_unmonitored)
        manifest["command_argv"] = commands
        manifest["git_commit"] = git.get("commit")
        manifest["dirty"] = bool(snapshot.get("dirty")) or git.get("porcelain_dirty")
        manifest["input_sha256"] = input_sha
        manifest = rundir_mod.init_run(layout, manifest, config, clock=clk)
        event("run_created", {"profile": profile, "workers": workers,
                              "exclusive_window": bool(spec.get("exclusive"))})

        reservation = reg.reserve(
            kind=kind, component=component, workers=workers, run_id=run_id,
            run_dir=str(run_dir), task_id=run_id, queue_timeout=queue_timeout,
            clock=clk, on_wait=on_wait, gate=gate)
        event("registered", {"registry_task_id": run_id, "workers": workers})

        if spec.get("compile_lock"):
            compile_lock = reg.compile_lock()
            acquired = acquire_with_gate(
                compile_lock, clk, queue_timeout, gate,
                on_wait=lambda: wait_event("compile_lock",
                                           "waiting_for_compile_lock"))
            if not acquired:
                raise registry_mod.QueueTimeout("等待全局编译锁超时")
            event("compile_lock_acquired")

        prepare = spec.get("prepare")
        if prepare is not None:
            prepare_result = prepare(layout, build_dir)
            event("prepare_done",
                  prepare_result if isinstance(prepare_result, dict) else {})

        rusage_before = resource_mod.getrusage(resource_mod.RUSAGE_CHILDREN)
        step_failed = False
        for index, step_command in enumerate(commands):
            event("step_started", {"index": index, "argv": step_command})
            outcome = run_child(
                step_command, str(spec.get("cwd") or repo_root), env, layout, clk,
                on_spawn=lambda proc, pgid, ticks: reg.update_process(
                    run_id, proc.pid, pgid, ticks),
                heartbeat_check=gate, step_index=index)
            steps_report.append({
                "index": index,
                "argv": step_command,
                "returncode": outcome.returncode,
                "monitor_lost": outcome.monitor_lost,
            })
            event("step_finished", {"index": index,
                                    "returncode": outcome.returncode,
                                    "monitor_lost": outcome.monitor_lost})
            if outcome.monitor_lost:
                status = "failed"
                failure_reason = _monitor_reason(outcome.error)
                exit_code = runtime.EXIT_TEMPFAIL
                step_failed = True
                break
            if outcome.returncode != 0:
                status = "failed"
                failure_reason = "step_%d_exit_%s" % (index, outcome.returncode)
                exit_code = exit_code_from_returncode(outcome.returncode)
                step_failed = True
                break
        rusage_after = resource_mod.getrusage(resource_mod.RUSAGE_CHILDREN)
        cpu_ms = 1000.0 * max(0.0,
                              (rusage_after.ru_utime - rusage_before.ru_utime) +
                              (rusage_after.ru_stime - rusage_before.ru_stime))

        if kind == "benchmark":
            merge_benchmark_metrics(layout, event=event)
        junit_source = spec.get("junit_source")
        if junit_source is not None:
            collect_component_junit(layout, junit_source, event=event)

        stop_reason = registry_mod.read_stop_reason(runtime_dir_path, run_id)
        if stop_reason is not None:
            status = stop_reason.get("status") or "resource_stopped"
            failure_reason = (stop_reason.get("failure_reason") or
                              stop_reason.get("reason") or failure_reason)
            if exit_code == 0:
                exit_code = runtime.EXIT_TEMPFAIL
        elif (not step_failed and steps_report and
              len(steps_report) == len(commands) and
              all(item.get("returncode") == 0 for item in steps_report)):
            status = "success"
            exit_code = runtime.EXIT_OK
        elif not step_failed:
            status = "failed"
            failure_reason = failure_reason or "no_steps_executed"
            exit_code = runtime.EXIT_SOFTWARE
    except registry_mod.GateFailure as exc:
        status = "failed"
        failure_reason = _monitor_reason(str(exc))
        exit_code = runtime.EXIT_TEMPFAIL
        event("gate_failed", {"reason": str(exc)})
    except registry_mod.DispatchPaused as exc:
        status = "failed"
        failure_reason = "dispatch_paused:%s" % (exc,)
        exit_code = runtime.EXIT_TEMPFAIL
        event("dispatch_paused", {"reason": str(exc)})
    except registry_mod.QueueTimeout as exc:
        status = "failed"
        failure_reason = "queue_timeout"
        exit_code = runtime.EXIT_TEMPFAIL
        event("queue_timeout", {"reason": str(exc)})
    except (ConfigError, fixtures_mod.FixtureError) as exc:
        status = "failed"
        failure_reason = "invalid_config:%s" % exc
        exit_code = runtime.EXIT_DATA
        event("error", {"error": str(exc)})
    except KeyboardInterrupt:
        status = "failed"
        failure_reason = "interrupted"
        exit_code = 130
        event("interrupted", {"reason": "KeyboardInterrupt"})
    except Exception as exc:  # noqa: BLE001 - 继续封存证据并返回内部错误
        traceback.print_exc()
        status = "failed"
        failure_reason = "internal_error:%s" % exc
        exit_code = runtime.EXIT_SOFTWARE
        event("error", {"error": str(exc)})
    finally:
        if compile_lock is not None:
            compile_lock.release()
        if reservation is not None:
            reservation.release()
        wall_ms = (clk.monotonic() - started_mono) * 1000.0
        peak_rss = peak_rss_from_resources(layout)
        try:
            if not layout.config.exists():
                runtime.atomic_write_json(layout.config, {
                    "schema_version": runtime.SCHEMA_VERSION,
                    "task_kind": kind,
                    "component": component,
                    "note": "生效参数在写入前失败，见 manifest.failure_reason",
                    "failure_reason": failure_reason,
                })
            for name in (rundir_mod.STDOUT_NAME, rundir_mod.STDERR_NAME,
                         rundir_mod.EVENTS_NAME):
                target = layout.child(name)
                if not target.exists():
                    target.write_bytes(b"")
            rundir_mod.append_metric_row(layout, {
                "run_id": run_id,
                "case_id": "task_summary",
                "repeat_id": "",
                "cache_mode": "",
                "seed": getattr(args, "seed", None) if getattr(args, "seed", None) is not None else "",
                "status": status,
                "failure_reason": failure_reason or "",
                "preprocess_ms": "",
                "wall_ms": "%.3f" % wall_ms,
                "cpu_ms": "" if cpu_ms is None else "%.3f" % cpu_ms,
                "peak_rss_bytes": "" if peak_rss is None else peak_rss,
                "valid": "true" if status == "success" else "false",
            })
            final_fields = {
                "status": status,
                "exit_code": exit_code,
                "failure_reason": failure_reason,
                "finished_at": runtime.now_iso(clk),
                "steps": steps_report,
                "source_snapshot": snapshot if snapshot is not None else {},
                "duration_ms": round(wall_ms, 3),
            }
            summary_manifest = dict(manifest)
            summary_manifest.update(final_fields)
            counts = summary_counts(layout, status)
            rundir_mod.write_task_summary(layout, summary_manifest, counts=counts,
                                          notes=summary_notes(snapshot, spec))
            event("sealed", {"artifact": rundir_mod.ARTIFACTS_NAME})
            rundir_mod.seal_run(
                run_dir, final_fields, clock=clk,
                lock_path=runtime.evidence_lock_path(runtime_dir_path, run_id))
            registry_mod.clear_stop_reason(runtime_dir_path, run_id)
        except Exception as exc:  # noqa: BLE001 - 封存失败必须显式暴露
            sys.stderr.write("封存失败（运行目录保留）: %s\n" % exc)
            exit_code = runtime.EXIT_SOFTWARE
    sys.stdout.write("run_id=%s status=%s exit_code=%s run_dir=%s\n"
                     % (run_id, status, exit_code, run_dir))
    return int(exit_code) if exit_code else runtime.EXIT_OK


# ---------------------------------------------------------------------------
# 子命令
# ---------------------------------------------------------------------------

def cmd_build(args, repo_root, components):
    try:
        entry = resolve_component(components, args.component)
        kind = entry.get("kind")
        build_dir = build_dir_for(args, repo_root, args.profile)
        if kind == "cmake":
            parallel = normalize_parallel(args.parallel)
            commands = compose_build_commands(repo_root, build_dir, args.profile,
                                              parallel, entry)
            prepare = None
            if getattr(args, "force_rebuild", False):
                source_dir = component_source_dir(entry, repo_root, args.component)

                def prepare(layout, current_build_dir, _dir=source_dir):
                    count = touch_component_sources(_dir)
                    return {"action": "force_rebuild",
                            "touched_files": count,
                            "component_dir": str(_dir)}

            spec = {
                "entry": entry,
                "workers": 0,
                "exclusive": False,
                "compile_lock": True,
                "commands": commands,
                "cwd": str(repo_root),
                "prepare": prepare,
            }
        elif kind == "npm":
            component_dir = npm_component_dir(entry, repo_root)
            try:
                build_workers = int(entry.get("build_workers", 1))
            except (TypeError, ValueError):
                raise ConfigError("npm 组件 build_workers 必须为整数")
            if build_workers < 0:
                raise ConfigError("npm 组件 build_workers 不能为负")

            def commands(layout, current_build_dir):
                return compose_npm_build_commands(
                    entry, component_dir, repo_root, current_build_dir,
                    layout.path, args.profile)

            prepare = None
            if getattr(args, "force_rebuild", False):
                def prepare(layout, current_build_dir, _dir=component_dir):
                    count = touch_component_sources(_dir)
                    return {"action": "force_rebuild",
                            "touched_files": count,
                            "component_dir": str(_dir)}

            spec = {
                "entry": entry,
                "workers": build_workers,
                "exclusive": False,
                "compile_lock": True,
                "commands": commands,
                "cwd": str(component_dir),
                "prepare": prepare,
            }
        else:
            raise ConfigError("组件 '%s' 的 kind=%r 不支持 build（支持 cmake/npm）"
                              % (args.component, kind))
    except ConfigError as exc:
        sys.stderr.write("参数错误: %s\n" % exc)
        return runtime.EXIT_DATA
    return execute_task(args, "build", repo_root, components, spec)


def cmd_test(args, repo_root, components):
    try:
        entry = resolve_component(components, args.component)
        kind = entry.get("kind")
        cap = int(components.get("worker_cap", runtime.DEFAULT_WORKER_CAP))
        workers = normalize_workers(args.workers, cap)
        build_dir = build_dir_for(args, repo_root, args.profile)
        if kind == "cmake":
            test_spec = entry.get("test") or {}
            if test_spec.get("runner") != "ctest":
                raise ConfigError("组件 '%s' 的 test.runner 不是 ctest" % args.component)
            if not (build_dir / "CMakeCache.txt").is_file():
                sys.stderr.write(
                    "构建目录未配置: %s；先运行 build --component %s --profile %s\n"
                    % (build_dir, args.component, args.profile))
                return runtime.EXIT_UNAVAILABLE
            junit_ok = ctest_supports_junit()

            def commands(layout, current_build_dir):
                return compose_test_commands(repo_root, current_build_dir,
                                             args.component, test_spec, workers,
                                             layout, junit_ok)

            spec = {
                "entry": entry,
                "workers": workers,
                "exclusive": False,
                "compile_lock": False,
                "commands": commands,
                "cwd": str(build_dir),
            }
        elif kind == "python":
            test_spec = entry.get("test") or {}
            if test_spec.get("runner") != "unittest":
                raise ConfigError("组件 '%s' 的 test.runner 不是 unittest"
                                  % args.component)

            def commands(layout, current_build_dir):
                return compose_test_commands(repo_root, current_build_dir,
                                             args.component, test_spec, workers,
                                             layout, True)

            spec = {
                "entry": entry,
                "workers": workers,
                "exclusive": False,
                "compile_lock": False,
                "commands": commands,
                "cwd": str(repo_root),
            }
        elif kind == "npm":
            component_dir = npm_component_dir(entry, repo_root)
            junit_rel = entry.get("test_junit")
            if junit_rel is not None and not isinstance(junit_rel, str):
                raise ConfigError("npm 组件 test_junit 必须是相对 dir 的路径字符串")

            def commands(layout, current_build_dir):
                return compose_npm_test_commands(entry, component_dir, repo_root,
                                                 current_build_dir, layout.path,
                                                 args.profile)

            spec = {
                "entry": entry,
                "workers": workers,
                "exclusive": False,
                "compile_lock": False,
                "commands": commands,
                "cwd": str(component_dir),
                "junit_source": (component_dir / junit_rel) if junit_rel else None,
            }
        else:
            raise ConfigError("组件 '%s' 的 kind=%r 不支持 test" % (args.component, kind))
    except ConfigError as exc:
        sys.stderr.write("参数错误: %s\n" % exc)
        return runtime.EXIT_DATA
    return execute_task(args, "test", repo_root, components, spec)


def cmd_benchmark(args, repo_root, components):
    try:
        entry = resolve_component(components, args.component)
        suite = args.suite or components.get("default_suite") or "v1"
        suites = entry.get("benchmark_suites") or {}
        suite_spec = suites.get(suite) if isinstance(suites, dict) else None
        if not isinstance(suite_spec, dict) or not suite_spec.get("command"):
            raise ConfigError("组件 '%s' 没有基准套件 '%s'（已注册: %s）"
                              % (args.component, suite,
                                 ", ".join(sorted(suites)) if suites else "无"))
        build_dir = build_dir_for(args, repo_root, args.profile)
    except ConfigError as exc:
        sys.stderr.write("参数错误: %s\n" % exc)
        return runtime.EXIT_DATA

    def commands(layout, current_build_dir):
        return [compose_template_command(suite_spec.get("command"), repo_root,
                                         current_build_dir, layout.path,
                                         profile=args.profile)]

    spec = {
        "entry": entry,
        "workers": 0,
        "exclusive": True,
        "compile_lock": False,
        "commands": commands,
        "cwd": str(repo_root),
    }
    return execute_task(args, "benchmark", repo_root, components, spec)


def cmd_run(args, repo_root, components):
    try:
        entry = resolve_component(components, args.component)
        kind = entry.get("kind")
        service = entry.get("service") or {}
        if not service.get("command"):
            raise ConfigError("组件 '%s' 未注册 service.command" % args.component)
        cap = int(components.get("worker_cap", runtime.DEFAULT_WORKER_CAP))
        try:
            default_workers = int(service.get("workers", 1))
        except (TypeError, ValueError):
            raise ConfigError("service.workers 必须为整数")
        workers = normalize_workers(
            args.workers if args.workers is not None else default_workers, cap)
        build_dir = build_dir_for(args, repo_root, args.profile)
        if kind == "npm":
            component_dir = npm_component_dir(entry, repo_root)

            def commands(layout, current_build_dir):
                return compose_npm_service_commands(
                    entry, component_dir, repo_root, current_build_dir,
                    layout.path, args.profile)

            cwd = str(component_dir)
        else:
            def commands(layout, current_build_dir):
                return [compose_template_command(service.get("command"), repo_root,
                                                 current_build_dir, layout.path,
                                                 profile=args.profile)]

            cwd = str(repo_root)
    except ConfigError as exc:
        sys.stderr.write("参数错误: %s\n" % exc)
        return runtime.EXIT_DATA

    spec = {
        "entry": entry,
        "workers": workers,
        "exclusive": False,
        "compile_lock": False,
        "commands": commands,
        "cwd": cwd,
    }
    return execute_task(args, "run", repo_root, components, spec)


def cmd_exec(args, repo_root, components):
    """一次性任务：登记名额、走监控门禁、建运行目录并按 §9 封存、转发退出码。

    ``--component`` 默认 ``adhoc``；组件未在 components.json 注册时也允许，
    仅作为记录标签（config.component_registered=false）。
    """
    try:
        command = normalize_exec_command(args.exec_command)
        cap = int(components.get("worker_cap", runtime.DEFAULT_WORKER_CAP))
        workers = normalize_workers(args.workers, cap)
        entry = (components.get("components") or {}).get(args.component)
        if not isinstance(entry, dict):
            entry = {}
        build_dir = build_dir_for(args, repo_root, args.profile)
    except ConfigError as exc:
        sys.stderr.write("参数错误: %s\n" % exc)
        return runtime.EXIT_DATA

    def commands(layout, current_build_dir):
        return [expand_exec_placeholders(command, repo_root, current_build_dir,
                                         layout.path, profile=args.profile)]

    spec = {
        "entry": entry,
        "workers": workers,
        "exclusive": False,
        "compile_lock": False,
        "commands": commands,
        "cwd": str(repo_root),
    }
    return execute_task(args, "exec", repo_root, components, spec)


def cmd_fixtures(args, repo_root):
    version = args.version
    fixture_root = Path(repo_root) / "tests" / "fixtures" / version
    if args.action == "freeze":
        try:
            items = fixtures_mod.freeze(fixture_root)
        except fixtures_mod.FixtureError as exc:
            sys.stderr.write("fixtures freeze 失败: %s\n" % exc)
            return runtime.EXIT_DATA
        print("fixtures freeze 完成: %d 个文件 -> %s"
              % (len(items), fixtures_mod.manifest_path(fixture_root)))
        return runtime.EXIT_OK
    result = fixtures_mod.verify(fixture_root)
    if result["ok"]:
        print("fixtures verify 通过: %d 个文件（清单: %s）"
              % (result["checked"], fixtures_mod.manifest_path(fixture_root)))
        return runtime.EXIT_OK
    sys.stderr.write("fixtures verify 失败（%d 个问题）:\n" % len(result["problems"]))
    for problem in result["problems"]:
        sys.stderr.write("  - %s\n" % problem)
    return runtime.EXIT_DATA


# ---------------------------------------------------------------------------
# 命令行
# ---------------------------------------------------------------------------

def add_common_runtime_args(parser):
    parser.add_argument("--allow-unmonitored", action="store_true",
                        help="引导期跳过心跳门禁（禁止用于正式实验；会记录到 config）")
    parser.add_argument("--retry-of", default=None,
                        help="重试来源 run ID（YYYYMMDDTHHMMSSffffffZ_<component>_<8位小写十六进制>；"
                             "写入 manifest.retry_of，不覆盖旧证据）")
    parser.add_argument("--queue-timeout", type=float, default=3600.0,
                        help="等待名额/编译锁/独占窗口的最长秒数；0=无限等待")
    parser.add_argument("--build-dir", default=None,
                        help="覆盖 build/<profile>（默认按接口冻结约定）")
    parser.add_argument("--backend", default=None, help="后端标签（默认 default）")
    parser.add_argument("--seed", type=int, default=None, help="随机种子（适用时）")
    parser.add_argument("--input", action="append", default=[],
                        help="复现输入路径；可重复；小文件复制进 inputs/，大文件记录 SHA256")
    parser.add_argument("--thread-env", action="append", default=[],
                        metavar="KEY=VALUE", help="覆盖默认线程环境变量")
    parser.add_argument("--no-default-thread-env", action="store_true",
                        help="不注入 OMP/OPENBLAS/MKL=1 默认值")


def build_parser():
    parser = argparse.ArgumentParser(
        prog="task_runner.py",
        description="E0 统一任务入口（build/test/benchmark/run/exec/fixtures）")
    sub = parser.add_subparsers(dest="command", required=True)

    build = sub.add_parser("build", help="配置 + 编译（全局唯一编译锁，最多 j4）")
    build.add_argument("--component", required=True)
    build.add_argument("--profile", choices=("release", "debug"), default="release")
    build.add_argument("--parallel", type=int, default=runtime.DEFAULT_MAX_PARALLEL)
    build.add_argument("--force-rebuild", action="store_true",
                       help="构建前刷新组件源目录文件 mtime（内容不变），"
                            "消除陈旧对象竞态；默认关闭")
    add_common_runtime_args(build)

    test = sub.add_parser("test", help="ctest / 标准库 unittest，生成 JUnit XML")
    test.add_argument("--component", required=True)
    test.add_argument("--workers", type=int, default=1)
    test.add_argument("--profile", choices=("release", "debug"), default="release")
    add_common_runtime_args(test)

    benchmark = sub.add_parser("benchmark", help="独占工作负载窗口的正式基准")
    benchmark.add_argument("--component", required=True)
    benchmark.add_argument("--suite", default=None)
    benchmark.add_argument("--profile", choices=("release", "debug"), default="release")
    add_common_runtime_args(benchmark)

    run = sub.add_parser("run", help="长驻服务任务（占 worker 名额）")
    run.add_argument("--component", required=True)
    run.add_argument("--workers", type=int, default=None)
    run.add_argument("--profile", choices=("release", "debug"), default="release")
    add_common_runtime_args(run)

    exec_parser = sub.add_parser(
        "exec", help="一次性任务：登记名额、监控门禁、§9 证据封存、退出码转发")
    exec_parser.add_argument("--component", default="adhoc",
                             help="组件标签（默认 adhoc；未注册组件也允许，仅作记录）")
    exec_parser.add_argument("--workers", type=int, default=1)
    exec_parser.add_argument("--profile", choices=("release", "debug"),
                             default="release")
    add_common_runtime_args(exec_parser)
    exec_parser.add_argument(
        "exec_command", nargs=argparse.REMAINDER, metavar="COMMAND",
        help="'-- <command> [args...]'；支持 {repo_root}/{build_dir}/{run_dir}/{profile} "
             "占位符；命令输出写入 {run_dir}/outputs")

    fixtures = sub.add_parser("fixtures", help="版本化夹具 freeze/verify")
    fixtures.add_argument("action", choices=("freeze", "verify"))
    fixtures.add_argument("--version", default="v1")
    return parser


def main(argv=None):
    parser = build_parser()
    args = parser.parse_args(argv)
    repo_root = runtime.repo_root_from(__file__)
    if hasattr(args, "retry_of"):
        try:
            args.retry_of = normalize_retry_of(args.retry_of)
        except ConfigError as exc:
            sys.stderr.write("参数错误: %s\n" % exc)
            return runtime.EXIT_DATA
    if args.command == "fixtures":
        return cmd_fixtures(args, repo_root)
    try:
        components = load_components(repo_root)
    except ConfigError as exc:
        sys.stderr.write("配置错误: %s\n" % exc)
        return runtime.EXIT_DATA
    dispatch = {
        "build": cmd_build,
        "test": cmd_test,
        "benchmark": cmd_benchmark,
        "run": cmd_run,
        "exec": cmd_exec,
    }
    handler = dispatch.get(args.command)
    if handler is None:
        parser.error("未知子命令: %s" % args.command)
    return handler(args, repo_root, components)


if __name__ == "__main__":
    sys.exit(main())
