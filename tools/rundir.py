"""运行目录生命周期：创建、初始脚手架、度量写入、封存与校验。

与 §9 对齐：

- run ID 形如 ``YYYYMMDDTHHMMSSffffffZ_<component>_<8位随机>``，独占创建；
- 运行目录固定包含 manifest.json、config.json、inputs/、source.patch、
  source_untracked.tar.gz（存在时）、metrics.csv、resources.csv、events.jsonl、
  stdout.log、stderr.log、junit.xml（适用时）、outputs/、plots/、summary.md、
  artifacts.sha256.json；
- ``artifacts.sha256.json`` 为按 POSIX 相对路径字典序排列的数组，每项含
  ``path, size_bytes, sha256``；排除 manifest.json、清单自身与 ``*.tmp``；
- ``manifest.json.artifact_sha256`` 等于清单文件原始字节的 SHA256；
- 封存顺序：关闭日志/资源写入 -> 生成 summary -> 计算清单 -> 保存清单并计算
  哈希 -> 原子写入最终 manifest；封存后目录不可变。

本模块只使用 CPython 3.8+ 标准库。
"""

import csv
import io
import json
import os
import tarfile
from pathlib import Path
from xml.etree import ElementTree

from tools import clock as clock_mod
from tools import locking
from tools import runtime

MANIFEST_NAME = "manifest.json"
ARTIFACTS_NAME = "artifacts.sha256.json"
SUMMARY_NAME = "summary.md"
CONFIG_NAME = "config.json"
METRICS_NAME = "metrics.csv"
RESOURCES_NAME = "resources.csv"
EVENTS_NAME = "events.jsonl"
STDOUT_NAME = "stdout.log"
STDERR_NAME = "stderr.log"
JUNIT_NAME = "junit.xml"

METRIC_COLUMNS = (
    "run_id", "case_id", "repeat_id", "cache_mode", "seed", "status",
    "failure_reason", "preprocess_ms", "wall_ms", "cpu_ms", "peak_rss_bytes",
    "expanded_nodes", "collision_checks", "path_length_m", "gear_changes",
    "endpoint_position_error_m", "endpoint_yaw_error_rad", "valid",
)

RESOURCE_COLUMNS = (
    "time_utc", "monotonic_elapsed_s", "cpu_percent", "mem_percent",
    "task_tree_rss_bytes", "active_tasks", "build_task_id",
)


class RunDirError(RuntimeError):
    pass


class RunLayout(object):
    """运行目录内固定文件名与子目录。"""

    def __init__(self, run_dir):
        self.path = Path(run_dir)

    def child(self, *parts):
        return self.path.joinpath(*parts)

    @property
    def manifest(self):
        return self.child(MANIFEST_NAME)

    @property
    def artifacts_manifest(self):
        return self.child(ARTIFACTS_NAME)

    @property
    def config(self):
        return self.child(CONFIG_NAME)

    @property
    def metrics(self):
        return self.child(METRICS_NAME)

    @property
    def resources(self):
        return self.child(RESOURCES_NAME)

    @property
    def events(self):
        return self.child(EVENTS_NAME)

    @property
    def stdout(self):
        return self.child(STDOUT_NAME)

    @property
    def stderr(self):
        return self.child(STDERR_NAME)

    @property
    def junit(self):
        return self.child(JUNIT_NAME)

    @property
    def summary(self):
        return self.child(SUMMARY_NAME)

    @property
    def inputs(self):
        return self.child("inputs")

    @property
    def outputs(self):
        return self.child("outputs")

    @property
    def plots(self):
        return self.child("plots")

    def ensure_dirs(self):
        self.path.mkdir(parents=True, exist_ok=True)
        for name in ("inputs", "outputs", "plots"):
            self.child(name).mkdir(parents=True, exist_ok=True)
        return self


def new_run_id(component, clock=None, random_hex=None):
    clk = clock if clock is not None else clock_mod.Clock()
    stamp = clock_mod.run_id_timestamp(clk.utc_now())
    token = random_hex if random_hex is not None else os.urandom(4).hex()
    return "%s_%s_%s" % (stamp, runtime.safe_name(component), token)


def parse_run_id(run_id):
    parts = str(run_id).split("_")
    if len(parts) < 3:
        return None
    return {"timestamp": parts[0], "component": "_".join(parts[1:-1]), "random": parts[-1]}


def create_run_dir(runs_root, component, retry_of=None, clock=None, run_id=None):
    """独占创建运行目录；返回 (run_id, Path)。"""
    clk = clock if clock is not None else clock_mod.Clock()
    root = Path(runs_root)
    root.mkdir(parents=True, exist_ok=True)
    attempts = 1 if run_id else 100
    for _ in range(attempts):
        candidate_id = run_id if run_id else new_run_id(component, clock=clk)
        candidate = root / candidate_id
        try:
            candidate.mkdir()
        except FileExistsError:
            if run_id:
                raise RunDirError("运行目录已存在: %s" % candidate)
            continue
        return candidate_id, candidate
    raise RunDirError("无法生成唯一 run ID（尝试 %d 次）" % attempts)


def write_json_file(path, obj):
    """原子写 JSON；返回写入字节 SHA256。"""
    return runtime.atomic_write_json(path, obj)


def append_csv_row(path, row, columns=None):
    """追加 CSV 行；``row is None`` 时只确保表头存在。"""
    columns = tuple(columns) if columns is not None else None
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    need_header = not path.exists() or path.stat().st_size == 0
    with open(str(path), "a", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        if need_header:
            if columns is None:
                raise RunDirError("首次写入 CSV 必须提供列定义: %s" % path)
            writer.writerow(list(columns))
        if row is not None:
            if columns is not None:
                writer.writerow([_csv_cell(row.get(name, "")) for name in columns])
            else:
                writer.writerow([_csv_cell(value) for value in row])
        handle.flush()


def _csv_cell(value):
    if value is None:
        return ""
    return value


def read_csv_rows(path):
    path = Path(path)
    if not path.is_file():
        return []
    with open(str(path), "r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        return [dict(item) for item in reader]


def init_run(layout, manifest, config, clock=None):
    """创建初始脚手架：manifest/config、表头、空日志与事件文件。"""
    clk = clock if clock is not None else clock_mod.Clock()
    layout.ensure_dirs()
    config_sha = write_json_file(layout.config, config)
    manifest = dict(manifest)
    manifest["config_sha256"] = config_sha
    write_json_file(layout.manifest, manifest)
    append_csv_row(layout.metrics, None, columns=METRIC_COLUMNS)
    append_csv_row(layout.resources, None, columns=RESOURCE_COLUMNS)
    for name in (EVENTS_NAME, STDOUT_NAME, STDERR_NAME):
        target = layout.child(name)
        if not target.exists():
            target.write_bytes(b"")
    return manifest


def record_event(layout, clock, event, task_id=None, component=None, details=None):
    clk = clock if clock is not None else clock_mod.Clock()
    record = {
        "time": runtime.now_iso(clk),
        "task_id": task_id,
        "run_id": layout.path.name,
        "component": component,
        "event": event,
        "details": dict(details) if details else {},
    }
    runtime.append_jsonl(layout.events, record)
    return record


def update_manifest(layout, fields):
    manifest = runtime.read_json(layout.manifest, default=None, strict=True)
    if not isinstance(manifest, dict):
        raise RunDirError("manifest.json 缺失或损坏: %s" % layout.manifest)
    manifest.update(fields)
    runtime.atomic_write_json(layout.manifest, manifest)
    return manifest


def append_metric_row(layout, row):
    append_csv_row(layout.metrics, row, columns=METRIC_COLUMNS)


def append_resource_row(layout, row):
    append_csv_row(layout.resources, row, columns=RESOURCE_COLUMNS)


def compute_artifact_manifest(run_dir):
    root = Path(run_dir)
    items = []
    for full in runtime.list_relative_files(root):
        rel = full.relative_to(root).as_posix()
        if rel in (MANIFEST_NAME, ARTIFACTS_NAME):
            continue
        if rel.endswith(runtime.TEMP_SUFFIX):
            continue
        items.append({
            "path": rel,
            "size_bytes": full.stat().st_size,
            "sha256": runtime.sha256_file(full),
        })
    items.sort(key=lambda item: item["path"])
    return items


def _seal_unlocked(layout, final_fields, clock):
    if not layout.summary.exists():
        write_task_summary(layout, {}, notes=["封存时补写的最小摘要。"])
    items = compute_artifact_manifest(layout.path)
    runtime.atomic_write_json(layout.artifacts_manifest, items)
    artifact_sha = runtime.sha256_file(layout.artifacts_manifest)
    manifest = runtime.read_json(layout.manifest, default=None, strict=False)
    if not isinstance(manifest, dict):
        manifest = {}
    manifest.update(dict(final_fields))
    manifest["artifact_sha256"] = artifact_sha
    if not manifest.get("finished_at"):
        clk = clock if clock is not None else clock_mod.Clock()
        manifest["finished_at"] = runtime.now_iso(clk)
    runtime.atomic_write_json(layout.manifest, manifest)
    return artifact_sha


def seal_run(run_dir, final_fields, clock=None, lock_path=None, lock_timeout=30.0):
    """按 §9 封存运行目录；返回 artifacts.sha256.json 的字节 SHA256。"""
    layout = RunLayout(run_dir)
    if not layout.path.is_dir():
        raise RunDirError("运行目录不存在: %s" % layout.path)
    lock = None
    if lock_path is not None:
        lock = locking.FileLock(lock_path)
        if not lock.acquire(timeout=lock_timeout):
            raise RunDirError("证据封存锁超时: %s" % lock_path)
    try:
        return _seal_unlocked(layout, final_fields, clock)
    finally:
        if lock is not None:
            lock.release()


def verify_run_dir(run_dir):
    """校验清单哈希、逐项大小/内容，并拒绝清单外额外文件。"""
    root = Path(run_dir)
    problems = []
    layout = RunLayout(root)
    if not root.is_dir():
        return {"ok": False, "problems": ["运行目录不存在: %s" % root],
                "checked": 0, "artifact_sha256": None,
                "manifest_artifact_sha256": None}
    try:
        manifest = runtime.read_json(layout.manifest, default=None, strict=True)
    except ValueError as exc:
        manifest = None
        problems.append("manifest.json 不是有效 JSON: %s" % exc)
    if not isinstance(manifest, dict):
        problems.append("manifest.json 缺失或结构错误")
        manifest = {}
    try:
        artifacts = runtime.read_json(layout.artifacts_manifest, default=None, strict=True)
    except ValueError as exc:
        artifacts = None
        problems.append("artifacts.sha256.json 不是有效 JSON: %s" % exc)
    if not isinstance(artifacts, list):
        problems.append("artifacts.sha256.json 缺失或不是数组")
        artifacts = []

    listed = {}
    paths = []
    for item in artifacts:
        if not isinstance(item, dict):
            problems.append("清单项不是对象: %r" % (item,))
            continue
        missing = [key for key in ("path", "size_bytes", "sha256") if key not in item]
        if missing:
            problems.append("清单项缺少字段 %s: %r" % (",".join(missing), item))
            continue
        rel = str(item["path"])
        paths.append(rel)
        if rel in listed:
            problems.append("清单路径重复: %s" % rel)
            continue
        if rel.startswith("/") or ".." in Path(rel).parts:
            problems.append("清单路径非法: %s" % rel)
            continue
        listed[rel] = item
    if paths != sorted(paths):
        problems.append("清单未按 POSIX 相对路径字典序排列")

    actual_sha = None
    if layout.artifacts_manifest.is_file():
        actual_sha = runtime.sha256_file(layout.artifacts_manifest)
        manifest_sha = manifest.get("artifact_sha256")
        if manifest_sha != actual_sha:
            problems.append(
                "artifact_sha256 与清单文件字节哈希不一致: manifest=%r actual=%r"
                % (manifest_sha, actual_sha))
    else:
        problems.append("artifacts.sha256.json 缺失")

    for rel, item in listed.items():
        full = root / rel
        if not full.is_file():
            problems.append("清单文件缺失: %s" % rel)
            continue
        size = full.stat().st_size
        try:
            expected_size = int(item["size_bytes"])
        except (TypeError, ValueError):
            problems.append("清单 size_bytes 非法: %s" % rel)
            continue
        if expected_size != size:
            problems.append("size_bytes 不一致: %s (清单 %d / 实际 %d)"
                            % (rel, expected_size, size))
        actual_file_sha = runtime.sha256_file(full)
        if item["sha256"] != actual_file_sha:
            problems.append("sha256 不一致（疑似篡改）: %s" % rel)

    on_disk = set()
    for full in runtime.list_relative_files(root):
        rel = full.relative_to(root).as_posix()
        if rel in (MANIFEST_NAME, ARTIFACTS_NAME):
            continue
        if rel.endswith(runtime.TEMP_SUFFIX):
            continue
        on_disk.add(rel)
    for rel in sorted(on_disk - set(listed)):
        problems.append("存在清单外额外文件: %s" % rel)

    return {
        "ok": not problems,
        "problems": problems,
        "checked": len(listed),
        "artifact_sha256": actual_sha,
        "manifest_artifact_sha256": manifest.get("artifact_sha256"),
    }


def recover_interrupted_run(run_dir, clock=None, status="failed", reason="interrupted",
                            lock_path=None, extra_fields=None, lock_timeout=30.0):
    """恢复未封存的中断运行：标记状态并按同一流程封存已有证据。

    返回 missing / missing_manifest / already_sealed / sealed。
    已在封存锁内二次核对，避免与正常封存竞争。
    """
    layout = RunLayout(run_dir)
    if not layout.path.is_dir():
        return "missing"
    clk = clock if clock is not None else clock_mod.Clock()
    lock = None
    if lock_path is not None:
        lock = locking.FileLock(lock_path)
        if not lock.acquire(timeout=lock_timeout):
            return "locked"
    try:
        manifest = runtime.read_json(layout.manifest, default=None, strict=False)
        if not isinstance(manifest, dict):
            return "missing_manifest"
        if manifest.get("artifact_sha256"):
            return "already_sealed"
        final = {"status": status, "failure_reason": reason}
        if extra_fields:
            final.update(extra_fields)
        if not layout.summary.exists():
            write_task_summary(layout, manifest, notes=["进程中断后由恢复流程补写。"])
        _seal_unlocked(layout, final, clk)
        return "sealed"
    finally:
        if lock is not None:
            lock.release()


def write_task_summary(layout, manifest, counts=None, notes=None):
    """生成运行目录内的 summary.md（封存前写入，不嵌入 artifact 哈希）。"""
    manifest = dict(manifest) if isinstance(manifest, dict) else {}
    lines = []
    component = manifest.get("component", layout.path.parent.name)
    kind = manifest.get("task_kind", "task")
    lines.append("# 运行摘要：%s / %s" % (component, kind))
    lines.append("")
    lines.append("> 本文件由 tools/task_runner.py 在封存前生成；")
    lines.append("> 完整产物哈希与状态见 manifest.json 与 artifacts.sha256.json。")
    lines.append("")
    lines.append("| 字段 | 值 |")
    lines.append("| --- | --- |")
    rows = [
        ("run ID", manifest.get("run_id", layout.path.name)),
        ("任务 ID", manifest.get("task_id", "-")),
        ("组件 / 后端", "%s / %s" % (manifest.get("component", "-"),
                                     manifest.get("backend", "-"))),
        ("任务类型", kind),
        ("状态", manifest.get("status", "running")),
        ("退出码", manifest.get("exit_code", "未确定")),
        ("开始 / 结束（UTC）", "%s / %s" % (manifest.get("started_at", "-"),
                                          manifest.get("finished_at", "-"))),
        ("Git commit / dirty", "%s / %s" % (manifest.get("git_commit", "-"),
                                            manifest.get("dirty", "-"))),
        ("配置 SHA256", manifest.get("config_sha256", "-")),
        ("worker 数", manifest.get("worker_count", "-")),
        ("失败原因", manifest.get("failure_reason", "-")),
        ("retry_of", manifest.get("retry_of", "N/A")),
    ]
    for key, value in rows:
        lines.append("| %s | %s |" % (key, _md_cell(value)))
    if counts:
        lines.append("")
        lines.append("## 计数")
        lines.append("")
        for key in sorted(counts):
            lines.append("- %s: %s" % (key, counts[key]))
    if notes:
        lines.append("")
        lines.append("## 备注")
        lines.append("")
        for note in notes:
            lines.append("- %s" % note)
    lines.append("")
    runtime.atomic_write_text(layout.summary, "\n".join(lines))
    return layout.summary


def _md_cell(value):
    text = "-" if value is None else str(value)
    return text.replace("|", "\\|").replace("\n", " ")


def parse_junit_counts(path):
    """解析 JUnit XML；文件缺失或损坏返回 None。"""
    path = Path(path)
    if not path.is_file():
        return None
    try:
        tree = ElementTree.parse(str(path))
    except ElementTree.ParseError:
        return None
    root = tree.getroot()
    if root.tag == "testsuites":
        suites = list(root)
    elif root.tag == "testsuite":
        suites = [root]
    else:
        suites = [root]
    counts = {"tests": 0, "failures": 0, "errors": 0, "skipped": 0, "time": 0.0}
    for suite in suites:
        for key in ("tests", "failures", "errors", "skipped"):
            try:
                counts[key] += int(float(suite.get(key, 0)))
            except (TypeError, ValueError):
                pass
        try:
            counts["time"] += float(suite.get("time", 0.0) or 0.0)
        except (TypeError, ValueError):
            pass
    return counts


def make_source_archive(layout, repo_root, relatives):
    """把未跟踪源文件打包为 source_untracked.tar.gz；返回文件数。"""
    if not relatives:
        return 0
    target = layout.child("source_untracked.tar.gz")
    with tarfile.open(str(target), "w:gz") as archive:
        for rel in sorted(relatives):
            full = Path(repo_root) / rel
            try:
                archive.add(str(full), arcname=rel)
            except OSError:
                continue
    return len(relatives)


def write_text_atomic(path, text):
    return runtime.atomic_write_text(path, text)


def read_events(layout):
    path = layout.events if isinstance(layout, RunLayout) else Path(layout)
    records = []
    if not path.is_file():
        return records
    with open(str(path), "r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                item = json.loads(line)
            except ValueError:
                continue
            if isinstance(item, dict):
                records.append(item)
    return records


def csv_text(columns, rows):
    buffer = io.StringIO()
    writer = csv.writer(buffer)
    writer.writerow(list(columns))
    for row in rows:
        writer.writerow([_csv_cell(row.get(name, "")) for name in columns])
    return buffer.getvalue()
