#!/usr/bin/env python3
"""由运行目录生成 Markdown/CSV 汇总与状态数据（§8.1、§8.2、§9）。

- 输入：``artifacts/runs/<run_id>/``（只读，校验产物清单，检测篡改）。
- 输出（默认到运行目录之外）：``artifacts/reports/<run_id>/`` 下的
  ``<run_id>.md``、``<run_id>.csv`` 与 ``status.json``；
  正式报告由集成负责人复制/链接到 ``Docs/reports/<component>/``。
- ``--sample`` 或 manifest 标记 ``sample: true`` 时，所有输出显著标注
  “样例 / 非实验结果”。
- 产物清单校验失败时仍生成报告（标注问题），退出码 65。

本模块只使用 CPython 3.8+ 标准库。
"""

import argparse
import math
import os
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools import clock as clock_mod
from tools import rundir
from tools import runtime

SAMPLE_BANNER = "样例 / 非实验结果"


class ReportError(RuntimeError):
    pass


def load_run(run_dir):
    layout = rundir.RunLayout(run_dir)
    if not layout.path.is_dir():
        raise ReportError("运行目录不存在: %s" % layout.path)
    try:
        manifest = runtime.read_json(layout.manifest, default=None, strict=True)
    except ValueError as exc:
        raise ReportError("manifest.json 不是有效 JSON: %s" % exc)
    if not isinstance(manifest, dict):
        raise ReportError("manifest.json 缺失或结构错误: %s" % layout.manifest)
    return {
        "layout": layout,
        "manifest": manifest,
        "metrics": rundir.read_csv_rows(layout.metrics),
        "resources": rundir.read_csv_rows(layout.resources),
        "events": rundir.read_events(layout),
        "junit": rundir.parse_junit_counts(layout.junit),
        "verification": rundir.verify_run_dir(layout.path),
    }


def count_statuses(data):
    counts = {"success": 0, "failed": 0, "resource_stopped": 0,
              "cancelled": 0, "interrupted": 0, "other": 0, "total": 0}
    rows = data["metrics"]
    if rows:
        for row in rows:
            status = (row.get("status") or "").strip()
            if not status:
                continue
            counts["total"] += 1
            if status in counts:
                counts[status] += 1
            else:
                counts["other"] += 1
    else:
        status = (data["manifest"].get("status") or "").strip()
        if status:
            counts["total"] = 1
            if status in counts:
                counts[status] = 1
            else:
                counts["other"] = 1
    return counts


def _numbers(rows, column):
    values = []
    for row in rows:
        raw = row.get(column)
        if raw is None or raw == "":
            continue
        try:
            values.append(float(raw))
        except (TypeError, ValueError):
            continue
    return values


def nearest_rank(values, percent):
    ordered = sorted(values)
    if not ordered:
        return None
    rank = int(math.ceil(float(percent) / 100.0 * len(ordered)))
    rank = max(1, min(rank, len(ordered)))
    return ordered[rank - 1]


def summarize_performance(data):
    metrics = data["metrics"]
    wall = _numbers(metrics, "wall_ms")
    cpu = _numbers(metrics, "cpu_ms")
    rss = _numbers(metrics, "peak_rss_bytes")
    resources = data["resources"]
    system_cpu = _numbers(resources, "cpu_percent")
    system_mem = _numbers(resources, "mem_percent")
    rss_resources = _numbers(resources, "task_tree_rss_bytes")
    peak_rss = max(rss + rss_resources) if (rss or rss_resources) else None
    return {
        "wall_p50_ms": nearest_rank(wall, 50),
        "wall_p95_ms": nearest_rank(wall, 95),
        "wall_min_ms": min(wall) if wall else None,
        "wall_max_ms": max(wall) if wall else None,
        "cpu_total_ms": sum(cpu) if cpu else None,
        "peak_rss_bytes": peak_rss,
        "system_cpu_max_percent": max(system_cpu) if system_cpu else None,
        "system_mem_max_percent": max(system_mem) if system_mem else None,
        "samples": len(resources),
    }


def _fmt(value, unit="", digits=3):
    if value is None:
        return "未测量"
    if isinstance(value, float):
        text = ("%%.%df" % digits) % value
    else:
        text = str(value)
    return text + unit if unit else text


def _md_cell(value):
    text = "未测量" if value is None or value == "" else str(value)
    return text.replace("|", "\\|").replace("\n", " ")


def render_markdown(data, sample=False, report_path=None):
    manifest = data["manifest"]
    counts = count_statuses(data)
    perf = summarize_performance(data)
    verification = data["verification"]
    run_id = manifest.get("run_id", data["layout"].path.name)
    lines = []
    lines.append("# %s 验收与实验报告" % manifest.get("component", "unknown"))
    lines.append("")
    if sample:
        lines.append("> **%s** — 本文件由固定样例数据生成，不代表任何真实实验结论；" % SAMPLE_BANNER)
        lines.append("> 不得作为算法正确性或性能证据。")
        lines.append("")
    lines.append("> 由 `tools/report_results.py` 自动生成；运行目录只读，本报告为外部文件。")
    lines.append("")
    lines.append("## 1. 结论")
    lines.append("")
    lines.append("- 开发交付状态：%s" % _md_cell(manifest.get("task_status")
                                              or manifest.get("status")))
    lines.append("- 本次验收/实验结果：%s" % _md_cell(manifest.get("status")))
    lines.append("- 关联问题与影响范围：见 `Docs/reports/ISSUES.md`（本工具不修改问题清单）")
    lines.append("- 本次目标与验收范围：%s / %s"
                 % (_md_cell(manifest.get("task_kind")), _md_cell(manifest.get("task_id"))))
    lines.append("- 是否满足交付：%s（产物清单校验：%s）"
                 % ("是" if manifest.get("status") == "success" else "否",
                    "通过" if verification.get("ok") else "失败"))
    lines.append("- 未验证项与限制：%s"
                 % ("无" if counts["total"] else "未测量（没有 metrics.csv 记录）"))
    lines.append("")
    lines.append("## 2. 复现信息")
    lines.append("")
    lines.append("| 字段 | 值 |")
    lines.append("| --- | --- |")
    rows = [
        ("Run ID / Task ID", "%s / %s" % (run_id, manifest.get("task_id"))),
        ("组件 / 后端", "%s / %s" % (manifest.get("component"), manifest.get("backend"))),
        ("UTC 开始 / 结束时间", "%s / %s" % (manifest.get("started_at"),
                                            manifest.get("finished_at"))),
        ("Git commit / dirty", "%s / %s" % (manifest.get("git_commit"),
                                            manifest.get("dirty"))),
        ("上游提交与依赖版本", _deps_text(manifest.get("dependency_versions"))),
        ("硬件 / 系统 / Python", _hardware_text(manifest.get("hardware"))),
        ("数据集版本 / 输入 SHA256", _inputs_text(manifest.get("input_sha256"))),
        ("配置 SHA256 / 种子 / worker 数", "%s / %s / %s"
         % (manifest.get("config_sha256"), manifest.get("seed"),
            manifest.get("worker_count"))),
        ("原始数据目录 / manifest", str(data["layout"].path / "manifest.json")),
        ("产物清单 / artifact_sha256 / 校验结果",
         "%s / %s / %s" % ("artifacts.sha256.json",
                           verification.get("artifact_sha256"),
                           "通过" if verification.get("ok") else "失败")),
        ("重试来源 retry_of", manifest.get("retry_of") or "N/A"),
        ("失败原因", manifest.get("failure_reason")),
    ]
    for key, value in rows:
        lines.append("| %s | %s |" % (key, _md_cell(value)))
    lines.append("")
    lines.append("执行目录：`%s`" % _md_cell(manifest.get("cwd")))
    lines.append("")
    lines.append("```sh")
    command = manifest.get("command_argv")
    if isinstance(command, list):
        import shlex
        if command and isinstance(command[0], list):
            for step in command:
                lines.append(shlex.join([str(part) for part in step]))
        else:
            lines.append(shlex.join([str(part) for part in command]))
    else:
        lines.append("未测量")
    lines.append("```")
    lines.append("")
    lines.append("## 3. 正确性")
    lines.append("")
    if data["junit"]:
        j = data["junit"]
        lines.append("JUnit：tests=%d, failures=%d, errors=%d, skipped=%d"
                     % (j["tests"], j["failures"], j["errors"], j["skipped"]))
        lines.append("")
    if data["metrics"]:
        lines.append("| 用例 | 状态 | 失败原因 | 证据 |")
        lines.append("| --- | --- | --- | --- |")
        for row in data["metrics"]:
            lines.append("| %s | %s | %s | %s |"
                         % (_md_cell(row.get("case_id")),
                            _md_cell(row.get("status")),
                            _md_cell(row.get("failure_reason")),
                            "junit.xml" if data["junit"] else "未测量"))
    else:
        lines.append("未测量（没有 metrics.csv 记录或 junit.xml）。")
    lines.append("")
    lines.append("## 4. 性能")
    lines.append("")
    lines.append("| 指标 | 基线 | 本次 | 相对变化 |")
    lines.append("| --- | --- | --- | --- |")
    perf_rows = [
        ("尝试 / 有效完成 / 成功 / 失败 / 中断数量",
         "%d / %d / %d / %d / %d" % (counts["total"], counts["total"] - counts["interrupted"],
                                     counts["success"], counts["failed"], counts["interrupted"])),
        ("算法成功率（分母=成功+失败）", _rate(counts)),
        ("wall_ms P50 / P95", "%s / %s" % (_fmt(perf["wall_p50_ms"]),
                                           _fmt(perf["wall_p95_ms"]))),
        ("总 CPU 时间（ms）", _fmt(perf["cpu_total_ms"])),
        ("任务树采样 RSS 峰值（bytes）", _fmt(perf["peak_rss_bytes"])),
        ("资源样本数", perf["samples"]),
    ]
    for key, value in perf_rows:
        lines.append("| %s | N/A | %s | N/A |" % (key, value))
    lines.append("")
    lines.append("样本数、预热次数、缓存状态与统计方法以 manifest/config 与原始表格为准；"
                 "中断单列，不删除原始记录。")
    lines.append("")
    lines.append("## 5. 资源与异常")
    lines.append("")
    lines.append("| 时间（UTC） | 事件 | 说明 |")
    lines.append("| --- | --- | --- |")
    interesting = [e for e in data["events"]
                   if e.get("event") in ("resource_stopped", "resource_stop_requested",
                                         "monitor_lost", "dispatch_paused",
                                         "dispatch_resumed", "queued", "error")]
    if interesting:
        for event in interesting:
            details = event.get("details") or {}
            lines.append("| %s | %s | %s |"
                         % (_md_cell(event.get("time")), _md_cell(event.get("event")),
                            _md_cell(_compact(details))))
    else:
        lines.append("| - | 无异常事件 | 系统峰值 CPU %s%% / 内存 %s%% |"
                     % (_fmt(perf["system_cpu_max_percent"], digits=2),
                        _fmt(perf["system_mem_max_percent"], digits=2)))
    lines.append("")
    lines.append("## 6. 产物与下一步")
    lines.append("")
    lines.append("- 原始指标、资源记录与日志：`metrics.csv`、`resources.csv`、`events.jsonl`、"
                 "`stdout.log`、`stderr.log`")
    lines.append("- 测试 XML、输出与图表：`junit.xml`、`outputs/`、`plots/`")
    lines.append("- 产物校验：%s" % ("通过" if verification.get("ok") else
                                   "失败：" + "; ".join(verification.get("problems", []))))
    lines.append("- 本次失败与遗留问题：%s" % _md_cell(manifest.get("failure_reason")))
    lines.append("- 下一步、负责人和前置条件：由组件负责人补充；本工具不推导")
    latest = manifest.get("latest_success_report")
    lines.append("- 最近成功报告：%s" % (latest or "尚无"))
    if report_path:
        lines.append("")
        lines.append("报告文件：`%s`" % report_path)
    lines.append("")
    return "\n".join(lines)


def _rate(counts):
    denominator = counts["success"] + counts["failed"]
    if denominator == 0:
        return "未测量"
    return "%.4f" % (counts["success"] / float(denominator))


def _compact(details):
    if not isinstance(details, dict):
        return str(details)
    parts = []
    for key in sorted(details):
        parts.append("%s=%s" % (key, details[key]))
    return ", ".join(parts)


def _deps_text(deps):
    if not isinstance(deps, dict) or not deps:
        return "未测量"
    parts = []
    for name in sorted(deps):
        item = deps[name]
        if isinstance(item, dict):
            parts.append("%s=%s(%s)" % (name, item.get("version"), item.get("status")))
        else:
            parts.append("%s=%s" % (name, item))
    return "; ".join(parts)


def _hardware_text(hardware):
    if not isinstance(hardware, dict) or not hardware:
        return "未测量"
    return "%s / %s / py%s" % (hardware.get("machine"), hardware.get("platform"),
                               hardware.get("python"))


def _inputs_text(inputs):
    if not isinstance(inputs, dict) or not inputs:
        return "未测量"
    return "; ".join("%s=%s" % (key, inputs[key]) for key in sorted(inputs))


def render_csv_text(data, sample=False):
    counts = count_statuses(data)
    perf = summarize_performance(data)
    rows = []
    if sample:
        rows.append({"metric": "sample_marker", "value": SAMPLE_BANNER})
    rows.extend([
        {"metric": "run_id", "value": data["manifest"].get("run_id", "")},
        {"metric": "task_id", "value": data["manifest"].get("task_id") or ""},
        {"metric": "component", "value": data["manifest"].get("component") or ""},
        {"metric": "status", "value": data["manifest"].get("status") or ""},
        {"metric": "success_count", "value": counts["success"]},
        {"metric": "failed_count", "value": counts["failed"]},
        {"metric": "resource_stopped_count", "value": counts["resource_stopped"]},
        {"metric": "cancelled_count", "value": counts["cancelled"]},
        {"metric": "interrupted_count", "value": counts["interrupted"]},
        {"metric": "total_count", "value": counts["total"]},
        {"metric": "wall_p50_ms", "value": perf["wall_p50_ms"]},
        {"metric": "wall_p95_ms", "value": perf["wall_p95_ms"]},
        {"metric": "cpu_total_ms", "value": perf["cpu_total_ms"]},
        {"metric": "peak_rss_bytes", "value": perf["peak_rss_bytes"]},
        {"metric": "verification_ok", "value": data["verification"].get("ok")},
    ])
    return rundir.csv_text(("metric", "value"), rows)


def build_status_data(data, sample=False, report_path=None):
    manifest = data["manifest"]
    counts = count_statuses(data)
    verification = data["verification"]
    overview = {
        "task_id": manifest.get("task_id"),
        "component": manifest.get("component"),
        "owner": manifest.get("owner"),
        "status": manifest.get("status"),
        "prerequisites": manifest.get("prerequisites"),
        "latest_run_id": manifest.get("run_id"),
        "passed_cases": counts["success"] if counts["total"] else None,
        "total_cases": counts["total"] if counts["total"] else None,
        "duration_s": _duration_s(data),
        "failure_reason": manifest.get("failure_reason"),
        "report_path": report_path,
        "next_step": manifest.get("next_step"),
        "updated_at": manifest.get("finished_at") or manifest.get("started_at"),
    }
    return {
        "schema_version": runtime.SCHEMA_VERSION,
        "type": "planner_run_status",
        "sample": bool(sample),
        "sample_banner": SAMPLE_BANNER if sample else None,
        "overview": overview,
        "counts": counts,
        "verification": verification,
    }


def _duration_s(data):
    manifest = data["manifest"]
    started = manifest.get("started_at")
    finished = manifest.get("finished_at")
    if not started or not finished:
        return None
    try:
        delta = (clock_mod.parse_utc_iso(finished) - clock_mod.parse_utc_iso(started))
        return round(delta.total_seconds(), 3)
    except (TypeError, ValueError):
        return None


def generate(run_dir, output_dir=None, sample=False, repo_root=None):
    data = load_run(run_dir)
    sample = bool(sample or data["manifest"].get("sample"))
    run_id = data["manifest"].get("run_id", Path(run_dir).name)
    if output_dir is None:
        root = Path(repo_root) if repo_root else runtime.repo_root_from(run_dir)
        output_dir = root / "artifacts" / "reports" / run_id
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    md_path = output_dir / ("%s.md" % run_id)
    csv_path = output_dir / ("%s.csv" % run_id)
    status_path = output_dir / "status.json"

    md_text = render_markdown(data, sample=sample, report_path=str(md_path))
    runtime.atomic_write_text(md_path, md_text)
    runtime.atomic_write_text(csv_path, render_csv_text(data, sample=sample))
    runtime.atomic_write_json(status_path, build_status_data(
        data, sample=sample, report_path=str(md_path)))
    return {
        "run_id": run_id,
        "sample": sample,
        "report": str(md_path),
        "csv": str(csv_path),
        "status": str(status_path),
        "verification": data["verification"],
        "counts": count_statuses(data),
    }


def build_parser():
    parser = argparse.ArgumentParser(
        prog="report_results.py",
        description="由运行目录生成 Markdown/CSV 汇总与状态数据（只读运行目录）")
    parser.add_argument("--run-dir", required=True, help="artifacts/runs/<run_id>")
    parser.add_argument("--output-dir", default=None,
                        help="输出目录（默认 artifacts/reports/<run_id>/）")
    parser.add_argument("--sample", action="store_true",
                        help="样例模式：所有输出标注“%s”" % SAMPLE_BANNER)
    parser.add_argument("--repo-root", default=None, help="仓库根（默认由运行目录推导）")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    try:
        result = generate(args.run_dir, output_dir=args.output_dir,
                          sample=args.sample, repo_root=args.repo_root)
    except ReportError as exc:
        sys.stderr.write("report_results: %s\n" % exc)
        return runtime.EXIT_DATA
    print("报告: %s" % result["report"])
    print("汇总 CSV: %s" % result["csv"])
    print("状态数据: %s" % result["status"])
    if result["sample"]:
        print("提示: %s" % SAMPLE_BANNER)
    if result["verification"]["ok"]:
        return runtime.EXIT_OK
    sys.stderr.write("产物清单校验失败：\n")
    for problem in result["verification"]["problems"]:
        sys.stderr.write("  - %s\n" % problem)
    return runtime.EXIT_DATA


if __name__ == "__main__":
    sys.exit(main())
