#!/usr/bin/env python3
"""E0-6 报告流程样例：生成固定样例运行目录并输出“样例 / 非实验结果”报告。

用途：验证 report_results 的完整流程（封存、校验、Markdown/CSV/状态数据）。
本脚本只生成合成样例，**不是实验证据**，所有输出均标注样例；重复运行会重建
``artifacts/samples/report_sample/``（只允许重建该目录）。

用法：
    python3 tools/sample_report.py [--repo-root PATH]

退出码：0 成功；65 数据/校验错误；70 内部错误。不占用 worker 名额、不登记、
不需要资源监控（纯离线报告流程验证）。
"""

import argparse
import os
import shutil
import sys
from pathlib import Path

if __package__ in (None, ""):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools import clock as clock_mod
from tools import report_results
from tools import rundir
from tools import runtime

SAMPLE_RUN_ID = "20260101T000000000000Z_sample_report_deadbeef"
SAMPLE_BANNER = "样例 / 非实验结果"


def build_sample_run(repo_root):
    repo_root = Path(repo_root)
    sample_root = repo_root / "artifacts" / "samples"
    run_dir = sample_root / "report_sample"
    if run_dir.exists():
        shutil.rmtree(str(run_dir))
    layout = rundir.RunLayout(run_dir).ensure_dirs()
    clk = clock_mod.FakeClock()
    manifest = {
        "schema_version": 1,
        "type": "planner_run_manifest",
        "run_id": SAMPLE_RUN_ID,
        "task_id": "sample:report",
        "component": "sample",
        "backend": "none",
        "task_kind": "test",
        "command_argv": [["cmake", "--build", "build/release", "--parallel", "4"]],
        "cwd": str(repo_root),
        "started_at": "2026-01-01T00:00:00.000Z",
        "finished_at": None,
        "status": "running",
        "exit_code": None,
        "git_commit": "sample",
        "dirty": False,
        "dependency_versions": {},
        "hardware": {"machine": "sample", "platform": "sample", "python": "3.8"},
        "build_profile": "release",
        "config_sha256": None,
        "input_sha256": {},
        "seed": 0,
        "worker_count": 1,
        "artifact_sha256": None,
        "retry_of": None,
        "failure_reason": None,
        "sample": True,
    }
    manifest = rundir.init_run(layout, manifest, {
        "schema_version": 1,
        "sample": True,
        "note": SAMPLE_BANNER,
    }, clock=clk)
    for case_id, status in (("sample_ok", "success"),
                            ("sample_fail", "failed"),
                            ("sample_stop", "resource_stopped"),
                            ("sample_interrupt", "interrupted")):
        rundir.append_metric_row(layout, {
            "run_id": SAMPLE_RUN_ID,
            "case_id": case_id,
            "repeat_id": "0",
            "cache_mode": "cold",
            "seed": "0",
            "status": status,
            "failure_reason": "" if status == "success" else status,
            "preprocess_ms": "0.500",
            "wall_ms": "12.000",
            "cpu_ms": "8.000",
            "peak_rss_bytes": "1048576",
            "valid": "true" if status == "success" else "false",
        })
    for index in range(3):
        rundir.append_resource_row(layout, {
            "time_utc": "2026-01-01T00:00:0%d.000Z" % (1 + index),
            "monotonic_elapsed_s": "%.1f" % (index + 1),
            "cpu_percent": "10.000",
            "mem_percent": "25.000",
            "task_tree_rss_bytes": "1048576",
            "active_tasks": "1",
            "build_task_id": "",
        })
    layout.junit.write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<testsuites name="sample" tests="4" failures="1" errors="0" skipped="0" '
        'time="1.0"><testsuite name="sample" tests="4" failures="1" errors="0" '
        'skipped="0" time="1.0"/></testsuites>\n',
        encoding="utf-8")
    rundir.record_event(layout, clk, "resource_stopped", task_id="sample:report",
                        component="sample",
                        details={"metric": "cpu_percent", "value": 95.0,
                                 "note": SAMPLE_BANNER})
    rundir.write_task_summary(layout, manifest,
                              notes=["固定样例，仅验证报告流程，不是实验证据。"])
    lock_path = sample_root / "report_sample.lock"
    rundir.seal_run(run_dir, {
        "status": "resource_stopped",
        "exit_code": 143,
        "failure_reason": "resource_stopped:cpu_percent=95.0>90.0",
    }, clock=clk, lock_path=lock_path)
    return run_dir


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="sample_report.py",
        description="生成固定样例运行目录并输出带样例标注的报告（非实验）")
    parser.add_argument("--repo-root", default=None)
    args = parser.parse_args(argv)
    repo_root = args.repo_root or runtime.repo_root_from(__file__)
    try:
        run_dir = build_sample_run(repo_root)
        output_dir = Path(repo_root) / "artifacts" / "reports" / "report_sample"
        result = report_results.generate(run_dir, output_dir=output_dir, sample=True)
    except (OSError, rundir.RunDirError, report_results.ReportError) as exc:
        sys.stderr.write("sample_report 失败: %s\n" % exc)
        return runtime.EXIT_DATA
    print("样例运行目录: %s" % run_dir)
    print("样例报告: %s" % result["report"])
    print("汇总 CSV: %s" % result["csv"])
    print("状态数据: %s" % result["status"])
    print("提示: %s —— 不得作为算法正确性或性能证据" % SAMPLE_BANNER)
    if not result["verification"]["ok"]:
        sys.stderr.write("内部错误：样例目录未通过清单校验\n")
        return runtime.EXIT_DATA
    return runtime.EXIT_OK


if __name__ == "__main__":
    sys.exit(main())
