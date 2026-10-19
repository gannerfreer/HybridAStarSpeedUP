#!/usr/bin/env python3
"""Phase 3 汇总：per-case P50（3 轮样本取中位）→ 每步提升率。

输出:
  timing/per_case_rates.csv     每例每状态 P50 + 三步速率
  timing/summary.txt            RATES.md 核心表数据
"""
from __future__ import annotations

import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

ROOT = Path("/home/gray/Planner/HybridAStarSpeedUP")
TIMING = ROOT / "timing"
STATES = ["base", "hypot", "sincos"]
ANCHORS = {
    "v3": ["case_39", "case_25", "case_3", "case_48", "case_30", "case_7", "case_80", "case_88"],
    "v4": ["pb-042", "pb-028", "pb-049", "pb-032", "pb-017", "pb-015", "pb-007", "pb-001"],
}

# (suite, case, state) -> list of wall_ms samples; also status/counters
samples: dict[tuple[str, str, str], list[float]] = defaultdict(list)
status: dict[tuple[str, str, str], set[str]] = defaultdict(set)
counters: dict[tuple[str, str, str], dict] = {}

for r in (1, 2, 3):
    for state in STATES:
        for suite in ("v3", "v4"):
            f = TIMING / f"r{r}" / state / suite / "benchmark_metrics.csv"
            with f.open() as fh:
                for row in csv.DictReader(fh):
                    key = (suite, row["case_id"], state)
                    samples[key].append(float(row["wall_ms"]))
                    status[key].add(row["status"])
                    counters[key] = {
                        "status": row["status"],
                        "wall_ms": float(row["wall_ms"]),
                        "expanded_nodes": int(row["expanded_nodes"] or 0),
                        "collision_checks": int(row["collision_checks"] or 0),
                    }

p50: dict[tuple[str, str, str], float] = {}
for key, vals in samples.items():
    assert len(vals) == 3, (key, vals)
    p50[key] = statistics.median(vals)

cases: dict[str, list[str]] = defaultdict(list)
for (suite, case, _state) in p50:
    cases[suite].append(case)
for suite in cases:
    cases[suite] = sorted(set(cases[suite]))

# 一致性核对
bad_status = []
for (suite, case, state), sts in sorted(status.items()):
    if sts != {"success"}:
        bad_status.append(f"{suite}/{case}/{state}: {sorted(sts)}")

lines = []
lines.append("== 状态一致性 ==")
lines.append(f"非 success 单元: {len(bad_status)}")
for b in bad_status:
    lines.append("  " + b)
# 同例跨状态状态翻转
flips = []
for suite in cases:
    for case in cases[suite]:
        sts = {tuple(sorted(status[(suite, case, s)])) for s in STATES}
        if len(sts) > 1:
            flips.append(f"{suite}/{case}: {sts}")
lines.append(f"跨状态 status 翻转: {len(flips)}")
for f in flips:
    lines.append("  " + f)


def rate(a: float, b: float) -> tuple[float, float]:
    """b 相对 a 的 (加速倍率, 耗时下降 %)。"""
    return a / b, (1.0 - b / a) * 100.0


def group_stats(label: str, keys: list[tuple[str, str]]) -> dict:
    ratios = {s: [] for s in ["hypot", "sincos", "cum"]}
    sums = {s: 0.0 for s in ["base", "hypot", "sincos"]}
    for suite, case in keys:
        vals = {s: p50[(suite, case, s)] for s in STATES}
        for s in STATES:
            sums[s] += vals[s]
        ratios["hypot"].append(vals["base"] / vals["hypot"])
        ratios["sincos"].append(vals["hypot"] / vals["sincos"])
        ratios["cum"].append(vals["base"] / vals["sincos"])
    out = {"label": label, "n": len(keys)}
    for step, arr in ratios.items():
        gm = math.exp(sum(math.log(x) for x in arr) / len(arr))
        out[f"{step}_gm"] = gm
        out[f"{step}_drop"] = (1.0 - 1.0 / gm) * 100.0
    out["sum_base"] = sums["base"]
    out["sum_hypot"] = sums["hypot"]
    out["sum_sincos"] = sums["sincos"]
    out["sum_hypot_ratio"] = sums["base"] / sums["hypot"]
    out["sum_sincos_ratio"] = sums["hypot"] / sums["sincos"]
    out["sum_cum_ratio"] = sums["base"] / sums["sincos"]
    return out


all_keys = [(s, c) for s in ("v3", "v4") for c in cases[s]]
slow = [(s, c) for s, c in all_keys if p50[(s, c, "base")] >= 5000.0]
mid = [(s, c) for s, c in all_keys if 500.0 <= p50[(s, c, "base")] < 5000.0]
fast = [(s, c) for s, c in all_keys if p50[(s, c, "base")] < 500.0]
anchor_keys = [(s, c) for s in ("v3", "v4") for c in ANCHORS[s]]

groups = [
    group_stats("全 166 例（几何平均）", all_keys),
    group_stats("v3 115 例", [(s, c) for s, c in all_keys if s == "v3"]),
    group_stats("v4 51 例", [(s, c) for s, c in all_keys if s == "v4"]),
    group_stats("慢档 base>=5s", slow),
    group_stats("中档 0.5~5s", mid),
    group_stats("快档 <0.5s", fast),
    group_stats("旧 16 例锚定子集", anchor_keys),
]

lines.append("")
lines.append("== 分组提升率（几何平均倍率 / 求和倍率）==")
lines.append("分组 | n | hypot 步骤 GM倍率(下降%) | sincos 步骤 GM倍率(下降%) | 累计 GM倍率(下降%) | Σbase s | Σhypot s | Σsincos s | 求和 hypot/sincos/累计")
for g in groups:
    lines.append(
        f"{g['label']} | {g['n']} | {g['hypot_gm']:.3f}x ({g['hypot_drop']:.1f}%) | "
        f"{g['sincos_gm']:.3f}x ({g['sincos_drop']:.1f}%) | {g['cum_gm']:.3f}x ({g['cum_drop']:.1f}%) | "
        f"{g['sum_base']/1000:.1f} | {g['sum_hypot']/1000:.1f} | {g['sum_sincos']/1000:.1f} | "
        f"{g['sum_hypot_ratio']:.3f} / {g['sum_sincos_ratio']:.3f} / {g['sum_cum_ratio']:.3f}"
    )

# per-case CSV
out_csv = TIMING / "per_case_rates.csv"
with out_csv.open("w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["suite", "case_id", "base_p50_ms", "hypot_p50_ms", "sincos_p50_ms",
                "hypot_speedup", "hypot_drop_pct", "sincos_speedup", "sincos_drop_pct",
                "cum_speedup", "cum_drop_pct", "bucket", "status_base"])
    for s, c in sorted(all_keys, key=lambda k: -p50[(k[0], k[1], "base")]):
        b, h, z = p50[(s, c, "base")], p50[(s, c, "hypot")], p50[(s, c, "sincos")]
        bucket = "slow" if b >= 5000 else ("mid" if b >= 500 else "fast")
        w.writerow([s, c, f"{b:.3f}", f"{h:.3f}", f"{z:.3f}",
                    f"{b/h:.4f}", f"{(1-h/b)*100:.2f}", f"{h/z:.4f}", f"{(1-z/h)*100:.2f}",
                    f"{b/z:.4f}", f"{(1-z/b)*100:.2f}", bucket,
                    sorted(status[(s, c, 'base')])[0]])

# 汇总 JSON（供 RATES.md / 报表使用）
summary = {
    "non_success": bad_status,
    "status_flips": flips,
    "groups": [
        {k: (v if not isinstance(v, float) else round(v, 6)) for k, v in g.items()}
        for g in groups
    ],
    "suite_cases": {s: len(cases[s]) for s in cases},
}
(TIMING / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2))

text = "\n".join(lines)
(TIMING / "summary.txt").write_text(text + "\n")
print(text)
