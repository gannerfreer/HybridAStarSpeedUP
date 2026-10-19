#!/usr/bin/env python3
"""fig_libm_* 自检：CSV 校验（原始行合计=100）+ HTML 内嵌 JSON 数据级校验。"""
import csv
import json
import re
from pathlib import Path

OUT = Path("/home/gray/Planner/HybridAStarSpeedUP/figures")
DATA = OUT / "data"
CASES = ["case_30", "case_39", "pb-017"]
ORDER = ["碰撞几何/碰撞检查", "libm hypot/三角函数（多为碰撞几何内层）", "哈希/容器",
         "势场 Dijkstra", "内存分配", "RS 距离求解/连接", "其它/未分类"]


def extract(html_path: Path, div_id: str):
    html = html_path.read_text(encoding="utf-8")
    m = re.search(r'Plotly\.newPlot\(\s*"' + re.escape(div_id) + r'"\s*,', html)
    assert m, f"未找到 {div_id} in {html_path}"
    i = m.end()
    while html[i] in " \t\r\n":
        i += 1
    dec = json.JSONDecoder()
    data, j = dec.raw_decode(html, i)
    k = j
    while html[k] in " ,\n\r\t":
        k += 1
    layout, _ = dec.raw_decode(html, k)
    return data, layout


print("== CSV 校验 ==")
rows = list(csv.DictReader(open(DATA / "stages_buckets_normalized.csv", encoding="utf-8")))
assert len(rows) == (3 * 3 + 3) * 7, len(rows)
sums = {}
for r in rows:
    eq = float(r["equiv_vs_original_pct"])
    ir, bt = int(r["ir"]), int(r["base_total_ir"])
    if r["case"] != "宏平均":
        expect = int(round(ir / bt * 100 * 10000)) / 10000
        assert abs(expect - eq) < 0.0002, (r, expect, eq)
    sums.setdefault((r["stage"], r["case"]), 0.0)
    sums[(r["stage"], r["case"])] += eq
for case in CASES + ["宏平均"]:
    s = sums[("base", case)]
    assert abs(s - 100.0) < 0.005, (case, s)
    print(f"  base {case:7s} 合计 = {s:.4f}%")
tot = {st: round(sums[(st, "宏平均")], 4) for st in ("base", "hypot", "sincos")}
print("  阶段合计：", tot)
assert abs(tot["hypot"] - 77.02) < 0.05 and abs(tot["sincos"] - 75.91) < 0.05, tot

print("== HTML 校验 ==")
html_h = (OUT / "fig_libm_hypot.html").read_text(encoding="utf-8")
html_s = (OUT / "fig_libm_sincos.html").read_text(encoding="utf-8")
for name, html in (("hypot", html_h), ("sincos", html_s)):
    assert "plotly.js v" in html and 'script src' not in html, name
    assert "原始等效" in html and "3 例等权宏平均（case_30 / case_39 / pb-017）" in html
    data_m, layout_m = extract(OUT / f"fig_libm_{name}.html", "figmain")
    n = 2 if name == "hypot" else 3
    assert len(data_m) == 7 and all(t["type"] == "bar" for t in data_m)
    stage_sum = [sum(t["x"][i] for t in data_m) for i in range(n)]
    print(f"  [{name}] 主图等效合计 =", [round(v, 2) for v in stage_sum])
    if name == "hypot":
        assert abs(stage_sum[0] - 100) < 0.01 and abs(stage_sum[1] - 77.02) < 0.05
    else:
        assert (abs(stage_sum[0] - 100) < 0.01 and abs(stage_sum[1] - 77.02) < 0.05
                and abs(stage_sum[2] - 75.91) < 0.05)
    ann = [a["text"] for a in layout_m.get("annotations", [])]
    print(f"  [{name}] 行尾标注 =", ann)
    assert len(layout_m.get("shapes", [])) == 7
    assert all(any("原始等效" in s for s in t["customdata"]) for t in data_m)
    data_s, layout_s = extract(OUT / f"fig_libm_{name}.html", "figsub")
    assert len(data_s) == 1 and data_s[0]["type"] == "bar"
    xs, ys = data_s[0]["x"], data_s[0]["y"]
    assert xs == sorted(xs), xs
    print(f"  [{name}] 副面板（自下而上）:",
          list(zip(ys, [round(v, 1) for v in xs])))
    print(f"  [{name}] plotlyjs_blocks =", len(re.findall(r"plotly\.js v", html)))

print("\nfig_libm_hypot / fig_libm_sincos 全部校验通过")
