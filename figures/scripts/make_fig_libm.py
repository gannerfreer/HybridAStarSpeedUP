#!/usr/bin/env python3
"""libm 分步提速实验出图（风格沿用 T7 fig1b，双面板）：
  figures/fig_libm_hypot.html   阶段 [原始, +hypot]
  figures/fig_libm_sincos.html  阶段 [原始, +hypot, +hypot +sincos]

口径（原始等效）：equivalent%(stage,bucket,case) = ir(stage,bucket,case) /
total_ir(base,case) * 100，再对 case_30 / case_39 / pb-017 等权宏平均；
原始行合计 = 100.0%（脚本内校验）。副面板"组分进步一览"：末阶段 ÷ 原始。
数据：figures/data/stages_buckets.csv（analyze_profiles.py 生成）。
"""
from __future__ import annotations

import csv
from pathlib import Path

import plotly.graph_objects as go

ROOT = Path("/home/gray/Planner/HybridAStarSpeedUP")
OUT_DIR = ROOT / "figures"
DATA = OUT_DIR / "data"
CASES = ["case_30", "case_39", "pb-017"]

FONT = ('"Noto Sans CJK SC","WenQuanYi Micro Hei","Microsoft YaHei",'
        '"PingFang SC","Source Han Sans SC",sans-serif')
CONFIG = {
    "displaylogo": False,
    "responsive": True,
    "modeBarButtonsToRemove": ["lasso2d", "select2d"],
    "toImageButtonOptions": {"format": "png", "scale": 2},
}
ORDER = [
    "碰撞几何/碰撞检查",
    "libm hypot/三角函数（多为碰撞几何内层）",
    "哈希/容器",
    "势场 Dijkstra",
    "内存分配",
    "RS 距离求解/连接",
    "其它/未分类",
]
SHORT = {
    "碰撞几何/碰撞检查": "碰撞几何/碰撞检查",
    "libm hypot/三角函数（多为碰撞几何内层）": "libm（hypot/三角函数）",
    "哈希/容器": "哈希/容器",
    "势场 Dijkstra": "势场 Dijkstra",
    "内存分配": "内存分配",
    "RS 距离求解/连接": "RS 距离求解/连接",
    "其它/未分类": "其它/未分类",
}
COLOR = {
    "碰撞几何/碰撞检查": "#b03a2e",
    "libm hypot/三角函数（多为碰撞几何内层）": "#e08a1e",
    "哈希/容器": "#2e6da4",
    "势场 Dijkstra": "#2e8b57",
    "内存分配": "#7d3c98",
    "RS 距离求解/连接": "#138d8d",
    "其它/未分类": "#7f8c8d",
}

PAGE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width, initial-scale=1"/>
<title>{title}</title>
<style>
  html, body {{ background: #ffffff; color: #1f1f1f; margin: 0; padding: 0; }}
  .wrap {{ max-width: 1280px; margin: 0 auto; padding: 26px 30px 36px;
           font-family: {font}; }}
  h1 {{ font-size: 22px; margin: 0 0 8px; }}
  p.note {{ font-size: 13px; line-height: 1.75; color: #3d3d3d; margin: 6px 0; }}
  p.src {{ font-size: 12px; line-height: 1.7; color: #767676;
           border-top: 1px solid #e6e6e6; padding-top: 10px; margin-top: 28px; }}
</style>
</head>
<body><div class="wrap">
<h1>{h1}</h1>
{notes}
{plots}
<p class="src">{src}</p>
</div></body></html>
"""


def render(out_path: Path, title: str, h1: str, notes: str, plots: str, src: str):
    out_path.write_text(PAGE.format(title=title, h1=h1, notes=notes, plots=plots,
                                    src=src, font=FONT), encoding="utf-8")
    print(f"[out] {out_path}  ({out_path.stat().st_size/1e6:.2f} MB)")


def load_data(stages: list[tuple[str, str]]):
    stage_ids = {s for s, _ in stages}
    per_ir, per_tot, macro_share, stage_label = {}, {}, {}, {}
    with (DATA / "stages_buckets.csv").open(encoding="utf-8") as fh:
        for r in csv.DictReader(fh):
            stage, case, bucket = r["stage"], r["case"], r["bucket"]
            stage_label[stage] = r["stage_label"]
            if stage not in stage_ids:
                continue
            if case == "宏平均":
                macro_share[(stage, bucket)] = float(r["share_pct"])
            else:
                per_ir[(stage, case, bucket)] = int(r["ir"])
                per_tot[(stage, case)] = int(r["total_ir"])
    equiv = {(s, c, b): per_ir[(s, c, b)] / per_tot[("base", c)] * 100
             for (s, c, b) in per_ir}
    macro_equiv = {(s, b): sum(equiv[(s, c, b)] for c in CASES) / len(CASES)
                   for s, _ in stages for b in ORDER}
    return per_ir, per_tot, equiv, macro_equiv, macro_share, stage_label


def write_normalized_csv(stages, per_ir, per_tot, equiv, macro_equiv, macro_share):
    path = DATA / "stages_buckets_normalized.csv"
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["stage", "stage_label", "case", "bucket", "ir",
                    "equiv_vs_original_pct", "stage_share_pct", "base_total_ir"])
        for stage, label in stages:
            for case in CASES:
                bt, tot = per_tot[("base", case)], per_tot[(stage, case)]
                for b in ORDER:
                    ir = per_ir[(stage, case, b)]
                    w.writerow([stage, label, case, b, ir,
                                round(equiv[(stage, case, b)], 4),
                                round(ir / tot * 100, 4), bt])
            for b in ORDER:
                mean_ir = sum(per_ir[(stage, c, b)] for c in CASES) / len(CASES)
                mean_bt = sum(per_tot[("base", c)] for c in CASES) / len(CASES)
                w.writerow([stage, label, "宏平均", b, round(mean_ir),
                            round(macro_equiv[(stage, b)], 4),
                            round(macro_share[(stage, b)], 4), round(mean_bt)])
    print(f"[out] {path}")


def hover_text(stages, stage_label, stage, bucket, equiv, macro_equiv, macro_share):
    bm = macro_equiv[("base", bucket)]
    cm = macro_equiv[(stage, bucket)]
    ratio = bm / cm if cm > 0 else float("inf")
    e = {c: equiv[(stage, c, bucket)] for c in CASES}
    return (f"{stage_label[stage]} · {SHORT[bucket]}<br>"
            f"原始等效 {bm:.2f}% ｜ 当前等效 {cm:.2f}%<br>"
            f"阶段内占比 {macro_share[(stage, bucket)]:.2f}% ｜ "
            f"缩减 {ratio:.2f}×（原始等效 ÷ 当前等效）<br>"
            f"逐例等效：case_30 {e['case_30']:.2f}% · case_39 {e['case_39']:.2f}% · "
            f"pb-017 {e['pb-017']:.2f}%")


def make_main(stages, stage_label, equiv, macro_equiv, macro_share, subtitle):
    y = [stage_label[s] for s, _ in stages]
    fig = go.Figure()
    for bucket in ORDER:
        xs = [macro_equiv[(s, bucket)] for s, _ in stages]
        custom = [hover_text(stages, stage_label, s, bucket, equiv, macro_equiv,
                             macro_share) for s, _ in stages]
        color = COLOR[bucket]
        txt = "#ffffff" if (0.299 * int(color[1:3], 16) + 0.587 * int(color[3:5], 16)
                            + 0.114 * int(color[5:7], 16)) < 140 else "#1f1f1f"
        fig.add_trace(go.Bar(
            name=SHORT[bucket], orientation="h", y=y, x=xs,
            text=[f"{v:.1f}%" if v >= 1.0 else "" for v in xs],
            textposition="inside", insidetextanchor="middle",
            textfont=dict(size=11.5, color=txt),
            customdata=custom,
            hovertemplate="%{customdata}<extra></extra>",
            marker=dict(color=color, line=dict(color="#ffffff", width=1)),
        ))
    for s, label in stages:
        total = sum(macro_equiv[(s, b)] for b in ORDER)
        fig.add_annotation(x=total + 1.0, y=label, text=f"= {total:.1f}%",
                           showarrow=False, xanchor="left", yanchor="middle",
                           font=dict(size=12.5, color="#333333", family=FONT))
    cum = 0.0
    for bucket in ORDER:
        cum += macro_equiv[("base", bucket)]
        fig.add_shape(type="line", x0=cum, x1=cum, y0=0, y1=1, yref="paper",
                      line=dict(color="#9a9a9a", width=1, dash="dot"),
                      opacity=0.45, layer="above")
    fig.update_layout(
        barmode="stack", bargap=0.42,
        height=430, margin=dict(l=10, r=88, t=54, b=44),
        paper_bgcolor="#ffffff", plot_bgcolor="#ffffff",
        font=dict(family=FONT, size=12.5, color="#1f1f1f"),
        title=dict(text=f"组件时间占比（原始等效）· {subtitle} · 3 例等权宏平均",
                   x=0.008, xanchor="left", font=dict(size=15.5, color="#111111")),
        xaxis=dict(range=[0, 117], ticksuffix="%", gridcolor="#e9e9e9", zeroline=False,
                   title=dict(text="占原始总 Ir 的比例（%；同 case 内以该例原始总 Ir 为分母）",
                              font=dict(size=12))),
        yaxis=dict(categoryorder="array",
                   categoryarray=[stage_label[s] for s, _ in stages][::-1],
                   title=None, automargin=True),
        legend=dict(orientation="h", yanchor="top", y=-0.18, xanchor="left", x=0,
                    font=dict(size=11.5), itemwidth=30, bgcolor="rgba(0,0,0,0)"),
        hoverlabel=dict(font=dict(family=FONT, size=12)),
    )
    return fig


def make_progress(stages, stage_label, equiv, macro_equiv, macro_share, subtitle):
    last = stages[-1][0]
    ratios = {b: macro_equiv[(last, b)] / macro_equiv[("base", b)] * 100 for b in ORDER}
    order = sorted(ORDER, key=lambda b: ratios[b])
    fig = go.Figure(go.Bar(
        orientation="h", y=[SHORT[b] for b in order], x=[ratios[b] for b in order],
        marker=dict(color=[COLOR[b] for b in order],
                    line=dict(color="#ffffff", width=1)),
        text=[f"缩到 {ratios[b]:.0f}%" for b in order],
        textposition="outside", cliponaxis=False,
        textfont=dict(size=11.5, color="#1f1f1f", family=FONT),
        customdata=[hover_text(stages, stage_label, last, b, equiv, macro_equiv,
                               macro_share) for b in order],
        hovertemplate="%{customdata}<extra></extra>", name=f"{subtitle} ÷ 原始",
    ))
    fig.add_shape(type="line", x0=100, x1=100, y0=0, y1=1, yref="paper",
                  line=dict(color="#8a8a8a", width=1, dash="dash"), opacity=0.8)
    xmax = max(118.0, max(ratios.values()) + 16.0)
    fig.update_layout(
        height=430, margin=dict(l=10, r=70, t=54, b=44), bargap=0.42,
        paper_bgcolor="#ffffff", plot_bgcolor="#ffffff",
        font=dict(family=FONT, size=12.5, color="#1f1f1f"),
        title=dict(text=f"组分进步一览（{subtitle} ÷ 原始，缩到 x%；按剩余比例降序）",
                   x=0.008, xanchor="left", font=dict(size=15.5, color="#111111")),
        xaxis=dict(range=[0, xmax], ticksuffix="%", gridcolor="#e9e9e9", zeroline=False,
                   title=dict(text="缩到原始的百分比（100% = 未动，灰虚线；线性轴）",
                              font=dict(size=12))),
        yaxis=dict(automargin=True, title=None),
        hoverlabel=dict(font=dict(family=FONT, size=12)),
        showlegend=False,
    )
    return fig


STAGE_SETS = {
    "hypot": {
        "stages": [("base", "原始"), ("hypot", "+hypot")],
        "subtitle": "阶段：原始 → +hypot（sqrt 快速路径）",
        "file": "fig_libm_hypot.html",
        "title": "libm 分步提速 · 组件占比 · 步骤 1：hypot→sqrt",
        "h1": "fig_libm_hypot · 组件时间占比（原始等效）：hypot→sqrt 快速路径",
    },
    "sincos": {
        "stages": [("base", "原始"), ("hypot", "+hypot"), ("sincos", "+hypot +sincos")],
        "subtitle": "阶段：原始 → +hypot → +hypot +sincos",
        "file": "fig_libm_sincos.html",
        "title": "libm 分步提速 · 组件占比 · 步骤 1+2：hypot→sqrt + sincos 削减",
        "h1": "fig_libm_sincos · 组件时间占比（原始等效）：两步累计",
    },
}

SRC = ('数据：figures/data/stages_buckets.csv（逐例 Ir：profiling/callgrind.<stage>.<case>.out，'
       'valgrind 3.15 callgrind，bench-prof = RelWithDebInfo (-O2 -g)，--time-budget-s 7200、'
       'taskset -c 12、cold-only 单次；三例均完整搜索，节点数 298/20453/910 与本机原版一致）；'
       '归因：/tmp/opencode/libm-lab/analyze_profiles.py（7 桶函数名匹配，三状态同一套映射；'
       '映射按 T1 证据 PROFILING.md 桶定义重建，偏差见 RATES.md）。'
       '阶段树：base = a49bd7a+tooling（320f63c）、hypot = 8845174、sincos = ddf8252。')


def main():
    # 用 sincos 的全集校验原始行合计
    per_ir, per_tot, equiv, macro_equiv, macro_share, stage_label = load_data(
        STAGE_SETS["sincos"]["stages"])
    print("== 校验：原始行合计（应为 100.0%）")
    for case in CASES:
        s = sum(equiv[("base", case, b)] for b in ORDER)
        assert abs(s - 100.0) < 0.005, (case, s)
        print(f"   base {case:7s} = {s:.4f}%")
    s = sum(macro_equiv[("base", b)] for b in ORDER)
    assert abs(s - 100.0) < 0.005, s
    print(f"   base 宏平均   = {s:.4f}%")

    write_normalized_csv(STAGE_SETS["sincos"]["stages"], per_ir, per_tot, equiv,
                         macro_equiv, macro_share)

    for key, cfg in STAGE_SETS.items():
        stages = cfg["stages"]
        print(f"== {key}:",
              {stage_label[st]: f"{sum(macro_equiv[(st, b)] for b in ORDER):.2f}%"
               for st, _ in stages})
        ratios = {b: macro_equiv[(stages[-1][0], b)] / macro_equiv[("base", b)] * 100
                  for b in ORDER}
        for b in sorted(ORDER, key=lambda x: -ratios[x]):
            print(f"   {b[:24]:26s} {macro_equiv[('base', b)]:6.2f}% -> "
                  f"{macro_equiv[(stages[-1][0], b)]:6.2f}%  缩到 {ratios[b]:5.1f}%")

        fig_main = make_main(stages, stage_label, equiv, macro_equiv, macro_share,
                             cfg["subtitle"])
        fig_sub = make_progress(stages, stage_label, equiv, macro_equiv, macro_share,
                                cfg["subtitle"])
        div_main = fig_main.to_html(full_html=False, include_plotlyjs=True,
                                    config=CONFIG, div_id="figmain")
        div_sub = fig_sub.to_html(full_html=False, include_plotlyjs=False,
                                  config=CONFIG, div_id="figsub")

        notes = (
            '<p class="note"><b>口径（原始等效）</b>：x = 该组分 Ir ÷ <b>同一 case 的“原始”总 Ir</b>'
            '× 100，再对 <b>3 例等权宏平均（case_30 / case_39 / pb-017）</b>；'
            '已校验<b>原始行合计 = 100.0%</b>。段长 = 该组分相对原始还剩的“绝对量”，'
            '行尾“= x%”为该阶段等效合计（即该阶段总 Ir 相对原始的比例，指令数口径、非墙钟）。</p>'
            '<p class="note"><b>怎么读</b>：libm 段覆盖 glibc <code>hypot/sincos/sqrt</code> 等数学库符号'
            '（多是碰撞几何内层调用）。① hypot→sqrt 步把 <code>__hypot_finite/hypot</code> 从 33–37% 降到'
            ' sqrt 级（≈1–2%），是主要收益；② sincos 削减步在<b>原始树</b>里只占 ~1.3%（被 hypot 大量掩盖），'
            '约去掉其中 2/3，故总收益 ~1%（T3 重构树里残差 sincos 占 6.7%，该步价值更大）。</p>'
            '<p class="note">浅灰点线 = 原始行各组分累计边界；hover 含：桶名、原始等效、当前等效、'
            '阶段内占比、逐例等效（3 例）、缩减倍数。段内标注 ≥1% 才显示文字（其余靠 hover）。</p>')

        render(OUT_DIR / cfg["file"], cfg["title"], cfg["h1"], notes,
               div_main + "\n" + div_sub, SRC)
    print("完成。")


if __name__ == "__main__":
    main()
