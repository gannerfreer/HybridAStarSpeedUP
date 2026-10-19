#!/usr/bin/env python3
"""停车基准 115 场景：4 个 HybridA* 后端的逐 case 对比图（Plotly HTML）。

输入：
    --fixtures-dir  夹具目录（默认 tests/fixtures/v8，读 cases 与车辆）
    --run-dir       基准运行输出目录（含 benchmark_metrics.csv 与
                    paths/<case_id>/<backend>.json —— 由
                    planner_hybrid_astar_bench --suite v3 --export-paths-dir 生成）
    --output-dir    输出目录（case_<i>.html × 115 + index.html + plotly.min.js）

每张图：
    * 场景层：障碍多边形（灰）、论文画布 [-20,20]² 虚线（可关）、起终点位姿（含朝向短线）；
    * 路径层：4 个后端路径固定配色叠加（图例可开关、悬停显示后端与坐标）；
    * 车辆框层：沿每条轨迹**每个节点**绘制一个虚拟车体矩形（默认步长 1，
      `--box-stride` 可抽稀；尺寸取夹具车辆的 vehicle_ref；颜色随所属后端），
      与后端共享图例组（点击图例联动开关），并支持一键隐藏/显示（Plotly
      updatemenu 按钮）；
    * 指标层：每后端墙钟时间分组条形图；图下附状态/耗时/路径长/换挡/扩展节点表格。

依赖：Python 3 + plotly（本机 6.7；目录模式写出单个共享 plotly.min.js，
各 HTML 体积小且离线可看）。本工具为实验辅助（用户指定用 Plotly 绘图），
不属于 E0 标准库工具集。
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import sys
from pathlib import Path

import plotly
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# 后端固定顺序与配色（跨全部 case 一致）。
BACKEND_ORDER = [
    "ros_free",
    "kth_normalized",
    "zm0612_normalized",
    "smac_normalized",
]
BACKEND_COLORS = {
    "ros_free": "#1f77b4",
    "kth_normalized": "#2ca02c",
    "zm0612_normalized": "#ff7f0e",
    "smac_normalized": "#d62728",
}
PAPER_CANVAS = {"x_min": -20.0, "x_max": 20.0, "y_min": -20.0, "y_max": 20.0}


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def load_cases_document(fixtures_dir: Path, cases_file: str):
    return read_json(fixtures_dir / "paths" / cases_file)


def load_vehicle(fixtures_dir: Path, cases_document):
    reference = cases_document.get("vehicle_ref") or {}
    file_name = reference.get("file")
    if not file_name:
        return None
    path = fixtures_dir / file_name
    if not path.is_file():
        return None
    return read_json(path)


def footprint_corners(x, y, yaw, vehicle):
    """后轴中心位姿 → 车体矩形四角（世界坐标，逆时针）。"""
    margin = vehicle.get("safety_margin_m", 0.0)
    rear = vehicle["rear_overhang_m"] + margin
    front = vehicle["wheelbase_m"] + vehicle["front_overhang_m"] + margin
    half = vehicle["width_m"] / 2.0 + margin
    local = [(-rear, -half), (front, -half), (front, half), (-rear, half)]
    cos_yaw, sin_yaw = math.cos(yaw), math.sin(yaw)
    return [
        (x + cos_yaw * px - sin_yaw * py, y + sin_yaw * px + cos_yaw * py)
        for px, py in local
    ]


def path_pose_samples(document, stride=1):
    """路径位姿序列用于车辆框：默认**每个节点**一个（stride=1），压缩相邻重复
    位姿（换向尖点会在两分段间重复同一姿态）；stride>1 时按步长抽稀。"""
    poses = []
    for segment in document.get("path", []):
        for point in segment.get("points", []):
            pose = point["pose"]
            entry = (pose["x_m"], pose["y_m"], pose["yaw_rad"])
            if poses:
                last = poses[-1]
                if (abs(entry[0] - last[0]) <= 1e-9
                        and abs(entry[1] - last[1]) <= 1e-9
                        and abs(entry[2] - last[2]) <= 1e-9):
                    continue
            poses.append(entry)
    if stride > 1:
        sampled = poses[::stride]
        if poses and sampled and sampled[-1] != poses[-1]:
            sampled.append(poses[-1])
        poses = sampled
    return poses


def load_metrics(run_dir: Path):
    """返回 {(case_id, backend_id): row}；同一键取 repeat_id 最大者。"""
    rows = {}
    with open(run_dir / "benchmark_metrics.csv", newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            if row.get("cache_mode") != "cold":
                continue
            key = (row["case_id"], row["backend_id"])
            previous = rows.get(key)
            if previous is None or int(row["repeat_id"]) >= int(previous["repeat_id"]):
                rows[key] = row
    return rows


def load_path(run_dir: Path, case_id: str, backend_id: str):
    path = run_dir / "paths" / case_id / f"{backend_id}.json"
    if not path.is_file():
        return None
    return read_json(path)


def path_polyline(document):
    """plan_result JSON -> [(x, y)] 折线（跨分段连续）。"""
    points = []
    for segment in document.get("path", []):
        for point in segment.get("points", []):
            pose = point["pose"]
            points.append((pose["x_m"], pose["y_m"]))
    return points


def heading_segment(x, y, yaw, length=1.0):
    return [x, x + length * math.cos(yaw)], [y, y + length * math.sin(yaw)]


def fmt(value, digits=1, suffix=""):
    if value in (None, ""):
        return "—"
    try:
        return f"{float(value):.{digits}f}{suffix}"
    except (TypeError, ValueError):
        return str(value)


def backend_label(backend_id):
    return backend_id.replace("_normalized", "").replace("_", " ")


def figure_subtitle(case, metrics, paths):
    parts = []
    for backend in BACKEND_ORDER:
        row = metrics.get((case["id"], backend))
        if row is None:
            parts.append(f"{backend_label(backend)}：未记录")
        elif row["status"] == "success" and row.get("valid") == "true":
            parts.append(f"{backend_label(backend)}：成功")
        else:
            reason = row.get("failure_reason") or row["status"]
            parts.append(f"{backend_label(backend)}：{row['status']}")
    return " ｜ ".join(parts)


def build_case_figure(case, metrics, paths_by_backend, paper_canvas=True,
                      vehicle=None, box_stride=1):
    box_indices = []
    map_title = ("路径叠加（论文画布 [-20,20]² 虚线）" if paper_canvas
                 else "路径叠加（灰色 = 墙体/车位分隔线）")
    fig = make_subplots(
        rows=2,
        cols=1,
        row_heights=[0.76, 0.24],
        vertical_spacing=0.06,
        subplot_titles=(map_title, "实测墙钟时间（单次冷启动）"),
    )

    # 障碍多边形。
    for index, obstacle in enumerate(case["obstacles"]):
        vertices = obstacle["vertices"]
        xs = [vertex["x_m"] for vertex in vertices] + [vertices[0]["x_m"]]
        ys = [vertex["y_m"] for vertex in vertices] + [vertices[0]["y_m"]]
        fig.add_trace(
            go.Scatter(
                x=xs,
                y=ys,
                fill="toself",
                fillcolor="rgba(120,120,120,0.35)",
                line={"color": "rgba(80,80,80,0.9)", "width": 1},
                name="障碍",
                legendgroup="scene",
                showlegend=index == 0,
                hoverinfo="skip",
            ),
            row=1,
            col=1,
        )

    # 论文画布虚线（v3 停车套件固定为 [-20,20]²；ParkBench 场景不适用）。
    if paper_canvas:
        fig.add_shape(
            type="rect",
            x0=PAPER_CANVAS["x_min"],
            y0=PAPER_CANVAS["y_min"],
            x1=PAPER_CANVAS["x_max"],
            y1=PAPER_CANVAS["y_max"],
            line={"color": "#999999", "width": 1.5, "dash": "dash"},
            row=1,
            col=1,
        )

    # 起点 / 终点（含朝向短线）。
    for label, pose, color, symbol, size in (
        ("起点", case["start"], "#2ca02c", "triangle-up", 14),
        ("终点", case["goal"], "#d62728", "star", 14),
    ):
        fig.add_trace(
            go.Scatter(
                x=[pose["x_m"]],
                y=[pose["y_m"]],
                mode="markers",
                marker={"color": color, "size": size, "symbol": symbol,
                        "line": {"color": "white", "width": 1}},
                name=label,
                legendgroup="scene",
                hovertemplate=f"{label} x=%{{x:.2f}}, y=%{{y:.2f}}<extra></extra>",
            ),
            row=1,
            col=1,
        )
        hx, hy = heading_segment(pose["x_m"], pose["y_m"], pose["yaw_rad"])
        fig.add_trace(
            go.Scatter(
                x=hx,
                y=hy,
                mode="lines",
                line={"color": color, "width": 2},
                showlegend=False,
                hoverinfo="skip",
            ),
            row=1,
            col=1,
        )

    # 4 个后端的路径。
    for backend in BACKEND_ORDER:
        document = paths_by_backend.get(backend)
        points = path_polyline(document) if document else []
        row = metrics.get((case["id"], backend))
        status = row["status"] if row else "missing"
        valid = row and row.get("valid") == "true" and status == "success"
        legend = f"{backend_label(backend)}"
        if points:
            fig.add_trace(
                go.Scatter(
                    x=[p[0] for p in points],
                    y=[p[1] for p in points],
                    mode="lines+markers",
                    line={"color": BACKEND_COLORS[backend], "width": 2.5},
                    marker={"color": BACKEND_COLORS[backend], "size": 3},
                    name=legend,
                    legendgroup=backend,
                    hovertemplate=(
                        backend
                        + "<br>x=%{x:.2f}, y=%{y:.2f}<extra></extra>"
                    ),
                ),
                row=1,
                col=1,
            )
        # 虚拟车辆框：默认**每个路径节点**一个位姿画车体矩形；与所属后端同一
        # 图例组（点击后端图例即联动开关；另有“隐藏/显示车辆框”一键按钮）。
        if points and vehicle is not None:
            box_x, box_y = [], []
            for pose in path_pose_samples(document, box_stride):
                corners = footprint_corners(pose[0], pose[1], pose[2], vehicle)
                for corner_x, corner_y in corners + [corners[0]]:
                    box_x.append(corner_x)
                    box_y.append(corner_y)
                box_x.append(None)
                box_y.append(None)
            box_indices.append(len(fig.data))
            fig.add_trace(
                go.Scatter(
                    x=box_x,
                    y=box_y,
                    mode="lines",
                    line={"color": BACKEND_COLORS[backend], "width": 1},
                    name=f"{backend_label(backend)} 车辆框",
                    legendgroup=backend,
                    showlegend=False,
                    hoverinfo="skip",
                ),
                row=1,
                col=1,
            )
        if row is not None:
            fig.add_trace(
                go.Bar(
                    x=[backend_label(backend)],
                    y=[float(row["wall_ms"])],
                    marker_color=BACKEND_COLORS[backend],
                    name=legend,
                    legendgroup=backend,
                    showlegend=False,
                    customdata=[[
                        status,
                        row.get("failure_reason", ""),
                        fmt(row.get("path_length_m"), 2, " m"),
                        row.get("gear_changes", "—"),
                        row.get("expanded_nodes", "—"),
                    ]],
                    hovertemplate=(
                        "%{x}<br>wall=%{y:.0f} ms<br>status=%{customdata[0]}"
                        "<br>%{customdata[1]}<br>path=%{customdata[2]}"
                        "<br>gears=%{customdata[3]}<br>expanded=%{customdata[4]}"
                        "<extra></extra>"
                    ),
                    opacity=0.9 if valid else 0.45,
                ),
                row=2,
                col=1,
            )

    margin = 1.0
    if paper_canvas:
        x_range = [PAPER_CANVAS["x_min"] - margin, PAPER_CANVAS["x_max"] + margin]
        y_range = [PAPER_CANVAS["y_min"] - margin, PAPER_CANVAS["y_max"] + margin]
    else:
        grid = case["grid"]
        x_origin = grid["origin"]["x_m"]
        y_origin = grid["origin"]["y_m"]
        x_range = [x_origin - 0.5,
                   x_origin + grid["width"] * grid["resolution_m"] + 0.5]
        y_range = [y_origin - 0.5,
                   y_origin + grid["height"] * grid["resolution_m"] + 0.5]
    fig.update_xaxes(
        range=x_range,
        scaleanchor="y",
        scaleratio=1,
        row=1,
        col=1,
    )
    fig.update_yaxes(
        range=y_range,
        row=1,
        col=1,
    )
    fig.update_yaxes(title_text="ms", row=2, col=1)
    layout = {
        "height": 980,
        "margin": {"l": 40, "r": 20, "t": 90, "b": 40},
        "legend": {
            "orientation": "h",
            "yanchor": "bottom",
            "y": 1.04,
            "x": 0.0,
            "font": {"size": 12},
        },
        "bargap": 0.35,
    }
    if box_indices:
        box_set = set(box_indices)
        visible_all = [True] * len(fig.data)
        visible_no_boxes = [index not in box_set
                            for index in range(len(fig.data))]
        layout["updatemenus"] = [{
            "type": "buttons",
            "direction": "right",
            "x": 1.0,
            "y": 1.16,
            "xanchor": "right",
            "yanchor": "top",
            "showactive": False,
            "bgcolor": "rgba(255,255,255,0.85)",
            "bordercolor": "#999999",
            "font": {"size": 12},
            "buttons": [
                {"label": "隐藏车辆框",
                 "method": "update",
                 "args": [{"visible": visible_no_boxes}]},
                {"label": "显示车辆框",
                 "method": "update",
                 "args": [{"visible": visible_all}]},
            ],
        }]
    fig.update_layout(**layout)
    return fig


def metrics_table_html(case, metrics):
    header = (
        "<tr><th>后端</th><th>状态</th><th>原因</th><th>墙钟 ms</th>"
        "<th>路径长 m</th><th>换挡</th><th>扩展节点</th></tr>"
    )
    rows = []
    for backend in BACKEND_ORDER:
        row = metrics.get((case["id"], backend))
        color = BACKEND_COLORS[backend]
        if row is None:
            rows.append(
                f"<tr><td style='color:{color}'>{html.escape(backend)}</td>"
                "<td colspan='6'>未记录</td></tr>"
            )
            continue
        status = row["status"]
        if status == "success" and row.get("valid") == "true":
            status_html = "<td style='color:#2ca02c'>success</td>"
        else:
            status_html = f"<td style='color:#d62728'>{html.escape(status)}</td>"
        rows.append(
            "<tr><td style='color:%s'>%s</td>%s<td>%s</td><td>%s</td>"
            "<td>%s</td><td>%s</td><td>%s</td></tr>"
            % (
                color,
                html.escape(backend),
                status_html,
                html.escape(row.get("failure_reason", "") or "—"),
                fmt(row.get("wall_ms"), 1),
                fmt(row.get("path_length_m"), 2),
                html.escape(row.get("gear_changes", "—") or "—"),
                html.escape(row.get("expanded_nodes", "—") or "—"),
            )
        )
    return "<table class='metrics'>%s%s</table>" % (header, "".join(rows))


def assemble_html(title, subtitle, divs, script_src):
    return (
        "<!DOCTYPE html>\n<html lang='zh'>\n<head>\n<meta charset='utf-8'>\n"
        f"<title>{html.escape(title)}</title>\n"
        "<style>body{font-family:sans-serif;margin:16px;color:#222}"
        ".metrics{border-collapse:collapse;margin-top:8px;font-size:13px}"
        ".metrics th,.metrics td{border:1px solid #ccc;padding:4px 10px;text-align:left}"
        ".metrics th{background:#f2f2f2}"
        "h1{font-size:20px} h2{font-size:15px;margin-top:4px;font-weight:normal}"
        "a{color:#1f77b4}</style>\n"
        f"<script src='{script_src}'></script>\n</head>\n<body>\n"
        f"<h1>{html.escape(title)}</h1>\n<h2>{html.escape(subtitle)}</h2>\n"
        + "\n".join(divs)
        + "\n</body>\n</html>\n"
    )


def write_shared_plotlyjs(output_dir: Path):
    target = output_dir / "plotly.min.js"
    if not target.is_file() or target.stat().st_size == 0:
        target.write_text(plotly.offline.get_plotlyjs(), encoding="utf-8")


def generate(args):
    fixtures_dir = Path(args.fixtures_dir).resolve()
    run_dir = Path(args.run_dir).resolve()
    output_dir = Path(args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    cases_document = load_cases_document(fixtures_dir, args.cases_file)
    cases = cases_document["cases"]
    vehicle = load_vehicle(fixtures_dir, cases_document)
    metrics = load_metrics(run_dir)
    only = set(args.only_case or [])
    paper_canvas = not args.no_paper_canvas

    write_shared_plotlyjs(output_dir)
    print(f"共享 plotly.min.js 就绪；共 {len(cases)} 个场景"
          + ("；车辆框 = %s" % vehicle["id"] if vehicle else "；未加载车辆（无车辆框）"))

    index_rows = []
    for case in cases:
        case_id = case["id"]
        if only and case_id not in only:
            continue
        paths_by_backend = {
            backend: load_path(run_dir, case_id, backend)
            for backend in BACKEND_ORDER
        }
        figure = build_case_figure(case, metrics, paths_by_backend, paper_canvas,
                                   vehicle, args.box_stride)
        div = figure.to_html(
            include_plotlyjs=False,
            full_html=False,
            div_id="fig",
            config={"displaylogo": False, "responsive": True},
        )
        title = f"停车基准对比：{case_id}"
        subtitle = figure_subtitle(case, metrics, paths_by_backend)
        page = assemble_html(
            title,
            subtitle,
            [div, metrics_table_html(case, metrics)],
            "plotly.min.js",
        )
        (output_dir / f"{case_id}.html").write_text(page, encoding="utf-8")

        badges = []
        for backend in BACKEND_ORDER:
            row = metrics.get((case_id, backend))
            if row is None:
                badges.append("<td>—</td>")
            elif row["status"] == "success" and row.get("valid") == "true":
                badges.append("<td style='color:#2ca02c'>✔</td>")
            else:
                badges.append(
                    "<td style='color:#d62728'>✘ %s</td>"
                    % html.escape(row["status"])
                )
        index_rows.append(
            "<tr><td><a href='%s.html'>%s</a></td>%s</tr>"
            % (case_id, case_id, "".join(badges))
        )

    # 汇总统计。
    summary_counts = {backend: [0, 0] for backend in BACKEND_ORDER}
    for case in cases:
        if only and case["id"] not in only:
            continue
        for backend in BACKEND_ORDER:
            row = metrics.get((case["id"], backend))
            if row is None:
                continue
            if row["status"] == "success" and row.get("valid") == "true":
                summary_counts[backend][0] += 1
            else:
                summary_counts[backend][1] += 1
    summary_bits = "；".join(
        "%s 成功 %d / 失败 %d" % (backend, *summary_counts[backend])
        for backend in BACKEND_ORDER
    )
    index_html = (
        "<!DOCTYPE html>\n<html lang='zh'>\n<head>\n<meta charset='utf-8'>\n"
        "<title>停车基准 115 场景对比图索引</title>\n"
        "<style>body{font-family:sans-serif;margin:16px}table{border-collapse:collapse}"
        "th,td{border:1px solid #ccc;padding:3px 8px;font-size:13px}"
        "th{background:#f2f2f2}</style>\n</head>\n<body>\n"
        "<h1>停车基准 115 场景 × 4 个 HybridA* 后端</h1>\n"
        f"<p>{html.escape(summary_bits)}</p>\n"
        "<table><tr><th>case</th>"
        + "".join(f"<th>{backend}</th>" for backend in BACKEND_ORDER)
        + "</tr>\n"
        + "\n".join(index_rows)
        + "\n</table>\n</body>\n</html>\n"
    )
    (output_dir / "index.html").write_text(index_html, encoding="utf-8")
    print(f"写出 {len(index_rows)} 个 case HTML + index.html -> {output_dir}")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--fixtures-dir",
        default=str(Path(__file__).resolve().parents[1] / "tests" / "fixtures" / "v8"),
        help="夹具目录（v3 用 v8；v4 ParkBench 用 v9）",
    )
    parser.add_argument(
        "--cases-file",
        default="cases_bench_parking.json",
        help="夹具 paths/ 下的场景文件（v4 用 cases_bench_parkbench.json）",
    )
    parser.add_argument(
        "--no-paper-canvas",
        action="store_true",
        help="不绘制论文画布 [-20,20]² 虚线、坐标范围按夹具画布自适应"
        "（ParkBench 场景用）",
    )
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument(
        "--box-stride",
        type=int,
        default=1,
        help="车辆框位姿步长（默认 1 = 每个轨迹节点一个；>1 抽稀）",
    )
    parser.add_argument(
        "--only-case",
        action="append",
        default=None,
        help="只生成指定 case（可重复；默认全部）",
    )
    args = parser.parse_args(argv)
    return generate(args)


if __name__ == "__main__":
    sys.exit(main())
