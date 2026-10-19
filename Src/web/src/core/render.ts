import type { LayerState } from './layers';
import type { Pose2d } from './pose';
import { buildPlaybackCommands } from './playback/commands';
import type { PlaybackOverlay } from './playback/overlay';
import { footprintWorldCorners, type VehicleModel } from './scene/vehicle';
import type { SceneObstacle } from './scene/types';
import type { ClippedPath, WirePoint } from './service/types';
import { worldToScreen, type Viewport } from './transform';
import type { BoundsM, MapSnapshot } from './types';

/**
 * 场景 → 绘制命令。核心渲染逻辑与 Canvas 解耦：
 * `buildDrawCommands`（基础图层）与 `buildOverlayCommands`（W2 交互覆盖层）都是
 * 纯函数（可 Node 侧测试 / 导出 SVG / 统计），Canvas 只是命令的一种执行后端
 * （`canvas.ts`），SVG 是另一种（`svg.ts`）。
 */

export type Pt = readonly [number, number];

export type LayerTag =
  | 'areas'
  | 'boundaries'
  | 'centerline'
  | 'selection'
  | 'topology'
  | 'route'
  | 'poses'
  | 'window'
  | 'obstacles'
  | 'vehicle'
  | 'result_path'
  | 'result_curve'
  | 'corridor'
  | 'trajectory'
  | 'collision';

export const ALL_LAYER_TAGS: readonly LayerTag[] = [
  'areas',
  'boundaries',
  'centerline',
  'selection',
  'topology',
  'route',
  'poses',
  'window',
  'obstacles',
  'vehicle',
  'result_path',
  'result_curve',
  'corridor',
  'trajectory',
  'collision',
];

export interface PolylineCommand {
  op: 'polyline';
  layer: LayerTag;
  points: readonly Pt[];
  stroke: string;
  lineWidth: number;
  dash?: readonly number[];
}

export interface PolygonCommand {
  op: 'polygon';
  layer: LayerTag;
  /** 外环 + 内环；配合 `fillRule: 'evenodd'` 实现洞的镂空。 */
  rings: readonly (readonly Pt[])[];
  fill: string;
  fillRule: 'nonzero' | 'evenodd';
  stroke?: string;
  lineWidth?: number;
}

export interface CircleCommand {
  op: 'circle';
  layer: LayerTag;
  cx: number;
  cy: number;
  radius: number;
  fill: string;
}

export interface TextCommand {
  op: 'text';
  layer: LayerTag;
  x: number;
  y: number;
  text: string;
  fill: string;
  fontPx: number;
  align?: 'left' | 'center' | 'right';
}

export type DrawCommand = PolylineCommand | PolygonCommand | CircleCommand | TextCommand;

export const MAP_COLORS = {
  background: '#0b1220',
  boundary: '#c7d2e4',
  centerline: '#f5a623',
  area_fill: 'rgba(56, 142, 255, 0.22)',
  area_stroke: 'rgba(96, 165, 250, 0.9)',
} as const;

/** W2 覆盖层颜色（与基础图层区分；不用于唯一语义，图例见界面）。 */
export const OVERLAY_COLORS = {
  selection: '#22d3ee',
  selection_fill: 'rgba(34, 211, 238, 0.15)',
  following: '#4ade80',
  preceding: '#fb923c',
  adjacent: '#a78bfa',
  route: '#f472b6',
  start_pose: '#22c55e',
  goal_pose: '#ef4444',
  window: '#facc15',
  crop_path: '#67e8f9',
  crop_point: '#a5f3fc',
  obstacle_stroke: '#ef4444',
  obstacle_fill: 'rgba(239, 68, 68, 0.22)',
  obstacle_selected: '#fde047',
  footprint_start_fill: 'rgba(34, 197, 94, 0.20)',
  footprint_goal_fill: 'rgba(239, 68, 68, 0.20)',
  footprint_cursor_fill: 'rgba(56, 189, 248, 0.15)',
  footprint_cursor: '#38bdf8',
} as const;

/** W3 足迹绘制输入（位姿 + 语义标签）。 */
export interface FootprintSpec {
  pose: Pose2d;
  kind: 'start' | 'goal' | 'cursor';
}

/** W2/W3 交互覆盖层输入（ID 列表 + 位姿 + 窗口/裁剪结果 + 障碍/车辆）。 */
export interface MapOverlays {
  selected_lanelets?: readonly string[];
  selected_areas?: readonly string[];
  following?: readonly string[];
  preceding?: readonly string[];
  adjacent?: readonly string[];
  route?: readonly string[];
  start_pose?: Pose2d | null;
  goal_pose?: Pose2d | null;
  /** 窗口查询的矩形（世界坐标；拖拽预览或已提交的查询窗口）。 */
  window?: BoundsM | null;
  /** 服务裁剪结果路径（地图坐标系）。 */
  crop_paths?: readonly ClippedPath[];
  crop_points?: readonly WirePoint[];
  /** W3 测试障碍（覆盖层；不改写地图数据）。 */
  obstacles?: readonly SceneObstacle[];
  selected_obstacle_id?: string | null;
  /** 障碍新建拖拽预览（世界坐标矩形）。 */
  obstacle_preview?: BoundsM | null;
  /** W3 车辆模型与足迹（起终位姿 / 光标）。 */
  vehicle?: VehicleModel | null;
  footprints?: readonly FootprintSpec[];
  /** W4 回放覆盖层（路径/曲线/走廊/轨迹/碰撞；世界坐标）。 */
  playback?: PlaybackOverlay | null;
}

/**
 * 构建一次完整重绘所需的命令序列（自下而上：区域 → 道路边界 → 中心线）。
 * 只输出可见图层；退化几何（点数 < 2 的折线、空环）被跳过。
 */
export function buildDrawCommands(
  snapshot: MapSnapshot,
  viewport: Viewport,
  layers: LayerState,
): DrawCommand[] {
  const commands: DrawCommand[] = [];

  if (layers.areas) {
    for (const area of snapshot.areas) {
      const rings: Pt[][] = [];
      for (const ring of [area.outer, ...area.inner]) {
        if (ring.length === 0) continue;
        rings.push(ring.map((point) => worldToScreen(viewport, point.x_m, point.y_m)));
      }
      if (rings.length === 0) continue;
      commands.push({
        op: 'polygon',
        layer: 'areas',
        rings,
        fill: MAP_COLORS.area_fill,
        fillRule: 'evenodd',
        stroke: MAP_COLORS.area_stroke,
        lineWidth: 1,
      });
    }
  }

  if (layers.boundaries) {
    for (const lanelet of snapshot.lanelets) {
      for (const side of [lanelet.left, lanelet.right]) {
        if (side.length < 2) continue;
        commands.push({
          op: 'polyline',
          layer: 'boundaries',
          points: side.map((point) => worldToScreen(viewport, point.x_m, point.y_m)),
          stroke: MAP_COLORS.boundary,
          lineWidth: 1.5,
        });
      }
    }
  }

  if (layers.centerline) {
    for (const lanelet of snapshot.lanelets) {
      const centerline = lanelet.centerline;
      if (!centerline || centerline.length < 2) continue;
      commands.push({
        op: 'polyline',
        layer: 'centerline',
        points: centerline.map((point) => worldToScreen(viewport, point.x_m, point.y_m)),
        stroke: MAP_COLORS.centerline,
        lineWidth: 1.25,
        dash: [7, 5],
      });
    }
  }

  return commands;
}

/**
 * W2 交互覆盖层命令（在基础场景之上绘制；顺序：窗口 → 裁剪结果 → 拓扑 →
 * 路由 → 选中 → 位姿）。纯函数，输入全部为规范化数据。
 */
export function buildOverlayCommands(
  snapshot: MapSnapshot,
  viewport: Viewport,
  overlays: MapOverlays,
): DrawCommand[] {
  const commands: DrawCommand[] = [];
  const laneletsById = new Map(snapshot.lanelets.map((lanelet) => [lanelet.id, lanelet]));
  const areasById = new Map(snapshot.areas.map((area) => [area.id, area]));

  const pushCenterline = (id: string, stroke: string, lineWidth: number, layer: LayerTag, dash?: number[]): void => {
    const lanelet = laneletsById.get(id);
    const centerline = lanelet?.centerline;
    if (!centerline || centerline.length < 2) return;
    commands.push({
      op: 'polyline',
      layer,
      points: centerline.map((point) => worldToScreen(viewport, point.x_m, point.y_m)),
      stroke,
      lineWidth,
      ...(dash ? { dash } : {}),
    });
  };

  const pushBoundaries = (id: string, stroke: string, lineWidth: number, layer: LayerTag): void => {
    const lanelet = laneletsById.get(id);
    if (!lanelet) return;
    for (const side of [lanelet.left, lanelet.right]) {
      if (side.length < 2) continue;
      commands.push({
        op: 'polyline',
        layer,
        points: side.map((point) => worldToScreen(viewport, point.x_m, point.y_m)),
        stroke,
        lineWidth,
      });
    }
  };

  // 1) 窗口矩形（虚线；拖拽预览与已提交查询共用）
  if (overlays.window) {
    const { min_x_m, min_y_m, max_x_m, max_y_m } = overlays.window;
    const corners: Pt[] = [
      worldToScreen(viewport, min_x_m, min_y_m),
      worldToScreen(viewport, max_x_m, min_y_m),
      worldToScreen(viewport, max_x_m, max_y_m),
      worldToScreen(viewport, min_x_m, max_y_m),
      worldToScreen(viewport, min_x_m, min_y_m),
    ];
    commands.push({
      op: 'polyline',
      layer: 'window',
      points: corners,
      stroke: OVERLAY_COLORS.window,
      lineWidth: 1.5,
      dash: [6, 4],
    });
  }

  // 2) 服务裁剪结果（窗口局部坐标 → 屏幕）
  for (const path of overlays.crop_paths ?? []) {
    if (path.points.length < 2) continue;
    commands.push({
      op: 'polyline',
      layer: 'window',
      points: path.points.map((point) => worldToScreen(viewport, point.x_m, point.y_m)),
      stroke: OVERLAY_COLORS.crop_path,
      lineWidth: 1.5,
    });
  }
  for (const point of overlays.crop_points ?? []) {
    const [sx, sy] = worldToScreen(viewport, point.x_m, point.y_m);
    commands.push({ op: 'circle', layer: 'window', cx: sx, cy: sy, radius: 2, fill: OVERLAY_COLORS.crop_point });
  }

  // 3) 拓扑结果
  for (const id of overlays.following ?? []) pushCenterline(id, OVERLAY_COLORS.following, 3, 'topology');
  for (const id of overlays.preceding ?? []) pushCenterline(id, OVERLAY_COLORS.preceding, 3, 'topology');
  for (const id of overlays.adjacent ?? []) pushCenterline(id, OVERLAY_COLORS.adjacent, 3, 'topology');

  // 4) 路由
  for (const id of overlays.route ?? []) {
    pushCenterline(id, OVERLAY_COLORS.route, 4, 'route');
  }

  // 5) 选中对象
  for (const id of overlays.selected_lanelets ?? []) {
    pushBoundaries(id, OVERLAY_COLORS.selection, 2.5, 'selection');
    pushCenterline(id, OVERLAY_COLORS.selection, 2.5, 'selection');
  }
  for (const id of overlays.selected_areas ?? []) {
    const area = areasById.get(id);
    if (!area) continue;
    const rings: Pt[][] = [];
    for (const ring of [area.outer, ...area.inner]) {
      if (ring.length < 3) continue;
      rings.push(ring.map((point) => worldToScreen(viewport, point.x_m, point.y_m)));
    }
    if (rings.length === 0) continue;
    commands.push({
      op: 'polygon',
      layer: 'selection',
      rings,
      fill: OVERLAY_COLORS.selection_fill,
      fillRule: 'evenodd',
      stroke: OVERLAY_COLORS.selection,
      lineWidth: 2.5,
    });
  }

  // 5.5) W3 测试障碍（覆盖层；红橙色与地图要素区分）
  if (overlays.obstacle_preview) {
    const preview = overlays.obstacle_preview;
    const corners: Pt[] = [
      worldToScreen(viewport, preview.min_x_m, preview.min_y_m),
      worldToScreen(viewport, preview.max_x_m, preview.min_y_m),
      worldToScreen(viewport, preview.max_x_m, preview.max_y_m),
      worldToScreen(viewport, preview.min_x_m, preview.max_y_m),
      worldToScreen(viewport, preview.min_x_m, preview.min_y_m),
    ];
    commands.push({
      op: 'polyline',
      layer: 'obstacles',
      points: corners,
      stroke: OVERLAY_COLORS.obstacle_stroke,
      lineWidth: 1.5,
      dash: [5, 4],
    });
  }
  for (const obstacle of overlays.obstacles ?? []) {
    const selected = overlays.selected_obstacle_id === obstacle.id;
    const stroke = selected ? OVERLAY_COLORS.obstacle_selected : OVERLAY_COLORS.obstacle_stroke;
    const lineWidth = selected ? 3 : 2;
    if (obstacle.type === 'rectangle') {
      const corners: Pt[] = [
        worldToScreen(viewport, obstacle.x_min_m, obstacle.y_min_m),
        worldToScreen(viewport, obstacle.x_max_m, obstacle.y_min_m),
        worldToScreen(viewport, obstacle.x_max_m, obstacle.y_max_m),
        worldToScreen(viewport, obstacle.x_min_m, obstacle.y_max_m),
      ];
      commands.push({
        op: 'polygon',
        layer: 'obstacles',
        rings: [corners],
        fill: OVERLAY_COLORS.obstacle_fill,
        fillRule: 'nonzero',
        stroke,
        lineWidth,
      });
    } else {
      const ring = obstacle.vertices.map((vertex) => worldToScreen(viewport, vertex.x_m, vertex.y_m));
      if (ring.length < 3) continue;
      commands.push({
        op: 'polygon',
        layer: 'obstacles',
        rings: [ring],
        fill: OVERLAY_COLORS.obstacle_fill,
        fillRule: 'nonzero',
        stroke,
        lineWidth,
      });
    }
  }

  // 5.6) W3 整车足迹：车体实线填充 + 安全裕量虚线轮廓
  if (overlays.vehicle) {
    const model = overlays.vehicle;
    for (const footprint of overlays.footprints ?? []) {
      const body = footprintWorldCorners(model, footprint.pose, false).map(([x, y]) =>
        worldToScreen(viewport, x, y),
      );
      const margin = footprintWorldCorners(model, footprint.pose, true).map(([x, y]) =>
        worldToScreen(viewport, x, y),
      );
      const color =
        footprint.kind === 'start'
          ? OVERLAY_COLORS.start_pose
          : footprint.kind === 'goal'
            ? OVERLAY_COLORS.goal_pose
            : OVERLAY_COLORS.footprint_cursor;
      const fill =
        footprint.kind === 'start'
          ? OVERLAY_COLORS.footprint_start_fill
          : footprint.kind === 'goal'
            ? OVERLAY_COLORS.footprint_goal_fill
            : OVERLAY_COLORS.footprint_cursor_fill;
      commands.push({
        op: 'polygon',
        layer: 'vehicle',
        rings: [body],
        fill,
        fillRule: 'nonzero',
        stroke: color,
        lineWidth: 2,
      });
      commands.push({
        op: 'polyline',
        layer: 'vehicle',
        points: [...margin, margin[0]],
        stroke: color,
        lineWidth: 1.5,
        dash: [5, 4],
      });
    }
  }

  // 6) 起终位姿（固定像素尺寸的圆点 + 航向箭头）
  const pushPose = (pose: Pose2d, color: string): void => {
    const [sx, sy] = worldToScreen(viewport, pose.x_m, pose.y_m);
    const handlePx = 30;
    const tipX = sx + Math.cos(pose.yaw_rad) * handlePx;
    const tipY = sy - Math.sin(pose.yaw_rad) * handlePx;
    commands.push({ op: 'circle', layer: 'poses', cx: sx, cy: sy, radius: 6, fill: color });
    commands.push({
      op: 'polyline',
      layer: 'poses',
      points: [
        [sx, sy],
        [tipX, tipY],
      ],
      stroke: color,
      lineWidth: 2,
    });
    const dx = tipX - sx;
    const dy = tipY - sy;
    const length = Math.hypot(dx, dy) || 1;
    const ux = dx / length;
    const uy = dy / length;
    const head = 9;
    const spread = 0.5;
    const leftX = tipX - head * (ux * Math.cos(spread) - uy * Math.sin(spread));
    const leftY = tipY - head * (uy * Math.cos(spread) + ux * Math.sin(spread));
    const rightX = tipX - head * (ux * Math.cos(spread) + uy * Math.sin(spread));
    const rightY = tipY - head * (uy * Math.cos(spread) - ux * Math.sin(spread));
    commands.push({
      op: 'polyline',
      layer: 'poses',
      points: [
        [leftX, leftY],
        [tipX, tipY],
        [rightX, rightY],
      ],
      stroke: color,
      lineWidth: 2,
    });
  };
  if (overlays.start_pose) pushPose(overlays.start_pose, OVERLAY_COLORS.start_pose);
  if (overlays.goal_pose) pushPose(overlays.goal_pose, OVERLAY_COLORS.goal_pose);

  // 7) W4 回放结果（路径/曲线/走廊/轨迹/碰撞；最上层）
  if (overlays.playback) {
    commands.push(...buildPlaybackCommands(overlays.playback, viewport));
  }

  return commands;
}

export interface DrawCommandSummary {
  total: number;
  by_op: { polyline: number; polygon: number; circle: number; text: number };
  by_layer: Record<LayerTag, number>;
  total_points: number;
}

export function summarizeDrawCommands(commands: readonly DrawCommand[]): DrawCommandSummary {
  const byLayer = {} as Record<LayerTag, number>;
  for (const tag of ALL_LAYER_TAGS) byLayer[tag] = 0;
  const summary: DrawCommandSummary = {
    total: commands.length,
    by_op: { polyline: 0, polygon: 0, circle: 0, text: 0 },
    by_layer: byLayer,
    total_points: 0,
  };
  for (const command of commands) {
    summary.by_op[command.op] += 1;
    summary.by_layer[command.layer] += 1;
    if (command.op === 'polyline') {
      summary.total_points += command.points.length;
    } else if (command.op === 'polygon') {
      for (const ring of command.rings) summary.total_points += ring.length;
    } else {
      summary.total_points += 1;
    }
  }
  return summary;
}

/** 检查命令中所有坐标均为有限数值（渲染检查证据用）。 */
export function commandsHaveFiniteCoordinates(commands: readonly DrawCommand[]): boolean {
  const finite = (value: number): boolean => Number.isFinite(value);
  for (const command of commands) {
    if (command.op === 'polyline') {
      for (const [x, y] of command.points) {
        if (!finite(x) || !finite(y)) return false;
      }
    } else if (command.op === 'polygon') {
      for (const ring of command.rings) {
        for (const [x, y] of ring) {
          if (!finite(x) || !finite(y)) return false;
        }
      }
    } else if (command.op === 'text') {
      if (!finite(command.x) || !finite(command.y)) return false;
    } else if (!finite(command.cx) || !finite(command.cy) || !finite(command.radius)) {
      return false;
    }
  }
  return true;
}
