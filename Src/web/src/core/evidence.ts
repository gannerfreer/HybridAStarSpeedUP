import { DEFAULT_LAYERS } from './layers';
import {
  buildDrawCommands,
  commandsHaveFiniteCoordinates,
  MAP_COLORS,
  summarizeDrawCommands,
  type DrawCommand,
} from './render';
import { chooseScaleBar } from './scalebar';
import { computeSnapshotStats, type SnapshotStats } from './stats';
import { drawCommandsToSvg } from './svg';
import {
  boundsToScreenRect,
  metersPerPixel,
  screenToWorld,
  worldToScreen,
  type ScreenRect,
  type Viewport,
} from './transform';
import { fitBounds } from './viewport';
import type { BoundsM, MapSnapshot } from './types';

/**
 * 渲染检查证据（确定性，无时间戳）：对官方示例/测试地图快照输出
 * 对象计数、包围盒、取景比例、比例尺与绘制命令统计。
 */

export interface SnapshotSourceDescriptor {
  rel_path: string;
  kind: string;
  sha256: string;
  bytes: number;
}

export interface RenderCheckOptions {
  width_px?: number;
  height_px?: number;
  padding_px?: number;
  scale_bar_max_width_px?: number;
}

export interface RenderCheckReport {
  schema_version: 1;
  type: 'web_render_check';
  generated_by: string;
  source: SnapshotSourceDescriptor;
  map: { map_id: string; map_version: string };
  counts: SnapshotStats;
  bounds_m: BoundsM;
  canvas: { width_px: number; height_px: number; padding_px: number };
  viewport: {
    scale_px_per_m: number;
    meters_per_pixel: number;
    center_x_m: number;
    center_y_m: number;
  };
  screen_bounds: ScreenRect;
  scale_bar: { length_m: number; width_px: number; label: string; max_width_px: number };
  layers: { areas: number; boundaries: number; centerline: number };
  commands: {
    total: number;
    by_op: { polyline: number; polygon: number; circle: number };
    total_points: number;
  };
  checks: {
    bounds_within_canvas: boolean;
    roundtrip_max_error_m: number;
    roundtrip_ok: boolean;
    roundtrip_max_error_px: number;
    roundtrip_px_ok: boolean;
    scale_bar_width_within_limit: boolean;
    counts_match_top_level: boolean;
    commands_all_finite: boolean;
    all_layers_nonempty: boolean;
  };
}

export interface RenderCheckArtifacts {
  report: RenderCheckReport;
  svg: string;
  commands: DrawCommand[];
  viewport: Viewport;
}

export function buildRenderCheckArtifacts(
  snapshot: MapSnapshot,
  source: SnapshotSourceDescriptor,
  options: RenderCheckOptions = {},
): RenderCheckArtifacts {
  const widthPx = options.width_px ?? 1280;
  const heightPx = options.height_px ?? 720;
  const paddingPx = options.padding_px ?? 24;
  const maxBarWidthPx = options.scale_bar_max_width_px ?? 140;

  const viewport = fitBounds(snapshot.bounds_m, widthPx, heightPx, paddingPx);
  const stats = computeSnapshotStats(snapshot);
  const commands = buildDrawCommands(snapshot, viewport, { ...DEFAULT_LAYERS });
  const summary = summarizeDrawCommands(commands);
  const screenBounds = boundsToScreenRect(snapshot.bounds_m, viewport);
  const scaleBar = chooseScaleBar(metersPerPixel(viewport), maxBarWidthPx);

  // 米制往返：世界角点 → 屏幕 → 世界。
  const bounds = snapshot.bounds_m;
  const worldProbes: Array<[number, number]> = [
    [bounds.min_x_m, bounds.min_y_m],
    [bounds.min_x_m, bounds.max_y_m],
    [bounds.max_x_m, bounds.min_y_m],
    [bounds.max_x_m, bounds.max_y_m],
    [(bounds.min_x_m + bounds.max_x_m) / 2, (bounds.min_y_m + bounds.max_y_m) / 2],
  ];
  let roundtripMaxErrorM = 0;
  for (const [x, y] of worldProbes) {
    const [sx, sy] = worldToScreen(viewport, x, y);
    const [rx, ry] = screenToWorld(viewport, sx, sy);
    roundtripMaxErrorM = Math.max(roundtripMaxErrorM, Math.abs(rx - x), Math.abs(ry - y));
  }

  // 像素往返：屏幕点 → 世界 → 屏幕（浏览器选点回写场景坐标的验收依据，容差 ≤ 1 px）。
  const pixelProbes: Array<[number, number]> = [
    [0, 0],
    [widthPx, 0],
    [0, heightPx],
    [widthPx, heightPx],
    [widthPx / 2, heightPx / 2],
  ];
  let roundtripMaxErrorPx = 0;
  for (const [sx, sy] of pixelProbes) {
    const [x, y] = screenToWorld(viewport, sx, sy);
    const [rx, ry] = worldToScreen(viewport, x, y);
    roundtripMaxErrorPx = Math.max(roundtripMaxErrorPx, Math.abs(rx - sx), Math.abs(ry - sy));
  }

  const epsilon = 1e-6;
  const countsMatch =
    stats.lanelets === snapshot.counts.lanelets &&
    stats.areas === snapshot.counts.areas &&
    stats.regulatory_elements === snapshot.counts.regulatory_elements;

  const report: RenderCheckReport = {
    schema_version: 1,
    type: 'web_render_check',
    generated_by: 'Src/web/src/core/evidence.ts',
    source,
    map: { map_id: snapshot.map_id, map_version: snapshot.map_version },
    counts: stats,
    bounds_m: bounds,
    canvas: { width_px: widthPx, height_px: heightPx, padding_px: paddingPx },
    viewport: {
      scale_px_per_m: viewport.scale_px_per_m,
      meters_per_pixel: metersPerPixel(viewport),
      center_x_m: viewport.center_x_m,
      center_y_m: viewport.center_y_m,
    },
    screen_bounds: screenBounds,
    scale_bar: { ...scaleBar, max_width_px: maxBarWidthPx },
    layers: summary.by_layer,
    commands: { total: summary.total, by_op: summary.by_op, total_points: summary.total_points },
    checks: {
      bounds_within_canvas:
        screenBounds.min_x_px >= -epsilon &&
        screenBounds.min_y_px >= -epsilon &&
        screenBounds.max_x_px <= widthPx + epsilon &&
        screenBounds.max_y_px <= heightPx + epsilon,
      roundtrip_max_error_m: roundtripMaxErrorM,
      roundtrip_ok: roundtripMaxErrorM <= 1e-6,
      roundtrip_max_error_px: roundtripMaxErrorPx,
      roundtrip_px_ok: roundtripMaxErrorPx <= 1e-6,
      scale_bar_width_within_limit: scaleBar.width_px <= maxBarWidthPx + epsilon,
      counts_match_top_level: countsMatch,
      commands_all_finite: commandsHaveFiniteCoordinates(commands),
      all_layers_nonempty:
        summary.by_layer.areas > 0 && summary.by_layer.boundaries > 0 && summary.by_layer.centerline > 0,
    },
  };

  const svg = drawCommandsToSvg(commands, widthPx, heightPx, MAP_COLORS.background);
  return { report, svg, commands, viewport };
}
