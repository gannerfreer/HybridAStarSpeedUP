import type { DrawCommand, Pt as ScreenPt } from '../render';
import { worldToScreen, type Viewport } from '../transform';
import type { PlaybackOverlay, Pt } from './overlay';

/** W4/W5 覆盖层 → 绘制命令（纯函数；世界坐标 → 屏幕）。 */

export const PLAYBACK_COLORS = {
  forward: '#38bdf8',
  reverse: '#f59e0b',
  cusp: '#f43f5e',
  curve: '#a78bfa',
  control: '#64748b',
  boundary: '#c4b5fd',
  corridor_fill: 'rgba(16, 185, 129, 0.18)',
  corridor_stroke: 'rgba(52, 211, 153, 0.9)',
  corridor_label: '#a7f3d0',
  trajectory_forward: '#60a5fa',
  trajectory_reverse: '#fb923c',
  pose: '#22d3ee',
  start: '#22c55e',
  end: '#ef4444',
  collision_stroke: '#ef4444',
  collision_fill: 'rgba(239, 68, 68, 0.18)',
  overlap_fill: 'rgba(253, 224, 71, 0.35)',
  overlap_stroke: '#fde047',
  footprint: '#38bdf8',
} as const;

function ringCentroid(ring: readonly Pt[]): Pt {
  let x = 0;
  let y = 0;
  for (const point of ring) {
    x += point[0];
    y += point[1];
  }
  return [x / ring.length, y / ring.length];
}

function formatSValue(value: number): string {
  return Number.isInteger(value) ? String(value) : value.toFixed(1);
}

export function buildPlaybackCommands(overlay: PlaybackOverlay, viewport: Viewport): DrawCommand[] {
  const commands: DrawCommand[] = [];
  const screen = (point: Pt): ScreenPt => worldToScreen(viewport, point[0], point[1]);
  // 路径（分段方向着色 + 换向点 + 起终点）
  for (const path of overlay.paths) {
    for (const segment of path.segments) {
      if (segment.points.length < 2) continue;
      commands.push({
        op: 'polyline',
        layer: 'result_path',
        points: segment.points.map(screen),
        stroke: segment.direction === 'reverse' ? PLAYBACK_COLORS.reverse : PLAYBACK_COLORS.forward,
        lineWidth: 3,
      });
    }
    for (const cusp of path.cusps) {
      const [cx, cy] = screen(cusp);
      commands.push({ op: 'circle', layer: 'result_path', cx, cy, radius: 5, fill: PLAYBACK_COLORS.cusp });
    }
    if (path.start) {
      const [sx, sy] = screen(path.start);
      commands.push({ op: 'circle', layer: 'result_path', cx: sx, cy: sy, radius: 5, fill: PLAYBACK_COLORS.start });
    }
    if (path.end) {
      const [ex, ey] = screen(path.end);
      commands.push({ op: 'circle', layer: 'result_path', cx: ex, cy: ey, radius: 5, fill: PLAYBACK_COLORS.end });
    }
  }

  // 曲线（B-spline / 多项式；控制多边形与段边界）
  for (const curve of overlay.curves) {
    if (curve.control_points.length >= 2) {
      commands.push({
        op: 'polyline',
        layer: 'result_curve',
        points: curve.control_points.map(screen),
        stroke: PLAYBACK_COLORS.control,
        lineWidth: 1,
        dash: [4, 4],
      });
    }
    if (curve.points.length >= 2) {
      commands.push({
        op: 'polyline',
        layer: 'result_curve',
        points: curve.points.map(screen),
        stroke: PLAYBACK_COLORS.curve,
        lineWidth: 2.5,
      });
    }
    for (const boundary of curve.boundaries) {
      const [bx, by] = screen(boundary);
      commands.push({ op: 'circle', layer: 'result_curve', cx: bx, cy: by, radius: 4, fill: PLAYBACK_COLORS.boundary });
    }
  }

  // 走廊（半透明凸多边形 + s 区间文字标注，web-002）
  for (const corridor of overlay.corridors) {
    for (const polygon of corridor.polygons) {
      if (polygon.ring.length < 3) continue;
      commands.push({
        op: 'polygon',
        layer: 'corridor',
        rings: [polygon.ring.map(screen)],
        fill: PLAYBACK_COLORS.corridor_fill,
        fillRule: 'nonzero',
        stroke: PLAYBACK_COLORS.corridor_stroke,
        lineWidth: 1.5,
      });
      const [labelX, labelY] = screen(ringCentroid(polygon.ring));
      commands.push({
        op: 'text',
        layer: 'corridor',
        x: labelX,
        y: labelY,
        text: `s ${formatSValue(polygon.s_start_m)}–${formatSValue(polygon.s_end_m)} m`,
        fill: PLAYBACK_COLORS.corridor_label,
        fontPx: 11,
        align: 'center',
      });
    }
  }

  // 轨迹（按速度符号着色 + 点列 + 当前回放位姿）
  for (const trajectory of overlay.trajectories) {
    let run: ScreenPt[] = [];
    let runSign = 0;
    const flush = () => {
      if (run.length >= 2) {
        commands.push({
          op: 'polyline',
          layer: 'trajectory',
          points: run,
          stroke: runSign >= 0 ? PLAYBACK_COLORS.trajectory_forward : PLAYBACK_COLORS.trajectory_reverse,
          lineWidth: 2.5,
        });
      }
      run = [];
    };
    for (const point of trajectory.points) {
      const sign = point.velocity_mps >= 0 ? 1 : -1;
      if (runSign !== 0 && sign !== runSign) flush();
      runSign = sign;
      run.push(screen(point.p));
      commands.push({
        op: 'circle',
        layer: 'trajectory',
        cx: screen(point.p)[0],
        cy: screen(point.p)[1],
        radius: 2.5,
        fill: sign >= 0 ? PLAYBACK_COLORS.trajectory_forward : PLAYBACK_COLORS.trajectory_reverse,
      });
    }
    flush();
  }
  if (overlay.playback_pose) {
    const [px, py] = worldToScreen(viewport, overlay.playback_pose.x_m, overlay.playback_pose.y_m);
    const tipX = px + Math.cos(overlay.playback_pose.yaw_rad) * 22;
    const tipY = py - Math.sin(overlay.playback_pose.yaw_rad) * 22;
    commands.push({ op: 'circle', layer: 'trajectory', cx: px, cy: py, radius: 6, fill: PLAYBACK_COLORS.pose });
    commands.push({
      op: 'polyline',
      layer: 'trajectory',
      points: [
        [px, py],
        [tipX, tipY],
      ],
      stroke: PLAYBACK_COLORS.pose,
      lineWidth: 2,
    });
  }

  // 碰撞样例（足迹 + 障碍 + 重叠区高亮）
  if (overlay.collision) {
    const collision = overlay.collision;
    if (collision.obstacle.length >= 3) {
      commands.push({
        op: 'polygon',
        layer: 'collision',
        rings: [collision.obstacle.map(screen)],
        fill: PLAYBACK_COLORS.collision_fill,
        fillRule: 'nonzero',
        stroke: PLAYBACK_COLORS.collision_stroke,
        lineWidth: 2,
      });
    }
    if (collision.overlap && collision.overlap.length >= 3) {
      commands.push({
        op: 'polygon',
        layer: 'collision',
        rings: [collision.overlap.map(screen)],
        fill: PLAYBACK_COLORS.overlap_fill,
        fillRule: 'nonzero',
        stroke: PLAYBACK_COLORS.overlap_stroke,
        lineWidth: 2,
      });
    }
    if (collision.footprint.length >= 2) {
      commands.push({
        op: 'polyline',
        layer: 'collision',
        points: [...collision.footprint.map(screen), screen(collision.footprint[0])],
        stroke: PLAYBACK_COLORS.footprint,
        lineWidth: 2,
      });
    }
  }

  return commands;
}
