import { describe, expect, it } from 'vitest';
import { buildPlaybackCommands, PLAYBACK_COLORS } from '../src/core/playback/commands';
import { buildPlaybackOverlay, playbackDuration, playbackProfile } from '../src/core/playback/overlay';
import { DEFAULT_PLAYBACK_LAYERS, type PlaybackSampleData } from '../src/core/playback/types';
import { summarizeDrawCommands, commandsHaveFiniteCoordinates, type DrawCommand } from '../src/core/render';
import { fitBounds } from '../src/core/viewport';
import { loadBundleFromDisk } from './playback_fixtures';

function deepFreeze<T>(value: T): T {
  if (value && typeof value === 'object') {
    for (const key of Object.keys(value as Record<string, unknown>)) {
      deepFreeze((value as Record<string, unknown>)[key]);
    }
    Object.freeze(value);
  }
  return value;
}

const bundle = await loadBundleFromDisk();
const viewport = fitBounds(
  { min_x_m: 0, min_y_m: 0, max_x_m: 50, max_y_m: 30 },
  800,
  500,
  20,
);

function sampleById(id: string): PlaybackSampleData {
  const found = bundle.samples.find((sample) => sample.sample.id === id);
  if (!found) throw new Error(`missing sample ${id}`);
  return found;
}

describe('回放覆盖层构建（世界坐标）', () => {
  it('RS 路径：2 段前进、无换向、起终点标记', () => {
    const overlay = buildPlaybackOverlay(sampleById('rs_path_open'), { ...DEFAULT_PLAYBACK_LAYERS }, 0);
    expect(overlay.paths).toHaveLength(1);
    expect(overlay.paths[0].segments).toHaveLength(2);
    expect(overlay.paths[0].segments.every((segment) => segment.direction === 'forward')).toBe(true);
    expect(overlay.paths[0].cusps).toEqual([]);
    expect(overlay.paths[0].start).toEqual([4, 4]);
    expect(overlay.paths[0].end).toEqual([15, 9]);
  });

  it('换向路径：前进+倒车两段、cusp 在 (36,5)', () => {
    const overlay = buildPlaybackOverlay(sampleById('hybrid_path_reverse'), { ...DEFAULT_PLAYBACK_LAYERS }, 0);
    expect(overlay.paths[0].segments.map((segment) => segment.direction)).toEqual(['forward', 'reverse']);
    expect(overlay.paths[0].cusps).toEqual([[36, 5]]);
  });

  it('失败样例不产生路径几何', () => {
    const overlay = buildPlaybackOverlay(sampleById('hybrid_path_failure'), { ...DEFAULT_PLAYBACK_LAYERS }, 0);
    expect(overlay.paths).toEqual([]);
    expect(overlay.trajectories).toEqual([]);
  });

  it('B 样条曲线：200 采样点 + 8 控制点', () => {
    const overlay = buildPlaybackOverlay(sampleById('bspline_curve_demo'), { ...DEFAULT_PLAYBACK_LAYERS }, 0);
    expect(overlay.curves).toHaveLength(1);
    expect(overlay.curves[0].points).toHaveLength(200);
    expect(overlay.curves[0].control_points).toHaveLength(8);
  });

  it('多项式曲线：200 点 + 段边界 + 剖面', () => {
    const overlay = buildPlaybackOverlay(sampleById('polynomial_curve_demo'), { ...DEFAULT_PLAYBACK_LAYERS }, 0);
    expect(overlay.curves[0].points).toHaveLength(200);
    expect(overlay.curves[0].boundaries).toEqual([[25, 8]]);
    expect(overlay.curves[0].profile).toHaveLength(100);
  });

  it('走廊：3 个多边形（s 区间保留）', () => {
    const overlay = buildPlaybackOverlay(sampleById('corridor_hybrid_reverse'), { ...DEFAULT_PLAYBACK_LAYERS }, 0);
    expect(overlay.corridors).toHaveLength(1);
    expect(overlay.corridors[0].polygons).toHaveLength(3);
    expect(overlay.corridors[0].polygons[1].s_start_m).toBe(3);
  });

  it('轨迹：8 点 + 回放位姿随 t 变化', () => {
    const overlay = buildPlaybackOverlay(sampleById('dftpav_trajectory_demo'), { ...DEFAULT_PLAYBACK_LAYERS }, 2);
    expect(overlay.trajectories).toHaveLength(1);
    expect(overlay.trajectories[0].points).toHaveLength(8);
    expect(overlay.playback_pose?.x_m).toBeCloseTo(34, 9);
    const later = buildPlaybackOverlay(sampleById('dftpav_trajectory_demo'), { ...DEFAULT_PLAYBACK_LAYERS }, 5.5);
    expect(later.playback_pose?.x_m).toBeCloseTo(38, 9);
    expect(playbackDuration(sampleById('dftpav_trajectory_demo'))).toBeCloseTo(7.5, 9);
  });

  it('碰撞样例：足迹/障碍/重叠区', () => {
    const overlay = buildPlaybackOverlay(sampleById('collision_demo'), { ...DEFAULT_PLAYBACK_LAYERS }, 0);
    expect(overlay.collision).not.toBeNull();
    expect(overlay.collision?.footprint).toHaveLength(4);
    expect(overlay.collision?.obstacle).toHaveLength(4);
    expect(overlay.collision?.overlap).toEqual([
      [11, 4.1],
      [13.7, 4.1],
      [13.7, 5.9],
      [11, 5.9],
    ]);
    expect(overlay.collision?.verdict).toBe('collision');
  });

  it('过期样例：不产生任何几何', () => {
    const overlay = buildPlaybackOverlay(sampleById('stale_result_demo'), { ...DEFAULT_PLAYBACK_LAYERS }, 0);
    expect(overlay.paths).toEqual([]);
    expect(overlay.curves).toEqual([]);
    expect(overlay.corridors).toEqual([]);
    expect(overlay.trajectories).toEqual([]);
    expect(overlay.collision).toBeNull();
  });

  it('图层开关：关闭路径/轨迹/曲线/走廊/碰撞后无对应几何', () => {
    const off = { paths: false, curves: false, corridors: false, trajectories: false, collision: false };
    for (const id of [
      'rs_path_open',
      'bspline_curve_demo',
      'corridor_hybrid_reverse',
      'dftpav_trajectory_demo',
      'collision_demo',
    ]) {
      const overlay = buildPlaybackOverlay(sampleById(id), off, 1);
      expect(overlay.paths).toEqual([]);
      expect(overlay.curves).toEqual([]);
      expect(overlay.corridors).toEqual([]);
      expect(overlay.trajectories).toEqual([]);
      expect(overlay.collision).toBeNull();
      expect(overlay.playback_pose).toBeNull();
    }
  });

  it('剖面：轨迹与多项式样例', () => {
    expect(playbackProfile(sampleById('dftpav_trajectory_demo'))).toHaveLength(8);
    expect(playbackProfile(sampleById('polynomial_curve_demo'))).toHaveLength(100);
    expect(playbackProfile(sampleById('rs_path_open'))).toEqual([]);
  });
});

describe('回放绘制命令', () => {
  it('RS 路径命令：2 折线 + 起终圆点（result_path）', () => {
    const overlay = buildPlaybackOverlay(sampleById('rs_path_open'), { ...DEFAULT_PLAYBACK_LAYERS }, 0);
    const commands = buildPlaybackCommands(overlay, viewport);
    const summary = summarizeDrawCommands(commands);
    expect(summary.by_layer.result_path).toBe(4);
    expect(commands.every((command) => command.op !== 'polyline' || command.layer !== 'result_path' || command.stroke === PLAYBACK_COLORS.forward)).toBe(true);
    expect(commandsHaveFiniteCoordinates(commands)).toBe(true);
  });

  it('换向命令：前进蓝/倒车橙 + cusp 标记', () => {
    const overlay = buildPlaybackOverlay(sampleById('hybrid_path_reverse'), { ...DEFAULT_PLAYBACK_LAYERS }, 0);
    const commands = buildPlaybackCommands(overlay, viewport);
    const strokes = commands
      .filter((command): command is Extract<DrawCommand, { op: 'polyline' }> => command.op === 'polyline')
      .map((command) => command.stroke);
    expect(strokes).toContain(PLAYBACK_COLORS.forward);
    expect(strokes).toContain(PLAYBACK_COLORS.reverse);
    const cuspCircles = commands.filter(
      (command) => command.op === 'circle' && command.fill === PLAYBACK_COLORS.cusp,
    );
    expect(cuspCircles).toHaveLength(1);
    expect(summarizeDrawCommands(commands).by_layer.result_path).toBe(5);
  });

  it('失败样例：无路径命令', () => {
    const overlay = buildPlaybackOverlay(sampleById('hybrid_path_failure'), { ...DEFAULT_PLAYBACK_LAYERS }, 0);
    expect(buildPlaybackCommands(overlay, viewport)).toEqual([]);
  });

  it('曲线命令：控制多边形虚线 + 曲线 + 段边界', () => {
    const bspline = buildPlaybackCommands(
      buildPlaybackOverlay(sampleById('bspline_curve_demo'), { ...DEFAULT_PLAYBACK_LAYERS }, 0),
      viewport,
    );
    expect(summarizeDrawCommands(bspline).by_layer.result_curve).toBe(2);
    const polynomial = buildPlaybackCommands(
      buildPlaybackOverlay(sampleById('polynomial_curve_demo'), { ...DEFAULT_PLAYBACK_LAYERS }, 0),
      viewport,
    );
    // 多项式：曲线折线 + 1 个段边界圆点（无控制多边形）。
    expect(summarizeDrawCommands(polynomial).by_layer.result_curve).toBe(2);
  });

  it('走廊命令：3 个多边形填充 + 3 条 s 区间文字标注', () => {
    const commands = buildPlaybackCommands(
      buildPlaybackOverlay(sampleById('corridor_hybrid_reverse'), { ...DEFAULT_PLAYBACK_LAYERS }, 0),
      viewport,
    );
    const summary = summarizeDrawCommands(commands);
    expect(summary.by_layer.corridor).toBe(6);
    expect(summary.by_op.text).toBe(3);
    const polygon = commands[0];
    expect(polygon.op).toBe('polygon');
    if (polygon.op === 'polygon') {
      expect(polygon.fill).toBe(PLAYBACK_COLORS.corridor_fill);
      expect(polygon.rings[0]).toHaveLength(4);
    }
  });

  it('轨迹命令：按速度符号分段 + 当前位姿箭头', () => {
    const commands = buildPlaybackCommands(
      buildPlaybackOverlay(sampleById('dftpav_trajectory_demo'), { ...DEFAULT_PLAYBACK_LAYERS }, 5.5),
      viewport,
    );
    const summary = summarizeDrawCommands(commands);
    // 8 个点圆 + 2 段折线（+ 段与 - 段）+ 位姿圆点 + 箭头折线
    expect(summary.by_layer.trajectory).toBe(12);
    const strokes = commands
      .filter((command): command is Extract<DrawCommand, { op: 'polyline' }> => command.op === 'polyline')
      .map((command) => command.stroke);
    expect(strokes).toContain(PLAYBACK_COLORS.trajectory_forward);
    expect(strokes).toContain(PLAYBACK_COLORS.trajectory_reverse);
    expect(strokes).toContain(PLAYBACK_COLORS.pose);
  });

  it('碰撞命令：障碍 + 重叠区 + 足迹轮廓', () => {
    const commands = buildPlaybackCommands(
      buildPlaybackOverlay(sampleById('collision_demo'), { ...DEFAULT_PLAYBACK_LAYERS }, 0),
      viewport,
    );
    expect(summarizeDrawCommands(commands).by_layer.collision).toBe(3);
  });

  it('深冻结样例可安全构建（不修改夹具数据）', () => {
    const frozen = deepFreeze(sampleById('dftpav_trajectory_demo'));
    const before = JSON.stringify(frozen);
    const overlay = buildPlaybackOverlay(frozen, { ...DEFAULT_PLAYBACK_LAYERS }, 3);
    const commands = buildPlaybackCommands(overlay, viewport);
    expect(commands.length).toBeGreaterThan(0);
    expect(JSON.stringify(frozen)).toBe(before);
  });
});
