import { describe, expect, it } from 'vitest';
import {
  buildOverlayCommands,
  commandsHaveFiniteCoordinates,
  OVERLAY_COLORS,
  summarizeDrawCommands,
  type DrawCommand,
} from '../src/core/render';
import { worldToScreen } from '../src/core/transform';
import { fitBounds } from '../src/core/viewport';
import { area, makeSnapshot, point, straightLanelet } from './fixtures';

const snapshot = makeSnapshot({
  lanelets: [straightLanelet('100', 0, 10), straightLanelet('200', 20, 30)],
  areas: [area('20', [point('a', 12, 0), point('b', 16, 0), point('c', 16, 4), point('d', 12, 4)])],
});
const viewport = fitBounds(snapshot.bounds_m, 800, 600, 20);

function polylineStrokes(commands: readonly DrawCommand[]): string[] {
  const strokes: string[] = [];
  for (const command of commands) {
    if (command.op === 'polyline') strokes.push(command.stroke);
  }
  return strokes;
}

describe('W2 覆盖层绘制命令', () => {
  it('空覆盖层 → 无命令', () => {
    expect(buildOverlayCommands(snapshot, viewport, {})).toEqual([]);
  });

  it('选中 lanelet：左右边界 + 中心线高亮', () => {
    const commands = buildOverlayCommands(snapshot, viewport, { selected_lanelets: ['100'] });
    const selection = commands.filter((command) => command.layer === 'selection');
    expect(selection).toHaveLength(3);
    for (const command of selection) {
      if (command.op === 'polyline' || command.op === 'polygon') {
        expect(command.stroke).toBe(OVERLAY_COLORS.selection);
      }
    }
    expect(commandsHaveFiniteCoordinates(commands)).toBe(true);
  });

  it('选中区域：evenodd 多边形描边', () => {
    const commands = buildOverlayCommands(snapshot, viewport, { selected_areas: ['20'] });
    expect(commands).toHaveLength(1);
    const command = commands[0];
    expect(command.op).toBe('polygon');
    if (command.op === 'polygon') {
      expect(command.fillRule).toBe('evenodd');
      expect(command.stroke).toBe(OVERLAY_COLORS.selection);
    }
  });

  it('拓扑列表：后继/前驱/邻接使用不同颜色', () => {
    const commands = buildOverlayCommands(snapshot, viewport, {
      following: ['100'],
      preceding: ['200'],
      adjacent: ['100'],
    });
    const strokes = polylineStrokes(commands);
    expect(strokes).toContain(OVERLAY_COLORS.following);
    expect(strokes).toContain(OVERLAY_COLORS.preceding);
    expect(strokes).toContain(OVERLAY_COLORS.adjacent);
    for (const command of commands) expect(command.layer).toBe('topology');
  });

  it('路由：加宽中心线', () => {
    const commands = buildOverlayCommands(snapshot, viewport, { route: ['100', '200'] });
    expect(commands).toHaveLength(2);
    for (const command of commands) {
      expect(command.op).toBe('polyline');
      if (command.op === 'polyline') {
        expect(command.layer).toBe('route');
        expect(command.lineWidth).toBe(4);
        expect(command.stroke).toBe(OVERLAY_COLORS.route);
      }
    }
  });

  it('位姿：圆点 + 固定像素长度的航向箭头', () => {
    const commands = buildOverlayCommands(snapshot, viewport, { start_pose: { x_m: 5, y_m: 0, yaw_rad: 0 } });
    expect(commands).toHaveLength(3);
    const [sx, sy] = worldToScreen(viewport, 5, 0);
    const circle = commands[0];
    expect(circle.op).toBe('circle');
    if (circle.op === 'circle') {
      expect(circle.cx).toBeCloseTo(sx, 9);
      expect(circle.cy).toBeCloseTo(sy, 9);
      expect(circle.fill).toBe(OVERLAY_COLORS.start_pose);
    }
    const shaft = commands[1];
    expect(shaft.op).toBe('polyline');
    if (shaft.op === 'polyline') {
      expect(shaft.points[0][0]).toBeCloseTo(sx, 9);
      expect(shaft.points[0][1]).toBeCloseTo(sy, 9);
      expect(shaft.points[1][0]).toBeCloseTo(sx + 30, 9);
      expect(shaft.points[1][1]).toBeCloseTo(sy, 9);
    }

    const goalCommands = buildOverlayCommands(snapshot, viewport, {
      goal_pose: { x_m: 5, y_m: 0, yaw_rad: Math.PI / 2 },
    });
    const goalShaft = goalCommands[1];
    if (goalShaft.op === 'polyline') {
      expect(goalShaft.points[1][0]).toBeCloseTo(sx, 9);
      expect(goalShaft.points[1][1]).toBeCloseTo(sy - 30, 9);
      expect(goalShaft.stroke).toBe(OVERLAY_COLORS.goal_pose);
    }
  });

  it('窗口矩形：闭合虚线', () => {
    const commands = buildOverlayCommands(snapshot, viewport, {
      window: { min_x_m: 0, min_y_m: 0, max_x_m: 10, max_y_m: 10 },
    });
    expect(commands).toHaveLength(1);
    const rect = commands[0];
    expect(rect.op).toBe('polyline');
    if (rect.op === 'polyline') {
      expect(rect.points).toHaveLength(5);
      expect(rect.points[0]).toEqual(rect.points[4]);
      expect(rect.dash).toEqual([6, 4]);
      expect(rect.stroke).toBe(OVERLAY_COLORS.window);
      expect(rect.layer).toBe('window');
    }
  });

  it('裁剪结果：路径折线与点标记（窗口图层）', () => {
    const commands = buildOverlayCommands(snapshot, viewport, {
      crop_paths: [
        {
          lanelet_id: '100',
          role: 'centerline',
          points: [
            { id: 'a', x_m: 1, y_m: 1, z_m: 0 },
            { id: 'b', x_m: 5, y_m: 5, z_m: 0 },
          ],
          start_clip_edge: true,
          end_clip_edge: false,
        },
      ],
      crop_points: [{ id: 'c', x_m: 2, y_m: 2, z_m: 0 }],
    });
    expect(commands).toHaveLength(2);
    expect(commands[0].op).toBe('polyline');
    expect(commands[1].op).toBe('circle');
    for (const command of commands) expect(command.layer).toBe('window');
  });

  it('汇总统计包含覆盖层且基础图层为零', () => {
    const commands = buildOverlayCommands(snapshot, viewport, {
      selected_lanelets: ['100'],
      route: ['200'],
      start_pose: { x_m: 5, y_m: 0, yaw_rad: 0 },
    });
    const summary = summarizeDrawCommands(commands);
    expect(summary.by_layer.selection).toBe(3);
    expect(summary.by_layer.route).toBe(1);
    expect(summary.by_layer.poses).toBe(3);
    expect(summary.by_layer.areas).toBe(0);
    expect(summary.by_layer.boundaries).toBe(0);
  });

  it('未知 ID 被静默跳过', () => {
    expect(buildOverlayCommands(snapshot, viewport, { selected_lanelets: ['nope'], route: ['nope'] })).toEqual([]);
  });
});
