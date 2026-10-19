import { describe, expect, it } from 'vitest';
import { findRepoRoot, moduleDir, readVehicleJson } from '../src/core/fixture';
import {
  buildOverlayCommands,
  commandsHaveFiniteCoordinates,
  OVERLAY_COLORS,
  summarizeDrawCommands,
} from '../src/core/render';
import { makeRectangleObstacle } from '../src/core/scene/obstacles';
import { parseVehicleModel } from '../src/core/scene/vehicle';
import { fitBounds } from '../src/core/viewport';
import { makeSnapshot, straightLanelet } from './fixtures';

const repoRoot = findRepoRoot(moduleDir(import.meta.url));
const vehicle = parseVehicleModel(readVehicleJson(repoRoot).value);
const snapshot = makeSnapshot({ lanelets: [straightLanelet('100', 0, 10)] });
const viewport = fitBounds(snapshot.bounds_m, 800, 600, 20);

function deepFreeze<T>(value: T): T {
  if (value && typeof value === 'object') {
    for (const key of Object.keys(value as Record<string, unknown>)) {
      deepFreeze((value as Record<string, unknown>)[key]);
    }
    Object.freeze(value);
  }
  return value;
}

describe('W3 覆盖层绘制（障碍 + 足迹）', () => {
  it('矩形障碍：多边形填充/描边，选中换色', () => {
    const obstacle = makeRectangleObstacle('obs-1', 0, 0, 4, 2);
    const commands = buildOverlayCommands(snapshot, viewport, { obstacles: [obstacle] });
    expect(commands).toHaveLength(1);
    const command = commands[0];
    expect(command.op).toBe('polygon');
    expect(command.layer).toBe('obstacles');
    if (command.op === 'polygon') {
      expect(command.rings[0]).toHaveLength(4);
      expect(command.fill).toBe(OVERLAY_COLORS.obstacle_fill);
      expect(command.stroke).toBe(OVERLAY_COLORS.obstacle_stroke);
    }

    const selected = buildOverlayCommands(snapshot, viewport, {
      obstacles: [obstacle],
      selected_obstacle_id: 'obs-1',
    });
    if (selected[0].op === 'polygon') {
      expect(selected[0].stroke).toBe(OVERLAY_COLORS.obstacle_selected);
      expect(selected[0].lineWidth).toBe(3);
    }
  });

  it('多边形障碍渲染为闭合多边形', () => {
    const commands = buildOverlayCommands(snapshot, viewport, {
      obstacles: [
        {
          id: 'obs-2',
          type: 'convex_polygon',
          vertices: [
            { x_m: 1, y_m: 1 },
            { x_m: 3, y_m: 1 },
            { x_m: 2, y_m: 3 },
          ],
        },
      ],
    });
    expect(commands).toHaveLength(1);
    expect(commands[0].op).toBe('polygon');
    if (commands[0].op === 'polygon') expect(commands[0].rings[0]).toHaveLength(3);
  });

  it('障碍新建预览：虚线闭合矩形', () => {
    const commands = buildOverlayCommands(snapshot, viewport, {
      obstacle_preview: { min_x_m: 0, min_y_m: 0, max_x_m: 4, max_y_m: 2 },
    });
    expect(commands).toHaveLength(1);
    const command = commands[0];
    expect(command.op).toBe('polyline');
    if (command.op === 'polyline') {
      expect(command.points).toHaveLength(5);
      expect(command.points[0]).toEqual(command.points[4]);
      expect(command.dash).toEqual([5, 4]);
      expect(command.stroke).toBe(OVERLAY_COLORS.obstacle_stroke);
    }
  });

  it('车辆足迹：车体多边形 + 裕量虚线轮廓，起终颜色区分', () => {
    const commands = buildOverlayCommands(snapshot, viewport, {
      vehicle,
      footprints: [
        { pose: { x_m: 5, y_m: 0, yaw_rad: 0 }, kind: 'start' },
        { pose: { x_m: 8, y_m: 0, yaw_rad: Math.PI / 2 }, kind: 'goal' },
      ],
    });
    expect(commands).toHaveLength(4); // 每个足迹：车体 + 裕量
    const [bodyStart, marginStart, bodyGoal, marginGoal] = commands;
    expect(bodyStart.op).toBe('polygon');
    expect(marginStart.op).toBe('polyline');
    if (bodyStart.op === 'polygon') {
      expect(bodyStart.layer).toBe('vehicle');
      expect(bodyStart.fill).toBe(OVERLAY_COLORS.footprint_start_fill);
      expect(bodyStart.stroke).toBe(OVERLAY_COLORS.start_pose);
    }
    if (marginStart.op === 'polyline') {
      expect(marginStart.points).toHaveLength(5);
      expect(marginStart.dash).toEqual([5, 4]);
    }
    if (bodyGoal.op === 'polygon') {
      expect(bodyGoal.fill).toBe(OVERLAY_COLORS.footprint_goal_fill);
      expect(bodyGoal.stroke).toBe(OVERLAY_COLORS.goal_pose);
    }
    // 车体宽度（屏幕像素）：1.8 m × scale
    if (bodyStart.op === 'polygon') {
      const ring = bodyStart.rings[0];
      const widthPx = Math.hypot(ring[0][0] - ring[1][0], ring[0][1] - ring[1][1]);
      expect(widthPx).toBeCloseTo(1.8 * viewport.scale_px_per_m, 6);
    }
    if (bodyGoal.op === 'polygon') {
      // yaw=π/2：车头朝屏幕上方（-y）
      const ring = bodyGoal.rings[0];
      expect(ring[0][1]).toBeLessThan(ring[3][1]);
    }
    expect(commandsHaveFiniteCoordinates(commands)).toBe(true);
  });

  it('无车辆模型时不绘制足迹', () => {
    const commands = buildOverlayCommands(snapshot, viewport, {
      footprints: [{ pose: { x_m: 5, y_m: 0, yaw_rad: 0 }, kind: 'start' }],
    });
    expect(commands).toEqual([]);
  });

  it('叠加层不修改底图数据（深冻结快照可安全构建）', () => {
    const frozenSnapshot = deepFreeze(makeSnapshot({ lanelets: [straightLanelet('100', 0, 10)] }));
    const frozenObstacles = deepFreeze([makeRectangleObstacle('obs-1', 0, 0, 2, 1)]);
    const before = JSON.stringify(frozenSnapshot);
    const commands = buildOverlayCommands(frozenSnapshot, viewport, {
      obstacles: frozenObstacles,
      vehicle: deepFreeze(vehicle),
      footprints: [{ pose: deepFreeze({ x_m: 5, y_m: 0, yaw_rad: 0 }), kind: 'start' }],
    });
    expect(commands.length).toBeGreaterThan(0);
    expect(JSON.stringify(frozenSnapshot)).toBe(before);
  });

  it('汇总统计包含 obstacles/vehicle 图层', () => {
    const commands = buildOverlayCommands(snapshot, viewport, {
      obstacles: [makeRectangleObstacle('obs-1', 0, 0, 2, 1)],
      vehicle,
      footprints: [{ pose: { x_m: 5, y_m: 0, yaw_rad: 0 }, kind: 'cursor' }],
    });
    const summary = summarizeDrawCommands(commands);
    expect(summary.by_layer.obstacles).toBe(1);
    expect(summary.by_layer.vehicle).toBe(2);
    expect(summary.by_layer.areas).toBe(0);
  });
});
