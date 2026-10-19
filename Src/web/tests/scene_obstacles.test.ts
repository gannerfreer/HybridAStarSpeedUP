import { describe, expect, it } from 'vitest';
import {
  hitTestObstacles,
  makeRectangleObstacle,
  moveObstacle,
  nextObstacleId,
  obstacleContainsPoint,
  obstacleSetsEqual,
  rectangleFromDrag,
  sortObstacles,
  updateRectangleField,
} from '../src/core/scene/obstacles';
import type { RectangleObstacle, SceneObstacle } from '../src/core/scene/types';

const rect = (id: string, x0: number, y0: number, x1: number, y1: number): RectangleObstacle =>
  makeRectangleObstacle(id, x0, y0, x1, y1);

const polygon: SceneObstacle = {
  id: 'obs-9',
  type: 'convex_polygon',
  vertices: [
    { x_m: 0, y_m: 0 },
    { x_m: 4, y_m: 0 },
    { x_m: 4, y_m: 4 },
  ],
};

describe('W3 障碍编辑逻辑', () => {
  it('矩形构造归一化（任意两点）', () => {
    expect(rect('a', 5, 3, -1, -2)).toEqual({
      id: 'a',
      type: 'rectangle',
      x_min_m: -1,
      y_min_m: -2,
      x_max_m: 5,
      y_max_m: 3,
    });
  });

  it('rectangleFromDrag：过小返回 null', () => {
    expect(rectangleFromDrag('a', 0, 0, 0.4, 2)).toBeNull();
    expect(rectangleFromDrag('a', 0, 0, 2, 0.2)).toBeNull();
    expect(rectangleFromDrag('a', 0, 0, 2, 1)?.x_max_m).toBe(2);
    expect(rectangleFromDrag('a', 0, 0, 0.5, 0.5, 0.5)).not.toBeNull();
  });

  it('nextObstacleId 跳过已用编号', () => {
    expect(nextObstacleId([])).toBe('obs-1');
    expect(nextObstacleId([rect('obs-1', 0, 0, 1, 1), rect('obs-3', 0, 0, 1, 1)])).toBe('obs-2');
    expect(nextObstacleId([rect('obs-1', 0, 0, 1, 1), rect('obs-2', 0, 0, 1, 1)])).toBe('obs-3');
  });

  it('moveObstacle 平移矩形与多边形（不修改原对象）', () => {
    const original = rect('a', 0, 0, 2, 1);
    const moved = moveObstacle(original, 10, -5);
    expect(moved.type).toBe('rectangle');
    if (moved.type === 'rectangle') {
      expect(moved).toEqual({ id: 'a', type: 'rectangle', x_min_m: 10, y_min_m: -5, x_max_m: 12, y_max_m: -4 });
    }
    expect(original.x_min_m).toBe(0);

    const movedPolygon = moveObstacle(polygon, 1, 2);
    expect(movedPolygon.type).toBe('convex_polygon');
    if (movedPolygon.type === 'convex_polygon') {
      expect(movedPolygon.vertices[2]).toEqual({ x_m: 5, y_m: 6 });
    }
    expect(polygon.type === 'convex_polygon' && polygon.vertices[2].x_m).toBe(4);
  });

  it('updateRectangleField：min/max 倒置时交换保持正 extent', () => {
    const updated = updateRectangleField(rect('a', 0, 0, 2, 1), 'x_min_m', 5);
    expect(updated.x_min_m).toBe(2);
    expect(updated.x_max_m).toBe(5);
    const updatedY = updateRectangleField(rect('a', 0, 0, 2, 1), 'y_max_m', -3);
    expect(updatedY.y_min_m).toBe(-3);
    expect(updatedY.y_max_m).toBe(0);
  });

  it('obstacleContainsPoint：矩形含边界、多边形 evenodd', () => {
    expect(obstacleContainsPoint(rect('a', 0, 0, 2, 1), 0, 0)).toBe(true);
    expect(obstacleContainsPoint(rect('a', 0, 0, 2, 1), 2, 1)).toBe(true);
    expect(obstacleContainsPoint(rect('a', 0, 0, 2, 1), 2.1, 0.5)).toBe(false);
    expect(obstacleContainsPoint(polygon, 3, 1)).toBe(true);
    expect(obstacleContainsPoint(polygon, 1, 3)).toBe(false);
  });

  it('hitTestObstacles：后创建者优先命中', () => {
    const first = rect('obs-1', 0, 0, 4, 4);
    const second = rect('obs-2', 1, 1, 3, 3);
    expect(hitTestObstacles([first, second], 2, 2)?.id).toBe('obs-2');
    expect(hitTestObstacles([first, second], 3.5, 3.5)?.id).toBe('obs-1');
    expect(hitTestObstacles([first, second], 10, 10)).toBeNull();
  });

  it('sortObstacles / obstacleSetsEqual 与顺序无关', () => {
    const a = rect('obs-2', 0, 0, 1, 1);
    const b = rect('obs-10', 0, 0, 1, 1);
    expect(sortObstacles([b, a]).map((obstacle) => obstacle.id)).toEqual(['obs-2', 'obs-10']);
    expect(obstacleSetsEqual([a, b], [b, a])).toBe(true);
    expect(obstacleSetsEqual([a], [a, b])).toBe(false);
    expect(obstacleSetsEqual([a], [rect('obs-2', 0, 0, 1.5, 1)])).toBe(false);
  });
});
