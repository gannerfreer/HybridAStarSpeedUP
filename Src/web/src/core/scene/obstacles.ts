import { pointInRing, type XY } from '../hit';
import type { RectangleObstacle, SceneObstacle } from './types';

/** W3 障碍编辑逻辑（纯函数；不触碰地图数据）。 */

export function makeRectangleObstacle(
  id: string,
  x0: number,
  y0: number,
  x1: number,
  y1: number,
): RectangleObstacle {
  return {
    id,
    type: 'rectangle',
    x_min_m: Math.min(x0, x1),
    y_min_m: Math.min(y0, y1),
    x_max_m: Math.max(x0, x1),
    y_max_m: Math.max(y0, y1),
  };
}

/**
 * 由拖拽矩形生成障碍；任一边小于 minSizeM 时返回 null（视为误点击）。
 */
export function rectangleFromDrag(
  id: string,
  x0: number,
  y0: number,
  x1: number,
  y1: number,
  minSizeM = 0.5,
): RectangleObstacle | null {
  if (Math.abs(x1 - x0) < minSizeM || Math.abs(y1 - y0) < minSizeM) return null;
  return makeRectangleObstacle(id, x0, y0, x1, y1);
}

/** 下一个可用 ID：obs-1、obs-2、…（与既有 ID 不冲突）。 */
export function nextObstacleId(existing: readonly SceneObstacle[]): string {
  const used = new Set(existing.map((obstacle) => obstacle.id));
  let index = 1;
  while (used.has(`obs-${index}`)) index += 1;
  return `obs-${index}`;
}

export function moveObstacle(obstacle: SceneObstacle, dxM: number, dyM: number): SceneObstacle {
  if (obstacle.type === 'rectangle') {
    return {
      ...obstacle,
      x_min_m: obstacle.x_min_m + dxM,
      y_min_m: obstacle.y_min_m + dyM,
      x_max_m: obstacle.x_max_m + dxM,
      y_max_m: obstacle.y_max_m + dyM,
    };
  }
  return {
    ...obstacle,
    vertices: obstacle.vertices.map((vertex) => ({ x_m: vertex.x_m + dxM, y_m: vertex.y_m + dyM })),
  };
}

/** 面板编辑矩形字段；min/max 倒置时自动交换，保证正 extent。 */
export function updateRectangleField(
  obstacle: RectangleObstacle,
  field: 'x_min_m' | 'y_min_m' | 'x_max_m' | 'y_max_m',
  value: number,
): RectangleObstacle {
  const next: RectangleObstacle = { ...obstacle, [field]: value };
  if (next.x_min_m > next.x_max_m) {
    [next.x_min_m, next.x_max_m] = [next.x_max_m, next.x_min_m];
  }
  if (next.y_min_m > next.y_max_m) {
    [next.y_min_m, next.y_max_m] = [next.y_max_m, next.y_min_m];
  }
  return next;
}

export function obstacleContainsPoint(obstacle: SceneObstacle, x: number, y: number): boolean {
  if (obstacle.type === 'rectangle') {
    return (
      x >= Math.min(obstacle.x_min_m, obstacle.x_max_m) &&
      x <= Math.max(obstacle.x_min_m, obstacle.x_max_m) &&
      y >= Math.min(obstacle.y_min_m, obstacle.y_max_m) &&
      y <= Math.max(obstacle.y_min_m, obstacle.y_max_m)
    );
  }
  const ring: XY[] = obstacle.vertices.map((vertex): XY => [vertex.x_m, vertex.y_m]);
  return ring.length >= 3 && pointInRing(ring, x, y);
}

/** 命中测试：返回最上层（后创建者优先）的障碍。 */
export function hitTestObstacles(obstacles: readonly SceneObstacle[], x: number, y: number): SceneObstacle | null {
  for (let index = obstacles.length - 1; index >= 0; index -= 1) {
    if (obstacleContainsPoint(obstacles[index], x, y)) return obstacles[index];
  }
  return null;
}

/** 障碍 ID 自然序（obs-2 < obs-10；纯数字与字典序回退）。 */
export function compareObstacleIds(a: string, b: string): number {
  const suffix = (id: string): number | null => {
    const match = /(\d+)$/.exec(id);
    return match ? Number(match[1]) : null;
  };
  const numericA = suffix(a);
  const numericB = suffix(b);
  if (numericA !== null && numericB !== null && numericA !== numericB) {
    return numericA - numericB;
  }
  return a < b ? -1 : a > b ? 1 : 0;
}

/** 按 ID 稳定排序（自然序，供序列化与对比）。 */
export function sortObstacles(obstacles: readonly SceneObstacle[]): SceneObstacle[] {
  const byId = new Map(obstacles.map((obstacle) => [obstacle.id, obstacle]));
  const ids = [...byId.keys()].sort(compareObstacleIds);
  return ids.map((id) => byId.get(id)!);
}

export function obstacleEquals(a: SceneObstacle, b: SceneObstacle, tolerance = 1e-9): boolean {
  if (a.id !== b.id || a.type !== b.type) return false;
  if (a.type === 'rectangle' && b.type === 'rectangle') {
    return (
      Math.abs(a.x_min_m - b.x_min_m) <= tolerance &&
      Math.abs(a.y_min_m - b.y_min_m) <= tolerance &&
      Math.abs(a.x_max_m - b.x_max_m) <= tolerance &&
      Math.abs(a.y_max_m - b.y_max_m) <= tolerance
    );
  }
  if (a.type === 'convex_polygon' && b.type === 'convex_polygon') {
    if (a.vertices.length !== b.vertices.length) return false;
    return a.vertices.every(
      (vertex, index) =>
        Math.abs(vertex.x_m - b.vertices[index].x_m) <= tolerance &&
        Math.abs(vertex.y_m - b.vertices[index].y_m) <= tolerance,
    );
  }
  return false;
}

export function obstacleSetsEqual(
  a: readonly SceneObstacle[],
  b: readonly SceneObstacle[],
  tolerance = 1e-9,
): boolean {
  const sortedA = sortObstacles(a);
  const sortedB = sortObstacles(b);
  if (sortedA.length !== sortedB.length) return false;
  return sortedA.every((obstacle, index) => obstacleEquals(obstacle, sortedB[index], tolerance));
}
