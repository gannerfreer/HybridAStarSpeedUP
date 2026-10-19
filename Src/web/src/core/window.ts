import { sortIds, type XY } from './hit';
import type { BoundsM, MapSnapshot } from './types';

/**
 * 局部窗口查询的本地回退：仅按包围盒过滤对象（不裁剪几何），
 * 结果标注 `approximate: true`；服务在线时以 `/map/query/window` 的
 * 裁剪结果（含窗口局部坐标变换）为准。
 */

export interface WindowFilterResult {
  lanelet_ids: string[];
  area_ids: string[];
  approximate: true;
}

export function normalizeBounds(x0: number, y0: number, x1: number, y1: number): BoundsM {
  return {
    min_x_m: Math.min(x0, x1),
    min_y_m: Math.min(y0, y1),
    max_x_m: Math.max(x0, x1),
    max_y_m: Math.max(y0, y1),
  };
}

export function boundsIntersect(a: BoundsM, b: BoundsM): boolean {
  return !(a.max_x_m < b.min_x_m || a.min_x_m > b.max_x_m || a.max_y_m < b.min_y_m || a.min_y_m > b.max_y_m);
}

export function pointsBounds(points: readonly { x_m: number; y_m: number }[]): BoundsM | null {
  if (points.length === 0) return null;
  let minX = Number.POSITIVE_INFINITY;
  let minY = Number.POSITIVE_INFINITY;
  let maxX = Number.NEGATIVE_INFINITY;
  let maxY = Number.NEGATIVE_INFINITY;
  for (const point of points) {
    minX = Math.min(minX, point.x_m);
    minY = Math.min(minY, point.y_m);
    maxX = Math.max(maxX, point.x_m);
    maxY = Math.max(maxY, point.y_m);
  }
  return { min_x_m: minX, min_y_m: minY, max_x_m: maxX, max_y_m: maxY };
}

function ringHasPointInside(ring: readonly { x_m: number; y_m: number }[], window: BoundsM): boolean {
  for (const point of ring) {
    if (
      point.x_m >= window.min_x_m &&
      point.x_m <= window.max_x_m &&
      point.y_m >= window.min_y_m &&
      point.y_m <= window.max_y_m
    ) {
      return true;
    }
  }
  return false;
}

export function filterSnapshotToWindow(snapshot: MapSnapshot, window: BoundsM): WindowFilterResult {
  const laneletIds: string[] = [];
  for (const lanelet of snapshot.lanelets) {
    const points: XY[] = [];
    for (const point of [...lanelet.left, ...lanelet.right, ...(lanelet.centerline ?? [])]) {
      points.push([point.x_m, point.y_m]);
    }
    if (points.length === 0) continue;
    const bounds = pointsBounds(points.map(([x, y]) => ({ x_m: x, y_m: y })));
    if (bounds && boundsIntersect(bounds, window)) laneletIds.push(lanelet.id);
  }

  const areaIds: string[] = [];
  for (const area of snapshot.areas) {
    const bounds = pointsBounds(area.outer);
    if (bounds && boundsIntersect(bounds, window)) {
      areaIds.push(area.id);
      continue;
    }
    if (ringHasPointInside(area.outer, window)) areaIds.push(area.id);
  }

  return { lanelet_ids: sortIds(laneletIds), area_ids: sortIds(areaIds), approximate: true };
}
