import type { Lanelet, MapSnapshot } from './types';

/**
 * 本地命中测试（服务离线回退；服务在线时以 `/map/query/point` 为准）。
 * 纯几何：lanelet 先做左右边界构成的闭合多边形判定，未命中再按中心线
 * 距离容差判定；area 用外环/内环 evenodd 判定。结果按数字 ID 升序。
 */

export type XY = readonly [number, number];

export interface HitResult {
  lanelet_ids: string[];
  area_ids: string[];
}

export interface HitOptions {
  /** 中心线距离容差（米），默认 2 m；用于窄 lanelet 或边界不闭合的情况。 */
  centerline_tolerance_m?: number;
}

export function pointInRing(ring: readonly XY[], x: number, y: number): boolean {
  let inside = false;
  for (let i = 0, j = ring.length - 1; i < ring.length; j = i, i += 1) {
    const [xi, yi] = ring[i];
    const [xj, yj] = ring[j];
    const crosses = yi > y !== yj > y;
    if (crosses && x < ((xj - xi) * (y - yi)) / (yj - yi) + xi) {
      inside = !inside;
    }
  }
  return inside;
}

export function pointInRingsEvenOdd(rings: readonly (readonly XY[])[], x: number, y: number): boolean {
  let inside = false;
  for (const ring of rings) {
    if (pointInRing(ring, x, y)) inside = !inside;
  }
  return inside;
}

export function laneletPolygon(lanelet: Lanelet): XY[] {
  const left = lanelet.left.map((point): XY => [point.x_m, point.y_m]);
  const right = lanelet.right.map((point): XY => [point.x_m, point.y_m]).reverse();
  return [...left, ...right];
}

export function distanceToSegment(
  px: number,
  py: number,
  ax: number,
  ay: number,
  bx: number,
  by: number,
): number {
  const dx = bx - ax;
  const dy = by - ay;
  const lengthSq = dx * dx + dy * dy;
  if (lengthSq === 0) return Math.hypot(px - ax, py - ay);
  let t = ((px - ax) * dx + (py - ay) * dy) / lengthSq;
  t = Math.max(0, Math.min(1, t));
  return Math.hypot(px - (ax + t * dx), py - (ay + t * dy));
}

export function distanceToPolyline(points: readonly XY[], x: number, y: number): number {
  if (points.length === 0) return Number.POSITIVE_INFINITY;
  if (points.length === 1) return Math.hypot(x - points[0][0], y - points[0][1]);
  let best = Number.POSITIVE_INFINITY;
  for (let index = 0; index + 1 < points.length; index += 1) {
    const [ax, ay] = points[index];
    const [bx, by] = points[index + 1];
    best = Math.min(best, distanceToSegment(x, y, ax, ay, bx, by));
  }
  return best;
}

/** 数字 ID 升序（非数字/混合时退回字典序）。 */
export function sortIds(ids: readonly string[]): string[] {
  return [...ids].sort((a, b) => {
    const na = Number(a);
    const nb = Number(b);
    if (Number.isFinite(na) && Number.isFinite(nb) && na !== nb) return na - nb;
    return a < b ? -1 : a > b ? 1 : 0;
  });
}

export function hitTestSnapshot(
  snapshot: MapSnapshot,
  x_m: number,
  y_m: number,
  options: HitOptions = {},
): HitResult {
  const tolerance = options.centerline_tolerance_m ?? 2.0;
  const laneletIds: string[] = [];
  for (const lanelet of snapshot.lanelets) {
    const polygon = laneletPolygon(lanelet);
    let hit = polygon.length >= 3 && pointInRing(polygon, x_m, y_m);
    if (!hit) {
      const centerline = lanelet.centerline && lanelet.centerline.length >= 2 ? lanelet.centerline : lanelet.left;
      const points = centerline.map((point): XY => [point.x_m, point.y_m]);
      hit = points.length >= 2 && distanceToPolyline(points, x_m, y_m) <= tolerance;
    }
    if (hit) laneletIds.push(lanelet.id);
  }

  const areaIds: string[] = [];
  for (const area of snapshot.areas) {
    const rings = [area.outer, ...area.inner]
      .filter((ring) => ring.length >= 3)
      .map((ring) => ring.map((point): XY => [point.x_m, point.y_m]));
    if (rings.length > 0 && pointInRingsEvenOdd(rings, x_m, y_m)) {
      areaIds.push(area.id);
    }
  }

  return { lanelet_ids: sortIds(laneletIds), area_ids: sortIds(areaIds) };
}
