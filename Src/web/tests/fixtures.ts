import type { Lanelet, MapArea, MapPoint, MapSnapshot } from '../src/core/types';

/** 合成快照构造器（W2 测试共用；与真实 v3 夹具结构一致的最小合法对象）。 */

export function point(id: string, x: number, y: number, z = 0): MapPoint {
  return { id, x_m: x, y_m: y, z_m: z };
}

export function lanelet(
  id: string,
  left: MapPoint[],
  right: MapPoint[],
  centerline?: MapPoint[],
  attributes: Record<string, string> = {},
): Lanelet {
  return { id, attributes, left, right, ...(centerline ? { centerline } : {}) };
}

/** 一条沿 x 轴、宽 4 m（y ±2）的直线 lanelet：x0→x1。 */
export function straightLanelet(id: string, x0: number, x1: number, yCenter = 0): Lanelet {
  const half = 2;
  return lanelet(
    id,
    [point(`${id}-l0`, x0, yCenter + half), point(`${id}-l1`, x1, yCenter + half)],
    [point(`${id}-r0`, x0, yCenter - half), point(`${id}-r1`, x1, yCenter - half)],
    [point(`${id}-c0`, x0, yCenter), point(`${id}-c1`, x1, yCenter)],
    { type: 'lanelet', subtype: 'road' },
  );
}

export function area(id: string, outer: MapPoint[], inner: MapPoint[][] = []): MapArea {
  return { id, attributes: { type: 'multipolygon' }, outer, inner };
}

export function makeSnapshot(options: { lanelets?: Lanelet[]; areas?: MapArea[] } = {}): MapSnapshot {
  const lanelets = options.lanelets ?? [straightLanelet('100', 0, 10)];
  const areas = options.areas ?? [];
  const xs: number[] = [];
  const ys: number[] = [];
  for (const item of lanelets) {
    for (const p of [...item.left, ...item.right, ...(item.centerline ?? [])]) {
      xs.push(p.x_m);
      ys.push(p.y_m);
    }
  }
  for (const item of areas) {
    for (const p of [...item.outer, ...item.inner.flat()]) {
      xs.push(p.x_m);
      ys.push(p.y_m);
    }
  }
  const minX = Math.min(...xs, 0);
  const maxX = Math.max(...xs, 10);
  const minY = Math.min(...ys, 0);
  const maxY = Math.max(...ys, 10);
  return {
    schema_version: 1,
    type: 'map_snapshot',
    map_id: 'synthetic_test',
    map_version: 'f'.repeat(64),
    source: { path: 'synthetic.osm', bytes: 1 },
    coordinate_system: {
      frame_id: 'map',
      unit: 'm',
      x_axis: 'east',
      y_axis: 'north',
      projection: 'utm',
      origin: { lat_deg: 49, lon_deg: 8, ele_m: 0 },
    },
    bounds_m: { min_x_m: minX, min_y_m: minY, max_x_m: maxX, max_y_m: maxY },
    counts: {
      lanelets: lanelets.length,
      areas: areas.length,
      points: 0,
      line_strings: 0,
      polygons: 0,
      regulatory_elements: 0,
    },
    traffic_rules: { location: 'de', participant: 'vehicle' },
    lanelets,
    areas,
    regulatory_elements: [],
  };
}
