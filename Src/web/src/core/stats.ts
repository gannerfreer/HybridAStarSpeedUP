import type { MapSnapshot } from './types';

/** 由快照几何直接推导的对象计数（用于与顶层 `counts` 交叉核对和渲染检查证据）。 */
export interface SnapshotStats {
  lanelets: number;
  lanelets_with_centerline: number;
  areas: number;
  areas_with_inner: number;
  /** 左右边界点总数（跨 lanelet 不去重）。 */
  boundary_points: number;
  centerline_points: number;
  /** 外环 + 内环点总数。 */
  area_points: number;
  regulatory_elements: number;
}

export function computeSnapshotStats(snapshot: MapSnapshot): SnapshotStats {
  let laneletsWithCenterline = 0;
  let boundaryPoints = 0;
  let centerlinePoints = 0;
  for (const lanelet of snapshot.lanelets) {
    boundaryPoints += lanelet.left.length + lanelet.right.length;
    if (lanelet.centerline && lanelet.centerline.length > 0) {
      laneletsWithCenterline += 1;
      centerlinePoints += lanelet.centerline.length;
    }
  }

  let areasWithInner = 0;
  let areaPoints = 0;
  for (const area of snapshot.areas) {
    if (area.inner.length > 0) areasWithInner += 1;
    areaPoints += area.outer.length;
    for (const ring of area.inner) areaPoints += ring.length;
  }

  return {
    lanelets: snapshot.lanelets.length,
    lanelets_with_centerline: laneletsWithCenterline,
    areas: snapshot.areas.length,
    areas_with_inner: areasWithInner,
    boundary_points: boundaryPoints,
    centerline_points: centerlinePoints,
    area_points: areaPoints,
    regulatory_elements: snapshot.regulatory_elements.length,
  };
}
