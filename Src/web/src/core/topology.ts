import { sortIds } from './hit';
import type { MapSnapshot } from './types';

/**
 * 拓扑查询的本地近似（服务离线回退）：
 *  - following：中心线起点与所选 lanelet 中心线终点重合（容差内）的 lanelet；
 *  - preceding：中心线终点与所选中心线起点重合的 lanelet；
 *  - adjacent：本地无可靠近似（返回空并由调用方标注“需要服务”）。
 * 注意：Lanelet2 的双向 lanelet 反向通行建模为独立路由顶点，本地近似只覆盖
 * 存储方向的连接；结果一律标注为“本地近似”，权威结果以服务为准。
 */

export const LOCAL_TOPOLOGY_TOLERANCE_M = 1.0;

function centerlineEnds(
  snapshot: MapSnapshot,
  laneletId: string,
): { start: { x_m: number; y_m: number } | null; end: { x_m: number; y_m: number } | null } {
  const lanelet = snapshot.lanelets.find((item) => item.id === laneletId);
  const centerline = lanelet?.centerline;
  if (!centerline || centerline.length < 2) return { start: null, end: null };
  return { start: centerline[0], end: centerline[centerline.length - 1] };
}

export function localFollowing(snapshot: MapSnapshot, laneletId: string): string[] {
  const { end } = centerlineEnds(snapshot, laneletId);
  if (!end) return [];
  const result: string[] = [];
  for (const other of snapshot.lanelets) {
    if (other.id === laneletId) continue;
    const centerline = other.centerline;
    if (!centerline || centerline.length < 2) continue;
    const start = centerline[0];
    if (Math.hypot(start.x_m - end.x_m, start.y_m - end.y_m) <= LOCAL_TOPOLOGY_TOLERANCE_M) {
      result.push(other.id);
    }
  }
  return sortIds(result);
}

export function localPreceding(snapshot: MapSnapshot, laneletId: string): string[] {
  const { start } = centerlineEnds(snapshot, laneletId);
  if (!start) return [];
  const result: string[] = [];
  for (const other of snapshot.lanelets) {
    if (other.id === laneletId) continue;
    const centerline = other.centerline;
    if (!centerline || centerline.length < 2) continue;
    const end = centerline[centerline.length - 1];
    if (Math.hypot(end.x_m - start.x_m, end.y_m - start.y_m) <= LOCAL_TOPOLOGY_TOLERANCE_M) {
      result.push(other.id);
    }
  }
  return sortIds(result);
}
