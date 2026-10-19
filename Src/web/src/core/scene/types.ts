import type { BoundsM } from '../types';
import type { Pose2d } from '../pose';

/**
 * W3 版本化场景 JSON（覆盖层，不改写地图夹具）。
 *
 * 字段（schema_version 1，type "web_scene"）：
 *   map_id / map_version   引用当前地图版本（与快照一致才允许套用）
 *   vehicle                车辆引用（ref + path；参数由 v3 夹具解析）
 *   start / goal           起终位姿（Pose2d，可缺省为 null）
 *   obstacles[]            测试障碍（世界坐标米；rectangle / convex_polygon，
 *                          与 tests/fixtures/v3 路径场景的障碍字段一致）
 *   algorithm_params       算法参数占位（W5 消费）
 */

export interface SceneMapRef {
  map_id: string;
  map_version: string;
}

export interface SceneVehicleRef {
  ref: string;
  path: string;
}

/** 轴对齐矩形（含边界；与 v3 夹具 obstacles 字段一致）。 */
export interface RectangleObstacle {
  id: string;
  type: 'rectangle';
  x_min_m: number;
  y_min_m: number;
  x_max_m: number;
  y_max_m: number;
}

/** 凸多边形障碍（顶点按序，含边界）。 */
export interface ConvexPolygonObstacle {
  id: string;
  type: 'convex_polygon';
  vertices: Array<{ x_m: number; y_m: number }>;
}

export type SceneObstacle = RectangleObstacle | ConvexPolygonObstacle;

export interface HybridAstarParams {
  time_budget_s: number;
  node_budget: number;
}

export interface CorridorParams {
  enabled: boolean;
  margin_m: number;
}

export interface AlgorithmParams {
  hybrid_astar: HybridAstarParams;
  corridor: CorridorParams;
}

export interface WebScene {
  schema_version: 1;
  type: 'web_scene';
  name: string;
  map_id: string;
  map_version: string;
  vehicle: SceneVehicleRef;
  start: Pose2d | null;
  goal: Pose2d | null;
  obstacles: SceneObstacle[];
  algorithm_params: AlgorithmParams;
}

export const DEFAULT_ALGORITHM_PARAMS: AlgorithmParams = {
  hybrid_astar: { time_budget_s: 5.0, node_budget: 1_000_000 },
  corridor: { enabled: true, margin_m: 0.5 },
};

export const DEFAULT_VEHICLE_REF: SceneVehicleRef = {
  ref: 'default_test_vehicle',
  path: 'tests/fixtures/v3/vehicles/default.json',
};

export function obstacleBounds(obstacle: SceneObstacle): BoundsM {
  if (obstacle.type === 'rectangle') {
    return {
      min_x_m: Math.min(obstacle.x_min_m, obstacle.x_max_m),
      min_y_m: Math.min(obstacle.y_min_m, obstacle.y_max_m),
      max_x_m: Math.max(obstacle.x_min_m, obstacle.x_max_m),
      max_y_m: Math.max(obstacle.y_min_m, obstacle.y_max_m),
    };
  }
  let minX = Number.POSITIVE_INFINITY;
  let minY = Number.POSITIVE_INFINITY;
  let maxX = Number.NEGATIVE_INFINITY;
  let maxY = Number.NEGATIVE_INFINITY;
  for (const vertex of obstacle.vertices) {
    minX = Math.min(minX, vertex.x_m);
    minY = Math.min(minY, vertex.y_m);
    maxX = Math.max(maxX, vertex.x_m);
    maxY = Math.max(maxY, vertex.y_m);
  }
  return { min_x_m: minX, min_y_m: minY, max_x_m: maxX, max_y_m: maxY };
}
