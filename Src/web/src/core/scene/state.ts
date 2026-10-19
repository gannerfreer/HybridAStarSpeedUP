import type { BoundsM, MapSnapshot } from '../types';
import type { Pose2d } from '../pose';
import type { AlgorithmParams, SceneObstacle, SceneVehicleRef, WebScene } from './types';
import { DEFAULT_ALGORITHM_PARAMS, DEFAULT_VEHICLE_REF } from './types';

/** 从当前编辑状态组装场景（纯函数；地图数据只读引用）。 */
export function buildScene(options: {
  map: Pick<MapSnapshot, 'map_id' | 'map_version'>;
  vehicle: SceneVehicleRef;
  start: Pose2d | null;
  goal: Pose2d | null;
  obstacles: readonly SceneObstacle[];
  algorithmParams: AlgorithmParams;
  name?: string;
}): WebScene {
  return {
    schema_version: 1,
    type: 'web_scene',
    name: options.name ?? '',
    map_id: options.map.map_id,
    map_version: options.map.map_version,
    vehicle: { ...options.vehicle },
    start: options.start ? { ...options.start } : null,
    goal: options.goal ? { ...options.goal } : null,
    obstacles: options.obstacles.map((obstacle) => ({ ...obstacle })),
    algorithm_params: {
      hybrid_astar: { ...options.algorithmParams.hybrid_astar },
      corridor: { ...options.algorithmParams.corridor },
    },
  };
}

/** 场景是否为空（无障碍、无位姿、参数为默认值）。 */
export function isSceneEmpty(scene: {
  start: Pose2d | null;
  goal: Pose2d | null;
  obstacles: readonly SceneObstacle[];
  algorithm_params: AlgorithmParams;
}): boolean {
  return (
    scene.obstacles.length === 0 &&
    scene.start === null &&
    scene.goal === null &&
    JSON.stringify(scene.algorithm_params) === JSON.stringify(DEFAULT_ALGORITHM_PARAMS)
  );
}

export { DEFAULT_ALGORITHM_PARAMS, DEFAULT_VEHICLE_REF };

/** 障碍包围盒并集（供面板概览与 CDP 检查）。 */
export function obstaclesBounds(obstacles: readonly SceneObstacle[]): BoundsM | null {
  if (obstacles.length === 0) return null;
  let minX = Number.POSITIVE_INFINITY;
  let minY = Number.POSITIVE_INFINITY;
  let maxX = Number.NEGATIVE_INFINITY;
  let maxY = Number.NEGATIVE_INFINITY;
  for (const obstacle of obstacles) {
    if (obstacle.type === 'rectangle') {
      minX = Math.min(minX, obstacle.x_min_m, obstacle.x_max_m);
      minY = Math.min(minY, obstacle.y_min_m, obstacle.y_max_m);
      maxX = Math.max(maxX, obstacle.x_min_m, obstacle.x_max_m);
      maxY = Math.max(maxY, obstacle.y_min_m, obstacle.y_max_m);
    } else {
      for (const vertex of obstacle.vertices) {
        minX = Math.min(minX, vertex.x_m);
        minY = Math.min(minY, vertex.y_m);
        maxX = Math.max(maxX, vertex.x_m);
        maxY = Math.max(maxY, vertex.y_m);
      }
    }
  }
  return { min_x_m: minX, min_y_m: minY, max_x_m: maxX, max_y_m: maxY };
}
