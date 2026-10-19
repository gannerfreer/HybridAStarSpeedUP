import { normalizeYaw, type Pose2d } from '../pose';
import { sortObstacles } from './obstacles';
import type {
  AlgorithmParams,
  ConvexPolygonObstacle,
  RectangleObstacle,
  SceneObstacle,
  WebScene,
} from './types';
import { DEFAULT_ALGORITHM_PARAMS, DEFAULT_VEHICLE_REF } from './types';

/**
 * 场景 JSON 解析/序列化与版本校验（纯函数）。
 * 校验规则：公共包络（schema_version 1 + type "web_scene"）；必填字段；
 * 数值有限；障碍类型/几何合法（矩形非退化、多边形 ≥3 顶点）；未知字段容忍。
 */

export class SceneParseError extends Error {
  readonly path: string;

  constructor(path: string, message: string) {
    super(`${path}: ${message}`);
    this.name = 'SceneParseError';
    this.path = path;
  }
}

function fail(path: string, message: string): never {
  throw new SceneParseError(path, message);
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null && !Array.isArray(value);
}

function describeValue(value: unknown): string {
  if (value === null) return 'null';
  if (Array.isArray(value)) return 'array';
  if (typeof value === 'string') return JSON.stringify(value);
  if (typeof value === 'number') return String(value);
  return typeof value;
}

function expectRecord(value: unknown, path: string): Record<string, unknown> {
  if (!isRecord(value)) fail(path, `expected object, got ${describeValue(value)}`);
  return value;
}

function expectArray(value: unknown, path: string): unknown[] {
  if (!Array.isArray(value)) fail(path, `expected array, got ${describeValue(value)}`);
  return value;
}

function expectString(value: unknown, path: string, allowEmpty = false): string {
  if (typeof value !== 'string') fail(path, `expected string, got ${describeValue(value)}`);
  if (!allowEmpty && value.length === 0) fail(path, 'must not be empty');
  return value;
}

function expectFiniteNumber(value: unknown, path: string): number {
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    fail(path, `expected finite number, got ${describeValue(value)}`);
  }
  return value;
}

function expectBoolean(value: unknown, path: string): boolean {
  if (typeof value !== 'boolean') fail(path, `expected boolean, got ${describeValue(value)}`);
  return value;
}

function parsePose(value: unknown, path: string): Pose2d | null {
  if (value === null || value === undefined) return null;
  const record = expectRecord(value, path);
  return {
    x_m: expectFiniteNumber(record.x_m, `${path}.x_m`),
    y_m: expectFiniteNumber(record.y_m, `${path}.y_m`),
    yaw_rad: normalizeYaw(expectFiniteNumber(record.yaw_rad, `${path}.yaw_rad`)),
  };
}

function parseObstacle(value: unknown, path: string): SceneObstacle {
  const record = expectRecord(value, path);
  const id = expectString(record.id, `${path}.id`);
  const type = expectString(record.type, `${path}.type`);
  if (type === 'rectangle') {
    const obstacle: RectangleObstacle = {
      id,
      type: 'rectangle',
      x_min_m: expectFiniteNumber(record.x_min_m, `${path}.x_min_m`),
      y_min_m: expectFiniteNumber(record.y_min_m, `${path}.y_min_m`),
      x_max_m: expectFiniteNumber(record.x_max_m, `${path}.x_max_m`),
      y_max_m: expectFiniteNumber(record.y_max_m, `${path}.y_max_m`),
    };
    if (obstacle.x_min_m >= obstacle.x_max_m || obstacle.y_min_m >= obstacle.y_max_m) {
      fail(path, 'rectangle must have positive extent (x_min < x_max, y_min < y_max)');
    }
    return obstacle;
  }
  if (type === 'convex_polygon') {
    const vertices = expectArray(record.vertices, `${path}.vertices`).map((vertex, index) => {
      const point = expectRecord(vertex, `${path}.vertices[${index}]`);
      return {
        x_m: expectFiniteNumber(point.x_m, `${path}.vertices[${index}].x_m`),
        y_m: expectFiniteNumber(point.y_m, `${path}.vertices[${index}].y_m`),
      };
    });
    if (vertices.length < 3) fail(`${path}.vertices`, 'convex polygon needs at least 3 vertices');
    const obstacle: ConvexPolygonObstacle = { id, type: 'convex_polygon', vertices };
    return obstacle;
  }
  fail(`${path}.type`, `unsupported obstacle type ${describeValue(type)}`);
}

function parseAlgorithmParams(value: unknown, path: string): AlgorithmParams {
  const record = expectRecord(value, path);
  const hybrid = expectRecord(record.hybrid_astar ?? {}, `${path}.hybrid_astar`);
  const corridor = expectRecord(record.corridor ?? {}, `${path}.corridor`);
  const timeBudget =
    hybrid.time_budget_s === undefined
      ? DEFAULT_ALGORITHM_PARAMS.hybrid_astar.time_budget_s
      : expectFiniteNumber(hybrid.time_budget_s, `${path}.hybrid_astar.time_budget_s`);
  const nodeBudget =
    hybrid.node_budget === undefined
      ? DEFAULT_ALGORITHM_PARAMS.hybrid_astar.node_budget
      : expectFiniteNumber(hybrid.node_budget, `${path}.hybrid_astar.node_budget`);
  if (timeBudget <= 0) fail(`${path}.hybrid_astar.time_budget_s`, 'must be > 0');
  if (nodeBudget <= 0) fail(`${path}.hybrid_astar.node_budget`, 'must be > 0');
  const margin =
    corridor.margin_m === undefined
      ? DEFAULT_ALGORITHM_PARAMS.corridor.margin_m
      : expectFiniteNumber(corridor.margin_m, `${path}.corridor.margin_m`);
  if (margin < 0) fail(`${path}.corridor.margin_m`, 'must be >= 0');
  return {
    hybrid_astar: { time_budget_s: timeBudget, node_budget: nodeBudget },
    corridor: {
      enabled: corridor.enabled === undefined ? true : expectBoolean(corridor.enabled, `${path}.corridor.enabled`),
      margin_m: margin,
    },
  };
}

export function parseScene(value: unknown): WebScene {
  const root = expectRecord(value, '$');
  if (root.schema_version !== 1) {
    fail('$.schema_version', `unsupported schema_version ${describeValue(root.schema_version)} (expected 1)`);
  }
  if (root.type !== 'web_scene') {
    fail('$.type', `expected "web_scene", got ${describeValue(root.type)}`);
  }
  const vehicleRaw = isRecord(root.vehicle) ? root.vehicle : {};
  const obstacles = expectArray(root.obstacles ?? [], '$.obstacles').map((obstacle, index) =>
    parseObstacle(obstacle, `$.obstacles[${index}]`),
  );
  const ids = new Set<string>();
  for (const obstacle of obstacles) {
    if (ids.has(obstacle.id)) fail('$.obstacles', `duplicate obstacle id ${obstacle.id}`);
    ids.add(obstacle.id);
  }
  return {
    schema_version: 1,
    type: 'web_scene',
    name: root.name === undefined ? '' : expectString(root.name, '$.name', true),
    map_id: expectString(root.map_id, '$.map_id'),
    map_version: expectString(root.map_version, '$.map_version'),
    vehicle: {
      ref:
        vehicleRaw.ref === undefined
          ? DEFAULT_VEHICLE_REF.ref
          : expectString(vehicleRaw.ref, '$.vehicle.ref'),
      path:
        vehicleRaw.path === undefined
          ? DEFAULT_VEHICLE_REF.path
          : expectString(vehicleRaw.path, '$.vehicle.path'),
    },
    start: parsePose(root.start, '$.start'),
    goal: parsePose(root.goal, '$.goal'),
    // 规范序：障碍按自然 ID 排序，保证解析结果与序列化字节确定。
    obstacles: sortObstacles(obstacles),
    algorithm_params: parseAlgorithmParams(root.algorithm_params ?? {}, '$.algorithm_params'),
  };
}

/** 确定性序列化（障碍按自然 ID 排序；键序固定，便于字节级对比与保存）。 */
export function serializeScene(scene: WebScene): string {
  const obstacles = sortObstacles(scene.obstacles);
  return (
    JSON.stringify(
      {
        schema_version: 1,
        type: 'web_scene',
        name: scene.name,
        map_id: scene.map_id,
        map_version: scene.map_version,
        vehicle: { ref: scene.vehicle.ref, path: scene.vehicle.path },
        start: scene.start,
        goal: scene.goal,
        obstacles,
        algorithm_params: scene.algorithm_params,
      },
      null,
      2,
    ) + '\n'
  );
}

export function sceneFromJson(text: string): WebScene {
  let value: unknown;
  try {
    value = JSON.parse(text) as unknown;
  } catch (error) {
    throw new SceneParseError('$', `invalid JSON: ${error instanceof Error ? error.message : String(error)}`);
  }
  return parseScene(value);
}

export interface SceneMapCheck {
  ok: boolean;
  reason: string;
  scene_map_id: string;
  scene_map_version: string;
  current_map_id: string;
  current_map_version: string;
}

/** 场景与当前地图的版本校验：map_id 与 map_version 均需一致才允许套用。 */
export function checkSceneMap(
  scene: Pick<WebScene, 'map_id' | 'map_version'>,
  current: { map_id: string; map_version: string },
): SceneMapCheck {
  const idMatches = scene.map_id === current.map_id;
  const versionMatches = scene.map_version === current.map_version;
  let reason = '';
  if (!idMatches) reason = `地图 ID 不一致（场景 ${scene.map_id} ≠ 当前 ${current.map_id}）`;
  else if (!versionMatches) reason = `地图版本不一致（场景 ${scene.map_version.slice(0, 12)}… ≠ 当前 ${current.map_version.slice(0, 12)}…）`;
  return {
    ok: idMatches && versionMatches,
    reason,
    scene_map_id: scene.map_id,
    scene_map_version: scene.map_version,
    current_map_id: current.map_id,
    current_map_version: current.map_version,
  };
}
