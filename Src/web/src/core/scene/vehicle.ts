import type { Pose2d } from '../pose';

/**
 * W3 车辆模型（公共 VehicleModel，§6.1 字段语义）与整车足迹几何。
 * 车辆系：x 向前、y 向左；后轴中心为原点。车体矩形 x ∈ [-rear_overhang,
 * wheelbase + front_overhang]、y ∈ [±width/2]；安全裕量向外扩展 margin。
 */

export interface VehicleModel {
  id: string;
  wheelbase_m: number;
  width_m: number;
  front_overhang_m: number;
  rear_overhang_m: number;
  max_steer_rad: number;
  safety_margin_m: number;
  footprint_shape: 'rectangle';
}

export class VehicleParseError extends Error {
  readonly path: string;

  constructor(path: string, message: string) {
    super(`${path}: ${message}`);
    this.name = 'VehicleParseError';
    this.path = path;
  }
}

function fail(path: string, message: string): never {
  throw new VehicleParseError(path, message);
}

function expectRecord(value: unknown, path: string): Record<string, unknown> {
  if (typeof value !== 'object' || value === null || Array.isArray(value)) {
    fail(path, `expected object, got ${value === null ? 'null' : Array.isArray(value) ? 'array' : typeof value}`);
  }
  return value as Record<string, unknown>;
}

function expectFinite(value: unknown, path: string): number {
  if (typeof value !== 'number' || !Number.isFinite(value)) fail(path, 'expected finite number');
  return value;
}

export function parseVehicleModel(value: unknown): VehicleModel {
  const root = expectRecord(value, '$');
  if (root.schema_version !== 1) fail('$.schema_version', 'unsupported schema_version (expected 1)');
  if (root.type !== 'vehicle_model') fail('$.type', `expected "vehicle_model", got ${String(root.type)}`);
  const model: VehicleModel = {
    id: typeof root.id === 'string' && root.id.length > 0 ? root.id : fail('$.id', 'must be a non-empty string'),
    wheelbase_m: expectFinite(root.wheelbase_m, '$.wheelbase_m'),
    width_m: expectFinite(root.width_m, '$.width_m'),
    front_overhang_m: expectFinite(root.front_overhang_m, '$.front_overhang_m'),
    rear_overhang_m: expectFinite(root.rear_overhang_m, '$.rear_overhang_m'),
    max_steer_rad: expectFinite(root.max_steer_rad, '$.max_steer_rad'),
    safety_margin_m: expectFinite(root.safety_margin_m, '$.safety_margin_m'),
    footprint_shape: 'rectangle',
  };
  if (model.wheelbase_m <= 0) fail('$.wheelbase_m', 'must be > 0');
  if (model.width_m <= 0) fail('$.width_m', 'must be > 0');
  if (model.front_overhang_m < 0 || model.rear_overhang_m < 0) {
    fail('$.front_overhang_m', 'overhangs must be >= 0');
  }
  if (!(model.max_steer_rad > 0 && model.max_steer_rad < Math.PI / 2)) {
    fail('$.max_steer_rad', 'must be in (0, pi/2)');
  }
  if (model.safety_margin_m < 0) fail('$.safety_margin_m', 'must be >= 0');
  if (root.footprint_shape !== undefined && root.footprint_shape !== 'rectangle') {
    fail('$.footprint_shape', `unsupported footprint shape ${String(root.footprint_shape)}`);
  }
  return model;
}

export function minTurnRadiusM(model: VehicleModel): number {
  return model.wheelbase_m / Math.tan(model.max_steer_rad);
}

export function maxCurvature1PerM(model: VehicleModel): number {
  return Math.tan(model.max_steer_rad) / model.wheelbase_m;
}

export function vehicleLengthM(model: VehicleModel): number {
  return model.rear_overhang_m + model.wheelbase_m + model.front_overhang_m;
}

type XY = [number, number];

/** 车体矩形（车辆系，后轴中心原点）。 */
export function bodyCorners(model: VehicleModel): XY[] {
  const front = model.wheelbase_m + model.front_overhang_m;
  const rear = -model.rear_overhang_m;
  const half = model.width_m / 2;
  return [
    [front, half],
    [front, -half],
    [rear, -half],
    [rear, half],
  ];
}

/** 含安全裕量的矩形（车辆系；向外扩展 margin）。 */
export function marginCorners(model: VehicleModel): XY[] {
  const front = model.wheelbase_m + model.front_overhang_m + model.safety_margin_m;
  const rear = -model.rear_overhang_m - model.safety_margin_m;
  const half = model.width_m / 2 + model.safety_margin_m;
  return [
    [front, half],
    [front, -half],
    [rear, -half],
    [rear, half],
  ];
}

/** 车辆系点 → 世界坐标（位姿旋转 + 平移）。 */
export function footprintWorldCorners(model: VehicleModel, pose: Pose2d, useMargin: boolean): XY[] {
  const cos = Math.cos(pose.yaw_rad);
  const sin = Math.sin(pose.yaw_rad);
  const corners = useMargin ? marginCorners(model) : bodyCorners(model);
  return corners.map(([x, y]): XY => [pose.x_m + cos * x - sin * y, pose.y_m + sin * x + cos * y]);
}

/** 足迹在给定位姿下是否与障碍相交（矩形足迹 vs 矩形/多边形障碍的保守包围盒检查）。 */
export function footprintIntersectsObstacleBounds(
  model: VehicleModel,
  pose: Pose2d,
  bounds: { min_x_m: number; min_y_m: number; max_x_m: number; max_y_m: number },
  useMargin = true,
): boolean {
  const corners = footprintWorldCorners(model, pose, useMargin);
  let minX = Number.POSITIVE_INFINITY;
  let minY = Number.POSITIVE_INFINITY;
  let maxX = Number.NEGATIVE_INFINITY;
  let maxY = Number.NEGATIVE_INFINITY;
  for (const [x, y] of corners) {
    minX = Math.min(minX, x);
    minY = Math.min(minY, y);
    maxX = Math.max(maxX, x);
    maxY = Math.max(maxY, y);
  }
  return !(maxX < bounds.min_x_m || minX > bounds.max_x_m || maxY < bounds.min_y_m || minY > bounds.max_y_m);
}
