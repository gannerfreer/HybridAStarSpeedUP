import { normalizeYaw, type Pose2d } from '../pose';
import type { PathPoint, PathSegment, PlaybackPoint, TrajectoryPoint } from '../playback/types';
import type {
  PlanCorridorPolygon,
  PlanEvent,
  PlanJobStatus,
  PlanResult,
  PlanSubmitResponse,
  PlanValidation,
} from './types';

/** 规划服务响应解析（纯函数；容忍 path 分段/平铺与 corridor 包装两种形状）。 */

export class PlanParseError extends Error {
  readonly path: string;

  constructor(path: string, message: string) {
    super(`${path}: ${message}`);
    this.name = 'PlanParseError';
    this.path = path;
  }
}

function fail(path: string, message: string): never {
  throw new PlanParseError(path, message);
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null && !Array.isArray(value);
}

function expectRecord(value: unknown, path: string): Record<string, unknown> {
  if (!isRecord(value)) fail(path, `expected object, got ${value === null ? 'null' : Array.isArray(value) ? 'array' : typeof value}`);
  return value;
}

function expectArray(value: unknown, path: string): unknown[] {
  if (!Array.isArray(value)) fail(path, `expected array, got ${typeof value}`);
  return value;
}

function expectString(value: unknown, path: string, allowEmpty = true): string {
  if (typeof value !== 'string') fail(path, 'expected string');
  if (!allowEmpty && value.length === 0) fail(path, 'must not be empty');
  return value;
}

function expectFinite(value: unknown, path: string): number {
  if (typeof value !== 'number' || !Number.isFinite(value)) fail(path, 'expected finite number');
  return value;
}

function optionalFinite(value: unknown, path: string): number | undefined {
  return value === undefined || value === null ? undefined : expectFinite(value, path);
}

/** 协议 §2.2/§5：scene_id 为 64 位小写十六进制 SHA-256；缺省容忍（旧响应）。 */
function optionalSceneId(value: unknown, path: string): string | undefined {
  if (value === undefined || value === null) return undefined;
  const text = expectString(value, path, false);
  if (!/^[0-9a-f]{64}$/.test(text)) fail(path, 'expected 64-hex scene_id');
  return text;
}

function optionalSceneVersion(value: unknown, path: string): string | undefined {
  if (value === undefined || value === null) return undefined;
  return expectString(value, path, false);
}

function parsePose(value: unknown, path: string): Pose2d {
  const record = expectRecord(value, path);
  return {
    x_m: expectFinite(record.x_m, `${path}.x_m`),
    y_m: expectFinite(record.y_m, `${path}.y_m`),
    yaw_rad: normalizeYaw(expectFinite(record.yaw_rad, `${path}.yaw_rad`)),
  };
}

function parsePathPoint(value: unknown, path: string): PathPoint {
  const record = expectRecord(value, path);
  const direction = record.direction === undefined ? 'forward' : expectString(record.direction, `${path}.direction`);
  if (direction !== 'forward' && direction !== 'reverse') fail(`${path}.direction`, `expected forward|reverse`);
  return {
    pose: parsePose(record.pose, `${path}.pose`),
    s_m: optionalFinite(record.s_m, `${path}.s_m`) ?? 0,
    curvature_inv_m: optionalFinite(record.curvature_inv_m, `${path}.curvature_inv_m`) ?? 0,
    direction,
  };
}

function parsePath(value: unknown, path: string): PathSegment[] {
  const entries = expectArray(value ?? [], path);
  if (entries.length === 0) return [];
  const first = expectRecord(entries[0], `${path}[0]`);
  if (Array.isArray(first.points)) {
    return entries.map((segment, index) => {
      const record = expectRecord(segment, `${path}[${index}]`);
      return {
        points: expectArray(record.points ?? [], `${path}[${index}].points`).map((point, pIndex) =>
          parsePathPoint(point, `${path}[${index}].points[${pIndex}]`),
        ),
      };
    });
  }
  // 平铺点列 → 单段（按方向拆分连续段）。
  const points = entries.map((point, index) => parsePathPoint(point, `${path}[${index}]`));
  const segments: PathSegment[] = [];
  let current: PathPoint[] = [];
  for (const point of points) {
    if (current.length > 0 && current[current.length - 1].direction !== point.direction) {
      segments.push({ points: current });
      current = [];
    }
    current.push(point);
  }
  if (current.length > 0) segments.push({ points: current });
  return segments;
}

function parseTrajectory(value: unknown, path: string): TrajectoryPoint[] {
  return expectArray(value ?? [], path).map((point, index) => {
    const record = expectRecord(point, `${path}[${index}]`);
    return {
      t_s: expectFinite(record.t_s, `${path}[${index}].t_s`),
      pose: parsePose(record.pose, `${path}[${index}].pose`),
      velocity_mps: optionalFinite(record.velocity_mps, `${path}[${index}].velocity_mps`) ?? 0,
      acceleration_mps2: optionalFinite(record.acceleration_mps2, `${path}[${index}].acceleration_mps2`) ?? 0,
      curvature_inv_m: optionalFinite(record.curvature_inv_m, `${path}[${index}].curvature_inv_m`) ?? 0,
    };
  });
}

function parseCorridorPolygon(value: unknown, path: string): PlanCorridorPolygon {
  const record = expectRecord(value, path);
  return {
    s_start_m: expectFinite(record.s_start_m, `${path}.s_start_m`),
    s_end_m: expectFinite(record.s_end_m, `${path}.s_end_m`),
    vertices: expectArray(record.vertices, `${path}.vertices`).map((vertex, index) => {
      const point = expectRecord(vertex, `${path}.vertices[${index}]`);
      return {
        x_m: expectFinite(point.x_m, `${path}.vertices[${index}].x_m`),
        y_m: expectFinite(point.y_m, `${path}.vertices[${index}].y_m`),
      } satisfies PlaybackPoint;
    }),
  };
}

function parseCorridor(value: unknown, path: string): PlanCorridorPolygon[] {
  if (value === undefined || value === null) return [];
  if (Array.isArray(value)) {
    return value.map((polygon, index) => parseCorridorPolygon(polygon, `${path}[${index}]`));
  }
  const record = expectRecord(value, path);
  const polygons = record.polygons ?? [];
  return expectArray(polygons, `${path}.polygons`).map((polygon, index) =>
    parseCorridorPolygon(polygon, `${path}.polygons[${index}]`),
  );
}

function parseValidation(value: unknown, path: string): PlanValidation | null {
  if (value === undefined || value === null) return null;
  const record = expectRecord(value, path);
  return {
    valid: record.valid === true,
    reason: record.reason === undefined ? '' : expectString(record.reason, `${path}.reason`),
  };
}

function parseStatistics(value: unknown, path: string): Record<string, unknown> {
  if (value === undefined || value === null) return {};
  return { ...expectRecord(value, path) };
}

/** Hybrid 段 forward/reverse 交替次数（真实服务未直接给出时的派生值）。 */
function countDirectionChanges(segments: readonly PathSegment[]): number {
  let changes = 0;
  let previous: string | null = null;
  for (const segment of segments) {
    for (const point of segment.points) {
      if (previous !== null && point.direction !== previous) changes += 1;
      previous = point.direction;
    }
  }
  return changes;
}

/**
 * 真实 planner_plan_service 统计 → 展示字段：
 * 展平 `statistics.hybrid`（适配器统计），派生别名
 * total_length_m/planning_time_s/solve_time_s/direction_changes。
 */
function normalizeStatistics(raw: Record<string, unknown>, path: readonly PathSegment[]): Record<string, unknown> {
  const hybrid = isRecord(raw.hybrid) ? raw.hybrid : {};
  const timings = isRecord(raw.timings_s) ? raw.timings_s : {};
  const merged: Record<string, unknown> = { ...raw, ...hybrid };
  const numberOr = (value: unknown): number | undefined =>
    typeof value === 'number' && Number.isFinite(value) ? value : undefined;
  if (numberOr(merged.total_length_m) === undefined) {
    const length = numberOr(raw.path_length_m) ?? numberOr(hybrid.path_length_m);
    if (length !== undefined) merged.total_length_m = length;
  }
  if (numberOr(merged.planning_time_s) === undefined) {
    const hybridTime = numberOr(timings.hybrid);
    const searchMs = numberOr(hybrid.search_time_ms);
    if (hybridTime !== undefined) merged.planning_time_s = hybridTime;
    else if (searchMs !== undefined) merged.planning_time_s = searchMs / 1000;
  }
  if (numberOr(merged.solve_time_s) === undefined) {
    const optimizeTime = numberOr(timings.optimize);
    if (optimizeTime !== undefined) merged.solve_time_s = optimizeTime;
  }
  if (numberOr(merged.direction_changes) === undefined) {
    merged.direction_changes = countDirectionChanges(path);
  }
  return merged;
}

export function parseSubmitResponse(value: unknown): PlanSubmitResponse {
  const root = expectRecord(value, '$');
  return {
    job_id: expectString(root.job_id, '$.job_id', false),
    job_seq: expectFinite(root.job_seq, '$.job_seq'),
    request_id: expectString(root.request_id ?? '', '$.request_id'),
    map_id: expectString(root.map_id ?? '', '$.map_id'),
    map_version: expectString(root.map_version ?? '', '$.map_version'),
    status: expectString(root.status ?? 'queued', '$.status'),
  };
}

export function parseJobStatus(value: unknown): PlanJobStatus {
  const root = expectRecord(value, '$');
  return {
    job_id: expectString(root.job_id ?? '', '$.job_id'),
    status: expectString(root.status, '$.status', false),
    stage: root.stage === undefined ? undefined : expectString(root.stage, '$.stage'),
    reason: root.reason === undefined ? undefined : expectString(root.reason, '$.reason'),
    job_seq: optionalFinite(root.job_seq, '$.job_seq'),
    scene_id: optionalSceneId(root.scene_id, '$.scene_id'),
    scene_version: optionalSceneVersion(root.scene_version, '$.scene_version'),
  };
}

export function parsePlanResult(value: unknown): PlanResult {
  const root = expectRecord(value, '$');
  const hybrid = isRecord(root.hybrid) ? root.hybrid : {};
  const path = parsePath(root.path ?? hybrid.segments, '$.path');
  return {
    job_id: expectString(root.job_id ?? '', '$.job_id'),
    job_seq: optionalFinite(root.job_seq, '$.job_seq') ?? 0,
    request_id: expectString(root.request_id ?? '', '$.request_id'),
    map_id: expectString(root.map_id ?? '', '$.map_id'),
    map_version: expectString(root.map_version ?? '', '$.map_version'),
    status: expectString(root.status, '$.status', false),
    reason: root.reason === undefined ? '' : expectString(root.reason, '$.reason'),
    scene_id: optionalSceneId(root.scene_id, '$.scene_id'),
    scene_version: optionalSceneVersion(root.scene_version, '$.scene_version'),
    path,
    corridor: parseCorridor(root.corridor, '$.corridor'),
    trajectory: parseTrajectory(root.trajectory, '$.trajectory'),
    validation: parseValidation(root.validation, '$.validation'),
    statistics: normalizeStatistics(parseStatistics(root.statistics, '$.statistics'), path),
  };
}

export function parsePlanEvent(value: unknown): PlanEvent {
  const root = expectRecord(value, '$');
  const kind = expectString(root.kind ?? root.type ?? '', '$.kind');
  if (kind !== 'job_update' && kind !== 'job_result') {
    fail('$.kind', `expected job_update|job_result, got ${JSON.stringify(kind)}`);
  }
  return {
    kind,
    job_seq: expectFinite(root.job_seq, '$.job_seq'),
    job_id: expectString(root.job_id, '$.job_id', false),
    status: root.status === undefined ? undefined : expectString(root.status, '$.status'),
    stage: root.stage === undefined ? undefined : expectString(root.stage, '$.stage'),
    reason: root.reason === undefined ? undefined : expectString(root.reason, '$.reason'),
    scene_id: optionalSceneId(root.scene_id, '$.scene_id'),
    result: root.result === undefined ? undefined : parsePlanResult(root.result),
  };
}
