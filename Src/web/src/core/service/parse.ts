import type { BoundsM } from '../types';
import type {
  AreaGeometry,
  ClippedPath,
  CropResult,
  IdListResult,
  LaneletGeometry,
  MapInfoPayload,
  ObjectSummary,
  PointQueryResult,
  RouteResult,
  WirePoint,
} from './types';

/**
 * 服务响应解析（纯函数，可 Node 侧测试）。
 *
 * 契约：公共 JSON 包络（`schema_version: 1` + 字符串 `type`）；
 * 对象 ID 字符串；数值有限；未知字段容忍；必填缺失报错。
 * 查询类响应允许携带 `status`（`success` 之外表示失败，`reason` 给出原因）。
 */

export class ServicePayloadError extends Error {
  readonly path: string;

  constructor(path: string, message: string) {
    super(`${path}: ${message}`);
    this.name = 'ServicePayloadError';
    this.path = path;
  }
}

function fail(path: string, message: string): never {
  throw new ServicePayloadError(path, message);
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

function expectString(value: unknown, path: string, allowEmpty = true): string {
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

function expectStringArray(value: unknown, path: string): string[] {
  return expectArray(value, path).map((item, index) => expectString(item, `${path}[${index}]`));
}

/** 校验公共包络；返回根对象（供各端点解析器继续读取字段）。 */
export function parseEnvelope(value: unknown): Record<string, unknown> {
  const root = expectRecord(value, '$');
  if (root.schema_version !== 1) {
    fail('$.schema_version', `unsupported schema_version ${describeValue(root.schema_version)} (expected 1)`);
  }
  if (typeof root.type !== 'string' || root.type.length === 0) {
    fail('$.type', `expected non-empty string, got ${describeValue(root.type)}`);
  }
  return root;
}

/** 查询失败（status !== "success"）时抛出带 reason 的载荷错误。 */
function expectSuccess(root: Record<string, unknown>): void {
  const status = root.status;
  if (typeof status === 'string' && status !== 'success') {
    const reason = typeof root.reason === 'string' && root.reason.length > 0 ? root.reason : '';
    fail('$.status', `service status=${status}${reason ? `: ${reason}` : ''}`);
  }
}

function parseStatusReason(root: Record<string, unknown>): { status: string; reason: string } {
  return {
    status: typeof root.status === 'string' ? root.status : 'success',
    reason: typeof root.reason === 'string' ? root.reason : '',
  };
}

export function parseWirePoint(value: unknown, path: string): WirePoint {
  const record = expectRecord(value, path);
  return {
    id: expectString(record.id, `${path}.id`),
    x_m: expectFiniteNumber(record.x_m, `${path}.x_m`),
    y_m: expectFiniteNumber(record.y_m, `${path}.y_m`),
    z_m: record.z_m === undefined ? 0 : expectFiniteNumber(record.z_m, `${path}.z_m`),
  };
}

function parseWirePoints(value: unknown, path: string): WirePoint[] {
  return expectArray(value, path).map((point, index) => parseWirePoint(point, `${path}[${index}]`));
}

function parseAttributes(value: unknown, path: string): Record<string, string> {
  const record = expectRecord(value, path);
  const attributes: Record<string, string> = {};
  for (const [key, raw] of Object.entries(record)) {
    if (typeof raw === 'string') {
      attributes[key] = raw;
    } else if (typeof raw === 'number' || typeof raw === 'boolean') {
      attributes[key] = String(raw);
    } else {
      fail(`${path}.${key}`, `expected string/number/boolean, got ${describeValue(raw)}`);
    }
  }
  return attributes;
}

function parseBounds(value: unknown, path: string): BoundsM {
  const record = expectRecord(value, path);
  const bounds: BoundsM = {
    min_x_m: expectFiniteNumber(record.min_x_m, `${path}.min_x_m`),
    min_y_m: expectFiniteNumber(record.min_y_m, `${path}.min_y_m`),
    max_x_m: expectFiniteNumber(record.max_x_m, `${path}.max_x_m`),
    max_y_m: expectFiniteNumber(record.max_y_m, `${path}.max_y_m`),
  };
  if (bounds.min_x_m > bounds.max_x_m || bounds.min_y_m > bounds.max_y_m) {
    fail(path, 'bounds min exceeds max');
  }
  return bounds;
}

export function parseLaneletGeometry(value: unknown): LaneletGeometry {
  const root = parseEnvelope(value);
  expectSuccess(root);
  return {
    id: expectString(root.id, '$.id', false),
    attributes: parseAttributes(root.attributes ?? {}, '$.attributes'),
    left: parseWirePoints(root.left ?? [], '$.left'),
    right: parseWirePoints(root.right ?? [], '$.right'),
    centerline: parseWirePoints(root.centerline ?? [], '$.centerline'),
  };
}

export function parseAreaGeometry(value: unknown): AreaGeometry {
  const root = parseEnvelope(value);
  expectSuccess(root);
  const innerRaw = root.inner ?? [];
  return {
    id: expectString(root.id, '$.id', false),
    attributes: parseAttributes(root.attributes ?? {}, '$.attributes'),
    outer: parseWirePoints(root.outer ?? [], '$.outer'),
    inner: expectArray(innerRaw, '$.inner').map((ring, index) =>
      parseWirePoints(ring, `$.inner[${index}]`),
    ),
  };
}

export function parseIdList(value: unknown): IdListResult {
  const root = parseEnvelope(value);
  const { status, reason } = parseStatusReason(root);
  // 容忍 `ids` 与 `lanelet_ids` 两种命名。
  const idsRaw = root.ids ?? root.lanelet_ids ?? [];
  return { status, reason, ids: expectStringArray(idsRaw, '$.ids') };
}

export function parseRouteResult(value: unknown): RouteResult {
  const root = parseEnvelope(value);
  const { status, reason } = parseStatusReason(root);
  return {
    status,
    reason,
    lanelet_ids: expectStringArray(root.lanelet_ids ?? [], '$.lanelet_ids'),
    length_m: root.length_m === undefined ? 0 : expectFiniteNumber(root.length_m, '$.length_m'),
  };
}

function parseClippedPath(value: unknown, path: string): ClippedPath {
  const record = expectRecord(value, path);
  return {
    lanelet_id: expectString(record.lanelet_id, `${path}.lanelet_id`),
    role: expectString(record.role, `${path}.role`),
    points: parseWirePoints(record.points ?? [], `${path}.points`),
    start_clip_edge: record.start_clip_edge === undefined ? false : expectBoolean(record.start_clip_edge, `${path}.start_clip_edge`),
    end_clip_edge: record.end_clip_edge === undefined ? false : expectBoolean(record.end_clip_edge, `${path}.end_clip_edge`),
  };
}

export function parseCropResult(value: unknown): CropResult {
  const root = parseEnvelope(value);
  expectSuccess(root);
  const frameRaw = expectRecord(root.window_frame ?? { x_m: 0, y_m: 0, yaw_rad: 0 }, '$.window_frame');
  return {
    window_m: parseBounds(root.window_m, '$.window_m'),
    window_frame: {
      x_m: expectFiniteNumber(frameRaw.x_m, '$.window_frame.x_m'),
      y_m: expectFiniteNumber(frameRaw.y_m, '$.window_frame.y_m'),
      yaw_rad: frameRaw.yaw_rad === undefined ? 0 : expectFiniteNumber(frameRaw.yaw_rad, '$.window_frame.yaw_rad'),
    },
    paths: expectArray(root.paths ?? [], '$.paths').map((path, index) =>
      parseClippedPath(path, `$.paths[${index}]`),
    ),
    points: parseWirePoints(root.points ?? [], '$.points'),
    lanelet_ids: expectStringArray(root.lanelet_ids ?? [], '$.lanelet_ids'),
    diagnostics: expectStringArray(root.diagnostics ?? [], '$.diagnostics'),
  };
}

function parseObjectSummary(value: unknown, path: string): ObjectSummary {
  const record = expectRecord(value, path);
  return {
    id: expectString(record.id, `${path}.id`, false),
    attributes: parseAttributes(record.attributes ?? {}, `${path}.attributes`),
    bounds_m: record.bounds_m === undefined ? null : parseBounds(record.bounds_m, `${path}.bounds_m`),
  };
}

export function parsePointQuery(value: unknown): PointQueryResult {
  const root = parseEnvelope(value);
  expectSuccess(root);
  return {
    x_m: expectFiniteNumber(root.x_m, '$.x_m'),
    y_m: expectFiniteNumber(root.y_m, '$.y_m'),
    lanelets: expectArray(root.lanelets ?? [], '$.lanelets').map((item, index) =>
      parseObjectSummary(item, `$.lanelets[${index}]`),
    ),
    areas: expectArray(root.areas ?? [], '$.areas').map((item, index) =>
      parseObjectSummary(item, `$.areas[${index}]`),
    ),
  };
}

export function parseMapInfo(value: unknown): MapInfoPayload {
  const root = parseEnvelope(value);
  expectSuccess(root);
  const countsRaw = isRecord(root.counts) ? root.counts : {};
  const counts: Record<string, number> = {};
  for (const [key, raw] of Object.entries(countsRaw)) {
    counts[key] = expectFiniteNumber(raw, `$.counts.${key}`);
  }
  return {
    map_id: expectString(root.map_id, '$.map_id', false),
    map_version: expectString(root.map_version, '$.map_version', false),
    counts,
    bounds_m: root.bounds_m === undefined ? null : parseBounds(root.bounds_m, '$.bounds_m'),
  };
}
