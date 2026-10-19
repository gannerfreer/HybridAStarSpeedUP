import type { BoundsM } from '../types';
import {
  GRID_STATE_FREE,
  GRID_STATE_OCCUPIED,
  GRID_STATE_UNKNOWN,
  type GridLayerData,
  type GridOrigin,
  type GridSource,
} from './types';

/** 栅格文档解析/校验错误（拒绝规则对齐协议 §2.4）。 */
export class GridParseError extends Error {
  constructor(message: string) {
    super(message);
    this.name = 'GridParseError';
  }
}

const BASE64_ALPHABET = 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/';
const BASE64_LOOKUP: number[] = (() => {
  const table = new Array<number>(128).fill(-1);
  for (let index = 0; index < BASE64_ALPHABET.length; index += 1) {
    table[BASE64_ALPHABET.charCodeAt(index)] = index;
  }
  return table;
})();

function fail(path: string, message: string): never {
  throw new GridParseError(`${path}: ${message}`);
}

function isObject(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null && !Array.isArray(value);
}

function requireObject(value: unknown, path: string): Record<string, unknown> {
  if (!isObject(value)) fail(path, '必须是对象');
  return value;
}

function requireString(value: unknown, path: string): string {
  if (typeof value !== 'string') fail(path, '必须是字符串');
  return value;
}

function requireNonEmptyString(value: unknown, path: string): string {
  const text = requireString(value, path);
  if (text.length === 0) fail(path, '不能为空');
  return text;
}

function requireFiniteNumber(value: unknown, path: string): number {
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    fail(path, '必须是有限数值（拒绝 NaN/Infinity）');
  }
  return value;
}

function requireInteger(value: unknown, path: string): number {
  const number = requireFiniteNumber(value, path);
  if (!Number.isInteger(number)) fail(path, '必须是整数');
  return number;
}

function requireBoolean(value: unknown, path: string): boolean {
  if (typeof value !== 'boolean') fail(path, '必须是布尔值');
  return value;
}

/** RFC 4648 §4 标准 base64（无空白、规范填充）；返回字节数组。 */
export function decodeBase64U8(text: string): Uint8Array {
  if (typeof text !== 'string' || text.length === 0) {
    fail('cells', '必须是规范 base64 字符串');
  }
  if (text.length % 4 !== 0) fail('cells', 'base64 长度不是 4 的倍数');
  let padding = 0;
  if (text.endsWith('==')) padding = 2;
  else if (text.endsWith('=')) padding = 1;
  const core = padding > 0 ? text.slice(0, text.length - padding) : text;
  if (core.includes('=')) fail('cells', 'base64 填充只允许出现在末尾');
  for (let index = 0; index < core.length; index += 1) {
    const code = core.charCodeAt(index);
    if (code >= 128 || BASE64_LOOKUP[code] < 0) {
      fail('cells', `base64 含非法字符: ${JSON.stringify(text[index])}`);
    }
  }
  const output = new Uint8Array((core.length * 6) >> 3);
  let accumulator = 0;
  let bits = 0;
  let written = 0;
  for (let index = 0; index < core.length; index += 1) {
    accumulator = (accumulator << 6) | BASE64_LOOKUP[core.charCodeAt(index)];
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      output[written] = (accumulator >> bits) & 0xff;
      written += 1;
    }
  }
  // 规范填充：剩余位必须为零（等价于重新编码一致）。
  if (bits > 0 && (accumulator & ((1 << bits) - 1)) !== 0) {
    fail('cells', 'base64 不是规范编码（填充位非零）');
  }
  if (written !== output.length) fail('cells', 'base64 解码长度异常');
  return output;
}

function requireBounds(value: unknown, path: string): BoundsM {
  const object = requireObject(value, path);
  const minX = requireFiniteNumber(object.min_x_m, `${path}.min_x_m`);
  const minY = requireFiniteNumber(object.min_y_m, `${path}.min_y_m`);
  const maxX = requireFiniteNumber(object.max_x_m, `${path}.max_x_m`);
  const maxY = requireFiniteNumber(object.max_y_m, `${path}.max_y_m`);
  if (!(minX < maxX)) fail(path, '要求 min_x_m < max_x_m');
  if (!(minY < maxY)) fail(path, '要求 min_y_m < max_y_m');
  return { min_x_m: minX, min_y_m: minY, max_x_m: maxX, max_y_m: maxY };
}

function requireOrigin(value: unknown, path: string): GridOrigin {
  const object = requireObject(value, path);
  const x = requireFiniteNumber(object.x_m, `${path}.x_m`);
  const y = requireFiniteNumber(object.y_m, `${path}.y_m`);
  const yaw = requireFiniteNumber(object.yaw_rad, `${path}.yaw_rad`);
  if (yaw < -Math.PI || yaw >= Math.PI) {
    fail(path, 'yaw_rad 必须归一化到 [-π, π)');
  }
  return { x_m: x, y_m: y, yaw_rad: yaw };
}

function requireOptionalInteger(object: Record<string, unknown>, key: string, path: string): number | null {
  if (!(key in object) || object[key] === undefined || object[key] === null) return null;
  const value = requireInteger(object[key], `${path}.${key}`);
  if (value < 0) fail(`${path}.${key}`, '必须 >= 0');
  return value;
}

function requireOptionalString(object: Record<string, unknown>, key: string, path: string): string | null {
  if (!(key in object) || object[key] === undefined || object[key] === null) return null;
  return requireString(object[key], `${path}.${key}`);
}

const GRID_SOURCES: readonly GridSource[] = ['driveable', 'planning_scene', 'fixture'];

function decodeIntegerCells(value: unknown, width: number, height: number): Uint8Array {
  if (!Array.isArray(value)) fail('cells', '必须是整数数组（grid_view）');
  if (value.length !== width * height) {
    fail('cells', `数组长度 ${value.length} != width×height ${width * height}`);
  }
  const cells = new Uint8Array(value.length);
  for (let index = 0; index < value.length; index += 1) {
    const state = value[index];
    if (state !== GRID_STATE_FREE && state !== GRID_STATE_OCCUPIED && state !== GRID_STATE_UNKNOWN) {
      fail('cells', `单元 ${index} 取值非法: ${String(state)}（允许 0/1/2）`);
    }
    cells[index] = state;
  }
  return cells;
}

function decodeBase64Cells(value: unknown, width: number, height: number): Uint8Array {
  if (typeof value !== 'string') fail('cells', '必须是 base64 字符串（grid_snapshot）');
  const raw = decodeBase64U8(value);
  if (raw.length !== width * height) {
    fail('cells', `解码长度 ${raw.length} != width×height ${width * height}`);
  }
  for (let index = 0; index < raw.length; index += 1) {
    if (raw[index] !== 0 && raw[index] !== 1 && raw[index] !== 2) {
      fail('cells', `单元 ${index} 取值非法: ${raw[index]}（允许 0/1/2）`);
    }
  }
  return raw;
}

/**
 * 解析 `grid_view` / `grid_snapshot` 文档（协议 v1 + v5 顶层约定）。
 * 拒绝：包络/类型、必填字段、非有限数值、空窗口、非法分辨率、长度/取值、
 * base64 规范性与 `scene_id` 格式（planning_scene 时）。
 */
export function parseGridDocument(value: unknown): GridLayerData {
  const root = requireObject(value, '$');
  const schemaVersion = requireInteger(root.schema_version, '$.schema_version');
  if (schemaVersion !== 1) fail('$.schema_version', `未知主版本 ${schemaVersion}（仅支持 1）`);
  const type = requireString(root.type, '$.type');
  if (type !== 'grid_view' && type !== 'grid_snapshot') {
    fail('$.type', `未知文档类型 ${JSON.stringify(type)}`);
  }
  const mapId = requireNonEmptyString(root.map_id, '$.map_id');
  const mapVersion = requireNonEmptyString(root.map_version, '$.map_version');
  const width = requireInteger(root.width, '$.width');
  const height = requireInteger(root.height, '$.height');
  if (width < 1) fail('$.width', '必须 >= 1');
  if (height < 1) fail('$.height', '必须 >= 1');
  const resolution = requireFiniteNumber(root.resolution_m, '$.resolution_m');
  if (!(resolution > 0)) fail('$.resolution_m', '必须 > 0');
  const origin = requireOrigin(root.origin, '$.origin');

  if (type === 'grid_view') {
    const cells = decodeIntegerCells(root.cells, width, height);
    return {
      kind: 'grid_view',
      source: 'grid_view',
      map_id: mapId,
      map_version: mapVersion,
      width,
      height,
      resolution_m: resolution,
      origin,
      cells,
      encoding: 'integer_array',
      window: null,
      requested_resolution_m: null,
      effective_resolution_m: null,
      degraded: false,
      scene_id: null,
      scene_version: null,
      request_seq: null,
      budget_cells: null,
      notes: requireOptionalString(root, 'notes', '$'),
    };
  }

  const source = requireString(root.source, '$.source');
  if (!GRID_SOURCES.includes(source as GridSource)) {
    fail('$.source', `未知来源 ${JSON.stringify(source)}（闭集 driveable/planning_scene/fixture）`);
  }
  const window = requireBounds(root.window, '$.window');
  const requested = requireFiniteNumber(root.requested_resolution_m, '$.requested_resolution_m');
  if (!(requested > 0)) fail('$.requested_resolution_m', '必须 > 0');
  const effective = requireFiniteNumber(root.effective_resolution_m, '$.effective_resolution_m');
  if (!(effective > 0)) fail('$.effective_resolution_m', '必须 > 0');
  if (effective !== resolution) {
    fail('$.effective_resolution_m', '必须等于 resolution_m（协议 §2.2）');
  }
  const degraded = requireBoolean(root.degraded, '$.degraded');
  if (degraded !== (effective > requested)) {
    fail('$.degraded', '必须等于 (effective_resolution_m > requested_resolution_m)');
  }
  const encoding = requireString(root.encoding, '$.encoding');
  if (encoding !== 'base64_u8') fail('$.encoding', `未知编码 ${JSON.stringify(encoding)}（仅 base64_u8）`);
  const cells = decodeBase64Cells(root.cells, width, height);
  const sceneId = requireOptionalString(root, 'scene_id', '$');
  const sceneVersion = requireOptionalString(root, 'scene_version', '$');
  if (source === 'planning_scene') {
    if (!sceneId || !/^[0-9a-f]{64}$/.test(sceneId)) {
      fail('$.scene_id', 'planning_scene 必须携带 64 位小写十六进制 scene_id');
    }
    if (sceneVersion !== 'scene_v1') {
      fail('$.scene_version', 'planning_scene 的 scene_version 恒为 scene_v1');
    }
  } else if (sceneId !== null && !/^[0-9a-f]{64}$/.test(sceneId)) {
    fail('$.scene_id', '必须为 64 位小写十六进制');
  }
  return {
    kind: 'grid_snapshot',
    source: source as GridSource,
    map_id: mapId,
    map_version: mapVersion,
    width,
    height,
    resolution_m: resolution,
    origin,
    cells,
    encoding: 'base64_u8',
    window,
    requested_resolution_m: requested,
    effective_resolution_m: effective,
    degraded,
    scene_id: sceneId,
    scene_version: sceneVersion,
    request_seq: requireOptionalInteger(root, 'request_seq', '$'),
    budget_cells: requireOptionalInteger(root, 'budget_cells', '$'),
    notes: requireOptionalString(root, 'notes', '$'),
  };
}

/** 图例说明（固定顺序：空闲/占据/未知）。 */
export const GRID_LEGEND_ORDER = [GRID_STATE_FREE, GRID_STATE_OCCUPIED, GRID_STATE_UNKNOWN] as const;
