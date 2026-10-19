import type {
  BoundsM,
  CoordinateSystem,
  Lanelet,
  MapArea,
  MapPoint,
  MapSnapshot,
  RegulatoryElement,
  TrafficRules,
} from './types';

/**
 * 快照解析与校验。拒绝规则（与 `Src/map/examples/data/README.md` §2 一致）：
 *  - `schema_version` 不等于 1、`type` 不是 `map_snapshot`；
 *  - 必填字段缺失或类型错误；对象 ID 不是字符串；
 *  - 任何被消费的数值非有限（NaN/Infinity）。
 * 未知字段被忽略（公共契约允许消费者容忍未知可选字段）。
 */

export class SnapshotParseError extends Error {
  readonly path: string;

  constructor(path: string, message: string) {
    super(`${path}: ${message}`);
    this.name = 'SnapshotParseError';
    this.path = path;
  }
}

function fail(path: string, message: string): never {
  throw new SnapshotParseError(path, message);
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

function requireRecord(value: unknown, path: string): Record<string, unknown> {
  if (!isRecord(value)) fail(path, `expected object, got ${describeValue(value)}`);
  return value;
}

function requireArray(value: unknown, path: string): unknown[] {
  if (!Array.isArray(value)) fail(path, `expected array, got ${describeValue(value)}`);
  return value;
}

function requireString(value: unknown, path: string, allowEmpty = true): string {
  if (typeof value !== 'string') fail(path, `expected string, got ${describeValue(value)}`);
  if (!allowEmpty && value.length === 0) fail(path, 'must not be empty');
  return value;
}

function requireFiniteNumber(value: unknown, path: string): number {
  if (typeof value !== 'number' || !Number.isFinite(value)) {
    fail(path, `expected finite number, got ${describeValue(value)}`);
  }
  return value;
}

function parsePoint(value: unknown, path: string): MapPoint {
  const record = requireRecord(value, path);
  return {
    id: requireString(record.id, `${path}.id`),
    x_m: requireFiniteNumber(record.x_m, `${path}.x_m`),
    y_m: requireFiniteNumber(record.y_m, `${path}.y_m`),
    z_m: requireFiniteNumber(record.z_m, `${path}.z_m`),
  };
}

function parsePointArray(value: unknown, path: string): MapPoint[] {
  return requireArray(value, path).map((point, index) => parsePoint(point, `${path}[${index}]`));
}

function parseAttributes(value: unknown, path: string): Record<string, string> {
  const record = requireRecord(value, path);
  const attributes: Record<string, string> = {};
  for (const [key, attribute] of Object.entries(record)) {
    if (typeof attribute !== 'string') {
      fail(`${path}.${key}`, `expected string, got ${describeValue(attribute)}`);
    }
    attributes[key] = attribute;
  }
  return attributes;
}

function parseLanelet(value: unknown, path: string): Lanelet {
  const record = requireRecord(value, path);
  const lanelet: Lanelet = {
    id: requireString(record.id, `${path}.id`, false),
    attributes: parseAttributes(record.attributes, `${path}.attributes`),
    left: parsePointArray(record.left, `${path}.left`),
    right: parsePointArray(record.right, `${path}.right`),
  };
  if (lanelet.left.length === 0) fail(`${path}.left`, 'must not be empty');
  if (lanelet.right.length === 0) fail(`${path}.right`, 'must not be empty');
  if (record.centerline !== undefined) {
    lanelet.centerline = parsePointArray(record.centerline, `${path}.centerline`);
  }
  return lanelet;
}

function parseArea(value: unknown, path: string): MapArea {
  const record = requireRecord(value, path);
  const area: MapArea = {
    id: requireString(record.id, `${path}.id`, false),
    attributes: parseAttributes(record.attributes, `${path}.attributes`),
    outer: parsePointArray(record.outer, `${path}.outer`),
    inner: [],
  };
  if (area.outer.length === 0) fail(`${path}.outer`, 'must not be empty');
  if (record.inner !== undefined) {
    area.inner = requireArray(record.inner, `${path}.inner`).map((ring, index) =>
      parsePointArray(ring, `${path}.inner[${index}]`),
    );
  }
  return area;
}

function parseRegulatoryElement(value: unknown, path: string): RegulatoryElement {
  const record = requireRecord(value, path);
  const rolesRecord = requireRecord(record.roles, `${path}.roles`);
  const roles: Record<string, string[]> = {};
  for (const [role, refs] of Object.entries(rolesRecord)) {
    roles[role] = requireArray(refs, `${path}.roles.${role}`).map((ref, index) =>
      requireString(ref, `${path}.roles.${role}[${index}]`),
    );
  }
  return {
    id: requireString(record.id, `${path}.id`, false),
    attributes: parseAttributes(record.attributes, `${path}.attributes`),
    roles,
  };
}

function parseBounds(value: unknown): BoundsM {
  const record = requireRecord(value, '$.bounds_m');
  const bounds: BoundsM = {
    min_x_m: requireFiniteNumber(record.min_x_m, '$.bounds_m.min_x_m'),
    min_y_m: requireFiniteNumber(record.min_y_m, '$.bounds_m.min_y_m'),
    max_x_m: requireFiniteNumber(record.max_x_m, '$.bounds_m.max_x_m'),
    max_y_m: requireFiniteNumber(record.max_y_m, '$.bounds_m.max_y_m'),
  };
  if (bounds.min_x_m > bounds.max_x_m) {
    fail('$.bounds_m', `min_x_m (${bounds.min_x_m}) > max_x_m (${bounds.max_x_m})`);
  }
  if (bounds.min_y_m > bounds.max_y_m) {
    fail('$.bounds_m', `min_y_m (${bounds.min_y_m}) > max_y_m (${bounds.max_y_m})`);
  }
  return bounds;
}

function parseCounts(value: unknown): Record<string, number> {
  const record = requireRecord(value, '$.counts');
  const counts: Record<string, number> = {};
  for (const [key, count] of Object.entries(record)) {
    counts[key] = requireFiniteNumber(count, `$.counts.${key}`);
  }
  return counts;
}

function parseCoordinateSystem(value: unknown): CoordinateSystem {
  const record = requireRecord(value, '$.coordinate_system');
  if (record.frame_id !== 'map') {
    fail('$.coordinate_system.frame_id', `expected "map", got ${describeValue(record.frame_id)}`);
  }
  if (record.unit !== 'm') {
    fail('$.coordinate_system.unit', `expected "m", got ${describeValue(record.unit)}`);
  }
  const origin = requireRecord(record.origin, '$.coordinate_system.origin');
  return {
    frame_id: 'map',
    unit: 'm',
    x_axis: requireString(record.x_axis, '$.coordinate_system.x_axis'),
    y_axis: requireString(record.y_axis, '$.coordinate_system.y_axis'),
    projection: requireString(record.projection, '$.coordinate_system.projection'),
    origin: {
      lat_deg: requireFiniteNumber(origin.lat_deg, '$.coordinate_system.origin.lat_deg'),
      lon_deg: requireFiniteNumber(origin.lon_deg, '$.coordinate_system.origin.lon_deg'),
      ele_m: requireFiniteNumber(origin.ele_m, '$.coordinate_system.origin.ele_m'),
    },
  };
}

function parseTrafficRules(value: unknown): TrafficRules {
  const record = requireRecord(value, '$.traffic_rules');
  return {
    location: requireString(record.location, '$.traffic_rules.location'),
    participant: requireString(record.participant, '$.traffic_rules.participant'),
  };
}

/** 解析并校验 M4 公共地图快照；失败抛出 `SnapshotParseError`。 */
export function parseSnapshot(value: unknown): MapSnapshot {
  const root = requireRecord(value, '$');
  if (root.schema_version !== 1) {
    fail('$.schema_version', `unsupported schema_version ${describeValue(root.schema_version)} (expected 1)`);
  }
  if (root.type !== 'map_snapshot') {
    fail('$.type', `expected "map_snapshot", got ${describeValue(root.type)}`);
  }

  const source = requireRecord(root.source, '$.source');
  const lanelets = requireArray(root.lanelets, '$.lanelets').map((lanelet, index) =>
    parseLanelet(lanelet, `$.lanelets[${index}]`),
  );

  let areas: MapArea[] = [];
  if (root.areas !== undefined) {
    areas = requireArray(root.areas, '$.areas').map((area, index) => parseArea(area, `$.areas[${index}]`));
  }

  let regulatoryElements: RegulatoryElement[] = [];
  if (root.regulatory_elements !== undefined) {
    regulatoryElements = requireArray(root.regulatory_elements, '$.regulatory_elements').map(
      (element, index) => parseRegulatoryElement(element, `$.regulatory_elements[${index}]`),
    );
  }

  return {
    schema_version: 1,
    type: 'map_snapshot',
    map_id: requireString(root.map_id, '$.map_id', false),
    map_version: requireString(root.map_version, '$.map_version', false),
    source: {
      path: requireString(source.path, '$.source.path'),
      bytes: requireFiniteNumber(source.bytes, '$.source.bytes'),
    },
    coordinate_system: parseCoordinateSystem(root.coordinate_system),
    bounds_m: parseBounds(root.bounds_m),
    counts: parseCounts(root.counts),
    traffic_rules: parseTrafficRules(root.traffic_rules),
    lanelets,
    areas,
    regulatory_elements: regulatoryElements,
  };
}
