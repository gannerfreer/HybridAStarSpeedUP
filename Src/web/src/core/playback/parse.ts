import { normalizeYaw, type Pose2d } from '../pose';
import type { MapArea, MapPoint, MapSnapshot, Lanelet } from '../types';
import type {
  BsplineCurve,
  CollisionCase,
  CorridorSample,
  PathPoint,
  PathSegment,
  PlanResult,
  PlaybackIndex,
  PlaybackIndexSample,
  PlaybackPoint,
  PolynomialCurve,
  PolynomialSegment,
  StaleResult,
  TrajectoryPoint,
} from './types';

/**
 * 回放索引与样例解析（纯函数）。校验：公共包络、必填字段、有限数值、
 * 枚举字符串、时间严格递增（轨迹）；失败抛出 `PlaybackParseError`。
 */

export class PlaybackParseError extends Error {
  readonly path: string;

  constructor(path: string, message: string) {
    super(`${path}: ${message}`);
    this.name = 'PlaybackParseError';
    this.path = path;
  }
}

function fail(path: string, message: string): never {
  throw new PlaybackParseError(path, message);
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

function expectInteger(value: unknown, path: string): number {
  const parsed = expectFinite(value, path);
  if (!Number.isInteger(parsed)) fail(path, 'expected integer');
  return parsed;
}

function parsePose(value: unknown, path: string): Pose2d {
  const record = expectRecord(value, path);
  return {
    x_m: expectFinite(record.x_m, `${path}.x_m`),
    y_m: expectFinite(record.y_m, `${path}.y_m`),
    yaw_rad: normalizeYaw(expectFinite(record.yaw_rad, `${path}.yaw_rad`)),
  };
}

function parsePoint(value: unknown, path: string): PlaybackPoint {
  const record = expectRecord(value, path);
  return { x_m: expectFinite(record.x_m, `${path}.x_m`), y_m: expectFinite(record.y_m, `${path}.y_m`) };
}

function parsePoints(value: unknown, path: string): PlaybackPoint[] {
  return expectArray(value, path).map((point, index) => parsePoint(point, `${path}[${index}]`));
}

function parseEnvelope(value: unknown): Record<string, unknown> {
  const root = expectRecord(value, '$');
  if (root.schema_version !== 1) fail('$.schema_version', 'unsupported schema_version (expected 1)');
  if (typeof root.type !== 'string' || root.type.length === 0) fail('$.type', 'expected non-empty string');
  return root;
}

export function parsePlaybackIndex(value: unknown): PlaybackIndex {
  const root = parseEnvelope(value);
  if (root.type !== 'playback_index') fail('$.type', `expected "playback_index", got ${String(root.type)}`);
  const samples = expectArray(root.samples, '$.samples').map((sample, index) => {
    const record = expectRecord(sample, `$.samples[${index}]`);
    return {
      id: expectString(record.id, `$.samples[${index}].id`, false),
      kind: expectString(record.kind, `$.samples[${index}].kind`, false),
      type: expectString(record.type, `$.samples[${index}].type`, false),
      schema_version: expectInteger(record.schema_version, `$.samples[${index}].schema_version`),
      map_id: expectString(record.map_id, `$.samples[${index}].map_id`, false),
      map_version: expectString(record.map_version, `$.samples[${index}].map_version`, false),
      file: expectString(record.file, `$.samples[${index}].file`, false),
      sha256: expectString(record.sha256, `$.samples[${index}].sha256`, false),
      expected_display: expectString(record.expected_display, `$.samples[${index}].expected_display`),
      synthetic: record.synthetic === true,
    };
  });
  return {
    schema_version: 1,
    type: 'playback_index',
    fixture_version: expectString(root.fixture_version, '$.fixture_version'),
    map_id: expectString(root.map_id, '$.map_id', false),
    map_version: expectString(root.map_version, '$.map_version', false),
    hash_algorithm: expectString(root.hash_algorithm, '$.hash_algorithm'),
    synthetic_note: expectString(root.synthetic_note ?? '', '$.synthetic_note'),
    samples,
  };
}

function parsePathPoint(value: unknown, path: string): PathPoint {
  const record = expectRecord(value, path);
  const direction = expectString(record.direction, `${path}.direction`);
  if (direction !== 'forward' && direction !== 'reverse') {
    fail(`${path}.direction`, `expected forward|reverse, got ${JSON.stringify(direction)}`);
  }
  return {
    pose: parsePose(record.pose, `${path}.pose`),
    s_m: expectFinite(record.s_m, `${path}.s_m`),
    curvature_inv_m: expectFinite(record.curvature_inv_m, `${path}.curvature_inv_m`),
    direction,
  };
}

export function parsePlanResult(value: unknown): PlanResult {
  const root = parseEnvelope(value);
  if (root.type !== 'plan_result') fail('$.type', `expected "plan_result", got ${String(root.type)}`);
  const path: PathSegment[] = expectArray(root.path ?? [], '$.path').map((segment, sIndex) => {
    const record = expectRecord(segment, `$.path[${sIndex}]`);
    return {
      points: expectArray(record.points ?? [], `$.path[${sIndex}].points`).map((point, pIndex) =>
        parsePathPoint(point, `$.path[${sIndex}].points[${pIndex}]`),
      ),
    };
  });
  const trajectory: TrajectoryPoint[] = expectArray(root.trajectory ?? [], '$.trajectory').map((point, index) => {
    const record = expectRecord(point, `$.trajectory[${index}]`);
    return {
      t_s: expectFinite(record.t_s, `$.trajectory[${index}].t_s`),
      pose: parsePose(record.pose, `$.trajectory[${index}].pose`),
      velocity_mps: expectFinite(record.velocity_mps, `$.trajectory[${index}].velocity_mps`),
      acceleration_mps2: expectFinite(record.acceleration_mps2, `$.trajectory[${index}].acceleration_mps2`),
      curvature_inv_m: expectFinite(record.curvature_inv_m, `$.trajectory[${index}].curvature_inv_m`),
    };
  });
  for (let index = 1; index < trajectory.length; index += 1) {
    if (trajectory[index].t_s <= trajectory[index - 1].t_s) {
      fail(`$.trajectory[${index}].t_s`, 'time must be strictly increasing');
    }
  }
  const statistics: Record<string, unknown> = {};
  if (root.statistics !== undefined) {
    const record = expectRecord(root.statistics, '$.statistics');
    for (const [key, entry] of Object.entries(record)) {
      statistics[key] = entry;
    }
  }
  return {
    request_id: expectString(root.request_id ?? '', '$.request_id'),
    status: expectString(root.status, '$.status', false),
    reason: expectString(root.reason ?? '', '$.reason'),
    map_id: expectString(root.map_id, '$.map_id', false),
    map_version: expectString(root.map_version, '$.map_version', false),
    path,
    trajectory,
    statistics,
    synthetic: root.synthetic === true,
    synthetic_note: expectString(root.synthetic_note ?? '', '$.synthetic_note'),
  };
}

export function parseBsplineCurve(value: unknown): BsplineCurve {
  const root = parseEnvelope(value);
  if (root.type !== 'bspline_curve') fail('$.type', `expected "bspline_curve", got ${String(root.type)}`);
  const domain = expectArray(root.parameter_domain, '$.parameter_domain');
  if (domain.length !== 2) fail('$.parameter_domain', 'expected [u_min, u_max]');
  const knots = expectArray(root.knots, '$.knots').map((knot, index) => expectFinite(knot, `$.knots[${index}]`));
  for (let index = 1; index < knots.length; index += 1) {
    if (knots[index] < knots[index - 1]) fail(`$.knots[${index}]`, 'knots must be non-decreasing');
  }
  const degree = expectInteger(root.degree, '$.degree');
  if (degree < 1) fail('$.degree', 'must be >= 1');
  const controlPoints = parsePoints(root.control_points, '$.control_points');
  if (controlPoints.length < degree + 1) fail('$.control_points', 'need at least degree+1 points');
  if (knots.length !== controlPoints.length + degree + 1) {
    fail('$.knots', 'expected control_points + degree + 1 knots');
  }
  return {
    id: expectString(root.id ?? '', '$.id'),
    degree,
    parameter_domain: [expectFinite(domain[0], '$.parameter_domain[0]'), expectFinite(domain[1], '$.parameter_domain[1]')],
    knots,
    control_points: controlPoints,
    map_id: expectString(root.map_id, '$.map_id', false),
    map_version: expectString(root.map_version, '$.map_version', false),
    synthetic: root.synthetic === true,
  };
}

export function parsePolynomialCurve(value: unknown): PolynomialCurve {
  const root = parseEnvelope(value);
  if (root.type !== 'polynomial_curve') fail('$.type', `expected "polynomial_curve", got ${String(root.type)}`);
  const segments: PolynomialSegment[] = expectArray(root.segments, '$.segments').map((segment, index) => {
    const record = expectRecord(segment, `$.segments[${index}]`);
    const coefficients = expectRecord(record.coefficients_ascending, `$.segments[${index}].coefficients_ascending`);
    const x = expectArray(coefficients.x_m, `$.segments[${index}].coefficients_ascending.x_m`).map((c, i) =>
      expectFinite(c, `$.segments[${index}].coefficients_ascending.x_m[${i}]`),
    );
    const y = expectArray(coefficients.y_m, `$.segments[${index}].coefficients_ascending.y_m`).map((c, i) =>
      expectFinite(c, `$.segments[${index}].coefficients_ascending.y_m[${i}]`),
    );
    const duration = expectFinite(record.duration_s, `$.segments[${index}].duration_s`);
    if (duration <= 0) fail(`$.segments[${index}].duration_s`, 'must be > 0');
    return {
      index: expectInteger(record.index, `$.segments[${index}].index`),
      t_start_s: expectFinite(record.t_start_s, `$.segments[${index}].t_start_s`),
      duration_s: duration,
      coefficients_ascending: { x_m: x, y_m: y },
    };
  });
  return {
    id: expectString(root.id ?? '', '$.id'),
    degree: expectInteger(root.degree, '$.degree'),
    segments,
    map_id: expectString(root.map_id, '$.map_id', false),
    map_version: expectString(root.map_version, '$.map_version', false),
    synthetic: root.synthetic === true,
  };
}

export function parseCorridor(value: unknown): CorridorSample {
  const root = parseEnvelope(value);
  if (root.type !== 'corridor') fail('$.type', `expected "corridor", got ${String(root.type)}`);
  const polygons = expectArray(root.polygons, '$.polygons').map((polygon, index) => {
    const record = expectRecord(polygon, `$.polygons[${index}]`);
    const vertices = parsePoints(record.vertices, `$.polygons[${index}].vertices`);
    if (vertices.length < 3) fail(`$.polygons[${index}].vertices`, 'need at least 3 vertices');
    return {
      s_start_m: expectFinite(record.s_start_m, `$.polygons[${index}].s_start_m`),
      s_end_m: expectFinite(record.s_end_m, `$.polygons[${index}].s_end_m`),
      vertices,
    };
  });
  return {
    id: expectString(root.id ?? '', '$.id'),
    polygons,
    map_id: expectString(root.map_id, '$.map_id', false),
    map_version: expectString(root.map_version, '$.map_version', false),
    synthetic: root.synthetic === true,
  };
}

export function parseCollisionCase(value: unknown): CollisionCase {
  const root = parseEnvelope(value);
  if (root.type !== 'collision_case') fail('$.type', `expected "collision_case", got ${String(root.type)}`);
  const obstacleRaw = expectRecord(root.obstacle, '$.obstacle');
  if (obstacleRaw.type !== 'rectangle') fail('$.obstacle.type', 'only rectangle supported');
  const obstacle = {
    id: expectString(obstacleRaw.id ?? '', '$.obstacle.id'),
    type: 'rectangle' as const,
    x_min_m: expectFinite(obstacleRaw.x_min_m, '$.obstacle.x_min_m'),
    y_min_m: expectFinite(obstacleRaw.y_min_m, '$.obstacle.y_min_m'),
    x_max_m: expectFinite(obstacleRaw.x_max_m, '$.obstacle.x_max_m'),
    y_max_m: expectFinite(obstacleRaw.y_max_m, '$.obstacle.y_max_m'),
  };
  let overlap: CollisionCase['overlap_region'] = null;
  if (root.overlap_region !== undefined && root.overlap_region !== null) {
    const record = expectRecord(root.overlap_region, '$.overlap_region');
    overlap = {
      min_x_m: expectFinite(record.x_min_m, '$.overlap_region.x_min_m'),
      min_y_m: expectFinite(record.y_min_m, '$.overlap_region.y_min_m'),
      max_x_m: expectFinite(record.x_max_m, '$.overlap_region.x_max_m'),
      max_y_m: expectFinite(record.y_max_m, '$.overlap_region.y_max_m'),
    };
  }
  return {
    id: expectString(root.id ?? '', '$.id'),
    pose: parsePose(root.pose, '$.pose'),
    footprint_corners: parsePoints(root.footprint_corners, '$.footprint_corners'),
    obstacle,
    verdict: expectString(root.verdict, '$.verdict', false),
    overlap_region: overlap,
    reason: expectString(root.reason ?? '', '$.reason'),
    map_id: expectString(root.map_id, '$.map_id', false),
    map_version: expectString(root.map_version, '$.map_version', false),
    synthetic: root.synthetic === true,
  };
}

export function parseStaleResult(value: unknown): StaleResult {
  const root = parseEnvelope(value);
  if (root.type !== 'stale_result') fail('$.type', `expected "stale_result", got ${String(root.type)}`);
  return {
    id: expectString(root.id ?? '', '$.id'),
    request_id: expectString(root.request_id ?? '', '$.request_id'),
    request_seq: expectInteger(root.request_seq, '$.request_seq'),
    map_version: expectString(root.map_version, '$.map_version', false),
    current_map_version: expectString(root.current_map_version, '$.current_map_version', false),
    current_request_seq: expectInteger(root.current_request_seq, '$.current_request_seq'),
    status: expectString(root.status, '$.status', false),
    reason: expectString(root.reason ?? '', '$.reason'),
  };
}

// ---------------------------------------------------------------------------
// 小型回放地图（playback map_snapshot）→ 内部 MapSnapshot 适配。
// 回放地图字段与 M4 快照不同（bounds / left_boundary / polygon，无地理原点），
// 仅作显示底图，适配后复用 W1 渲染管线。
// ---------------------------------------------------------------------------

function playbackMapPoint(value: unknown, path: string): MapPoint {
  const record = expectRecord(value, path);
  return { id: '', x_m: expectFinite(record.x_m, `${path}.x_m`), y_m: expectFinite(record.y_m, `${path}.y_m`), z_m: 0 };
}

function playbackMapPoints(value: unknown, path: string): MapPoint[] {
  return expectArray(value, path).map((point, index) => playbackMapPoint(point, `${path}[${index}]`));
}

function stringAttributes(value: unknown, path: string): Record<string, string> {
  const record = expectRecord(value ?? {}, path);
  const attributes: Record<string, string> = {};
  for (const [key, entry] of Object.entries(record)) {
    attributes[key] = typeof entry === 'string' ? entry : String(entry);
  }
  return attributes;
}

export function parsePlaybackMap(value: unknown): MapSnapshot {
  const root = parseEnvelope(value);
  if (root.type !== 'map_snapshot') fail('$.type', `expected "map_snapshot", got ${String(root.type)}`);
  const boundsRaw = expectRecord(root.bounds, '$.bounds');
  const bounds = {
    min_x_m: expectFinite(boundsRaw.x_min_m, '$.bounds.x_min_m'),
    min_y_m: expectFinite(boundsRaw.y_min_m, '$.bounds.y_min_m'),
    max_x_m: expectFinite(boundsRaw.x_max_m, '$.bounds.x_max_m'),
    max_y_m: expectFinite(boundsRaw.y_max_m, '$.bounds.y_max_m'),
  };
  const lanelets: Lanelet[] = expectArray(root.lanelets ?? [], '$.lanelets').map((lanelet, index) => {
    const record = expectRecord(lanelet, `$.lanelets[${index}]`);
    return {
      id: expectString(record.id, `$.lanelets[${index}].id`, false),
      attributes: stringAttributes(record.attributes, `$.lanelets[${index}].attributes`),
      left: playbackMapPoints(record.left_boundary ?? [], `$.lanelets[${index}].left_boundary`),
      right: playbackMapPoints(record.right_boundary ?? [], `$.lanelets[${index}].right_boundary`),
      centerline: playbackMapPoints(record.centerline ?? [], `$.lanelets[${index}].centerline`),
    };
  });
  const areas: MapArea[] = expectArray(root.areas ?? [], '$.areas').map((area, index) => {
    const record = expectRecord(area, `$.areas[${index}]`);
    return {
      id: expectString(record.id, `$.areas[${index}].id`, false),
      attributes: stringAttributes(record.attributes, `$.areas[${index}].attributes`),
      outer: playbackMapPoints(record.polygon ?? [], `$.areas[${index}].polygon`),
      inner: [],
    };
  });
  return {
    schema_version: 1,
    type: 'map_snapshot',
    map_id: expectString(root.map_id, '$.map_id', false),
    map_version: expectString(root.map_version, '$.map_version', false),
    source: { path: expectString(root.frame_id ?? '', '$.frame_id'), bytes: 0 },
    coordinate_system: {
      frame_id: 'map',
      unit: 'm',
      x_axis: 'east',
      y_axis: 'north',
      projection: 'none',
      origin: { lat_deg: 0, lon_deg: 0, ele_m: 0 },
    },
    bounds_m: bounds,
    counts: {
      lanelets: lanelets.length,
      areas: areas.length,
      points: 0,
      line_strings: 0,
      polygons: 0,
      regulatory_elements: 0,
    },
    traffic_rules: { location: '', participant: '' },
    lanelets,
    areas,
    regulatory_elements: [],
  };
}
