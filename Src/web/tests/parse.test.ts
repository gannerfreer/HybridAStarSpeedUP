import { describe, expect, it } from 'vitest';
import { findRepoRoot, moduleDir, readSnapshotJson, resolveSnapshotSource } from '../src/core/fixture';
import { parseSnapshot, SnapshotParseError } from '../src/core/parse';
import { computeSnapshotStats } from '../src/core/stats';

const repoRoot = findRepoRoot(moduleDir(import.meta.url));
const source = resolveSnapshotSource(repoRoot);

function point(id: string, x: number, y: number): Record<string, unknown> {
  return { id, x_m: x, y_m: y, z_m: 0 };
}

function validSnapshot(): Record<string, unknown> {
  return {
    schema_version: 1,
    type: 'map_snapshot',
    map_id: 'unit_test',
    map_version: 'a'.repeat(64),
    source: { path: 'unit_test.osm', bytes: 1234 },
    coordinate_system: {
      frame_id: 'map',
      unit: 'm',
      x_axis: 'east',
      y_axis: 'north',
      projection: 'utm',
      origin: { lat_deg: 49.0, lon_deg: 8.0, ele_m: 0.0 },
    },
    bounds_m: { min_x_m: -10, min_y_m: -5, max_x_m: 10, max_y_m: 5 },
    counts: { lanelets: 1, areas: 1, points: 4, line_strings: 2, polygons: 0, regulatory_elements: 0 },
    traffic_rules: { location: 'de', participant: 'vehicle' },
    lanelets: [
      {
        id: '1',
        attributes: { type: 'lanelet' },
        left: [point('11', 0, 5), point('12', 10, 5)],
        right: [point('13', 0, -5), point('14', 10, -5)],
        centerline: [point('15', 0, 0), point('16', 10, 0)],
      },
    ],
    areas: [
      {
        id: '20',
        attributes: {},
        outer: [point('21', -10, -5), point('22', -5, -5), point('23', -5, 0), point('24', -10, 0)],
        inner: [],
      },
    ],
  };
}

function withOverride(mutate: (snapshot: Record<string, unknown>) => void): Record<string, unknown> {
  const snapshot = validSnapshot();
  mutate(snapshot);
  return snapshot;
}

function expectParseError(value: unknown, pattern: RegExp | string, path?: string): SnapshotParseError {
  try {
    parseSnapshot(value);
  } catch (error) {
    expect(error).toBeInstanceOf(SnapshotParseError);
    const parseError = error as SnapshotParseError;
    expect(parseError.message).toMatch(pattern);
    if (path !== undefined) expect(parseError.path).toBe(path);
    return parseError;
  }
  throw new Error('expected parseSnapshot to throw');
}

describe('parseSnapshot（合成夹具）', () => {
  it('接受最小合法快照并归一化可选数组', () => {
    const snapshot = parseSnapshot(validSnapshot());
    expect(snapshot.schema_version).toBe(1);
    expect(snapshot.type).toBe('map_snapshot');
    expect(snapshot.lanelets).toHaveLength(1);
    expect(snapshot.areas).toHaveLength(1);
    expect(snapshot.regulatory_elements).toEqual([]);
    expect(snapshot.lanelets[0].centerline).toHaveLength(2);
    expect(snapshot.coordinate_system.frame_id).toBe('map');
    expect(snapshot.coordinate_system.unit).toBe('m');
  });

  it('接受缺少 centerline / areas 的快照', () => {
    const raw = validSnapshot();
    const lanelets = raw.lanelets as Array<Record<string, unknown>>;
    delete lanelets[0].centerline;
    delete raw.areas;
    const snapshot = parseSnapshot(raw);
    expect(snapshot.lanelets[0].centerline).toBeUndefined();
    expect(snapshot.areas).toEqual([]);
  });

  it('容忍未知字段', () => {
    const raw = validSnapshot();
    raw.future_field = { anything: true };
    (raw.lanelets as Array<Record<string, unknown>>)[0].extra = 'x';
    const snapshot = parseSnapshot(raw);
    expect(snapshot.map_id).toBe('unit_test');
  });

  it('拒绝不支持的 schema_version', () => {
    expectParseError(
      withOverride((snapshot) => {
        snapshot.schema_version = 2;
      }),
      /schema_version/,
      '$.schema_version',
    );
  });

  it('拒绝错误的 type', () => {
    expectParseError(
      withOverride((snapshot) => {
        snapshot.type = 'crop_result';
      }),
      /map_snapshot/,
      '$.type',
    );
  });

  it('拒绝缺失 lanelets', () => {
    expectParseError(
      withOverride((snapshot) => {
        delete snapshot.lanelets;
      }),
      /expected array/,
      '$.lanelets',
    );
  });

  it('拒绝缺失 right 边界', () => {
    expectParseError(
      withOverride((snapshot) => {
        delete (snapshot.lanelets as Array<Record<string, unknown>>)[0].right;
      }),
      /expected array/,
      '$.lanelets[0].right',
    );
  });

  it('拒绝非字符串对象 ID', () => {
    expectParseError(
      withOverride((snapshot) => {
        (snapshot.lanelets as Array<Record<string, unknown>>)[0].id = 42440;
      }),
      /expected string/,
      '$.lanelets[0].id',
    );
  });

  it('拒绝非有限坐标并给出 JSON 路径', () => {
    expectParseError(
      withOverride((snapshot) => {
        (snapshot.lanelets as Array<Record<string, unknown>>)[0].left = [
          { id: 'x', x_m: Number.POSITIVE_INFINITY, y_m: 0, z_m: 0 },
        ];
      }),
      /finite number/,
      '$.lanelets[0].left[0].x_m',
    );
  });

  it('拒绝非有限 bounds 与倒置 bounds', () => {
    expectParseError(
      withOverride((snapshot) => {
        (snapshot.bounds_m as Record<string, unknown>).max_x_m = Number.NaN;
      }),
      /finite number/,
      '$.bounds_m.max_x_m',
    );
    expectParseError(
      withOverride((snapshot) => {
        (snapshot.bounds_m as Record<string, unknown>).min_x_m = 100;
      }),
      /min_x_m/,
      '$.bounds_m',
    );
  });

  it('拒绝空 map_id 与非有限 counts', () => {
    expectParseError(
      withOverride((snapshot) => {
        snapshot.map_id = '';
      }),
      /must not be empty/,
      '$.map_id',
    );
    expectParseError(
      withOverride((snapshot) => {
        (snapshot.counts as Record<string, unknown>).lanelets = Number.POSITIVE_INFINITY;
      }),
      /finite number/,
      '$.counts.lanelets',
    );
  });

  it('拒绝错误的坐标系单位', () => {
    expectParseError(
      withOverride((snapshot) => {
        (snapshot.coordinate_system as Record<string, unknown>).unit = 'ft';
      }),
      /expected "m"/,
      '$.coordinate_system.unit',
    );
  });
});

describe('parseSnapshot（仓库快照：v3 夹具或 M4 样例回退）', () => {
  const { value } = readSnapshotJson(repoRoot, source);
  const snapshot = parseSnapshot(value);
  const stats = computeSnapshotStats(snapshot);

  it(`消费 ${source.rel_path}（kind=${source.kind}）`, () => {
    expect(snapshot.map_id).toBe('mapping_example');
    expect(snapshot.schema_version).toBe(1);
    expect(snapshot.lanelets.length).toBeGreaterThan(0);
    expect(snapshot.areas.length).toBeGreaterThan(0);
    expect(source.bytes).toBeGreaterThan(0);
    expect(source.sha256).toMatch(/^[0-9a-f]{64}$/);
  });

  it('lanelet / area 对象 ID 均为非空字符串', () => {
    for (const lanelet of snapshot.lanelets) {
      expect(typeof lanelet.id).toBe('string');
      expect(lanelet.id.length).toBeGreaterThan(0);
    }
    for (const area of snapshot.areas) {
      expect(typeof area.id).toBe('string');
      expect(area.id.length).toBeGreaterThan(0);
    }
  });

  it('所有被消费数值有限', () => {
    const points = snapshot.lanelets.flatMap((lanelet) => [
      ...lanelet.left,
      ...lanelet.right,
      ...(lanelet.centerline ?? []),
    ]);
    expect(points.length).toBeGreaterThan(0);
    for (const p of points) {
      expect(Number.isFinite(p.x_m)).toBe(true);
      expect(Number.isFinite(p.y_m)).toBe(true);
      expect(Number.isFinite(p.z_m)).toBe(true);
    }
    for (const area of snapshot.areas) {
      for (const ring of [area.outer, ...area.inner]) {
        for (const p of ring) {
          expect(Number.isFinite(p.x_m) && Number.isFinite(p.y_m)).toBe(true);
        }
      }
    }
  });

  it('几何计数与顶层 counts 一致', () => {
    expect(stats.lanelets).toBe(snapshot.counts.lanelets);
    expect(stats.areas).toBe(snapshot.counts.areas);
    expect(stats.regulatory_elements).toBe(snapshot.counts.regulatory_elements);
  });

  it('bounds_m 覆盖全部几何点', () => {
    const tolerance = 1e-6;
    for (const lanelet of snapshot.lanelets) {
      for (const p of [...lanelet.left, ...lanelet.right, ...(lanelet.centerline ?? [])]) {
        expect(p.x_m).toBeGreaterThanOrEqual(snapshot.bounds_m.min_x_m - tolerance);
        expect(p.x_m).toBeLessThanOrEqual(snapshot.bounds_m.max_x_m + tolerance);
        expect(p.y_m).toBeGreaterThanOrEqual(snapshot.bounds_m.min_y_m - tolerance);
        expect(p.y_m).toBeLessThanOrEqual(snapshot.bounds_m.max_y_m + tolerance);
      }
    }
    for (const area of snapshot.areas) {
      for (const ring of [area.outer, ...area.inner]) {
        for (const p of ring) {
          expect(p.x_m).toBeGreaterThanOrEqual(snapshot.bounds_m.min_x_m - tolerance);
          expect(p.x_m).toBeLessThanOrEqual(snapshot.bounds_m.max_x_m + tolerance);
          expect(p.y_m).toBeGreaterThanOrEqual(snapshot.bounds_m.min_y_m - tolerance);
          expect(p.y_m).toBeLessThanOrEqual(snapshot.bounds_m.max_y_m + tolerance);
        }
      }
    }
  });
});
