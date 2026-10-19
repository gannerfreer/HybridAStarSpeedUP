import { describe, expect, it } from 'vitest';
import { decodeBase64U8, GridParseError, parseGridDocument } from '../src/core/grid/parse';

function gridView(overrides: Record<string, unknown> = {}): Record<string, unknown> {
  return {
    schema_version: 1,
    type: 'grid_view',
    map_id: 'grid_test',
    map_version: 'v6',
    width: 2,
    height: 2,
    resolution_m: 0.5,
    origin: { x_m: 0, y_m: 0, yaw_rad: 0 },
    cells: [0, 1, 2, 0],
    ...overrides,
  };
}

function snapshot(overrides: Record<string, unknown> = {}): Record<string, unknown> {
  return {
    schema_version: 1,
    type: 'grid_snapshot',
    source: 'fixture',
    map_id: 'grid_snapshot_small',
    map_version: 'v6',
    window: { min_x_m: 0, min_y_m: 0, max_x_m: 0.3, max_y_m: 0.2 },
    requested_resolution_m: 0.1,
    effective_resolution_m: 0.1,
    resolution_m: 0.1,
    degraded: false,
    width: 3,
    height: 2,
    origin: { x_m: 0, y_m: 0, yaw_rad: 0 },
    encoding: 'base64_u8',
    cells: 'AAEAAAAC',
    ...overrides,
  };
}

describe('栅格文档解析（协议 v1）', () => {
  it('解析 grid_view 整数数组', () => {
    const layer = parseGridDocument(gridView());
    expect(layer.kind).toBe('grid_view');
    expect(layer.source).toBe('grid_view');
    expect(layer.encoding).toBe('integer_array');
    expect(Array.from(layer.cells)).toEqual([0, 1, 2, 0]);
    expect(layer.window).toBeNull();
  });

  it('解析 grid_snapshot base64_u8（snapshot_small 向量）', () => {
    const layer = parseGridDocument(snapshot());
    expect(layer.kind).toBe('grid_snapshot');
    expect(layer.source).toBe('fixture');
    expect(layer.encoding).toBe('base64_u8');
    expect(Array.from(layer.cells)).toEqual([0, 1, 0, 0, 0, 2]);
    expect(layer.window).toEqual({ min_x_m: 0, min_y_m: 0, max_x_m: 0.3, max_y_m: 0.2 });
    expect(layer.degraded).toBe(false);
  });

  it('解析降级快照（degraded=true 且 effective>requested）', () => {
    const layer = parseGridDocument(
      snapshot({
        requested_resolution_m: 0.1,
        effective_resolution_m: 0.4,
        resolution_m: 0.4,
        degraded: true,
        width: 2,
        height: 2,
        cells: 'AAAAAA==',
      }),
    );
    expect(layer.degraded).toBe(true);
    expect(layer.effective_resolution_m).toBe(0.4);
  });

  it('planning_scene 需要 64 位小写十六进制 scene_id', () => {
    const sceneId = 'a'.repeat(64);
    const layer = parseGridDocument(
      snapshot({ source: 'planning_scene', scene_id: sceneId, scene_version: 'scene_v1' }),
    );
    expect(layer.scene_id).toBe(sceneId);
    expect(() => parseGridDocument(snapshot({ source: 'planning_scene' }))).toThrow(GridParseError);
    expect(() =>
      parseGridDocument(
        snapshot({ source: 'planning_scene', scene_id: 'A'.repeat(64), scene_version: 'scene_v1' }),
      ),
    ).toThrow(GridParseError);
  });

  it('拒绝未知 schema/type/source/encoding', () => {
    expect(() => parseGridDocument(gridView({ schema_version: 2 }))).toThrow(/未知主版本/);
    expect(() => parseGridDocument(gridView({ type: 'grid_other' }))).toThrow(/未知文档类型/);
    expect(() => parseGridDocument(snapshot({ source: 'sensor' }))).toThrow(/未知来源/);
    expect(() => parseGridDocument(snapshot({ encoding: 'gzip' }))).toThrow(/未知编码/);
  });

  it('拒绝非有限数值与非法窗口/分辨率', () => {
    expect(() => parseGridDocument(gridView({ resolution_m: Number.POSITIVE_INFINITY }))).toThrow(
      /有限数值/,
    );
    expect(() => parseGridDocument(gridView({ resolution_m: 0 }))).toThrow(/> 0/);
    expect(() => parseGridDocument(gridView({ width: 0 }))).toThrow(/>= 1/);
    expect(() =>
      parseGridDocument(snapshot({ window: { min_x_m: 0, min_y_m: 0, max_x_m: 0, max_y_m: 1 } })),
    ).toThrow(/min_x_m < max_x_m/);
    expect(() =>
      parseGridDocument(snapshot({ window: { min_x_m: 0, min_y_m: 1, max_x_m: 1, max_y_m: 0 } })),
    ).toThrow(/min_y_m < max_y_m/);
  });

  it('拒绝 degraded 与分辨率不一致', () => {
    expect(() =>
      parseGridDocument(snapshot({ requested_resolution_m: 0.1, effective_resolution_m: 0.2, degraded: false })),
    ).toThrow(/必须等于/);
    expect(() =>
      parseGridDocument(
        snapshot({
          requested_resolution_m: 0.1,
          effective_resolution_m: 0.4,
          resolution_m: 0.4,
          degraded: true,
          cells: 'AAAAAA==',
          width: 2,
          height: 2,
        }),
      ),
    ).not.toThrow();
  });

  it('拒绝 yaw 未归一化', () => {
    expect(() => parseGridDocument(gridView({ origin: { x_m: 0, y_m: 0, yaw_rad: Math.PI } }))).toThrow(
      /归一化/,
    );
    expect(() =>
      parseGridDocument(gridView({ origin: { x_m: 0, y_m: 0, yaw_rad: -Math.PI } })),
    ).not.toThrow();
  });

  it('拒绝 cells 长度/取值/编码错误', () => {
    expect(() => parseGridDocument(gridView({ cells: [0, 1, 2] }))).toThrow(/长度/);
    expect(() => parseGridDocument(gridView({ cells: [0, 1, 3, 0] }))).toThrow(/取值非法/);
    expect(() => parseGridDocument(snapshot({ cells: 'AAEAAA' }))).toThrow(/4 的倍数/);
    expect(() => parseGridDocument(snapshot({ cells: 'AAE!AAAA' }))).toThrow(/非法字符/);
    expect(() => parseGridDocument(snapshot({ cells: 'AAE=' }))).toThrow(GridParseError);
    expect(() => parseGridDocument(snapshot({ cells: 'AAEAAAAD' }))).toThrow(/取值非法/);
    expect(() => parseGridDocument(snapshot({ cells: 'AAEAAAACAA' }))).toThrow(/长度/);
  });

  it('拒绝非规范 base64 填充位', () => {
    expect(() => decodeBase64U8('AB==')).toThrow(/规范/);
    expect(() => decodeBase64U8('AA==')).not.toThrow();
    expect(Array.from(decodeBase64U8('AA=='))).toEqual([0]);
  });

  it('缺失必填字段抛出可读错误', () => {
    expect(() => parseGridDocument({ schema_version: 1, type: 'grid_view' })).toThrow(/map_id/);
    expect(() => parseGridDocument(snapshot({ window: undefined }))).toThrow(/window/);
    expect(() => parseGridDocument(null)).toThrow(/必须是对象/);
  });
});
