import { describe, expect, it } from 'vitest';
import {
  ServicePayloadError,
  parseAreaGeometry,
  parseCropResult,
  parseEnvelope,
  parseIdList,
  parseLaneletGeometry,
  parseMapInfo,
  parsePointQuery,
  parseRouteResult,
} from '../src/core/service/parse';

const point = (id: string, x: number, y: number) => ({ id, x_m: x, y_m: y, z_m: 0 });

describe('服务响应解析（公共 JSON 包络）', () => {
  it('校验包络：schema_version 必须为 1、type 必须为非空字符串', () => {
    expect(parseEnvelope({ schema_version: 1, type: 'lanelet_geometry' })).toBeTruthy();
    expect(() => parseEnvelope({ schema_version: 2, type: 'x' })).toThrowError(/schema_version/);
    expect(() => parseEnvelope({ schema_version: 1 })).toThrowError(/type/);
    expect(() => parseEnvelope(null)).toThrowError(/expected object/);
  });

  it('解析 lanelet 几何（属性、左右边界、中心线）', () => {
    const value = {
      schema_version: 1,
      type: 'lanelet_geometry',
      id: '38992',
      attributes: { type: 'lanelet', one_way: 'yes', speed: 50 },
      left: [point('a', 0, 2), point('b', 10, 2)],
      right: [point('c', 0, -2), point('d', 10, -2)],
      centerline: [point('e', 0, 0), point('f', 10, 0)],
    };
    const parsed = parseLaneletGeometry(value);
    expect(parsed.id).toBe('38992');
    expect(parsed.left).toHaveLength(2);
    expect(parsed.right).toHaveLength(2);
    expect(parsed.centerline).toHaveLength(2);
    // 数值属性转字符串展示
    expect(parsed.attributes.speed).toBe('50');
    expect(parsed.attributes.one_way).toBe('yes');
  });

  it('lanelet 缺省字段容忍：无 centerline / attributes 时归一化为空', () => {
    const parsed = parseLaneletGeometry({
      schema_version: 1,
      type: 'lanelet_geometry',
      id: '1',
      left: [],
      right: [],
    });
    expect(parsed.centerline).toEqual([]);
    expect(parsed.attributes).toEqual({});
  });

  it('查询失败（status !== success）抛出带 reason 的载荷错误', () => {
    expect(() =>
      parseLaneletGeometry({ schema_version: 1, type: 'lanelet_geometry', status: 'invalid_input', reason: 'unknown id' }),
    ).toThrowError(/invalid_input: unknown id/);
  });

  it('拒绝非有限坐标与非字符串 ID', () => {
    expect(() =>
      parseLaneletGeometry({
        schema_version: 1,
        type: 'lanelet_geometry',
        id: '1',
        left: [{ id: 'a', x_m: Number.POSITIVE_INFINITY, y_m: 0, z_m: 0 }],
        right: [],
      }),
    ).toThrowError(/finite number/);
    expect(() =>
      parseLaneletGeometry({
        schema_version: 1,
        type: 'lanelet_geometry',
        id: 38992,
        left: [],
        right: [],
      }),
    ).toThrowError(/expected string/);
  });

  it('拒绝属性中的嵌套对象', () => {
    expect(() =>
      parseLaneletGeometry({
        schema_version: 1,
        type: 'lanelet_geometry',
        id: '1',
        left: [],
        right: [],
        attributes: { nested: { a: 1 } },
      }),
    ).toThrowError(ServicePayloadError);
  });

  it('解析区域几何（外环 + 内环）', () => {
    const parsed = parseAreaGeometry({
      schema_version: 1,
      type: 'area_geometry',
      id: '45034',
      attributes: { subtype: 'keepout' },
      outer: [point('a', 0, 0), point('b', 4, 0), point('c', 4, 4), point('d', 0, 4)],
      inner: [[point('e', 1, 1), point('f', 2, 1), point('g', 2, 2), point('h', 1, 2)]],
    });
    expect(parsed.id).toBe('45034');
    expect(parsed.outer).toHaveLength(4);
    expect(parsed.inner).toHaveLength(1);
  });

  it('解析拓扑 ID 列表（容忍 ids / lanelet_ids 命名）', () => {
    const a = parseIdList({ schema_version: 1, type: 'id_list', status: 'success', ids: ['1', '2'] });
    expect(a.ids).toEqual(['1', '2']);
    const b = parseIdList({ schema_version: 1, type: 'topology_result', lanelet_ids: ['3'] });
    expect(b.ids).toEqual(['3']);
    expect(b.status).toBe('success');
  });

  it('解析路由结果（含失败状态保留）', () => {
    const ok = parseRouteResult({
      schema_version: 1,
      type: 'route_result',
      status: 'success',
      lanelet_ids: ['1', '2', '3'],
      length_m: 123.45,
    });
    expect(ok.length_m).toBeCloseTo(123.45, 9);
    expect(ok.lanelet_ids).toHaveLength(3);
    const failed = parseRouteResult({ schema_version: 1, type: 'route_result', status: 'no_path', reason: 'unreachable' });
    expect(failed.status).toBe('no_path');
    expect(failed.lanelet_ids).toEqual([]);
  });

  it('解析窗口裁剪结果（window_frame 缺省为原点）', () => {
    const parsed = parseCropResult({
      schema_version: 1,
      type: 'crop_result',
      window_m: { min_x_m: 0, min_y_m: 0, max_x_m: 10, max_y_m: 10 },
      paths: [
        {
          lanelet_id: '1',
          role: 'centerline',
          points: [point('a', 1, 1), point('', 5, 5)],
          start_clip_edge: true,
          end_clip_edge: false,
        },
      ],
      points: [point('a', 1, 1)],
      lanelet_ids: ['1'],
      diagnostics: ['centerline approximated'],
    });
    expect(parsed.window_frame).toEqual({ x_m: 0, y_m: 0, yaw_rad: 0 });
    expect(parsed.paths[0].start_clip_edge).toBe(true);
    expect(parsed.paths[0].points[1].id).toBe('');
    expect(parsed.diagnostics).toEqual(['centerline approximated']);
  });

  it('窗口 bounds 倒置时报错', () => {
    expect(() =>
      parseCropResult({
        schema_version: 1,
        type: 'crop_result',
        window_m: { min_x_m: 10, min_y_m: 0, max_x_m: 0, max_y_m: 10 },
      }),
    ).toThrowError(/bounds/);
  });

  it('解析点查询（对象摘要：id/属性/包围盒）', () => {
    const parsed = parsePointQuery({
      schema_version: 1,
      type: 'point_query',
      x_m: 1,
      y_m: 2,
      lanelets: [
        {
          id: '10',
          attributes: { type: 'lanelet', one_way: 'yes' },
          bounds_m: { min_x_m: 0, min_y_m: -2, max_x_m: 10, max_y_m: 2 },
        },
      ],
      areas: [{ id: '20', attributes: { subtype: 'keepout' } }],
    });
    expect(parsed.lanelets.map((item) => item.id)).toEqual(['10']);
    expect(parsed.areas.map((item) => item.id)).toEqual(['20']);
    expect(parsed.lanelets[0].attributes.one_way).toBe('yes');
    expect(parsed.lanelets[0].bounds_m?.max_x_m).toBe(10);
    expect(parsed.areas[0].bounds_m).toBeNull();
    const empty = parsePointQuery({ schema_version: 1, type: 'point_query', x_m: 0, y_m: 0 });
    expect(empty.lanelets).toEqual([]);
    expect(empty.areas).toEqual([]);
  });

  it('解析地图信息', () => {
    const info = parseMapInfo({
      schema_version: 1,
      type: 'map_info',
      map_id: 'mapping_example',
      map_version: 'a'.repeat(64),
      counts: { lanelets: 371, areas: 76 },
      bounds_m: { min_x_m: -1, min_y_m: -2, max_x_m: 3, max_y_m: 4 },
    });
    expect(info.map_id).toBe('mapping_example');
    expect(info.counts.lanelets).toBe(371);
    expect(info.bounds_m?.max_y_m).toBe(4);
  });

  it('载荷错误包含 JSON 路径', () => {
    try {
      parseLaneletGeometry({ schema_version: 1, type: 'lanelet_geometry', id: '', left: [], right: [] });
      throw new Error('should have thrown');
    } catch (error) {
      expect(error).toBeInstanceOf(ServicePayloadError);
      expect((error as ServicePayloadError).path).toBe('$.id');
    }
  });
});
