import { describe, expect, it } from 'vitest';
import {
  parseBsplineCurve,
  parseCollisionCase,
  parseCorridor,
  parsePlaybackIndex,
  parsePlaybackMap,
  parsePlanResult,
  parsePolynomialCurve,
  parseStaleResult,
  PlaybackParseError,
} from '../src/core/playback/parse';
import { readPlaybackFile } from './playback_fixtures';

describe('回放索引解析', () => {
  const index = parsePlaybackIndex(readPlaybackFile('index.json').value);

  it('解析 11 个样例并保留哈希与 expected_display', () => {
    expect(index.type).toBe('playback_index');
    expect(index.map_id).toBe('map_synthetic_min');
    expect(index.map_version).toBe('v1');
    expect(index.hash_algorithm).toBe('sha256');
    expect(index.samples.length).toBe(11);
    for (const sample of index.samples) {
      expect(sample.sha256).toMatch(/^[0-9a-f]{64}$/);
      expect(sample.expected_display.length).toBeGreaterThan(0);
      expect(sample.synthetic).toBe(true);
    }
    expect(index.samples.map((sample) => sample.type)).toEqual([
      'map_snapshot',
      'plan_result',
      'plan_result',
      'plan_result',
      'plan_result',
      'bspline_curve',
      'polynomial_curve',
      'corridor',
      'plan_result',
      'collision_case',
      'stale_result',
    ]);
  });

  it('拒绝错误包络/缺字段', () => {
    expect(() => parsePlaybackIndex({ schema_version: 2, type: 'playback_index' })).toThrowError(/schema_version/);
    expect(() => parsePlaybackIndex({ schema_version: 1, type: 'other' })).toThrowError(/playback_index/);
    expect(() =>
      parsePlaybackIndex({ schema_version: 1, type: 'playback_index', samples: [{ id: 'x' }] }),
    ).toThrowError(PlaybackParseError);
  });
});

describe('各类型样例解析', () => {
  it('RS 路径：2 段、方向 forward、统计字段', () => {
    const result = parsePlanResult(readPlaybackFile('path_rs_open.json').value);
    expect(result.status).toBe('success');
    expect(result.request_id).toBe('req-rs-demo-001');
    expect(result.path).toHaveLength(2);
    expect(result.path[0].points).toHaveLength(4);
    expect(result.path[0].points[0].direction).toBe('forward');
    expect(result.path[1].points[0].curvature_inv_m).toBeCloseTo(0.2, 9);
    expect(result.statistics.total_length_m).toBeCloseTo(13.853982, 6);
    expect(result.trajectory).toEqual([]);
    expect(result.synthetic).toBe(true);
  });

  it('换向路径：forward + reverse 两段', () => {
    const result = parsePlanResult(readPlaybackFile('path_hybrid_reverse.json').value);
    expect(result.path).toHaveLength(2);
    expect(result.path[0].points.every((point) => point.direction === 'forward')).toBe(true);
    expect(result.path[1].points.every((point) => point.direction === 'reverse')).toBe(true);
    expect(result.statistics.direction_changes).toBe(1);
  });

  it('失败路径：no_path 且 path 为空', () => {
    const result = parsePlanResult(readPlaybackFile('path_hybrid_failure.json').value);
    expect(result.status).toBe('no_path');
    expect(result.reason).toMatch(/不存在无碰撞路径/);
    expect(result.path).toEqual([]);
    expect(result.statistics.reached_goal).toBe(false);
  });

  it('Dftpav 轨迹：时间严格递增、换向停车 v=0', () => {
    const result = parsePlanResult(readPlaybackFile('dftpav_trajectory.json').value);
    expect(result.trajectory).toHaveLength(8);
    for (let index = 1; index < result.trajectory.length; index += 1) {
      expect(result.trajectory[index].t_s).toBeGreaterThan(result.trajectory[index - 1].t_s);
    }
    const switchPoint = result.trajectory.find((point) => point.t_s === 4.0);
    expect(switchPoint?.velocity_mps).toBe(0);
    expect(result.statistics.velocity_switch_time_s).toBe(4.0);
  });

  it('拒绝时间非递增的轨迹', () => {
    const raw = structuredClone(readPlaybackFile('dftpav_trajectory.json').value) as Record<string, unknown>;
    const trajectory = raw.trajectory as Array<Record<string, unknown>>;
    trajectory[1].t_s = 0;
    expect(() => parsePlanResult(raw)).toThrowError(/strictly increasing/);
  });

  it('B 样条：节点/次数/控制点与参数域', () => {
    const curve = parseBsplineCurve(readPlaybackFile('bspline_curve.json').value);
    expect(curve.degree).toBe(3);
    expect(curve.control_points).toHaveLength(8);
    expect(curve.knots).toHaveLength(12);
    expect(curve.parameter_domain).toEqual([0, 1]);
    expect(curve.knots[3]).toBe(0);
    expect(curve.knots[8]).toBe(1);
  });

  it('拒绝节点数量不匹配与递减节点', () => {
    const raw = structuredClone(readPlaybackFile('bspline_curve.json').value) as Record<string, unknown>;
    (raw.knots as number[]).pop();
    expect(() => parseBsplineCurve(raw)).toThrowError(/knots/);
    const bad = structuredClone(readPlaybackFile('bspline_curve.json').value) as Record<string, unknown>;
    (bad.knots as number[])[5] = -1;
    expect(() => parseBsplineCurve(bad)).toThrowError(/non-decreasing/);
  });

  it('多项式：两段五次、升幂系数、段时长', () => {
    const curve = parsePolynomialCurve(readPlaybackFile('polynomial_curve.json').value);
    expect(curve.degree).toBe(5);
    expect(curve.segments).toHaveLength(2);
    expect(curve.segments[0].duration_s).toBe(4.0);
    expect(curve.segments[0].coefficients_ascending.x_m).toHaveLength(6);
    expect(curve.segments[1].t_start_s).toBe(4.0);
  });

  it('拒绝非正段时长', () => {
    const raw = structuredClone(readPlaybackFile('polynomial_curve.json').value) as Record<string, unknown>;
    (raw.segments as Array<Record<string, unknown>>)[0].duration_s = 0;
    expect(() => parsePolynomialCurve(raw)).toThrowError(/must be > 0/);
  });

  it('走廊：3 个多边形与 s 区间', () => {
    const corridor = parseCorridor(readPlaybackFile('corridor_hybrid_reverse.json').value);
    expect(corridor.polygons).toHaveLength(3);
    expect(corridor.polygons[0].s_start_m).toBe(0);
    expect(corridor.polygons[2].s_end_m).toBe(9);
    expect(corridor.polygons[0].vertices).toHaveLength(4);
  });

  it('碰撞样例：verdict/重叠区/足迹', () => {
    const collision = parseCollisionCase(readPlaybackFile('collision_case.json').value);
    expect(collision.verdict).toBe('collision');
    expect(collision.footprint_corners).toHaveLength(4);
    expect(collision.obstacle.x_min_m).toBe(11);
    expect(collision.overlap_region).toEqual({ min_x_m: 11, min_y_m: 4.1, max_x_m: 13.7, max_y_m: 5.9 });
  });

  it('过期样例：版本与请求序号', () => {
    const stale = parseStaleResult(readPlaybackFile('stale_result.json').value);
    expect(stale.map_version).toBe('v0');
    expect(stale.current_map_version).toBe('v1');
    expect(stale.request_seq).toBe(7);
    expect(stale.current_request_seq).toBe(12);
  });

  it('小型回放地图适配为内部快照（bounds/left_boundary/polygon）', () => {
    const map = parsePlaybackMap(readPlaybackFile('map_snapshot_minimal.json').value);
    expect(map.map_id).toBe('map_synthetic_min');
    expect(map.map_version).toBe('v1');
    expect(map.bounds_m).toEqual({ min_x_m: 0, min_y_m: 0, max_x_m: 50, max_y_m: 30 });
    expect(map.lanelets).toHaveLength(4);
    expect(map.lanelets[0].left).toHaveLength(2);
    expect(map.lanelets[0].attributes.one_way).toBe('true');
    expect(map.areas).toHaveLength(1);
    expect(map.areas[0].outer).toHaveLength(4);
    expect(map.counts.lanelets).toBe(4);
  });
});
