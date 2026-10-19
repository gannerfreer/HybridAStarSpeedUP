import { describe, expect, it } from 'vitest';
import {
  parseJobStatus,
  parsePlanEvent,
  parsePlanResult,
  parseSubmitResponse,
  PlanParseError,
} from '../src/core/plan/parse';

const submit = {
  job_id: 'job-1',
  job_seq: 3,
  request_id: 'req-1',
  map_id: 'mapping_example',
  map_version: 'v1',
  status: 'queued',
};

const pathPoint = (x: number, y: number, direction = 'forward') => ({
  pose: { x_m: x, y_m: y, yaw_rad: 0 },
  s_m: x,
  curvature_inv_m: 0,
  direction,
});

describe('规划服务响应解析', () => {
  it('提交响应', () => {
    const parsed = parseSubmitResponse(submit);
    expect(parsed.job_id).toBe('job-1');
    expect(parsed.job_seq).toBe(3);
    expect(parsed.status).toBe('queued');
    expect(() => parseSubmitResponse({ ...submit, job_id: '' })).toThrowError(/job_id/);
    expect(() => parseSubmitResponse({ ...submit, job_seq: Number.NaN })).toThrowError(/finite/);
  });

  it('状态响应（status/stage/reason，job_seq 可选）', () => {
    const parsed = parseJobStatus({ job_id: 'job-1', status: 'running', stage: 'hybrid_search', job_seq: 4 });
    expect(parsed.status).toBe('running');
    expect(parsed.stage).toBe('hybrid_search');
    expect(parsed.job_seq).toBe(4);
    const minimal = parseJobStatus({ job_id: 'job-1', status: 'cancelled' });
    expect(minimal.stage).toBeUndefined();
  });

  it('结果：分段路径 + 走廊 + 轨迹 + 校验 + 统计', () => {
    const parsed = parsePlanResult({
      job_id: 'job-1',
      job_seq: 5,
      request_id: 'req-1',
      map_id: 'mapping_example',
      map_version: 'v1',
      status: 'success',
      reason: '',
      path: [
        { points: [pathPoint(0, 0), pathPoint(1, 0)] },
        { points: [pathPoint(1, 0, 'reverse'), pathPoint(0.5, 0, 'reverse')] },
      ],
      corridor: { polygons: [{ s_start_m: 0, s_end_m: 2, vertices: [{ x_m: 0, y_m: 0 }, { x_m: 1, y_m: 0 }, { x_m: 1, y_m: 1 }] }] },
      trajectory: [{ t_s: 0, pose: { x_m: 0, y_m: 0, yaw_rad: 0 }, velocity_mps: 0, acceleration_mps2: 1, curvature_inv_m: 0 }],
      validation: { valid: true, reason: '' },
      statistics: { planning_time_s: 0.01, direction_changes: 1 },
    });
    expect(parsed.path).toHaveLength(2);
    expect(parsed.path[1].points[0].direction).toBe('reverse');
    expect(parsed.corridor).toHaveLength(1);
    expect(parsed.corridor[0].s_end_m).toBe(2);
    expect(parsed.trajectory).toHaveLength(1);
    expect(parsed.validation?.valid).toBe(true);
    expect(parsed.statistics.direction_changes).toBe(1);
  });

  it('结果：平铺路径点按方向自动分段、走廊数组形状兼容', () => {
    const parsed = parsePlanResult({
      job_id: 'job-2',
      status: 'success',
      path: [pathPoint(0, 0), pathPoint(1, 0), pathPoint(1, 0, 'reverse')],
      corridor: [{ s_start_m: 0, s_end_m: 1, vertices: [{ x_m: 0, y_m: 0 }, { x_m: 1, y_m: 0 }, { x_m: 0, y_m: 1 }] }],
    });
    expect(parsed.path).toHaveLength(2);
    expect(parsed.path[0].points).toHaveLength(2);
    expect(parsed.path[1].points[0].direction).toBe('reverse');
    expect(parsed.corridor).toHaveLength(1);
  });

  it('失败结果：状态与原因保留、无几何', () => {
    const parsed = parsePlanResult({ job_id: 'job-3', status: 'no_path', reason: '搜索穷举' });
    expect(parsed.status).toBe('no_path');
    expect(parsed.reason).toBe('搜索穷举');
    expect(parsed.path).toEqual([]);
    expect(parsed.validation).toBeNull();
  });

  it('真实服务形状：hybrid.segments 路径 + statistics.hybrid/timings_s 展平派生', () => {
    const parsed = parsePlanResult({
      job_id: 'job-real',
      job_seq: 9,
      request_id: 'req-r',
      map_id: 'mapping_example',
      map_version: 'v1',
      status: 'succeeded',
      stage: 'done',
      hybrid: {
        backend: 'ros_free',
        segments: [
          { points: [pathPoint(0, 0), pathPoint(1, 0)] },
          { points: [pathPoint(1, 0, 'reverse'), pathPoint(0.5, 0, 'reverse')] },
        ],
        statistics: { expanded_nodes: 1234, collision_checks: 5678, path_length_m: 12.5, search_time_ms: 310 },
      },
      corridor: {
        id: 'c1',
        polygons: [{ s_start_m: 0, s_end_m: 3, vertices: [{ x_m: 0, y_m: 0 }, { x_m: 3, y_m: 0 }, { x_m: 3, y_m: 2 }] }],
      },
      trajectory: [],
      validation: { valid: true, reason: '', checks: { trajectory_points: 0 } },
      statistics: {
        hybrid_backend: 'ros_free',
        stage: 'done',
        timings_s: { hybrid: 0.31, corridor: 0.02, optimize: 0.44, validate: 0.01 },
        hybrid: { expanded_nodes: 1234, path_length_m: 12.5 },
        path_length_m: 12.5,
      },
    });
    expect(parsed.path).toHaveLength(2);
    expect(parsed.path[1].points[0].direction).toBe('reverse');
    expect(parsed.corridor).toHaveLength(1);
    expect(parsed.statistics.expanded_nodes).toBe(1234);
    expect(parsed.statistics.total_length_m).toBe(12.5);
    expect(parsed.statistics.planning_time_s).toBe(0.31);
    expect(parsed.statistics.solve_time_s).toBe(0.44);
    expect(parsed.statistics.direction_changes).toBe(1);
    expect(parsed.validation?.valid).toBe(true);
  });

  it('校验未通过结果', () => {
    const parsed = parsePlanResult({
      job_id: 'job-4',
      status: 'validation_failed',
      validation: { valid: false, reason: '速度超限' },
    });
    expect(parsed.validation).toEqual({ valid: false, reason: '速度超限' });
  });

  it('事件解析：job_update / job_result（容忍 type 字段）', () => {
    const update = parsePlanEvent({ kind: 'job_update', job_seq: 6, job_id: 'job-1', status: 'running', stage: 'corridor' });
    expect(update.kind).toBe('job_update');
    expect(update.stage).toBe('corridor');
    const resultEvent = parsePlanEvent({
      type: 'job_result',
      job_seq: 7,
      job_id: 'job-1',
      result: { job_id: 'job-1', status: 'success', statistics: { total_length_m: 3 } },
    });
    expect(resultEvent.kind).toBe('job_result');
    expect(resultEvent.result?.statistics.total_length_m).toBe(3);
    expect(() => parsePlanEvent({ kind: 'other', job_seq: 1, job_id: 'x' })).toThrowError(PlanParseError);
    expect(() => parsePlanEvent({ kind: 'job_update', job_seq: 1 })).toThrowError(/job_id/);
  });
});
