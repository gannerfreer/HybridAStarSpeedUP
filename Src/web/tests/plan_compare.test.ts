import { describe, expect, it } from 'vitest';
import { buildComparisonTable } from '../src/core/plan/compare';
import { planCorridorsToOverlay, planResultToPlaybackSample } from '../src/core/plan/convert';
import type { PlanResult } from '../src/core/plan/types';

function makeResult(overrides: Partial<PlanResult> = {}): PlanResult {
  return {
    job_id: 'job-1',
    job_seq: 1,
    request_id: 'req-1',
    map_id: 'mapping_example',
    map_version: 'v1',
    status: 'success',
    reason: '',
    path: [],
    corridor: [],
    trajectory: [],
    validation: null,
    statistics: {},
    ...overrides,
  };
}

describe('算法比较表', () => {
  it('两个后端的并排统计（含单位与 N/A）', () => {
    const table = buildComparisonTable([
      {
        backend: 'kth_normalized',
        status: 'success',
        error: null,
        result: makeResult({
          statistics: {
            planning_time_s: 0.031,
            total_length_m: 12.5,
            direction_changes: 1,
            expanded_nodes: 4123,
          },
          validation: { valid: true, reason: '' },
        }),
      },
      {
        backend: 'ros_free',
        status: 'success',
        error: null,
        result: makeResult({
          statistics: { planning_time_s: 0.052, total_length_m: 12.4, direction_changes: 0 },
          validation: { valid: false, reason: '速度超限' },
        }),
      },
    ]);
    expect(table.backends).toEqual(['kth_normalized', 'ros_free']);
    const byLabel = new Map(table.rows.map((row) => [row.label, row]));
    expect(byLabel.get('状态')?.values).toEqual(['success', 'success']);
    expect(byLabel.get('规划耗时')?.values).toEqual(['0.031', '0.052']);
    expect(byLabel.get('规划耗时')?.unit).toBe('s');
    expect(byLabel.get('总长度')?.values).toEqual(['12.500', '12.400']);
    expect(byLabel.get('换向次数')?.values).toEqual(['1', '0']);
    expect(byLabel.get('展开节点数')?.values).toEqual(['4123', 'N/A']);
    expect(byLabel.get('轨迹校验')?.values).toEqual(['通过', '未通过（速度超限）']);
  });

  it('失败/错误的运行显示原因', () => {
    const table = buildComparisonTable([
      { backend: 'a', status: 'error', error: 'HTTP 500', result: null },
      { backend: 'b', status: 'no_path', error: null, result: makeResult({ status: 'no_path', reason: '穷举' }) },
    ]);
    const statusRow = table.rows[0];
    expect(statusRow.values[0]).toBe('错误：HTTP 500');
    expect(statusRow.values[1]).toBe('no_path');
    expect(table.rows.find((row) => row.label === '规划耗时')?.values).toEqual(['N/A', 'N/A']);
  });
});

describe('规划结果 → W4 渲染管线', () => {
  it('planResultToPlaybackSample 保留路径/轨迹/统计与版本', () => {
    const result = makeResult({
      path: [
        {
          points: [
            { pose: { x_m: 0, y_m: 0, yaw_rad: 0 }, s_m: 0, curvature_inv_m: 0, direction: 'forward' },
            { pose: { x_m: 1, y_m: 0, yaw_rad: 0 }, s_m: 1, curvature_inv_m: 0, direction: 'forward' },
          ],
        },
      ],
      trajectory: [{ t_s: 0, pose: { x_m: 0, y_m: 0, yaw_rad: 0 }, velocity_mps: 0, acceleration_mps2: 0, curvature_inv_m: 0 }],
      statistics: { total_length_m: 1 },
    });
    const sample = planResultToPlaybackSample(result);
    expect(sample.kind).toBe('plan_result');
    expect(sample.sample.id).toBe('plan-job-1');
    expect(sample.sample.map_version).toBe('v1');
    if (sample.kind === 'plan_result') {
      expect(sample.result.path).toHaveLength(1);
      expect(sample.result.trajectory).toHaveLength(1);
      expect(sample.result.statistics.total_length_m).toBe(1);
      expect(sample.result.synthetic).toBe(false);
    }
  });

  it('planCorridorsToOverlay 保留 s 区间（web-002 标注数据）', () => {
    const result = makeResult({
      corridor: [
        { s_start_m: 0, s_end_m: 3, vertices: [{ x_m: 0, y_m: 0 }, { x_m: 3, y_m: 0 }, { x_m: 3, y_m: 2 }] },
        { s_start_m: 3, s_end_m: 6.5, vertices: [{ x_m: 3, y_m: 0 }, { x_m: 6, y_m: 0 }, { x_m: 6, y_m: 2 }] },
      ],
    });
    const overlay = planCorridorsToOverlay(result);
    expect(overlay).toHaveLength(1);
    expect(overlay[0].polygons).toHaveLength(2);
    expect(overlay[0].polygons[1].s_end_m).toBe(6.5);
    expect(planCorridorsToOverlay(makeResult())).toEqual([]);
  });
});
