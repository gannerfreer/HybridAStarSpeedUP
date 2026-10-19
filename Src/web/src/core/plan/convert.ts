import type { CorridorOverlay, Pt } from '../playback/overlay';
import type { PlaybackIndexSample, PlaybackSampleData } from '../playback/types';
import type { PlanResult } from './types';

/** 规划服务结果 → W4 回放渲染管线（复用 buildPlaybackOverlay/命令）。 */

export function planResultToPlaybackSample(result: PlanResult): PlaybackSampleData {
  const sample: PlaybackIndexSample = {
    id: `plan-${result.job_id || result.request_id || 'result'}`,
    kind: 'plan_result',
    type: 'plan_result',
    schema_version: 1,
    map_id: result.map_id,
    map_version: result.map_version,
    file: '',
    sha256: '',
    expected_display: '规划服务结果（来自 planner_plan_service）',
    synthetic: false,
  };
  return {
    kind: 'plan_result',
    sample,
    result: {
      request_id: result.request_id,
      status: result.status,
      reason: result.reason,
      map_id: result.map_id,
      map_version: result.map_version,
      path: result.path,
      trajectory: result.trajectory,
      statistics: result.statistics,
      synthetic: false,
      synthetic_note: '',
    },
  };
}

/** 规划结果的走廊 → 覆盖层走廊（含 s 区间，供 web-002 文字标注）。 */
export function planCorridorsToOverlay(result: PlanResult): CorridorOverlay[] {
  if (result.corridor.length === 0) return [];
  return [
    {
      id: `plan-${result.job_id || result.request_id}`,
      polygons: result.corridor.map((polygon) => ({
        ring: polygon.vertices.map((vertex): Pt => [vertex.x_m, vertex.y_m]),
        s_start_m: polygon.s_start_m,
        s_end_m: polygon.s_end_m,
      })),
    },
  ];
}
