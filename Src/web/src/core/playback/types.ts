import type { MapSnapshot } from '../types';
import type { Pose2d } from '../pose';

/**
 * W4 固定回放样例类型（tests/fixtures/v4/playback/）。
 * 全部样例 synthetic: true —— 仅作显示测试，不是算法正确性证据。
 */

export interface PlaybackIndexSample {
  id: string;
  kind: string;
  type: string;
  schema_version: number;
  map_id: string;
  map_version: string;
  file: string;
  sha256: string;
  expected_display: string;
  synthetic: boolean;
}

export interface PlaybackIndex {
  schema_version: 1;
  type: 'playback_index';
  fixture_version: string;
  map_id: string;
  map_version: string;
  hash_algorithm: string;
  synthetic_note: string;
  samples: PlaybackIndexSample[];
}

export interface PlaybackPoint {
  x_m: number;
  y_m: number;
}

export interface PathPoint {
  pose: Pose2d;
  s_m: number;
  curvature_inv_m: number;
  direction: 'forward' | 'reverse';
}

export interface PathSegment {
  points: PathPoint[];
}

export interface TrajectoryPoint {
  t_s: number;
  pose: Pose2d;
  velocity_mps: number;
  acceleration_mps2: number;
  curvature_inv_m: number;
}

export interface PlanResult {
  request_id: string;
  status: string;
  reason: string;
  map_id: string;
  map_version: string;
  path: PathSegment[];
  trajectory: TrajectoryPoint[];
  statistics: Record<string, unknown>;
  synthetic: boolean;
  synthetic_note: string;
}

export interface BsplineCurve {
  id: string;
  degree: number;
  parameter_domain: [number, number];
  knots: number[];
  control_points: PlaybackPoint[];
  map_id: string;
  map_version: string;
  synthetic: boolean;
}

export interface PolynomialSegment {
  index: number;
  t_start_s: number;
  duration_s: number;
  coefficients_ascending: { x_m: number[]; y_m: number[] };
}

export interface PolynomialCurve {
  id: string;
  degree: number;
  segments: PolynomialSegment[];
  map_id: string;
  map_version: string;
  synthetic: boolean;
}

export interface CorridorPolygon {
  s_start_m: number;
  s_end_m: number;
  vertices: PlaybackPoint[];
}

export interface CorridorSample {
  id: string;
  polygons: CorridorPolygon[];
  map_id: string;
  map_version: string;
  synthetic: boolean;
}

export interface CollisionObstacleRect {
  id: string;
  type: 'rectangle';
  x_min_m: number;
  y_min_m: number;
  x_max_m: number;
  y_max_m: number;
}

export interface CollisionCase {
  id: string;
  pose: Pose2d;
  footprint_corners: PlaybackPoint[];
  obstacle: CollisionObstacleRect;
  verdict: string;
  overlap_region: { min_x_m: number; min_y_m: number; max_x_m: number; max_y_m: number } | null;
  reason: string;
  map_id: string;
  map_version: string;
  synthetic: boolean;
}

export interface StaleResult {
  id: string;
  request_id: string;
  request_seq: number;
  map_version: string;
  current_map_version: string;
  current_request_seq: number;
  status: string;
  reason: string;
}

export type PlaybackSampleData =
  | { kind: 'map_snapshot'; sample: PlaybackIndexSample; map: MapSnapshot }
  | { kind: 'plan_result'; sample: PlaybackIndexSample; result: PlanResult }
  | { kind: 'bspline_curve'; sample: PlaybackIndexSample; curve: BsplineCurve }
  | { kind: 'polynomial_curve'; sample: PlaybackIndexSample; curve: PolynomialCurve }
  | { kind: 'corridor'; sample: PlaybackIndexSample; corridor: CorridorSample }
  | { kind: 'collision_case'; sample: PlaybackIndexSample; collision: CollisionCase }
  | { kind: 'stale_result'; sample: PlaybackIndexSample; stale: StaleResult };

export interface PlaybackBundle {
  index: PlaybackIndex;
  samples: PlaybackSampleData[];
  /** 校验/解析失败项（拒绝加载，不影响其他样例）。 */
  issues: string[];
  /** 小型回放地图（map_snapshot 样例，适配为内部快照类型）。 */
  map: MapSnapshot | null;
  map_file: string | null;
}

/** 回放图层开关（路径/曲线/走廊/轨迹/碰撞点）。 */
export interface PlaybackLayers {
  paths: boolean;
  curves: boolean;
  corridors: boolean;
  trajectories: boolean;
  collision: boolean;
}

export const DEFAULT_PLAYBACK_LAYERS: Readonly<PlaybackLayers> = Object.freeze({
  paths: true,
  curves: true,
  corridors: true,
  trajectories: true,
  collision: true,
});
