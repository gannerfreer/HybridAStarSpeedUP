import type { Pose2d } from '../pose';
import type { SceneObstacle } from '../scene/types';
import type { PathSegment, PlaybackPoint, TrajectoryPoint } from '../playback/types';

/**
 * W5 规划服务（planner_plan_service，端口 8766）契约类型。
 * 提交/取消/状态/结果 + 推送事件（job_update/job_result，含单调 job_seq）。
 */

export interface PlanAlgorithmParams {
  hybrid_backend?: string;
  /** 后端配置键（透传 hybrid_config，未知键由服务拒绝）。 */
  hybrid?: Record<string, unknown>;
  time_budget_s?: number;
  node_budget?: number;
  dftpav_time_budget_s?: number;
  /** 与真实服务 CorridorOptions 对齐（clearance_m 等；未知键会被服务拒绝）。 */
  corridor?: {
    clearance_m?: number;
    sample_step_m?: number;
    construction_step_m?: number;
    verification_step_m?: number;
    max_expansion_m?: number;
  };
  dftpav?: {
    time_budget_s?: number;
    limits?: Record<string, unknown>;
    options?: Record<string, unknown>;
  };
}

export interface PlanSubmitRequest {
  request_id: string;
  map_id: string;
  map_version: string;
  start: Pose2d;
  goal: Pose2d;
  vehicle_ref: string;
  obstacles: SceneObstacle[];
  algorithm_params: PlanAlgorithmParams;
}

export interface PlanSubmitResponse {
  job_id: string;
  job_seq: number;
  request_id: string;
  map_id: string;
  map_version: string;
  status: string;
}

export interface PlanJobStatus {
  job_id: string;
  status: string;
  stage?: string;
  reason?: string;
  job_seq?: number;
  /** 协议 §3.4：map 阶段（场景构建）完成后出现；之前缺省。 */
  scene_id?: string;
  scene_version?: string;
}

export interface PlanCorridorPolygon {
  s_start_m: number;
  s_end_m: number;
  vertices: PlaybackPoint[];
}

export interface PlanValidation {
  valid: boolean;
  reason: string;
}

export interface PlanResult {
  job_id: string;
  job_seq: number;
  request_id: string;
  map_id: string;
  map_version: string;
  status: string;
  reason: string;
  /** 协议 §3.4：规划实际使用场景的内容标识与组合规则版本（场景已构建即出现）。 */
  scene_id?: string;
  scene_version?: string;
  /** Hybrid 路径段（前进/倒车逐点方向）。 */
  path: PathSegment[];
  /** 安全走廊多边形（s 区间 + 顶点）。 */
  corridor: PlanCorridorPolygon[];
  /** 优化后轨迹（时间、速度、曲率）。 */
  trajectory: TrajectoryPoint[];
  validation: PlanValidation | null;
  statistics: Record<string, unknown>;
}

export interface PlanEvent {
  kind: 'job_update' | 'job_result';
  job_seq: number;
  job_id: string;
  status?: string;
  stage?: string;
  reason?: string;
  /** 协议 §3.4：map 阶段完成后可携带（HTTP 状态/结果为规范来源）。 */
  scene_id?: string;
  result?: PlanResult;
}

export const TERMINAL_STATUSES: readonly string[] = [
  'succeeded',
  'failed',
  'cancelled',
  // 兼容其他算法状态命名（样例/其他组件）
  'success',
  'no_path',
  'timeout',
  'optimization_failed',
  'validation_failed',
  'invalid_input',
];

export function isTerminalStatus(status: string): boolean {
  return TERMINAL_STATUSES.includes(status);
}

/** 默认对比后端（Hybrid 适配器名，与选型报告一致）。 */
export const DEFAULT_COMPARE_BACKENDS: readonly string[] = ['kth_normalized', 'ros_free'];

export const DEFAULT_HYBRID_BACKEND = 'kth_normalized';

/** 已知 Hybrid 适配器（选型报告的 normalized 候选）。 */
export const KNOWN_HYBRID_BACKENDS: readonly string[] = [
  'kth_normalized',
  'ros_free',
  'zm0612_normalized',
  'smac_normalized',
];
