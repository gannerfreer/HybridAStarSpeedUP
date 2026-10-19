import { parseGridDocument } from '../grid/parse';
import type { GridLayerData } from '../grid/types';

/**
 * X-W2 规划实际场景快照（协议 §3.4）：规划结果携带 `scene_id`/`scene_version`
 * 后，浏览器经 `GET /plan/scene/{scene_id}` 拉取该次规划实际使用的输入快照，
 * 与"当前预览"（地图服务 `/map/scene/preview`）显式区分、独立展示。
 * 历史场景不覆盖预览：预览层与规划快照层是两个独立状态。
 */

export type PlanSceneSnapshotStatus = 'idle' | 'loading' | 'ready' | 'not_found' | 'error' | 'disabled';

export interface PlanSceneSnapshotState {
  status: PlanSceneSnapshotStatus;
  scene_id: string | null;
  scene_version: string | null;
  layer: GridLayerData | null;
  error: string | null;
  /** 来源规划任务（面板标注"某次规划实际使用的场景"）。 */
  job_id: string | null;
  backend: string | null;
  fetched_at_ms: number | null;
}

export function createPlanSceneSnapshotState(): PlanSceneSnapshotState {
  return {
    status: 'idle',
    scene_id: null,
    scene_version: null,
    layer: null,
    error: null,
    job_id: null,
    backend: null,
    fetched_at_ms: null,
  };
}

/**
 * 晚到响应防护：只应用与最新请求 `scene_id` 相同的响应；
 * 旧 scene_id 的迟到响应丢弃并计数（与 W5 tracker 同语义）。
 */
export class PlanSceneSnapshotTracker {
  private latest: string | null = null;

  private stale = 0;

  request(scene_id: string): void {
    this.latest = scene_id;
  }

  accept(scene_id: string): boolean {
    if (this.latest === scene_id) return true;
    this.stale += 1;
    return false;
  }

  get latest_scene_id(): string | null {
    return this.latest;
  }

  get stale_ignored(): number {
    return this.stale;
  }
}

/** 校验 `/plan/scene/{scene_id}` 响应：grid_snapshot + planning_scene + scene_id 一致。 */
export function parsePlanSceneSnapshot(value: unknown, expected_scene_id: string): GridLayerData {
  const layer = parseGridDocument(value);
  if (layer.kind !== 'grid_snapshot') {
    throw new Error(`规划场景快照必须是 grid_snapshot（收到 ${layer.kind}）`);
  }
  if (layer.source !== 'planning_scene') {
    throw new Error(`规划场景快照来源必须是 planning_scene（收到 ${layer.source}）`);
  }
  if (layer.scene_id !== expected_scene_id) {
    throw new Error(
      `规划场景快照 scene_id ${layer.scene_id ?? '(缺失)'} ≠ 请求 ${expected_scene_id}`,
    );
  }
  return layer;
}

export type PreviewPlanComparison = 'no_preview' | 'same' | 'different';

/**
 * "当前预览"（地图服务 /map/scene/preview）与"规划实际场景"（/plan/scene/{scene_id}）
 * 的显式区分：同 `map_version` ≠ 同场景，必须以 `scene_id` 比较（协议 §3.5 规则 4）。
 */
export function comparePreviewAndPlanScene(
  preview_scene_id: string | null,
  plan_scene_id: string | null,
): PreviewPlanComparison {
  if (preview_scene_id === null || plan_scene_id === null) return 'no_preview';
  return preview_scene_id === plan_scene_id ? 'same' : 'different';
}
