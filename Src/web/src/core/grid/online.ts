import type { SceneObstacle } from '../scene/types';
import type { BoundsM } from '../types';
import type { Viewport } from '../transform';
import { parseGridDocument } from './parse';
import type { GridLayerData } from './types';

/**
 * X-W2 在线栅格（协议 v1 `Docs/expansion/grid_protocol.md`）：
 * - 数据源：`POST /map/grid/driveable`（可行驶域）与 `POST /map/scene/preview`
 *   （规划场景预览，障碍叠加，含 `scene_id`）；均携带单调 `request_seq`（§6）。
 * - 版本一致性（§3.5）：`/health` 的 `grid_protocol_version` 主版本必须为 1；
 *   `scene_rules_version` 必须为 `scene_v1`，否则禁用图层并显示可见状态。
 * - 陈旧拒绝（§6）：每来源独立 `latest_seq`；晚到响应不覆盖新请求；
 *   响应 `map_id`/`map_version` 与当前底图不一致一律丢弃。
 * - 显示范围：在线显示 ≤1024² 单元（benchmark_tiers §4.1）；超限按 2 的幂
 *   提高请求分辨率并在界面标注"显示上限调整"，不得标注为原请求精度。
 */

export type GridSourceName = 'driveable' | 'planning_scene';

export const GRID_PROTOCOL_MAJOR = 1;
export const GRID_PROTOCOL_VERSION_STRING = '1.0';
export const GRID_SCENE_RULES_VERSION = 'scene_v1';
export const GRID_DISPLAY_MAX_CELLS = 1024 * 1024;

export const GRID_SOURCE_LABELS: Record<GridSourceName, string> = {
  driveable: '可行驶域（在线）',
  planning_scene: '规划场景预览（在线）',
};

// ---------------------------------------------------------------------------
// 健康与版本一致性（协议 §3.5）
// ---------------------------------------------------------------------------

export interface GridHealthInfo {
  map_id: string | null;
  map_version: string | null;
  grid_protocol_version: string | null;
  scene_rules_version: string | null;
}

export interface GridVersionCheck {
  protocol_version: string | null;
  scene_rules_version: string | null;
  protocol_ok: boolean;
  scene_rules_ok: boolean;
  /** 禁用/警告原因；全部通过时为空串。 */
  reason: string;
}

/** `"<major>.<minor>"` → 主版本；格式非法返回 null。 */
export function protocolMajor(version: string | null | undefined): number | null {
  if (typeof version !== 'string') return null;
  const match = /^(\d+)\.(\d+)$/.exec(version);
  if (!match) return null;
  return Number(match[1]);
}

/** `/health` 包络解析（缺失版本字段按 null，调用方视为不匹配并禁用）。 */
export function parseGridHealth(value: unknown): GridHealthInfo {
  if (typeof value !== 'object' || value === null || Array.isArray(value)) {
    throw new Error('health 响应必须是对象');
  }
  const record = value as Record<string, unknown>;
  if (record.schema_version !== undefined && record.schema_version !== 1) {
    throw new Error(`health schema_version 未知：${String(record.schema_version)}（仅支持 1）`);
  }
  if (record.type !== undefined && record.type !== 'health') {
    throw new Error(`health type 非法：${JSON.stringify(record.type)}`);
  }
  const text = (key: string): string | null =>
    typeof record[key] === 'string' && (record[key] as string).length > 0 ? (record[key] as string) : null;
  return {
    map_id: text('map_id'),
    map_version: text('map_version'),
    grid_protocol_version: text('grid_protocol_version'),
    scene_rules_version: text('scene_rules_version'),
  };
}

/** 协议 §3.5 一致性判定：主版本必须为 1；场景规则必须为 scene_v1。 */
export function checkGridVersionConsistency(health: GridHealthInfo | null): GridVersionCheck {
  const protocolVersion = health?.grid_protocol_version ?? null;
  const sceneRulesVersion = health?.scene_rules_version ?? null;
  const major = protocolMajor(protocolVersion);
  const protocolOk = major === GRID_PROTOCOL_MAJOR;
  const sceneRulesOk = sceneRulesVersion === GRID_SCENE_RULES_VERSION;
  const problems: string[] = [];
  if (protocolVersion === null) {
    problems.push('地图服务未声明 grid_protocol_version（旧服务或字段缺失）');
  } else if (!protocolOk) {
    problems.push(
      `协议主版本不匹配：服务 grid_protocol_version=${protocolVersion}（需要 ${GRID_PROTOCOL_MAJOR}.x）`,
    );
  }
  if (sceneRulesVersion === null) {
    problems.push('地图服务未声明 scene_rules_version（场景组合规则版本缺失）');
  } else if (!sceneRulesOk) {
    problems.push(
      `场景组合规则版本不匹配：服务 scene_rules_version=${sceneRulesVersion}（需要 ${GRID_SCENE_RULES_VERSION}）`,
    );
  }
  return {
    protocol_version: protocolVersion,
    scene_rules_version: sceneRulesVersion,
    protocol_ok: protocolOk,
    scene_rules_ok: sceneRulesOk,
    reason: problems.join('；'),
  };
}

// ---------------------------------------------------------------------------
// 请求窗口与显示上限
// ---------------------------------------------------------------------------

export function intersectBounds(a: BoundsM, b: BoundsM): BoundsM | null {
  const minX = Math.max(a.min_x_m, b.min_x_m);
  const minY = Math.max(a.min_y_m, b.min_y_m);
  const maxX = Math.min(a.max_x_m, b.max_x_m);
  const maxY = Math.min(a.max_y_m, b.max_y_m);
  if (!(minX < maxX) || !(minY < maxY)) return null;
  return { min_x_m: minX, min_y_m: minY, max_x_m: maxX, max_y_m: maxY };
}

/** 当前视口覆盖的世界矩形（与地图边界求交；无交集返回 null）。 */
export function viewportWindow(viewport: Viewport, bounds: BoundsM): BoundsM | null {
  if (!(viewport.scale_px_per_m > 0)) return null;
  const halfW = viewport.width_px / 2 / viewport.scale_px_per_m;
  const halfH = viewport.height_px / 2 / viewport.scale_px_per_m;
  const visible: BoundsM = {
    min_x_m: viewport.center_x_m - halfW,
    min_y_m: viewport.center_y_m - halfH,
    max_x_m: viewport.center_x_m + halfW,
    max_y_m: viewport.center_y_m + halfH,
  };
  return intersectBounds(visible, bounds);
}

/** 与服务 `cell_count` 一致的单元数：ceil(dx/res) × ceil(dy/res)。 */
export function cellCountFor(window: BoundsM, resolution_m: number): number {
  if (!(resolution_m > 0)) return Number.POSITIVE_INFINITY;
  const cols = Math.ceil((window.max_x_m - window.min_x_m) / resolution_m);
  const rows = Math.ceil((window.max_y_m - window.min_y_m) / resolution_m);
  return cols * rows;
}

export interface DisplayResolutionFit {
  /** 实际请求给服务的分辨率（可能因显示上限被 2 的幂提高）。 */
  resolution_m: number;
  /** 是否因显示上限调整（区别于服务端降级）。 */
  adjusted: boolean;
  factor: number;
  cells: number;
  within: boolean;
}

/**
 * 在线显示上限（≤1024²，benchmark_tiers §4.1）：单元数超限时按 2 的幂提高
 * 请求分辨率（最多 32 次，与协议 §4 服务降级循环同构）；仍超限返回 within=false。
 */
export function fitDisplayResolution(
  window: BoundsM,
  requested_resolution_m: number,
  max_cells: number = GRID_DISPLAY_MAX_CELLS,
): DisplayResolutionFit {
  let resolution = requested_resolution_m > 0 ? requested_resolution_m : 1;
  let factor = 1;
  let cells = cellCountFor(window, resolution);
  for (let step = 0; step < 32 && cells > max_cells; step += 1) {
    resolution *= 2;
    factor *= 2;
    cells = cellCountFor(window, resolution);
  }
  return { resolution_m: resolution, adjusted: factor > 1, factor, cells, within: cells <= max_cells };
}

// ---------------------------------------------------------------------------
// 陈旧拒绝（协议 §6）
// ---------------------------------------------------------------------------

export type GridAcceptDecision = 'apply' | 'stale' | 'anomaly';

/**
 * 单来源 `request_seq` 跟踪器（每来源独立实例）：
 * - `begin()` 分配 `latest_seq + 1`；
 * - 响应回显 seq 与请求不符 → anomaly；
 * - `< latest` 陈旧丢弃并计数；`> latest` 未发起 → anomaly；
 * - 缺失回显且已有更晚请求时按陈旧处理（`seq < latest` 分支）。
 */
export class GridSourceTracker {
  private latest = 0;

  private stale = 0;

  private anomalies = 0;

  private mapMismatch = 0;

  begin(): number {
    this.latest += 1;
    return this.latest;
  }

  get latest_seq(): number {
    return this.latest;
  }

  get stale_ignored(): number {
    return this.stale;
  }

  get anomaly_count(): number {
    return this.anomalies;
  }

  get map_mismatch_ignored(): number {
    return this.mapMismatch;
  }

  accept(seq: number, echoed_seq: number | null = null): GridAcceptDecision {
    if (echoed_seq !== null && echoed_seq !== seq) {
      this.anomalies += 1;
      return 'anomaly';
    }
    if (seq < this.latest) {
      this.stale += 1;
      return 'stale';
    }
    if (seq > this.latest) {
      this.anomalies += 1;
      return 'anomaly';
    }
    return 'apply';
  }

  noteMapMismatch(): void {
    this.mapMismatch += 1;
  }
}

export type OnlineAcceptDecision = 'apply' | 'stale' | 'anomaly' | 'map_mismatch';

export interface OnlineAcceptOutcome {
  decision: OnlineAcceptDecision;
  apply: boolean;
  reason: string;
}

export interface OnlineAcceptInput {
  tracker: GridSourceTracker;
  seq: number;
  echoed_seq: number | null;
  source: GridSourceName;
  layer: GridLayerData;
  map_id: string;
  map_version: string;
}

/**
 * 在线响应验收管线（协议 §6 规则 2–3）：seq 判定 → 来源核对 → 地图版本核对。
 * 纯函数（仅副作用为跟踪器计数），供 App 与测试共用。
 */
export function acceptOnlineGridResponse(input: OnlineAcceptInput): OnlineAcceptOutcome {
  const decision = input.tracker.accept(input.seq, input.echoed_seq);
  if (decision === 'stale') {
    return {
      decision: 'stale',
      apply: false,
      reason: `响应 seq ${input.seq} < latest ${input.tracker.latest_seq}（陈旧，已丢弃）`,
    };
  }
  if (decision === 'anomaly') {
    return {
      decision: 'anomaly',
      apply: false,
      reason:
        input.echoed_seq !== null && input.echoed_seq !== input.seq
          ? `响应 request_seq 回显 ${input.echoed_seq} ≠ 请求 ${input.seq}（协议异常，已丢弃）`
          : `响应 seq ${input.seq} > latest ${input.tracker.latest_seq}（未发起，协议异常）`,
    };
  }
  if (input.layer.source !== input.source) {
    return {
      decision: 'anomaly',
      apply: false,
      reason: `响应来源 ${input.layer.source} ≠ 请求 ${input.source}（协议异常，已丢弃）`,
    };
  }
  if (input.layer.map_id !== input.map_id || input.layer.map_version !== input.map_version) {
    input.tracker.noteMapMismatch();
    return {
      decision: 'map_mismatch',
      apply: false,
      reason:
        `快照地图版本不一致：响应 ${input.layer.map_id}@${input.layer.map_version} ` +
        `≠ 当前底图 ${input.map_id}@${input.map_version}（已丢弃）`,
    };
  }
  return { decision: 'apply', apply: true, reason: '' };
}

// ---------------------------------------------------------------------------
// HTTP 客户端
// ---------------------------------------------------------------------------

export type OnlineFetchLike = (input: string, init?: RequestInit) => Promise<Response>;

export type OnlineGridErrorKind = 'network' | 'timeout' | 'http' | 'payload';

export class OnlineGridError extends Error {
  readonly kind: OnlineGridErrorKind;
  readonly status?: number;
  readonly reason?: string;

  constructor(kind: OnlineGridErrorKind, message: string, options: { status?: number; reason?: string } = {}) {
    super(message);
    this.name = 'OnlineGridError';
    this.kind = kind;
    this.status = options.status;
    this.reason = options.reason;
  }

  get isStaleMap(): boolean {
    return this.status === 409 && (this.reason ?? '').includes('stale_map');
  }
}

export interface OnlineGridClientOptions {
  /** 默认 `/api`（Vite dev 代理 / CDP 代理 → 地图服务 8765）。 */
  base_url?: string;
  fetch_fn?: OnlineFetchLike;
  timeout_ms?: number;
}

export interface OnlineGridRequest {
  window: BoundsM;
  resolution_m: number;
  /** 仅 planning_scene 使用（缺省服务按 [] 处理）。 */
  obstacles?: SceneObstacle[];
}

export interface OnlineGridResult {
  layer: GridLayerData;
  /** 响应原始文本字节数（显示与证据记录）。 */
  bytes: number;
  /** 响应原始 JSON 文本（调试/证据；不参与业务判定）。 */
  raw_text: string;
}

export class OnlineGridClient {
  readonly baseUrl: string;
  private readonly fetchFn: OnlineFetchLike;
  private readonly timeoutMs: number;

  constructor(options: OnlineGridClientOptions = {}) {
    this.baseUrl = (options.base_url ?? '/api').replace(/\/+$/, '');
    this.fetchFn = options.fetch_fn ?? ((input, init) => fetch(input, init));
    this.timeoutMs = options.timeout_ms ?? 8000;
  }

  health(): Promise<GridHealthInfo> {
    return this.request('GET', '/health', undefined, parseGridHealth);
  }

  driveable(request: OnlineGridRequest, request_seq: number): Promise<OnlineGridResult> {
    return this.request(
      'POST',
      '/map/grid/driveable',
      { window: request.window, resolution_m: request.resolution_m, request_seq },
      (value) => ({ layer: this.parseSnapshot(value, 'driveable'), bytes: 0, raw_text: '' }),
    );
  }

  preview(request: OnlineGridRequest, request_seq: number): Promise<OnlineGridResult> {
    return this.request(
      'POST',
      '/map/scene/preview',
      {
        window: request.window,
        resolution_m: request.resolution_m,
        obstacles: request.obstacles ?? [],
        request_seq,
      },
      (value) => ({ layer: this.parseSnapshot(value, 'planning_scene'), bytes: 0, raw_text: '' }),
    );
  }

  private parseSnapshot(value: unknown, source: GridSourceName): GridLayerData {
    const layer = parseGridDocument(value);
    if (layer.kind !== 'grid_snapshot') {
      throw new Error(`在线栅格响应必须是 grid_snapshot（收到 ${layer.kind}）`);
    }
    if (layer.source !== source) {
      throw new Error(`在线栅格响应来源 ${layer.source} ≠ 请求 ${source}`);
    }
    if (source === 'planning_scene' && (layer.scene_id === null || layer.scene_version === null)) {
      throw new Error('planning_scene 快照必须携带 scene_id/scene_version');
    }
    return layer;
  }

  private async request<T>(
    method: 'GET' | 'POST',
    path: string,
    body: unknown,
    parser: (value: unknown) => T,
  ): Promise<T> {
    const url = `${this.baseUrl}${path}`;
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), this.timeoutMs);
    let response: Response;
    try {
      response = await this.fetchFn(url, {
        method,
        signal: controller.signal,
        headers: {
          Accept: 'application/json',
          ...(body !== undefined ? { 'Content-Type': 'application/json' } : {}),
        },
        ...(body !== undefined ? { body: JSON.stringify(body) } : {}),
      });
    } catch (error) {
      if (controller.signal.aborted) {
        throw new OnlineGridError('timeout', `请求超时（${this.timeoutMs} ms）：${url}`);
      }
      throw new OnlineGridError(
        'network',
        `无法连接地图服务：${error instanceof Error ? error.message : String(error)}`,
      );
    } finally {
      clearTimeout(timer);
    }

    if (!response.ok) {
      let statusText = '';
      let reason = '';
      try {
        const payload = (await response.json()) as Record<string, unknown>;
        if (typeof payload.status === 'string') statusText = payload.status;
        if (typeof payload.reason === 'string') reason = payload.reason;
      } catch {
        // 错误体不可解析时仅保留 HTTP 状态。
      }
      const detail = [statusText, reason].filter((part) => part.length > 0).join(': ');
      throw new OnlineGridError('http', `HTTP ${response.status}${detail ? ` ${detail}` : ''}：${url}`, {
        status: response.status,
        reason: detail,
      });
    }

    let text: string;
    try {
      text = await response.text();
    } catch (error) {
      throw new OnlineGridError(
        'payload',
        `响应读取失败：${error instanceof Error ? error.message : String(error)}`,
      );
    }
    let value: unknown;
    try {
      value = JSON.parse(text) as unknown;
    } catch (error) {
      throw new OnlineGridError(
        'payload',
        `响应不是合法 JSON：${error instanceof Error ? error.message : String(error)}`,
      );
    }
    try {
      const parsed = parser(value);
      if (typeof parsed === 'object' && parsed !== null && 'layer' in (parsed as Record<string, unknown>)) {
        const result = parsed as unknown as OnlineGridResult;
        result.bytes = new TextEncoder().encode(text).byteLength;
        result.raw_text = text;
      }
      return parsed;
    } catch (error) {
      throw new OnlineGridError('payload', error instanceof Error ? error.message : String(error));
    }
  }
}

/** 在线图层结果信息（面板与证据展示用）。 */
export interface OnlineGridInfo {
  request_seq: number;
  requested_resolution_m: number;
  effective_resolution_m: number;
  resolution_m: number;
  degraded: boolean;
  /** 因在线显示上限（≤1024²）在请求前提高分辨率。 */
  display_adjusted: boolean;
  display_factor: number;
  window: BoundsM | null;
  scene_id: string | null;
  scene_version: string | null;
  bytes: number;
  fetched_at_ms: number;
}
