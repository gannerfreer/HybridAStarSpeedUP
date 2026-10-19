import {
  parseJobStatus,
  parsePlanEvent,
  parsePlanResult,
  parseSubmitResponse,
} from './parse';
import { parsePlanSceneSnapshot } from './scene_snapshot';
import type { GridLayerData } from '../grid/types';
import type { PlanEvent, PlanJobStatus, PlanResult, PlanSubmitRequest, PlanSubmitResponse } from './types';

/** 规划服务客户端（planner_plan_service；默认 /plan-api 代理 → 8766）。 */

export type FetchLike = (input: string, init?: RequestInit) => Promise<Response>;

export class PlanServiceError extends Error {
  readonly kind: 'network' | 'timeout' | 'http' | 'payload';
  readonly status?: number;
  readonly reason?: string;

  constructor(kind: PlanServiceError['kind'], message: string, options: { status?: number; reason?: string } = {}) {
    super(message);
    this.name = 'PlanServiceError';
    this.kind = kind;
    this.status = options.status;
    this.reason = options.reason;
  }

  get isStaleMap(): boolean {
    return this.status === 409 && (this.reason ?? '').includes('stale_map');
  }

  /** 协议 §3.4：未知或已淘汰 scene_id（404 not_found / unknown_scene）。 */
  get isUnknownScene(): boolean {
    return this.status === 404 && (this.reason ?? '').includes('unknown_scene');
  }
}

export function resolvePlanApiBase(search: string): string {
  const params = new URLSearchParams(search);
  const override = params.get('planApi');
  if (override && override.length > 0) return override.replace(/\/+$/, '');
  return '/plan-api';
}

/** WebSocket 事件地址：`?planWs=` 覆盖；否则由 HTTP base 推导（http→ws、/plan-api 保留前缀）。 */
export function resolvePlanWsBase(search: string): string | null {
  const params = new URLSearchParams(search);
  const override = params.get('planWs');
  if (override && override.length > 0) return override.replace(/\/+$/, '');
  return null;
}

export function deriveWsUrl(baseUrl: string, path: string, pageLocation?: { protocol: string; host: string }): string | null {
  if (baseUrl.startsWith('http://')) return `ws://${baseUrl.slice('http://'.length)}${path}`;
  if (baseUrl.startsWith('https://')) return `wss://${baseUrl.slice('https://'.length)}${path}`;
  const location = pageLocation ?? (typeof window !== 'undefined' ? window.location : undefined);
  if (!location) return null;
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  return `${scheme}://${location.host}${baseUrl}${path}`;
}

export interface PlanServiceClientOptions {
  base_url?: string;
  ws_base?: string;
  fetch_fn?: FetchLike;
  timeout_ms?: number;
}

export interface PlanEventsHandle {
  close(): void;
}

export type PlanEventsState = 'connecting' | 'open' | 'closed';

export class PlanServiceClient {
  readonly baseUrl: string;
  readonly wsBase: string | null;
  private readonly fetchFn: FetchLike;
  private readonly timeoutMs: number;

  constructor(options: PlanServiceClientOptions = {}) {
    this.baseUrl = (options.base_url ?? '/plan-api').replace(/\/+$/, '');
    this.wsBase = options.ws_base ? options.ws_base.replace(/\/+$/, '') : null;
    this.fetchFn = options.fetch_fn ?? ((input, init) => fetch(input, init));
    this.timeoutMs = options.timeout_ms ?? 5000;
  }

  eventsUrl(): string | null {
    if (this.wsBase) return `${this.wsBase}/plan/events`;
    return deriveWsUrl(this.baseUrl, '/plan/events');
  }

  /**
   * 订阅 /plan/events 推送（WebSocket；job_update/job_result 含单调 job_seq）。
   * 断线自动重连（最多 5 次，间隔 2 s）；调用方通过 close() 停止。
   */
  connectEvents(
    onEvent: (event: PlanEvent) => void,
    onState?: (state: PlanEventsState) => void,
  ): PlanEventsHandle {
    const url = this.eventsUrl();
    if (!url || typeof WebSocket === 'undefined') {
      onState?.('closed');
      return { close: () => {} };
    }
    let socket: WebSocket | null = null;
    let closed = false;
    let timer: ReturnType<typeof setTimeout> | null = null;
    let attempt = 0;
    const connect = () => {
      if (closed) return;
      onState?.('connecting');
      socket = new WebSocket(url);
      socket.onopen = () => {
        attempt = 0;
        onState?.('open');
      };
      socket.onmessage = (message) => {
        try {
          onEvent(parsePlanEvent(JSON.parse(String(message.data))));
        } catch {
          // 忽略非法推送消息（不中断订阅）。
        }
      };
      socket.onclose = () => {
        onState?.('closed');
        if (closed) return;
        attempt += 1;
        if (attempt > 5) return;
        timer = setTimeout(connect, 2000);
      };
      socket.onerror = () => {
        socket?.close();
      };
    };
    connect();
    return {
      close: () => {
        closed = true;
        if (timer) clearTimeout(timer);
        socket?.close();
      },
    };
  }

  health(): Promise<Record<string, unknown>> {
    return this.request('GET', '/health', undefined, (value) => ({ ...(value as Record<string, unknown>) }));
  }

  submit(request: PlanSubmitRequest): Promise<PlanSubmitResponse> {
    return this.request('POST', '/plan/submit', request, parseSubmitResponse);
  }

  cancel(job_id: string): Promise<PlanJobStatus> {
    return this.request('POST', '/plan/cancel', { job_id }, parseJobStatus);
  }

  status(job_id: string): Promise<PlanJobStatus> {
    return this.request('GET', `/plan/status/${encodeURIComponent(job_id)}`, undefined, parseJobStatus);
  }

  result(job_id: string): Promise<PlanResult> {
    return this.request('GET', `/plan/result/${encodeURIComponent(job_id)}`, undefined, parsePlanResult);
  }

  /** 协议 §3.4：拉取该次规划实际使用的输入快照（逐字节一致的 grid_snapshot）。 */
  scene(scene_id: string): Promise<GridLayerData> {
    return this.request(
      'GET',
      `/plan/scene/${encodeURIComponent(scene_id)}`,
      undefined,
      (value) => parsePlanSceneSnapshot(value, scene_id),
    );
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
        throw new PlanServiceError('timeout', `请求超时（${this.timeoutMs} ms）：${url}`);
      }
      throw new PlanServiceError(
        'network',
        `无法连接规划服务：${error instanceof Error ? error.message : String(error)}`,
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
        else if (typeof payload.error === 'string') reason = payload.error;
      } catch {
        // 错误体不可解析时仅保留 HTTP 状态。
      }
      const detail = [statusText, reason].filter((part) => part.length > 0).join(': ');
      throw new PlanServiceError('http', `HTTP ${response.status}${detail ? ` ${detail}` : ''}：${url}`, {
        status: response.status,
        reason: detail,
      });
    }

    let value: unknown;
    try {
      value = await response.json();
    } catch (error) {
      throw new PlanServiceError(
        'payload',
        `响应不是合法 JSON：${error instanceof Error ? error.message : String(error)}`,
      );
    }
    try {
      return parser(value);
    } catch (error) {
      throw new PlanServiceError('payload', error instanceof Error ? error.message : String(error));
    }
  }
}
