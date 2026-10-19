import type { BoundsM } from '../types';
import {
  ServicePayloadError,
  parseAreaGeometry,
  parseCropResult,
  parseIdList,
  parseLaneletGeometry,
  parseMapInfo,
  parsePointQuery,
  parseRouteResult,
} from './parse';
import type {
  AdjacentSide,
  AreaGeometry,
  CropResult,
  IdListResult,
  LaneletGeometry,
  MapInfoPayload,
  PointQueryResult,
  RouteResult,
} from './types';

/** 可注入的 fetch（测试注入 mock；默认全局 fetch）。 */
export type FetchLike = (input: string, init?: RequestInit) => Promise<Response>;

export type ServiceErrorKind = 'network' | 'timeout' | 'http' | 'payload';

export class ServiceError extends Error {
  readonly kind: ServiceErrorKind;
  readonly status?: number;
  readonly detail?: string;

  constructor(kind: ServiceErrorKind, message: string, options: { status?: number; detail?: string } = {}) {
    super(message);
    this.name = 'ServiceError';
    this.kind = kind;
    this.status = options.status;
    this.detail = options.detail;
  }
}

/**
 * API 基地址：默认 `/api`（Vite dev 代理 / CDP 代理 → http://127.0.0.1:8765）；
 * `?api=<url>` 可覆盖（例如直连服务且服务提供 CORS 时）。
 */
export function resolveApiBase(search: string): string {
  const params = new URLSearchParams(search);
  const override = params.get('api');
  if (override && override.length > 0) return override.replace(/\/+$/, '');
  return '/api';
}

export interface MapServiceClientOptions {
  base_url?: string;
  fetch_fn?: FetchLike;
  timeout_ms?: number;
}

/**
 * 本地地图服务客户端（planner_map_service，见 Src/map/src/service_main.cpp）。
 * 每个方法：构造请求 → fetch（带超时）→ 包络校验与解析；失败统一抛出
 * `ServiceError`（kind 区分网络/超时/HTTP/载荷；HTTP 错误尽量携带服务端 reason）。
 */
export class MapServiceClient {
  readonly baseUrl: string;
  private readonly fetchFn: FetchLike;
  private readonly timeoutMs: number;

  constructor(options: MapServiceClientOptions = {}) {
    this.baseUrl = (options.base_url ?? '/api').replace(/\/+$/, '');
    this.fetchFn = options.fetch_fn ?? ((input, init) => fetch(input, init));
    this.timeoutMs = options.timeout_ms ?? 5000;
  }

  info(): Promise<MapInfoPayload> {
    return this.request('GET', '/map/info', undefined, undefined, parseMapInfo);
  }

  /** `/health` 原始字段（X-W2：grid_protocol_version / scene_rules_version）。 */
  health(): Promise<Record<string, unknown>> {
    return this.request('GET', '/health', undefined, undefined, (value) => {
      if (typeof value !== 'object' || value === null || Array.isArray(value)) {
        throw new ServicePayloadError('$', 'health 响应必须是对象');
      }
      return { ...(value as Record<string, unknown>) };
    });
  }

  queryPoint(x_m: number, y_m: number): Promise<PointQueryResult> {
    return this.request('POST', '/map/query/point', undefined, { x_m, y_m }, parsePointQuery);
  }

  lanelet(id: string): Promise<LaneletGeometry> {
    return this.request('GET', `/map/lanelet/${encodeURIComponent(id)}`, undefined, undefined, parseLaneletGeometry);
  }

  area(id: string): Promise<AreaGeometry> {
    return this.request('GET', `/map/area/${encodeURIComponent(id)}`, undefined, undefined, parseAreaGeometry);
  }

  following(id: string): Promise<IdListResult> {
    return this.request('GET', `/map/lanelet/${encodeURIComponent(id)}/following`, undefined, undefined, parseIdList);
  }

  preceding(id: string): Promise<IdListResult> {
    return this.request('GET', `/map/lanelet/${encodeURIComponent(id)}/preceding`, undefined, undefined, parseIdList);
  }

  adjacent(id: string, side: AdjacentSide): Promise<IdListResult> {
    return this.request(
      'GET',
      `/map/lanelet/${encodeURIComponent(id)}/adjacent`,
      { side },
      undefined,
      parseIdList,
    );
  }

  route(from: string, to: string): Promise<RouteResult> {
    return this.request('POST', '/map/route', undefined, { from_id: from, to_id: to }, parseRouteResult);
  }

  queryWindow(bounds: BoundsM): Promise<CropResult> {
    return this.request('POST', '/map/query/window', undefined, { bounds }, parseCropResult);
  }

  private async request<T>(
    method: 'GET' | 'POST',
    path: string,
    params: Record<string, string> | undefined,
    body: unknown,
    parser: (value: unknown) => T,
  ): Promise<T> {
    const query = params ? `?${new URLSearchParams(params).toString()}` : '';
    const url = `${this.baseUrl}${path}${query}`;
    const controller = new AbortController();
    const timer = setTimeout(() => controller.abort(), this.timeoutMs);
    const init: RequestInit = {
      method,
      signal: controller.signal,
      headers: {
        Accept: 'application/json',
        ...(body !== undefined ? { 'Content-Type': 'application/json' } : {}),
      },
      ...(body !== undefined ? { body: JSON.stringify(body) } : {}),
    };
    let response: Response;
    try {
      response = await this.fetchFn(url, init);
    } catch (error) {
      if (controller.signal.aborted) {
        throw new ServiceError('timeout', `请求超时（${this.timeoutMs} ms）：${url}`);
      }
      throw new ServiceError(
        'network',
        `无法连接地图服务：${error instanceof Error ? error.message : String(error)}`,
      );
    } finally {
      clearTimeout(timer);
    }

    if (!response.ok) {
      let reason = '';
      let status = '';
      try {
        const payload = (await response.json()) as Record<string, unknown>;
        if (typeof payload.reason === 'string') reason = payload.reason;
        if (typeof payload.status === 'string') status = payload.status;
      } catch {
        // 错误体不可解析时仅保留 HTTP 状态。
      }
      const detail = [status, reason].filter((part) => part.length > 0).join(': ');
      throw new ServiceError(
        'http',
        `HTTP ${response.status}${detail ? ` ${detail}` : ''}：${url}`,
        { status: response.status },
      );
    }

    let value: unknown;
    try {
      value = await response.json();
    } catch (error) {
      throw new ServiceError(
        'payload',
        `响应不是合法 JSON：${error instanceof Error ? error.message : String(error)}`,
      );
    }

    try {
      return parser(value);
    } catch (error) {
      if (error instanceof ServicePayloadError) {
        throw new ServiceError('payload', error.message, { detail: error.path });
      }
      throw error;
    }
  }
}
