import { describe, expect, it } from 'vitest';
import { MapServiceClient, ServiceError, resolveApiBase, type FetchLike } from '../src/core/service/client';

interface RecordedCall {
  url: string;
  init?: RequestInit;
}

function jsonResponse(body: unknown, status = 200): Response {
  return {
    ok: status >= 200 && status < 300,
    status,
    json: async () => body,
  } as unknown as Response;
}

function mockFetch(handler: (url: string) => Response | Promise<Response>): {
  fetchFn: FetchLike;
  calls: RecordedCall[];
} {
  const calls: RecordedCall[] = [];
  const fetchFn: FetchLike = async (url, init) => {
    calls.push({ url, init });
    return handler(url);
  };
  return { fetchFn, calls };
}

const envelope = (type: string, extra: Record<string, unknown>) => ({ schema_version: 1, type, ...extra });

function parseBody(call: RecordedCall | undefined): unknown {
  expect(call?.init?.body).toBeTruthy();
  return JSON.parse(String(call?.init?.body));
}

describe('resolveApiBase', () => {
  it('默认 /api；?api= 覆盖并去除尾部斜杠', () => {
    expect(resolveApiBase('')).toBe('/api');
    expect(resolveApiBase('?map=x')).toBe('/api');
    expect(resolveApiBase('?api=http://127.0.0.1:8765/')).toBe('http://127.0.0.1:8765');
  });
});

describe('MapServiceClient（对齐 planner_map_service 契约）', () => {
  it('构造端点请求：方法、URL 与 JSON 请求体', async () => {
    const { fetchFn, calls } = mockFetch((url) => {
      if (url.includes('/map/info')) {
        return jsonResponse(envelope('map_info', { map_id: 'm', map_version: 'v', counts: {} }));
      }
      if (url.includes('/map/query/point')) {
        return jsonResponse(
          envelope('point_query', { x_m: 1.5, y_m: 2, lanelets: [{ id: '1', attributes: {} }], areas: [] }),
        );
      }
      if (url.includes('/adjacent')) return jsonResponse(envelope('lanelet_relations', { status: 'success', ids: ['9'] }));
      if (url.includes('/following')) return jsonResponse(envelope('lanelet_relations', { status: 'success', ids: ['2'] }));
      if (url.includes('/map/route')) {
        return jsonResponse(envelope('route_result', { status: 'success', lanelet_ids: ['1', '2'], length_m: 5 }));
      }
      if (url.includes('/map/query/window')) {
        return jsonResponse(envelope('map_window', { window_m: { min_x_m: 0, min_y_m: 0, max_x_m: 1, max_y_m: 1 } }));
      }
      if (url.includes('/map/lanelet/')) {
        return jsonResponse(envelope('lanelet', { id: '38992', left: [], right: [] }));
      }
      if (url.includes('/map/area/')) {
        return jsonResponse(envelope('area', { id: '45034', outer: [], inner: [] }));
      }
      throw new Error(`unexpected url ${url}`);
    });
    const client = new MapServiceClient({ base_url: '/api', fetch_fn: fetchFn });

    await client.info();
    await client.queryPoint(1.5, 2);
    await client.lanelet('38992');
    await client.area('45034');
    await client.following('38992');
    await client.preceding('38992');
    await client.adjacent('38992', 'left');
    await client.route('38992', '38994');
    await client.queryWindow({ min_x_m: -1, min_y_m: -2, max_x_m: 3, max_y_m: 4 });

    const urls = calls.map((call) => call.url);
    expect(urls[0]).toBe('/api/map/info');
    expect(urls[1]).toBe('/api/map/query/point');
    expect(urls[2]).toBe('/api/map/lanelet/38992');
    expect(urls[3]).toBe('/api/map/area/45034');
    expect(urls[4]).toBe('/api/map/lanelet/38992/following');
    expect(urls[5]).toBe('/api/map/lanelet/38992/preceding');
    expect(urls[6]).toBe('/api/map/lanelet/38992/adjacent?side=left');
    expect(urls[7]).toBe('/api/map/route');
    expect(urls[8]).toBe('/api/map/query/window');

    expect(calls[0].init?.method).toBe('GET');
    expect(calls[1].init?.method).toBe('POST');
    expect(parseBody(calls[1])).toEqual({ x_m: 1.5, y_m: 2 });
    expect(calls[2].init?.method).toBe('GET');
    expect(calls[7].init?.method).toBe('POST');
    expect(parseBody(calls[7])).toEqual({ from_id: '38992', to_id: '38994' });
    expect(calls[8].init?.method).toBe('POST');
    expect(parseBody(calls[8])).toEqual({ bounds: { min_x_m: -1, min_y_m: -2, max_x_m: 3, max_y_m: 4 } });
    expect((calls[7].init?.headers as Record<string, string>)['Content-Type']).toBe('application/json');
  });

  it('HTTP 错误 → ServiceError(kind=http, status)，携带服务端 status/reason', async () => {
    const { fetchFn } = mockFetch(() =>
      jsonResponse(
        { schema_version: 1, type: 'error', status: 'invalid_input', reason: 'unknown lanelet id: 999' },
        404,
      ),
    );
    const client = new MapServiceClient({ fetch_fn: fetchFn });
    try {
      await client.lanelet('999');
      throw new Error('should have thrown');
    } catch (error) {
      expect(error).toBeInstanceOf(ServiceError);
      const serviceError = error as ServiceError;
      expect(serviceError.kind).toBe('http');
      expect(serviceError.status).toBe(404);
      expect(serviceError.message).toContain('unknown lanelet id: 999');
    }
  });

  it('无结构化错误体时保留 HTTP 状态', async () => {
    const { fetchFn } = mockFetch(() => jsonResponse({ error: 'boom' }, 500));
    const client = new MapServiceClient({ fetch_fn: fetchFn });
    await expect(client.info()).rejects.toMatchObject({ kind: 'http', status: 500 });
  });

  it('网络失败 → ServiceError(kind=network)', async () => {
    const fetchFn: FetchLike = async () => {
      throw new TypeError('fetch failed');
    };
    const client = new MapServiceClient({ fetch_fn: fetchFn });
    await expect(client.info()).rejects.toMatchObject({ kind: 'network' });
  });

  it('非 JSON 响应 → ServiceError(kind=payload)', async () => {
    const fetchFn: FetchLike = async () =>
      ({ ok: true, status: 200, json: async () => Promise.reject(new SyntaxError('bad json')) }) as unknown as Response;
    const client = new MapServiceClient({ fetch_fn: fetchFn });
    await expect(client.info()).rejects.toMatchObject({ kind: 'payload' });
  });

  it('包络非法 → ServiceError(kind=payload) 且保留 JSON 路径', async () => {
    const { fetchFn } = mockFetch(() => jsonResponse({ schema_version: 2, type: 'x' }));
    const client = new MapServiceClient({ fetch_fn: fetchFn });
    try {
      await client.info();
      throw new Error('should have thrown');
    } catch (error) {
      expect(error).toBeInstanceOf(ServiceError);
      expect((error as ServiceError).kind).toBe('payload');
      expect((error as ServiceError).detail).toBe('$.schema_version');
    }
  });

  it('超时 → ServiceError(kind=timeout)', async () => {
    const fetchFn: FetchLike = (_url, init) =>
      new Promise((_resolve, reject) => {
        init?.signal?.addEventListener('abort', () => {
          reject(new DOMException('Aborted', 'AbortError'));
        });
      });
    const client = new MapServiceClient({ fetch_fn: fetchFn, timeout_ms: 20 });
    await expect(client.info()).rejects.toMatchObject({ kind: 'timeout' });
  });

  it('ID 参与 URL 编码', async () => {
    const { fetchFn, calls } = mockFetch(() => jsonResponse(envelope('lanelet', { id: 'a/b', left: [], right: [] })));
    const client = new MapServiceClient({ fetch_fn: fetchFn });
    await client.lanelet('a/b');
    expect(calls[0].url).toBe('/api/map/lanelet/a%2Fb');
  });
});
