import { describe, expect, it } from 'vitest';
import { PlanServiceClient, PlanServiceError, resolvePlanApiBase, type FetchLike } from '../src/core/plan/client';

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

const request = {
  request_id: 'req-1',
  map_id: 'mapping_example',
  map_version: 'v1',
  start: { x_m: 0, y_m: 0, yaw_rad: 0 },
  goal: { x_m: 10, y_m: 0, yaw_rad: 0 },
  vehicle_ref: 'default_test_vehicle',
  obstacles: [],
  algorithm_params: { hybrid_backend: 'kth_normalized' },
};

describe('PlanServiceClient', () => {
  it('端点与方法/请求体', async () => {
    const { fetchFn, calls } = mockFetch((url) => {
      if (url.endsWith('/health')) return jsonResponse({ status: 'ok' });
      if (url.endsWith('/plan/submit')) {
        return jsonResponse({ job_id: 'job-1', job_seq: 1, request_id: 'req-1', map_id: 'm', map_version: 'v1', status: 'queued' });
      }
      if (url.endsWith('/plan/cancel')) return jsonResponse({ job_id: 'job-1', status: 'cancelled' });
      if (url.includes('/plan/status/')) return jsonResponse({ job_id: 'job-1', status: 'running', stage: 'hybrid' });
      if (url.includes('/plan/result/')) return jsonResponse({ job_id: 'job-1', status: 'success', statistics: {} });
      throw new Error(`unexpected ${url}`);
    });
    const client = new PlanServiceClient({ base_url: '/plan-api', fetch_fn: fetchFn });

    await client.health();
    await client.submit(request);
    await client.cancel('job-1');
    await client.status('job-1');
    await client.result('job-1');

    expect(calls.map((call) => call.url)).toEqual([
      '/plan-api/health',
      '/plan-api/plan/submit',
      '/plan-api/plan/cancel',
      '/plan-api/plan/status/job-1',
      '/plan-api/plan/result/job-1',
    ]);
    expect(calls[1].init?.method).toBe('POST');
    expect(JSON.parse(String(calls[1].init?.body))).toEqual(request);
    expect(calls[2].init?.method).toBe('POST');
    expect(JSON.parse(String(calls[2].init?.body))).toEqual({ job_id: 'job-1' });
    expect(calls[3].init?.method).toBe('GET');
  });

  it('stale_map：409 与原因可识别（isStaleMap）', async () => {
    const { fetchFn } = mockFetch(() =>
      jsonResponse({ status: 'stale_map', reason: 'map_version mismatch: v0 != v1' }, 409),
    );
    const client = new PlanServiceClient({ fetch_fn: fetchFn });
    try {
      await client.submit(request);
      throw new Error('should have thrown');
    } catch (error) {
      expect(error).toBeInstanceOf(PlanServiceError);
      const planError = error as PlanServiceError;
      expect(planError.status).toBe(409);
      expect(planError.isStaleMap).toBe(true);
      expect(planError.reason).toMatch(/stale_map/);
    }
  });

  it('其他 HTTP 错误/网络/超时/非 JSON', async () => {
    const { fetchFn: http } = mockFetch(() => jsonResponse({ reason: 'invalid input: missing start' }, 400));
    await expect(new PlanServiceClient({ fetch_fn: http }).submit(request)).rejects.toMatchObject({ kind: 'http', status: 400 });

    const network: FetchLike = async () => {
      throw new TypeError('fetch failed');
    };
    await expect(new PlanServiceClient({ fetch_fn: network }).health()).rejects.toMatchObject({ kind: 'network' });

    const timeout: FetchLike = (_url, init) =>
      new Promise((_resolve, reject) => {
        init?.signal?.addEventListener('abort', () => reject(new DOMException('Aborted', 'AbortError')));
      });
    await expect(new PlanServiceClient({ fetch_fn: timeout, timeout_ms: 20 }).health()).rejects.toMatchObject({ kind: 'timeout' });

    const badJson: FetchLike = async () =>
      ({ ok: true, status: 200, json: async () => Promise.reject(new SyntaxError('bad')) }) as unknown as Response;
    await expect(new PlanServiceClient({ fetch_fn: badJson }).health()).rejects.toMatchObject({ kind: 'payload' });
  });

  it('resolvePlanApiBase：默认 /plan-api；?planApi= 覆盖', () => {
    expect(resolvePlanApiBase('')).toBe('/plan-api');
    expect(resolvePlanApiBase('?planApi=http://127.0.0.1:8766/')).toBe('http://127.0.0.1:8766');
  });
});
