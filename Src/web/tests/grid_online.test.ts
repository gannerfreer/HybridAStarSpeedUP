import { describe, expect, it } from 'vitest';
import {
  acceptOnlineGridResponse,
  cellCountFor,
  checkGridVersionConsistency,
  fitDisplayResolution,
  GRID_DISPLAY_MAX_CELLS,
  GridSourceTracker,
  intersectBounds,
  OnlineGridClient,
  OnlineGridError,
  parseGridHealth,
  protocolMajor,
  viewportWindow,
  type GridHealthInfo,
  type OnlineFetchLike,
} from '../src/core/grid/online';
import type { BoundsM } from '../src/core/types';
import type { Viewport } from '../src/core/transform';

function jsonResponse(body: unknown, status = 200): Response {
  const text = JSON.stringify(body);
  return {
    ok: status >= 200 && status < 300,
    status,
    text: async () => text,
    json: async () => JSON.parse(text),
  } as unknown as Response;
}

function mockFetch(handler: (url: string, init?: RequestInit) => Response | Promise<Response>): {
  fetchFn: OnlineFetchLike;
  calls: Array<{ url: string; init?: RequestInit }>;
} {
  const calls: Array<{ url: string; init?: RequestInit }> = [];
  const fetchFn: OnlineFetchLike = async (url, init) => {
    calls.push({ url, init });
    return handler(url, init);
  };
  return { fetchFn, calls };
}

const windowM: BoundsM = { min_x_m: 0, min_y_m: 0, max_x_m: 0.3, max_y_m: 0.2 };

function snapshotBody(overrides: Record<string, unknown> = {}): Record<string, unknown> {
  return {
    schema_version: 1,
    type: 'grid_snapshot',
    source: 'driveable',
    map_id: 'mapping_example',
    map_version: 'v1',
    window: windowM,
    requested_resolution_m: 0.1,
    effective_resolution_m: 0.1,
    resolution_m: 0.1,
    degraded: false,
    width: 3,
    height: 2,
    origin: { x_m: 0, y_m: 0, yaw_rad: 0 },
    encoding: 'base64_u8',
    cells: 'AAEAAAAA',
    request_seq: 1,
    budget_cells: 16777216,
    ...overrides,
  };
}

describe('栅格协议版本一致性（协议 §3.5）', () => {
  it('protocolMajor：合法 "<major>.<minor>" 解析，其余 null', () => {
    expect(protocolMajor('1.0')).toBe(1);
    expect(protocolMajor('2.3')).toBe(2);
    expect(protocolMajor('1')).toBeNull();
    expect(protocolMajor('1.0.0')).toBeNull();
    expect(protocolMajor('')).toBeNull();
    expect(protocolMajor(null)).toBeNull();
  });

  it('1.0 + scene_v1 通过；主版本 2 / 规则 v2 / 缺失字段均不通过', () => {
    const ok = checkGridVersionConsistency({
      map_id: 'm',
      map_version: 'v1',
      grid_protocol_version: '1.0',
      scene_rules_version: 'scene_v1',
    });
    expect(ok.protocol_ok).toBe(true);
    expect(ok.scene_rules_ok).toBe(true);
    expect(ok.reason).toBe('');

    const major2 = checkGridVersionConsistency({
      map_id: 'm',
      map_version: 'v1',
      grid_protocol_version: '2.0',
      scene_rules_version: 'scene_v1',
    });
    expect(major2.protocol_ok).toBe(false);
    expect(major2.scene_rules_ok).toBe(true);
    expect(major2.reason).toMatch(/协议主版本不匹配/);

    const rules2 = checkGridVersionConsistency({
      map_id: 'm',
      map_version: 'v1',
      grid_protocol_version: '1.0',
      scene_rules_version: 'scene_v2',
    });
    expect(rules2.protocol_ok).toBe(true);
    expect(rules2.scene_rules_ok).toBe(false);
    expect(rules2.reason).toMatch(/场景组合规则版本不匹配/);

    const missing = checkGridVersionConsistency(null);
    expect(missing.protocol_ok).toBe(false);
    expect(missing.scene_rules_ok).toBe(false);
    expect(missing.reason).toMatch(/未声明 grid_protocol_version/);
  });

  it('parseGridHealth：包络校验与字段提取', () => {
    const parsed = parseGridHealth({
      schema_version: 1,
      type: 'health',
      status: 'ok',
      map_id: 'mapping_example',
      map_version: 'v1',
      grid_protocol_version: '1.0',
      scene_rules_version: 'scene_v1',
    });
    expect(parsed).toEqual({
      map_id: 'mapping_example',
      map_version: 'v1',
      grid_protocol_version: '1.0',
      scene_rules_version: 'scene_v1',
    });
    expect(() => parseGridHealth({ schema_version: 2 })).toThrow(/schema_version/);
    expect(() => parseGridHealth({ type: 'map_snapshot' })).toThrow(/type/);
    expect(() => parseGridHealth(null)).toThrow(/对象/);
    // 缺失版本字段按 null 返回（由一致性检查禁用）。
    const partial = parseGridHealth({ schema_version: 1, status: 'ok' });
    expect(partial.grid_protocol_version).toBeNull();
  });
});

describe('在线请求窗口与显示上限（≤1024²）', () => {
  it('viewportWindow：视口与地图边界求交', () => {
    const viewport: Viewport = {
      width_px: 100,
      height_px: 100,
      scale_px_per_m: 1,
      center_x_m: 0,
      center_y_m: 0,
    };
    expect(viewportWindow(viewport, { min_x_m: -10, min_y_m: -10, max_x_m: 10, max_y_m: 10 })).toEqual({
      min_x_m: -10,
      min_y_m: -10,
      max_x_m: 10,
      max_y_m: 10,
    });
    // 视口完全在地图外 → null
    expect(viewportWindow(viewport, { min_x_m: 100, min_y_m: 100, max_x_m: 200, max_y_m: 200 })).toBeNull();
    expect(intersectBounds(windowM, { min_x_m: 1, min_y_m: 1, max_x_m: 2, max_y_m: 2 })).toBeNull();
  });

  it('cellCountFor 与服务公式一致（ceil×ceil）', () => {
    expect(cellCountFor({ min_x_m: 0, min_y_m: 0, max_x_m: 0.25, max_y_m: 0.25 }, 0.1)).toBe(9);
    expect(cellCountFor({ min_x_m: 0, min_y_m: 0, max_x_m: 0.3, max_y_m: 0.2 }, 0.1)).toBe(6);
  });

  it('fitDisplayResolution：超限按 2 的幂提高请求分辨率并标注 adjusted', () => {
    const large: BoundsM = { min_x_m: 0, min_y_m: 0, max_x_m: 100000, max_y_m: 100000 };
    const fit = fitDisplayResolution(large, 0.1);
    expect(fit.adjusted).toBe(true);
    expect(fit.factor).toBeGreaterThan(1);
    expect(fit.resolution_m).toBeCloseTo(0.1 * fit.factor, 9);
    expect(fit.cells).toBeLessThanOrEqual(GRID_DISPLAY_MAX_CELLS);
    expect(fit.within).toBe(true);

    const small = fitDisplayResolution(windowM, 0.1);
    expect(small.adjusted).toBe(false);
    expect(small.factor).toBe(1);
    expect(small.resolution_m).toBe(0.1);
  });
});

describe('每来源 request_seq 陈旧防护（协议 §6）', () => {
  it('begin 单调、stale/anomaly 计数、缺失回显按 stale 处理', () => {
    const tracker = new GridSourceTracker();
    const seq1 = tracker.begin();
    const seq2 = tracker.begin();
    expect(tracker.latest_seq).toBe(2);
    expect(tracker.accept(seq2)).toBe('apply');
    expect(tracker.accept(seq1)).toBe('stale');
    expect(tracker.stale_ignored).toBe(1);
    expect(tracker.accept(seq2 + 1)).toBe('anomaly');
    expect(tracker.anomaly_count).toBe(1);
    // 缺失回显且已有更晚请求 → 陈旧（seq < latest 分支）
    expect(tracker.accept(seq1, null)).toBe('stale');
    // 回显与请求不符 → anomaly
    expect(tracker.accept(seq2, seq1)).toBe('anomaly');
  });

  it('每来源独立：driveable 的 seq 不影响 planning_scene', () => {
    const driveable = new GridSourceTracker();
    const preview = new GridSourceTracker();
    const d1 = driveable.begin();
    const p1 = preview.begin();
    expect(driveable.accept(d1)).toBe('apply');
    expect(preview.accept(p1)).toBe('apply');
    expect(driveable.latest_seq).toBe(1);
    expect(preview.latest_seq).toBe(1);
  });

  it('晚到响应不覆盖新请求：B(2) 先应用、A(1) 后到丢弃', () => {
    const tracker = new GridSourceTracker();
    const seqA = tracker.begin();
    const seqB = tracker.begin();
    const layerA = { source: 'driveable', map_id: 'm', map_version: 'v1' };
    const layerB = { source: 'driveable', map_id: 'm', map_version: 'v1' };
    const arrivals = [
      { seq: seqB, echoed: seqB, layer: layerB },
      { seq: seqA, echoed: seqA, layer: layerA },
    ];
    const applied: unknown[] = [];
    for (const arrival of arrivals) {
      const outcome = acceptOnlineGridResponse({
        tracker,
        seq: arrival.seq,
        echoed_seq: arrival.echoed,
        source: 'driveable',
        layer: arrival.layer as never,
        map_id: 'm',
        map_version: 'v1',
      });
      if (outcome.apply) applied.push(arrival.layer);
    }
    expect(applied).toEqual([layerB]);
    expect(tracker.stale_ignored).toBe(1);
  });

  it('地图版本不一致一律丢弃并计数；来源不匹配按异常', () => {
    const tracker = new GridSourceTracker();
    const seq = tracker.begin();
    const mismatch = acceptOnlineGridResponse({
      tracker,
      seq,
      echoed_seq: seq,
      source: 'driveable',
      layer: { source: 'driveable', map_id: 'other', map_version: 'v0' } as never,
      map_id: 'm',
      map_version: 'v1',
    });
    expect(mismatch.decision).toBe('map_mismatch');
    expect(mismatch.apply).toBe(false);
    expect(tracker.map_mismatch_ignored).toBe(1);

    const seq2 = tracker.begin();
    const wrongSource = acceptOnlineGridResponse({
      tracker,
      seq: seq2,
      echoed_seq: seq2,
      source: 'planning_scene',
      layer: { source: 'driveable', map_id: 'm', map_version: 'v1' } as never,
      map_id: 'm',
      map_version: 'v1',
    });
    expect(wrongSource.decision).toBe('anomaly');
    expect(wrongSource.apply).toBe(false);
  });
});

describe('OnlineGridClient（/map/grid/driveable 与 /map/scene/preview）', () => {
  it('端点、请求体与 request_seq 回显', async () => {
    const { fetchFn, calls } = mockFetch((url) => {
      if (url.endsWith('/health')) {
        return jsonResponse({
          schema_version: 1,
          type: 'health',
          status: 'ok',
          grid_protocol_version: '1.0',
          scene_rules_version: 'scene_v1',
        });
      }
      if (url.endsWith('/map/grid/driveable')) return jsonResponse(snapshotBody({ request_seq: 7 }));
      if (url.endsWith('/map/scene/preview')) {
        return jsonResponse(
          snapshotBody({
            source: 'planning_scene',
            scene_id: 'a'.repeat(64),
            scene_version: 'scene_v1',
            request_seq: 8,
          }),
        );
      }
      throw new Error(`unexpected ${url}`);
    });
    const client = new OnlineGridClient({ base_url: '/api', fetch_fn: fetchFn });
    const health = await client.health();
    expect(health.grid_protocol_version).toBe('1.0');

    const driveable = await client.driveable({ window: windowM, resolution_m: 0.1 }, 7);
    expect(driveable.layer.source).toBe('driveable');
    expect(driveable.layer.request_seq).toBe(7);
    expect(driveable.bytes).toBeGreaterThan(0);
    expect(driveable.raw_text).toContain('"request_seq":7');

    const preview = await client.preview({ window: windowM, resolution_m: 0.1, obstacles: [] }, 8);
    expect(preview.layer.source).toBe('planning_scene');
    expect(preview.layer.scene_id).toBe('a'.repeat(64));

    expect(calls.map((call) => call.url)).toEqual([
      '/api/health',
      '/api/map/grid/driveable',
      '/api/map/scene/preview',
    ]);
    expect(calls[1].init?.method).toBe('POST');
    expect(JSON.parse(String(calls[1].init?.body))).toEqual({
      window: windowM,
      resolution_m: 0.1,
      request_seq: 7,
    });
    expect(JSON.parse(String(calls[2].init?.body))).toEqual({
      window: windowM,
      resolution_m: 0.1,
      obstacles: [],
      request_seq: 8,
    });
  });

  it('来源不匹配 / planning_scene 缺 scene_id / 非 grid_snapshot 均拒绝', async () => {
    const wrongSource = new OnlineGridClient({
      fetch_fn: mockFetch(() =>
        jsonResponse(snapshotBody({ source: 'planning_scene', scene_id: 'b'.repeat(64), scene_version: 'scene_v1' })),
      ).fetchFn,
    });
    await expect(wrongSource.driveable({ window: windowM, resolution_m: 0.1 }, 1)).rejects.toMatchObject({
      kind: 'payload',
    });

    const missingScene = new OnlineGridClient({
      fetch_fn: mockFetch(() => jsonResponse(snapshotBody({ source: 'planning_scene' }))).fetchFn,
    });
    await expect(missingScene.preview({ window: windowM, resolution_m: 0.1 }, 1)).rejects.toMatchObject({
      kind: 'payload',
    });

    const gridView = new OnlineGridClient({
      fetch_fn: mockFetch(() =>
        jsonResponse({
          schema_version: 1,
          type: 'grid_view',
          map_id: 'm',
          map_version: 'v1',
          width: 1,
          height: 1,
          resolution_m: 1,
          origin: { x_m: 0, y_m: 0, yaw_rad: 0 },
          cells: [0],
        }),
      ).fetchFn,
    });
    await expect(gridView.driveable({ window: windowM, resolution_m: 0.1 }, 1)).rejects.toMatchObject({
      kind: 'payload',
    });
  });

  it('HTTP 错误携带服务端 status/reason；网络/超时/非 JSON 分类', async () => {
    const http = new OnlineGridClient({
      fetch_fn: mockFetch(() =>
        jsonResponse({ schema_version: 1, type: 'error', status: 'invalid_input', reason: 'empty_window' }, 400),
      ).fetchFn,
    });
    await expect(http.driveable({ window: windowM, resolution_m: 0.1 }, 1)).rejects.toMatchObject({
      kind: 'http',
      status: 400,
    });

    const network: OnlineFetchLike = async () => {
      throw new TypeError('fetch failed');
    };
    await expect(new OnlineGridClient({ fetch_fn: network }).health()).rejects.toMatchObject({ kind: 'network' });

    const timeout: OnlineFetchLike = (_url, init) =>
      new Promise((_resolve, reject) => {
        init?.signal?.addEventListener('abort', () => reject(new DOMException('Aborted', 'AbortError')));
      });
    await expect(new OnlineGridClient({ fetch_fn: timeout, timeout_ms: 20 }).health()).rejects.toMatchObject({
      kind: 'timeout',
    });

    const badJson = new OnlineGridClient({
      fetch_fn: mockFetch(() => ({ ok: true, status: 200, text: async () => '<html>' }) as unknown as Response).fetchFn,
    });
    await expect(badJson.health()).rejects.toMatchObject({ kind: 'payload' });
  });

  it('OnlineGridError.isStaleMap：409 stale_map 可识别', async () => {
    const client = new OnlineGridClient({
      fetch_fn: mockFetch(() => jsonResponse({ status: 'stale_map', reason: 'map_version mismatch' }, 409)).fetchFn,
    });
    try {
      await client.driveable({ window: windowM, resolution_m: 0.1 }, 1);
      throw new Error('should have thrown');
    } catch (error) {
      expect(error).toBeInstanceOf(OnlineGridError);
      expect((error as OnlineGridError).isStaleMap).toBe(true);
    }
  });

  it('健康信息可用于版本禁用（模拟 2.0 服务）', () => {
    const health: GridHealthInfo = {
      map_id: 'm',
      map_version: 'v1',
      grid_protocol_version: '2.0',
      scene_rules_version: 'scene_v1',
    };
    expect(checkGridVersionConsistency(health).protocol_ok).toBe(false);
  });
});
