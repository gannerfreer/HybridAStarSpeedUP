import { describe, expect, it } from 'vitest';
import { PlanServiceClient, PlanServiceError, type FetchLike } from '../src/core/plan/client';
import { parseJobStatus, parsePlanEvent, parsePlanResult } from '../src/core/plan/parse';
import {
  comparePreviewAndPlanScene,
  createPlanSceneSnapshotState,
  parsePlanSceneSnapshot,
  PlanSceneSnapshotTracker,
} from '../src/core/plan/scene_snapshot';
import { buildGridImageData, buildPlanSceneImageData, GridTextureCache } from '../src/core/grid/texture';
import { GRID_COLORS, PLAN_SCENE_COLORS } from '../src/core/grid/palette';
import type { GridLayerData } from '../src/core/grid/types';

const SCENE_ID = '20821dc86d1519b59d252e17f34a3ccf35dcccb1f7a0f876eedd108718e4e776';
const OTHER_SCENE_ID = 'a'.repeat(64);

function jsonResponse(body: unknown, status = 200): Response {
  return {
    ok: status >= 200 && status < 300,
    status,
    json: async () => body,
  } as unknown as Response;
}

function mockFetch(handler: (url: string) => Response | Promise<Response>): {
  fetchFn: FetchLike;
  calls: string[];
} {
  const calls: string[] = [];
  const fetchFn: FetchLike = async (url) => {
    calls.push(url);
    return handler(url);
  };
  return { fetchFn, calls };
}

function sceneSnapshotBody(sceneId: string, overrides: Record<string, unknown> = {}): Record<string, unknown> {
  return {
    schema_version: 1,
    type: 'grid_snapshot',
    source: 'planning_scene',
    scene_id: sceneId,
    scene_version: 'scene_v1',
    map_id: 'mapping_example',
    map_version: 'v1',
    window: { min_x_m: 0, min_y_m: 0, max_x_m: 0.6, max_y_m: 0.4 },
    requested_resolution_m: 0.2,
    effective_resolution_m: 0.2,
    resolution_m: 0.2,
    degraded: false,
    width: 3,
    height: 2,
    origin: { x_m: 0, y_m: 0, yaw_rad: 0 },
    encoding: 'base64_u8',
    cells: 'AAEAAAAA',
    request_seq: 12,
    budget_cells: 16777216,
    ...overrides,
  };
}

function makeLayer(states: number[], overrides: Partial<GridLayerData> = {}): GridLayerData {
  return {
    kind: 'grid_snapshot',
    source: 'planning_scene',
    map_id: 'mapping_example',
    map_version: 'v1',
    width: 2,
    height: 2,
    resolution_m: 0.5,
    origin: { x_m: 0, y_m: 0, yaw_rad: 0 },
    cells: new Uint8Array(states),
    encoding: 'base64_u8',
    window: { min_x_m: 0, min_y_m: 0, max_x_m: 1, max_y_m: 1 },
    requested_resolution_m: 0.5,
    effective_resolution_m: 0.5,
    degraded: false,
    scene_id: SCENE_ID,
    scene_version: 'scene_v1',
    request_seq: 12,
    budget_cells: null,
    notes: null,
    ...overrides,
  };
}

describe('规划结果 scene_id/scene_version 解析（协议 §3.4）', () => {
  it('status/result/event 携带 64 位十六进制 scene_id 时解析', () => {
    const status = parseJobStatus({ job_id: 'job-1', status: 'running', stage: 'hybrid', scene_id: SCENE_ID, scene_version: 'scene_v1' });
    expect(status.scene_id).toBe(SCENE_ID);
    expect(status.scene_version).toBe('scene_v1');

    const result = parsePlanResult({
      job_id: 'job-1',
      status: 'succeeded',
      map_id: 'm',
      map_version: 'v1',
      scene_id: SCENE_ID,
      scene_version: 'scene_v1',
      statistics: {},
    });
    expect(result.scene_id).toBe(SCENE_ID);
    expect(result.scene_version).toBe('scene_v1');

    const event = parsePlanEvent({ kind: 'job_update', job_seq: 1, job_id: 'job-1', status: 'running', scene_id: SCENE_ID });
    expect(event.scene_id).toBe(SCENE_ID);
  });

  it('缺省容忍（旧响应）；非法格式拒绝', () => {
    expect(parseJobStatus({ job_id: 'job-1', status: 'queued' }).scene_id).toBeUndefined();
    expect(parsePlanResult({ status: 'succeeded', map_id: 'm', map_version: 'v1', statistics: {} }).scene_id).toBeUndefined();
    expect(() => parseJobStatus({ job_id: 'job-1', status: 'running', scene_id: 'XYZ' })).toThrow(/scene_id/);
    expect(() => parsePlanResult({ status: 'succeeded', map_id: 'm', map_version: 'v1', scene_id: 'A'.repeat(64) })).toThrow(
      /scene_id/,
    );
  });
});

describe('PlanServiceClient.scene（GET /plan/scene/{scene_id}）', () => {
  it('端点、解析与 scene_id 一致性校验', async () => {
    const { fetchFn, calls } = mockFetch(() => jsonResponse(sceneSnapshotBody(SCENE_ID)));
    const client = new PlanServiceClient({ base_url: '/plan-api', fetch_fn: fetchFn });
    const layer = await client.scene(SCENE_ID);
    expect(calls).toEqual([`/plan-api/plan/scene/${SCENE_ID}`]);
    expect(layer.source).toBe('planning_scene');
    expect(layer.scene_id).toBe(SCENE_ID);
    expect(layer.scene_version).toBe('scene_v1');
  });

  it('响应 scene_id 与请求不一致 → payload 错误', async () => {
    const { fetchFn } = mockFetch(() => jsonResponse(sceneSnapshotBody(OTHER_SCENE_ID)));
    const client = new PlanServiceClient({ fetch_fn: fetchFn });
    await expect(client.scene(SCENE_ID)).rejects.toMatchObject({ kind: 'payload' });
  });

  it('404 unknown_scene → isUnknownScene；来源错误拒绝', async () => {
    const { fetchFn } = mockFetch(() =>
      jsonResponse({ schema_version: 1, type: 'error', status: 'not_found', reason: 'unknown_scene' }, 404),
    );
    const client = new PlanServiceClient({ fetch_fn: fetchFn });
    try {
      await client.scene(SCENE_ID);
      throw new Error('should have thrown');
    } catch (error) {
      expect(error).toBeInstanceOf(PlanServiceError);
      const planError = error as PlanServiceError;
      expect(planError.status).toBe(404);
      expect(planError.isUnknownScene).toBe(true);
    }

    const wrongSource = new PlanServiceClient({
      fetch_fn: mockFetch(() => jsonResponse(sceneSnapshotBody(SCENE_ID, { source: 'driveable', scene_id: undefined, scene_version: undefined }))).fetchFn,
    });
    await expect(wrongSource.scene(SCENE_ID)).rejects.toMatchObject({ kind: 'payload' });
  });

  it('parsePlanSceneSnapshot：grid_view/缺 scene_id/格式非法均拒绝', () => {
    expect(() =>
      parsePlanSceneSnapshot(
        {
          schema_version: 1,
          type: 'grid_view',
          map_id: 'm',
          map_version: 'v1',
          width: 1,
          height: 1,
          resolution_m: 1,
          origin: { x_m: 0, y_m: 0, yaw_rad: 0 },
          cells: [0],
        },
        SCENE_ID,
      ),
    ).toThrow(/grid_snapshot/);
    expect(() => parsePlanSceneSnapshot(sceneSnapshotBody('not-hex'), 'not-hex')).toThrow(/scene_id/);
  });
});

describe('规划快照陈旧防护与预览区分', () => {
  it('PlanSceneSnapshotTracker：只接受最新请求的 scene_id', () => {
    const tracker = new PlanSceneSnapshotTracker();
    tracker.request(OTHER_SCENE_ID);
    tracker.request(SCENE_ID);
    expect(tracker.accept(SCENE_ID)).toBe(true);
    expect(tracker.accept(OTHER_SCENE_ID)).toBe(false);
    expect(tracker.stale_ignored).toBe(1);
    expect(tracker.latest_scene_id).toBe(SCENE_ID);
  });

  it('历史场景不覆盖预览：状态独立保存，scene_id 显式比较', () => {
    const preview = makeLayer([0, 0, 0, 0], { scene_id: OTHER_SCENE_ID, request_seq: 1 });
    const plan = makeLayer([0, 1, 0, 0], { scene_id: SCENE_ID });
    const state = createPlanSceneSnapshotState();
    // 规划快照状态与预览图层是独立对象：拉取规划快照不修改预览对象。
    const nextState = { ...state, status: 'ready' as const, scene_id: SCENE_ID, layer: plan };
    expect(nextState.layer).toBe(plan);
    expect(preview.scene_id).toBe(OTHER_SCENE_ID);
    expect(comparePreviewAndPlanScene(preview.scene_id, nextState.scene_id)).toBe('different');
    expect(comparePreviewAndPlanScene(SCENE_ID, SCENE_ID)).toBe('same');
    expect(comparePreviewAndPlanScene(null, SCENE_ID)).toBe('no_preview');
  });
});

describe('规划实际场景叠加纹理（与预览显式区分）', () => {
  it('buildPlanSceneImageData：空闲透明、占据品红、未知琥珀', () => {
    const image = buildPlanSceneImageData(makeLayer([0, 1, 2, 0]));
    expect(image.data.length).toBe(16);
    expect([image.data[0], image.data[1], image.data[2], image.data[3]]).toEqual([0, 0, 0, 0]);
    expect(image.data[4]).toBe(PLAN_SCENE_COLORS[1].r);
    expect(image.data[8]).toBe(PLAN_SCENE_COLORS[2].r);
    // 与三态预览表不同（占据颜色不同）。
    expect(PLAN_SCENE_COLORS[1].r).not.toBe(GRID_COLORS[1].r);
  });

  it('GridTextureCache 可注入规划场景纹理生成器（缓存语义不变）', () => {
    const log: Array<[number, number]> = [];
    const factory = (width: number, height: number) => {
      log.push([width, height]);
      return {
        width,
        height,
        getContext() {
          return { putImageData: () => {} };
        },
      };
    };
    const cache = new GridTextureCache(factory, buildPlanSceneImageData);
    const layer = makeLayer([0, 1, 0, 0]);
    cache.get(layer);
    expect(cache.rebuildCount).toBe(1);
    cache.get(layer);
    expect(cache.rebuildCount).toBe(1);
    // 默认生成器仍输出三态颜色（兼容性）。
    expect(buildGridImageData(makeLayer([0, 1, 0, 0])).data[4]).toBe(GRID_COLORS[1].r);
  });
});
