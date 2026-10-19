import { describe, expect, it } from 'vitest';
import { makeRectangleObstacle } from '../src/core/scene/obstacles';
import {
  buildScene,
  isSceneEmpty,
  obstaclesBounds,
} from '../src/core/scene/state';
import {
  clearSceneStorage,
  loadSceneFromStorage,
  saveSceneToStorage,
  SCENE_STORAGE_KEY,
  sceneFileName,
  type StorageLike,
} from '../src/core/scene/storage';
import { DEFAULT_ALGORITHM_PARAMS, DEFAULT_VEHICLE_REF } from '../src/core/scene/types';

class FakeStorage implements StorageLike {
  private readonly map = new Map<string, string>();

  getItem(key: string): string | null {
    return this.map.has(key) ? (this.map.get(key) as string) : null;
  }

  setItem(key: string, value: string): void {
    this.map.set(key, value);
  }

  removeItem(key: string): void {
    this.map.delete(key);
  }
}

const MAP = { map_id: 'mapping_example', map_version: 'v'.repeat(64) };

function sampleScene() {
  return buildScene({
    map: MAP,
    vehicle: DEFAULT_VEHICLE_REF,
    start: { x_m: 1, y_m: 2, yaw_rad: 0.1 },
    goal: { x_m: 3, y_m: 4, yaw_rad: -0.2 },
    obstacles: [makeRectangleObstacle('obs-1', 0, 0, 2, 1)],
    algorithmParams: DEFAULT_ALGORITHM_PARAMS,
  });
}

describe('场景 localStorage 往返', () => {
  it('保存 → 读取逐字段一致', () => {
    const storage = new FakeStorage();
    const scene = sampleScene();
    saveSceneToStorage(storage, scene);
    expect(storage.getItem(SCENE_STORAGE_KEY)).toContain('"type": "web_scene"');
    const result = loadSceneFromStorage(storage);
    expect(result.kind).toBe('loaded');
    if (result.kind === 'loaded') {
      expect(result.scene).toEqual(scene);
    }
  });

  it('无记录返回 none；损坏 JSON/非法场景返回 error 不抛出', () => {
    const storage = new FakeStorage();
    expect(loadSceneFromStorage(storage).kind).toBe('none');
    storage.setItem(SCENE_STORAGE_KEY, '{ broken');
    const broken = loadSceneFromStorage(storage);
    expect(broken.kind).toBe('error');
    storage.setItem(SCENE_STORAGE_KEY, JSON.stringify({ schema_version: 2, type: 'web_scene' }));
    expect(loadSceneFromStorage(storage).kind).toBe('error');
  });

  it('清除存储', () => {
    const storage = new FakeStorage();
    saveSceneToStorage(storage, sampleScene());
    clearSceneStorage(storage);
    expect(loadSceneFromStorage(storage).kind).toBe('none');
  });

  it('sceneFileName 过滤特殊字符', () => {
    expect(sceneFileName({ map_id: 'mapping_example' })).toBe('planner_scene_mapping_example.json');
    expect(sceneFileName({ map_id: 'a/b c' })).toBe('planner_scene_a_b_c.json');
  });
});

describe('场景组装与概览（不修改输入）', () => {
  it('buildScene 深拷贝输入（改场景不影响地图/障碍源）', () => {
    const obstacles = [makeRectangleObstacle('obs-1', 0, 0, 2, 1)];
    const map = { ...MAP };
    const scene = buildScene({
      map,
      vehicle: DEFAULT_VEHICLE_REF,
      start: { x_m: 1, y_m: 2, yaw_rad: 0.1 },
      goal: null,
      obstacles,
      algorithmParams: DEFAULT_ALGORITHM_PARAMS,
    });
    scene.obstacles[0] = makeRectangleObstacle('obs-1', 9, 9, 10, 10);
    if (scene.start) scene.start.x_m = 99;
    scene.algorithm_params.corridor.margin_m = 99;
    expect(obstacles[0].x_min_m).toBe(0);
    expect(DEFAULT_ALGORITHM_PARAMS.corridor.margin_m).toBe(0.5);
    expect(map).toEqual(MAP);
  });

  it('isSceneEmpty 判定（障碍/位姿/参数）', () => {
    expect(isSceneEmpty({ start: null, goal: null, obstacles: [], algorithm_params: DEFAULT_ALGORITHM_PARAMS })).toBe(true);
    expect(
      isSceneEmpty({
        start: { x_m: 0, y_m: 0, yaw_rad: 0 },
        goal: null,
        obstacles: [],
        algorithm_params: DEFAULT_ALGORITHM_PARAMS,
      }),
    ).toBe(false);
    expect(
      isSceneEmpty({
        start: null,
        goal: null,
        obstacles: [makeRectangleObstacle('obs-1', 0, 0, 1, 1)],
        algorithm_params: DEFAULT_ALGORITHM_PARAMS,
      }),
    ).toBe(false);
    expect(
      isSceneEmpty({
        start: null,
        goal: null,
        obstacles: [],
        algorithm_params: { ...DEFAULT_ALGORITHM_PARAMS, corridor: { enabled: false, margin_m: 0.5 } },
      }),
    ).toBe(false);
  });

  it('obstaclesBounds 并集（矩形与多边形）', () => {
    expect(obstaclesBounds([])).toBeNull();
    const bounds = obstaclesBounds([
      makeRectangleObstacle('obs-1', 0, 0, 2, 1),
      {
        id: 'obs-2',
        type: 'convex_polygon',
        vertices: [
          { x_m: -3, y_m: 5 },
          { x_m: -1, y_m: 6 },
          { x_m: -2, y_m: 8 },
        ],
      },
    ]);
    expect(bounds).toEqual({ min_x_m: -3, min_y_m: 0, max_x_m: 2, max_y_m: 8 });
  });
});
