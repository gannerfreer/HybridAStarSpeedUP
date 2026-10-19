import { describe, expect, it } from 'vitest';
import { checkSceneMap, parseScene, SceneParseError, sceneFromJson, serializeScene } from '../src/core/scene/parse';
import type { WebScene } from '../src/core/scene/types';

function validScene(): Record<string, unknown> {
  return {
    schema_version: 1,
    type: 'web_scene',
    name: 'unit test scene',
    map_id: 'mapping_example',
    map_version: 'a'.repeat(64),
    vehicle: { ref: 'default_test_vehicle', path: 'tests/fixtures/v3/vehicles/default.json' },
    start: { x_m: 1, y_m: 2, yaw_rad: 0.5 },
    goal: { x_m: 3, y_m: 4, yaw_rad: -0.5 },
    obstacles: [
      { id: 'obs-1', type: 'rectangle', x_min_m: 0, y_min_m: 0, x_max_m: 5, y_max_m: 2 },
      {
        id: 'obs-2',
        type: 'convex_polygon',
        vertices: [
          { x_m: 10, y_m: 0 },
          { x_m: 12, y_m: 0 },
          { x_m: 11, y_m: 3 },
        ],
      },
    ],
    algorithm_params: {
      hybrid_astar: { time_budget_s: 4.0, node_budget: 500000 },
      corridor: { enabled: false, margin_m: 0.3 },
    },
  };
}

function withOverride(mutate: (scene: Record<string, unknown>) => void): Record<string, unknown> {
  const scene = structuredClone(validScene());
  mutate(scene);
  return scene;
}

function expectSceneError(value: unknown, pattern: RegExp, path?: string): SceneParseError {
  try {
    parseScene(value);
  } catch (error) {
    expect(error).toBeInstanceOf(SceneParseError);
    const sceneError = error as SceneParseError;
    expect(sceneError.message).toMatch(pattern);
    if (path !== undefined) expect(sceneError.path).toBe(path);
    return sceneError;
  }
  throw new Error('expected parseScene to throw');
}

describe('场景 JSON 解析（web_scene v1）', () => {
  it('接受合法场景并保留全部字段', () => {
    const scene = parseScene(validScene());
    expect(scene.schema_version).toBe(1);
    expect(scene.type).toBe('web_scene');
    expect(scene.map_id).toBe('mapping_example');
    expect(scene.start).toEqual({ x_m: 1, y_m: 2, yaw_rad: 0.5 });
    expect(scene.goal?.yaw_rad).toBeCloseTo(-0.5, 12);
    expect(scene.obstacles).toHaveLength(2);
    expect(scene.obstacles[1].type).toBe('convex_polygon');
    expect(scene.algorithm_params.hybrid_astar.node_budget).toBe(500000);
    expect(scene.algorithm_params.corridor.enabled).toBe(false);
  });

  it('缺省字段：vehicle 引用、algorithm_params、name、位姿 null', () => {
    const raw = validScene();
    delete raw.vehicle;
    delete raw.algorithm_params;
    delete raw.name;
    delete raw.start;
    delete raw.goal;
    const scene = parseScene(raw);
    expect(scene.vehicle.ref).toBe('default_test_vehicle');
    expect(scene.vehicle.path).toBe('tests/fixtures/v3/vehicles/default.json');
    expect(scene.start).toBeNull();
    expect(scene.goal).toBeNull();
    expect(scene.name).toBe('');
    expect(scene.algorithm_params.hybrid_astar.time_budget_s).toBe(5.0);
    expect(scene.algorithm_params.corridor.margin_m).toBe(0.5);
  });

  it('序列化 → 解析 往返逐字段一致，且字节确定（障碍按 ID 排序）', () => {
    const raw = validScene();
    (raw.obstacles as unknown[]).reverse(); // 乱序输入
    const scene = parseScene(raw);
    const text = serializeScene(scene);
    const again = parseScene(JSON.parse(text) as unknown);
    expect(again).toEqual(scene);
    expect(serializeScene(again)).toBe(text);
    // 障碍按数字 ID 升序输出
    const parsed = JSON.parse(text) as WebScene;
    expect(parsed.obstacles.map((obstacle) => obstacle.id)).toEqual(['obs-1', 'obs-2']);
  });

  it('拒绝错误包络/必填缺失', () => {
    expectSceneError(withOverride((scene) => (scene.schema_version = 2)), /schema_version/, '$.schema_version');
    expectSceneError(withOverride((scene) => (scene.type = 'other')), /web_scene/, '$.type');
    expectSceneError(withOverride((scene) => delete scene.map_id), /expected string/, '$.map_id');
    expectSceneError(withOverride((scene) => (scene.map_version = '')), /must not be empty/, '$.map_version');
  });

  it('拒绝非有限数值与非法障碍几何', () => {
    expectSceneError(
      withOverride((scene) => ((scene.start as Record<string, unknown>).x_m = Number.NaN)),
      /finite number/,
      '$.start.x_m',
    );
    expectSceneError(
      withOverride((scene) => {
        (scene.obstacles as Array<Record<string, unknown>>)[0].x_min_m = 10;
      }),
      /positive extent/,
      '$.obstacles[0]',
    );
    expectSceneError(
      withOverride((scene) => {
        (scene.obstacles as Array<Record<string, unknown>>)[1].vertices = [{ x_m: 0, y_m: 0 }];
      }),
      /at least 3 vertices/,
      '$.obstacles[1].vertices',
    );
    expectSceneError(
      withOverride((scene) => {
        (scene.obstacles as Array<Record<string, unknown>>)[0].type = 'circle';
      }),
      /unsupported obstacle type/,
      '$.obstacles[0].type',
    );
  });

  it('拒绝重复障碍 ID 与非法参数', () => {
    expectSceneError(
      withOverride((scene) => {
        (scene.obstacles as Array<Record<string, unknown>>)[1].id = 'obs-1';
      }),
      /duplicate obstacle id/,
      '$.obstacles',
    );
    expectSceneError(
      withOverride((scene) => {
        (scene.algorithm_params as Record<string, Record<string, unknown>>).hybrid_astar.time_budget_s = 0;
      }),
      /must be > 0/,
      '$.algorithm_params.hybrid_astar.time_budget_s',
    );
    expectSceneError(
      withOverride((scene) => {
        (scene.algorithm_params as Record<string, Record<string, unknown>>).corridor.margin_m = -1;
      }),
      /must be >= 0/,
      '$.algorithm_params.corridor.margin_m',
    );
  });

  it('sceneFromJson 拒绝非法 JSON 并给出路径', () => {
    try {
      sceneFromJson('{ not json');
      throw new Error('should have thrown');
    } catch (error) {
      expect(error).toBeInstanceOf(SceneParseError);
      expect((error as SceneParseError).path).toBe('$');
    }
  });
});

describe('场景/地图版本校验', () => {
  const current = { map_id: 'mapping_example', map_version: 'v'.repeat(64) };

  it('一致时通过', () => {
    const check = checkSceneMap({ map_id: 'mapping_example', map_version: 'v'.repeat(64) }, current);
    expect(check.ok).toBe(true);
    expect(check.reason).toBe('');
  });

  it('地图 ID 不一致：拒绝并给出原因', () => {
    const check = checkSceneMap({ map_id: 'other_map', map_version: 'v'.repeat(64) }, current);
    expect(check.ok).toBe(false);
    expect(check.reason).toMatch(/地图 ID 不一致/);
  });

  it('地图版本不一致：拒绝并给出原因', () => {
    const check = checkSceneMap({ map_id: 'mapping_example', map_version: 'x'.repeat(64) }, current);
    expect(check.ok).toBe(false);
    expect(check.reason).toMatch(/地图版本不一致/);
  });
});
