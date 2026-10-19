import { describe, expect, it } from 'vitest';
import { findRepoRoot, moduleDir, readVehicleJson } from '../src/core/fixture';
import {
  bodyCorners,
  footprintIntersectsObstacleBounds,
  footprintWorldCorners,
  marginCorners,
  maxCurvature1PerM,
  minTurnRadiusM,
  parseVehicleModel,
  VehicleParseError,
  vehicleLengthM,
  type VehicleModel,
} from '../src/core/scene/vehicle';

const repoRoot = findRepoRoot(moduleDir(import.meta.url));
const fixture = readVehicleJson(repoRoot);

describe('车辆模型（v3 夹具）', () => {
  it('解析 tests/fixtures/v3/vehicles/default.json', () => {
    const model = parseVehicleModel(fixture.value);
    expect(model.id).toBe('default_test_vehicle');
    expect(model.wheelbase_m).toBeCloseTo(2.8, 12);
    expect(model.width_m).toBeCloseTo(1.8, 12);
    expect(model.front_overhang_m).toBeCloseTo(0.9, 12);
    expect(model.rear_overhang_m).toBeCloseTo(1.0, 12);
    expect(model.max_steer_rad).toBeCloseTo(0.5, 12);
    expect(model.safety_margin_m).toBeCloseTo(0.1, 12);
    expect(model.footprint_shape).toBe('rectangle');
  });

  it('派生量：最小转弯半径与最大曲率（夹具注释值）', () => {
    const model = parseVehicleModel(fixture.value);
    expect(minTurnRadiusM(model)).toBeCloseTo(5.1254, 3);
    expect(maxCurvature1PerM(model)).toBeCloseTo(0.195108, 5);
    expect(vehicleLengthM(model)).toBeCloseTo(4.7, 12);
  });

  it('拒绝非法车辆参数', () => {
    const mutate = (change: (model: Record<string, unknown>) => void) => {
      const raw = structuredClone(fixture.value) as Record<string, unknown>;
      change(raw);
      return raw;
    };
    expect(() => parseVehicleModel(mutate((m) => (m.wheelbase_m = 0)))).toThrowError(/wheelbase_m/);
    expect(() => parseVehicleModel(mutate((m) => (m.max_steer_rad = Math.PI / 2)))).toThrowError(/max_steer_rad/);
    expect(() => parseVehicleModel(mutate((m) => (m.safety_margin_m = -0.1)))).toThrowError(/safety_margin_m/);
    expect(() => parseVehicleModel(mutate((m) => (m.type = 'other')))).toThrowError(/vehicle_model/);
    try {
      parseVehicleModel(mutate((m) => (m.width_m = Number.NaN)));
      throw new Error('should have thrown');
    } catch (error) {
      expect(error).toBeInstanceOf(VehicleParseError);
      expect((error as VehicleParseError).path).toBe('$.width_m');
    }
  });
});

describe('整车足迹几何', () => {
  const model: VehicleModel = parseVehicleModel(fixture.value);

  it('车体矩形：x ∈ [-rear, wheelbase+front]，y ∈ [±width/2]', () => {
    const corners = bodyCorners(model);
    expect(corners).toHaveLength(4);
    expect(corners[0][0]).toBeCloseTo(3.7, 12);
    expect(corners[0][1]).toBeCloseTo(0.9, 12);
    expect(corners[1][0]).toBeCloseTo(3.7, 12);
    expect(corners[1][1]).toBeCloseTo(-0.9, 12);
    expect(corners[2][0]).toBeCloseTo(-1.0, 12);
    expect(corners[2][1]).toBeCloseTo(-0.9, 12);
    expect(corners[3][0]).toBeCloseTo(-1.0, 12);
    expect(corners[3][1]).toBeCloseTo(0.9, 12);
  });

  it('裕量矩形向外扩展 margin（0.1 m）', () => {
    const corners = marginCorners(model);
    expect(corners[0][0]).toBeCloseTo(3.8, 12);
    expect(corners[0][1]).toBeCloseTo(1.0, 12);
    expect(corners[2][0]).toBeCloseTo(-1.1, 12);
    expect(corners[2][1]).toBeCloseTo(-1.0, 12);
    // 裕量面积严格大于车体
    const bodyArea = 4.7 * 1.8;
    const marginArea = 4.9 * 2.0;
    expect(marginArea).toBeGreaterThan(bodyArea);
  });

  it('世界坐标旋转：yaw=π/2 时车头指向 +y', () => {
    const pose = { x_m: 10, y_m: 20, yaw_rad: Math.PI / 2 };
    const world = footprintWorldCorners(model, pose, false);
    // 车头角 (front=3.7, half=0.9) → 世界 (10 - 0.9, 20 + 3.7)
    expect(world[0][0]).toBeCloseTo(9.1, 12);
    expect(world[0][1]).toBeCloseTo(23.7, 12);
    // 车尾角 (-1.0, 0.9) → (10 - 0.9, 20 - 1.0)
    expect(world[3][0]).toBeCloseTo(9.1, 12);
    expect(world[3][1]).toBeCloseTo(19.0, 12);
  });

  it('足迹-障碍包围盒相交（含裕量）', () => {
    const pose = { x_m: 0, y_m: 0, yaw_rad: 0 };
    const hit = { min_x_m: 3.0, min_y_m: -1, max_x_m: 4, max_y_m: 1 };
    const miss = { min_x_m: 4.5, min_y_m: -1, max_x_m: 5, max_y_m: 1 };
    expect(footprintIntersectsObstacleBounds(model, pose, hit, false)).toBe(true);
    expect(footprintIntersectsObstacleBounds(model, pose, miss, false)).toBe(false);
    // 裕量 0.1 m 时 3.7+0.1=3.8 仍与 4.5 不交
    expect(footprintIntersectsObstacleBounds(model, pose, miss, true)).toBe(false);
    const near = { min_x_m: 3.75, min_y_m: -1, max_x_m: 4, max_y_m: 1 };
    expect(footprintIntersectsObstacleBounds(model, pose, near, false)).toBe(false);
    expect(footprintIntersectsObstacleBounds(model, pose, near, true)).toBe(true);
  });
});
