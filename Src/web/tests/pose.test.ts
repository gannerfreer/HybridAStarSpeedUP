import { describe, expect, it } from 'vitest';
import {
  degToRad,
  headingTargetPoint,
  normalizeYaw,
  poseFromLanelet,
  radToDeg,
  yawFromCenterline,
  yawFromDelta,
} from '../src/core/pose';
import { lanelet, point, straightLanelet } from './fixtures';

describe('位姿与航向工具', () => {
  it('normalizeYaw：归一化到 [-π, π)', () => {
    expect(normalizeYaw(0)).toBeCloseTo(0, 12);
    expect(normalizeYaw(Math.PI)).toBeCloseTo(-Math.PI, 12);
    expect(normalizeYaw(-Math.PI)).toBeCloseTo(-Math.PI, 12);
    expect(normalizeYaw((3 * Math.PI) / 2)).toBeCloseTo(-Math.PI / 2, 12);
    expect(normalizeYaw(2 * Math.PI + 0.5)).toBeCloseTo(0.5, 12);
    expect(normalizeYaw(-2 * Math.PI - 0.5)).toBeCloseTo(-0.5, 12);
  });

  it('角度换算', () => {
    expect(radToDeg(Math.PI / 2)).toBeCloseTo(90, 12);
    expect(degToRad(180)).toBeCloseTo(Math.PI, 12);
    expect(degToRad(radToDeg(1.234))).toBeCloseTo(1.234, 12);
  });

  it('yawFromDelta', () => {
    expect(yawFromDelta(1, 0)).toBeCloseTo(0, 12);
    expect(yawFromDelta(0, 1)).toBeCloseTo(Math.PI / 2, 12);
    expect(yawFromDelta(-1, 0)).toBeCloseTo(-Math.PI, 12); // 归一化到 [-π, π)
  });

  it('yawFromCenterline：取最近段切向', () => {
    const horizontal = straightLanelet('1', 0, 10); // 中心线 (0,0)→(10,0)
    expect(yawFromCenterline(horizontal, 5, 0)).toBeCloseTo(0, 12);

    const vertical = lanelet(
      '2',
      [point('l0', -2, 0), point('l1', -2, 10)],
      [point('r0', 2, 0), point('r1', 2, 10)],
      [point('c0', 0, 0), point('c1', 0, 10)],
    );
    expect(yawFromCenterline(vertical, 0, 5)).toBeCloseTo(Math.PI / 2, 12);

    const noCenterline = lanelet('3', [point('l0', 0, 0), point('l1', 1, 0)], [point('r0', 0, 1)]);
    expect(yawFromCenterline(noCenterline, 0.5, 0.5)).toBeNull();
  });

  it('poseFromLanelet：命中时取切向，无命中时用回退航向', () => {
    const item = straightLanelet('1', 0, 10);
    const withLanelet = poseFromLanelet(item, 5, 0);
    expect(withLanelet.x_m).toBe(5);
    expect(withLanelet.yaw_rad).toBeCloseTo(0, 12);
    const without = poseFromLanelet(null, 5, 5, Math.PI / 2);
    expect(without.yaw_rad).toBeCloseTo(Math.PI / 2, 12);
  });

  it('headingTargetPoint：沿航向延伸', () => {
    expect(headingTargetPoint({ x_m: 1, y_m: 2, yaw_rad: 0 }, 5)).toEqual([6, 2]);
    const [tx, ty] = headingTargetPoint({ x_m: 0, y_m: 0, yaw_rad: Math.PI / 2 }, 2);
    expect(tx).toBeCloseTo(0, 12);
    expect(ty).toBeCloseTo(2, 12);
  });
});
