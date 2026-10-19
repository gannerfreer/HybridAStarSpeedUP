import { describe, expect, it } from 'vitest';
import { parsePolynomialCurve } from '../src/core/playback/parse';
import {
  evaluatePolynomialSegment,
  polynomialProfile,
  polynomialSegmentBoundaries,
  samplePolynomialCurve,
  samplePolynomialSegment,
} from '../src/core/playback/polynomial';
import { readPlaybackFile } from './playback_fixtures';

/**
 * 夹具系数为段内局部时间 τ（秒）的升幂系数：p(τ) = c0 + c1 τ + …；
 * 与 boundary 的 p0/p1/v0/v1 及 smoothstep 注释一致。
 */
describe('多项式分段求值（局部时间 τ 秒）', () => {
  const curve = parsePolynomialCurve(readPlaybackFile('polynomial_curve.json').value);

  it('段边界位置与夹具 boundary 一致', () => {
    const start = evaluatePolynomialSegment(curve.segments[0], 0);
    const mid = evaluatePolynomialSegment(curve.segments[0], 4.0);
    const end = evaluatePolynomialSegment(curve.segments[1], 2.0);
    expect(start.x_m).toBeCloseTo(10, 9);
    expect(start.y_m).toBeCloseTo(5, 9);
    expect(mid.x_m).toBeCloseTo(25, 9);
    expect(mid.y_m).toBeCloseTo(8, 9);
    expect(end.x_m).toBeCloseTo(32.5, 9);
    expect(end.y_m).toBeCloseTo(8, 9);
  });

  it('段间连续（t=4.0 s 位置一致）', () => {
    const lastOfFirst = evaluatePolynomialSegment(curve.segments[0], 4.0);
    const firstOfSecond = evaluatePolynomialSegment(curve.segments[1], 0);
    expect(firstOfSecond.x_m).toBeCloseTo(lastOfFirst.x_m, 9);
    expect(firstOfSecond.y_m).toBeCloseTo(lastOfFirst.y_m, 9);
  });

  it('速度：x 恒速 3.75 m/s、y 端点静止（smoothstep）', () => {
    const start = samplePolynomialSegment(curve.segments[0], 0);
    const end = samplePolynomialSegment(curve.segments[0], 4.0);
    expect(start.velocity_mps).toBeCloseTo(3.75, 6);
    expect(end.velocity_mps).toBeCloseTo(3.75, 6);
    const middle = samplePolynomialSegment(curve.segments[0], 2.0);
    expect(middle.velocity_mps).toBeGreaterThan(3.75);
    // smoothstep 在中点 y''=0（拐点）→ κ=0；四分之一处曲率非零（左转）。
    expect(Math.abs(middle.curvature_inv_m)).toBeLessThan(1e-9);
    const quarter = samplePolynomialSegment(curve.segments[0], 1.0);
    expect(quarter.curvature_inv_m).toBeGreaterThan(0);
  });

  it('y 的 smoothstep 数值（3·(10u³-15u⁴+6u⁵)，u=t/4）', () => {
    const sample = samplePolynomialSegment(curve.segments[0], 1.0);
    const u = 1.0 / 4.0;
    const expectedY = 5 + 3 * (10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5);
    expect(sample.position.y_m).toBeCloseTo(expectedY, 9);
  });

  it('域外局部时间抛出', () => {
    expect(() => evaluatePolynomialSegment(curve.segments[0], -0.01)).toThrowError(RangeError);
    expect(() => evaluatePolynomialSegment(curve.segments[0], 4.01)).toThrowError(RangeError);
  });

  it('曲线采样 2×100 点、剖面 2×50 点、段边界 [4.0]', () => {
    expect(samplePolynomialCurve(curve, 100)).toHaveLength(200);
    const profile = polynomialProfile(curve, 50);
    expect(profile).toHaveLength(100);
    expect(profile[0].t_s).toBeCloseTo(0, 9);
    expect(profile[profile.length - 1].t_s).toBeCloseTo(6, 9);
    expect(polynomialSegmentBoundaries(curve)).toEqual([4.0]);
  });
});
