import { describe, expect, it } from 'vitest';
import { evaluateBspline, sampleBspline } from '../src/core/playback/bspline';
import { parseBsplineCurve } from '../src/core/playback/parse';
import { readPlaybackFile } from './playback_fixtures';

describe('B 样条 de Boor 求值', () => {
  const curve = parseBsplineCurve(readPlaybackFile('bspline_curve.json').value);

  it('夹紧端点：u=0 与 u=1 分别等于首/末控制点（1e-6 内）', () => {
    const first = evaluateBspline(curve.control_points, curve.knots, curve.degree, 0);
    const last = evaluateBspline(curve.control_points, curve.knots, curve.degree, 1);
    expect(first.x_m).toBeCloseTo(curve.control_points[0].x_m, 6);
    expect(first.y_m).toBeCloseTo(curve.control_points[0].y_m, 6);
    expect(last.x_m).toBeCloseTo(curve.control_points[7].x_m, 6);
    expect(last.y_m).toBeCloseTo(curve.control_points[7].y_m, 6);
  });

  it('一次（直线）样条精确复现直线', () => {
    const knots = [0, 0, 1, 2, 3, 3];
    const points = [
      { x_m: 0, y_m: 0 },
      { x_m: 2, y_m: 1 },
      { x_m: 4, y_m: 2 },
      { x_m: 6, y_m: 3 },
    ];
    for (const u of [0, 0.5, 1, 1.5, 2, 2.5, 3]) {
      const point = evaluateBspline(points, knots, 1, u);
      // 域端点按 1e-9 内收（夹紧节点向量保护），容差 1e-6。
      expect(point.x_m).toBeCloseTo(2 * u, 6);
      expect(point.y_m).toBeCloseTo(u, 6);
    }
  });

  it('域外参数抛出（不插值外推）', () => {
    expect(() => evaluateBspline(curve.control_points, curve.knots, curve.degree, -0.01)).toThrowError(RangeError);
    expect(() => evaluateBspline(curve.control_points, curve.knots, curve.degree, 1.01)).toThrowError(RangeError);
  });

  it('等距采样 200 点、端点吻合且全部有限', () => {
    const points = sampleBspline(curve, 200);
    expect(points).toHaveLength(200);
    expect(points[0].x_m).toBeCloseTo(curve.control_points[0].x_m, 6);
    expect(points[199].x_m).toBeCloseTo(curve.control_points[7].x_m, 6);
    for (const point of points) {
      expect(Number.isFinite(point.x_m) && Number.isFinite(point.y_m)).toBe(true);
    }
  });

  it('采样点数下限校验', () => {
    expect(() => sampleBspline(curve, 1)).toThrowError(RangeError);
  });
});
