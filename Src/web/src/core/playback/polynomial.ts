import type { PlaybackPoint, PolynomialCurve, PolynomialSegment } from './types';

/**
 * 五次（任意阶）多项式分段求值（Horner；升幂系数）。
 * 系数按段内局部时间 τ ∈ [0, duration_s]（秒）给出（与夹具 boundary 的
 * p0/v0/p1/v1 数值一致；v 单位 m/s）。v = |dp/dτ|；
 * κ = (x'y'' - y'x'') / v³（左转正）。
 */

function evaluateSeries(coefficients: readonly number[], tau: number): number {
  let value = 0;
  for (let index = coefficients.length - 1; index >= 0; index -= 1) {
    value = value * tau + coefficients[index];
  }
  return value;
}

function evaluateSeriesDerivative(coefficients: readonly number[], tau: number): number {
  let value = 0;
  for (let index = coefficients.length - 1; index >= 1; index -= 1) {
    value = value * tau + index * coefficients[index];
  }
  return value;
}

function evaluateSeriesSecondDerivative(coefficients: readonly number[], tau: number): number {
  let value = 0;
  for (let index = coefficients.length - 1; index >= 2; index -= 1) {
    value = value * tau + index * (index - 1) * coefficients[index];
  }
  return value;
}

export interface PolynomialSample {
  t_s: number;
  position: PlaybackPoint;
  velocity_mps: number;
  curvature_inv_m: number;
}

export function evaluatePolynomialSegment(segment: PolynomialSegment, tau: number): PlaybackPoint {
  if (!(tau >= 0 && tau <= segment.duration_s)) {
    throw new RangeError(`polynomial local time ${tau} outside [0, ${segment.duration_s}]`);
  }
  return {
    x_m: evaluateSeries(segment.coefficients_ascending.x_m, tau),
    y_m: evaluateSeries(segment.coefficients_ascending.y_m, tau),
  };
}

export function samplePolynomialSegment(segment: PolynomialSegment, tau: number): PolynomialSample {
  if (!(tau >= 0 && tau <= segment.duration_s)) {
    throw new RangeError(`polynomial local time ${tau} outside [0, ${segment.duration_s}]`);
  }
  const { x_m: cx, y_m: cy } = segment.coefficients_ascending;
  const x = evaluateSeries(cx, tau);
  const y = evaluateSeries(cy, tau);
  const dx = evaluateSeriesDerivative(cx, tau);
  const dy = evaluateSeriesDerivative(cy, tau);
  const ddx = evaluateSeriesSecondDerivative(cx, tau);
  const ddy = evaluateSeriesSecondDerivative(cy, tau);
  const speed = Math.hypot(dx, dy);
  const curvature = speed > 1e-9 ? (dx * ddy - dy * ddx) / (speed * speed * speed) : 0;
  return {
    t_s: segment.t_start_s + tau,
    position: { x_m: x, y_m: y },
    velocity_mps: speed,
    curvature_inv_m: curvature,
  };
}

/** 按每段局部时间等距采样（含段边界；每段 pointsPerSegment 点）。 */
export function samplePolynomialCurve(curve: PolynomialCurve, pointsPerSegment = 100): PlaybackPoint[] {
  if (pointsPerSegment < 2) throw new RangeError('pointsPerSegment must be >= 2');
  const points: PlaybackPoint[] = [];
  for (const segment of curve.segments) {
    for (let index = 0; index < pointsPerSegment; index += 1) {
      const tau = (segment.duration_s * index) / (pointsPerSegment - 1);
      points.push(evaluatePolynomialSegment(segment, tau));
    }
  }
  return points;
}

/** 速度/曲率剖面（按每段局部时间采样；用于面板 v(t)/κ(t) 小图）。 */
export function polynomialProfile(curve: PolynomialCurve, samplesPerSegment = 50): PolynomialSample[] {
  if (samplesPerSegment < 2) throw new RangeError('samplesPerSegment must be >= 2');
  const samples: PolynomialSample[] = [];
  for (const segment of curve.segments) {
    for (let index = 0; index < samplesPerSegment; index += 1) {
      const tau = (segment.duration_s * index) / (samplesPerSegment - 1);
      samples.push(samplePolynomialSegment(segment, tau));
    }
  }
  return samples;
}

/** 段边界时刻（显示标注用）。 */
export function polynomialSegmentBoundaries(curve: PolynomialCurve): number[] {
  return curve.segments.slice(1).map((segment) => segment.t_start_s);
}
