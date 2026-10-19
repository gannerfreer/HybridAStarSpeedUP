import type { BsplineCurve, PlaybackPoint } from './types';

/**
 * 三次（任意阶）B 样条 de Boor 求值（纯函数）。
 * 参数域外不插值（抛出 RangeError）；域端点评估按 1e-9 相对量内收，
 * 避免夹紧节点向量的 0/0（端点误差远小于显示精度）。
 */

function findSpan(knots: readonly number[], controlCount: number, degree: number, u: number): number {
  const n = controlCount - 1;
  if (u >= knots[n + 1]) return n;
  if (u <= knots[degree]) return degree;
  let low = degree;
  let high = n + 1;
  let mid = Math.floor((low + high) / 2);
  while (u < knots[mid] || u >= knots[mid + 1]) {
    if (u < knots[mid]) high = mid;
    else low = mid;
    mid = Math.floor((low + high) / 2);
  }
  return mid;
}

export function evaluateBspline(
  controlPoints: readonly PlaybackPoint[],
  knots: readonly number[],
  degree: number,
  u: number,
): PlaybackPoint {
  const uMin = knots[degree];
  const uMax = knots[knots.length - degree - 1];
  if (!(u >= uMin && u <= uMax)) {
    throw new RangeError(`B-spline parameter ${u} outside domain [${uMin}, ${uMax}]（不插值外推）`);
  }
  // 域端点按 1e-9 相对量内收，避免夹紧节点向量的 0/0（端点误差远小于显示精度）。
  const uEval = u >= uMax ? uMax - 1e-9 * Math.max(1e-12, uMax - knots[knots.length - degree - 2]) : u;
  const k = findSpan(knots, controlPoints.length, degree, uEval);
  const d: PlaybackPoint[] = [];
  for (let j = 0; j <= degree; j += 1) {
    const point = controlPoints[j + k - degree];
    d.push({ x_m: point.x_m, y_m: point.y_m });
  }
  for (let r = 1; r <= degree; r += 1) {
    for (let j = degree; j >= r; j -= 1) {
      const left = knots[j + k - degree];
      const right = knots[j + 1 + k - r];
      const denominator = right - left;
      const alpha = denominator === 0 ? 0 : (uEval - left) / denominator;
      d[j] = {
        x_m: (1 - alpha) * d[j - 1].x_m + alpha * d[j].x_m,
        y_m: (1 - alpha) * d[j - 1].y_m + alpha * d[j].y_m,
      };
    }
  }
  return d[degree];
}

/** 域内等距采样（含两端点；不越界）。 */
export function sampleBspline(curve: BsplineCurve, count = 200): PlaybackPoint[] {
  if (count < 2) throw new RangeError('count must be >= 2');
  const [uMin, uMax] = curve.parameter_domain;
  const points: PlaybackPoint[] = [];
  for (let index = 0; index < count; index += 1) {
    const u = uMin + ((uMax - uMin) * index) / (count - 1);
    points.push(evaluateBspline(curve.control_points, curve.knots, curve.degree, u));
  }
  return points;
}
