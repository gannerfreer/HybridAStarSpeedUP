import { normalizeYaw, type Pose2d } from '../pose';
import type { TrajectoryPoint } from './types';

/** 轨迹时间映射与剖面（纯函数；线性插值，时间越界夹紧）。 */

export interface TrajectorySample {
  pose: Pose2d;
  velocity_mps: number;
  acceleration_mps2: number;
  curvature_inv_m: number;
  /** 左侧邻近点索引。 */
  index: number;
  clamped: boolean;
}

export function trajectoryTimeRange(trajectory: readonly TrajectoryPoint[]): [number, number] {
  if (trajectory.length === 0) return [0, 0];
  return [trajectory[0].t_s, trajectory[trajectory.length - 1].t_s];
}

export function trajectoryDuration(trajectory: readonly TrajectoryPoint[]): number {
  const [start, end] = trajectoryTimeRange(trajectory);
  return Math.max(0, end - start);
}

export function interpolateYaw(from: number, to: number, fraction: number): number {
  return normalizeYaw(from + normalizeYaw(to - from) * fraction);
}

export function sampleTrajectoryAt(
  trajectory: readonly TrajectoryPoint[],
  t_s: number,
): TrajectorySample | null {
  if (trajectory.length === 0) return null;
  const [tMin, tMax] = trajectoryTimeRange(trajectory);
  const clamped = t_s < tMin || t_s > tMax;
  const t = Math.min(tMax, Math.max(tMin, t_s));
  let index = trajectory.length - 1;
  for (let i = 0; i + 1 < trajectory.length; i += 1) {
    if (t >= trajectory[i].t_s && t <= trajectory[i + 1].t_s) {
      index = i;
      break;
    }
  }
  if (trajectory.length === 1) {
    const only = trajectory[0];
    return {
      pose: { ...only.pose },
      velocity_mps: only.velocity_mps,
      acceleration_mps2: only.acceleration_mps2,
      curvature_inv_m: only.curvature_inv_m,
      index: 0,
      clamped,
    };
  }
  const a = trajectory[index];
  const b = trajectory[Math.min(index + 1, trajectory.length - 1)];
  const span = b.t_s - a.t_s;
  const fraction = span > 0 ? (t - a.t_s) / span : 0;
  return {
    pose: {
      x_m: a.pose.x_m + (b.pose.x_m - a.pose.x_m) * fraction,
      y_m: a.pose.y_m + (b.pose.y_m - a.pose.y_m) * fraction,
      yaw_rad: interpolateYaw(a.pose.yaw_rad, b.pose.yaw_rad, fraction),
    },
    velocity_mps: a.velocity_mps + (b.velocity_mps - a.velocity_mps) * fraction,
    acceleration_mps2: a.acceleration_mps2 + (b.acceleration_mps2 - a.acceleration_mps2) * fraction,
    curvature_inv_m: a.curvature_inv_m + (b.curvature_inv_m - a.curvature_inv_m) * fraction,
    index,
    clamped,
  };
}

export interface ProfilePoint {
  t_s: number;
  velocity_mps: number;
  curvature_inv_m: number;
}

export function trajectoryProfile(trajectory: readonly TrajectoryPoint[]): ProfilePoint[] {
  return trajectory.map((point) => ({
    t_s: point.t_s,
    velocity_mps: point.velocity_mps,
    curvature_inv_m: point.curvature_inv_m,
  }));
}

/** 换向时刻（跨越零速点的符号变化；无零速点时线性插值零点）。 */
export function directionSwitchTimes(trajectory: readonly TrajectoryPoint[]): number[] {
  const switches: number[] = [];
  let lastNonZeroIndex = -1;
  for (let index = 0; index < trajectory.length; index += 1) {
    const velocity = trajectory[index].velocity_mps;
    if (velocity === 0) continue;
    const sign = Math.sign(velocity);
    if (lastNonZeroIndex >= 0 && sign !== Math.sign(trajectory[lastNonZeroIndex].velocity_mps)) {
      let zeroIndex = -1;
      for (let probe = lastNonZeroIndex + 1; probe < index; probe += 1) {
        if (trajectory[probe].velocity_mps === 0) {
          zeroIndex = probe;
          break;
        }
      }
      if (zeroIndex >= 0) {
        switches.push(trajectory[zeroIndex].t_s);
      } else {
        const a = trajectory[lastNonZeroIndex];
        const b = trajectory[index];
        const fraction = Math.abs(a.velocity_mps) / (Math.abs(a.velocity_mps) + Math.abs(b.velocity_mps));
        switches.push(a.t_s + (b.t_s - a.t_s) * fraction);
      }
    }
    lastNonZeroIndex = index;
  }
  return switches;
}
