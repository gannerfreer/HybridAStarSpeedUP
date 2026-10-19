import { distanceToSegment } from './hit';
import type { Lanelet } from './types';

/** 起终位姿（后轴中心；航向逆时针为正，归一化到 [-π, π)）。 */
export interface Pose2d {
  x_m: number;
  y_m: number;
  yaw_rad: number;
}

export function normalizeYaw(yaw_rad: number): number {
  // 已归一化的值原样返回，避免 JSON 往返引入 1e-16 级扰动（场景逐字段一致）。
  if (yaw_rad >= -Math.PI && yaw_rad < Math.PI) return yaw_rad;
  const twoPi = Math.PI * 2;
  let value = (yaw_rad + Math.PI) % twoPi;
  if (value < 0) value += twoPi;
  return value - Math.PI;
}

export function radToDeg(rad: number): number {
  return (rad * 180) / Math.PI;
}

export function degToRad(deg: number): number {
  return (deg * Math.PI) / 180;
}

export function yawFromDelta(dx: number, dy: number): number {
  return normalizeYaw(Math.atan2(dy, dx));
}

/**
 * 由 lanelet 中心线最近段的切向得到航向（用于点击设位姿时的默认朝向）；
 * 无可用中心线时返回 null（调用方使用回退航向）。
 */
export function yawFromCenterline(lanelet: Lanelet, x_m: number, y_m: number): number | null {
  const line = lanelet.centerline && lanelet.centerline.length >= 2 ? lanelet.centerline : null;
  if (!line) return null;
  let bestDistance = Number.POSITIVE_INFINITY;
  let bestYaw: number | null = null;
  for (let index = 0; index + 1 < line.length; index += 1) {
    const a = line[index];
    const b = line[index + 1];
    const distance = distanceToSegment(x_m, y_m, a.x_m, a.y_m, b.x_m, b.y_m);
    if (distance < bestDistance) {
      bestDistance = distance;
      bestYaw = Math.atan2(b.y_m - a.y_m, b.x_m - a.x_m);
    }
  }
  return bestYaw === null ? null : normalizeYaw(bestYaw);
}

/** 从命中 lanelet 生成默认位姿；无命中时航向回退为 fallback。 */
export function poseFromLanelet(
  lanelet: Lanelet | null,
  x_m: number,
  y_m: number,
  fallbackYawRad = 0,
): Pose2d {
  const yaw = lanelet ? yawFromCenterline(lanelet, x_m, y_m) : null;
  return { x_m, y_m, yaw_rad: normalizeYaw(yaw ?? fallbackYawRad) };
}

/** 位姿航向在给定长度处的目标点（世界坐标，用于航向拖拽/校验）。 */
export function headingTargetPoint(pose: Pose2d, length_m: number): [number, number] {
  return [pose.x_m + Math.cos(pose.yaw_rad) * length_m, pose.y_m + Math.sin(pose.yaw_rad) * length_m];
}
