import { describe, expect, it } from 'vitest';
import { parsePlanResult } from '../src/core/playback/parse';
import {
  directionSwitchTimes,
  interpolateYaw,
  sampleTrajectoryAt,
  trajectoryDuration,
  trajectoryProfile,
  trajectoryTimeRange,
} from '../src/core/playback/time';
import { readPlaybackFile } from './playback_fixtures';

const result = parsePlanResult(readPlaybackFile('dftpav_trajectory.json').value);
const trajectory = result.trajectory;

describe('轨迹时间映射', () => {
  it('时间范围与时长（0～7.5 s）', () => {
    expect(trajectoryTimeRange(trajectory)).toEqual([0, 7.5]);
    expect(trajectoryDuration(trajectory)).toBeCloseTo(7.5, 9);
  });

  it('精确点：t=0/4/7.5 与样例一致', () => {
    const at0 = sampleTrajectoryAt(trajectory, 0);
    expect(at0?.pose.x_m).toBeCloseTo(30, 9);
    expect(at0?.velocity_mps).toBe(0);
    const at4 = sampleTrajectoryAt(trajectory, 4);
    expect(at4?.pose.x_m).toBeCloseTo(40, 9);
    expect(at4?.velocity_mps).toBe(0);
    const atEnd = sampleTrajectoryAt(trajectory, 7.5);
    expect(atEnd?.pose.x_m).toBeCloseTo(35, 9);
    expect(atEnd?.clamped).toBe(false);
  });

  it('线性插值：t=5.5（5～6.5 s 段中间 1/3）', () => {
    const sample = sampleTrajectoryAt(trajectory, 5.5);
    expect(sample?.pose.x_m).toBeCloseTo(38, 9);
    expect(sample?.pose.y_m).toBeCloseTo(5, 9);
    expect(sample?.velocity_mps).toBeCloseTo(-2, 9);
    expect(sample?.index).toBe(5);
  });

  it('越界夹紧并标记 clamped', () => {
    const before = sampleTrajectoryAt(trajectory, -1);
    expect(before?.pose.x_m).toBeCloseTo(30, 9);
    expect(before?.clamped).toBe(true);
    const after = sampleTrajectoryAt(trajectory, 99);
    expect(after?.pose.x_m).toBeCloseTo(35, 9);
    expect(after?.clamped).toBe(true);
  });

  it('空轨迹返回 null', () => {
    expect(sampleTrajectoryAt([], 1)).toBeNull();
  });

  it('换向时刻：t=4.0（v 由 + 变 -）', () => {
    expect(directionSwitchTimes(trajectory)).toEqual([4.0]);
  });

  it('航向插值跨越 ±π', () => {
    // π 与 -π 等价；归一化到 [-π, π) 后中点表示为 -π。
    expect(interpolateYaw(Math.PI - 0.1, -Math.PI + 0.1, 0.5)).toBeCloseTo(-Math.PI, 9);
    expect(interpolateYaw(0, Math.PI / 2, 0.5)).toBeCloseTo(Math.PI / 4, 9);
  });

  it('剖面点数与轨迹一致', () => {
    const profile = trajectoryProfile(trajectory);
    expect(profile).toHaveLength(trajectory.length);
    expect(profile[4]).toEqual({ t_s: 4, velocity_mps: 0, curvature_inv_m: 0 });
  });
});
