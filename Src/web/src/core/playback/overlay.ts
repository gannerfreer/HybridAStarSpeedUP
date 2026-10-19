import type { Pose2d } from '../pose';
import { sampleBspline } from './bspline';
import { polynomialProfile, samplePolynomialCurve } from './polynomial';
import { sampleTrajectoryAt } from './time';
import type { PlaybackLayers, PlaybackSampleData } from './types';

/**
 * 回放样例 → 世界坐标覆盖层数据（纯函数，供渲染与调试状态复用）。
 * 失败样例不产生路径；过期样例不产生任何几何（由界面给出警告）。
 */

export type Pt = readonly [number, number];

export interface PathOverlay {
  id: string;
  segments: Array<{ points: Pt[]; direction: 'forward' | 'reverse' }>;
  cusps: Pt[];
  start: Pt | null;
  end: Pt | null;
}

export interface CurveOverlay {
  id: string;
  points: Pt[];
  control_points: Pt[];
  boundaries: Pt[];
  profile: Array<{ t_s: number; velocity_mps: number; curvature_inv_m: number }>;
}

export interface CorridorOverlay {
  id: string;
  polygons: Array<{ ring: Pt[]; s_start_m: number; s_end_m: number }>;
}

export interface TrajectoryOverlay {
  id: string;
  points: Array<{ p: Pt; velocity_mps: number }>;
}

export interface CollisionOverlay {
  footprint: Pt[];
  obstacle: Pt[];
  overlap: Pt[] | null;
  verdict: string;
  reason: string;
}

export interface PlaybackOverlay {
  layers: PlaybackLayers;
  paths: PathOverlay[];
  curves: CurveOverlay[];
  corridors: CorridorOverlay[];
  trajectories: TrajectoryOverlay[];
  collision: CollisionOverlay | null;
  playback_pose: Pose2d | null;
}

export function emptyPlaybackOverlay(layers: PlaybackLayers): PlaybackOverlay {
  return { layers, paths: [], curves: [], corridors: [], trajectories: [], collision: null, playback_pose: null };
}

/** 附加覆盖层数据（规划服务结果的走廊等，与样例渲染管线共用）。 */
export interface PlaybackOverlayExtras {
  corridors?: CorridorOverlay[];
}

function planResultOverlay(sample: Extract<PlaybackSampleData, { kind: 'plan_result' }>, playbackTime: number): {
  paths: PathOverlay[];
  trajectories: TrajectoryOverlay[];
  playback_pose: Pose2d | null;
} {
  const { result, sample: meta } = sample;
  const paths: PathOverlay[] = [];
  const succeeded = result.status === 'success' || result.status === 'succeeded';
  if (succeeded && result.path.length > 0) {
    const segments = result.path
      .map((segment) => ({
        points: segment.points.map((point): Pt => [point.pose.x_m, point.pose.y_m]),
        direction: segment.points[0]?.direction ?? 'forward',
      }))
      .filter((segment) => segment.points.length > 0);
    const cusps: Pt[] = [];
    const allPoints: Array<{ direction: 'forward' | 'reverse'; p: Pt }> = [];
    for (const segment of result.path) {
      for (const point of segment.points) {
        allPoints.push({ direction: point.direction, p: [point.pose.x_m, point.pose.y_m] });
      }
    }
    for (let index = 1; index < allPoints.length; index += 1) {
      if (allPoints[index].direction !== allPoints[index - 1].direction) {
        cusps.push(allPoints[index].p);
      }
    }
    paths.push({
      id: meta.id,
      segments,
      cusps,
      start: allPoints[0]?.p ?? null,
      end: allPoints[allPoints.length - 1]?.p ?? null,
    });
  }

  const trajectories: TrajectoryOverlay[] = [];
  let playbackPose: Pose2d | null = null;
  if (result.trajectory.length > 0) {
    trajectories.push({
      id: meta.id,
      points: result.trajectory.map((point) => ({
        p: [point.pose.x_m, point.pose.y_m] as Pt,
        velocity_mps: point.velocity_mps,
      })),
    });
    const at = sampleTrajectoryAt(result.trajectory, playbackTime);
    playbackPose = at ? at.pose : null;
  }
  return { paths, trajectories, playback_pose: playbackPose };
}

export function buildPlaybackOverlay(
  sample: PlaybackSampleData | null,
  layers: PlaybackLayers,
  playbackTime = 0,
  extras?: PlaybackOverlayExtras,
): PlaybackOverlay {
  const overlay = emptyPlaybackOverlay(layers);
  if (extras?.corridors && layers.corridors) {
    overlay.corridors.push(...extras.corridors);
  }
  if (!sample) return overlay;
  if (sample.kind === 'plan_result') {
    const built = planResultOverlay(sample, playbackTime);
    if (layers.paths) overlay.paths = built.paths;
    if (layers.trajectories) {
      overlay.trajectories = built.trajectories;
      overlay.playback_pose = built.playback_pose;
    }
  } else if (sample.kind === 'bspline_curve' && layers.curves) {
    overlay.curves.push({
      id: sample.curve.id,
      points: sampleBspline(sample.curve, 200).map((point): Pt => [point.x_m, point.y_m]),
      control_points: sample.curve.control_points.map((point): Pt => [point.x_m, point.y_m]),
      boundaries: [],
      profile: [],
    });
  } else if (sample.kind === 'polynomial_curve' && layers.curves) {
    const profile = polynomialProfile(sample.curve, 50).map((entry) => ({
      t_s: entry.t_s,
      velocity_mps: entry.velocity_mps,
      curvature_inv_m: entry.curvature_inv_m,
    }));
    const boundaries = sample.curve.segments.slice(1).map((segment): Pt => {
      const first = segment.coefficients_ascending;
      return [first.x_m[0] ?? 0, first.y_m[0] ?? 0];
    });
    overlay.curves.push({
      id: sample.curve.id,
      points: samplePolynomialCurve(sample.curve, 100).map((point): Pt => [point.x_m, point.y_m]),
      control_points: [],
      boundaries,
      profile,
    });
  } else if (sample.kind === 'corridor' && layers.corridors) {
    overlay.corridors.push({
      id: sample.corridor.id,
      polygons: sample.corridor.polygons.map((polygon) => ({
        ring: polygon.vertices.map((vertex): Pt => [vertex.x_m, vertex.y_m]),
        s_start_m: polygon.s_start_m,
        s_end_m: polygon.s_end_m,
      })),
    });
  } else if (sample.kind === 'collision_case' && layers.collision) {
    const { obstacle, footprint_corners, overlap_region } = sample.collision;
    overlay.collision = {
      footprint: footprint_corners.map((corner): Pt => [corner.x_m, corner.y_m]),
      obstacle: [
        [obstacle.x_min_m, obstacle.y_min_m],
        [obstacle.x_max_m, obstacle.y_min_m],
        [obstacle.x_max_m, obstacle.y_max_m],
        [obstacle.x_min_m, obstacle.y_max_m],
      ],
      overlap: overlap_region
        ? [
            [overlap_region.min_x_m, overlap_region.min_y_m],
            [overlap_region.max_x_m, overlap_region.min_y_m],
            [overlap_region.max_x_m, overlap_region.max_y_m],
            [overlap_region.min_x_m, overlap_region.max_y_m],
          ]
        : null,
      verdict: sample.collision.verdict,
      reason: sample.collision.reason,
    };
  }
  return overlay;
}

/** 剖面数据（v(t)/κ(t) 小图）：轨迹或多项式曲线。 */
export function playbackProfile(sample: PlaybackSampleData | null): Array<{
  t_s: number;
  velocity_mps: number;
  curvature_inv_m: number;
}> {
  if (!sample) return [];
  if (sample.kind === 'plan_result' && sample.result.trajectory.length > 0) {
    return sample.result.trajectory.map((point) => ({
      t_s: point.t_s,
      velocity_mps: point.velocity_mps,
      curvature_inv_m: point.curvature_inv_m,
    }));
  }
  if (sample.kind === 'polynomial_curve') {
    return polynomialProfile(sample.curve, 50).map((entry) => ({
      t_s: entry.t_s,
      velocity_mps: entry.velocity_mps,
      curvature_inv_m: entry.curvature_inv_m,
    }));
  }
  return [];
}

/** 回放时长（轨迹样例）；无轨迹返回 0。 */
export function playbackDuration(sample: PlaybackSampleData | null): number {
  if (sample && sample.kind === 'plan_result' && sample.result.trajectory.length > 1) {
    const trajectory = sample.result.trajectory;
    return trajectory[trajectory.length - 1].t_s - trajectory[0].t_s;
  }
  return 0;
}
