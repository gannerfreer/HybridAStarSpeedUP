import type { BoundsM } from '../types';

/**
 * W2 本地 C++ 地图服务（planner_map_service）的规范化类型。
 * 与地图组件公共类型（Src/map/include/planner/map/types.hpp）及服务实现
 * （Src/map/src/service_main.cpp）对齐：对象 ID 一律字符串、数值有限、
 * 坐标系为局部米制地图系（x 东 / y 北）；窗口裁剪结果的路径坐标位于
 * 窗口局部系（平移 + 可选 yaw，`window_frame` 给出变换）。
 */

export interface WirePoint {
  id: string;
  x_m: number;
  y_m: number;
  z_m: number;
}

/** 点/包围盒查询返回的对象摘要（`lanelets` / `areas` 元素）。 */
export interface ObjectSummary {
  id: string;
  attributes: Record<string, string>;
  bounds_m: BoundsM | null;
}

export interface LaneletGeometry {
  id: string;
  attributes: Record<string, string>;
  left: WirePoint[];
  right: WirePoint[];
  centerline: WirePoint[];
}

export interface AreaGeometry {
  id: string;
  attributes: Record<string, string>;
  outer: WirePoint[];
  inner: WirePoint[][];
}

export interface IdListResult {
  status: string;
  reason: string;
  ids: string[];
}

export interface RouteResult {
  status: string;
  reason: string;
  lanelet_ids: string[];
  length_m: number;
}

export interface ClippedPath {
  lanelet_id: string;
  role: string;
  points: WirePoint[];
  start_clip_edge: boolean;
  end_clip_edge: boolean;
}

export interface CropResult {
  window_m: BoundsM;
  /** 窗口局部系原点在地图坐标中的位姿（平移为主，yaw 通常为 0）。 */
  window_frame: { x_m: number; y_m: number; yaw_rad: number };
  paths: ClippedPath[];
  points: WirePoint[];
  lanelet_ids: string[];
  diagnostics: string[];
}

export interface PointQueryResult {
  x_m: number;
  y_m: number;
  lanelets: ObjectSummary[];
  areas: ObjectSummary[];
}

export interface MapInfoPayload {
  map_id: string;
  map_version: string;
  counts: Record<string, number>;
  bounds_m: BoundsM | null;
}

export type AdjacentSide = 'left' | 'right';
