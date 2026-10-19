import type { BoundsM } from '../types';
import type { Pose2d } from '../pose';

/**
 * 栅格夹具类型（协议 v1，`Docs/expansion/grid_protocol.md`）：
 * - `grid_view`：公共序列化冻结契约的整数数组形态（v5 minimal.json 体例）；
 * - `grid_snapshot`：传输/显示文档，`encoding: "base64_u8"`，三态 0/1/2。
 * `source: "fixture"` 为版本化离线显示夹具（合成数据，非算法正确性证据）。
 */

export type GridCellState = 0 | 1 | 2;

export const GRID_STATE_FREE: GridCellState = 0;
export const GRID_STATE_OCCUPIED: GridCellState = 1;
export const GRID_STATE_UNKNOWN: GridCellState = 2;

export interface GridOrigin {
  x_m: number;
  y_m: number;
  yaw_rad: number;
}

export type GridDocumentKind = 'grid_view' | 'grid_snapshot';
export type GridSource = 'driveable' | 'planning_scene' | 'fixture';

/** 解析后的栅格图层（cells 已解码为三态字节）。 */
export interface GridLayerData {
  kind: GridDocumentKind;
  /** grid_view 无 source 字段，解析时记 `"grid_view"`。 */
  source: GridSource | 'grid_view';
  map_id: string;
  map_version: string;
  width: number;
  height: number;
  resolution_m: number;
  origin: GridOrigin;
  /** 行主序三态字节；长度 = width × height。 */
  cells: Uint8Array;
  encoding: 'integer_array' | 'base64_u8';
  /** grid_snapshot 字段（grid_view 为 null）。 */
  window: BoundsM | null;
  requested_resolution_m: number | null;
  effective_resolution_m: number | null;
  degraded: boolean;
  scene_id: string | null;
  scene_version: string | null;
  request_seq: number | null;
  budget_cells: number | null;
  notes: string | null;
}

/** 夹具加载元信息（文件、哈希、字节数；由加载层填充）。 */
export interface GridFixtureInfo {
  file: string;
  sha256: string;
  bytes: number;
}

export interface GridViewState {
  status: 'idle' | 'loading' | 'ready' | 'error' | 'empty';
  layer: GridLayerData | null;
  info: GridFixtureInfo | null;
  error: string | null;
}

export interface GridCellRef {
  row: number;
  col: number;
}

/** 单元格检查结果（行列、状态、中心世界坐标）。 */
export interface GridCellInfo extends GridCellRef {
  state: GridCellState;
  center_x_m: number;
  center_y_m: number;
  in_bounds: boolean;
}

export const GRID_STATE_LABELS: Record<GridCellState, string> = {
  0: '空闲 free',
  1: '占据 occupied',
  2: '未知 unknown',
};

export const GRID_ORIGIN_POSE = (origin: GridOrigin): Pose2d => ({ ...origin });
