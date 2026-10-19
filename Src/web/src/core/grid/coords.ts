import type { BoundsM } from '../types';
import type { GridCellInfo, GridCellState, GridLayerData, GridOrigin } from './types';

/**
 * 栅格坐标换算（协议 §2.3）：`origin` 是 cell (0,0) 左下角位姿；
 * cell (r,c) 中心 = origin + R(yaw)·((c+0.5)·res, (r+0.5)·res)；
 * row 沿 origin 的 +y 递增、col 沿 +x 递增。旋转原点（G1/G2）必须按位姿换算，
 * 不得按轴对齐假设。
 */

export function cellCenter(layer: GridLayerData, row: number, col: number): [number, number] {
  return cellCenterOf(layer.origin, layer.resolution_m, row, col);
}

export function cellCenterOf(
  origin: GridOrigin,
  resolution_m: number,
  row: number,
  col: number,
): [number, number] {
  const cos = Math.cos(origin.yaw_rad);
  const sin = Math.sin(origin.yaw_rad);
  const dx = (col + 0.5) * resolution_m;
  const dy = (row + 0.5) * resolution_m;
  return [origin.x_m + cos * dx - sin * dy, origin.y_m + sin * dx + cos * dy];
}

/** cell 四角（世界坐标；按 origin 局部系逆时针）。 */
export function cellCorners(layer: GridLayerData, row: number, col: number): Array<[number, number]> {
  const { origin, resolution_m: resolution } = layer;
  const cos = Math.cos(origin.yaw_rad);
  const sin = Math.sin(origin.yaw_rad);
  const local: Array<[number, number]> = [
    [col * resolution, row * resolution],
    [(col + 1) * resolution, row * resolution],
    [(col + 1) * resolution, (row + 1) * resolution],
    [col * resolution, (row + 1) * resolution],
  ];
  return local.map(([x, y]) => [origin.x_m + cos * x - sin * y, origin.y_m + sin * x + cos * y]);
}

export interface WorldCellHit {
  row: number;
  col: number;
  in_bounds: boolean;
}

/** 世界点 → 单元：col = floor(局部 x/res)、row = floor(局部 y/res)；右/上边界属于下一个单元。 */
export function worldToCell(layer: GridLayerData, x_m: number, y_m: number): WorldCellHit {
  const { origin, resolution_m: resolution } = layer;
  const cos = Math.cos(origin.yaw_rad);
  const sin = Math.sin(origin.yaw_rad);
  const dx = x_m - origin.x_m;
  const dy = y_m - origin.y_m;
  const localX = cos * dx + sin * dy;
  const localY = -sin * dx + cos * dy;
  const col = Math.floor(localX / resolution);
  const row = Math.floor(localY / resolution);
  const inBounds = row >= 0 && row < layer.height && col >= 0 && col < layer.width;
  return { row, col, in_bounds: inBounds };
}

export function cellState(layer: GridLayerData, row: number, col: number): GridCellState | null {
  if (row < 0 || row >= layer.height || col < 0 || col >= layer.width) return null;
  return layer.cells[row * layer.width + col] as GridCellState;
}

/** 单元格检查信息（选中单元查看行列/世界坐标/状态）。 */
export function cellInfo(layer: GridLayerData, row: number, col: number): GridCellInfo {
  const [x, y] = cellCenter(layer, row, col);
  const state = cellState(layer, row, col);
  return {
    row,
    col,
    state: (state ?? 2) as GridCellState,
    center_x_m: x,
    center_y_m: y,
    in_bounds: state !== null,
  };
}

/** 栅格覆盖的世界包围盒（含旋转原点的四角）。 */
export function gridWorldBounds(layer: GridLayerData): BoundsM {
  const points: Array<[number, number]> = [
    ...cellCorners(layer, 0, 0),
    ...cellCorners(layer, 0, layer.width - 1),
    ...cellCorners(layer, layer.height - 1, layer.width - 1),
    ...cellCorners(layer, layer.height - 1, 0),
  ];
  return {
    min_x_m: Math.min(...points.map((point) => point[0])),
    min_y_m: Math.min(...points.map((point) => point[1])),
    max_x_m: Math.max(...points.map((point) => point[0])),
    max_y_m: Math.max(...points.map((point) => point[1])),
  };
}

/** 三态计数（面板与证据使用）。 */
export function countStates(layer: GridLayerData): { free: number; occupied: number; unknown: number } {
  let free = 0;
  let occupied = 0;
  let unknown = 0;
  for (const state of layer.cells) {
    if (state === 0) free += 1;
    else if (state === 1) occupied += 1;
    else unknown += 1;
  }
  return { free, occupied, unknown };
}
