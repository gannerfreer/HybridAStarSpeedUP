import type { GridCellState } from './types';

/**
 * 三态固定颜色表（协议 §4.1 纹理生成：解码 uint8 → RGBA ImageData）。
 * 颜色只用于显示区分，不构成算法语义；图例（空闲/占据/未知）与纹理共用本表。
 */
export interface GridRgba {
  r: number;
  g: number;
  b: number;
  a: number;
  css: string;
}

export const GRID_COLORS: Record<GridCellState, GridRgba> = {
  0: { r: 52, g: 211, b: 153, a: 110, css: 'rgba(52, 211, 153, 0.55)' },
  1: { r: 248, g: 113, b: 113, a: 165, css: 'rgba(248, 113, 113, 0.75)' },
  2: { r: 148, g: 163, b: 184, a: 130, css: 'rgba(148, 163, 184, 0.62)' },
} as const;

/**
 * 规划实际场景叠加层颜色表（X-W2）：只强调"规划快照中的占据单元"，
 * 空闲透明（避免遮住当前预览/底图），未知为琥珀色；与三态表视觉区分。
 */
export const PLAN_SCENE_COLORS: Record<GridCellState, GridRgba> = {
  0: { r: 0, g: 0, b: 0, a: 0, css: 'transparent' },
  1: { r: 217, g: 70, b: 239, a: 205, css: 'rgba(217, 70, 239, 0.80)' },
  2: { r: 251, g: 191, b: 36, a: 150, css: 'rgba(251, 191, 36, 0.59)' },
} as const;

export const GRID_LAYER_DEFAULT_OPACITY = 0.7;

export function clampOpacity(value: number): number {
  if (!Number.isFinite(value)) return GRID_LAYER_DEFAULT_OPACITY;
  return Math.min(1, Math.max(0, value));
}
