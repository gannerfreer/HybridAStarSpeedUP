import type { BoundsM } from './types';

/**
 * 世界坐标（米，x 向东、y 向北，右手系）↔ 屏幕坐标（CSS 像素，x 向右、y 向下）。
 *
 * 视图状态 `Viewport` 只包含 4 个量：画布尺寸（CSS px）、比例（px/m）与视图中心
 * （世界米）。高 DPI 由 Canvas 执行层一次性 `setTransform(dpr, ...)` 处理，
 * 本层坐标全部是 CSS 像素，因此浏览器往返误差与 DPR 无关。
 */
export interface Viewport {
  width_px: number;
  height_px: number;
  /** CSS 像素/米，恒 > 0。 */
  scale_px_per_m: number;
  center_x_m: number;
  center_y_m: number;
}

/** 当前缩放下的地面采样距离（米/像素）。 */
export function metersPerPixel(viewport: Viewport): number {
  return 1 / viewport.scale_px_per_m;
}

export function worldToScreen(viewport: Viewport, x_m: number, y_m: number): [number, number] {
  const sx = viewport.width_px / 2 + (x_m - viewport.center_x_m) * viewport.scale_px_per_m;
  const sy = viewport.height_px / 2 - (y_m - viewport.center_y_m) * viewport.scale_px_per_m;
  return [sx, sy];
}

export function screenToWorld(viewport: Viewport, sx_px: number, sy_px: number): [number, number] {
  const x_m = viewport.center_x_m + (sx_px - viewport.width_px / 2) / viewport.scale_px_per_m;
  const y_m = viewport.center_y_m - (sy_px - viewport.height_px / 2) / viewport.scale_px_per_m;
  return [x_m, y_m];
}

export interface ScreenRect {
  min_x_px: number;
  min_y_px: number;
  max_x_px: number;
  max_y_px: number;
}

/** 世界包围盒在屏幕上的矩形（y 轴翻转，min_y_px 对应北边界）。 */
export function boundsToScreenRect(bounds: BoundsM, viewport: Viewport): ScreenRect {
  const [x0, y0] = worldToScreen(viewport, bounds.min_x_m, bounds.max_y_m);
  const [x1, y1] = worldToScreen(viewport, bounds.max_x_m, bounds.min_y_m);
  return {
    min_x_px: Math.min(x0, x1),
    max_x_px: Math.max(x0, x1),
    min_y_px: Math.min(y0, y1),
    max_y_px: Math.max(y0, y1),
  };
}
