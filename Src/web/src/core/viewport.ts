import type { BoundsM } from './types';
import { screenToWorld, type Viewport } from './transform';

/** 缩放下限/上限：0.02 px/m（50 m/px）～ 400 px/m（2.5 mm/px）。 */
export const MIN_SCALE_PX_PER_M = 0.02;
export const MAX_SCALE_PX_PER_M = 400;

export function clampScale(scale: number): number {
  if (!Number.isFinite(scale) || scale <= 0) return MIN_SCALE_PX_PER_M;
  return Math.min(MAX_SCALE_PX_PER_M, Math.max(MIN_SCALE_PX_PER_M, scale));
}

/**
 * 按世界包围盒自动取景：保持各向同性比例（单一 scale），留出 `paddingPx` 边距，
 * 使整个 bounds 完整可见。比例先按较紧的一维取 min，再夹到允许范围。
 */
export function fitBounds(bounds: BoundsM, widthPx: number, heightPx: number, paddingPx = 16): Viewport {
  const width = Math.max(1, widthPx);
  const height = Math.max(1, heightPx);
  const padding = Math.max(0, Math.min(paddingPx, Math.min(width, height) / 2 - 1));
  const availableWidth = Math.max(1, width - 2 * padding);
  const availableHeight = Math.max(1, height - 2 * padding);
  const spanX = Math.max(bounds.max_x_m - bounds.min_x_m, 1e-9);
  const spanY = Math.max(bounds.max_y_m - bounds.min_y_m, 1e-9);
  const scale = clampScale(Math.min(availableWidth / spanX, availableHeight / spanY));
  return {
    width_px: width,
    height_px: height,
    scale_px_per_m: scale,
    center_x_m: (bounds.min_x_m + bounds.max_x_m) / 2,
    center_y_m: (bounds.min_y_m + bounds.max_y_m) / 2,
  };
}

/** 以屏幕锚点为中心缩放：缩放前后锚点下的世界点保持在同一像素位置。 */
export function zoomAt(viewport: Viewport, factor: number, anchorXpx: number, anchorYpx: number): Viewport {
  if (!Number.isFinite(factor) || factor <= 0) return viewport;
  const scale = clampScale(viewport.scale_px_per_m * factor);
  const [worldX, worldY] = screenToWorld(viewport, anchorXpx, anchorYpx);
  return {
    ...viewport,
    scale_px_per_m: scale,
    center_x_m: worldX - (anchorXpx - viewport.width_px / 2) / scale,
    center_y_m: worldY + (anchorYpx - viewport.height_px / 2) / scale,
  };
}

/** 拖拽平移：内容跟随指针位移 (dxPx, dyPx)，即视图中心反向移动。 */
export function panBy(viewport: Viewport, dxPx: number, dyPx: number): Viewport {
  return {
    ...viewport,
    center_x_m: viewport.center_x_m - dxPx / viewport.scale_px_per_m,
    center_y_m: viewport.center_y_m + dyPx / viewport.scale_px_per_m,
  };
}
