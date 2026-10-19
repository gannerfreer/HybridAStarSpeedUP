import { worldToScreen, type Viewport } from '../transform';
import { cellCenter, cellCorners } from './coords';
import { clampOpacity } from './palette';
import type { GridTextureCache, GridTextureCanvas } from './texture';
import type { GridLayerData } from './types';

/**
 * 栅格图层绘制：纹理（cell 空间，1 像素 = 1 单元）通过仿射变换映射到屏幕，
 * 缩放/平移复用同一纹理（缓存命中时不重建）；矢量图层在栅格之上绘制（MapCanvas 顺序）。
 *
 * 变换推导（yaw=0 时退化为普通轴对齐贴图）：
 *   screen_x = s·cos·u − s·sin·v + (W/2 + (x0 − cx)·s)
 *   screen_y = −s·sin·u − s·cos·v + (H/2 − (y0 − cy)·s)
 * 其中 (u, v) 为栅格局部米坐标（u 沿列、v 沿行），image 行 0（底行）经 d = −s·cos
 * 翻转后显示在下（世界 +y 向上）。
 */

export type Affine = [number, number, number, number, number, number];

export function gridScreenTransform(viewport: Viewport, layer: GridLayerData): Affine {
  const scale = viewport.scale_px_per_m;
  const cos = Math.cos(layer.origin.yaw_rad);
  const sin = Math.sin(layer.origin.yaw_rad);
  const a = scale * cos;
  const b = -scale * sin;
  const c = -scale * sin;
  const d = -scale * cos;
  const e = viewport.width_px / 2 + (layer.origin.x_m - viewport.center_x_m) * scale;
  const f = viewport.height_px / 2 - (layer.origin.y_m - viewport.center_y_m) * scale;
  return [a, b, c, d, e, f];
}

/** Canvas 2D 最小接口（CanvasRenderingContext2D 结构兼容）。 */
export interface GridCanvas2DLike {
  save(): void;
  restore(): void;
  transform(a: number, b: number, c: number, d: number, e: number, f: number): void;
  drawImage(image: GridTextureCanvas | CanvasImageSource, dx: number, dy: number, dw: number, dh: number): void;
  globalAlpha: number;
  imageSmoothingEnabled: boolean;
}

/** 绘制栅格纹理；返回本帧是否重建了纹理（证据/基准记录用）。 */
export function drawGridLayer(
  ctx: GridCanvas2DLike,
  layer: GridLayerData,
  viewport: Viewport,
  opacity: number,
  cache: GridTextureCache,
): boolean {
  const alpha = clampOpacity(opacity);
  if (alpha <= 0) return false;
  const before = cache.rebuildCount;
  const texture = cache.get(layer);
  const [a, b, c, d, e, f] = gridScreenTransform(viewport, layer);
  ctx.save();
  ctx.globalAlpha = alpha;
  ctx.imageSmoothingEnabled = false;
  ctx.transform(a, b, c, d, e, f);
  ctx.drawImage(texture, 0, 0, layer.width * layer.resolution_m, layer.height * layer.resolution_m);
  ctx.restore();
  return cache.rebuildCount > before;
}

/** 选中单元轮廓（屏幕坐标；旋转原点按位姿换算）。 */
export function gridCellScreenCorners(
  layer: GridLayerData,
  viewport: Viewport,
  row: number,
  col: number,
): Array<[number, number]> {
  return cellCorners(layer, row, col).map(([x, y]) => worldToScreen(viewport, x, y));
}

/** 单元中心在屏幕上的坐标。 */
export function gridCellCenterScreen(
  layer: GridLayerData,
  viewport: Viewport,
  row: number,
  col: number,
): [number, number] {
  const [x, y] = cellCenter(layer, row, col);
  return worldToScreen(viewport, x, y);
}
