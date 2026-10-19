import { GRID_COLORS, PLAN_SCENE_COLORS, type GridRgba } from './palette';
import type { GridCellState, GridLayerData } from './types';

/**
 * 纹理生成（协议 §4.1）：解码 uint8 → RGBA ImageData（固定颜色表）→
 * putImageData 到 offscreen canvas。仅在栅格变化时重建（缓存键 = 图层对象引用）。
 */

export interface GridImageData {
  width: number;
  height: number;
  data: Uint8ClampedArray;
}

/** 按颜色表生成纹理（三态固定表 / 规划场景叠加表共用）。 */
export function buildGridImageDataWithPalette(
  layer: GridLayerData,
  palette: Record<GridCellState, GridRgba>,
): GridImageData {
  const { width, height, cells } = layer;
  const data = new Uint8ClampedArray(width * height * 4);
  for (let index = 0; index < cells.length; index += 1) {
    const color = palette[cells[index] as GridCellState];
    const offset = index * 4;
    data[offset] = color.r;
    data[offset + 1] = color.g;
    data[offset + 2] = color.b;
    data[offset + 3] = color.a;
  }
  return { width, height, data };
}

export function buildGridImageData(layer: GridLayerData): GridImageData {
  return buildGridImageDataWithPalette(layer, GRID_COLORS);
}

/** 规划实际场景叠加纹理：空闲透明、占据品红、未知琥珀（与预览显式区分）。 */
export function buildPlanSceneImageData(layer: GridLayerData): GridImageData {
  return buildGridImageDataWithPalette(layer, PLAN_SCENE_COLORS);
}

/** offscreen canvas 最小接口（浏览器 OffscreenCanvas/HTMLCanvasElement 结构兼容）。 */
export interface GridTextureCanvas {
  width: number;
  height: number;
  getContext(type: '2d'): { putImageData(image: GridImageData, x: number, y: number): void } | null;
}

export type GridTextureFactory = (width: number, height: number) => GridTextureCanvas;

export function defaultGridTextureFactory(width: number, height: number): GridTextureCanvas {
  if (typeof OffscreenCanvas !== 'undefined') {
    return new OffscreenCanvas(width, height) as unknown as GridTextureCanvas;
  }
  if (typeof document !== 'undefined') {
    const canvas = document.createElement('canvas');
    canvas.width = width;
    canvas.height = height;
    return canvas as unknown as GridTextureCanvas;
  }
  throw new Error('当前环境没有可用的 canvas（需要 OffscreenCanvas 或 DOM）');
}

/**
 * 纹理缓存：同一图层对象命中缓存；图层变化才重建（缩放/平移复用）。
 * `rebuildCount` 供证据/基准记录（帧内重建次数应为 0）。
 */
export class GridTextureCache {
  private key: GridLayerData | null = null;

  private canvas: GridTextureCanvas | null = null;

  private rebuilds = 0;

  private lastRebuildAt: GridLayerData | null = null;

  constructor(
    private readonly factory: GridTextureFactory = defaultGridTextureFactory,
    private readonly builder: (layer: GridLayerData) => GridImageData = buildGridImageData,
  ) {}

  get(layer: GridLayerData): GridTextureCanvas {
    if (this.key === layer && this.canvas) return this.canvas;
    const image = this.builder(layer);
    const canvas = this.factory(layer.width, layer.height);
    const context = canvas.getContext('2d');
    if (!context) throw new Error('纹理 canvas 缺少 2d 上下文');
    if (typeof ImageData !== 'undefined') {
      // 浏览器 putImageData 只接受真正的 ImageData 实例（共享同一缓冲，不复制）。
      context.putImageData(
        new ImageData(image.data as unknown as ImageDataArray, image.width, image.height),
        0,
        0,
      );
    } else {
      // Node 测试注入的结构化假实现。
      context.putImageData(image, 0, 0);
    }
    this.key = layer;
    this.canvas = canvas;
    this.lastRebuildAt = layer;
    this.rebuilds += 1;
    return canvas;
  }

  get rebuildCount(): number {
    return this.rebuilds;
  }

  /** 最近一次重建对应的图层（用于判断本帧是否发生了重建）。 */
  get lastRebuiltLayer(): GridLayerData | null {
    return this.lastRebuildAt;
  }

  invalidate(): void {
    this.key = null;
    this.canvas = null;
  }
}
