import { describe, expect, it } from 'vitest';
import { GRID_COLORS, clampOpacity } from '../src/core/grid/palette';
import {
  buildGridImageData,
  GridTextureCache,
  type GridTextureCanvas,
} from '../src/core/grid/texture';
import type { GridLayerData } from '../src/core/grid/types';

function makeLayer(states: number[], overrides: Partial<GridLayerData> = {}): GridLayerData {
  return {
    kind: 'grid_view',
    source: 'grid_view',
    map_id: 'grid_test',
    map_version: 'v6',
    width: 2,
    height: 2,
    resolution_m: 0.5,
    origin: { x_m: 0, y_m: 0, yaw_rad: 0 },
    cells: new Uint8Array(states),
    encoding: 'integer_array',
    window: null,
    requested_resolution_m: null,
    effective_resolution_m: null,
    degraded: false,
    scene_id: null,
    scene_version: null,
    request_seq: null,
    budget_cells: null,
    notes: null,
    ...overrides,
  };
}

interface RecordedCanvas extends GridTextureCanvas {
  puts: Array<{ width: number; height: number; data: Uint8ClampedArray }>;
}

function recordingFactory(log: Array<[number, number]>) {
  return (width: number, height: number): RecordedCanvas => {
    log.push([width, height]);
    const canvas: RecordedCanvas = {
      width,
      height,
      puts: [],
      getContext() {
        return {
          putImageData: (image: { width: number; height: number; data: Uint8ClampedArray }) => {
            canvas.puts.push({ width: image.width, height: image.height, data: image.data });
          },
        };
      },
    };
    return canvas;
  };
}

describe('栅格纹理与缓存', () => {
  it('三态固定颜色表 RGBA', () => {
    const image = buildGridImageData(makeLayer([0, 1, 2, 0]));
    expect(image.width).toBe(2);
    expect(image.height).toBe(2);
    expect(image.data.length).toBe(16);
    expect([image.data[0], image.data[1], image.data[2], image.data[3]]).toEqual([
      GRID_COLORS[0].r,
      GRID_COLORS[0].g,
      GRID_COLORS[0].b,
      GRID_COLORS[0].a,
    ]);
    expect(image.data[4]).toBe(GRID_COLORS[1].r);
    expect(image.data[8]).toBe(GRID_COLORS[2].r);
    expect(GRID_COLORS[2].a).toBeGreaterThan(0);
  });

  it('纹理缓存仅在同一图层对象内复用；图层变化重建', () => {
    const log: Array<[number, number]> = [];
    const cache = new GridTextureCache(recordingFactory(log));
    const first = makeLayer([0, 0, 0, 0]);
    const canvasA = cache.get(first);
    expect(cache.rebuildCount).toBe(1);
    expect(cache.get(first)).toBe(canvasA);
    expect(cache.rebuildCount).toBe(1);
    const second = makeLayer([1, 1, 1, 1]);
    cache.get(second);
    expect(cache.rebuildCount).toBe(2);
    expect(log).toEqual([
      [2, 2],
      [2, 2],
    ]);
  });

  it('putImageData 写入完整尺寸图像', () => {
    const log: Array<[number, number]> = [];
    const cache = new GridTextureCache(recordingFactory(log));
    const canvas = cache.get(makeLayer([0, 1, 2, 0])) as RecordedCanvas;
    expect(canvas.puts.length).toBe(1);
    expect(canvas.puts[0].width).toBe(2);
    expect(canvas.puts[0].height).toBe(2);
    expect(canvas.puts[0].data.length).toBe(16);
  });

  it('透明度夹紧', () => {
    expect(clampOpacity(0.5)).toBe(0.5);
    expect(clampOpacity(-1)).toBe(0);
    expect(clampOpacity(2)).toBe(1);
    expect(clampOpacity(Number.NaN)).toBeGreaterThan(0);
  });
});
