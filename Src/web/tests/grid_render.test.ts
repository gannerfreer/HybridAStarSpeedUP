import { describe, expect, it } from 'vitest';
import { cellCenter } from '../src/core/grid/coords';
import {
  drawGridLayer,
  gridCellScreenCorners,
  gridScreenTransform,
  type Affine,
  type GridCanvas2DLike,
} from '../src/core/grid/render';
import { GridTextureCache, type GridTextureCanvas } from '../src/core/grid/texture';
import type { GridLayerData } from '../src/core/grid/types';
import { worldToScreen, type Viewport } from '../src/core/transform';

function makeLayer(overrides: Partial<GridLayerData> = {}): GridLayerData {
  return {
    kind: 'grid_view',
    source: 'grid_view',
    map_id: 'grid_test',
    map_version: 'v6',
    width: 4,
    height: 4,
    resolution_m: 0.5,
    origin: { x_m: 10, y_m: -5, yaw_rad: Math.PI / 2 },
    cells: new Uint8Array(16),
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

const viewport: Viewport = {
  width_px: 800,
  height_px: 600,
  scale_px_per_m: 40,
  center_x_m: 9,
  center_y_m: -4,
};

function applyAffine(affine: Affine, x: number, y: number): [number, number] {
  const [a, b, c, d, e, f] = affine;
  return [a * x + c * y + e, b * x + d * y + f];
}

function fakeCanvas(): { ctx: GridCanvas2DLike; calls: string[]; drawn: Array<[number, number, number, number]> } {
  const calls: string[] = [];
  const drawn: Array<[number, number, number, number]> = [];
  const ctx: GridCanvas2DLike = {
    globalAlpha: 1,
    imageSmoothingEnabled: true,
    save() {
      calls.push('save');
    },
    restore() {
      calls.push('restore');
    },
    transform() {
      calls.push('transform');
    },
    drawImage(_image: GridTextureCanvas, dx: number, dy: number, dw: number, dh: number) {
      calls.push('drawImage');
      drawn.push([dx, dy, dw, dh]);
    },
  };
  return { ctx, calls, drawn };
}

function textureFactory(): () => GridTextureCanvas {
  return () => ({
    width: 4,
    height: 4,
    getContext() {
      return { putImageData: () => undefined };
    },
  });
}

describe('栅格渲染变换与绘制', () => {
  it('yaw=0 时仿射变换与 worldToScreen 一致', () => {
    const layer = makeLayer({ origin: { x_m: 0, y_m: 0, yaw_rad: 0 } });
    const affine = gridScreenTransform(viewport, layer);
    for (const [row, col] of [
      [0, 0],
      [3, 3],
      [1, 2],
    ]) {
      const [x, y] = cellCenter(layer, row, col);
      const expected = worldToScreen(viewport, x, y);
      const actual = applyAffine(affine, (col + 0.5) * layer.resolution_m, (row + 0.5) * layer.resolution_m);
      expect(actual[0]).toBeCloseTo(expected[0], 9);
      expect(actual[1]).toBeCloseTo(expected[1], 9);
    }
  });

  it('旋转原点（G1）仿射变换与 cellCenter 一致', () => {
    const layer = makeLayer();
    const affine = gridScreenTransform(viewport, layer);
    const cases: Array<[number, number]> = [
      [0, 0],
      [1, 2],
      [3, 3],
    ];
    for (const [row, col] of cases) {
      const [x, y] = cellCenter(layer, row, col);
      const expected = worldToScreen(viewport, x, y);
      const actual = applyAffine(affine, (col + 0.5) * layer.resolution_m, (row + 0.5) * layer.resolution_m);
      expect(actual[0]).toBeCloseTo(expected[0], 6);
      expect(actual[1]).toBeCloseTo(expected[1], 6);
    }
  });

  it('drawGridLayer：保存/变换/绘制/恢复，重建仅首次', () => {
    const layer = makeLayer();
    const cache = new GridTextureCache(textureFactory());
    const first = fakeCanvas();
    const rebuiltFirst = drawGridLayer(first.ctx, layer, viewport, 0.7, cache);
    expect(rebuiltFirst).toBe(true);
    expect(first.calls).toEqual(['save', 'transform', 'drawImage', 'restore']);
    expect(first.drawn[0]).toEqual([0, 0, 2, 2]);
    expect(first.ctx.globalAlpha).toBeCloseTo(0.7, 9);
    expect(first.ctx.imageSmoothingEnabled).toBe(false);
    const second = fakeCanvas();
    const rebuiltSecond = drawGridLayer(second.ctx, layer, viewport, 0.5, cache);
    expect(rebuiltSecond).toBe(false);
    expect(cache.rebuildCount).toBe(1);
  });

  it('drawGridLayer：透明度 0 时不绘制', () => {
    const layer = makeLayer();
    const cache = new GridTextureCache(textureFactory());
    const { ctx, calls } = fakeCanvas();
    expect(drawGridLayer(ctx, layer, viewport, 0, cache)).toBe(false);
    expect(calls).toEqual([]);
  });

  it('选中单元轮廓与单元四角一致', () => {
    const layer = makeLayer();
    const corners = gridCellScreenCorners(layer, viewport, 0, 0);
    expect(corners.length).toBe(4);
    const expected = [
      worldToScreen(viewport, 10, -5),
      worldToScreen(viewport, 10, -4.5),
      worldToScreen(viewport, 9.5, -4.5),
      worldToScreen(viewport, 9.5, -5),
    ];
    corners.forEach((corner, index) => {
      expect(corner[0]).toBeCloseTo(expected[index][0], 6);
      expect(corner[1]).toBeCloseTo(expected[index][1], 6);
    });
  });
});
