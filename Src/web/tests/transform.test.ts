import { describe, expect, it } from 'vitest';
import {
  boundsToScreenRect,
  metersPerPixel,
  screenToWorld,
  worldToScreen,
  type Viewport,
} from '../src/core/transform';

/** 确定性 LCG（不依赖外部随机源，回归可复现）。 */
function lcg(seed: number): () => number {
  let state = seed >>> 0;
  return () => {
    state = (state * 1664525 + 1013904223) >>> 0;
    return state / 0x100000000;
  };
}

const viewport: Viewport = {
  width_px: 1280,
  height_px: 720,
  scale_px_per_m: 0.3596,
  center_x_m: 813.32,
  center_y_m: 335.29,
};

describe('世界坐标 ↔ 屏幕坐标变换', () => {
  it('视图中心映射到画布中心', () => {
    const [sx, sy] = worldToScreen(viewport, viewport.center_x_m, viewport.center_y_m);
    expect(sx).toBeCloseTo(640, 9);
    expect(sy).toBeCloseTo(360, 9);
  });

  it('翻转 y 轴（北向上）', () => {
    const [sx0, sy0] = worldToScreen(viewport, viewport.center_x_m, viewport.center_y_m);
    const [sx1, sy1] = worldToScreen(viewport, viewport.center_x_m, viewport.center_y_m + 100);
    expect(sx1).toBeCloseTo(sx0, 9);
    expect(sy1).toBeLessThan(sy0);
    expect(sy0 - sy1).toBeCloseTo(100 * viewport.scale_px_per_m, 9);
  });

  it('比例尺一致性：米/像素 = 1/scale，两方向同比例', () => {
    expect(metersPerPixel(viewport)).toBeCloseTo(1 / viewport.scale_px_per_m, 12);
    const [ax, ay] = worldToScreen(viewport, 0, 0);
    const [bx] = worldToScreen(viewport, 100, 0);
    const [, cy] = worldToScreen(viewport, 0, 100);
    expect(bx - ax).toBeCloseTo(100 / metersPerPixel(viewport), 9);
    expect(ay - cy).toBeCloseTo(100 / metersPerPixel(viewport), 9);
  });

  it('往返误差远小于 1 像素（验收上限 ≤ 1 px）', () => {
    const random = lcg(20261010);
    let maxMeterError = 0;
    let maxPixelError = 0;
    for (let index = 0; index < 2000; index += 1) {
      const x = (random() - 0.5) * 4000;
      const y = (random() - 0.5) * 4000;
      const [sx, sy] = worldToScreen(viewport, x, y);
      const [rx, ry] = screenToWorld(viewport, sx, sy);
      maxMeterError = Math.max(maxMeterError, Math.abs(rx - x), Math.abs(ry - y));

      const px = random() * viewport.width_px;
      const py = random() * viewport.height_px;
      const [wx, wy] = screenToWorld(viewport, px, py);
      const [backX, backY] = worldToScreen(viewport, wx, wy);
      maxPixelError = Math.max(maxPixelError, Math.abs(backX - px), Math.abs(backY - py));
    }
    expect(maxMeterError).toBeLessThanOrEqual(1e-9);
    expect(maxPixelError).toBeLessThanOrEqual(1e-9);
    expect(maxPixelError).toBeLessThanOrEqual(1);
  });

  it('boundsToScreenRect 覆盖四角并保持尺寸', () => {
    const bounds = { min_x_m: -100, min_y_m: -50, max_x_m: 100, max_y_m: 50 };
    const rect = boundsToScreenRect(bounds, viewport);
    expect(rect.min_x_px).toBeLessThan(rect.max_x_px);
    expect(rect.min_y_px).toBeLessThan(rect.max_y_px);
    const [left, top] = worldToScreen(viewport, bounds.min_x_m, bounds.max_y_m);
    expect(rect.min_x_px).toBeCloseTo(left, 9);
    expect(rect.min_y_px).toBeCloseTo(top, 9);
    expect(rect.max_x_px - rect.min_x_px).toBeCloseTo(200 * viewport.scale_px_per_m, 9);
    expect(rect.max_y_px - rect.min_y_px).toBeCloseTo(100 * viewport.scale_px_per_m, 9);
  });
});
