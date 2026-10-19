import { describe, expect, it } from 'vitest';
import { findRepoRoot, moduleDir, readSnapshotJson, resolveSnapshotSource } from '../src/core/fixture';
import { parseSnapshot } from '../src/core/parse';
import { chooseScaleBar, formatScaleBarLabel } from '../src/core/scalebar';
import { boundsToScreenRect, metersPerPixel, screenToWorld, worldToScreen, type Viewport } from '../src/core/transform';
import { fitBounds, MAX_SCALE_PX_PER_M, MIN_SCALE_PX_PER_M, panBy, zoomAt } from '../src/core/viewport';

const repoRoot = findRepoRoot(moduleDir(import.meta.url));
const source = resolveSnapshotSource(repoRoot);
const snapshot = parseSnapshot(readSnapshotJson(repoRoot, source).value);

describe('fitBounds 按 bounds 自动取景', () => {
  it('仓库快照 bounds 完整落入画布并保留边距', () => {
    const width = 1280;
    const height = 720;
    const padding = 24;
    const viewport = fitBounds(snapshot.bounds_m, width, height, padding);
    const rect = boundsToScreenRect(snapshot.bounds_m, viewport);
    expect(rect.min_x_px).toBeGreaterThanOrEqual(padding - 1e-6);
    expect(rect.min_y_px).toBeGreaterThanOrEqual(padding - 1e-6);
    expect(rect.max_x_px).toBeLessThanOrEqual(width - padding + 1e-6);
    expect(rect.max_y_px).toBeLessThanOrEqual(height - padding + 1e-6);
  });

  it('较紧的一维贴满可用空间，bounds 居中', () => {
    const width = 1280;
    const height = 720;
    const padding = 24;
    const viewport = fitBounds(snapshot.bounds_m, width, height, padding);
    const rect = boundsToScreenRect(snapshot.bounds_m, viewport);
    const availableWidth = width - 2 * padding;
    const availableHeight = height - 2 * padding;
    const fillWidth = (rect.max_x_px - rect.min_x_px) / availableWidth;
    const fillHeight = (rect.max_y_px - rect.min_y_px) / availableHeight;
    expect(Math.max(fillWidth, fillHeight)).toBeCloseTo(1, 6);
    expect(Math.min(fillWidth, fillHeight)).toBeLessThanOrEqual(1 + 1e-9);
    expect((rect.min_x_px + rect.max_x_px) / 2).toBeCloseTo(width / 2, 6);
    expect((rect.min_y_px + rect.max_y_px) / 2).toBeCloseTo(height / 2, 6);
  });

  it('比例各向同性（x/y 同尺度）', () => {
    const viewport = fitBounds(snapshot.bounds_m, 800, 600, 10);
    const [x0, y0] = worldToScreen(viewport, 0, 0);
    const [x1] = worldToScreen(viewport, 100, 0);
    const [, y1] = worldToScreen(viewport, 0, 100);
    expect(x1 - x0).toBeCloseTo(y0 - y1, 9);
  });

  it('退化（单点）bounds 不产生非有限比例', () => {
    const viewport = fitBounds({ min_x_m: 5, min_y_m: 5, max_x_m: 5, max_y_m: 5 }, 640, 480, 16);
    expect(Number.isFinite(viewport.scale_px_per_m)).toBe(true);
    expect(viewport.scale_px_per_m).toBe(MAX_SCALE_PX_PER_M);
    expect(viewport.center_x_m).toBe(5);
    expect(viewport.center_y_m).toBe(5);
  });
});

describe('zoomAt 光标中心缩放', () => {
  const base: Viewport = fitBounds(snapshot.bounds_m, 1280, 720, 24);

  it('锚点下的世界点在缩放前后保持同一像素位置', () => {
    const anchors: Array<[number, number]> = [
      [0, 0],
      [640, 360],
      [1279, 719],
    ];
    for (const [anchorX, anchorY] of anchors) {
      const [worldX, worldY] = screenToWorld(base, anchorX, anchorY);
      const zoomed = zoomAt(base, 1.7, anchorX, anchorY);
      const [sx, sy] = worldToScreen(zoomed, worldX, worldY);
      expect(sx).toBeCloseTo(anchorX, 6);
      expect(sy).toBeCloseTo(anchorY, 6);
      expect(zoomed.scale_px_per_m).toBeCloseTo(base.scale_px_per_m * 1.7, 9);
    }
  });

  it('缩放上下限被夹紧', () => {
    expect(zoomAt(base, 1e9, 640, 360).scale_px_per_m).toBe(MAX_SCALE_PX_PER_M);
    expect(zoomAt(base, 1e-9, 640, 360).scale_px_per_m).toBe(MIN_SCALE_PX_PER_M);
  });

  it('非法 factor 返回原视图', () => {
    expect(zoomAt(base, 0, 1, 1)).toEqual(base);
    expect(zoomAt(base, Number.NaN, 1, 1)).toEqual(base);
  });
});

describe('panBy 拖拽平移', () => {
  it('内容跟随指针位移', () => {
    const base: Viewport = fitBounds(snapshot.bounds_m, 1280, 720, 24);
    const [x0, y0] = worldToScreen(base, 100, 100);
    const moved = panBy(base, 50, -30);
    const [x1, y1] = worldToScreen(moved, 100, 100);
    expect(x1 - x0).toBeCloseTo(50, 6);
    expect(y1 - y0).toBeCloseTo(-30, 6);
  });

  it('两次平移与合并平移等价', () => {
    const base: Viewport = fitBounds(snapshot.bounds_m, 1280, 720, 24);
    const chained = panBy(panBy(base, 20, 10), -5, 40);
    const combined = panBy(base, 15, 50);
    expect(chained.center_x_m).toBeCloseTo(combined.center_x_m, 9);
    expect(chained.center_y_m).toBeCloseTo(combined.center_y_m, 9);
  });
});

describe('chooseScaleBar 比例尺', () => {
  it('仓库快照取景下的比例尺宽度不超过上限且接近上限', () => {
    const mpp = metersPerPixel(fitBounds(snapshot.bounds_m, 1280, 720, 24));
    const bar = chooseScaleBar(mpp, 140);
    expect(bar.width_px).toBeLessThanOrEqual(140 + 1e-9);
    expect(bar.width_px).toBeGreaterThan(140 / 2.5);
  });

  it('固定米/像素的确定性输出', () => {
    expect(chooseScaleBar(0.5, 140)).toEqual({ length_m: 50, width_px: 100, label: '50 m' });
    expect(chooseScaleBar(2.5, 140)).toEqual({ length_m: 200, width_px: 80, label: '200 m' });
    expect(chooseScaleBar(0.05, 140)).toEqual({ length_m: 5, width_px: 100, label: '5 m' });
    expect(chooseScaleBar(1000, 140)).toEqual({ length_m: 100000, width_px: 100, label: '100 km' });
    expect(chooseScaleBar(0.005, 140)).toEqual({ length_m: 0.5, width_px: 100, label: '50 cm' });
  });

  it('标签格式', () => {
    expect(formatScaleBarLabel(1)).toBe('1 m');
    expect(formatScaleBarLabel(1000)).toBe('1 km');
    expect(formatScaleBarLabel(0.2)).toBe('20 cm');
  });

  it('非法输入抛出 RangeError', () => {
    expect(() => chooseScaleBar(0)).toThrow(RangeError);
    expect(() => chooseScaleBar(Number.NaN)).toThrow(RangeError);
    expect(() => chooseScaleBar(1, -5)).toThrow(RangeError);
  });
});
