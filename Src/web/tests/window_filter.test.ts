import { describe, expect, it } from 'vitest';
import {
  boundsIntersect,
  filterSnapshotToWindow,
  normalizeBounds,
  pointsBounds,
} from '../src/core/window';
import { area, makeSnapshot, point, straightLanelet } from './fixtures';

describe('窗口查询（本地回退与坐标变换）', () => {
  it('normalizeBounds 归一化任意两点', () => {
    expect(normalizeBounds(10, 10, 0, 0)).toEqual({ min_x_m: 0, min_y_m: 0, max_x_m: 10, max_y_m: 10 });
    expect(normalizeBounds(0, 0, -5, 5)).toEqual({ min_x_m: -5, min_y_m: 0, max_x_m: 0, max_y_m: 5 });
  });

  it('boundsIntersect / pointsBounds', () => {
    const a = { min_x_m: 0, min_y_m: 0, max_x_m: 10, max_y_m: 10 };
    expect(boundsIntersect(a, { min_x_m: 5, min_y_m: 5, max_x_m: 6, max_y_m: 6 })).toBe(true);
    expect(boundsIntersect(a, { min_x_m: 11, min_y_m: 0, max_x_m: 12, max_y_m: 1 })).toBe(false);
    expect(pointsBounds([])).toBeNull();
    expect(pointsBounds([point('a', 1, 2), point('b', -1, 5)])).toEqual({
      min_x_m: -1,
      min_y_m: 2,
      max_x_m: 1,
      max_y_m: 5,
    });
  });

  it('filterSnapshotToWindow：包围盒相交过滤（未裁剪，标注近似）', () => {
    const snapshot = makeSnapshot({
      lanelets: [straightLanelet('100', 0, 10), straightLanelet('200', 20, 30)],
      areas: [area('20', [point('a', 12, 0), point('b', 16, 0), point('c', 16, 4), point('d', 12, 4)])],
    });
    const result = filterSnapshotToWindow(snapshot, { min_x_m: 5, min_y_m: -5, max_x_m: 25, max_y_m: 5 });
    expect(result.lanelet_ids).toEqual(['100', '200']);
    expect(result.area_ids).toEqual(['20']);
    expect(result.approximate).toBe(true);

    const empty = filterSnapshotToWindow(snapshot, { min_x_m: 100, min_y_m: 100, max_x_m: 110, max_y_m: 110 });
    expect(empty.lanelet_ids).toEqual([]);
    expect(empty.area_ids).toEqual([]);
  });
});
