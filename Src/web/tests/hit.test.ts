import { describe, expect, it } from 'vitest';
import {
  distanceToPolyline,
  distanceToSegment,
  hitTestSnapshot,
  laneletPolygon,
  pointInRing,
  pointInRingsEvenOdd,
  sortIds,
} from '../src/core/hit';
import { area, makeSnapshot, point, straightLanelet } from './fixtures';

describe('几何命中测试', () => {
  it('pointInRing：矩形内外', () => {
    const square: Array<[number, number]> = [
      [0, 0],
      [10, 0],
      [10, 10],
      [0, 10],
    ];
    expect(pointInRing(square, 5, 5)).toBe(true);
    expect(pointInRing(square, 15, 5)).toBe(false);
    expect(pointInRing(square, -1, 5)).toBe(false);
    expect(pointInRing(square, 5, 12)).toBe(false);
  });

  it('pointInRingsEvenOdd：内环镂空', () => {
    const outer: Array<[number, number]> = [
      [0, 0],
      [10, 0],
      [10, 10],
      [0, 10],
    ];
    const hole: Array<[number, number]> = [
      [3, 3],
      [7, 3],
      [7, 7],
      [3, 7],
    ];
    expect(pointInRingsEvenOdd([outer, hole], 1, 1)).toBe(true);
    expect(pointInRingsEvenOdd([outer, hole], 5, 5)).toBe(false);
    expect(pointInRingsEvenOdd([outer, hole], 12, 5)).toBe(false);
  });

  it('laneletPolygon：左边界 + 反向右边界', () => {
    const item = straightLanelet('1', 0, 10);
    const polygon = laneletPolygon(item);
    expect(polygon).toHaveLength(4);
    expect(polygon[0]).toEqual([0, 2]);
    expect(polygon[1]).toEqual([10, 2]);
    expect(polygon[2]).toEqual([10, -2]);
    expect(polygon[3]).toEqual([0, -2]);
  });

  it('distanceToSegment / distanceToPolyline', () => {
    expect(distanceToSegment(5, 3, 0, 0, 10, 0)).toBeCloseTo(3, 12);
    expect(distanceToSegment(-5, 0, 0, 0, 10, 0)).toBeCloseTo(5, 12);
    expect(distanceToSegment(15, 0, 0, 0, 10, 0)).toBeCloseTo(5, 12);
    expect(distanceToPolyline([[0, 0], [10, 0], [10, 10]], 12, 5)).toBeCloseTo(2, 12);
    expect(distanceToPolyline([], 0, 0)).toBe(Number.POSITIVE_INFINITY);
  });

  it('hitTestSnapshot：多边形内部命中 lanelet', () => {
    const snapshot = makeSnapshot({ lanelets: [straightLanelet('100', 0, 10), straightLanelet('200', 20, 30)] });
    const hit = hitTestSnapshot(snapshot, 5, 0);
    expect(hit.lanelet_ids).toEqual(['100']);
    expect(hit.area_ids).toEqual([]);
    expect(hitTestSnapshot(snapshot, 25, 1).lanelet_ids).toEqual(['200']);
    expect(hitTestSnapshot(snapshot, 15, 0).lanelet_ids).toEqual([]);
  });

  it('hitTestSnapshot：中心线容差回退', () => {
    const item = straightLanelet('7', 0, 10); // 完整矩形（y ±2）+ 中心线 y=0
    const snapshot = makeSnapshot({ lanelets: [item] });
    // (5, 2.6) 在矩形外，距中心线 2.6 m：默认 2 m 容差不命中
    expect(hitTestSnapshot(snapshot, 5, 2.6).lanelet_ids).toEqual([]);
    expect(hitTestSnapshot(snapshot, 5, 2.6, { centerline_tolerance_m: 3 }).lanelet_ids).toEqual(['7']);
  });

  it('hitTestSnapshot：区域命中与内环镂空', () => {
    const outer = [point('a', 12, 0), point('b', 16, 0), point('c', 16, 4), point('d', 12, 4)];
    const hole = [point('e', 13, 1), point('f', 15, 1), point('g', 15, 3), point('h', 13, 3)];
    const snapshot = makeSnapshot({
      lanelets: [straightLanelet('100', 0, 10)],
      areas: [area('20', outer, [hole])],
    });
    expect(hitTestSnapshot(snapshot, 14, 0.5).area_ids).toEqual(['20']);
    expect(hitTestSnapshot(snapshot, 14, 2).area_ids).toEqual([]); // 内环内
    expect(hitTestSnapshot(snapshot, 14, 5).area_ids).toEqual([]);
  });

  it('hitTestSnapshot：重叠对象按数字 ID 升序返回', () => {
    const snapshot = makeSnapshot({
      lanelets: [straightLanelet('300', 0, 10), straightLanelet('100', 0, 10)],
    });
    expect(hitTestSnapshot(snapshot, 5, 0).lanelet_ids).toEqual(['100', '300']);
  });

  it('sortIds：数字优先、字典序回退', () => {
    expect(sortIds(['100', '2', '30'])).toEqual(['2', '30', '100']);
    expect(sortIds(['b', 'a'])).toEqual(['a', 'b']);
  });
});
