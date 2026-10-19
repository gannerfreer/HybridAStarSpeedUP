import { describe, expect, it } from 'vitest';
import { localFollowing, localPreceding } from '../src/core/topology';
import { lanelet, makeSnapshot, point, straightLanelet } from './fixtures';

describe('拓扑查询的本地近似', () => {
  const snapshot = makeSnapshot({
    lanelets: [
      straightLanelet('1', 0, 10), // 中心线 (0,0)→(10,0)
      straightLanelet('2', 10, 20), // 起点与 1 终点重合
      straightLanelet('3', 12, 22), // 起点距 1 终点 2 m（超出容差）
    ],
  });

  it('following：中心线起点与所选终点重合（容差 1 m）', () => {
    expect(localFollowing(snapshot, '1')).toEqual(['2']);
    expect(localFollowing(snapshot, '2')).toEqual([]);
    expect(localFollowing(snapshot, '3')).toEqual([]);
  });

  it('preceding：中心线终点与所选起点重合', () => {
    expect(localPreceding(snapshot, '2')).toEqual(['1']);
    expect(localPreceding(snapshot, '3')).toEqual([]);
    expect(localPreceding(snapshot, '1')).toEqual([]);
  });

  it('无中心线时返回空', () => {
    const noCenterline = makeSnapshot({
      lanelets: [lanelet('9', [point('l0', 0, 0), point('l1', 1, 0)], [point('r0', 0, 1)])],
    });
    expect(localFollowing(noCenterline, '9')).toEqual([]);
    expect(localPreceding(noCenterline, '9')).toEqual([]);
  });
});
