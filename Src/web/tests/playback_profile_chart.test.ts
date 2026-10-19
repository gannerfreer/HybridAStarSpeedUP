import { describe, expect, it } from 'vitest';
import { buildProfilePolyline, profileValueRange } from '../src/components/ProfileChart';

const points = [
  { t_s: 0, value: 0 },
  { t_s: 1, value: 2 },
  { t_s: 2, value: 4 },
  { t_s: 3, value: 0 },
];

describe('v(t)/κ(t) 小图路径构建', () => {
  it('取值范围含 8% 边距', () => {
    const [min, max] = profileValueRange(points);
    expect(min).toBeLessThan(0);
    expect(max).toBeGreaterThan(4);
    expect(profileValueRange([])).toEqual([0, 1]);
    expect(profileValueRange([{ t_s: 0, value: 3 }])[0]).toBeLessThan(3);
  });

  it('折线路径：首点 M、末点在绘图区内', () => {
    const path = buildProfilePolyline(points, 280, 64, 6);
    expect(path.startsWith('M ')).toBe(true);
    expect(path.match(/L /g)).toHaveLength(3);
    const numbers = [...path.matchAll(/[ML] ([\d.]+) ([\d.]+)/g)].map((match) => [
      Number(match[1]),
      Number(match[2]),
    ]);
    expect(numbers[0][0]).toBeCloseTo(6, 6); // 左 padding
    expect(numbers[3][0]).toBeCloseTo(274, 6); // 右 padding
    for (const [, y] of numbers) {
      expect(y).toBeGreaterThanOrEqual(6 - 1e-9);
      expect(y).toBeLessThanOrEqual(64 - 6 + 1e-9);
    }
  });

  it('空序列返回空串', () => {
    expect(buildProfilePolyline([])).toBe('');
  });
});
