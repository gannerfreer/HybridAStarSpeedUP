import { describe, expect, it } from 'vitest';
import {
  cellCenter,
  cellCorners,
  cellInfo,
  cellState,
  countStates,
  gridWorldBounds,
  worldToCell,
} from '../src/core/grid/coords';
import type { GridLayerData } from '../src/core/grid/types';

function makeLayer(overrides: Partial<GridLayerData> = {}): GridLayerData {
  return {
    kind: 'grid_view',
    source: 'grid_view',
    map_id: 'grid_test',
    map_version: 'v6',
    width: 4,
    height: 4,
    resolution_m: 0.5,
    origin: { x_m: 0, y_m: 0, yaw_rad: 0 },
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

describe('栅格坐标换算（含旋转原点 G1/G2）', () => {
  const g1 = makeLayer({ origin: { x_m: 10, y_m: -5, yaw_rad: Math.PI / 2 } });
  const g2 = makeLayer({ origin: { x_m: -3, y_m: 2, yaw_rad: -Math.PI / 2 } });

  it('G1 旋转原点中心（π/2）', () => {
    const cases: Array<[number, number, [number, number]]> = [
      [0, 0, [9.75, -4.75]],
      [1, 2, [9.25, -3.75]],
      [3, 3, [8.25, -3.25]],
    ];
    for (const [row, col, expected] of cases) {
      const [x, y] = cellCenter(g1, row, col);
      expect(x).toBeCloseTo(expected[0], 9);
      expect(y).toBeCloseTo(expected[1], 9);
    }
  });

  it('G1 (0,0) 单元多边形四角', () => {
    const corners = cellCorners(g1, 0, 0);
    const expected = [
      [10, -5],
      [10, -4.5],
      [9.5, -4.5],
      [9.5, -5],
    ];
    corners.forEach((corner, index) => {
      expect(corner[0]).toBeCloseTo(expected[index][0], 9);
      expect(corner[1]).toBeCloseTo(expected[index][1], 9);
    });
  });

  it('G2 旋转原点中心（-π/2）', () => {
    const cases: Array<[number, number, [number, number]]> = [
      [0, 0, [-2.75, 1.75]],
      [0, 3, [-2.75, 0.25]],
      [2, 1, [-1.75, 1.25]],
      [3, 0, [-1.25, 1.75]],
      [3, 3, [-1.25, 0.25]],
    ];
    for (const [row, col, expected] of cases) {
      const [x, y] = cellCenter(g2, row, col);
      expect(x).toBeCloseTo(expected[0], 9);
      expect(y).toBeCloseTo(expected[1], 9);
    }
  });

  it('旋转原点正反换算互为逆（全部单元）', () => {
    for (const layer of [g1, g2]) {
      for (let row = 0; row < layer.height; row += 1) {
        for (let col = 0; col < layer.width; col += 1) {
          const [x, y] = cellCenter(layer, row, col);
          expect(worldToCell(layer, x, y)).toEqual({ row, col, in_bounds: true });
        }
      }
    }
  });

  it('G6 正向/反向换算与出界', () => {
    const g6 = makeLayer({
      width: 6,
      height: 5,
      resolution_m: 0.25,
      origin: { x_m: 1.5, y_m: -2, yaw_rad: 0 },
      cells: new Uint8Array(30),
    });
    const forward: Array<[number, number, [number, number]]> = [
      [0, 0, [1.625, -1.875]],
      [0, 5, [2.875, -1.875]],
      [4, 0, [1.625, -0.875]],
      [4, 5, [2.875, -0.875]],
      [2, 2, [2.125, -1.375]],
    ];
    for (const [row, col, expected] of forward) {
      const [x, y] = cellCenter(g6, row, col);
      expect(x).toBeCloseTo(expected[0], 9);
      expect(y).toBeCloseTo(expected[1], 9);
    }
    const inverse: Array<[[number, number], [number, number]]> = [
      [[1.5, -2], [0, 0]],
      [[1.7499, -1.7501], [0, 0]],
      [[1.75, -1.75], [1, 1]],
      [[2.9999, -0.7501], [4, 5]],
    ];
    for (const [point, expected] of inverse) {
      const hit = worldToCell(g6, point[0], point[1]);
      expect([hit.row, hit.col]).toEqual(expected);
      expect(hit.in_bounds).toBe(true);
    }
    const outside = worldToCell(g6, 3.0, -0.75);
    expect([outside.row, outside.col]).toEqual([5, 6]);
    expect(outside.in_bounds).toBe(false);
  });

  it('cellInfo/状态/计数/包围盒', () => {
    const g4 = makeLayer({
      width: 5,
      height: 5,
      origin: { x_m: -2, y_m: -2, yaw_rad: 0 },
      cells: new Uint8Array([
        0, 0, 0, 0, 2,
        0, 2, 2, 2, 0,
        0, 2, 2, 2, 0,
        0, 0, 0, 1, 1,
        2, 0, 0, 0, 0,
      ]),
    });
    expect(cellState(g4, 0, 4)).toBe(2);
    expect(cellState(g4, 3, 3)).toBe(1);
    expect(cellState(g4, -1, 0)).toBeNull();
    const info = cellInfo(g4, 1, 1);
    expect(info.state).toBe(2);
    expect(info.center_x_m).toBeCloseTo(-1.25, 9);
    expect(info.center_y_m).toBeCloseTo(-1.25, 9);
    expect(countStates(g4)).toEqual({ free: 15, occupied: 2, unknown: 8 });
    const bounds = gridWorldBounds(g1);
    expect(bounds.min_x_m).toBeCloseTo(8, 9);
    expect(bounds.max_x_m).toBeCloseTo(10, 9);
    expect(bounds.min_y_m).toBeCloseTo(-5, 9);
    expect(bounds.max_y_m).toBeCloseTo(-3, 9);
  });
});
