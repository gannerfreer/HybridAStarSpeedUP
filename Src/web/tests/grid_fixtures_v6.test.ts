import { createHash } from 'node:crypto';
import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import { describe, expect, it } from 'vitest';
import { cellCenter, countStates } from '../src/core/grid/coords';
import { parseGridDocument } from '../src/core/grid/parse';
import { GRID_STATE_LABELS } from '../src/core/grid/types';
import { findRepoRoot, moduleDir } from '../src/core/fixture';
import { listGridFixtures } from '../vite-plugin-map-fixtures';

/**
 * 冻结 v6 夹具的 Web 侧消费检查（与 tools/grid_fixtures.py 的 Python 复算独立）：
 * 全部可加载夹具可解析、抽查期望值、显示档 sha256 与 spot_checks 跨语言复算一致。
 */

const repoRoot = findRepoRoot(moduleDir(import.meta.url));
const gridDir = join(repoRoot, 'tests', 'fixtures', 'v6', 'grids');
const fixtures = listGridFixtures(repoRoot);

function readGrid(name: string): unknown {
  return JSON.parse(readFileSync(join(gridDir, name), 'utf8')) as unknown;
}

function sha256File(name: string): string {
  return createHash('sha256').update(readFileSync(join(gridDir, name))).digest('hex');
}

function splitmix64(x: number): bigint {
  const mask = (1n << 64n) - 1n;
  let value = (BigInt(x) + 0x9e3779b97f4a7c15n) & mask;
  let z = value;
  z = ((z ^ (z >> 30n)) * 0xbf58476d1ce4e5b9n) & mask;
  z = ((z ^ (z >> 27n)) * 0x94d049bb133111ebn) & mask;
  return (z ^ (z >> 31n)) & mask;
}

function thresholdState(h: bigint): number {
  const value = Number(h % 100n);
  return value < 82 ? 0 : value < 94 ? 1 : 2;
}

describe('v6 冻结栅格夹具（Web 消费）', () => {
  it('目录清单含显示档/解码样例/手算夹具', () => {
    expect(fixtures.map((entry) => entry.name)).toEqual([
      'display_256.json',
      'display_512.json',
      'display_1024.json',
      'snapshot_small.json',
      'rotated_origin_ccw.json',
      'rotated_origin_cw.json',
      'edge_cells.json',
      'unknown_edges.json',
      'obstacle_overlay.json',
      'coord_conversion.json',
      'pattern_64.json',
      'minimal.json',
    ]);
  });

  it('全部夹具通过协议解析', () => {
    for (const entry of fixtures) {
      const layer = parseGridDocument(readGrid(entry.name));
      expect(layer.cells.length).toBe(layer.width * layer.height);
      for (const state of layer.cells) {
        if (state !== 0 && state !== 1 && state !== 2) {
          throw new Error(`${entry.name} 含非法状态 ${state}`);
        }
      }
    }
  });

  it('冻结清单 sha256 与显示档 spec 一致', () => {
    const manifest = JSON.parse(
      readFileSync(join(repoRoot, 'tests', 'fixtures', 'v6', 'fixture_manifest.sha256.json'), 'utf8'),
    ) as Array<{ path: string; sha256: string }>;
    const listed = new Map(manifest.map((item) => [item.path, item.sha256]));
    for (const entry of fixtures) {
      const rel = `grids/${entry.name}`;
      expect(listed.get(rel)).toBe(sha256File(entry.name));
    }
    const spec = JSON.parse(readFileSync(join(gridDir, 'display_spec.json'), 'utf8')) as {
      tiers: Array<{ name: string; file: string; sha256: string; seed: number; width: number }>;
      spot_checks: Array<{ tier: string; row: number; col: number; h: string; v: number; state: number }>;
    };
    for (const tier of spec.tiers) {
      const name = `${tier.name}.json`;
      expect(tier.file).toBe(`grids/${name}`);
      expect(tier.sha256).toBe(sha256File(name));
    }
    // spot_checks 跨语言独立复算（BigInt splitmix64）
    for (const spot of spec.spot_checks) {
      const tier = spec.tiers.find((item) => item.name === spot.tier);
      expect(tier).toBeDefined();
      if (!tier) continue;
      const h = splitmix64(tier.seed + spot.row * tier.width + spot.col);
      expect(h.toString()).toBe(spot.h);
      expect(Number(h % 100n)).toBe(spot.v);
      expect(thresholdState(h)).toBe(spot.state);
    }
  });

  it('显示档全量三态与 splitmix64_thresholds_v1 复算一致', () => {
    for (const tier of [
      { name: 'display_256.json', seed: 20261012, width: 256, height: 256 },
      { name: 'display_512.json', seed: 20261013, width: 512, height: 512 },
    ]) {
      const layer = parseGridDocument(readGrid(tier.name));
      expect([layer.width, layer.height]).toEqual([tier.width, tier.height]);
      for (let row = 0; row < layer.height; row += 1) {
        for (let col = 0; col < layer.width; col += 1) {
          const expected = thresholdState(splitmix64(tier.seed + row * tier.width + col));
          if (layer.cells[row * tier.width + col] !== expected) {
            throw new Error(`${tier.name} (${row},${col}) 状态不一致`);
          }
        }
      }
    }
  });

  it('G1/G2 旋转原点中心（Web 换算）', () => {
    const g1 = parseGridDocument(readGrid('rotated_origin_ccw.json'));
    const [x1, y1] = cellCenter(g1, 1, 2);
    expect(x1).toBeCloseTo(9.25, 9);
    expect(y1).toBeCloseTo(-3.75, 9);
    expect(g1.cells[1 * g1.width + 2]).toBe(2);
    const g2 = parseGridDocument(readGrid('rotated_origin_cw.json'));
    const [x2, y2] = cellCenter(g2, 2, 1);
    expect(x2).toBeCloseTo(-1.75, 9);
    expect(y2).toBeCloseTo(1.25, 9);
    expect(g2.cells[2 * g2.width + 1]).toBe(2);
  });

  it('G4/G5/G8 三态计数', () => {
    const g4 = parseGridDocument(readGrid('unknown_edges.json'));
    expect(countStates(g4)).toEqual({ free: 15, occupied: 2, unknown: 8 });
    const g5 = parseGridDocument(readGrid('obstacle_overlay.json'));
    expect(countStates(g5)).toEqual({ free: 16, occupied: 8, unknown: 0 });
    const g8 = parseGridDocument(readGrid('pattern_64.json'));
    expect(countStates(g8)).toEqual({ free: 3026, occupied: 1008, unknown: 62 });
  });

  it('snapshot_small 解码向量', () => {
    const layer = parseGridDocument(readGrid('snapshot_small.json'));
    expect(Array.from(layer.cells)).toEqual([0, 1, 0, 0, 0, 2]);
    expect(layer.source).toBe('fixture');
    expect(layer.window).toEqual({ min_x_m: 0, min_y_m: 0, max_x_m: 0.3, max_y_m: 0.2 });
  });

  it('图例三态标签固定', () => {
    expect(GRID_STATE_LABELS[0]).toContain('空闲');
    expect(GRID_STATE_LABELS[1]).toContain('占据');
    expect(GRID_STATE_LABELS[2]).toContain('未知');
  });
});
