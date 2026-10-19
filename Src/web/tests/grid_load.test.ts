import { describe, expect, it } from 'vitest';
import {
  GridLoadTracker,
  loadGridFixture,
  parseGridCatalog,
  type GridFetchLike,
} from '../src/core/grid/load';

const snapshotText = JSON.stringify({
  schema_version: 1,
  type: 'grid_snapshot',
  source: 'fixture',
  map_id: 'grid_snapshot_small',
  map_version: 'v6',
  window: { min_x_m: 0, min_y_m: 0, max_x_m: 0.3, max_y_m: 0.2 },
  requested_resolution_m: 0.1,
  effective_resolution_m: 0.1,
  resolution_m: 0.1,
  degraded: false,
  width: 3,
  height: 2,
  origin: { x_m: 0, y_m: 0, yaw_rad: 0 },
  encoding: 'base64_u8',
  cells: 'AAEAAAAC',
});

function fetcherFor(text: string, status = 200): GridFetchLike {
  return async () => ({
    ok: status >= 200 && status < 300,
    status,
    text: async () => text,
  });
}

describe('栅格夹具加载与陈旧跟踪', () => {
  it('目录解析与非法目录拒绝', () => {
    const catalog = parseGridCatalog({
      fixture_version: 'v6',
      files: ['display_256.json', 'snapshot_small.json'],
    });
    expect(catalog.files).toEqual(['display_256.json', 'snapshot_small.json']);
    expect(() => parseGridCatalog({ files: [] })).toThrow(/fixture_version/);
    expect(() => parseGridCatalog({ fixture_version: 'v6', files: ['a/b.json'] })).toThrow(/非法/);
  });

  it('GridLoadTracker：apply/stale/anomaly 与计数', () => {
    const tracker = new GridLoadTracker();
    const seq1 = tracker.begin();
    const seq2 = tracker.begin();
    expect(tracker.latest_seq).toBe(2);
    expect(tracker.accept(seq2)).toBe('apply');
    expect(tracker.accept(seq1)).toBe('stale');
    expect(tracker.stale_ignored).toBe(1);
    expect(tracker.accept(seq2 + 1)).toBe('anomaly');
    expect(tracker.stale_ignored).toBe(1);
  });

  it('乱序响应：B(2) 先到应用、A(1) 后到丢弃', () => {
    const tracker = new GridLoadTracker();
    const seqA = tracker.begin();
    const seqB = tracker.begin();
    const arrivals = [
      { seq: seqB, window: 'B' },
      { seq: seqA, window: 'A' },
    ];
    const applied: string[] = [];
    for (const arrival of arrivals) {
      if (tracker.accept(arrival.seq) === 'apply') applied.push(arrival.window);
    }
    expect(applied).toEqual(['B']);
    expect(tracker.stale_ignored).toBe(1);
  });

  it('loadGridFixture：解析成功并记录字节数/哈希', async () => {
    const loaded = await loadGridFixture(fetcherFor(snapshotText), 'snapshot_small.json', async () => 'a'.repeat(64));
    expect(loaded.layer.kind).toBe('grid_snapshot');
    expect(Array.from(loaded.layer.cells)).toEqual([0, 1, 0, 0, 0, 2]);
    expect(loaded.info.file).toBe('snapshot_small.json');
    expect(loaded.info.bytes).toBe(new TextEncoder().encode(snapshotText).byteLength);
    expect(loaded.info.sha256).toBe('a'.repeat(64));
  });

  it('loadGridFixture：HTTP 失败/非 JSON/解析失败可见错误', async () => {
    await expect(loadGridFixture(fetcherFor('', 404), 'missing.json')).rejects.toThrow(/HTTP 404/);
    await expect(loadGridFixture(fetcherFor('<html>'), 'bad.json')).rejects.toThrow(/不是有效 JSON/);
    await expect(loadGridFixture(fetcherFor('{"type":"grid_view"}'), 'bad2.json')).rejects.toThrow(
      /schema_version/,
    );
  });
});
