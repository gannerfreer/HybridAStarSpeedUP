import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import { describe, expect, it } from 'vitest';
import { verifySampleHashes } from '../src/core/playback/hash';
import { loadPlaybackBundle } from '../src/core/playback/load';
import { parsePlaybackIndex } from '../src/core/playback/parse';
import { loadBundleFromDisk, nodeHashHex, playbackDir, readPlaybackFile } from './playback_fixtures';

describe('SHA256 校验', () => {
  const index = parsePlaybackIndex(readPlaybackFile('index.json').value);

  it('全部索引样例哈希与文件字节一致', async () => {
    const result = await verifySampleHashes({
      samples: index.samples,
      loadBytes: async (file) => new Uint8Array(readFileSync(join(playbackDir, file))),
      hashHex: async (bytes) => nodeHashHex(bytes),
    });
    expect(result.failures).toEqual([]);
    expect(result.verified.size).toBe(index.samples.length);
  });

  it('哈希不匹配的样例被拒绝并记录原因', async () => {
    const corrupted = index.samples.map((sample, position) =>
      position === 1 ? { ...sample, sha256: '0'.repeat(64) } : sample,
    );
    const result = await verifySampleHashes({
      samples: corrupted,
      loadBytes: async (file) => new Uint8Array(readFileSync(join(playbackDir, file))),
      hashHex: async (bytes) => nodeHashHex(bytes),
    });
    expect(result.failures).toHaveLength(1);
    expect(result.failures[0]).toMatch(/SHA256 不匹配/);
    expect(result.verified.size).toBe(index.samples.length - 1);
  });

  it('文件读取失败记录 issue 且不抛出', async () => {
    const result = await verifySampleHashes({
      samples: index.samples.slice(0, 1),
      loadBytes: async () => {
        throw new Error('ENOENT');
      },
      hashHex: async () => '',
    });
    expect(result.failures[0]).toMatch(/读取失败/);
    expect(result.verified.size).toBe(0);
  });
});

describe('回放包加载', () => {
  it('从磁盘加载完整包：11 样例、0 issue、含小型地图', async () => {
    const bundle = await loadBundleFromDisk();
    expect(bundle.samples).toHaveLength(11);
    expect(bundle.issues).toEqual([]);
    expect(bundle.map).not.toBeNull();
    expect(bundle.map?.map_id).toBe('map_synthetic_min');
    expect(bundle.map_file).toBe('map_snapshot_minimal.json');
    const kinds = bundle.samples.map((sample) => sample.kind);
    expect(kinds).toContain('plan_result');
    expect(kinds).toContain('bspline_curve');
    expect(kinds).toContain('corridor');
    expect(kinds).toContain('collision_case');
    expect(kinds).toContain('stale_result');
  });

  it('索引地图版本变化 → 非过期样例被拒绝（过期样例保留演示）', async () => {
    const index = parsePlaybackIndex(readPlaybackFile('index.json').value);
    const mutated = { ...index, map_version: 'v9' };
    const indexBytes = new TextEncoder().encode(JSON.stringify(mutated));
    const bundle = await loadPlaybackBundle({
      index_bytes: indexBytes,
      load_file_bytes: async (file) => new Uint8Array(readFileSync(join(playbackDir, file))),
      hash_hex: async (bytes) => nodeHashHex(bytes),
    });
    expect(bundle.samples.map((sample) => sample.kind)).toEqual(['stale_result']);
    expect(bundle.issues.filter((issue) => issue.includes('地图版本')).length).toBe(10);
    expect(bundle.map).toBeNull();
  });
});
