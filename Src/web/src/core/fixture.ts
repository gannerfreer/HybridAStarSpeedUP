import { createHash } from 'node:crypto';
import { existsSync, readFileSync } from 'node:fs';
import { dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

/**
 * Node 侧夹具定位（测试与渲染检查使用，不进入浏览器构建）：
 * 优先消费 v3 晋升夹具，未就绪时回退 M4 样例。文件内容不复制，
 * 权威源保持唯一，实际消费路径与 SHA256 写入证据报告。
 */

export const V3_FIXTURE_REL_PATH = 'tests/fixtures/v3/map/mapping_example.snapshot.json';
export const EXAMPLE_REL_PATH = 'Src/map/examples/data/mapping_example.snapshot.json';
export const VEHICLE_FIXTURE_REL_PATH = 'tests/fixtures/v3/vehicles/default.json';

export interface SnapshotSource {
  rel_path: string;
  kind: 'fixture_v3' | 'example_fallback';
  sha256: string;
  bytes: number;
}

export function moduleDir(importMetaUrl: string): string {
  return dirname(fileURLToPath(importMetaUrl));
}

export function findRepoRoot(startDir: string): string {
  let dir = resolve(startDir);
  for (;;) {
    if (existsSync(join(dir, 'AGENTS.md')) && existsSync(join(dir, 'tools', 'components.json'))) {
      return dir;
    }
    const parent = dirname(dir);
    if (parent === dir) {
      throw new Error(`repository root not found above ${startDir}`);
    }
    dir = parent;
  }
}

export function resolveSnapshotSource(repoRoot: string): SnapshotSource {
  const candidates: Array<{ rel: string; kind: SnapshotSource['kind'] }> = [
    { rel: V3_FIXTURE_REL_PATH, kind: 'fixture_v3' },
    { rel: EXAMPLE_REL_PATH, kind: 'example_fallback' },
  ];
  for (const candidate of candidates) {
    const absPath = join(repoRoot, ...candidate.rel.split('/'));
    if (!existsSync(absPath)) continue;
    const bytes = readFileSync(absPath);
    return {
      rel_path: candidate.rel,
      kind: candidate.kind,
      sha256: createHash('sha256').update(bytes).digest('hex'),
      bytes: bytes.byteLength,
    };
  }
  throw new Error(`no map snapshot found; tried: ${candidates.map((c) => c.rel).join(', ')}`);
}

export function readSnapshotJson(
  repoRoot: string,
  source: SnapshotSource,
): { text: string; value: unknown } {
  const text = readFileSync(join(repoRoot, ...source.rel_path.split('/')), 'utf8');
  return { text, value: JSON.parse(text) as unknown };
}

/** 读取 v3 车辆模型夹具（W3 场景编辑）。 */
export function readVehicleJson(repoRoot: string): {
  rel_path: string;
  sha256: string;
  bytes: number;
  value: unknown;
} {
  const absPath = join(repoRoot, ...VEHICLE_FIXTURE_REL_PATH.split('/'));
  if (!existsSync(absPath)) {
    throw new Error(`vehicle fixture not found: ${VEHICLE_FIXTURE_REL_PATH}`);
  }
  const bytes = readFileSync(absPath);
  return {
    rel_path: VEHICLE_FIXTURE_REL_PATH,
    sha256: createHash('sha256').update(bytes).digest('hex'),
    bytes: bytes.byteLength,
    value: JSON.parse(bytes.toString('utf8')) as unknown,
  };
}
