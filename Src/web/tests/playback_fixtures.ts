import { createHash } from 'node:crypto';
import { readFileSync } from 'node:fs';
import { join } from 'node:path';
import { findRepoRoot, moduleDir } from '../src/core/fixture';
import { loadPlaybackBundle } from '../src/core/playback/load';
import type { PlaybackBundle } from '../src/core/playback/types';

/** W4 测试共享：从磁盘读取 playback 夹具（Node 侧 SHA256）。 */

export const repoRoot = findRepoRoot(moduleDir(import.meta.url));
export const playbackDir = join(repoRoot, 'tests', 'fixtures', 'v4', 'playback');

export function readPlaybackFile(name: string): { text: string; value: unknown; bytes: Uint8Array } {
  const bytes = readFileSync(join(playbackDir, name));
  return { text: bytes.toString('utf8'), value: JSON.parse(bytes.toString('utf8')) as unknown, bytes };
}

export function nodeHashHex(bytes: Uint8Array): string {
  return createHash('sha256').update(bytes).digest('hex');
}

export async function loadBundleFromDisk(): Promise<PlaybackBundle> {
  return loadPlaybackBundle({
    index_bytes: readPlaybackFile('index.json').bytes,
    load_file_bytes: async (file) => new Uint8Array(readFileSync(join(playbackDir, file))),
    hash_hex: async (bytes) => nodeHashHex(bytes),
  });
}
