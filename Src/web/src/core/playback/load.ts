import { verifySampleHashes } from './hash';
import {
  parseBsplineCurve,
  parseCollisionCase,
  parseCorridor,
  parsePlaybackIndex,
  parsePlaybackMap,
  parsePlanResult,
  parsePolynomialCurve,
  parseStaleResult,
} from './parse';
import type { PlaybackBundle, PlaybackSampleData } from './types';

/** 回放样例包加载：索引解析 → SHA256 校验 → 逐类解析 → 地图适配。 */

export interface PlaybackLoadInput {
  index_bytes: Uint8Array;
  load_file_bytes: (file: string) => Promise<Uint8Array>;
  hash_hex: (bytes: Uint8Array) => Promise<string>;
}

export async function loadPlaybackBundle(input: PlaybackLoadInput): Promise<PlaybackBundle> {
  const decoder = new TextDecoder();
  const index = parsePlaybackIndex(JSON.parse(decoder.decode(input.index_bytes)) as unknown);
  const { verified, failures } = await verifySampleHashes({
    samples: index.samples,
    loadBytes: input.load_file_bytes,
    hashHex: input.hash_hex,
  });
  const issues: string[] = [...failures];
  const samples: PlaybackSampleData[] = [];
  let map: PlaybackBundle['map'] = null;
  let mapFile: string | null = null;

  for (const meta of index.samples) {
    const bytes = verified.get(meta.file);
    if (!bytes) continue; // 校验/读取失败已记录
    try {
      const value = JSON.parse(decoder.decode(bytes)) as unknown;
      let parsed: PlaybackSampleData | null = null;
      switch (meta.type) {
        case 'map_snapshot':
          parsed = { kind: 'map_snapshot', sample: meta, map: parsePlaybackMap(value) };
          break;
        case 'plan_result':
          parsed = { kind: 'plan_result', sample: meta, result: parsePlanResult(value) };
          break;
        case 'bspline_curve':
          parsed = { kind: 'bspline_curve', sample: meta, curve: parseBsplineCurve(value) };
          break;
        case 'polynomial_curve':
          parsed = { kind: 'polynomial_curve', sample: meta, curve: parsePolynomialCurve(value) };
          break;
        case 'corridor':
          parsed = { kind: 'corridor', sample: meta, corridor: parseCorridor(value) };
          break;
        case 'collision_case':
          parsed = { kind: 'collision_case', sample: meta, collision: parseCollisionCase(value) };
          break;
        case 'stale_result':
          parsed = { kind: 'stale_result', sample: meta, stale: parseStaleResult(value) };
          break;
        default:
          issues.push(`样例 ${meta.id} 类型 ${meta.type} 不支持，已跳过`);
          continue;
      }
      // 地图版本一致性：过期样例（stale_request）是显式演示项，不在此拒绝。
      if (
        meta.type !== 'stale_result' &&
        (meta.map_id !== index.map_id || meta.map_version !== index.map_version)
      ) {
        issues.push(
          `样例 ${meta.id} 地图版本 ${meta.map_id}/${meta.map_version} 与索引 ${index.map_id}/${index.map_version} 不一致，已拒绝加载`,
        );
        continue;
      }
      samples.push(parsed);
      if (parsed.kind === 'map_snapshot' && map === null) {
        map = parsed.map;
        mapFile = meta.file;
      }
    } catch (error) {
      issues.push(`样例 ${meta.id} 解析失败：${error instanceof Error ? error.message : String(error)}`);
    }
  }

  return { index, samples, issues, map, map_file: mapFile };
}
