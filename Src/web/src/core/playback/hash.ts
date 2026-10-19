import type { PlaybackIndexSample } from './types';

/**
 * 回放样例 SHA256 校验（哈希函数与读取注入，Node/浏览器共用）。
 * 索引哈希不匹配的样例被拒绝加载（不进入渲染），并记录 issue。
 */

export interface HashVerifyInput {
  samples: readonly PlaybackIndexSample[];
  loadBytes: (file: string) => Promise<Uint8Array>;
  hashHex: (bytes: Uint8Array) => Promise<string>;
}

export interface HashVerifyResult {
  /** file → 已校验字节（仅通过校验的样例）。 */
  verified: Map<string, Uint8Array>;
  failures: string[];
}

export async function verifySampleHashes(input: HashVerifyInput): Promise<HashVerifyResult> {
  const verified = new Map<string, Uint8Array>();
  const failures: string[] = [];
  for (const sample of input.samples) {
    try {
      const bytes = await input.loadBytes(sample.file);
      const actual = (await input.hashHex(bytes)).toLowerCase();
      if (actual !== sample.sha256.toLowerCase()) {
        failures.push(
          `样例 ${sample.id}（${sample.file}）SHA256 不匹配：期望 ${sample.sha256.slice(0, 12)}…，实际 ${actual.slice(0, 12)}…，已拒绝加载`,
        );
        continue;
      }
      verified.set(sample.file, bytes);
    } catch (error) {
      failures.push(
        `样例 ${sample.id}（${sample.file}）读取失败：${error instanceof Error ? error.message : String(error)}`,
      );
    }
  }
  return { verified, failures };
}

/** 浏览器侧 SHA256（crypto.subtle；非安全上下文返回错误）。 */
export async function sha256HexBrowser(bytes: Uint8Array): Promise<string> {
  if (!globalThis.crypto?.subtle) {
    throw new Error('crypto.subtle 不可用（需要安全上下文）');
  }
  const digest = await globalThis.crypto.subtle.digest('SHA-256', bytes as unknown as BufferSource);
  return Array.from(new Uint8Array(digest), (byte) => byte.toString(16).padStart(2, '0')).join('');
}
