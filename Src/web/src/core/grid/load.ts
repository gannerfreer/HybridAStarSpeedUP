import { parseGridDocument } from './parse';
import type { GridFixtureInfo, GridLayerData } from './types';

/**
 * 离线夹具加载（无后端；`/fixtures/grids/` 由 Vite 插件从
 * `tests/fixtures/v6/grids/` 提供）。陈旧规则沿用协议 §6 语义：
 * `latest_seq` 单调递增，响应 seq < latest 丢弃并计数，> latest 视为协议异常。
 */

export const GRID_FIXTURE_URL_PREFIX = '/fixtures/grids/';
export const GRID_CATALOG_URL = '/fixtures/grids/catalog.json';

export interface GridCatalog {
  fixture_version: string;
  files: string[];
}

export function parseGridCatalog(value: unknown): GridCatalog {
  if (typeof value !== 'object' || value === null) throw new Error('栅格夹具目录必须是对象');
  const record = value as Record<string, unknown>;
  const version = record.fixture_version;
  if (typeof version !== 'string' || version.length === 0) {
    throw new Error('栅格夹具目录缺少 fixture_version');
  }
  const files = record.files;
  if (!Array.isArray(files) || files.length === 0) throw new Error('栅格夹具目录 files 为空');
  const names: string[] = [];
  for (const file of files) {
    if (typeof file !== 'string' || file.length === 0 || file.includes('/')) {
      throw new Error(`栅格夹具目录含非法文件名: ${String(file)}`);
    }
    names.push(file);
  }
  return { fixture_version: version, files: names };
}

export type GridAcceptDecision = 'apply' | 'stale' | 'anomaly';

/** 协议 §6 规则 1–2 的单来源跟踪器（响应乱序防护）。 */
export class GridLoadTracker {
  private latestSeq = 0;

  private staleIgnored = 0;

  begin(): number {
    this.latestSeq += 1;
    return this.latestSeq;
  }

  get latest_seq(): number {
    return this.latestSeq;
  }

  get stale_ignored(): number {
    return this.staleIgnored;
  }

  accept(seq: number): GridAcceptDecision {
    if (seq === this.latestSeq) return 'apply';
    if (seq < this.latestSeq) {
      this.staleIgnored += 1;
      return 'stale';
    }
    return 'anomaly';
  }
}

export interface GridFetchResponse {
  ok: boolean;
  status: number;
  text(): Promise<string>;
}

export type GridFetchLike = (url: string) => Promise<GridFetchResponse>;

export interface LoadedGridFixture {
  layer: GridLayerData;
  info: GridFixtureInfo;
}

/** 拉取并解析单个夹具；哈希函数可选注入（浏览器侧用 crypto.subtle）。 */
export async function loadGridFixture(
  fetcher: GridFetchLike,
  file: string,
  hashText?: (text: string) => Promise<string>,
): Promise<LoadedGridFixture> {
  const response = await fetcher(`${GRID_FIXTURE_URL_PREFIX}${file}`);
  if (!response.ok) {
    throw new Error(`加载栅格夹具 ${file} 失败：HTTP ${response.status}`);
  }
  const text = await response.text();
  let value: unknown;
  try {
    value = JSON.parse(text) as unknown;
  } catch (error) {
    throw new Error(`栅格夹具 ${file} 不是有效 JSON：${(error as Error).message}`);
  }
  const layer = parseGridDocument(value);
  const bytes = new TextEncoder().encode(text).byteLength;
  const sha256 = hashText ? await hashText(text) : '';
  return { layer, info: { file, sha256, bytes } };
}
