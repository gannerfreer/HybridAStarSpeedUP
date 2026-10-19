import { parseScene, serializeScene, SceneParseError } from './parse';
import type { WebScene } from './types';

/**
 * 场景 localStorage 自动保存/恢复（注入式存储，便于 Node 侧测试）。
 * 恢复失败（损坏 JSON/字段非法）返回错误信息，不抛出。
 */

export interface StorageLike {
  getItem(key: string): string | null;
  setItem(key: string, value: string): void;
  removeItem(key: string): void;
}

export const SCENE_STORAGE_KEY = 'planner.web_scene.v1';

export function saveSceneToStorage(storage: StorageLike, scene: WebScene): void {
  storage.setItem(SCENE_STORAGE_KEY, serializeScene(scene));
}

export type SceneRestoreResult =
  | { kind: 'none' }
  | { kind: 'loaded'; scene: WebScene }
  | { kind: 'error'; error: string };

export function loadSceneFromStorage(storage: StorageLike): SceneRestoreResult {
  const text = storage.getItem(SCENE_STORAGE_KEY);
  if (text === null || text.length === 0) return { kind: 'none' };
  try {
    return { kind: 'loaded', scene: parseScene(JSON.parse(text) as unknown) };
  } catch (error) {
    const message = error instanceof SceneParseError ? error.message : error instanceof Error ? error.message : String(error);
    return { kind: 'error', error: message };
  }
}

export function clearSceneStorage(storage: StorageLike): void {
  storage.removeItem(SCENE_STORAGE_KEY);
}

/** 场景文件名（下载用；不含时间戳，便于重复导出对比）。 */
export function sceneFileName(scene: Pick<WebScene, 'map_id'>): string {
  const safe = scene.map_id.replace(/[^A-Za-z0-9_-]+/g, '_') || 'map';
  return `planner_scene_${safe}.json`;
}
