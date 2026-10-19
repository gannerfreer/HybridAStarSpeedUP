import { createHash } from 'node:crypto';
import { existsSync, readFileSync, readdirSync } from 'node:fs';
import { basename, dirname, join, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import type { Plugin } from 'vite';

/**
 * 将仓库内的地图快照夹具提供给浏览器：
 *  - `vite dev`：中间件直接读取磁盘（不复制数据，权威源保持唯一）；
 *  - `vite build`：把快照与来源描述作为静态资源写入 `dist/maps/`。
 *
 * 数据源优先级：`tests/fixtures/v3/map/mapping_example.snapshot.json`（v3 晋升夹具）
 * 优先于 `Src/map/examples/data/mapping_example.snapshot.json`（M4 样例回退）。
 * 运行界面通过 `/maps/source.json` 显示实际消费的数据源与 SHA256。
 */

export const MAP_SNAPSHOT_URL = '/maps/mapping_example.snapshot.json';
export const MAP_SOURCE_URL = '/maps/source.json';
export const VEHICLE_MODEL_URL = '/fixtures/vehicles/default.json';
export const PLAYBACK_URL_PREFIX = '/fixtures/playback/';
export const GRID_FIXTURE_URL_PREFIX = '/fixtures/grids/';
export const GRID_CATALOG_URL = '/fixtures/grids/catalog.json';

export interface MapFixtureInfo {
  abs_path: string;
  rel_path: string;
  kind: 'fixture_v3' | 'example_fallback';
  sha256: string;
  bytes: number;
}

const V3_FIXTURE_REL_PATH = 'tests/fixtures/v3/map/mapping_example.snapshot.json';
const EXAMPLE_REL_PATH = 'Src/map/examples/data/mapping_example.snapshot.json';
const VEHICLE_REL_PATH = 'tests/fixtures/v3/vehicles/default.json';
const PLAYBACK_REL_DIR = 'tests/fixtures/v4/playback';
const GRID_FIXTURE_REL_DIR = 'tests/fixtures/v6/grids';

/**
 * v6 栅格夹具目录（X-W1）：显示档 + 手算夹具 + 解码样例。
 * `display_spec.json`（生成规范）与 `protocol_cases.json`（协议用例）不是栅格文档，
 * 不进入加载目录；`minimal.json` 为 v5 回退样例。
 */
const GRID_FIXTURE_ORDER = [
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
] as const;

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

export function resolveMapFixture(repoRoot: string): MapFixtureInfo {
  const candidates: Array<{ rel: string; kind: MapFixtureInfo['kind'] }> = [
    { rel: V3_FIXTURE_REL_PATH, kind: 'fixture_v3' },
    { rel: EXAMPLE_REL_PATH, kind: 'example_fallback' },
  ];
  for (const candidate of candidates) {
    const absPath = join(repoRoot, ...candidate.rel.split('/'));
    if (!existsSync(absPath)) continue;
    const bytes = readFileSync(absPath);
    return {
      abs_path: absPath,
      rel_path: candidate.rel,
      kind: candidate.kind,
      sha256: createHash('sha256').update(bytes).digest('hex'),
      bytes: bytes.byteLength,
    };
  }
  throw new Error(`map snapshot not found; tried: ${candidates.map((c) => c.rel).join(', ')}`);
}

/** 车辆模型夹具（v3；W3 场景编辑加载）。 */
export function resolveVehicleFixture(repoRoot: string): { abs_path: string; rel_path: string } {
  const absPath = join(repoRoot, ...VEHICLE_REL_PATH.split('/'));
  if (!existsSync(absPath)) {
    throw new Error(`vehicle fixture not found: ${VEHICLE_REL_PATH}`);
  }
  return { abs_path: absPath, rel_path: VEHICLE_REL_PATH };
}

/** W4 回放夹具文件清单（v4/playback 下的 *.json）。 */
export function listPlaybackFixtures(repoRoot: string): Array<{ name: string; abs_path: string }> {
  const dir = join(repoRoot, ...PLAYBACK_REL_DIR.split('/'));
  if (!existsSync(dir)) throw new Error(`playback fixture dir not found: ${PLAYBACK_REL_DIR}`);
  return readdirSync(dir)
    .filter((name) => name.endsWith('.json'))
    .sort()
    .map((name) => ({ name, abs_path: join(dir, name) }));
}

/** v6 栅格夹具文件清单（存在性过滤；顺序固定）。 */
export function listGridFixtures(repoRoot: string): Array<{ name: string; abs_path: string }> {
  const dir = join(repoRoot, ...GRID_FIXTURE_REL_DIR.split('/'));
  if (!existsSync(dir)) throw new Error(`grid fixture dir not found: ${GRID_FIXTURE_REL_DIR}`);
  const available = new Set(readdirSync(dir).filter((name) => name.endsWith('.json')));
  return GRID_FIXTURE_ORDER.filter((name) => available.has(name)).map((name) => ({
    name,
    abs_path: join(dir, name),
  }));
}

function gridCatalogDescriptor(entries: Array<{ name: string }>): string {
  return (
    JSON.stringify(
      {
        schema_version: 1,
        type: 'grid_fixture_catalog',
        fixture_version: 'v6',
        source_dir: GRID_FIXTURE_REL_DIR,
        files: entries.map((entry) => entry.name),
      },
      null,
      2,
    ) + '\n'
  );
}

function sourceDescriptor(fixture: MapFixtureInfo): string {
  return (
    JSON.stringify(
      {
        rel_path: fixture.rel_path,
        kind: fixture.kind,
        sha256: fixture.sha256,
        bytes: fixture.bytes,
      },
      null,
      2,
    ) + '\n'
  );
}

export function mapFixturesPlugin(): Plugin {
  const repoRoot = findRepoRoot(dirname(fileURLToPath(import.meta.url)));
  let cached: MapFixtureInfo | null = null;
  const fixture = (): MapFixtureInfo => (cached ??= resolveMapFixture(repoRoot));

  return {
    name: 'planner-map-fixtures',
    configureServer(server) {
      const info = fixture();
      const vehicle = resolveVehicleFixture(repoRoot);
      const playback = listPlaybackFixtures(repoRoot);
      const grids = listGridFixtures(repoRoot);
      server.config.logger.info(
        `[map-fixtures] serving ${info.rel_path} (${info.kind}, sha256 ${info.sha256.slice(0, 12)}…) at ${MAP_SNAPSHOT_URL}`,
      );
      server.config.logger.info(`[map-fixtures] serving ${vehicle.rel_path} at ${VEHICLE_MODEL_URL}`);
      server.config.logger.info(`[map-fixtures] serving ${playback.length} playback fixtures at ${PLAYBACK_URL_PREFIX}`);
      server.config.logger.info(`[map-fixtures] serving ${grids.length} grid fixtures at ${GRID_FIXTURE_URL_PREFIX}`);
      server.middlewares.use((req, res, next) => {
        const url = (req.url ?? '').split('?')[0];
        if (url === VEHICLE_MODEL_URL) {
          res.setHeader('Content-Type', 'application/json; charset=utf-8');
          res.setHeader('Cache-Control', 'no-store');
          res.end(readFileSync(vehicle.abs_path));
          return;
        }
        if (url === GRID_CATALOG_URL) {
          res.setHeader('Content-Type', 'application/json; charset=utf-8');
          res.setHeader('Cache-Control', 'no-store');
          res.end(gridCatalogDescriptor(grids));
          return;
        }
        if (url.startsWith(GRID_FIXTURE_URL_PREFIX)) {
          const name = basename(url.slice(GRID_FIXTURE_URL_PREFIX.length));
          const entry = grids.find((item) => item.name === name);
          if (!entry) {
            res.statusCode = 404;
            res.end('grid fixture not found');
            return;
          }
          res.setHeader('Content-Type', 'application/json; charset=utf-8');
          res.setHeader('Cache-Control', 'no-store');
          res.end(readFileSync(entry.abs_path));
          return;
        }
        if (url.startsWith(PLAYBACK_URL_PREFIX)) {
          const name = basename(url.slice(PLAYBACK_URL_PREFIX.length));
          const entry = playback.find((item) => item.name === name);
          if (!entry) {
            res.statusCode = 404;
            res.end('playback fixture not found');
            return;
          }
          res.setHeader('Content-Type', 'application/json; charset=utf-8');
          res.setHeader('Cache-Control', 'no-store');
          res.end(readFileSync(entry.abs_path));
          return;
        }
        if (url !== MAP_SNAPSHOT_URL && url !== MAP_SOURCE_URL) {
          next();
          return;
        }
        const current = fixture();
        res.setHeader('Content-Type', 'application/json; charset=utf-8');
        res.setHeader('Cache-Control', 'no-store');
        res.end(url === MAP_SNAPSHOT_URL ? readFileSync(current.abs_path) : sourceDescriptor(current));
      });
    },
    generateBundle() {
      const info = fixture();
      const vehicle = resolveVehicleFixture(repoRoot);
      this.info(
        `[map-fixtures] bundling ${info.rel_path} (${info.kind}) as maps/mapping_example.snapshot.json`,
      );
      this.emitFile({
        type: 'asset',
        fileName: 'maps/mapping_example.snapshot.json',
        source: readFileSync(info.abs_path),
      });
      this.emitFile({
        type: 'asset',
        fileName: 'maps/source.json',
        source: sourceDescriptor(info),
      });
      this.emitFile({
        type: 'asset',
        fileName: 'fixtures/vehicles/default.json',
        source: readFileSync(vehicle.abs_path),
      });
      for (const entry of listPlaybackFixtures(repoRoot)) {
        this.emitFile({
          type: 'asset',
          fileName: `fixtures/playback/${entry.name}`,
          source: readFileSync(entry.abs_path),
        });
      }
      const grids = listGridFixtures(repoRoot);
      for (const entry of grids) {
        this.emitFile({
          type: 'asset',
          fileName: `fixtures/grids/${entry.name}`,
          source: readFileSync(entry.abs_path),
        });
      }
      this.emitFile({
        type: 'asset',
        fileName: 'fixtures/grids/catalog.json',
        source: gridCatalogDescriptor(grids),
      });
    },
  };
}
