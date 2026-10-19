import { mkdirSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';
import { describe, expect, it } from 'vitest';
import { buildRenderCheckArtifacts } from '../src/core/evidence';
import { findRepoRoot, moduleDir, readSnapshotJson, resolveSnapshotSource } from '../src/core/fixture';
import { parseSnapshot } from '../src/core/parse';

const repoRoot = findRepoRoot(moduleDir(import.meta.url));
const source = resolveSnapshotSource(repoRoot);

/**
 * W1 渲染检查证据：对官方示例/测试地图快照输出确定性报告
 * （对象计数、包围盒、取景比例、比例尺、绘制命令统计）并导出 SVG。
 * 产物：`Src/web/evidence/render_check.{json,svg}`；
 * 经统一入口运行时同时写入 `$PLANNER_RUN_DIR/outputs/web_render_check/`。
 */
describe('渲染检查证据（确定性）', () => {
  it('输出计数 / 包围盒 / 比例尺并写入证据文件', () => {
    const { value } = readSnapshotJson(repoRoot, source);
    const snapshot = parseSnapshot(value);
    const options = { width_px: 1280, height_px: 720, padding_px: 24 } as const;
    const { report, svg, commands } = buildRenderCheckArtifacts(snapshot, source, options);

    expect(report.checks.bounds_within_canvas).toBe(true);
    expect(report.checks.roundtrip_ok).toBe(true);
    expect(report.checks.roundtrip_px_ok).toBe(true);
    expect(report.checks.scale_bar_width_within_limit).toBe(true);
    expect(report.checks.counts_match_top_level).toBe(true);
    expect(report.checks.commands_all_finite).toBe(true);
    expect(report.checks.all_layers_nonempty).toBe(true);
    expect(commands.length).toBe(report.commands.total);
    expect(report.counts.lanelets).toBeGreaterThan(0);
    expect(report.counts.areas).toBeGreaterThan(0);
    expect(report.viewport.scale_px_per_m).toBeGreaterThan(0);

    // 确定性：重复构建字节一致（无时间戳、无随机量）。
    const second = buildRenderCheckArtifacts(snapshot, source, options);
    expect(JSON.stringify(second.report)).toBe(JSON.stringify(report));
    expect(second.svg).toBe(svg);

    const json = JSON.stringify(report, null, 2) + '\n';
    const outputDirs = [join(repoRoot, 'Src', 'web', 'evidence')];
    const runDir = process.env.PLANNER_RUN_DIR;
    if (runDir) outputDirs.push(join(runDir, 'outputs', 'web_render_check'));
    for (const dir of outputDirs) {
      mkdirSync(dir, { recursive: true });
      writeFileSync(join(dir, 'render_check.json'), json, 'utf8');
      writeFileSync(join(dir, 'render_check.svg'), svg, 'utf8');
    }

    const label = source.kind === 'fixture_v3' ? 'v3 fixture' : 'M4 example fallback';
    console.log(
      `[render-check] source=${label} (${source.rel_path}) sha256=${source.sha256.slice(0, 12)}… bytes=${source.bytes}`,
    );
    console.log(
      `[render-check] lanelets=${report.counts.lanelets} (centerline=${report.counts.lanelets_with_centerline}) ` +
        `areas=${report.counts.areas} boundary_points=${report.counts.boundary_points} area_points=${report.counts.area_points}`,
    );
    console.log(
      `[render-check] scale=${report.viewport.scale_px_per_m.toFixed(6)} px/m ` +
        `(${report.viewport.meters_per_pixel.toFixed(4)} m/px) ` +
        `scale_bar=${report.scale_bar.label}/${report.scale_bar.width_px.toFixed(1)}px commands=${report.commands.total}`,
    );
    console.log(`[render-check] evidence dirs: ${outputDirs.join(' ; ')}`);
  });
});
