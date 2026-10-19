import { describe, expect, it } from 'vitest';
import { findRepoRoot, moduleDir, readSnapshotJson, resolveSnapshotSource } from '../src/core/fixture';
import { DEFAULT_LAYERS, toggleLayer, type LayerState } from '../src/core/layers';
import { parseSnapshot } from '../src/core/parse';
import {
  buildDrawCommands,
  commandsHaveFiniteCoordinates,
  summarizeDrawCommands,
} from '../src/core/render';
import { fitBounds } from '../src/core/viewport';

const repoRoot = findRepoRoot(moduleDir(import.meta.url));
const source = resolveSnapshotSource(repoRoot);
const snapshot = parseSnapshot(readSnapshotJson(repoRoot, source).value);
const viewport = fitBounds(snapshot.bounds_m, 1280, 720, 24);

function build(layers: LayerState) {
  return buildDrawCommands(snapshot, viewport, layers);
}

describe('图层开关与命令构建', () => {
  const all = build({ ...DEFAULT_LAYERS });
  const summary = summarizeDrawCommands(all);

  it('全开时各图层命令数与几何对应', () => {
    const expectedBoundaries = snapshot.lanelets.reduce(
      (count, lanelet) =>
        count + (lanelet.left.length >= 2 ? 1 : 0) + (lanelet.right.length >= 2 ? 1 : 0),
      0,
    );
    const expectedCenterlines = snapshot.lanelets.filter(
      (lanelet) => (lanelet.centerline?.length ?? 0) >= 2,
    ).length;
    const expectedAreas = snapshot.areas.filter((area) => area.outer.length > 0).length;
    expect(summary.by_layer.boundaries).toBe(expectedBoundaries);
    expect(summary.by_layer.centerline).toBe(expectedCenterlines);
    expect(summary.by_layer.areas).toBe(expectedAreas);
    expect(summary.total).toBe(expectedBoundaries + expectedCenterlines + expectedAreas);
  });

  it('关闭道路边界只移除该层', () => {
    const filtered = build({ ...DEFAULT_LAYERS, boundaries: false });
    expect(filtered.some((command) => command.layer === 'boundaries')).toBe(false);
    const filteredSummary = summarizeDrawCommands(filtered);
    expect(filteredSummary.by_layer.areas).toBe(summary.by_layer.areas);
    expect(filteredSummary.by_layer.centerline).toBe(summary.by_layer.centerline);
    expect(filteredSummary.by_layer.boundaries).toBe(0);
  });

  it('关闭中心线只移除该层', () => {
    const filtered = build({ ...DEFAULT_LAYERS, centerline: false });
    expect(filtered.some((command) => command.layer === 'centerline')).toBe(false);
    const filteredSummary = summarizeDrawCommands(filtered);
    expect(filteredSummary.by_layer.boundaries).toBe(summary.by_layer.boundaries);
    expect(filteredSummary.by_layer.areas).toBe(summary.by_layer.areas);
    expect(filteredSummary.by_layer.centerline).toBe(0);
  });

  it('关闭区域只移除该层', () => {
    const filtered = build({ ...DEFAULT_LAYERS, areas: false });
    expect(filtered.some((command) => command.layer === 'areas')).toBe(false);
    const filteredSummary = summarizeDrawCommands(filtered);
    expect(filteredSummary.by_layer.boundaries).toBe(summary.by_layer.boundaries);
    expect(filteredSummary.by_layer.centerline).toBe(summary.by_layer.centerline);
    expect(filteredSummary.by_layer.areas).toBe(0);
  });

  it('全部关闭时命令序列为空', () => {
    expect(build({ areas: false, boundaries: false, centerline: false })).toEqual([]);
  });

  it('toggleLayer 翻转单一图层且不改原对象', () => {
    const base: LayerState = { ...DEFAULT_LAYERS };
    const toggled = toggleLayer(base, 'areas');
    expect(toggled.areas).toBe(false);
    expect(toggled.boundaries).toBe(true);
    expect(base.areas).toBe(true);
  });

  it('所有命令坐标有限', () => {
    expect(commandsHaveFiniteCoordinates(all)).toBe(true);
  });

  it('区域命令为 evenodd 填充（支持内环镂空）', () => {
    const areaCommands = all.filter((command) => command.op === 'polygon');
    expect(areaCommands.length).toBeGreaterThan(0);
    for (const command of areaCommands) {
      expect(command.fillRule).toBe('evenodd');
    }
  });
});
