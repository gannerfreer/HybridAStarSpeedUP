/** 统计字段格式化（单位明确；缺失显示 N/A；未知字段原样展示）。 */

export interface StatRow {
  key: string;
  label: string;
  unit: string;
  value: string;
}

interface StatSpec {
  label: string;
  unit: string;
  format?: (value: number | boolean | string) => string;
}

const NUMBER_FORMAT = (digits: number) => (value: number | boolean | string): string =>
  typeof value === 'number' ? value.toFixed(digits) : String(value);

const STAT_SPECS: Record<string, StatSpec> = {
  expanded_nodes: { label: '展开节点数', unit: '', format: NUMBER_FORMAT(0) },
  collision_checks: { label: '碰撞检查次数', unit: '', format: NUMBER_FORMAT(0) },
  planning_time_s: { label: '规划耗时', unit: 's', format: NUMBER_FORMAT(3) },
  solve_time_s: { label: '求解耗时', unit: 's', format: NUMBER_FORMAT(3) },
  segment_count: { label: '段数', unit: '', format: NUMBER_FORMAT(0) },
  direction_changes: { label: '换向次数', unit: '', format: NUMBER_FORMAT(0) },
  total_length_m: { label: '总长度', unit: 'm', format: NUMBER_FORMAT(3) },
  optimization_iterations: { label: '优化迭代', unit: '', format: NUMBER_FORMAT(0) },
  min_clearance_m: { label: '最小间隙', unit: 'm', format: NUMBER_FORMAT(3) },
  velocity_switch_time_s: { label: '换向停车时刻', unit: 's', format: NUMBER_FORMAT(2) },
  budget_exhausted: { label: '预算耗尽', unit: '', format: (value) => (value === true ? '是' : '否') },
  reached_goal: { label: '到达目标', unit: '', format: (value) => (value === true ? '是' : '否') },
  synthetic_seed: { label: '合成种子', unit: '', format: NUMBER_FORMAT(0) },
};

export const PATH_STAT_KEYS: readonly string[] = [
  'expanded_nodes',
  'collision_checks',
  'planning_time_s',
  'segment_count',
  'direction_changes',
  'total_length_m',
];

export const TRAJECTORY_STAT_KEYS: readonly string[] = [
  'optimization_iterations',
  'solve_time_s',
  'min_clearance_m',
  'velocity_switch_time_s',
  'direction_changes',
];

/** 单值格式化（比较表复用）：已知键按规格，缺失 N/A。 */
export function formatStatValue(key: string, value: unknown): string {
  if (value === undefined || value === null) return 'N/A';
  const spec = STAT_SPECS[key];
  if (spec?.format) return spec.format(value as number | boolean | string);
  return formatRaw(value);
}

/** 已知键的标签与单位（比较表列头）。 */
export function statSpecOf(key: string): { label: string; unit: string } {
  const spec = STAT_SPECS[key];
  return { label: spec?.label ?? key, unit: spec?.unit ?? '' };
}

/**
 * 按期望键序列出统计（缺失 → N/A），随后追加样例中的未知键。
 * 布尔按 是/否；数值按规格格式化；字符串原样。
 */
export function formatStatistics(
  statistics: Record<string, unknown>,
  expectedKeys: readonly string[] = [],
): StatRow[] {
  const rows: StatRow[] = [];
  const seen = new Set<string>();
  for (const key of expectedKeys) {
    const spec = STAT_SPECS[key] ?? { label: key, unit: '' };
    const value = statistics[key];
    seen.add(key);
    rows.push({
      key,
      label: spec.label,
      unit: spec.unit,
      value:
        value === undefined || value === null
          ? 'N/A'
          : spec.format
            ? spec.format(value as number | boolean | string)
            : formatRaw(value),
    });
  }
  for (const key of Object.keys(statistics).sort()) {
    if (seen.has(key)) continue;
    const spec = STAT_SPECS[key];
    const value = statistics[key];
    rows.push({
      key,
      label: spec?.label ?? key,
      unit: spec?.unit ?? '',
      value:
        value === undefined || value === null
          ? 'N/A'
          : spec?.format
            ? spec.format(value as number | boolean | string)
            : formatRaw(value),
    });
  }
  return rows;
}

function formatRaw(value: unknown): string {
  if (typeof value === 'number') return Number.isFinite(value) ? String(value) : 'N/A';
  if (typeof value === 'boolean') return value ? '是' : '否';
  if (typeof value === 'string') return value;
  return JSON.stringify(value) ?? 'N/A';
}
