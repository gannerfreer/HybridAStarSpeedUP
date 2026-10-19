import { formatStatValue, statSpecOf } from '../playback/stats';
import type { PlanResult } from './types';

/** 算法比较表（同一场景、不同 hybrid_backend 的并排统计）。 */

export interface ComparisonRun {
  backend: string;
  status: string;
  result: PlanResult | null;
  error: string | null;
}

export interface ComparisonRow {
  label: string;
  unit: string;
  values: string[];
}

export interface ComparisonTable {
  backends: string[];
  rows: ComparisonRow[];
}

const COMPARE_STAT_KEYS: readonly string[] = [
  'planning_time_s',
  'solve_time_s',
  'total_length_m',
  'direction_changes',
  'expanded_nodes',
  'collision_checks',
];

export function buildComparisonTable(runs: readonly ComparisonRun[]): ComparisonTable {
  const rows: ComparisonRow[] = [];
  rows.push({
    label: '状态',
    unit: '',
    values: runs.map((run) => (run.error ? `错误：${run.error}` : run.status)),
  });
  for (const key of COMPARE_STAT_KEYS) {
    const spec = statSpecOf(key);
    rows.push({
      label: spec.label,
      unit: spec.unit,
      values: runs.map((run) => (run.result ? formatStatValue(key, run.result.statistics[key]) : 'N/A')),
    });
  }
  rows.push({
    label: '轨迹校验',
    unit: '',
    values: runs.map((run) => {
      if (!run.result) return 'N/A';
      if (!run.result.validation) return 'N/A';
      return run.result.validation.valid ? '通过' : `未通过（${run.result.validation.reason}）`;
    }),
  });
  return { backends: runs.map((run) => run.backend), rows };
}
