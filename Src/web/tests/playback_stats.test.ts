import { describe, expect, it } from 'vitest';
import {
  formatStatistics,
  PATH_STAT_KEYS,
  TRAJECTORY_STAT_KEYS,
} from '../src/core/playback/stats';
import { parsePlanResult } from '../src/core/playback/parse';
import { readPlaybackFile } from './playback_fixtures';

describe('统计格式化', () => {
  it('路径统计：标签、单位、格式化（RS 样例）', () => {
    const result = parsePlanResult(readPlaybackFile('path_rs_open.json').value);
    const rows = formatStatistics(result.statistics, PATH_STAT_KEYS);
    const byKey = new Map(rows.map((row) => [row.key, row]));
    expect(byKey.get('segment_count')?.value).toBe('2');
    expect(byKey.get('total_length_m')?.value).toBe('13.854');
    expect(byKey.get('total_length_m')?.unit).toBe('m');
    expect(byKey.get('planning_time_s')?.value).toBe('0.002');
    expect(byKey.get('direction_changes')?.value).toBe('0');
    expect(byKey.get('expanded_nodes')?.value).toBe('N/A'); // 缺失 → N/A
    // 未列出的样例字段追加（synthetic_seed）
    expect(byKey.get('synthetic_seed')?.value).toBe('17');
  });

  it('失败样例：缺失字段 N/A、布尔 是/否', () => {
    const result = parsePlanResult(readPlaybackFile('path_hybrid_failure.json').value);
    const rows = formatStatistics(result.statistics, PATH_STAT_KEYS);
    const byKey = new Map(rows.map((row) => [row.key, row]));
    expect(byKey.get('expanded_nodes')?.value).toBe('258401');
    expect(byKey.get('segment_count')?.value).toBe('N/A');
    expect(byKey.get('total_length_m')?.value).toBe('N/A');
    expect(byKey.get('budget_exhausted')?.value).toBe('否');
    expect(byKey.get('reached_goal')?.value).toBe('否');
  });

  it('轨迹统计：Dftpav 样例字段齐全', () => {
    const result = parsePlanResult(readPlaybackFile('dftpav_trajectory.json').value);
    const rows = formatStatistics(result.statistics, TRAJECTORY_STAT_KEYS);
    const byKey = new Map(rows.map((row) => [row.key, row]));
    expect(byKey.get('optimization_iterations')?.value).toBe('42');
    expect(byKey.get('solve_time_s')?.value).toBe('0.860');
    expect(byKey.get('min_clearance_m')?.value).toBe('0.350');
    expect(byKey.get('velocity_switch_time_s')?.value).toBe('4.00');
    expect(byKey.get('direction_changes')?.value).toBe('1');
  });

  it('未知键原样追加、非有限数值 N/A', () => {
    const rows = formatStatistics({ custom_flag: 'x', bad: Number.NaN }, ['missing_key']);
    expect(rows[0]).toMatchObject({ key: 'missing_key', value: 'N/A' });
    expect(rows.find((row) => row.key === 'custom_flag')?.value).toBe('x');
    expect(rows.find((row) => row.key === 'bad')?.value).toBe('N/A');
  });
});
