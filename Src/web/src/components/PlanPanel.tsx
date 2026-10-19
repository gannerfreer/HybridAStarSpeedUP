import { formatStatistics, PATH_STAT_KEYS, TRAJECTORY_STAT_KEYS } from '../core/playback/stats';
import type { TrajectorySample } from '../core/playback/time';
import type { PlaybackLayers } from '../core/playback/types';
import type { JobTrackerState } from '../core/plan/tracker';
import type { PlanResult } from '../core/plan/types';
import { buildComparisonTable, type ComparisonRun } from '../core/plan/compare';
import { ProfileChart, type ProfileSeriesPoint } from './ProfileChart';

export interface PlanRunHistoryEntry {
  status: string;
  stage: string;
  at_ms: number;
}

export interface PlanRunView {
  backend: string;
  request_id: string;
  job_id: string;
  job_seq: number;
  status: string;
  stage: string;
  reason: string;
  error: string | null;
  result: PlanResult | null;
  /** 协议 §3.4：规划实际场景内容标识（map 阶段后出现）。 */
  scene_id: string | null;
  scene_version: string | null;
  /** 状态/阶段变迁记录（事件推送与轮询合并；供 UI 与验收证据）。 */
  history: PlanRunHistoryEntry[];
}

export interface PlanSceneSummary {
  map_id: string;
  map_version: string;
  map_matches: boolean;
  has_poses: boolean;
  obstacle_count: number;
  vehicle_ref: string;
}

export interface PlanPanelProps {
  health: 'unknown' | 'online' | 'offline';
  base: string;
  scene: PlanSceneSummary;
  backends: readonly string[];
  backend: string;
  compareA: string;
  compareB: string;
  runs: Record<string, PlanRunView>;
  activeBackend: string | null;
  tracker: JobTrackerState;
  busy: boolean;
  error: string | null;
  layers: PlaybackLayers;
  time_s: number;
  duration_s: number;
  playing: boolean;
  current: TrajectorySample | null;
  profile: ProfileSeriesPoint[];
  curvatureProfile: ProfileSeriesPoint[];
  onRefreshHealth: () => void;
  onBackendChange: (backend: string) => void;
  onCompareChange: (slot: 'a' | 'b', backend: string) => void;
  onSubmit: () => void;
  onSubmitCompare: () => void;
  onCancel: () => void;
  onToggleLayer: (name: keyof PlaybackLayers) => void;
  onPlayPause: () => void;
  onSeek: (t: number) => void;
  onClearResults: () => void;
}

const PLAN_LAYER_LABELS: Record<keyof PlaybackLayers, string> = {
  paths: '路径',
  curves: '曲线',
  corridors: '走廊',
  trajectories: '轨迹',
  collision: '碰撞点',
};

const STATUS_LABELS: Record<string, string> = {
  queued: '排队中',
  running: '运行中',
  succeeded: '成功',
  failed: '失败',
  cancelled: '已取消',
  success: '成功',
  no_path: '无解',
  timeout: '超时',
  optimization_failed: '优化失败',
  validation_failed: '校验未通过',
  invalid_input: '无效输入',
};

function isSuccessStatus(status: string): boolean {
  return status === 'succeeded' || status === 'success';
}

function statusLabel(status: string): string {
  return STATUS_LABELS[status] ?? status;
}

export function PlanPanel(props: PlanPanelProps) {
  const { scene, tracker, runs } = props;
  const active = props.activeBackend ? runs[props.activeBackend] : undefined;
  const result = active?.result ?? null;
  const compareRuns: ComparisonRun[] = Object.values(runs).map((run) => ({
    backend: run.backend,
    status: run.error ? 'error' : run.status,
    result: run.result,
    error: run.error,
  }));
  const comparison = compareRuns.length >= 2 ? buildComparisonTable(compareRuns) : null;
  const statKeys = [...new Set([...PATH_STAT_KEYS, ...TRAJECTORY_STAT_KEYS])];
  const statRows = result ? formatStatistics(result.statistics, statKeys) : [];

  const canSubmit = props.health === 'online' && scene.has_poses && scene.map_matches && !props.busy;

  return (
    <>
      <section className="panel-section" data-testid="plan-panel">
        <h3>
          规划服务
          <span
            className={
              props.health === 'online' ? 'service-dot online' : props.health === 'offline' ? 'service-dot offline' : 'service-dot'
            }
          />
        </h3>
        <div className="kv">
          <span>状态</span>
          <b>{props.health === 'online' ? '在线' : props.health === 'offline' ? '离线' : '检查中…'}</b>
        </div>
        <div className="kv">
          <span>地址</span>
          <code>{props.base}</code>
        </div>
        <button type="button" onClick={props.onRefreshHealth} disabled={props.busy}>
          重新检查服务
        </button>
      </section>

      <section className="panel-section">
        <h3>提交规划</h3>
        <div className="kv">
          <span>地图</span>
          <b>{scene.map_id || '-'}</b>
          <span className={scene.map_matches ? 'status-online' : 'status-offline'}>
            {scene.map_matches ? '版本一致' : '版本不匹配（禁止提交）'}
          </span>
        </div>
        <div className="kv">
          <span>起终位姿</span>
          <span className={scene.has_poses ? '' : 'status-offline'}>
            {scene.has_poses ? '已设置' : '未设置（设起点/设终点模式）'}
          </span>
        </div>
        <div className="kv">
          <span>障碍 / 车辆</span>
          <span>
            {scene.obstacle_count} 个 · {scene.vehicle_ref || '-'}
          </span>
        </div>
        <div className="field-row">
          <label className="field">
            <span>Hybrid 后端</span>
            <select
              value={props.backend}
              onChange={(event) => props.onBackendChange(event.target.value)}
              data-testid="plan-backend"
            >
              {props.backends.map((backend) => (
                <option key={backend} value={backend}>
                  {backend}
                </option>
              ))}
            </select>
          </label>
        </div>
        <div className="button-row">
          <button type="button" onClick={props.onSubmit} disabled={!canSubmit} data-testid="plan-submit">
            提交规划
          </button>
          <button type="button" onClick={props.onSubmitCompare} disabled={!canSubmit} data-testid="plan-submit-compare">
            对比提交（{props.compareA} / {props.compareB}）
          </button>
        </div>
        <div className="field-row">
          <label className="field">
            <span>对比后端 A</span>
            <select value={props.compareA} onChange={(event) => props.onCompareChange('a', event.target.value)}>
              {props.backends.map((backend) => (
                <option key={backend} value={backend}>
                  {backend}
                </option>
              ))}
            </select>
          </label>
          <label className="field">
            <span>对比后端 B</span>
            <select value={props.compareB} onChange={(event) => props.onCompareChange('b', event.target.value)}>
              {props.backends.map((backend) => (
                <option key={backend} value={backend}>
                  {backend}
                </option>
              ))}
            </select>
          </label>
        </div>
        {props.error && <p className="panel-error" data-testid="plan-error">{props.error}</p>}
        <p className="panel-hint">
          请求携带当前场景（web_scene）：地图版本、起终位姿、障碍与算法参数；地图版本不一致时提交被阻止。
        </p>
      </section>

      <section className="panel-section">
        <h3>当前任务</h3>
        <div className="kv">
          <span>job_seq</span>
          <span className="mono">
            {tracker.latest_seq}（陈旧忽略 {tracker.stale_ignored} 次）
          </span>
        </div>
        {!active && <p className="panel-hint">尚未提交任务。</p>}
        {active && (
          <>
            <div className="kv">
              <span>后端</span>
              <b>{active.backend}</b>
            </div>
            <div className="kv">
              <span>请求 ID</span>
              <code data-testid="plan-request-id">{active.request_id}</code>
            </div>
            <div className="kv">
              <span>任务 ID</span>
              <code data-testid="plan-job-id">{active.job_id}</code>
            </div>
            <div className="kv">
              <span>状态</span>
              <b className={isSuccessStatus(active.status) ? 'status-online' : active.error ? 'status-offline' : ''}>
                {active.error ? '错误' : statusLabel(active.status)}
              </b>
              {active.stage && <span className="panel-hint">阶段：{active.stage}</span>}
            </div>
            {active.reason && <p className="panel-hint">{active.reason}</p>}
            {active.history.length > 0 && (
              <div className="kv">
                <span>进度</span>
                <span className="mono" data-testid="plan-history">
                  {active.history
                    .map((entry) => (entry.stage ? `${entry.status}/${entry.stage}` : entry.status))
                    .join(' → ')}
                </span>
              </div>
            )}
            {active.error && (
              <p className="panel-error" data-testid="plan-run-error">
                {active.error}
              </p>
            )}
            <div className="button-row">
              <button
                type="button"
                onClick={props.onCancel}
                disabled={active.status === 'queued' || active.status === 'running' ? false : true}
                data-testid="plan-cancel"
              >
                取消任务
              </button>
              <button type="button" onClick={props.onClearResults} data-testid="plan-clear">
                清除结果
              </button>
            </div>
          </>
        )}
      </section>

      {result && (
        <section className="panel-section">
          <h3>规划结果</h3>
          <div className="kv">
            <span>地图版本</span>
            <code title={result.map_version}>{result.map_version ? `${result.map_version.slice(0, 12)}…` : '-'}</code>
          </div>
          <div className="kv">
            <span>请求 ID</span>
            <code>{result.request_id || '-'}</code>
          </div>
          <div className="kv">
            <span>状态</span>
            <b className={isSuccessStatus(result.status) ? 'status-online' : 'status-offline'}>{statusLabel(result.status)}</b>
          </div>
          {result.reason && <p className="panel-hint">{result.reason}</p>}
          {result.validation && (
            <div className={result.validation.valid ? 'validation-ok' : 'mismatch-warning'} role="status">
              轨迹校验：{result.validation.valid ? '通过' : `未通过 —— ${result.validation.reason}`}
            </div>
          )}
          <div className="kv">
            <span>几何</span>
            <span>
              {result.path.reduce((sum, segment) => sum + segment.points.length, 0)} 路径点 · {result.corridor.length} 走廊 ·{' '}
              {result.trajectory.length} 轨迹点
            </span>
          </div>
          {statRows.length > 0 && (
            <table className="stats-table">
              <tbody>
                {statRows.map((row) => (
                  <tr key={row.key}>
                    <td>{row.label}</td>
                    <td className="mono">
                      {row.value}
                      {row.unit ? ` ${row.unit}` : ''}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          )}
          <div className="toolbar-group" role="group" aria-label="结果图层">
            {(Object.keys(PLAN_LAYER_LABELS) as Array<keyof PlaybackLayers>).map((name) => (
              <label className="layer-toggle" key={name}>
                <input type="checkbox" checked={props.layers[name]} onChange={() => props.onToggleLayer(name)} />
                {PLAN_LAYER_LABELS[name]}
              </label>
            ))}
          </div>
          {props.duration_s > 0 && (
            <>
              <div className="button-row">
                <button type="button" onClick={props.onPlayPause} data-testid="plan-play">
                  {props.playing ? '暂停' : '播放'}
                </button>
                <span className="pose-preview">
                  t = {props.time_s.toFixed(2)} / {props.duration_s.toFixed(2)} s
                </span>
              </div>
              <input
                type="range"
                min={0}
                max={props.duration_s}
                step={0.01}
                value={Math.min(props.time_s, props.duration_s)}
                onChange={(event) => props.onSeek(Number(event.target.value))}
                className="playback-slider"
                data-testid="plan-slider"
                aria-label="结果回放时间"
              />
              {props.current && (
                <div className="kv">
                  <span>当前</span>
                  <span className="mono">
                    v = {props.current.velocity_mps.toFixed(3)} m/s · κ = {props.current.curvature_inv_m.toFixed(5)} 1/m ·{' '}
                    {props.current.velocity_mps >= 0 ? '前进' : '倒车'}
                  </span>
                </div>
              )}
              <ProfileChart
                title="v(t)"
                unit="m/s"
                points={props.profile}
                currentTime={props.time_s}
                testId="plan-profile-velocity"
              />
              <ProfileChart
                title="κ(t)"
                unit="1/m"
                points={props.curvatureProfile}
                currentTime={props.time_s}
                testId="plan-profile-curvature"
              />
            </>
          )}
        </section>
      )}

      {comparison && (
        <section className="panel-section">
          <h3>算法比较</h3>
          <table className="stats-table compare-table" data-testid="plan-comparison">
            <thead>
              <tr>
                <th>指标</th>
                {comparison.backends.map((backend) => (
                  <th key={backend}>{backend}</th>
                ))}
              </tr>
            </thead>
            <tbody>
              {comparison.rows.map((row) => (
                <tr key={row.label}>
                  <td>
                    {row.label}
                    {row.unit ? ` (${row.unit})` : ''}
                  </td>
                  {row.values.map((value, index) => (
                    <td key={comparison.backends[index]} className="mono">
                      {value}
                    </td>
                  ))}
                </tr>
              ))}
            </tbody>
          </table>
        </section>
      )}
    </>
  );
}
