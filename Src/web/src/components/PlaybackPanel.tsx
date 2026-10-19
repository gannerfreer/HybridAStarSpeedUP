import { useEffect, useRef, type ChangeEvent } from 'react';
import type { PlaybackBundle, PlaybackLayers, PlaybackSampleData } from '../core/playback/types';
import { formatStatistics, PATH_STAT_KEYS, TRAJECTORY_STAT_KEYS } from '../core/playback/stats';
import type { TrajectorySample } from '../core/playback/time';
import { ProfileChart, type ProfileSeriesPoint } from './ProfileChart';

export interface PlaybackPanelProps {
  active: boolean;
  loading: boolean;
  error: string | null;
  bundle: PlaybackBundle | null;
  selected: PlaybackSampleData | null;
  layers: PlaybackLayers;
  time_s: number;
  duration_s: number;
  playing: boolean;
  staleWarning: string | null;
  current: TrajectorySample | null;
  profile: ProfileSeriesPoint[];
  curvatureProfile: ProfileSeriesPoint[];
  onLoad: () => void;
  onExit: () => void;
  onSelect: (id: string) => void;
  onToggleLayer: (name: keyof PlaybackLayers) => void;
  onPlayPause: () => void;
  onSeek: (t_s: number) => void;
}

const LAYER_LABELS: Record<keyof PlaybackLayers, string> = {
  paths: '路径',
  curves: '曲线',
  corridors: '走廊',
  trajectories: '轨迹',
  collision: '碰撞点',
};

const KIND_LABELS: Record<string, string> = {
  map_snapshot: '地图',
  path: '路径',
  path_direction_change: '换向路径',
  path_failure: '失败',
  bspline_curve: 'B 样条',
  polynomial_curve: '多项式',
  corridor: '走廊',
  trajectory: '轨迹',
  collision_case: '碰撞',
  stale_request: '过期',
};

export function PlaybackPanel(props: PlaybackPanelProps) {
  const { bundle, selected } = props;
  const rootRef = useRef<HTMLElement | null>(null);
  useEffect(() => {
    if (props.active && rootRef.current) {
      rootRef.current.scrollIntoView({ block: 'start' });
    }
  }, [props.active]);
  const statKeys =
    selected?.kind === 'plan_result' && selected.result.trajectory.length > 0
      ? TRAJECTORY_STAT_KEYS
      : PATH_STAT_KEYS;
  const statRows =
    selected?.kind === 'plan_result' ? formatStatistics(selected.result.statistics, statKeys) : [];
  const isStale = selected?.kind === 'stale_result';
  const failed = selected?.kind === 'plan_result' && selected.result.status !== 'success';

  return (
    <>
      <section className="panel-section" ref={rootRef} data-testid="playback-panel">
        <h3>回放样例</h3>
        <div className="synthetic-banner" role="note">
          合成显示样例（synthetic）——仅作显示测试，<b>非算法正确性证据</b>。
          {bundle && <span> 索引：tests/fixtures/v4/playback/index.json</span>}
        </div>
        <div className="button-row">
          {!props.active && (
            <button type="button" onClick={props.onLoad} disabled={props.loading}>
              {props.loading ? '加载中…' : '加载回放样例'}
            </button>
          )}
          {props.active && (
            <button type="button" onClick={props.onExit}>
              退出回放（恢复默认地图）
            </button>
          )}
        </div>
        {props.error && <p className="panel-error">{props.error}</p>}
        {bundle && bundle.issues.length > 0 && (
          <div className="panel-error">
            {bundle.issues.map((issue) => (
              <div key={issue}>{issue}</div>
            ))}
          </div>
        )}
        {bundle && (
          <div className="kv">
            <span>样例</span>
            <span>
              {bundle.samples.length} 个已加载（SHA256 校验通过） · {bundle.index.samples.length} 个索引
            </span>
          </div>
        )}
      </section>

      {bundle && (
        <section className="panel-section">
          <h3>样例列表</h3>
          <div className="sample-list">
            {bundle.samples.map((sample) => (
              <button
                type="button"
                key={sample.sample.id}
                className={sample.sample.id === props.selected?.sample.id ? 'sample-chip selected' : 'sample-chip'}
                onClick={() => props.onSelect(sample.sample.id)}
                title={sample.sample.expected_display}
                data-sample-id={sample.sample.id}
              >
                {KIND_LABELS[sample.sample.kind] ?? sample.sample.kind} · {sample.sample.id}
              </button>
            ))}
          </div>
          {selected && (
            <p className="panel-hint">
              <b>{selected.sample.id}</b>（{selected.sample.kind}）：{selected.sample.expected_display}
            </p>
          )}
        </section>
      )}

      {bundle && (
        <section className="panel-section">
          <h3>结果图层</h3>
          <div className="toolbar-group">
            {(Object.keys(LAYER_LABELS) as Array<keyof PlaybackLayers>).map((name) => (
              <label className="layer-toggle" key={name}>
                <input
                  type="checkbox"
                  checked={props.layers[name]}
                  onChange={() => props.onToggleLayer(name)}
                />
                {LAYER_LABELS[name]}
              </label>
            ))}
          </div>
        </section>
      )}

      {selected && isStale && (
        <section className="panel-section">
          <h3>过期结果</h3>
          <div className="mismatch-warning" role="alert">
            {props.staleWarning ?? '过期结果：已忽略，不覆盖当前结果'}
          </div>
          <div className="kv">
            <span>请求</span>
            <span>
              {selected.stale.request_id}（seq {selected.stale.request_seq} &lt; 当前 {selected.stale.current_request_seq}）
            </span>
          </div>
          <div className="kv">
            <span>版本</span>
            <span>
              {selected.stale.map_version} ≠ 当前 {selected.stale.current_map_version}
            </span>
          </div>
          <p className="panel-hint">{selected.stale.reason}</p>
        </section>
      )}

      {selected && failed && (
        <section className="panel-section">
          <h3>规划失败</h3>
          <div className="mismatch-warning" role="alert">
            状态：<b>{selected.result.status}</b>（算法无解，非资源终止） · 不绘制路径
          </div>
          <p className="panel-hint">{selected.result.reason}</p>
          {selected.result.statistics.budget_exhausted !== undefined && (
            <div className="kv">
              <span>预算耗尽</span>
              <span>{selected.result.statistics.budget_exhausted ? '是' : '否'}</span>
            </div>
          )}
        </section>
      )}

      {selected && statRows.length > 0 && (
        <section className="panel-section">
          <h3>统计</h3>
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
        </section>
      )}

      {selected && selected.kind === 'collision_case' && (
        <section className="panel-section">
          <h3>碰撞样例</h3>
          <div className="mismatch-warning" role="alert">
            verdict = <b>{selected.collision.verdict}</b>：{selected.collision.reason}
          </div>
        </section>
      )}

      {selected && props.duration_s > 0 && (
        <section className="panel-section">
          <h3>时间回放</h3>
          <div className="button-row">
            <button type="button" onClick={props.onPlayPause} data-testid="playback-play">
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
            data-testid="playback-slider"
            aria-label="回放时间"
          />
          {props.current && (
            <div className="kv">
              <span>当前</span>
              <span className="mono">
                x = {props.current.pose.x_m.toFixed(2)} m · y = {props.current.pose.y_m.toFixed(2)} m ·{' '}
                {props.current.velocity_mps >= 0 ? '前进' : '倒车'}
              </span>
            </div>
          )}
          <table className="stats-table">
            <tbody>
              <tr>
                <td>速度 v(t)</td>
                <td className="mono">{props.current ? `${props.current.velocity_mps.toFixed(3)} m/s` : 'N/A'}</td>
              </tr>
              <tr>
                <td>曲率 κ(t)</td>
                <td className="mono">
                  {props.current ? `${props.current.curvature_inv_m.toFixed(5)} 1/m` : 'N/A'}
                </td>
              </tr>
              <tr>
                <td>加速度 a(t)</td>
                <td className="mono">
                  {props.current ? `${props.current.acceleration_mps2.toFixed(3)} m/s²` : 'N/A'}
                </td>
              </tr>
            </tbody>
          </table>
          <ProfileChart
            title="v(t)"
            unit="m/s"
            points={props.profile}
            currentTime={props.time_s}
            testId="profile-velocity"
          />
          <ProfileChart
            title="κ(t)"
            unit="1/m"
            points={props.curvatureProfile}
            currentTime={props.time_s}
            testId="profile-curvature"
          />
        </section>
      )}
    </>
  );
}
