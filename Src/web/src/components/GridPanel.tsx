import { GRID_STATE_LABELS, type GridCellRef, type GridLayerData, type GridViewState } from '../core/grid/types';
import { cellInfo, countStates, gridWorldBounds } from '../core/grid/coords';
import { GRID_COLORS } from '../core/grid/palette';

/**
 * 栅格面板：X-W1 离线夹具 + X-W2 在线数据源（可行驶域 / 规划场景预览）。
 * 图例、透明度、单元检查复用同一套 W1 组件；在线数据明确标注"来自服务"，
 * 降级时显示实际分辨率与降级标记（不得标注为原请求精度）。
 */

export type GridPanelSource = 'fixture' | 'driveable' | 'planning_scene';

export interface GridOnlineSourceView {
  status: 'idle' | 'loading' | 'ready' | 'error' | 'disabled';
  layer: GridLayerData | null;
  error: string | null;
  request_seq: number | null;
  requested_resolution_m: number | null;
  effective_resolution_m: number | null;
  resolution_m: number | null;
  degraded: boolean;
  /** 因在线显示上限（≤1024²）在请求前提高了分辨率。 */
  display_adjusted: boolean;
  display_factor: number;
  stale_ignored: number;
  map_mismatch_ignored: number;
  scene_id: string | null;
  bytes: number | null;
  fetched_at_ms: number | null;
}

export interface GridHealthView {
  status: 'unknown' | 'online' | 'offline';
  protocol_version: string | null;
  scene_rules_version: string | null;
  protocol_ok: boolean;
  scene_rules_ok: boolean;
  reason: string;
  map_matches: boolean;
}

export interface GridPanelProps {
  catalog: string[] | null;
  catalogError: string | null;
  selectedFile: string;
  state: GridViewState;
  enabled: boolean;
  opacity: number;
  pickMode: boolean;
  selectedCell: GridCellRef | null;
  /** X-W2：数据源与在线状态。 */
  source: GridPanelSource;
  online: GridOnlineSourceView;
  health: GridHealthView;
  resolution: number;
  requestWindowLabel: string;
  onSelectSource: (source: GridPanelSource) => void;
  onResolutionChange: (value: number) => void;
  onRequestOnline: () => void;
  onSelectFile: (file: string) => void;
  onReload: () => void;
  onFitView: () => void;
  onToggleEnabled: (next: boolean) => void;
  onOpacityChange: (next: number) => void;
  onTogglePick: () => void;
  onClearCell: () => void;
  onClear: () => void;
}

function formatBytes(bytes: number): string {
  if (bytes >= 1024 * 1024) return `${(bytes / (1024 * 1024)).toFixed(2)} MiB`;
  if (bytes >= 1024) return `${(bytes / 1024).toFixed(1)} KiB`;
  return `${bytes} B`;
}

const SOURCE_OPTIONS: Array<{ value: GridPanelSource; label: string }> = [
  { value: 'fixture', label: '夹具（离线）' },
  { value: 'driveable', label: '可行驶域（在线）' },
  { value: 'planning_scene', label: '规划场景预览（在线）' },
];

export function GridPanel(props: GridPanelProps) {
  const { state, source } = props;
  const fixtureMode = source === 'fixture';
  const layer: GridLayerData | null = fixtureMode ? state.layer : props.online.layer;
  const status = fixtureMode ? state.status : props.online.status;
  const error = fixtureMode ? state.error : props.online.error;
  const cell =
    layer && props.selectedCell ? cellInfo(layer, props.selectedCell.row, props.selectedCell.col) : null;
  const counts = layer ? countStates(layer) : null;
  const bounds = layer ? gridWorldBounds(layer) : null;
  const sourceDisabled =
    !fixtureMode && (!props.health.protocol_ok || (source === 'planning_scene' && !props.health.scene_rules_ok));

  return (
    <section className="panel-section" data-testid="grid-panel">
      <h3>
        栅格图层
        <span className="badge">{fixtureMode ? 'X-W1' : 'X-W2'}</span>
      </h3>

      <div className="kv">
        <span>数据源</span>
        <select
          value={source}
          onChange={(event) => props.onSelectSource(event.target.value as GridPanelSource)}
          aria-label="栅格数据源"
          data-testid="grid-source-select"
        >
          {SOURCE_OPTIONS.map((option) => (
            <option key={option.value} value={option.value}>
              {option.label}
            </option>
          ))}
        </select>
      </div>

      {fixtureMode && (
        <>
          <p className="grid-synthetic-banner" role="note">
            合成显示夹具（fixture），非算法正确性证据
          </p>
          <div className="kv">
            <span>夹具</span>
            <select
              value={props.selectedFile}
              onChange={(event) => props.onSelectFile(event.target.value)}
              aria-label="栅格夹具文件"
              data-testid="grid-file-select"
            >
              {(props.catalog ?? [props.selectedFile]).map((file) => (
                <option key={file} value={file}>
                  {file}
                </option>
              ))}
            </select>
          </div>
        </>
      )}

      {!fixtureMode && (
        <>
          <p className="grid-online-banner" role="note" data-testid="grid-online-banner">
            来自服务（planner_map_service）；请求携带 request_seq，晚到响应不覆盖新请求
          </p>
          <div className="kv">
            <span>服务版本</span>
            <span data-testid="grid-online-health">
              协议 {props.health.protocol_version ?? '缺失'} · 场景规则 {props.health.scene_rules_version ?? '缺失'}
            </span>
          </div>
          <div className="kv">
            <span>地图版本</span>
            <span className={props.health.map_matches ? 'status-online' : 'status-offline'}>
              {props.health.map_matches ? '与当前底图一致' : '与当前底图不一致（禁止请求）'}
            </span>
          </div>
          {sourceDisabled && (
            <p className="panel-error" role="alert" data-testid="grid-online-disabled">
              在线栅格已禁用：{props.health.reason || '版本一致性检查未通过'}
            </p>
          )}
          <div className="kv">
            <span>分辨率 (m/格)</span>
            <input
              type="number"
              min={0.01}
              step={0.01}
              value={props.resolution}
              onChange={(event) => props.onResolutionChange(Number(event.target.value))}
              aria-label="在线栅格请求分辨率"
              data-testid="grid-online-resolution"
              className="number-input"
            />
          </div>
          <div className="kv">
            <span>请求窗口</span>
            <span className="mono" data-testid="grid-online-window">
              {props.requestWindowLabel}
            </span>
          </div>
          <div className="grid-actions">
            <button
              type="button"
              onClick={props.onRequestOnline}
              disabled={sourceDisabled}
              data-testid="grid-online-request"
            >
              请求在线栅格
            </button>
          </div>
        </>
      )}

      <div className="grid-controls">
        <label className="layer-toggle">
          <input
            type="checkbox"
            checked={props.enabled}
            disabled={!layer && status !== 'idle' && status !== 'error' && status !== 'disabled'}
            onChange={(event) => props.onToggleEnabled(event.target.checked)}
            data-testid="grid-enabled"
          />
          显示栅格
        </label>
        <label className="layer-toggle">
          <input
            type="checkbox"
            checked={props.pickMode}
            disabled={!layer}
            onChange={props.onTogglePick}
            data-testid="grid-pick-mode"
          />
          拾取单元
        </label>
      </div>
      <div className="kv grid-opacity">
        <span>透明度</span>
        <input
          type="range"
          min={0}
          max={100}
          step={1}
          value={Math.round(props.opacity * 100)}
          disabled={!layer}
          onChange={(event) => props.onOpacityChange(Number(event.target.value) / 100)}
          aria-label="栅格图层透明度"
          data-testid="grid-opacity"
        />
        <b>{Math.round(props.opacity * 100)}%</b>
      </div>
      <div className="grid-actions">
        <button type="button" onClick={props.onReload} disabled={status === 'loading'}>
          重新加载
        </button>
        <button type="button" onClick={props.onFitView} disabled={!layer}>
          定位到栅格
        </button>
        <button type="button" onClick={props.onClear} disabled={!layer && status !== 'error'}>
          清空栅格
        </button>
      </div>

      <div className="grid-legend" aria-label="栅格图例" data-testid="grid-legend">
        {([0, 1, 2] as const).map((value) => (
          <span className="grid-legend-item" key={value} data-testid={`grid-legend-${value}`}>
            <i style={{ background: GRID_COLORS[value].css }} />
            {GRID_STATE_LABELS[value]}
          </span>
        ))}
      </div>

      {fixtureMode && props.catalogError && <p className="panel-error">夹具目录加载失败：{props.catalogError}</p>}
      {fixtureMode && status === 'loading' && <p className="panel-hint">正在加载栅格夹具…</p>}
      {fixtureMode && status === 'idle' && (
        <p className="panel-hint" data-testid="grid-empty-state">
          未加载栅格夹具（选择文件后自动加载；默认 snapshot_small 解码样例）。
        </p>
      )}
      {fixtureMode && status === 'error' && (
        <p className="panel-error" role="alert" data-testid="grid-error-state">
          栅格夹具加载失败：{error ?? '未知错误'}
        </p>
      )}

      {!fixtureMode && status === 'idle' && (
        <p className="panel-hint" data-testid="grid-online-empty">
          尚未请求在线栅格（选择数据源后自动请求，或点击"请求在线栅格"）。
        </p>
      )}
      {!fixtureMode && status === 'loading' && (
        <p className="panel-hint" data-testid="grid-online-loading">
          正在请求在线栅格…（{source === 'driveable' ? '/map/grid/driveable' : '/map/scene/preview'}）
        </p>
      )}
      {!fixtureMode && status === 'error' && (
        <p className="panel-error" role="alert" data-testid="grid-online-error">
          在线栅格请求失败：{error ?? '未知错误'}
        </p>
      )}

      {layer && (
        <>
          <div className="kv">
            <span>文档</span>
            <b>{layer.kind}</b>
            <span className={fixtureMode ? 'badge badge-local' : 'badge badge-service'}>
              {fixtureMode ? layer.source : `${layer.source} · 来自服务`}
            </span>
            {layer.degraded && <span className="badge badge-approx">已降级</span>}
          </div>
          <div className="kv">
            <span>尺寸</span>
            <span data-testid="grid-online-meta">
              {layer.width}×{layer.height} · 实际 {layer.resolution_m} m/格
              {!fixtureMode && props.online.display_adjusted && (
                <span className="badge badge-approx" data-testid="grid-online-display-cap">
                  显示上限调整 ×{props.online.display_factor}
                </span>
              )}
            </span>
          </div>
          {!fixtureMode && (
            <div className="kv">
              <span>请求分辨率</span>
              <span className="mono">
                {props.online.requested_resolution_m !== null
                  ? `${props.online.requested_resolution_m} m/格`
                  : '-'}
                {props.online.degraded && props.online.effective_resolution_m !== null && (
                  <span data-testid="grid-online-degraded">
                    {' '}
                    → 服务降级为实际 {props.online.effective_resolution_m} m/格（不得按原请求精度标注）
                  </span>
                )}
              </span>
            </div>
          )}
          <div className="kv">
            <span>原点</span>
            <code>
              ({layer.origin.x_m}, {layer.origin.y_m}) · yaw {layer.origin.yaw_rad.toFixed(3)} rad
            </code>
          </div>
          {bounds && (
            <div className="kv">
              <span>范围</span>
              <span>
                x [{bounds.min_x_m.toFixed(2)}, {bounds.max_x_m.toFixed(2)}] · y [
                {bounds.min_y_m.toFixed(2)}, {bounds.max_y_m.toFixed(2)}]
              </span>
            </div>
          )}
          {counts && (
            <div className="kv">
              <span>计数</span>
              <span>
                空闲 {counts.free} · 占据 {counts.occupied} · 未知 {counts.unknown}
              </span>
            </div>
          )}
          {!fixtureMode && (
            <div className="kv">
              <span>请求序号</span>
              <span className="mono" data-testid="grid-online-seq">
                request_seq {props.online.request_seq ?? '-'} · 陈旧丢弃 {props.online.stale_ignored} · 地图版本丢弃{' '}
                {props.online.map_mismatch_ignored}
              </span>
            </div>
          )}
          {!fixtureMode && layer.source === 'planning_scene' && layer.scene_id && (
            <div className="kv">
              <span>scene_id</span>
              <code title={layer.scene_id} data-testid="grid-online-scene-id">
                {layer.scene_id.slice(0, 16)}…
              </code>
              <span className="badge badge-local">当前预览</span>
            </div>
          )}
          {fixtureMode && state.info && (
            <div className="kv">
              <span>文件</span>
              <span>
                {state.info.file} · {formatBytes(state.info.bytes)}
                {state.info.sha256 ? ` · ${state.info.sha256.slice(0, 12)}…` : ''}
              </span>
            </div>
          )}
          {!fixtureMode && props.online.bytes !== null && (
            <div className="kv">
              <span>传输</span>
              <span>{formatBytes(props.online.bytes)}（JSON 原文）</span>
            </div>
          )}
        </>
      )}

      {layer && props.pickMode && (
        <>
          <h4 className="grid-subtitle">选中单元</h4>
          {!cell && <p className="panel-hint">单击画布拾取栅格单元（旋转原点按位姿换算）。</p>}
          {cell && (
            <div data-testid="grid-cell-info">
              <div className="kv">
                <span>行列</span>
                <b data-testid="grid-cell-rowcol">
                  row {cell.row} · col {cell.col}
                </b>
              </div>
              <div className="kv">
                <span>状态</span>
                <span data-testid="grid-cell-state">{GRID_STATE_LABELS[cell.state]}</span>
              </div>
              <div className="kv">
                <span>中心</span>
                <code data-testid="grid-cell-center">
                  x = {cell.center_x_m.toFixed(4)} m · y = {cell.center_y_m.toFixed(4)} m
                </code>
              </div>
              <button type="button" onClick={props.onClearCell}>
                清除选中
              </button>
            </div>
          )}
        </>
      )}
    </section>
  );
}
