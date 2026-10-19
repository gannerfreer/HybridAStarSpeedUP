import { countStates, gridWorldBounds } from '../core/grid/coords';
import { PLAN_SCENE_COLORS } from '../core/grid/palette';
import { comparePreviewAndPlanScene, type PlanSceneSnapshotState } from '../core/plan/scene_snapshot';

/**
 * X-W2 规划实际场景面板：展示某次规划经 `GET /plan/scene/{scene_id}` 拉取的
 * 输入快照（与"当前预览"显式区分；历史场景不覆盖预览）。
 * 404（未知/已淘汰）与协议版本不一致均有可见状态。
 */

export interface PlanScenePanelProps {
  state: PlanSceneSnapshotState;
  overlayEnabled: boolean;
  /** 当前预览层的 scene_id（在线规划场景预览）；用于显式比较。 */
  previewSceneId: string | null;
  mapMatches: boolean;
  protocolOk: boolean;
  protocolReason: string;
  /** 地图服务与规划服务 scene_rules_version 不一致时的警告。 */
  sceneRulesMismatch: string | null;
  onToggleOverlay: (next: boolean) => void;
  onRefetch: () => void;
  onFitView: () => void;
  onClear: () => void;
}

const STATUS_LABELS: Record<PlanSceneSnapshotState['status'], string> = {
  idle: '未获取（提交规划并取得结果后自动拉取）',
  loading: '正在拉取规划输入快照…',
  ready: '已展示',
  not_found: '已淘汰或未知（404 unknown_scene）',
  error: '拉取失败',
  disabled: '已禁用（协议版本不一致）',
};

export function PlanScenePanel(props: PlanScenePanelProps) {
  const { state } = props;
  const layer = state.layer;
  const counts = layer ? countStates(layer) : null;
  const bounds = layer ? gridWorldBounds(layer) : null;
  const comparison = comparePreviewAndPlanScene(props.previewSceneId, state.scene_id);
  const sameAsPreview = comparison === 'same';

  return (
    <section className="panel-section" data-testid="plan-scene-panel">
      <h3>
        规划实际场景
        <span className="badge badge-history" data-testid="plan-scene-history-badge">
          历史（不覆盖当前预览）
        </span>
      </h3>
      <p className="panel-hint" role="note">
        某次规划实际使用的输入快照（GET /plan/scene/&#123;scene_id&#125;，内容寻址）；与"当前预览"独立标注。
      </p>
      {props.sceneRulesMismatch && (
        <p className="mismatch-warning" role="alert" data-testid="plan-scene-rules-warning">
          {props.sceneRulesMismatch}
        </p>
      )}
      {!props.protocolOk && (
        <p className="panel-error" role="alert" data-testid="plan-scene-protocol-disabled">
          规划快照展示已禁用：{props.protocolReason || '协议版本一致性检查未通过'}
        </p>
      )}
      <div className="kv">
        <span>状态</span>
        <b data-testid="plan-scene-status">{STATUS_LABELS[state.status]}</b>
      </div>
      {state.scene_id && (
        <div className="kv">
          <span>scene_id</span>
          <code title={state.scene_id} data-testid="plan-scene-scene-id">
            {state.scene_id.slice(0, 16)}…
          </code>
        </div>
      )}
      {state.scene_version && (
        <div className="kv">
          <span>scene_version</span>
          <code data-testid="plan-scene-scene-version">{state.scene_version}</code>
        </div>
      )}
      {(state.job_id || state.backend) && (
        <div className="kv">
          <span>来源任务</span>
          <span className="mono">
            {state.backend ?? '-'} · {state.job_id ?? '-'}
          </span>
        </div>
      )}
      {state.status === 'not_found' && (
        <p className="panel-hint" data-testid="plan-scene-not-found">
          该 scene_id 不在规划服务保留集合（LRU 16，重启失效）；可凭有效窗口/分辨率/障碍用当前预览重新生成。
        </p>
      )}
      {state.status === 'error' && (
        <p className="panel-error" data-testid="plan-scene-error">
          {state.error ?? '未知错误'}
        </p>
      )}
      {state.status === 'disabled' && state.error && (
        <p className="panel-error" data-testid="plan-scene-disabled-error">
          {state.error}
        </p>
      )}

      {layer && (
        <>
          <div className="kv">
            <span>地图版本</span>
            <span className={props.mapMatches ? 'status-online' : 'status-offline'}>
              {props.mapMatches ? '与当前底图一致' : '与当前底图不一致（不叠加）'}
            </span>
          </div>
          <div className="kv">
            <span>尺寸</span>
            <span data-testid="plan-scene-meta">
              {layer.width}×{layer.height} · 实际 {layer.resolution_m} m/格
              {layer.degraded && <span className="badge badge-approx">已降级</span>}
            </span>
          </div>
          {layer.degraded && layer.effective_resolution_m !== null && (
            <div className="kv">
              <span>降级</span>
              <span className="mono" data-testid="plan-scene-degraded">
                请求 {layer.requested_resolution_m} m/格 → 实际 {layer.effective_resolution_m} m/格
              </span>
            </div>
          )}
          {layer.window && (
            <div className="kv">
              <span>窗口</span>
              <span className="mono">
                x [{layer.window.min_x_m.toFixed(2)}, {layer.window.max_x_m.toFixed(2)}] · y [
                {layer.window.min_y_m.toFixed(2)}, {layer.window.max_y_m.toFixed(2)}]
              </span>
            </div>
          )}
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
              <span data-testid="plan-scene-counts">
                空闲 {counts.free} · 占据 {counts.occupied} · 未知 {counts.unknown}
              </span>
            </div>
          )}
          <div className="kv">
            <span>与当前预览</span>
            <span className={sameAsPreview ? 'status-online' : 'status-offline'} data-testid="plan-scene-vs-preview">
              {props.previewSceneId === null
                ? '无当前预览 scene_id 可比较'
                : sameAsPreview
                  ? 'scene_id 一致（内容相同）'
                  : 'scene_id 不同（独立历史场景）'}
            </span>
          </div>
          <div className="grid-controls">
            <label className="layer-toggle">
              <input
                type="checkbox"
                checked={props.overlayEnabled && props.mapMatches}
                disabled={!props.mapMatches}
                onChange={(event) => props.onToggleOverlay(event.target.checked)}
                data-testid="plan-scene-overlay"
              />
              叠加显示（占据=品红）
            </label>
          </div>
          <div className="grid-legend" aria-label="规划场景图例">
            <span className="grid-legend-item">
              <i style={{ background: PLAN_SCENE_COLORS[1].css }} />
              规划场景占据
            </span>
            <span className="grid-legend-item">
              <i style={{ background: PLAN_SCENE_COLORS[2].css }} />
              未知
            </span>
          </div>
          <div className="grid-actions">
            <button type="button" onClick={props.onRefetch} disabled={state.status === 'loading'}>
              重新拉取
            </button>
            <button type="button" onClick={props.onFitView}>
              定位到场景
            </button>
            <button type="button" onClick={props.onClear}>
              清除快照
            </button>
          </div>
        </>
      )}
    </section>
  );
}
