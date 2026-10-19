import { INTERACTION_MODE_HINTS, type InteractionMode } from '../core/interaction';
import { degToRad, radToDeg, type Pose2d } from '../core/pose';
import type { ReactNode } from 'react';
import type {
  RouteState,
  SelectionInfo,
  ServiceStatus,
  TopologyKind,
  TopologyState,
  WindowQueryState,
} from '../core/query_state';
import type { MapInfoPayload } from '../core/service/types';
import type { BoundsM } from '../core/types';
import { NumberField } from './NumberField';

const TOPOLOGY_LABELS: Record<TopologyKind, string> = {
  following: '后继 following',
  preceding: '前驱 preceding',
  adjacent_left: '左邻 adjacent',
  adjacent_right: '右邻 adjacent',
};

function SourceBadge(props: { source: 'service' | 'local' | 'approx' }) {
  if (props.source === 'service') return <span className="badge badge-service">服务</span>;
  if (props.source === 'local') return <span className="badge badge-local">本地</span>;
  return <span className="badge badge-approx">本地近似</span>;
}

export interface InspectorPanelProps {
  service: ServiceStatus;
  mapInfo: MapInfoPayload | null;
  mode: InteractionMode;
  selection: SelectionInfo | null;
  topology: TopologyState | null;
  routeState: RouteState;
  startPose: Pose2d | null;
  goalPose: Pose2d | null;
  windowState: WindowQueryState | null;
  busy: boolean;
  children?: ReactNode;
  onRefreshService: () => void;
  onClearSelection: () => void;
  onSelectLanelet: (id: string) => void;
  onSelectArea: (id: string) => void;
  onTopology: (kind: TopologyKind) => void;
  onSetPoseFromSelection: (kind: 'start' | 'goal') => void;
  onPoseFieldChange: (kind: 'start' | 'goal', field: 'x' | 'y' | 'yaw_deg', value: number) => void;
  onClearPose: (kind: 'start' | 'goal') => void;
  onRouteChange: (field: 'from' | 'to', value: string) => void;
  onComputeRoute: () => void;
  onClearRoute: () => void;
  onWindowBoundsChange: (bounds: BoundsM) => void;
  onQueryWindow: () => void;
  onClearWindow: () => void;
}

export function InspectorPanel(props: InspectorPanelProps) {
  const {
    service,
    mapInfo,
    selection,
    topology,
    routeState,
    startPose,
    goalPose,
    windowState,
  } = props;

  const attributeEntries = selection ? Object.entries(selection.attributes) : [];

  return (
    <aside className="inspector" aria-label="查询交互面板">
      <section className="panel-section">
        <h3>
          服务
          <span className={service.online ? 'service-dot online' : 'service-dot offline'} />
        </h3>
        <div className="kv">
          <span>状态</span>
          <b>{service.online ? '在线' : service.checked ? '离线' : '检查中…'}</b>
        </div>
        <div className="kv">
          <span>地址</span>
          <code>{service.base}</code>
        </div>
        {service.error && <p className="panel-error">{service.error}</p>}
        {mapInfo && (
          <>
            <div className="kv">
              <span>地图</span>
              <b>{mapInfo.map_id}</b>
            </div>
            <div className="kv">
              <span>版本</span>
              <code title={mapInfo.map_version}>{mapInfo.map_version.slice(0, 12)}…</code>
            </div>
            <div className="kv">
              <span>对象</span>
              <span>
                {mapInfo.counts.lanelets ?? '-'} lanelets · {mapInfo.counts.areas ?? '-'} areas
              </span>
            </div>
          </>
        )}
        <button type="button" onClick={props.onRefreshService} disabled={props.busy}>
          重新检查服务
        </button>
        <p className="panel-hint">{INTERACTION_MODE_HINTS[props.mode]}</p>
      </section>

      <section className="panel-section">
        <h3>选中对象</h3>
        {!selection && <p className="panel-hint">单击地图选择 lanelet 或区域（服务离线时使用本地命中）。</p>}
        {selection && (
          <>
            <div className="kv">
              <span>类型</span>
              <b>{selection.kind === 'lanelet' ? 'lanelet' : '区域 area'}</b>
              <SourceBadge source={selection.source} />
            </div>
            <div className="kv">
              <span>ID</span>
              <code>{selection.id}</code>
            </div>
            <div className="kv">
              <span>几何</span>
              <span>
                {selection.kind === 'lanelet'
                  ? `左 ${selection.geometry.left ?? 0} / 右 ${selection.geometry.right ?? 0} / 中心线 ${selection.geometry.centerline ?? 0} 点`
                  : `外环 ${selection.geometry.outer ?? 0} 点 / 内环 ${selection.geometry.inner_rings ?? 0}`}
              </span>
            </div>
            <div className="attributes">
              <table>
                <tbody>
                  {attributeEntries.length === 0 && (
                    <tr>
                      <td colSpan={2}>（无属性）</td>
                    </tr>
                  )}
                  {attributeEntries.map(([key, value]) => (
                    <tr key={key}>
                      <td>{key}</td>
                      <td>{value}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
            <div className="button-row">
              {selection.kind === 'lanelet' && (
                <>
                  <button type="button" onClick={() => props.onSetPoseFromSelection('start')}>
                    设为起点
                  </button>
                  <button type="button" onClick={() => props.onSetPoseFromSelection('goal')}>
                    设为终点
                  </button>
                </>
              )}
              <button type="button" onClick={props.onClearSelection}>
                清除选择
              </button>
            </div>
            {selection.kind === 'lanelet' && (
              <div className="button-row" role="group" aria-label="拓扑查询">
                {(['following', 'preceding', 'adjacent_left', 'adjacent_right'] as TopologyKind[]).map((kind) => (
                  <button type="button" key={kind} onClick={() => props.onTopology(kind)} disabled={props.busy}>
                    {TOPOLOGY_LABELS[kind]}
                  </button>
                ))}
              </div>
            )}
          </>
        )}
      </section>

      {topology && (
        <section className="panel-section">
          <h3>拓扑结果</h3>
          <div className="kv">
            <span>查询</span>
            <b>{TOPOLOGY_LABELS[topology.kind]}</b>
            <SourceBadge source={topology.source} />
          </div>
          <div className="kv">
            <span>状态</span>
            <span>
              {topology.status}
              {topology.reason ? `（${topology.reason}）` : ''}
            </span>
          </div>
          <div className="id-list">
            {topology.ids.length === 0 && <span className="panel-hint">无结果</span>}
            {topology.ids.map((id) => (
              <button type="button" key={id} className="id-chip" onClick={() => props.onSelectLanelet(id)}>
                {id}
              </button>
            ))}
          </div>
        </section>
      )}

      <section className="panel-section">
        <h3>起终位姿</h3>
        {(['start', 'goal'] as const).map((kind) => {
          const pose = kind === 'start' ? startPose : goalPose;
          return (
            <div className="pose-block" key={kind}>
              <div className="pose-title">
                <b>{kind === 'start' ? '起点' : '终点'}</b>
                {pose ? (
                  <span className="pose-preview">
                    {pose.x_m.toFixed(2)}, {pose.y_m.toFixed(2)} m · {radToDeg(pose.yaw_rad).toFixed(1)}°
                  </span>
                ) : (
                  <span className="panel-hint">未设置</span>
                )}
              </div>
              {pose && (
                <>
                  <div className="field-row">
                    <NumberField
                      label="x (m)"
                      value={Number(pose.x_m.toFixed(3))}
                      onCommit={(value) => props.onPoseFieldChange(kind, 'x', value)}
                    />
                    <NumberField
                      label="y (m)"
                      value={Number(pose.y_m.toFixed(3))}
                      onCommit={(value) => props.onPoseFieldChange(kind, 'y', value)}
                    />
                  </div>
                  <div className="field-row">
                    <NumberField
                      label="航向 (°)"
                      value={Number(radToDeg(pose.yaw_rad).toFixed(2))}
                      step={1}
                      onCommit={(value) => props.onPoseFieldChange(kind, 'yaw_deg', value)}
                    />
                    <button type="button" onClick={() => props.onClearPose(kind)}>
                      清除
                    </button>
                  </div>
                </>
              )}
            </div>
          );
        })}
        <p className="panel-hint">在画布上拖动起点/终点圆点可直接调整航向（度 → 弧度 {degToRad(1).toFixed(4)}）。</p>
      </section>

      <section className="panel-section">
        <h3>路由</h3>
        <div className="field-row">
          <label className="field">
            <span>起点 lanelet</span>
            <input
              type="text"
              value={routeState.from}
              onChange={(event) => props.onRouteChange('from', event.target.value)}
              placeholder="如 38992"
            />
          </label>
          <label className="field">
            <span>终点 lanelet</span>
            <input
              type="text"
              value={routeState.to}
              onChange={(event) => props.onRouteChange('to', event.target.value)}
              placeholder="如 38994"
            />
          </label>
        </div>
        <div className="button-row">
          <button type="button" onClick={props.onComputeRoute} disabled={props.busy || routeState.loading}>
            {routeState.loading ? '计算中…' : '计算路由'}
          </button>
          <button type="button" onClick={props.onClearRoute}>
            清除路由
          </button>
        </div>
        {routeState.error && <p className="panel-error">{routeState.error}</p>}
        {routeState.result && routeState.result.status === 'success' && (
          <>
            <div className="kv">
              <span>长度</span>
              <b>{routeState.result.length_m.toFixed(2)} m</b>
            </div>
            <div className="kv">
              <span>序列</span>
              <span>{routeState.result.lanelet_ids.length} 段</span>
            </div>
            <div className="id-list">
              {routeState.result.lanelet_ids.map((id) => (
                <button type="button" key={id} className="id-chip" onClick={() => props.onSelectLanelet(id)}>
                  {id}
                </button>
              ))}
            </div>
          </>
        )}
        {routeState.result && routeState.result.status !== 'success' && (
          <p className="panel-error">
            路由状态：{routeState.result.status}
            {routeState.result.reason ? `（${routeState.result.reason}）` : ''}
          </p>
        )}
      </section>

      <section className="panel-section">
        <h3>窗口查询</h3>
        <p className="panel-hint">切换到“窗口查询”模式后在画布拖拽矩形；也可直接编辑坐标。</p>
        <div className="field-row">
          <NumberField
            label="min x (m)"
            value={windowState ? windowState.bounds.min_x_m : 0}
            onCommit={(value) =>
              props.onWindowBoundsChange({
                ...(windowState?.bounds ?? { min_x_m: 0, min_y_m: 0, max_x_m: 0, max_y_m: 0 }),
                min_x_m: value,
              })
            }
          />
          <NumberField
            label="min y (m)"
            value={windowState ? windowState.bounds.min_y_m : 0}
            onCommit={(value) =>
              props.onWindowBoundsChange({
                ...(windowState?.bounds ?? { min_x_m: 0, min_y_m: 0, max_x_m: 0, max_y_m: 0 }),
                min_y_m: value,
              })
            }
          />
        </div>
        <div className="field-row">
          <NumberField
            label="max x (m)"
            value={windowState ? windowState.bounds.max_x_m : 0}
            onCommit={(value) =>
              props.onWindowBoundsChange({
                ...(windowState?.bounds ?? { min_x_m: 0, min_y_m: 0, max_x_m: 0, max_y_m: 0 }),
                max_x_m: value,
              })
            }
          />
          <NumberField
            label="max y (m)"
            value={windowState ? windowState.bounds.max_y_m : 0}
            onCommit={(value) =>
              props.onWindowBoundsChange({
                ...(windowState?.bounds ?? { min_x_m: 0, min_y_m: 0, max_x_m: 0, max_y_m: 0 }),
                max_y_m: value,
              })
            }
          />
        </div>
        <div className="button-row">
          <button type="button" onClick={props.onQueryWindow} disabled={props.busy || windowState?.loading}>
            {windowState?.loading ? '查询中…' : '查询窗口'}
          </button>
          <button type="button" onClick={props.onClearWindow}>
            清除窗口
          </button>
        </div>
        {windowState?.error && <p className="panel-error">{windowState.error}</p>}
        {windowState && (
          <>
            <div className="kv">
              <span>来源</span>
              {windowState.approximate ? <SourceBadge source="approx" /> : <SourceBadge source="service" />}
            </div>
            {windowState.crop && (
              <>
                <div className="kv">
                  <span>窗口原点</span>
                  <span>
                    ({windowState.crop.window_frame.x_m.toFixed(2)}, {windowState.crop.window_frame.y_m.toFixed(2)}) m ·
                    yaw {radToDeg(windowState.crop.window_frame.yaw_rad).toFixed(1)}°
                  </span>
                </div>
                <div className="kv">
                  <span>变换</span>
                  <span className="panel-hint">地图坐标 = 窗口坐标 + 窗口原点（yaw=0 时）</span>
                </div>
                <div className="kv">
                  <span>裁剪</span>
                  <span>
                    {windowState.crop.paths.length} 路径 · {windowState.crop.points.length} 点 ·{' '}
                    {windowState.crop.lanelet_ids.length} lanelets
                  </span>
                </div>
                {windowState.crop.diagnostics.length > 0 && (
                  <p className="panel-hint">{windowState.crop.diagnostics.join('；')}</p>
                )}
                <div className="id-list">
                  {windowState.crop.lanelet_ids.slice(0, 30).map((id) => (
                    <button type="button" key={id} className="id-chip" onClick={() => props.onSelectLanelet(id)}>
                      {id}
                    </button>
                  ))}
                </div>
              </>
            )}
            {windowState.local && (
              <div className="kv">
                <span>本地过滤</span>
                <span>
                  {windowState.local.lanelet_ids.length} lanelets · {windowState.local.area_ids.length} areas（未裁剪）
                </span>
              </div>
            )}
          </>
        )}
      </section>
      {props.children}
    </aside>
  );
}
