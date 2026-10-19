import { useRef, type ChangeEvent } from 'react';
import {
  maxCurvature1PerM,
  minTurnRadiusM,
  vehicleLengthM,
  type VehicleModel,
} from '../core/scene/vehicle';
import type { AlgorithmParams, SceneObstacle } from '../core/scene/types';
import { NumberField } from './NumberField';

export interface ScenePanelProps {
  mapId: string;
  mapVersion: string;
  vehicle: VehicleModel | null;
  vehicleSource: string | null;
  vehicleError: string | null;
  showCursorFootprint: boolean;
  obstacles: SceneObstacle[];
  selectedObstacleId: string | null;
  algorithmParams: AlgorithmParams;
  autosaveStatus: string;
  mismatchWarning: string | null;
  onSelectObstacle: (id: string | null) => void;
  onUpdateObstacleRect: (id: string, field: 'x_min_m' | 'y_min_m' | 'x_max_m' | 'y_max_m', value: number) => void;
  onDeleteObstacle: (id: string) => void;
  onAddObstacleAtCenter: () => void;
  onToggleCursorFootprint: (next: boolean) => void;
  onParamsChange: (params: AlgorithmParams) => void;
  onExportScene: () => void;
  onImportScene: (file: File) => void;
  onRestoreScene: () => void;
  onClearScene: () => void;
  onDismissMismatch: () => void;
}

export function ScenePanel(props: ScenePanelProps) {
  const fileInputRef = useRef<HTMLInputElement | null>(null);
  const { vehicle, obstacles, selectedObstacleId } = props;
  const selected = obstacles.find((obstacle) => obstacle.id === selectedObstacleId) ?? null;

  const handleFileChange = (event: ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.item(0);
    if (file) props.onImportScene(file);
    event.target.value = '';
  };

  return (
    <>
      <section className="panel-section">
        <h3>车辆</h3>
        {!vehicle && !props.vehicleError && <p className="panel-hint">正在加载车辆模型…</p>}
        {props.vehicleError && <p className="panel-error">{props.vehicleError}</p>}
        {vehicle && (
          <>
            <div className="kv">
              <span>模型</span>
              <b>{vehicle.id}</b>
            </div>
            <div className="kv">
              <span>来源</span>
              <code>{props.vehicleSource ?? '-'}</code>
            </div>
            <div className="attributes">
              <table>
                <tbody>
                  <tr>
                    <td>轴距 wheelbase</td>
                    <td>{vehicle.wheelbase_m.toFixed(2)} m</td>
                  </tr>
                  <tr>
                    <td>车宽 width</td>
                    <td>{vehicle.width_m.toFixed(2)} m</td>
                  </tr>
                  <tr>
                    <td>前悬 / 后悬</td>
                    <td>
                      {vehicle.front_overhang_m.toFixed(2)} / {vehicle.rear_overhang_m.toFixed(2)} m
                    </td>
                  </tr>
                  <tr>
                    <td>最大转角</td>
                    <td>{(vehicle.max_steer_rad * 180 / Math.PI).toFixed(1)}° ({vehicle.max_steer_rad.toFixed(3)} rad)</td>
                  </tr>
                  <tr>
                    <td>安全裕量</td>
                    <td>{vehicle.safety_margin_m.toFixed(2)} m</td>
                  </tr>
                  <tr>
                    <td>整车长（含悬）</td>
                    <td>{vehicleLengthM(vehicle).toFixed(2)} m</td>
                  </tr>
                  <tr>
                    <td>最小转弯半径</td>
                    <td>{minTurnRadiusM(vehicle).toFixed(3)} m（轴距/tan 转角）</td>
                  </tr>
                  <tr>
                    <td>最大曲率</td>
                    <td>{maxCurvature1PerM(vehicle).toFixed(6)} 1/m</td>
                  </tr>
                </tbody>
              </table>
            </div>
            <label className="layer-toggle">
              <input
                type="checkbox"
                checked={props.showCursorFootprint}
                onChange={(event) => props.onToggleCursorFootprint(event.target.checked)}
              />
              光标处显示足迹（航向 0°）
            </label>
            <p className="panel-hint">足迹在起终位姿处绘制：车体实线 + 安全裕量虚线。</p>
          </>
        )}
      </section>

      <section className="panel-section">
        <h3>
          测试障碍
          <span className="badge">{obstacles.length}</span>
        </h3>
        <p className="panel-hint">
          切到“障碍”模式：画布拖拽新建矩形、拖动已有障碍移动；此处可编辑坐标与删除。障碍仅为覆盖层，不改写地图夹具。
        </p>
        <div className="button-row">
          <button type="button" onClick={props.onAddObstacleAtCenter} disabled={props.mapId.length === 0}>
            添加障碍（视口中心）
          </button>
        </div>
        {obstacles.length === 0 && <p className="panel-hint">暂无障碍。</p>}
        {obstacles.length > 0 && (
          <div className="obstacle-list">
            {obstacles.map((obstacle) => (
              <button
                type="button"
                key={obstacle.id}
                className={obstacle.id === selectedObstacleId ? 'obstacle-chip selected' : 'obstacle-chip'}
                onClick={() => props.onSelectObstacle(obstacle.id)}
              >
                {obstacle.id} · {obstacle.type === 'rectangle' ? '矩形' : '多边形'}
              </button>
            ))}
          </div>
        )}
        {selected && selected.type === 'rectangle' && (
          <div className="obstacle-editor">
            <div className="kv">
              <span>选中</span>
              <b>{selected.id}</b>
            </div>
            <div className="field-row">
              <NumberField
                label="min x (m)"
                value={Number(selected.x_min_m.toFixed(3))}
                onCommit={(value) => props.onUpdateObstacleRect(selected.id, 'x_min_m', value)}
              />
              <NumberField
                label="min y (m)"
                value={Number(selected.y_min_m.toFixed(3))}
                onCommit={(value) => props.onUpdateObstacleRect(selected.id, 'y_min_m', value)}
              />
            </div>
            <div className="field-row">
              <NumberField
                label="max x (m)"
                value={Number(selected.x_max_m.toFixed(3))}
                onCommit={(value) => props.onUpdateObstacleRect(selected.id, 'x_max_m', value)}
              />
              <NumberField
                label="max y (m)"
                value={Number(selected.y_max_m.toFixed(3))}
                onCommit={(value) => props.onUpdateObstacleRect(selected.id, 'y_max_m', value)}
              />
            </div>
            <div className="kv">
              <span>尺寸</span>
              <span>
                {(selected.x_max_m - selected.x_min_m).toFixed(2)} × {(selected.y_max_m - selected.y_min_m).toFixed(2)} m
              </span>
            </div>
            <div className="button-row">
              <button type="button" onClick={() => props.onDeleteObstacle(selected.id)}>
                删除障碍
              </button>
            </div>
          </div>
        )}
        {selected && selected.type === 'convex_polygon' && (
          <div className="obstacle-editor">
            <div className="kv">
              <span>选中</span>
              <b>{selected.id}</b>
              <span className="panel-hint">（多边形仅支持查看/删除，编辑请用 JSON 场景导入）</span>
            </div>
            <div className="button-row">
              <button type="button" onClick={() => props.onDeleteObstacle(selected.id)}>
                删除障碍
              </button>
            </div>
          </div>
        )}
      </section>

      <section className="panel-section">
        <h3>算法参数</h3>
        <div className="field-row">
          <NumberField
            label="Hybrid 时间预算 (s)"
            value={props.algorithmParams.hybrid_astar.time_budget_s}
            step={0.5}
            onCommit={(value) =>
              props.onParamsChange({
                ...props.algorithmParams,
                hybrid_astar: { ...props.algorithmParams.hybrid_astar, time_budget_s: value },
              })
            }
          />
          <NumberField
            label="Hybrid 节点预算"
            value={props.algorithmParams.hybrid_astar.node_budget}
            step={1000}
            onCommit={(value) =>
              props.onParamsChange({
                ...props.algorithmParams,
                hybrid_astar: { ...props.algorithmParams.hybrid_astar, node_budget: Math.round(value) },
              })
            }
          />
        </div>
        <div className="field-row">
          <label className="layer-toggle">
            <input
              type="checkbox"
              checked={props.algorithmParams.corridor.enabled}
              onChange={(event) =>
                props.onParamsChange({
                  ...props.algorithmParams,
                  corridor: { ...props.algorithmParams.corridor, enabled: event.target.checked },
                })
              }
            />
            安全走廊启用
          </label>
          <NumberField
            label="走廊裕量 (m)"
            value={props.algorithmParams.corridor.margin_m}
            step={0.1}
            onCommit={(value) =>
              props.onParamsChange({
                ...props.algorithmParams,
                corridor: { ...props.algorithmParams.corridor, margin_m: value },
              })
            }
          />
        </div>
        <p className="panel-hint">占位参数，随场景保存/重载；W5 由规划服务消费。</p>
      </section>

      <section className="panel-section">
        <h3>场景</h3>
        <div className="kv">
          <span>地图</span>
          <b>{props.mapId || '-'}</b>
        </div>
        <div className="kv">
          <span>版本</span>
          <code title={props.mapVersion}>{props.mapVersion ? `${props.mapVersion.slice(0, 12)}…` : '-'}</code>
        </div>
        <div className="kv">
          <span>自动保存</span>
          <span>{props.autosaveStatus}</span>
        </div>
        {props.mismatchWarning && (
          <div className="mismatch-warning" role="alert">
            {props.mismatchWarning}
            <button type="button" onClick={props.onDismissMismatch}>
              知道了
            </button>
          </div>
        )}
        <div className="button-row">
          <button type="button" onClick={props.onExportScene}>
            保存场景（下载 JSON）
          </button>
          <button type="button" onClick={() => fileInputRef.current?.click()}>
            导入场景…
          </button>
          <input
            ref={fileInputRef}
            type="file"
            accept=".json,application/json"
            onChange={handleFileChange}
            hidden
            data-testid="scene-import-input"
          />
        </div>
        <div className="button-row">
          <button type="button" onClick={props.onRestoreScene}>
            从本地恢复
          </button>
          <button type="button" onClick={props.onClearScene}>
            清除场景
          </button>
        </div>
        <p className="panel-hint">
          场景含地图引用、起终位姿、障碍与算法参数；地图版本不一致时拒绝套用并提示。
        </p>
      </section>
    </>
  );
}
