import { useRef, type ChangeEvent } from 'react';
import {
  INTERACTION_MODES,
  INTERACTION_MODE_LABELS,
  type InteractionMode,
} from '../core/interaction';
import type { LayerName, LayerState } from '../core/layers';

const LAYER_LABELS: Record<LayerName, string> = {
  areas: '区域',
  boundaries: '道路边界',
  centerline: '中心线',
};

const LAYER_ORDER: readonly LayerName[] = ['areas', 'boundaries', 'centerline'];

export interface ToolbarProps {
  layers: LayerState;
  hasSnapshot: boolean;
  mode: InteractionMode;
  playbackActive: boolean;
  onToggleLayer: (name: LayerName) => void;
  onModeChange: (mode: InteractionMode) => void;
  onResetView: () => void;
  onOpenFile: (file: File) => void;
  onTogglePlayback: () => void;
}

export function Toolbar(props: ToolbarProps) {
  const fileInputRef = useRef<HTMLInputElement | null>(null);

  const handleFileChange = (event: ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.item(0);
    if (file) props.onOpenFile(file);
    event.target.value = '';
  };

  return (
    <header className="toolbar">
      <div className="toolbar-title">
        Planner 地图显示
        <span className="badge">W2</span>
      </div>
      <div className="toolbar-group" role="group" aria-label="交互模式">
        {INTERACTION_MODES.map((mode) => (
          <button
            type="button"
            key={mode}
            className={props.mode === mode ? 'mode-button active' : 'mode-button'}
            onClick={() => props.onModeChange(mode)}
            disabled={!props.hasSnapshot && mode !== 'select'}
          >
            {INTERACTION_MODE_LABELS[mode]}
          </button>
        ))}
      </div>
      <div className="toolbar-group" role="group" aria-label="图层开关">
        {LAYER_ORDER.map((name) => (
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
      <div className="toolbar-group">
        <button
          type="button"
          onClick={props.onTogglePlayback}
          className={props.playbackActive ? 'mode-button active' : undefined}
          data-testid="toggle-playback"
        >
          {props.playbackActive ? '退出回放' : '回放样例'}
        </button>
        <button type="button" onClick={props.onResetView} disabled={!props.hasSnapshot}>
          重置视图
        </button>
        <button type="button" onClick={() => fileInputRef.current?.click()}>
          打开快照…
        </button>
        <input
          ref={fileInputRef}
          type="file"
          accept=".json,application/json"
          onChange={handleFileChange}
          hidden
        />
      </div>
    </header>
  );
}
