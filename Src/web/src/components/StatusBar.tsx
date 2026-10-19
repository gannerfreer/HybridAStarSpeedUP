import { INTERACTION_MODE_LABELS, type InteractionMode } from '../core/interaction';
import { metersPerPixel, type Viewport } from '../core/transform';
import type { ServiceStatus } from '../core/query_state';
import type { MapSnapshot } from '../core/types';
import type { MapSourceInfo } from '../App';

export interface StatusBarProps {
  snapshot: MapSnapshot | null;
  sourceInfo: MapSourceInfo | null;
  viewport: Viewport | null;
  canvasSize: { width_px: number; height_px: number } | null;
  service: ServiceStatus;
  mode: InteractionMode;
  playbackActive: boolean;
}

export function StatusBar(props: StatusBarProps) {
  const { snapshot, sourceInfo, viewport, canvasSize, service, mode, playbackActive } = props;
  return (
    <footer className="statusbar">
      <span>
        地图：
        {snapshot
          ? `${snapshot.map_id} · ${snapshot.lanelets.length} lanelets · ${snapshot.areas.length} areas · ${snapshot.counts.points ?? '-'} points`
          : '-'}
      </span>
      <span>数据源：{sourceInfo ? `${sourceInfo.kind} · ${sourceInfo.rel_path}` : '-'}</span>
      <span className="mono">
        sha256：{sourceInfo?.sha256 ? `${sourceInfo.sha256.slice(0, 12)}…` : '-'}
      </span>
      <span>比例：{viewport ? `${metersPerPixel(viewport).toFixed(3)} m/px` : '-'}</span>
      <span>
        服务：
        <b className={service.online ? 'status-online' : 'status-offline'}>
          {service.online ? '在线' : service.checked ? '离线' : '检查中'}
        </b>
        · 模式：{INTERACTION_MODE_LABELS[mode]}
        {playbackActive && <b className="status-online"> · 回放：合成样例</b>}
      </span>
      <span className="mono">
        画布：{canvasSize ? `${canvasSize.width_px}×${canvasSize.height_px}` : '-'} · 窗口：
        {typeof window !== 'undefined' ? `${window.innerWidth}×${window.innerHeight}` : '-'} · DPR{' '}
        {typeof window !== 'undefined' ? window.devicePixelRatio : '-'}
      </span>
    </footer>
  );
}
