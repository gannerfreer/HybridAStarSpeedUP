import type { CropResult, RouteResult } from './service/types';
import type { WindowFilterResult } from './window';
import type { BoundsM } from './types';

/** W2 查询交互的界面状态类型（App 持有，面板/画布消费）。 */

export type SelectionKind = 'lanelet' | 'area';
export type QuerySource = 'service' | 'local';

export interface SelectionInfo {
  kind: SelectionKind;
  id: string;
  attributes: Record<string, string>;
  /** lanelet：左/右/中心线点数；area：外环/内环数与点数。 */
  geometry: {
    left?: number;
    right?: number;
    centerline?: number;
    outer?: number;
    inner_rings?: number;
    inner_points?: number;
  };
  source: QuerySource;
}

export interface ServiceStatus {
  online: boolean;
  checked: boolean;
  base: string;
  error?: string;
}

export type TopologyKind = 'following' | 'preceding' | 'adjacent_left' | 'adjacent_right';

export interface TopologyState {
  kind: TopologyKind;
  ids: string[];
  status: string;
  reason: string;
  source: QuerySource;
}

export interface RouteState {
  from: string;
  to: string;
  loading: boolean;
  result: RouteResult | null;
  error: string | null;
}

export interface WindowQueryState {
  bounds: BoundsM;
  loading: boolean;
  approximate: boolean;
  crop: CropResult | null;
  local: WindowFilterResult | null;
  error: string | null;
}
