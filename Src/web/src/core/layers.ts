/** 图层开关状态（W1：区域 / 道路边界 / 中心线）。 */
export interface LayerState {
  areas: boolean;
  boundaries: boolean;
  centerline: boolean;
}

export type LayerName = keyof LayerState;

export const DEFAULT_LAYERS: Readonly<LayerState> = Object.freeze({
  areas: true,
  boundaries: true,
  centerline: true,
});

export function toggleLayer(state: LayerState, name: LayerName): LayerState {
  return { ...state, [name]: !state[name] };
}
