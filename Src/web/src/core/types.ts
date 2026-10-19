/**
 * M4 公共地图快照（schema_version 1, type=map_snapshot）的 TypeScript 类型。
 * 字段语义以 `Src/map/examples/data/README.md` 字段表为准；解析与校验见 `parse.ts`。
 */

export interface MapPoint {
  /** 对象 ID 一律字符串（原始 OSM 节点 ID 或 Lanelet2 计算 ID），避免整数精度丢失。 */
  id: string;
  x_m: number;
  y_m: number;
  z_m: number;
}

export interface BoundsM {
  min_x_m: number;
  min_y_m: number;
  max_x_m: number;
  max_y_m: number;
}

export interface CoordinateSystem {
  /** 恒为 "map"：局部米制地图系。 */
  frame_id: 'map';
  /** 恒为 "m"。 */
  unit: 'm';
  x_axis: string;
  y_axis: string;
  projection: string;
  origin: { lat_deg: number; lon_deg: number; ele_m: number };
}

export interface MapSourceRef {
  path: string;
  bytes: number;
}

export interface TrafficRules {
  location: string;
  participant: string;
}

export interface Lanelet {
  id: string;
  attributes: Record<string, string>;
  /** 左/北边界，点序与地图一致。 */
  left: MapPoint[];
  /** 右/南边界。 */
  right: MapPoint[];
  /** 几何中心线；`include_centerline=false` 的导出中缺省。 */
  centerline?: MapPoint[];
}

export interface MapArea {
  id: string;
  attributes: Record<string, string>;
  /** 多段线串接的外环。 */
  outer: MapPoint[];
  /** 每个洞环一项。 */
  inner: MapPoint[][];
}

export interface RegulatoryElement {
  id: string;
  attributes: Record<string, string>;
  /** 角色名 → 被引用对象 ID 字符串数组（如 refers、ref_line）。 */
  roles: Record<string, string[]>;
}

/**
 * 解析后的快照。`areas` / `regulatory_elements` 在原始 JSON 中可选，
 * 解析时统一归一化为数组（缺省为 `[]`），其余字段原样保留。
 */
export interface MapSnapshot {
  schema_version: 1;
  type: 'map_snapshot';
  map_id: string;
  map_version: string;
  source: MapSourceRef;
  coordinate_system: CoordinateSystem;
  bounds_m: BoundsM;
  counts: Record<string, number>;
  traffic_rules: TrafficRules;
  lanelets: Lanelet[];
  areas: MapArea[];
  regulatory_elements: RegulatoryElement[];
}
