/**
 * 比例尺：在当前米/像素下选择“整长度”标尺（0.1 m ～ 100 km），
 * 使其像素宽度不超过上限且尽可能大。纯函数，浏览器与 Node 测试共用。
 */

export interface ScaleBarSpec {
  length_m: number;
  width_px: number;
  label: string;
}

const NICE_LENGTHS_M: readonly number[] = [
  0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000, 10000, 20000, 50000, 100000,
];

export function formatScaleBarLabel(lengthM: number): string {
  if (lengthM >= 1000) return `${lengthM / 1000} km`;
  if (lengthM >= 1) return `${lengthM} m`;
  return `${Math.round(lengthM * 100)} cm`;
}

export function chooseScaleBar(metersPerPixel: number, maxWidthPx = 140): ScaleBarSpec {
  if (!Number.isFinite(metersPerPixel) || metersPerPixel <= 0) {
    throw new RangeError(`metersPerPixel must be a positive finite number, got ${metersPerPixel}`);
  }
  if (!Number.isFinite(maxWidthPx) || maxWidthPx <= 0) {
    throw new RangeError(`maxWidthPx must be a positive finite number, got ${maxWidthPx}`);
  }
  let chosen = NICE_LENGTHS_M[0];
  for (const candidate of NICE_LENGTHS_M) {
    if (candidate / metersPerPixel <= maxWidthPx) chosen = candidate;
  }
  return {
    length_m: chosen,
    width_px: chosen / metersPerPixel,
    label: formatScaleBarLabel(chosen),
  };
}
