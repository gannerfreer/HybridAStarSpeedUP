/** v(t)/κ(t) 小图（纯 SVG；路径构建为纯函数，便于测试）。 */

export interface ProfileSeriesPoint {
  t_s: number;
  value: number;
}

export interface ProfileChartProps {
  title: string;
  unit: string;
  points: readonly ProfileSeriesPoint[];
  currentTime: number | null;
  testId?: string;
}

const CHART_WIDTH = 280;
const CHART_HEIGHT = 64;
const PADDING = 6;

export function profileValueRange(points: readonly ProfileSeriesPoint[]): [number, number] {
  if (points.length === 0) return [0, 1];
  let min = Number.POSITIVE_INFINITY;
  let max = Number.NEGATIVE_INFINITY;
  for (const point of points) {
    min = Math.min(min, point.value);
    max = Math.max(max, point.value);
  }
  if (min === max) {
    const pad = Math.max(1e-3, Math.abs(min) * 0.1);
    return [min - pad, max + pad];
  }
  const pad = (max - min) * 0.08;
  return [min - pad, max + pad];
}

export function buildProfilePolyline(
  points: readonly ProfileSeriesPoint[],
  width = CHART_WIDTH,
  height = CHART_HEIGHT,
  padding = PADDING,
): string {
  if (points.length === 0) return '';
  const tMin = points[0].t_s;
  const tMax = points[points.length - 1].t_s;
  const [vMin, vMax] = profileValueRange(points);
  const spanT = tMax - tMin || 1;
  const spanV = vMax - vMin || 1;
  return points
    .map((point, index) => {
      const x = padding + ((point.t_s - tMin) / spanT) * (width - 2 * padding);
      const y = height - padding - ((point.value - vMin) / spanV) * (height - 2 * padding);
      return `${index === 0 ? 'M' : 'L'} ${x.toFixed(2)} ${y.toFixed(2)}`;
    })
    .join(' ');
}

export function ProfileChart(props: ProfileChartProps) {
  const polyline = buildProfilePolyline(props.points);
  const [vMin, vMax] = profileValueRange(props.points);
  const tMin = props.points[0]?.t_s ?? 0;
  const tMax = props.points[props.points.length - 1]?.t_s ?? 1;
  const spanT = tMax - tMin || 1;
  const markerX =
    props.currentTime === null
      ? null
      : PADDING + ((Math.min(tMax, Math.max(tMin, props.currentTime)) - tMin) / spanT) * (CHART_WIDTH - 2 * PADDING);
  return (
    <div className="profile-chart" data-testid={props.testId}>
      <div className="profile-title">
        {props.title} <span className="panel-hint">{props.unit}</span>
        <span className="profile-range">
          [{vMin.toFixed(2)}, {vMax.toFixed(2)}]
        </span>
      </div>
      <svg width="100%" viewBox={`0 0 ${CHART_WIDTH} ${CHART_HEIGHT}`} role="img" aria-label={props.title}>
        <rect x="0" y="0" width={CHART_WIDTH} height={CHART_HEIGHT} fill="#0b1220" stroke="#1e293b" />
        {polyline && <path d={polyline} fill="none" stroke="#38bdf8" strokeWidth="1.5" />}
        {markerX !== null && (
          <line x1={markerX} y1="0" x2={markerX} y2={CHART_HEIGHT} stroke="#facc15" strokeWidth="1" />
        )}
      </svg>
    </div>
  );
}
