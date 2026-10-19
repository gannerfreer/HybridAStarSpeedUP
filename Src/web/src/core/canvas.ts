import type { DrawCommand } from './render';

/**
 * Canvas 2D 的最小结构化接口：`CanvasRenderingContext2D` 结构上兼容，
 * Node 侧测试可用记录型假实现验证命令执行序列。
 */
export interface Canvas2DLike {
  clearRect(x: number, y: number, width: number, height: number): void;
  fillRect(x: number, y: number, width: number, height: number): void;
  beginPath(): void;
  moveTo(x: number, y: number): void;
  lineTo(x: number, y: number): void;
  closePath(): void;
  arc(x: number, y: number, radius: number, startAngle: number, endAngle: number): void;
  fill(fillRule?: 'nonzero' | 'evenodd'): void;
  stroke(): void;
  setLineDash(segments: number[]): void;
  fillText(text: string, x: number, y: number): void;
  fillStyle: string | CanvasGradient | CanvasPattern;
  strokeStyle: string | CanvasGradient | CanvasPattern;
  lineWidth: number;
  font: string;
  textAlign: CanvasTextAlign;
  textBaseline: CanvasTextBaseline;
}

/** 执行绘制命令序列（命令由 `render.ts` 纯函数生成）。 */
export function executeDrawCommands(ctx: Canvas2DLike, commands: readonly DrawCommand[]): void {
  for (const command of commands) {
    if (command.op === 'polygon') {
      ctx.beginPath();
      for (const ring of command.rings) {
        if (ring.length === 0) continue;
        ctx.moveTo(ring[0][0], ring[0][1]);
        for (let index = 1; index < ring.length; index += 1) {
          ctx.lineTo(ring[index][0], ring[index][1]);
        }
        ctx.closePath();
      }
      ctx.fillStyle = command.fill;
      ctx.fill(command.fillRule);
      if (command.stroke) {
        ctx.strokeStyle = command.stroke;
        ctx.lineWidth = command.lineWidth ?? 1;
        ctx.stroke();
      }
    } else if (command.op === 'polyline') {
      if (command.points.length < 2) continue;
      ctx.beginPath();
      ctx.moveTo(command.points[0][0], command.points[0][1]);
      for (let index = 1; index < command.points.length; index += 1) {
        ctx.lineTo(command.points[index][0], command.points[index][1]);
      }
      ctx.setLineDash(command.dash ? [...command.dash] : []);
      ctx.strokeStyle = command.stroke;
      ctx.lineWidth = command.lineWidth;
      ctx.stroke();
      ctx.setLineDash([]);
    } else if (command.op === 'text') {
      ctx.font = `${command.fontPx}px sans-serif`;
      ctx.textAlign = command.align ?? 'center';
      ctx.textBaseline = 'middle';
      ctx.fillStyle = command.fill;
      ctx.fillText(command.text, command.x, command.y);
    } else {
      ctx.beginPath();
      ctx.arc(command.cx, command.cy, command.radius, 0, Math.PI * 2);
      ctx.fillStyle = command.fill;
      ctx.fill();
    }
  }
}

/** 清屏 → 背景 → 命令序列（供 Canvas 组件单次重绘调用）。 */
export function renderSceneToCanvas(
  ctx: Canvas2DLike,
  commands: readonly DrawCommand[],
  widthPx: number,
  heightPx: number,
  background: string,
): void {
  ctx.clearRect(0, 0, widthPx, heightPx);
  ctx.fillStyle = background;
  ctx.fillRect(0, 0, widthPx, heightPx);
  executeDrawCommands(ctx, commands);
}
