import { describe, expect, it } from 'vitest';
import { executeDrawCommands, renderSceneToCanvas, type Canvas2DLike } from '../src/core/canvas';
import type { DrawCommand } from '../src/core/render';

/** 记录型假 Canvas 上下文：验证命令执行序列，无需浏览器。 */
class RecordingContext implements Canvas2DLike {
  ops: string[] = [];
  fillStyle: string | CanvasGradient | CanvasPattern = '';
  strokeStyle: string | CanvasGradient | CanvasPattern = '';
  lineWidth = 1;
  font = '';
  textAlign: CanvasTextAlign = 'start';
  textBaseline: CanvasTextBaseline = 'alphabetic';

  clearRect(x: number, y: number, width: number, height: number): void {
    this.ops.push(`clearRect(${x},${y},${width},${height})`);
  }

  fillRect(x: number, y: number, width: number, height: number): void {
    this.ops.push(`fillRect(${x},${y},${width},${height})`);
  }

  beginPath(): void {
    this.ops.push('beginPath');
  }

  moveTo(x: number, y: number): void {
    this.ops.push(`moveTo(${x},${y})`);
  }

  lineTo(x: number, y: number): void {
    this.ops.push(`lineTo(${x},${y})`);
  }

  closePath(): void {
    this.ops.push('closePath');
  }

  arc(x: number, y: number, radius: number, startAngle: number, endAngle: number): void {
    this.ops.push(`arc(${x},${y},${radius},${startAngle},${endAngle})`);
  }

  fill(fillRule?: 'nonzero' | 'evenodd'): void {
    this.ops.push(`fill(${fillRule ?? 'nonzero'})`);
  }

  stroke(): void {
    this.ops.push(`stroke(${String(this.strokeStyle)})`);
  }

  setLineDash(segments: number[]): void {
    this.ops.push(`dash(${segments.join(' ')})`);
  }

  fillText(text: string, x: number, y: number): void {
    this.ops.push(`fillText(${text},${x},${y})`);
  }
}

describe('Canvas 命令执行器', () => {
  it('polygon 命令：单次 beginPath，每个环 closePath，evenodd 填充', () => {
    const ctx = new RecordingContext();
    const commands: DrawCommand[] = [
      {
        op: 'polygon',
        layer: 'areas',
        rings: [
          [
            [0, 0],
            [10, 0],
            [10, 10],
          ],
          [
            [2, 2],
            [4, 2],
            [4, 4],
          ],
        ],
        fill: '#112233',
        fillRule: 'evenodd',
        stroke: '#445566',
        lineWidth: 1,
      },
    ];
    executeDrawCommands(ctx, commands);
    expect(ctx.ops.filter((op) => op === 'beginPath')).toHaveLength(1);
    expect(ctx.ops.filter((op) => op === 'closePath')).toHaveLength(2);
    expect(ctx.ops).toContain('fill(evenodd)');
    expect(ctx.ops.filter((op) => op.startsWith('stroke('))).toHaveLength(1);
    expect(ctx.fillStyle).toBe('#112233');
  });

  it('polyline 命令：设置虚线后复位，退化为单点时跳过', () => {
    const ctx = new RecordingContext();
    const commands: DrawCommand[] = [
      {
        op: 'polyline',
        layer: 'centerline',
        points: [
          [0, 0],
          [5, 5],
          [10, 0],
        ],
        stroke: '#f5a623',
        lineWidth: 1.25,
        dash: [7, 5],
      },
      { op: 'polyline', layer: 'boundaries', points: [[1, 1]], stroke: '#fff', lineWidth: 1 },
    ];
    executeDrawCommands(ctx, commands);
    expect(ctx.ops).toContain('dash(7 5)');
    expect(ctx.ops[ctx.ops.length - 1]).toBe('dash()');
    expect(ctx.ops.filter((op) => op.startsWith('moveTo'))).toHaveLength(1);
    expect(ctx.ops.filter((op) => op.startsWith('stroke('))).toHaveLength(1);
  });

  it('circle 命令使用 arc + fill', () => {
    const ctx = new RecordingContext();
    executeDrawCommands(ctx, [{ op: 'circle', layer: 'areas', cx: 3, cy: 4, radius: 2, fill: '#fff' }]);
    expect(ctx.ops).toContain(`arc(3,4,2,0,${Math.PI * 2})`);
    expect(ctx.ops).toContain('fill(nonzero)');
  });

  it('renderSceneToCanvas 先清屏再填充背景', () => {
    const ctx = new RecordingContext();
    renderSceneToCanvas(ctx, [], 100, 50, '#0b1220');
    expect(ctx.ops[0]).toBe('clearRect(0,0,100,50)');
    expect(ctx.ops[1]).toBe('fillRect(0,0,100,50)');
    expect(ctx.fillStyle).toBe('#0b1220');
  });
});
