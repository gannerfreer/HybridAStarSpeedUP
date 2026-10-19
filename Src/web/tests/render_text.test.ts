import { describe, expect, it } from 'vitest';
import { executeDrawCommands, type Canvas2DLike } from '../src/core/canvas';
import { buildPlaybackCommands } from '../src/core/playback/commands';
import { buildPlaybackOverlay } from '../src/core/playback/overlay';
import { DEFAULT_PLAYBACK_LAYERS, type PlaybackSampleData } from '../src/core/playback/types';
import { commandsHaveFiniteCoordinates, summarizeDrawCommands, type TextCommand } from '../src/core/render';
import { drawCommandsToSvg } from '../src/core/svg';
import { fitBounds } from '../src/core/viewport';
import { loadBundleFromDisk } from './playback_fixtures';

class TextRecordingContext implements Canvas2DLike {
  ops: string[] = [];
  fillStyle: string | CanvasGradient | CanvasPattern = '';
  strokeStyle: string | CanvasGradient | CanvasPattern = '';
  lineWidth = 1;
  font = '';
  textAlign: CanvasTextAlign = 'start';
  textBaseline: CanvasTextBaseline = 'alphabetic';
  clearRect(): void {}
  fillRect(): void {}
  beginPath(): void {}
  moveTo(): void {}
  lineTo(): void {}
  closePath(): void {}
  arc(): void {}
  fill(): void {}
  stroke(): void {}
  setLineDash(): void {}
  fillText(text: string, x: number, y: number): void {
    this.ops.push(`fillText(${text}@${x.toFixed(1)},${y.toFixed(1)}) align=${this.textAlign} font=${this.font}`);
  }
}

const bundle = await loadBundleFromDisk();
const viewport = fitBounds({ min_x_m: 0, min_y_m: 0, max_x_m: 50, max_y_m: 30 }, 800, 500, 20);

describe('走廊 s 区间文字标注（web-002）', () => {
  const sample = bundle.samples.find((entry) => entry.sample.id === 'corridor_hybrid_reverse');
  if (!sample) throw new Error('missing corridor sample');
  const overlay = buildPlaybackOverlay(sample, { ...DEFAULT_PLAYBACK_LAYERS }, 0);
  const commands = buildPlaybackCommands(overlay, viewport);

  it('每个走廊多边形生成一条 s 区间文字命令', () => {
    const texts = commands.filter((command): command is TextCommand => command.op === 'text');
    expect(texts).toHaveLength(3);
    expect(texts.map((command) => command.text)).toEqual(['s 0–3 m', 's 3–6 m', 's 6–9 m']);
    for (const command of texts) {
      expect(command.layer).toBe('corridor');
      expect(Number.isFinite(command.x) && Number.isFinite(command.y)).toBe(true);
    }
    const summary = summarizeDrawCommands(commands);
    expect(summary.by_op.text).toBe(3);
    expect(summary.by_layer.corridor).toBe(6); // 3 多边形 + 3 标注
    expect(commandsHaveFiniteCoordinates(commands)).toBe(true);
  });

  it('Canvas 执行器渲染文字（字体/对齐）', () => {
    const ctx = new TextRecordingContext();
    const text: TextCommand = { op: 'text', layer: 'corridor', x: 10, y: 20, text: 's 0–3 m', fill: '#fff', fontPx: 11, align: 'center' };
    executeDrawCommands(ctx, [text]);
    expect(ctx.ops).toHaveLength(1);
    expect(ctx.ops[0]).toContain('fillText(s 0–3 m@10.0,20.0)');
    expect(ctx.ops[0]).toContain('align=center');
    expect(ctx.ops[0]).toContain('font=11px sans-serif');
  });

  it('SVG 导出包含文字与转义', () => {
    const svg = drawCommandsToSvg(
      [
        { op: 'text', layer: 'corridor', x: 1, y: 2, text: 's 0–3 m <&>', fill: '#fff', fontPx: 11 },
      ],
      100,
      50,
      '#000',
    );
    expect(svg).toContain('<text');
    expect(svg).toContain('text-anchor="middle"');
    expect(svg).toContain('s 0–3 m &lt;&amp;&gt;');
  });

  it('s 值格式：整数不带小数、非整数一位小数', () => {
    const corridorSample: PlaybackSampleData = {
      kind: 'corridor',
      sample: {
        id: 'c',
        kind: 'corridor',
        type: 'corridor',
        schema_version: 1,
        map_id: 'm',
        map_version: 'v1',
        file: '',
        sha256: '',
        expected_display: '',
        synthetic: true,
      },
      corridor: {
        id: 'c',
        map_id: 'm',
        map_version: 'v1',
        synthetic: true,
        polygons: [
          { s_start_m: 0.5, s_end_m: 2.25, vertices: [{ x_m: 0, y_m: 0 }, { x_m: 1, y_m: 0 }, { x_m: 1, y_m: 1 }] },
        ],
      },
    };
    const overlayWithFraction = buildPlaybackOverlay(corridorSample, { ...DEFAULT_PLAYBACK_LAYERS }, 0);
    const texts = buildPlaybackCommands(overlayWithFraction, viewport).filter(
      (command): command is TextCommand => command.op === 'text',
    );
    expect(texts[0].text).toBe('s 0.5–2.3 m');
  });
});
