import type { DrawCommand } from './render';

/**
 * 绘制命令 → SVG。用于渲染检查证据（无浏览器环境下可检视的静态产物），
 * 与 Canvas 执行共享同一命令序列，保证两条渲染路径一致。
 */

function formatNumber(value: number): string {
  return String(Math.round(value * 100) / 100);
}

function escapeXml(text: string): string {
  return text
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;');
}

export function drawCommandsToSvg(
  commands: readonly DrawCommand[],
  widthPx: number,
  heightPx: number,
  background: string,
): string {
  const lines: string[] = [];
  lines.push(
    `<svg xmlns="http://www.w3.org/2000/svg" width="${formatNumber(widthPx)}" height="${formatNumber(heightPx)}" viewBox="0 0 ${formatNumber(widthPx)} ${formatNumber(heightPx)}">`,
  );
  lines.push(`<rect width="100%" height="100%" fill="${background}"/>`);

  for (const command of commands) {
    if (command.op === 'polygon') {
      const path = command.rings
        .map((ring) => `M ${ring.map(([x, y]) => `${formatNumber(x)} ${formatNumber(y)}`).join(' L ')} Z`)
        .join(' ');
      const stroke =
        command.stroke !== undefined
          ? ` stroke="${command.stroke}" stroke-width="${formatNumber(command.lineWidth ?? 1)}"`
          : '';
      lines.push(`<path d="${path}" fill="${command.fill}" fill-rule="${command.fillRule}"${stroke}/>`);
    } else if (command.op === 'polyline') {
      const points = command.points.map(([x, y]) => `${formatNumber(x)},${formatNumber(y)}`).join(' ');
      const dash = command.dash ? ` stroke-dasharray="${command.dash.join(' ')}"` : '';
      lines.push(
        `<polyline points="${points}" fill="none" stroke="${command.stroke}" stroke-width="${formatNumber(command.lineWidth)}"${dash}/>`,
      );
    } else if (command.op === 'text') {
      const anchor = command.align === 'left' ? 'start' : command.align === 'right' ? 'end' : 'middle';
      lines.push(
        `<text x="${formatNumber(command.x)}" y="${formatNumber(command.y)}" font-size="${formatNumber(command.fontPx)}" ` +
          `fill="${command.fill}" text-anchor="${anchor}" dominant-baseline="middle">${escapeXml(command.text)}</text>`,
      );
    } else {
      lines.push(
        `<circle cx="${formatNumber(command.cx)}" cy="${formatNumber(command.cy)}" r="${formatNumber(command.radius)}" fill="${command.fill}"/>`,
      );
    }
  }

  lines.push('</svg>');
  return lines.join('\n') + '\n';
}
