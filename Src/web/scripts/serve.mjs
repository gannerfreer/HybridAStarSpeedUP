#!/usr/bin/env node
/**
 * W5 本地服务（components.json web 条目 `npm run serve`）：
 *   - 静态服务 dist/（默认 http://127.0.0.1:4173/）；
 *   - 反向代理 /api/*      → planner_map_service（默认 http://127.0.0.1:8765）；
 *   - 反向代理 /plan-api/* → planner_plan_service（默认 http://127.0.0.1:8766），
 *     含 /plan-api/plan/events WebSocket 升级透传；
 *   - GET /health 返回本服务状态（便于脚本探活）。
 *
 * 仅使用 Node 标准库；正式证据仍经 tools/task_runner.py。
 *
 * 用法：
 *   node scripts/serve.mjs --port 4173 --run-dir <run_dir>
 *     [--api-target http://127.0.0.1:8765] [--plan-api-target http://127.0.0.1:8766]
 */

import http from 'node:http';
import fs from 'node:fs';
import net from 'node:net';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const distDir = path.resolve(here, '..', 'dist');

function parseArgs(argv) {
  const args = { port: 4173, runDir: '', apiTarget: 'http://127.0.0.1:8765', planApiTarget: 'http://127.0.0.1:8766' };
  for (let index = 0; index < argv.length; index += 1) {
    const key = argv[index];
    const value = argv[index + 1];
    if (key === '--port') args.port = Number(value);
    else if (key === '--run-dir') args.runDir = value ?? '';
    else if (key === '--api-target') args.apiTarget = value ?? args.apiTarget;
    else if (key === '--plan-api-target') args.planApiTarget = value ?? args.planApiTarget;
    if (key.startsWith('--')) index += 1;
  }
  return args;
}

const args = parseArgs(process.argv.slice(2));

const CONTENT_TYPES = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.svg': 'image/svg+xml',
  '.png': 'image/png',
  '.jpg': 'image/jpeg',
  '.ico': 'image/x-icon',
  '.map': 'application/json; charset=utf-8',
  '.txt': 'text/plain; charset=utf-8',
};

function targetFor(pathname) {
  if (pathname.startsWith('/plan-api/')) {
    return { target: args.planApiTarget, prefix: '/plan-api', label: 'plan-api' };
  }
  if (pathname.startsWith('/api/')) {
    return { target: args.apiTarget, prefix: '/api', label: 'api' };
  }
  return null;
}

function proxyHttp(req, res, route) {
  const base = new URL(route.target);
  const options = {
    hostname: base.hostname,
    port: base.port || 80,
    path: req.url.slice(route.prefix.length) || '/',
    method: req.method,
    headers: { ...req.headers, host: base.host },
  };
  const upstream = http.request(options, (upstreamRes) => {
    res.writeHead(upstreamRes.statusCode ?? 502, upstreamRes.headers);
    upstreamRes.pipe(res);
  });
  upstream.on('error', (error) => {
    res.writeHead(502, { 'Content-Type': 'application/json; charset=utf-8' });
    res.end(JSON.stringify({ status: 'proxy_error', reason: `${route.label}: ${error.message}` }));
  });
  req.pipe(upstream);
}

function serveStatic(req, res, pathname) {
  const safe = pathname.replace(/\/+$/, '') || '/';
  let file = path.join(distDir, safe === '/' ? 'index.html' : safe);
  if (!file.startsWith(distDir)) {
    res.writeHead(403).end('forbidden');
    return;
  }
  if (!fs.existsSync(file) || fs.statSync(file).isDirectory()) {
    // SPA 回退（?map= 等查询参数不影响静态路径）
    const fallback = path.join(distDir, 'index.html');
    if (fs.existsSync(fallback)) file = fallback;
    else {
      res.writeHead(404).end('not found');
      return;
    }
  }
  const type = CONTENT_TYPES[path.extname(file).toLowerCase()] ?? 'application/octet-stream';
  res.writeHead(200, { 'Content-Type': type });
  fs.createReadStream(file).pipe(res);
}

const server = http.createServer((req, res) => {
  const pathname = new URL(req.url ?? '/', 'http://127.0.0.1').pathname;
  if (pathname === '/health') {
    res.writeHead(200, { 'Content-Type': 'application/json; charset=utf-8' });
    res.end(JSON.stringify({ status: 'ok', service: 'web-serve', api_target: args.apiTarget, plan_api_target: args.planApiTarget }));
    return;
  }
  const route = targetFor(pathname);
  if (route) {
    proxyHttp(req, res, route);
    return;
  }
  serveStatic(req, res, pathname);
});

// WebSocket 升级透传（/plan-api/plan/events → 规划服务）
server.on('upgrade', (req, socket, head) => {
  const pathname = new URL(req.url ?? '/', 'http://127.0.0.1').pathname;
  const route = targetFor(pathname);
  if (!route || route.label !== 'plan-api') {
    socket.destroy();
    return;
  }
  const base = new URL(route.target);
  const upstream = net.connect(Number(base.port || 80), base.hostname, () => {
    const headers = [
      `GET ${req.url.slice(route.prefix.length)} HTTP/1.1`,
      `Host: ${base.host}`,
      ...Object.entries(req.headers)
        .filter(([key]) => key !== 'host')
        .map(([key, value]) => `${key}: ${value}`),
      '',
      '',
    ].join('\r\n');
    upstream.write(headers);
    if (head && head.length > 0) upstream.write(head);
    upstream.pipe(socket);
    socket.pipe(upstream);
  });
  upstream.on('error', () => socket.destroy());
  socket.on('error', () => upstream.destroy());
});

server.listen(args.port, '127.0.0.1', () => {
  console.log(`[web-serve] http://127.0.0.1:${args.port}/ (dist=${distDir})`);
  console.log(`[web-serve] /api → ${args.apiTarget}；/plan-api → ${args.planApiTarget}（含 WS /plan/events）`);
  if (args.runDir) console.log(`[web-serve] run_dir=${args.runDir}`);
});
