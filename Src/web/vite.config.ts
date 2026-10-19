import { defineConfig } from 'vitest/config';
import react from '@vitejs/plugin-react';
import { mapFixturesPlugin } from './vite-plugin-map-fixtures';

// W1 地图显示工作台。构建/测试经统一入口执行：
//   python3 tools/task_runner.py build --component web
//   python3 tools/task_runner.py test  --component web
export default defineConfig({
  plugins: [react(), mapFixturesPlugin()],
  server: {
    host: '127.0.0.1',
    port: 5173,
    strictPort: true,
    // W2/W5：本地 C++ 服务代理。
    // /api/map/...      → http://127.0.0.1:8765/map/...（地图服务）
    // /plan-api/plan/... → http://127.0.0.1:8766/plan/...（规划服务）
    proxy: {
      '/api': {
        target: process.env.PLANNER_WEB_API_TARGET ?? 'http://127.0.0.1:8765',
        changeOrigin: false,
        rewrite: (path) => path.replace(/^\/api/, ''),
      },
      '/plan-api': {
        target: process.env.PLANNER_WEB_PLAN_API_TARGET ?? 'http://127.0.0.1:8766',
        changeOrigin: false,
        ws: true, // /plan-api/plan/events WebSocket 透传（真实服务推送）
        rewrite: (path) => path.replace(/^\/plan-api/, ''),
      },
    },
  },
  preview: {
    host: '127.0.0.1',
    port: 4173,
    strictPort: true,
  },
  build: {
    outDir: 'dist',
    assetsDir: 'assets',
    sourcemap: false,
  },
  test: {
    environment: 'node',
    include: ['tests/**/*.test.ts'],
    testTimeout: 20000,
  },
});
