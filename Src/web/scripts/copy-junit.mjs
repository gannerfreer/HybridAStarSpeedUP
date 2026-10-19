// 将 vitest 的 JUnit 输出（reports/junit.xml）同时复制到入口运行目录：
//   $PLANNER_RUN_DIR/junit.xml（存在时）
// 这样无论 components.json 是否配置 `test_junit`，统一入口 `test --component web`
// 的运行目录都能收集到 JUnit XML（task_runner 对已存在的 junit.xml 直接保留）。
// 仅标准库；未设置 PLANNER_RUN_DIR 时静默跳过（本地直连 npm 场景）。
import { copyFileSync, existsSync, mkdirSync, statSync } from 'node:fs';
import { join } from 'node:path';

const source = new URL('../reports/junit.xml', import.meta.url);
const runDir = process.env.PLANNER_RUN_DIR;

if (!existsSync(source)) {
  console.error('[junit] reports/junit.xml 不存在（vitest junit reporter 未产出？）');
  process.exitCode = 1;
} else if (!runDir) {
  console.log('[junit] PLANNER_RUN_DIR 未设置，跳过运行目录复制');
} else {
  mkdirSync(runDir, { recursive: true });
  const target = join(runDir, 'junit.xml');
  copyFileSync(source, target);
  const bytes = statSync(target).size;
  console.log(`[junit] copied reports/junit.xml -> ${target} (${bytes} B)`);
}
