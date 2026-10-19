# E0 工具与运行基础设施（tools/）

> 状态：本目录代码为 E0 交付物（A 组：Python 工具与自身测试）。
> **截至本文件写入时，尚未通过统一入口实际运行任何编译、测试或基准**；
> 验收证据由后续受监控运行生成并记录到 `Docs/reports/`。
> 本文件描述的是工具的使用方式，不代表仓库已具备运行结果。

执行依据：`Docs/component_development_plan.md` §5.1、§5.3、§7、§8、§9。
所有工具兼容 CPython 3.8～3.12，只使用标准库，平台为 Linux（`/proc`、`fcntl.flock`、POSIX 信号）。

## 1. 文件清单

| 文件 | 职责 |
| --- | --- |
| `task_runner.py` | 统一任务入口：`build` / `test` / `benchmark` / `run` / `exec` / `fixtures` |
| `resource_monitor.py` | 2 秒资源采样、90% 阈值处置、心跳、派发状态、启动恢复 |
| `report_results.py` | 由运行目录生成 Markdown / CSV / status.json（只读运行目录） |
| `sample_report.py` | E0-6 固定样例报告流程（合成样例，全部标注“样例 / 非实验结果”） |
| `runtime.py` | 共享路径、原子 JSON 写入、进程身份与进程组终止、`/proc` 读取 |
| `registry.py` | 活动登记、worker 名额、编译锁、心跳/派发状态/停止原因读写 |
| `rundir.py` | 运行目录生命周期、metrics/resources 写入、封存与清单校验、中断恢复 |
| `fixtures.py` | `tests/fixtures/<version>/` 的 freeze/verify |
| `parking_fixtures.py` | v8 停车夹具生成器（v7 全量复制 + 论文 115 场景集；`generate`/`check`，标准库） |
| `parkbench_fixtures.py` | v9 ParkBench 夹具生成器（v8 全量复制 + 受限停车 51 场景；线段清洗→薄凸四边形；`generate`/`check`，标准库） |
| `parking_compare_plots.py` | 停车基准逐 case 对比图（Plotly HTML；实验辅助，依赖 plotly，见 §7.5） |
| `junit_runner.py` | 标准库 unittest 发现 + JUnit XML 生成（task_runner 内部调用） |
| `clock.py` | 真实时钟与可注入 FakeClock；UTC ISO 8601 毫秒格式 |
| `locking.py` | 基于 `fcntl.flock` 的共享/独占文件锁 |
| `components.json` | 组件注册表（构建、测试、服务、基准套件、夹具根） |
| `tests/test_*.py` | 自身测试（unittest；隔离临时目录、注入时钟/读数） |

公共逻辑拆分说明：`runtime/registry/rundir/fixtures` 为可复用模块，
`junit_runner` 为测试运行辅助，入口脚本只做编排。

### 各工具文件说明（分文件）

按文件拆分的逐工具说明（用法、输入输出、依赖与调用关系、证据约定）：

- [`docs/task_runner.md`](docs/task_runner.md) — 统一任务入口 build/test/benchmark/run/exec/fixtures 的编排、并发与退出码。
- [`docs/registry.md`](docs/registry.md) — 活动登记、worker 名额、编译锁与心跳/派发状态读写。
- [`docs/locking.md`](docs/locking.md) — 基于 `fcntl.flock` 的共享/独占文件锁。
- [`docs/runtime.md`](docs/runtime.md) — 共享路径、原子 JSON 写入、进程身份与进程组终止、`/proc` 读取。
- [`docs/rundir.md`](docs/rundir.md) — 运行目录生命周期、metrics/resources 写入、封存与清单校验、中断恢复。
- [`docs/clock.md`](docs/clock.md) — 真实时钟与可注入 FakeClock；UTC ISO 8601 毫秒格式。
- [`docs/resource_monitor.md`](docs/resource_monitor.md) — 2 秒采样、90% 阈值处置、心跳、派发状态与启动恢复。
- [`docs/report_results.md`](docs/report_results.md) — 由运行目录只读生成 Markdown/CSV/status.json 并校验产物清单。
- [`docs/sample_report.md`](docs/sample_report.md) — E0-6 固定样例报告流程（合成样例，标注“样例 / 非实验结果”）。
- [`docs/junit_runner.md`](docs/junit_runner.md) — 标准库 unittest 发现 + JUnit XML 生成（`test --component tools` 内部调用）。
- [`docs/fixtures.md`](docs/fixtures.md) — `tests/fixtures/<version>/` 的 freeze/verify（哈希清单）。
- [`docs/grid_fixtures.md`](docs/grid_fixtures.md) — v6 栅格夹具生成器（G1–G8、P1–P13、显示档；`--check` 复算）。

## 2. 先启动资源监控（派发前置）

```sh
python3 tools/resource_monitor.py            # 前台长驻；由监控 agent 看护
python3 tools/resource_monitor.py --once     # 冒烟自检：采样一轮后退出
```

- 心跳文件**定死在** `planner-runtime/monitor_heartbeat.json`（位于
  `git rev-parse --git-common-dir` 下，多个 worktree 共享）。调度器在心跳缺失、
  `status=stopped` 或超过 6 秒未更新时**拒绝派发**；运行中的任务检测到心跳失效
  会终止自己的子进程并记录 `monitor_lost`。
- 单实例保护：`planner-runtime/monitor.lock` 被占用时第二个监控进程退出（69）。
- 实时状态：`artifacts/live/status.json`（至多每 10 秒更新）与
  `artifacts/live/events.jsonl`（异常事件立即记录）；采样同时向活动运行目录追加
  `resources.csv`。
- E0 引导期可用 `--allow-unmonitored` 跳过心跳门禁做无负载自检；该标志会写入
  config/manifest，**正式实验禁止使用**。
- 单测注入的假时钟/假读数只存在于测试进程内；正式入口不暴露任何模拟开关，
  模拟配置无法作用于真实监控或正式任务。

## 3. 任务入口与并发限制

```sh
# 编译（全局唯一编译锁；配置 + 编译；--parallel 上限 4）
python3 tools/task_runner.py build --component common --profile release --parallel 4

# 测试（预占 worker 名额；ctest 或标准库 unittest；生成 junit.xml）
python3 tools/task_runner.py test --component common --workers 2
python3 tools/task_runner.py test --component tools --workers 1

# 正式基准（等待编译/测试/服务退出后登记独占窗口，监控继续）
python3 tools/task_runner.py benchmark --component common --suite v1

# 长驻服务（占名额、登记与回收）
python3 tools/task_runner.py run --component web --workers 1

# 重试：新 run ID，manifest.retry_of 记录来源（格式非法在派发前拒绝，65）
python3 tools/task_runner.py test --component rs --workers 1 \
  --retry-of 20261009T151800491894Z_tools_00c5bc64

# 一次性任务：占 worker 名额、监控门禁、§9 证据封存、退出码转发
python3 tools/task_runner.py exec --component adhoc --workers 1 -- \
  sha256sum dependencies.lock.json
# 碰撞 100k 类一次性验收（本波次不重跑 100k；命令自行写 {run_dir}/outputs）
python3 tools/task_runner.py exec --component collision --workers 1 -- \
  '{build_dir}/Src/collision/planner_collision_acceptance' \
  --suite collision-100k --seed 0 --output-dir '{run_dir}/outputs'

# Wave 3 搜索组件（graph_search/grid_search/sampling_search；注册表 test.workers=1）
python3 tools/task_runner.py build --component graph_search --parallel 4
python3 tools/task_runner.py test  --component graph_search --workers 1
python3 tools/task_runner.py build --component grid_search --parallel 4
python3 tools/task_runner.py test  --component grid_search --workers 1
python3 tools/task_runner.py build --component sampling_search --parallel 4
python3 tools/task_runner.py test  --component sampling_search --workers 1

# 搜索组件基准（X-B1 就绪，独占窗口运行）
python3 tools/task_runner.py benchmark --component graph_search --suite v1
python3 tools/task_runner.py benchmark --component grid_search --suite v1
python3 tools/task_runner.py benchmark --component grid_search --suite capacity
python3 tools/task_runner.py benchmark --component sampling_search --suite v1

# npm 组件（web）：install（首轮，node_modules 缺失时）→ build；test；W5 服务
python3 tools/task_runner.py build --component web
python3 tools/task_runner.py test --component web --workers 1
python3 tools/task_runner.py run --component web --workers 1

# 显示档基准（web，suite=grid_display；256²/512²/1024²，X-W1 首次实测）
python3 tools/task_runner.py benchmark --component web --suite grid_display

# 停车基准探索性套件（hybrid_astar，suite=v3；fixtures v8 的 115 个论文
# 停车场景 × 4 个 normalized 后端；注册命令 = 冷启动单次 + 30 s 预算 +
# 8 并行 worker + 路径导出，详见 §7.5）
python3 tools/task_runner.py benchmark --component hybrid_astar --suite v3

# ParkBench 探索性套件（hybrid_astar，suite=v4；fixtures v9 的 51 个受限
# 倒车入库场景 × 4 个后端，论文车辆 4.95×2.00 m；协议同 v3，详见 §7.6）
python3 tools/task_runner.py benchmark --component hybrid_astar --suite v4

# 停车对比图（实验辅助，需 plotly；逐 case 输出 HTML + index.html）
python3 tools/parking_compare_plots.py \
    --run-dir artifacts/runs/<run_id>/outputs \
    --output-dir artifacts/runs/<run_id>/outputs/plots/cases

# ParkBench 对比图（切换夹具/场景文件并关闭论文画布虚线）
python3 tools/parking_compare_plots.py --fixtures-dir tests/fixtures/v9 \
    --cases-file cases_bench_parkbench.json --no-paper-canvas \
    --run-dir artifacts/runs/<run_id>/outputs --output-dir <dir>

# 夹具清单
python3 tools/task_runner.py fixtures freeze --version v1
python3 tools/task_runner.py fixtures verify --version v1
```

- 登记、锁、心跳、停止原因位于 `planner-runtime/`：
  - `registry.json`：活动任务（登记任务 ID=run ID、类型、PID、进程组、进程启动标识、
    worker 名额、run 目录、启动时间/序号），`registry.lock` 原子保护；
  - `build.lock`：跨 worktree 唯一编译锁；
  - `dispatch_state.json`：监控发布的暂停/恢复状态（任务入口只读）；
  - `stop_reasons/<run_id>.json`：`resource_stopped` / `interrupted` 的原因；
  - `locks/`：登记锁、每运行证据写入锁、事件流锁。
- `test --component tools` 通过 `tools/junit_runner.py` 复现
  `python3 -m unittest discover -s tools/tests -p 'test_*.py'` 的发现逻辑，
  并用标准库 `xml.etree.ElementTree` 生成 JUnit XML。
- 并发上限：测试 / 服务 / `exec` 一次性任务 worker 合计 ≤ 4；第 5 个任务**排队等待**
  （`--queue-timeout` 秒后放弃，默认 3600；0=无限等待）。编译不占 worker 名额，
  但始终只允许一个。`ctest` 测试按 `--workers N` 并行（`ctest --parallel N`）；
  python（unittest）组件顺序执行，`--workers` 只用于预占名额，不会并行运行用例。
- 数值库默认环境：`OMP_NUM_THREADS=1`、`OPENBLAS_NUM_THREADS=1`、
  `MKL_NUM_THREADS=1`；用 `--thread-env KEY=VALUE` 覆盖，或
  `--no-default-thread-env` 完全不注入。
- 重试：`--retry-of <run_id>`（`build`/`test`/`benchmark`/`run`/`exec` 均可用）校验格式
  `YYYYMMDDTHHMMSSffffffZ_<component>_<8位小写十六进制>`，写入 `manifest.retry_of`
  （默认 null）；非法格式在派发前以退出码 65 拒绝，不创建运行目录、不占名额。
  例：`python3 tools/task_runner.py test --component rs --workers 1 --retry-of <失败runID>`
- `exec` 一次性任务：`exec --component <name> --workers N -- <command> [args...]`。
  与 `test`/`run` 同样占 worker 名额、走监控心跳门禁、按 §9 建运行目录并封存；
  退出码原样转发（信号死亡折算 128+signal）。`--component` 默认 `adhoc`，未注册
  组件也允许（仅作记录，`config.component_registered=false`）。命令中的
  `{repo_root}`/`{build_dir}`/`{run_dir}`/`{profile}` 占位符会被替换（shell 中需加
  引号）；命令输出写 `{run_dir}/outputs`。适用于碰撞 100k 验收等一次性命令。
- 输入哈希（tools-004）：`--input <path>`（可重复）在任务开始时计算 SHA256 并写入
  `manifest.input_sha256`：文件 ≤10 MiB 复制到 `inputs/<basename>`（键
  `inputs/<basename>`）；>10 MiB 记录 `external/<basename>` 外部引用（原路径见
  `config.inputs`）；目录打包为 `inputs/<name>.tar.gz`。输入缺失在派发中以 65
  拒绝并保留失败证据；**无 `--input` 时 `input_sha256` 保持空对象 `{}`**。
- `build --force-rebuild`（默认关闭，tools-006）：构建前刷新组件源目录内所有普通
  文件的 mtime（内容不变），消除“对象 mtime 比源码新”的陈旧对象竞态（corridor/map
  各遇到一次）；事件 `prepare_done` 记录 `touched_files`。cmake 源目录为 `Src/<id>`
  （条目含 `dir` 时以其为准），npm 组件用其 `dir`。
- 子进程上下文环境（组件基准/服务按契约读取）：`PLANNER_RUN_ID`、`PLANNER_RUN_DIR`、
  `PLANNER_TASK_ID`、`PLANNER_COMPONENT`、`PLANNER_PROFILE`、`PLANNER_BUILD_DIR`。
- 子进程在独立进程组中运行；终止前核对 `/proc/<pid>/stat` 第 22 字段（进程启动
  标识），防止 PID 复用误杀；先 SIGTERM，5 秒后 SIGKILL。

### 退出码

| 码 | 含义 |
| --- | --- |
| 0 | 成功 |
| 其他正整数 | 子进程真实退出码原样转发（信号死亡折算 128+signal） |
| 2 | 参数用法错误（argparse） |
| 65 | 配置/数据错误（组件、套件、输入、fixtures、报告校验失败） |
| 69 | 资源不可用（构建目录未配置、监控未运行、已有监控） |
| 70 | 工具内部错误 |
| 75 | 暂时无法派发（监控暂停、排队/锁超时、`monitor_lost`） |

## 4. 运行目录结构与封存（§9）

```
artifacts/runs/<run_id>/            # run_id: YYYYMMDDTHHMMSSffffffZ_<component>_<8位随机>
  manifest.json                     # 元数据、命令、版本、哈希、状态、退出码
  artifacts.sha256.json             # 排序清单数组 [{path,size_bytes,sha256}]（字节哈希=manifest.artifact_sha256）
  config.json                       # 全部生效参数（含默认值）
  inputs/                           # 复现输入（≤10 MiB 复制；>10 MiB 记 SHA256 外部引用；哈希见 manifest.input_sha256）
  source.patch                      # git diff --binary HEAD
  source_untracked.tar.gz           # 策展后的未跟踪源码/脚本/前端文件（存在时，见下）
  metrics.csv                       # 固定列，每次试验一行；任务级汇总行 case_id=task_summary
  resources.csv                     # 每 2 秒资源样本
  events.jsonl                      # 排队/开始/结束/终止等结构化事件
  stdout.log / stderr.log
  junit.xml                         # 测试任务；由标准库生成（--output-junit 可用时用 ctest）
  outputs/ plots/
  summary.md
```

- 封存顺序：释放登记 → 独占证据锁 → 写 summary → 计算清单 → 写清单并算哈希 →
  原子写最终 manifest（`artifact_sha256`=清单文件原始字节 SHA256）。此后运行目录
  不可变；报告生成器 `report_results.py` 只读运行目录，输出到外部目录。
- `source_untracked.tar.gz` 策展（tools-007）：收录源码/脚本/前端类型
  （`.py/.pyi/.c/.cc/.cpp/.cxx/.h/.hh/.hpp/.hxx/.cmake/.json/.ts/.tsx/.js/.mjs/.cjs/.css/.html/.sh/.toml/.yaml/.yml`
  与 `CMakeLists.txt`），单文件 ≤5 MiB；排除 `node_modules/`、`dist/`、`reports/`、
  `build/`、`artifacts/`、`.cache/`、`__pycache__/`、`coverage/` 等派生目录；
  **历史 run 不回溯**（规则自 tools-007 波次起对新 run 生效）。
- 重试用新 run ID 并记录 `retry_of`；中断目录保留，恢复流程按同一方式封存已有证据。

## 5. 报告生成

```sh
python3 tools/report_results.py --run-dir artifacts/runs/<run_id>
python3 tools/report_results.py --run-dir artifacts/runs/<run_id> --sample   # 样例模式
python3 tools/sample_report.py    # E0-6：重建固定样例并生成样例报告（非实验）
```

- 默认输出 `artifacts/reports/<run_id>/`：`<run_id>.md`（按
  `Docs/reports/report_template.md` 结构）、`<run_id>.csv`（成功/失败/中断等计数与
  性能摘要）、`status.json`（机器可读状态数据）。正式报告由集成负责人复制/链接到
  `Docs/reports/<component>/`。
- 每次生成都会校验 `artifacts.sha256.json`：逐项大小/内容、清单外额外文件、清单
  哈希与 `manifest.artifact_sha256`。检测到篡改仍生成报告但退出码 65。
- `--sample`（或 manifest 标记 `sample: true`）时所有输出显著标注
  **“样例 / 非实验结果”**，不得作为算法证据。

## 6. 故障恢复

| 情况 | 行为 |
| --- | --- |
| 监控心跳缺失/停止/超 6 秒 | 拒绝新派发；运行中的受管任务被调度器终止，记录 `monitor_lost` |
| CPU/内存 > 90% | 暂停派发；终止最新登记任务（SIGTERM→5 秒→SIGKILL），运行标记 `resource_stopped`，原因写入 `stop_reasons/`；连续 3 次 ≤90% 才恢复 |
| 无本仓库任务可停 | 只暂停派发并报警（`dispatch_paused_no_task`），不终止外部进程 |
| 监控重启 | 核对残留登记：未完成运行标记 `failed` / `interrupted` 并按流程封存；仍存活的遗留任务先按中断终止 |
| 任务失败 | 证据全部保留；修复后以新 run ID 重试，manifest 记录 `retry_of` |

## 7. components.json 与组件接入契约

### 7.1 组件接入要求

```json
"example": {
  "kind": "cmake",                                   // cmake 或 python
  "cmake": {"options": {"PLANNER_BUILD_EXAMPLE": "ON"}, "target": "planner_example"},
  "test": {"runner": "ctest", "label": "example"},   // 或 {"runner": "unittest", "start_dir": "...", "pattern": "test_*.py"}
  "service": {"command": ["{build_dir}/bin/server"], "workers": 1},   // run 子命令
  "benchmark_suites": {"v1": {"command": ["{build_dir}/bin/bench"]}} // benchmark 子命令
}
```

- 组件 CMake 选项必须与根 `CMakeLists.txt` 的 `PLANNER_BUILD_<ID>` 完全一致；
- 库目标命名 `planner_<id>`；
- **CTest 测试必须打上与组件 id 同名的 label**（`gtest_discover_tests(... PROPERTIES LABELS "<id>")`），
  否则 `test --component <id>` 的 `ctest -L <id>` 选不中任何用例；
- 基准二进制命名 `planner_<id>_bench`，必须接受 `--suite v1 --output-dir <dir>`；
- 组件可执行文件路径约定 `{build_dir}/Src/<id>/...`（与注册表条目一致）；
- 新增组件只需在 `components.json` 追加条目并按 7.2 产出指标，不改 `task_runner.py`。

npm 组件（如 `web`）条目：

```json
"web": {
  "kind": "npm",
  "dir": "Src/web",
  "install": ["npm", "install", "--no-audit", "--no-fund"],
  "install_ci": ["npm", "ci", "--no-audit", "--no-fund"],
  "build": ["npm", "run", "build"],
  "test": ["npm", "run", "test"],
  "test_junit": "reports/junit.xml",
  "service": {"command": ["npm", "run", "serve", "--", "--run-dir", "{run_dir}"],
              "workers": 1}
}
```

- `dir` 相对仓库根；npm 命令一律以组件目录为 cwd 运行；
- `build`：`node_modules` 缺失时先 install——存在 `package-lock.json` 且配置了
  `install_ci` 时用 `npm ci`，否则用 `install`；已存在 `node_modules` 时跳过 install；
  npm build 持有与 cmake 相同的全局编译锁，并默认占 1 个 worker 名额
  （`build_workers` 可覆盖；cmake build 仍不占名额、仅持锁）；
- `test`：尽力产出 JUnit XML——命令可直接写 `{run_dir}/junit.xml`，或配置
  `test_junit`（相对 `dir`）由入口在测试结束后收入运行目录；两者都没有时如实记录
  `junit_missing` 事件，不伪造 XML；
- `run`：使用 `service.command`（占 `workers` 名额）；脚本名（示例为 `serve`）由前端
  包实现，改名时同步更新条目；供 W5 联调；
- npm 命令占位符：`{repo_root}`、`{build_dir}`、`{run_dir}`、`{profile}`、`{dir}`；
  未知花括号按字面保留（不像 cmake 模板那样报错）。

当前已注册：`tools`、`common`、`grid`、`scene`、`rs`、`collision`、`bspline`、
`polynomial`、`corridor`、`hybrid_astar`、`validation`、`map`、`dftpav`、
`service`、`graph_search`、`grid_search`、`sampling_search`（`rs`/`collision`/
`bspline`/`polynomial`/`hybrid_astar`/`map` 含 `v1` 基准套件；`graph_search`
含 `v1`+`generate`、`grid_search` 含 `v1`+`capacity`、`sampling_search` 含 `v1`，
三套件 X-B1 正式运行与补跑均已完成，详见 X-B1 四份报告）；`web`（npm）已注册
install/build/test/service 模板与 `grid_display` 显示档基准套件（`benchmark
--component web --suite grid_display`），W5 服务脚本名以实际实现为准。
`grid`/`scene` 为 X-A1 新增的 cmake 组件（标签 `grid`/`scene`，无基准套件）；
`graph_search`/`grid_search`/`sampling_search` 为 X-S wiring 由 X-A1-19 占位登记
提升的 cmake 组件（标签同名；条目含 `test.workers=1` 记录字段，入口并发仍按
`test --workers` 取值，默认 1）。
占位符：`{repo_root}`、`{build_dir}`、`{run_dir}`、`{profile}`、`{dir}`；cmake/python/
benchmark 模板中的未知占位符直接报错。
顶层 `worker_cap`、`max_parallel`、`default_profile`、`default_suite` 为全局默认值。
X-A1-19 的 `planned_components` 占位登记已随 X-S wiring 删除：三个搜索组件进入
`components`，每个上游候选保留独立开关（未启用不下载、不链接）。

**依赖选项约定：** `cmake.options` 只列出该组件自身的构建开关及其所需的**同仓组件依赖开关**。任务入口只把该条目的键值作为 `-D` 传给一次根工程配置（未列出的组件保持各自默认值：`common`/`grid`/`scene` 默认 ON，其余组件默认 OFF；`scene` 还需要 `PLANNER_BUILD_MAP=ON` 才会实际加入，否则根项目跳过并给出说明），组件 CMake 据此通过 `add_subdirectory` 与目标链接消费依赖。同仓依赖必须显式列入并逐项维护，当前为：`map` → `PLANNER_BUILD_GRID=ON`、`PLANNER_BUILD_SCENE=ON`、`PLANNER_BUILD_COLLISION=ON`；`scene` → `PLANNER_BUILD_MAP=ON`、`PLANNER_BUILD_GRID=ON`、`PLANNER_BUILD_COLLISION=ON`；`hybrid_astar` → `PLANNER_BUILD_RS=ON`、`PLANNER_BUILD_COLLISION=ON`；`validation` → `PLANNER_BUILD_COLLISION=ON`、`PLANNER_BUILD_CORRIDOR=ON`；`dftpav` → `PLANNER_BUILD_CORRIDOR=ON`、`PLANNER_BUILD_VALIDATION=ON`、`PLANNER_BUILD_HYBRID_ASTAR=ON`、`PLANNER_BUILD_COLLISION=ON`；`service` → `PLANNER_BUILD_MAP=ON`、`PLANNER_BUILD_RS=ON`、`PLANNER_BUILD_COLLISION=ON`、`PLANNER_BUILD_HYBRID_ASTAR=ON`、`PLANNER_BUILD_CORRIDOR=ON`、`PLANNER_BUILD_VALIDATION=ON`、`PLANNER_BUILD_DFTPAV=ON`、`PLANNER_BUILD_SCENE=ON`、`PLANNER_BUILD_GRID=ON`；`graph_search`/`grid_search`/`sampling_search` → `PLANNER_BUILD_COMMON=ON`（同仓依赖仅 common；各上游适配器开关见 §7.4）。第三方依赖由根 `CMakeLists.txt` 按 `dependencies.lock.json`（URL + URL_HASH）获取，不在 `cmake.options` 中重复开关；Wave 3 组件的 FetchContent 条目（igraph、Klamp't）由组件在锁条目存在时优先消费（根项目传入 `PLANNER_LOCK_JSON`；条目缺失时以内嵌固定值兜底，见各组件 provenance），grid_search 全部来源为抽取副本、不 FetchContent。系统构建提供者（map 的 Boost ≥1.65 与 Eigen3 ≥3.3、dftpav 与 rs 对照适配器的系统 Eigen3、graph_search S-01 的系统 Boost 1.71、grid_search jps3d/libMultiRobot 的系统 Eigen3/Boost 头、sampling_search OMPL 构建的系统 Eigen3）没有锁条目，属已登记的例外（X-A1-12/X-A1-02/X-A1-07 与 X-S 来源记录）；锁条目 `eigen` 只服务 bspline/polynomial 的对照后端。

**服务约定：** `service.command` 指向组件交付的长驻二进制（当前：地图 `{build_dir}/Src/map/planner_map_service`（端口 8765）、规划服务 `{build_dir}/Src/service/planner_plan_service`（端口 8766）；CLI 契约均为 `--map <osm> --origin-lat <deg> --origin-lon <deg> --port <port> [--run-dir <dir>]`）；二进制由组件负责交付，缺失或启动失败时 `run` 以子进程非零退出码失败，属预期行为，不做静默回退。

### 7.2 基准输出契约（组件代理按此实现 `planner_<id>_bench`）

入口调用：`benchmark --component <id> --suite v1`，实际执行
`{build_dir}/Src/<id>/planner_<id>_bench --suite v1 --output-dir {run_dir}/outputs`，
在独占工作负载窗口内运行；子进程环境含 7.1/§3 的 `PLANNER_*` 上下文。

1. **退出码**：0 = 整套基准正常完成；非 0 = 任务失败（已产出的部分指标仍会保留）。
2. **指标文件**：必须写 `<output-dir>/benchmark_metrics.csv`（UTF-8 CSV）：
   - 表头使用 §9 固定列名（顺序任意，允许附加组件专有列；并入运行目录 `metrics.csv`
     时未知列被忽略）：`run_id, case_id, repeat_id, cache_mode, seed, status,
     failure_reason, preprocess_ms, wall_ms, cpu_ms, peak_rss_bytes, expanded_nodes,
     collision_checks, path_length_m, gear_changes, endpoint_position_error_m,
     endpoint_yaw_error_rad, valid`；
   - 每个 trial 一行；失败、超时、中断同样写入，**不得删除**；未测量留空（不用 0）；
   - `run_id` 取 `PLANNER_RUN_ID`；`repeat_id` 从 0 递增；`cache_mode` 为
     `cold`/`warm`；`seed` 记录实际种子；计时单位 ms、内存 bytes、长度 m；
   - 命令结束后 task_runner 将该文件逐行并入运行目录 `metrics.csv`，再追加一行
     `task_summary`；文件缺失不阻塞任务，但记录 `benchmark_metrics_missing` 事件。
3. **图表**：`<output-dir>/plots/*.svg` 与同名 `.csv` 数据（PNG 可选；颜色不作唯一标识）。
4. **其他证据**（轨迹、逐场景输出等）写在 `<output-dir>/` 内，不写运行目录其他位置，
   也不改写运行目录的 `metrics.csv`/`manifest.json`。
5. **证据采集**：stdout/stderr 由入口采集为 `stdout.log`/`stderr.log`；`resources.csv`
   由监控每 2 秒写入；`{run_dir}` 在结束后按 §9 封存，缺失/多余文件会被清单校验发现。

### 7.3 后续波次记录规范（tools-004）

- **重试**：任何失败、中断或资源停止后的重跑必须使用 `--retry-of <原 run_id>`
  （新 run ID，manifest.retry_of 记录来源）；不得覆盖或复用旧 run 目录。
- **哈希输入**：任务存在哈希输入（夹具、数据集、大文件）时，必须填充
  `manifest.input_sha256`：优先用 `--input <path>` 传入（键约定与空对象语义见 §3
  “输入哈希”，入口自动计算 SHA256），或引用 `fixtures verify` 生成的版本化清单
  哈希并在报告“复现信息”中给出。
- 历史 run 不追溯修改；本规范自 tools-004 波次起对新 run 生效。

### 7.4 可选构建目标与开关（X-A1）

原则：默认值保持既有行为（默认仍构建）；关闭开关时**不得下载或链接**对应依赖。
各组件 CMake 在未由根项目添加时提供相同默认值，独立构建路径保持一致。

| 开关 | 范围 | 默认 | 作用 |
| --- | --- | --- | --- |
| `PLANNER_BUILD_EXAMPLES` | 根（全局） | ON | 各组件命令行示例及其 CTest 自检用例；OFF 时不构建示例目标，相关用例一并跳过 |
| `PLANNER_BUILD_BENCHMARKS` | 根（全局） | ON | `planner_<id>_bench` 与基准冒烟 CTest 用例；OFF 时 `benchmark` 子命令因缺少二进制失败（预期，不静默回退） |
| `PLANNER_RS_WITH_OMPL` | rs | ON | OMPL 1.6.0 对照适配器；OFF 时不 FetchContent，测试/基准不链接对照库（X-A1-01） |
| `PLANNER_BSPLINE_WITH_EGO` | bspline | ON | GPL-3.0 EGO 对照库（仅测试/基准链接）；OFF 时不编译该库、不构建含 ego 后端的基准，测试套件保留主库用例（X-A1-03） |
| `PLANNER_POLYNOMIAL_WITH_MAV_COMPARE` | polynomial | ON | mav_trajectory_generation 对照库；OFF 时不下载 Eigen/glog/mtg，不构建对照库、对照测试用例与基准（X-A1-05） |
| `PLANNER_DFTPAV_WITH_E2E` | dftpav | ON | 示例/端到端测试的 Hybrid A* 链路；OFF 时库与单元测试不引入 `hybrid_astar`（X-A1-06） |
| `PLANNER_MAP_WITH_SERVICE` | map | ON | `planner_map_service`（W2 HTTP 服务）及其测试；OFF 时不获取/链接 httplib（X-A1-08） |
| `PLANNER_SERVICE_WITH_HTTP` | service | ON | `planner_plan_service`（HTTP 二进制）与集成测试；OFF 时 `planner_service` 库与单元测试不链接 httplib（X-A1-14） |
| `PLANNER_GRAPH_SEARCH_WITH_{BOOST,LMRP,JUSTINHJ,LEMON,IGRAPH}` | graph_search | ON | 五个图搜索适配器独立开关；OFF 不编译/不下载/不链接对应上游；IGRAPH 为 GPL-2.0-or-later 隔离组（仅 `planner_graph_search_igraph` 静态链接，见 provenance §6） |
| `PLANNER_GRID_SEARCH_WITH_{VSS2SN,SMAC2D,JPS3D,LIBMULTIROBOT,JUSTINHJ}` | grid_search | ON | 五个栅格搜索适配器独立开关；构建不联网（vendored/抽取副本）；JPS3D 缺系统 Eigen3 时打印警告并跳过该适配器（其余照常） |
| `PLANNER_SAMPLING_SEARCH_WITH_{OMPL,CH0P1K3,KLAMPT,VSS2SN,AIWINTER}` | sampling_search | OMPL/Ch0p1k3/Klamp't/vss2sn ON；AIWINTER OFF | 五个 RRT* 适配器独立开关；OFF 不下载/不链接；AIWINTER（GPL-3.0 隔离组）适配延期，显式置 ON 会报错（见 provenance §6） |

补充说明：

- bspline 主库的 Eigen Splines 后端属于公开 `BackendKind` 契约，Eigen 记录为
  **必要**依赖（X-A1-04）；可选的只有 EGO 对照库。
- `Src/web` 是 npm 组件，没有 `PLANNER_BUILD_WEB` 选项（X-A1-15）：用
  `build/test/run --component web`。
- `grid`（X-A1-17）与 `scene`（X-A1-18）为新增 cmake 组件；`scene` 依赖
  map/grid/collision，注册表条目已列出依赖开关。`planner_map_service` 在
  scene 可用时链接 `planner::scene`（X-A1-09，供 X-M1 的栅格/场景预览复用），
  不依赖 Hybrid A* / Dftpav。
- 最小依赖验证与回归证据记录在 `Docs/reports/`（X-A1 报告），原始日志在
  `artifacts/runs/<run_id>/`。

### 7.5 停车基准套件（hybrid_astar，suite v3，2026-10-17）

**夹具 v8**（`tests/fixtures/v8/`，`fixtures freeze/verify --version v8`）：

- 来源：IEEE TITS 2022 论文 115 个停车基准场景（ParkingMotionPlanningTITS21
  `ParkingBenchmarks/CaseNo_1..115.mat`）。转换链：`cpp/tools/export_cases.py`
  （停车仓库侧，scipy 一次性导出中立 JSON，含扇形三角剖分与凸性校验）→
  `tools/parking_fixtures.py`（本仓库，标准库；v8 = v7 全量复制 + 停车场景集）。
- 画布：论文画布 [-20,20]² 每边外扩 0.1 m（一格）为 402×402 格 @0.1 m
  （源数据 2/115 例起点包络擦边界 2.1/5.2 cm，详见 v8 README）；车辆
  `vehicles/parking_bench.json`（论文参数、安全边距 0）。
- 生成时几何自检 + 正式碰撞检查器起终点预检（`--precheck`，115/115 通过）
  写入夹具 README 与 cases 文件 `self_check`。

**基准**（`planner_hybrid_astar_bench --suite v3`；注册命令见
`tools/components.json`）：

- 场景：v8 全部 115 例按文件顺序；后端固定为 4 个统一栅格 normalized 后端
  （`ros_free` / `kth_normalized` / `zm0612_normalized` / `smac_normalized`），
  configure overlay 与 suite v2 相同；统一 `expected=success`（探索性对比，
  失败只记录）。
- 协议（注册命令）：`--cold-only --warmup-runs 0 --measured-runs 1
  --time-budget-s 30 --jobs 8 --vehicle parking_bench.json`；`--jobs N` 以
  （后端×场景）组合为并行单元，组合内保持预热/实测/热模式语义，输出为单
  进程集中写入（CSV/selection/路径互不交错）；单 trial CPU 口径为线程级。
- 输出：`benchmark_metrics.csv`（460 行）、`paths/<case_id>/<backend>.json`
  （plan_result JSON，成功与失败都导出）、`selection.json`（
  `parking_suite=true`、`experimental=true`、`protocol` 回显、不替换 v1 选型）；
  预检模式 `--precheck` 输出 `precheck.json` 后退出。
- **绘图工具** `tools/parking_compare_plots.py`（实验辅助，**依赖 plotly**，
  非标准库）：读 fixtures v8 + 运行目录输出，逐 case 生成
  `case_<i>.html`（障碍/画布虚线/起终点/4 条路径叠加 + 随轨迹虚拟车辆框 +
  一键隐藏/显示车辆框按钮 + 耗时条形图 + 指标表）与 `index.html`；目录模式
  写出单个共享 `plotly.min.js`，HTML 离线可看。

### 7.6 ParkBench 套件（hybrid_astar，suite v4，2026-10-17）

**夹具 v9**（`tests/fixtures/v9/`，`fixtures freeze/verify --version v9`）：

- 来源：ParkBench（`dqm5rtfg9b-collab/Constrained_Parking_Scenarios`，
  arXiv:2601.22545，**CC-BY-NC-4.0**，学术/非商用），51 个受限倒车入库场景；
  源 zip 按 SHA256 寻址归档（`artifacts/data/sha256/5435c67b…/`，commit
  `20b94635…`）。转换链：`tools/parkbench_fixtures.py`（标准库）——
  解析 `rear_in/*.json` → 离群段过滤（≥50 m）→ 目标/起点足迹框（外扩
  0.10 m）Liang-Barsky 裁剪（对齐官方 `utils.py` 语义并扩展到起点框；
  余量 0.10 m 覆盖 0.1 m 栅格量化）→ 线段转厚 0.05 m 薄凸四边形 → 每例
  稳健画布（0.1 m）。逐例清洗计数写入场景文件 notes 与 v9 README 审计表。
- 车辆：`vehicles/parkbench_vehicle.json`（ParkBench 论文 Appendix B Table 2：
  4.95×2.00 m、轴距 3.0 m、前悬 0.925 m、后悬 1.025 m、满舵 32°、边距 0）。
- 预检（`planner_hybrid_astar_bench --suite v4 --precheck`）51/51 通过，结论
  并入夹具 `self_check.collision_precheck`。

**基准**（`planner_hybrid_astar_bench --suite v4`；注册命令见
`tools/components.json`）：

- 场景：v9 全部 51 例按文件顺序；后端与统一栅格 overlay、协议
  （`--cold-only --warmup-runs 0 --measured-runs 1 --time-budget-s 30
  --jobs 8 --export-paths-dir`）与 v3 相同；`selection.json` 标记
  `parkbench_suite=true`、`experimental=true`，不替换 v1 选型。
- 正式运行（run `20261017T024554664760Z_hybrid_astar_45911dbb`，墙钟 85.0 s）：
  **204/204 成功且通过冻结校验**（0 失败/0 超时）；P50/P95（ms）：`smac_normalized`
  849/6937、`kth_normalized` 982/9349、`ros_free` 1426/9872、`zm0612_normalized`
  1671/16823（单次样本）。报告：
  `Docs/reports/hybrid_astar/20261017T024554664760Z_hybrid_astar_45911dbb.md`。
- 对比图：`tools/parking_compare_plots.py --cases-file cases_bench_parkbench.json
  --no-paper-canvas`（不画论文画布虚线、坐标按夹具画布自适应；含随轨迹
  虚拟车辆框与一键隐藏/显示按钮）；图集
  `artifacts/figures/parkbench_compare_v4_20261017T024554Z_v2/`（首版见同级
  `parkbench_compare_v4_20261017T024554Z/`）。

## 8. 已知限制

- 平台锁定 Linux；Windows/macOS 不支持。
- 资源采样为 `/proc` 快照估计；任务树 RSS 按登记进程组聚合，峰值是采样估计值，
  不等价于内核精确计量。
- `benchmark` / `run` 目前没有已注册的组件命令（`tools`、`common` 的空套件会明确
  报错）；接入方式见第 7 节。
- 所有验证状态以 `Docs/reports/` 为准；本 README 不构成验收证据。
