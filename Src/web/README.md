# Src/web — 浏览器显示与交互（W1 地图显示 + W2 地图查询 + W3 场景编辑 + W4 结果回放 + W5 规划服务联调 + X-W1 栅格夹具渲染 + X-W2 在线栅格与规划快照）

> 状态：W1–W5 已交付（W5 服务与联调为本波次），X-W1 栅格夹具渲染与显示档首次实测已交付，
> X-W2 在线栅格（可行驶域/场景预览）与规划实际场景快照联调为本波次。
> 构建/测试/集成冒烟经统一入口执行；
> 本文件描述的直连 npm 命令仅供本地开发便利，正式运行与证据一律经 `tools/task_runner.py`。

执行依据：`Docs/component_development_plan.md` §2.2（W1–W5）、§6.1（JSON 契约）、§9（证据保存）。
数据字段语义：`Src/map/examples/data/README.md`（M4 快照）、`tests/fixtures/v3/README.md`
（车辆/路径/回放夹具）、`Src/map/src/service_main.cpp`（地图服务接口）、
`Src/service/`（规划服务 `planner_plan_service`，端口 8766）。

## 1. 功能范围

**W1 地图显示**

- 加载 `map_snapshot` JSON（`schema_version: 1`）：解析并校验
  `coordinate_system` / `bounds_m` / `counts` / `lanelets{left,right,centerline}` / `areas{outer,inner}`；
  对象 ID 按字符串保精度；任何被消费数值必须有限（拒绝 NaN/Infinity）。
- 世界坐标（米，x 东 / y 北）→ 屏幕坐标（CSS 像素，y 下）变换，高 DPI 通过一次性
  `setTransform(dpr,…)` 适配；往返误差远小于 1 像素（渲染检查报告实测值）。
- 渲染：区域（外环 + 内环 evenodd 镂空）、道路左右边界、中心线（虚线）；分层开关。
- 交互：滚轮以光标为中心缩放（0.02–400 px/m）、拖拽平移、重置视图；比例尺；坐标读数。
- 数据加载：默认加载仓库内官方示例快照；支持拖拽 / 文件选择任意快照 JSON；`?map=<url>` 覆盖。

**W2 地图查询交互**（本地 C++ 服务 `planner_map_service`，默认端口 8765）

- 对象选择：单击画布 → 世界坐标 → `/map/query/point`（服务在线）或本地命中回退
  （多边形 + 中心线容差）→ 选中 lanelet/area 高亮。
- ID/属性面板：`/map/lanelet/{id}`（或 `/map/area/{id}`）显示属性与几何统计（服务/本地来源徽标）。
- 起终位姿：模式按钮点击设位姿（航向取命中 lanelet 中心线切向）、面板数值编辑、
  拖动圆点调整航向；画布显示位姿箭头。
- 拓扑：选中 lanelet 的 `following` / `preceding` / `adjacent?side=left|right` 查询与高亮；
  服务离线时 following/preceding 提供本地近似（明确标注）。
- 路由：起终 lanelet 输入（设位姿自动填充）→ `/map/route` → 序列/长度展示与路由高亮。
- 局部窗口：拖拽矩形 → `/map/query/window` 裁剪结果（路径/点/语义引用）叠加显示 +
  坐标变换说明（窗口局部系原点/yaw；裁剪几何为地图坐标系）。

**W3 场景编辑**（覆盖层，不改写任何地图/夹具文件）

- 测试障碍：画布"障碍"模式拖拽新建轴对齐矩形、拖动移动、单击选中后在面板编辑坐标/尺寸/删除；
  也支持"视口中心"快捷添加与多边形障碍的渲染/删除（多边形编辑经 JSON 场景导入）。
  渲染为红橙色填充/描边，与地图要素视觉区分。
- 车辆模型与足迹：加载 `tests/fixtures/v3/vehicles/default.json`（公共 VehicleModel，
  §6.1 字段）；面板显示参数与派生量（最小转弯半径 = 轴距/tan 转角、最大曲率）；
  起终位姿处绘制整车足迹（车体实线 + 安全裕量虚线），可选光标处足迹。
- 场景保存/重载：版本化 `web_scene` JSON（见 §5.1），导出下载/导入上传 +
  localStorage 自动保存/恢复；重载后障碍集合、位姿与算法参数逐字段一致；
  `map_id`/`map_version` 不匹配时显式警告并拒绝套用（旧场景不覆盖当前地图）。
- 算法参数：Hybrid 时间/节点预算、走廊启用/裕量占位字段，随场景保存/重载（W5 消费）。

**W4 算法结果展示**（固定样例回放，无需算法服务）

- 加载 `tests/fixtures/v4/playback/index.json` 与全部样例：**加载前按索引 SHA256 逐文件校验**，
  不匹配/读取失败/解析失败的样例被拒绝并列出原因；地图版本不一致的样例（stale 演示项除外）拒绝加载。
- 小型合成地图（`map_snapshot_minimal`）作为回放底图；进入回放模式后服务查询自动回退本地
  （服务地图与回放地图版本不一致时不使用服务结果）。
- 逐类渲染：RS/Hybrid 路径（前进/倒车分段着色 + 换向点 cusp 标记 + 起终点）、
  B-spline（de Boor 域内 200 点采样 + 控制多边形）、多项式（段局部时间 τ 秒采样 + 段边界）、
  走廊（半透明凸多边形）、Dftpav 轨迹（按速度符号着色 + 点列 + 当前位姿指示）、
  碰撞样例（足迹 + 障碍 + 重叠区高亮）、失败样例（仅状态与原因，不绘制路径）、
  过期样例（警告 + 不产生几何，不覆盖当前结果）。
- 统计面板：按 `PlanResult.statistics` 展示（已知字段含单位，缺失显示 N/A，布尔 是/否，未知键原样追加）。
- 时间回放：播放/暂停/进度拖拽；当前位姿指示；v(t)/κ(t)/a(t) 数值面板与两幅 SVG 小图随回放联动。
- 结果图层开关：路径/曲线/走廊/轨迹/碰撞点。
- 界面明确标注"合成显示样例（synthetic），非算法正确性证据"（面板横幅 + 状态栏）。

**W5 规划服务联调**（`planner_plan_service`，默认端口 8766；HTTP/JSON + WebSocket 事件）

- 任务提交：以当前场景（W3 `web_scene`）构造 `/plan/submit` 请求（地图版本、起终位姿、
  障碍、`algorithm_params{hybrid_backend,time_budget_s,node_budget,corridor.clearance_m}`）；
  面板显示 request_id / job_id / job_seq / 状态（queued/running/succeeded/failed/cancelled）与阶段
  （map/hybrid/corridor/optimize/validate/done）。
- 状态与结果：轮询 `/plan/status/{job_id}`（800 ms）+ `/plan/events` WebSocket 推送
  （`job_update`/`job_result`，均含单调 `job_seq`）；终态后经 HTTP 拉取 `/plan/result/{job_id}`
  （事件只带状态，几何在 HTTP 结果中），结果复用 W4 渲染管线（Hybrid 段/走廊/轨迹/校验/统计）。
- 陈旧防护（验收关键）：客户端维护最新 `job_seq` 与"当前代际活动任务"集合；
  `job_seq` 落后的事件/提交响应一律丢弃（计入陈旧忽略计数）；旧 job 的迟到状态/结果不得覆盖
  新提交；新提交清空旧结果并开启新代际；地图版本与请求不一致时阻止提交（服务端 409
  `stale_map` 与客户端前置检查双保险，均显式提示）。
- 取消：`/plan/cancel` 取消活动任务（queued 立即取消；running 在阶段边界协作取消），
  结果状态显示"已取消"。
- 算法比较：同一场景对两个 `hybrid_backend`（默认 `kth_normalized` / `ros_free`）提交，
  并排统计表（状态/规划耗时/求解耗时/总长度/换向次数/展开节点/碰撞检查/轨迹校验；
  真实服务统计经 `statistics.hybrid`/`timings_s` 展平与别名派生）。
- 结果回放：规划结果轨迹支持播放/暂停/拖拽与 v(t)/κ(t) 小图（与 W4 控件一致）；
  结果图层开关与 W4 共用；结果标注地图版本与请求 ID；结果地图与当前底图不一致时仅展示面板并警告。
- 走廊 s 区间标注（web-002）：走廊多边形质心绘制 `s <start>–<end> m` 文字（Canvas/SVG 双执行器）。

**X-W1 栅格夹具渲染（离线显示档，协议 v1）**

- 夹具加载：`tests/fixtures/v6/grids/`（由 Vite 插件从仓库读取，不复制数据），支持
  `grid_view`（整数数组）与 `grid_snapshot`（`base64_u8`）两种文档；解析拒绝规则对齐协议
  §2.4（长度/取值/规范 base64/窗口/分辨率/降级一致性/yaw 归一化）。
- 渲染：栅格纹理（1 像素 = 1 单元，三态固定颜色表）经仿射变换映射到屏幕；旋转原点
  （G1/G2）按 `origin` 位姿换算；纹理仅在图层变化时重建，缩放/平移复用缓存；矢量边界
  绘制于栅格之上；选中单元轮廓高亮。
- 控件：图层开关、透明度滑杆、夹具选择、重新加载/定位到栅格/清空；图例（空闲/占据/未知）；
  拾取模式下单击画布显示选中单元的行列、状态与中心世界坐标（出界不静默夹取）。
- 可见状态：加载中/未加载（空数据）/加载失败（含 404 与解析错误）均在面板显示；
  陈旧防护沿用协议 §6：`latest_seq` 单调，旧响应丢弃并计数（`GridLoadTracker`）。
- 界面明确标注"合成显示夹具（fixture），非算法正确性证据"。
- 显示性能基准（benchmark_tiers §4.1）：`?gridBench=1` 暴露 `window.__plannerGridBench`
  （`loadFixture`/`hotRedraw`/`interact`/`runPipeline`），仅基准脚本使用。

**X-W2 在线栅格与规划快照联调（协议 v1；`Docs/expansion/grid_protocol.md`）**

- 在线数据源（数据源切换）：`POST /map/grid/driveable`（可行驶域）与
  `POST /map/scene/preview`（规划场景预览，携带当前障碍；可选 `?gridSource=driveable|planning_scene`）；
  请求携带单调 `request_seq`，每来源独立 `latest_seq`；晚到响应不覆盖新请求（`stale_ignored` 计数），
  回显 seq 不符/未发起视为协议异常；`map_id`/`map_version` 与当前底图不一致一律丢弃
  （`map_mismatch_ignored`，与 `/plan/submit` 的 `stale_map` 双保险）。
- 版本一致性（协议 §3.5）：`/health` 的 `grid_protocol_version` 主版本必须为 1，
  `scene_rules_version` 必须为 `scene_v1`；不匹配时禁用图层并显示可见状态（主版本不匹配禁用
  全部在线源；场景规则不匹配仅禁用预览）。地图服务与规划服务 `scene_rules_version` 不一致时
  面板警告，禁止把当前预览当作规划输入证据。
- 降级显示：服务 `degraded=true` 时显示**实际**分辨率与"已降级"标记（不得标注为原请求精度）；
  在线显示上限 ≤1024²（benchmark_tiers §4.1），超限按 2 的幂提高请求分辨率并显示
  "显示上限调整 ×N"。
- 复用 W1：三态图例、透明度、单元拾取/检查、定位到栅格、纹理缓存对在线图层同样有效；
  在线数据明确标注"来自服务"。
- 规划实际场景（`GET /plan/scene/{scene_id}`）：规划状态/结果携带 `scene_id` 后自动拉取该次规划
  实际使用的输入快照；独立面板"规划实际场景（历史，不覆盖当前预览）"展示 scene_id/scene_version/
  窗口/实际分辨率/三态计数，画布以品红叠加层绘制（与预览显式区分）；404（未知/已淘汰）与拉取
  失败均有可见状态；晚到旧 `scene_id` 响应丢弃（`PlanSceneSnapshotTracker`）。
- 夹具模式保留：离线/版本不匹配时回退 X-W1 夹具（数据源选择切换，无服务依赖）。

## 2. 目录结构

| 路径 | 职责 |
| --- | --- |
| `src/core/types.ts` | 快照 TypeScript 类型（对齐 M4 字段表） |
| `src/core/parse.ts` | 快照解析与校验（拒绝规则、JSON 路径错误信息） |
| `src/core/stats.ts` | 几何计数（与顶层 `counts` 交叉核对） |
| `src/core/transform.ts` | 世界 ↔ 屏幕变换、`boundsToScreenRect`、米/像素 |
| `src/core/viewport.ts` | `fitBounds` 自动取景、`zoomAt` 光标中心缩放、`panBy` 平移、缩放夹紧 |
| `src/core/layers.ts` | 图层开关状态 |
| `src/core/render.ts` | 场景/覆盖层 → 绘制命令（纯函数）；命令统计与有限性检查 |
| `src/core/canvas.ts` | Canvas 2D 命令执行器（`Canvas2DLike` 最小接口） |
| `src/core/svg.ts` | 绘制命令 → SVG（证据导出，与 Canvas 共享同一命令序列） |
| `src/core/scalebar.ts` | 比例尺整长度选择与标签 |
| `src/core/evidence.ts` | 渲染检查确定性报告（计数/包围盒/比例/命令统计） |
| `src/core/fixture.ts` | Node 侧夹具定位（v3 优先、M4 回退；不复制数据） |
| `src/core/hit.ts` | 本地命中测试（多边形 evenodd、中心线容差、ID 排序） |
| `src/core/pose.ts` | 位姿/航向工具（归一化、中心线切向、角度换算） |
| `src/core/topology.ts` | following/preceding 本地近似（服务离线回退） |
| `src/core/window.ts` | 窗口查询本地回退（包围盒过滤，标注近似） |
| `src/core/interaction.ts` | 画布交互模式（选择/设起点/设终点/窗口查询/障碍） |
| `src/core/query_state.ts` | W2 查询交互界面状态类型 |
| `src/core/scene/types.ts` | W3 场景 JSON 类型（web_scene v1；障碍字段与 v3 路径夹具一致） |
| `src/core/scene/parse.ts` | 场景解析/确定性序列化/地图版本校验 |
| `src/core/scene/obstacles.ts` | 障碍编辑逻辑（构造/移动/命中/自然 ID 排序） |
| `src/core/scene/vehicle.ts` | 车辆模型解析与足迹几何（车体/裕量/世界坐标旋转） |
| `src/core/scene/state.ts` | 场景组装（buildScene/isSceneEmpty/包围盒） |
| `src/core/scene/storage.ts` | localStorage 保存/恢复（注入式存储，Node 可测） |
| `src/core/playback/types.ts` | W4 回放类型（索引/样例/图层开关） |
| `src/core/playback/parse.ts` | 索引与各类型样例解析 + 小型回放地图适配 |
| `src/core/playback/hash.ts` | SHA256 校验编排（哈希/读取注入） |
| `src/core/playback/bspline.ts` | B 样条 de Boor 求值与域内采样 |
| `src/core/playback/polynomial.ts` | 多项式分段求值（τ 秒）/采样/剖面 |
| `src/core/playback/time.ts` | 轨迹时间映射、插值、换向时刻 |
| `src/core/playback/stats.ts` | 统计字段格式化（单位/N/A） |
| `src/core/playback/overlay.ts` | 样例 → 世界坐标覆盖层（失败/过期不产生几何） |
| `src/core/playback/commands.ts` | 回放覆盖层 → 绘制命令（方向/曲线/走廊/轨迹/碰撞） |
| `src/core/playback/load.ts` | 回放包加载（索引 → 校验 → 解析 → 地图） |
| `src/core/service/` | 服务契约：`types.ts`（规范化类型）、`parse.ts`（包络与载荷解析）、`client.ts`（fetch 客户端、超时/错误分类、API 基地址） |
| `src/core/plan/types.ts` | W5 规划服务类型（请求/状态/结果/事件；状态与阶段常量） |
| `src/core/plan/parse.ts` | 规划响应解析（submit/status/result/event；`hybrid.segments` 与统计展平/派生） |
| `src/core/plan/client.ts` | 规划服务客户端（submit/cancel/status/result + `/plan/events` WebSocket 订阅与重连） |
| `src/core/plan/tracker.ts` | 陈旧防护：`latest_seq` + 当前代际活动任务集合（提交/事件/轮询判定） |
| `src/core/plan/compare.ts` | 两后端并排比较表（状态/耗时/长度/换向/节点/碰撞/校验） |
| `src/core/plan/convert.ts` | 规划结果 → W4 回放管线（样例转换 + 走廊覆盖层） |
| `src/core/grid/types.ts` | 栅格夹具类型（`grid_view`/`grid_snapshot` 解析结果、三态、面板状态） |
| `src/core/grid/parse.ts` | 栅格文档解析与校验（协议 v1 拒绝规则、规范 base64 解码） |
| `src/core/grid/coords.ts` | 单元 ↔ 世界坐标换算（旋转原点）、三态计数、包围盒 |
| `src/core/grid/texture.ts` | 三态 RGBA 纹理生成与纹理缓存（仅图层变化重建） |
| `src/core/grid/render.ts` | 纹理仿射变换绘制、选中单元轮廓（Canvas 解耦） |
| `src/core/grid/load.ts` | 夹具目录解析、加载编排与陈旧跟踪（`GridLoadTracker`） |
| `src/core/grid/online.ts` | X-W2 在线栅格：`/health` 版本一致性、在线客户端（driveable/preview）、每来源 `GridSourceTracker`、显示上限与请求窗口 |
| `src/core/plan/scene_snapshot.ts` | X-W2 规划实际场景快照：`GET /plan/scene/{scene_id}` 校验、晚到响应跟踪、预览/规划场景显式比较 |
| `src/components/GridPanel.tsx` | 栅格面板（数据源切换/在线请求与版本状态/开关/透明度/图例/单元检查/失败与空状态） |
| `src/components/PlanScenePanel.tsx` | 规划实际场景面板（scene_id/窗口/分辨率/计数/叠加开关；404 与禁用可见状态） |
| `scripts/mock_grid_service.py` | X-W2 临时 mock 地图服务（乱序延迟/版本字段/降级；**非算法证据**，冒烟回退） |
| `scripts/grid_display_bench.py` | 显示档性能首次实测（CDP；传输/解析/纹理/帧/堆，门槛对照 §4.2） |
| `src/components/` | React 组件：`MapCanvas`、`Toolbar`、`StatusBar`、`InspectorPanel`、`ScenePanel`、`PlanPanel`、`PlanScenePanel`、`PlaybackPanel`、`ProfileChart`、`NumberField`、`GridPanel` |
| `src/App.tsx` | 状态编排：加载、取景、图层、选择/拓扑/位姿/路由/窗口、服务状态、W5 提交/取消/轮询/事件/陈旧防护、X-W2 在线栅格/规划快照 |
| `tests/` | vitest（Node 环境，无浏览器依赖） |
| `evidence/` | 渲染检查产物（`render_check.json`、`render_check.svg`，确定性） |
| `scripts/browser_cdp_check.py` | 真浏览器检查（CDP over pipe，标准库）：W1 交互 + W2 选择/拓扑/路由/窗口 + W3 场景 + W4 回放 + W5 规划（`--api-target` 反代 `/api`，`--plan-api-target` 反代 `/plan-api` 并直连 WS）+ X-W2 在线栅格/规划快照（`--xw2`；mock 阶段 `--grid-api-target` + `--xw2-mock-mode reorder|protocol|scene_rules`） |
| `scripts/browser_smoke.sh` | 简易冒烟回退（无头 Chrome `--screenshot` + DOM 校验；截图存在无头捕获尺寸偏差，见 §8） |
| `scripts/service_compare.py` | 集成冒烟：地图服务响应 vs v3 快照一致性对比（20 项检查） |
| `scripts/mock_plan_service.py` | W5 临时 mock 规划服务（确定性替身；真实二进制未构建时的冒烟回退，非算法证据；含 scene_id 与 `/plan/scene/{scene_id}`） |
| `scripts/integration_smoke.sh` | 启动地图/规划服务（真实优先，mock 回退）→ 服务对比 → CDP W1–W5 + X-W2 → stale_map 相位 → X-W2 mock 阶段（乱序/版本不匹配）→ 停止服务；阶段失败按 §5.4 继续并汇总 |
| `scripts/serve.mjs` | `npm run serve`：静态 dist + `/api`、`/plan-api` 代理与 WS 升级透传 |
| `scripts/copy-junit.mjs` | 把 vitest JUnit 输出复制到入口运行目录（`$PLANNER_RUN_DIR/junit.xml`） |
| `vite-plugin-map-fixtures.ts` | dev 中间件 / build 静态资源：把仓库快照供浏览器消费 |

**设计要点**：核心解析、变换、命中、绘制命令构建与 Canvas 完全解耦；Canvas 与 SVG 只是命令序列的
两个执行后端；服务交互集中在 `core/service/`，界面状态类型集中在 `core/query_state.ts`。
全部正确性测试（解析、变换、图层、取景、比例尺、命令执行、命中、位姿、窗口过滤、服务解析/客户端）
可在 Node 侧无浏览器运行。

## 3. 命令

统一入口（正式）：

```sh
python3 tools/task_runner.py build --component web            # 类型检查 + vite build
python3 tools/task_runner.py test  --component web --workers 1  # vitest + JUnit（reports/junit.xml → 运行目录）
python3 tools/task_runner.py run   --component web --workers 1  # 长驻开发服务（占 1 个 worker 名额，W5）

# W2 集成冒烟（启动 planner_map_service + 服务对比 + 真浏览器检查）：
python3 tools/task_runner.py exec --component map --workers 3 -- \
  bash Src/web/scripts/integration_smoke.sh

# 仅浏览器 W1/W2 检查（服务需已在 8765 运行）：
python3 tools/task_runner.py exec --component web --workers 2 -- \
  python3 Src/web/scripts/browser_cdp_check.py --api-target http://127.0.0.1:8765

# W5 集成冒烟（真实 planner_plan_service 已构建时自动启动；否则用临时 mock 并标注）：
#   W1–W5 浏览器检查 + 服务端 409 stale_map 独立相位
python3 tools/task_runner.py exec --component map --workers 3 -- \
  bash Src/web/scripts/integration_smoke.sh

# 仅浏览器 W5 检查（规划服务需已在 8766 运行；/plan/events 经 WS 直连）：
python3 tools/task_runner.py exec --component web --workers 2 -- \
  python3 Src/web/scripts/browser_cdp_check.py \
    --plan-api-target http://127.0.0.1:8766 --expect-ws

# X-W1 显示性能首次实测（benchmark_tiers §4.1；默认 30 次重复 + 11 s 交互窗口）：
python3 tools/task_runner.py exec --component web --workers 2 -- \
  python3 Src/web/scripts/grid_display_bench.py

# X-W2 在线栅格 + 规划实际场景（需 map 8765 与 plan 8766 已运行；可与集成冒烟一并执行）：
python3 tools/task_runner.py exec --component web --workers 2 -- \
  python3 Src/web/scripts/browser_cdp_check.py \
    --api-target http://127.0.0.1:8765 --plan-api-target http://127.0.0.1:8766 --xw2

# X-W2 mock 阶段（乱序响应/版本不匹配；由集成冒烟内部管理 mock_grid_service，非算法证据）：
python3 tools/task_runner.py exec --component map --workers 4 -- \
  bash Src/web/scripts/integration_smoke.sh

# 本地服务（web 组件 run 入口：静态 dist + /api、/plan-api 代理 + WS 透传）：
python3 tools/task_runner.py run --component web --workers 1
```

本地开发（非正式、不经入口，仅在依赖已安装时使用）：

```sh
cd Src/web
npm run dev      # http://127.0.0.1:5173/ ；/api → http://127.0.0.1:8765（vite 代理）
npm run build    # tsc --noEmit && vite build → dist/
npm run preview  # 预览 dist/（http://127.0.0.1:4173/）
npm test         # vitest run（JUnit → reports/junit.xml）
```

依赖版本在 `package.json` 精确锁定，`package-lock.json` 由首次 `npm ci/install` 生成后随仓库维护。

## 4. 数据源与夹具对接

浏览器默认加载 `/maps/mapping_example.snapshot.json`，由 Vite 插件从仓库读取（dev 中间件 /
build 静态资源），**不复制数据文件**。解析优先级：

1. `tests/fixtures/v3/map/mapping_example.snapshot.json`（v3 晋升夹具，权威源）；
2. `Src/map/examples/data/mapping_example.snapshot.json`（M4 样例回退）。

Node 侧测试与渲染检查使用同一优先级（`src/core/fixture.ts`），证据报告 `source.kind` 字段
记录实际消费的是 `fixture_v3` 还是 `example_fallback`，界面状态栏显示来源路径与 SHA256 前缀。

## 5. 服务接口（W2，planner_map_service）

| 端点 | 方法 | 说明 |
| --- | --- | --- |
| `/health` | GET | 存活、map_id/map_version 与 `grid_protocol_version`/`scene_rules_version`（协议 §3.5） |
| `/map/info` | GET | 元信息、对象计数、包围盒 |
| `/map/snapshot` | GET | 完整 M4 快照（与 v3 夹具一致） |
| `/map/query/point` | POST `{x_m,y_m}` | 点命中（严格多边形包含；返回 lanelets/areas 摘要） |
| `/map/lanelet/{id}` | GET | 属性 + 左右边界 + 中心线 |
| `/map/area/{id}` | GET | 属性 + 外环/内环 |
| `/map/lanelet/{id}/following` | GET | 后继 ID 列表 |
| `/map/lanelet/{id}/preceding` | GET | 前驱 ID 列表 |
| `/map/lanelet/{id}/adjacent?side=left\|right` | GET | 邻接 ID 列表 |
| `/map/route` | POST `{from_id,to_id}` | 最短 lanelet 序列 + 长度 |
| `/map/query/window` | POST `{bounds}` 或 `{center,size}` | 窗口裁剪（路径/点为地图坐标系；`window_frame` 描述局部系） |
| `/map/grid/driveable` | POST `{window,resolution_m,request_seq?}` | X-W2 可行驶域 `grid_snapshot`（`source=driveable`；协议 §3.2） |
| `/map/scene/preview` | POST `{window,resolution_m,obstacles?,request_seq?}` | X-W2 规划场景预览（`source=planning_scene`，含可复算 `scene_id`；协议 §3.3） |

响应统一公共 JSON 包络（`schema_version: 1` + `type`）；错误使用 HTTP 400/404/500 +
`{status, reason}`。客户端 `core/service/client.ts` 对网络/超时/HTTP/载荷错误分类并尽量携带服务端 reason。

### 5.1 场景 JSON（W3，`web_scene` schema_version 1）

```json
{
  "schema_version": 1,
  "type": "web_scene",
  "name": "",
  "map_id": "mapping_example",
  "map_version": "<OSM 字节 SHA256>",
  "vehicle": {"ref": "default_test_vehicle", "path": "tests/fixtures/v3/vehicles/default.json"},
  "start": {"x_m": 0, "y_m": 0, "yaw_rad": 0},
  "goal": null,
  "obstacles": [
    {"id": "obs-1", "type": "rectangle", "x_min_m": 0, "y_min_m": 0, "x_max_m": 20, "y_max_m": 10},
    {"id": "obs-2", "type": "convex_polygon", "vertices": [{"x_m": 0, "y_m": 0}, {"x_m": 1, "y_m": 0}, {"x_m": 0, "y_m": 1}]}
  ],
  "algorithm_params": {
    "hybrid_astar": {"time_budget_s": 5.0, "node_budget": 1000000},
    "corridor": {"enabled": true, "margin_m": 0.5}
  }
}
```

- 障碍字段与 `tests/fixtures/v3` 路径场景一致（矩形含边界；多边形顶点按序）；世界坐标米。
- 解析时障碍按自然 ID（obs-2 < obs-10）规范排序；序列化字节确定，可逐字节对比。
- `map_id`/`map_version` 与当前快照不一致时拒绝套用（导入与本地恢复同一规则）。
- localStorage 键 `planner.web_scene.v1`；自动保存去抖 400 ms；空场景清除存储。
- 编辑内容仅为覆盖层：地图快照只读，导出/导入/自动保存均不写回任何地图或夹具文件。

### 5.2 回放样例（W4，`tests/fixtures/v4/playback/`）

- `index.json`（`playback_index`）：11 个样例的类型、schema 版本、地图版本、相对文件名、
  SHA256 与 `expected_display`；`synthetic_note` 声明全部样例仅作显示测试。
- 样例类型：`map_snapshot`（小型合成地图，`bounds`/`left_boundary`/`polygon` 字段）、
  `plan_result`（`path[]` 分段 + 逐点 `direction/curvature_inv_m`；`trajectory[]` 含
  `t_s/velocity_mps/acceleration_mps2/curvature_inv_m`；`statistics`）、`bspline_curve`
  （`degree/knots/parameter_domain/control_points`）、`polynomial_curve`
  （`segments[].coefficients_ascending` 按段内局部时间 τ 秒升幂）、`corridor`
  （`polygons[].s_start_m/s_end_m/vertices`）、`collision_case`（足迹/障碍/重叠区/verdict）、
  `stale_result`（`map_version` v0 ≠ `current_map_version` v1、`request_seq` 落后）。
- 加载顺序：索引解析 → 逐文件 SHA256 校验（不匹配即拒绝）→ 类型解析 → 地图版本一致性检查
  （stale 演示项保留）；全部在浏览器内完成（`crypto.subtle`），无需算法服务。
- 回放模式使用小型合成地图作底图；退出回放恢复默认 M4 地图。

### 5.3 规划服务（W5，`planner_plan_service`，端口 8766）

| 端点 | 方法 | 说明 |
| --- | --- | --- |
| `/health` | GET | 存活、`map_id`/`map_version` 与 `grid_protocol_version`/`scene_rules_version`（协议 §3.5） |
| `/plan/submit` | POST | 提交任务（`request_id/map_id/map_version/start/goal/vehicle_ref/obstacles/algorithm_params`）→ `job_id/job_seq/status=queued`；地图版本不匹配 → 409 `stale_map` |
| `/plan/cancel` | POST `{job_id}` | queued 立即取消；running 协作取消（阶段边界）；已结束返回 `already_finished` |
| `/plan/status/{job_id}` | GET | `status`（queued/running/succeeded/failed/cancelled）+ `stage`（map/hybrid/corridor/optimize/validate/done）+ `reason` |
| `/plan/result/{job_id}` | GET | `hybrid.segments`（PathSegment[]）+ `corridor.polygons` + `trajectory[]` + `validation` + `statistics`（含 `timings_s` 与 `hybrid` 子统计） |
| `/plan/scene/{scene_id}` | GET | X-W2 该次规划实际使用的输入快照（`grid_snapshot`，逐字节一致；404 `unknown_scene` 表示未知/已淘汰；协议 §3.4） |
| `/plan/events` | WS | `job_update`（每次状态/阶段变迁）与 `job_result`（终态；仅状态，几何经 HTTP 结果拉取），均含单调 `job_seq` |

- 客户端（`src/core/plan/`）：submit/cancel/status/result 解析（`parse.ts`，容忍 `path` 与
  `hybrid.segments` 两种路径形状及 `corridor` 包装）、陈旧防护（`tracker.ts`：`latest_seq` +
  当前代际活动任务集合）、比较表（`compare.ts`）、结果 → W4 渲染管线（`convert.ts`）。
- 统计展示：真实服务 `statistics.hybrid` 展平 + `timings_s.hybrid/optimize` → 规划/求解耗时；
  `path_length_m` → 总长度；`direction_changes` 由路径方向交替派生（服务未直接提供）。
- 事件传输：`?planWs=` 指定 WS 地址（CDP 直连 `ws://127.0.0.1:8766`）；Vite dev 代理
  `/plan-api` 开启 `ws: true`；`npm run serve` 提供 WS 升级透传。未连接事件通道时回退 800 ms 状态轮询。
- `vehicle_ref` 传车辆文件路径（`tests/fixtures/v3/vehicles/default.json`，服务按仓库根相对路径读取）。
- `algorithm_params.corridor` 使用服务 `CorridorOptions` 键（场景 `margin_m` → `clearance_m`）；
  未知键会被服务拒绝（400 `unknown_algorithm_parameter`）。
- `Src/web/scripts/mock_plan_service.py` 为临时确定性替身（真实二进制未构建时的冒烟回退，
  输出与真实服务同形状；日志明确标注，**非算法正确性证据**）。

## 6. 消费的字段

| 字段/端点 | 用途 |
| --- | --- |
| `schema_version` / `type` | 版本与类型门禁（快照必须为 1 / `map_snapshot`；服务包络校验） |
| `map_id` / `map_version` | 状态栏与面板展示；版本一致性追踪 |
| `coordinate_system` | 校验 `frame_id="map"`、`unit="m"`；记录投影原点 |
| `bounds_m` | 自动取景、渲染检查包围盒、窗口查询默认范围 |
| `counts` | 与几何计数交叉核对 |
| `lanelets[].left/right/centerline` | 道路边界、中心线、本地命中、位姿航向 |
| `areas[].outer/inner` | 区域填充（evenodd）与命中 |
| `attributes` | 选中对象属性表（服务优先，本地回退） |
| 服务查询结果 | 选择高亮、拓扑高亮、路由序列、窗口裁剪叠加 |
| `/health` 版本字段 | `grid_protocol_version` 主版本 / `scene_rules_version` 门禁（在线栅格禁用与警告） |
| `grid_snapshot`（在线） | 可行驶域/场景预览栅格：实际分辨率、`degraded`、`scene_id`、`request_seq`、窗口 |
| `/plan/status`·`/plan/result` 的 `scene_id`/`scene_version` | 规划实际场景快照拉取入口（协议 §3.4） |
| `grid_snapshot`（`/plan/scene/{scene_id}`） | 规划实际场景展示（独立面板 + 品红叠加层；历史不覆盖预览） |

## 7. 验收方法（W1 + W2 + W3 + W4 + W5 + X-W2）

1. `python3 tools/task_runner.py build --component web` 通过（tsc 严格模式 + vite 构建）。
2. `python3 tools/task_runner.py test --component web --workers 1` 通过；测试覆盖：
   - W1/W2/W3：见前版；W4：索引解析与 SHA256 校验（含篡改/缺文件/版本拒绝）、
     各类型样例解析（含非法输入）、B 样条 de Boor（端点/直线/域外）、多项式（τ 秒边界/
     连续性/smoothstep）、时间映射（插值/夹紧/换向时刻/跨 π 航向）、统计格式化（单位/N/A）、
     覆盖层与绘制命令（失败/过期不产生几何、图层开关、深冻结不改底图）、剖面小图路径；
     W5：submit/cancel/status/result 解析（含真实服务 `hybrid.segments`/`statistics` 形状与
     别名派生）、客户端端点/请求体/错误分类（stale_map/网络/超时/非 JSON）、**陈旧覆盖防护**
     （乱序提交响应、陈旧事件、旧 job 迟到响应、新代际清空、对比模式双 job）、比较表（单位/N/A/
     校验原因）、结果 → 回放管线转换、走廊 s 区间文字命令（Canvas 执行 + SVG 导出）；
     X-W2：在线栅格客户端（端点/请求体/request_seq 回显/来源校验/错误分类）、`/health` 版本
     一致性（主版本 2.0 与 `scene_v2` 拒绝、字段缺失拒绝）、每来源 seq 陈旧/异常/地图版本丢弃、
     显示上限 2 的幂调整、在线窗口求交、规划快照解析与 scene_id 校验（404 `unknown_scene`）、
     晚到旧 `scene_id` 丢弃、预览 vs 规划场景显式比较、规划场景叠加纹理（空闲透明/占据品红）。
   JUnit 输出 `reports/junit.xml` 并复制到 `artifacts/runs/<run_id>/junit.xml`（入口收集）。
3. 渲染检查证据：`Src/web/evidence/render_check.json` + `render_check.svg`（W1 确定性报告）。
4. 集成冒烟：`exec --component map --workers 4 -- bash Src/web/scripts/integration_smoke.sh` 通过：
   - `outputs/service_compare.json`：地图服务 vs v3 快照 20 项检查；
   - `outputs/browser_cdp_w2/browser_cdp_report.json` + 截图：W1 交互、W2 查询、W3 场景编辑、
     W4 逐类回放（RS/成功/换向/失败/B-spline/多项式/走廊/碰撞/过期）与推进/拖拽联动、
     W5 提交→状态/阶段→结果渲染→结果回放、`/plan/events` WS 推送连通、陈旧事件忽略/乱序新事件接受、
     取消（填充任务占住队列后 UI 取消 → cancelled）、对比提交并排统计表、回放模式下提交阻止；
     X-W2（`--xw2`）：可行驶域在线图层（`request_seq`/来自服务标注/像素）、旧 seq 注入不得覆盖
     新请求、场景预览 `scene_id`、提交→结果→`GET /plan/scene/{scene_id}`→"规划实际场景"面板与
     品红叠加层像素、与当前预览 scene_id 显式区分（历史场景展示后预览 scene_id 保持不变）；
   - `outputs/browser_cdp_w5_stale/browser_cdp_report.json`：规划服务地图版本不匹配 → 409 `stale_map` 显式提示；
   - `outputs/browser_cdp_xw2_reorder/…`：mock 延迟首响应 → 乱序响应被丢弃（`stale_ignored`）、
     展示为最新请求（单元编码 seq）、服务降级显示实际分辨率；
   - `outputs/browser_cdp_xw2_protocol/…`：mock `grid_protocol_version=2.0` → 在线图层禁用 +
     可见状态（"在线栅格已禁用：协议主版本不匹配…2.0"），且不发起请求；
   - `outputs/browser_cdp_xw2_scene_rules/…`：mock `scene_rules_version=scene_v2` → 可行驶域仍可用、
     规划场景预览禁用 + 可见状态；
   - 规划服务优先使用真实二进制；未构建时回退 mock（日志标注 `plan service kind: mock`），
     mock 结果非算法正确性证据；各阶段失败按 §5.4 继续并汇总首个非零退出码。
   - W4 检查不依赖算法服务（无 `--api-target` 的 CDP 运行同样全部通过）。
5. 人工浏览器检查（可选）：`npm run dev`（或 `npm run serve`）+ 本地服务后确认交互。

## 8. 已知限制

- 未做像素级视觉回归（截图基线对比）；CDP 检查为指标断言（像素计数、比例、ID、位移、场景字段）。
- 仅 Chrome 无头环境验证；Firefox 等未测。
- 无头 `--screenshot` 一次性捕获（`browser_smoke.sh`）存在已知环境偏差；CDP 固定视口截图为准。
- 服务离线时拓扑仅 following/preceding 有本地近似（按中心线端点），adjacent 需要服务；
  命中为本地几何近似，权威结果以服务为准。
- 多边形障碍仅支持渲染/删除/场景导入，交互式顶点编辑未实现（矩形已完整支持）。
- 光标足迹固定航向 0°（未按最近 lanelet 自动对齐）。
- W4 回放底图为小型合成地图，与主 M4 地图互斥（切换回放会替换底图）；W2/W3 面板在回放模式下
  仍显示但对合成地图无查询意义（服务查询已按地图版本自动回退本地）。
- 回放速度为实时 1×（无倍速选项）；轨迹按线性插值（样例点距 ≤1.5 s，显示足够）。
- 大地图性能未评估（全量重绘）；窗口裁剪叠加显示全量路径，未做视野剔除。
- 画布尺寸变化不再自动重新取景（保留当前视图，仅更新画布尺寸）；重置视图显式取景。
- W5 取消语义：真实服务在 v3 夹具上单任务约 25 ms，运行中取消依赖阶段边界（可能返回
  `already_finished`）；验收以"填充任务占住服务队列后取消 queued 任务"演示，UI 的取消按钮
  仅作用于当前代际活动（最后提交）任务。
- W5 事件通道：`job_result` 推送不含几何（服务设计），结果经 HTTP `/plan/result` 拉取；
  断线自动重连最多 5 次（间隔 2 s），未做增量重放（重连后经 HTTP 状态/结果重同步）。
- `npm run serve` 为静态 dist + 反向代理（无鉴权/限流），仅本地开发与检查使用。
- X-W1 栅格层为离线合成夹具（`source: "fixture"`）：不依赖服务、不做在线栅格请求；
  服务端 `driveable`/`planning_scene` 快照消费属 X-W2 联调。显示性能门槛实测见
  `grid_display_bench.py` 报告；benchmark_tiers §4.1 要求的"每尺寸开阔/迷宫两个快照"
  待 `grid_scenario_gen v1` 实现后补充（当前按 v6 显示档单夹具口径评估，已在报告登记）。
- X-W2 在线请求窗口取当前视口与地图边界的交集；在线显示按 ≤1024² 限制在请求前把分辨率按
  2 的幂提高（"显示上限调整"），服务端另有 16,777,216 格预算降级（`degraded=true`）。
- X-W2 规划实际场景快照来自规划服务内存 LRU 16，重启失效；404（未知/已淘汰）状态可见，
  可凭有效窗口/分辨率/障碍用当前预览重新生成（内容寻址、确定性）。
- X-W2 mock 阶段（`mock_grid_service.py`：乱序延迟/版本字段/降级）为确定性替身，
  **非算法正确性证据**；真实服务行为以 `Src/map` 为准。
- 服务侧（b5247df 起）要求起终车辆足迹位于 lanelet 可行驶域内：W2 CDP 检查在点击选点后
  把终点数值调整到 lanelet B 中心线内点（`w2_goal_interior_adjust`），否则短 lanelet 末端/
  地图边缘的点击目标会使规划以 `invalid_input`（footprint in collision）失败；该约束属服务侧，
  网页端仅记录并在报告中单列。
