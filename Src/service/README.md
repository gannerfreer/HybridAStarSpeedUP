# Src/service — W5 规划服务（planner_plan_service）

> 状态：W5 服务侧交付。二进制 `planner_plan_service`（默认端口 8766），
> 在启动时导入一次 Lanelet2 OSM 地图，串行执行
> Hybrid A* → 安全走廊 → Dftpav 优化 → planner::validation 硬门禁，
> 通过 HTTP/JSON 提供任务提交、取消、状态与结果查询，通过 WebSocket
> `ws://…/plan/events` 推送任务状态与结果通知。
> 服务是 Dftpav（GPL-3.0 实验后端）在集成层的唯一消费者；归属与许可证
> 保留在 `Src/dftpav/PROVENANCE.md` 与 `LICENSE-GPL-3.0.txt`。

执行依据：`Docs/component_development_plan.md` §2.2（W5）、§6.1/§6.2、§7.1/§7.2；
接入约定：`tools/README.md` §7（`components.json` 的 `service` 条目）。
本文档描述实际实现与契约；运行证据由入口任务记录到 `artifacts/runs/`，
报告由集成负责人归档到 `Docs/reports/`。

## 1. 构建与运行

经统一入口（推荐）：

```sh
python3 tools/task_runner.py build --component service --profile release --parallel 4
python3 tools/task_runner.py test  --component service --workers 2
# 长驻服务（占 1 个 worker 名额）：
python3 tools/task_runner.py run --component service --workers 1
```

直接运行（仓库根为工作目录）：

```sh
build/release/Src/service/planner_plan_service \
  --map tests/fixtures/v3/map/mapping_example.osm \
  --origin-lat 49.00345654351 --origin-lon 8.42427590707 \
  --port 8766 [--run-dir <dir>]
```

- CLI 契约与地图服务一致：`--map <osm> --origin-lat <deg> --origin-lon <deg>
  --port <port> [--run-dir <dir>]`；默认 host `127.0.0.1`、默认端口 **8766**；
  `--port 0` 自动分配并打印实际端口。其余可选项：`--projection`、
  `--origin-ele`、`--no-validation`、`--quiet`。
- 退出码：0 正常退出；1 地图导入/运行失败；2 参数用法错误；3 端口绑定失败。
- `--run-dir` 提供时写入 `service_status.json`（端口、地图身份、启动时间、
  推送方式），供运行证据引用。
- 地图版本为 OSM 文件原始字节的 SHA256（与地图组件一致），`map_id` 为文件
  stem；服务启动时导入一次并在整个进程内复用。

## 2. HTTP 契约

所有响应为 UTF-8 JSON，带公共包络 `{"schema_version": 1, "type": ...}`。
错误体含 `status`（稳定代码）与 `reason`（可读说明），HTTP 码：
400 无效输入（`invalid_input`）、404 未知任务/路径（`not_found`）、
409 地图过期（`stale_map`）、500 内部错误（`internal_error`）。

| 方法与路径 | 请求 | 响应 `type` |
| --- | --- | --- |
| `GET /health` | — | `health`（status/map_id/map_version/uptime_s/grid_protocol_version/scene_rules_version） |
| `POST /plan/submit` | 见 §2.1 | `plan_submit`（job_id/job_seq/request_id/map_id/map_version/status="queued"） |
| `POST /plan/cancel` | `{job_id}` | `plan_cancel`（status=`cancelled`\|`not_found`\|`already_finished`，附 job_id/job_seq） |
| `GET /plan/status/{job_id}` | — | `plan_status`（job_id/request_id/job_seq/map_id/map_version/status/stage/reason?/detail?/scene_id?/scene_version?） |
| `GET /plan/result/{job_id}` | — | `plan_result`（见 §2.3，含 scene_id?/scene_version?） |
| `GET /plan/scene/{scene_id}`（X-M1） | — | `grid_snapshot`（规划实际使用的输入快照，见 §2.4） |
| `WS /plan/events` | — | 推送事件（见 §3） |

- `status` ∈ `queued` / `running` / `succeeded` / `failed` / `cancelled`；
  `stage` ∈ `map` / `hybrid` / `corridor` / `optimize` / `validate` / `done`。
- `job_seq` 为进程内单调递增序号（提交成功即分配；`stale_map`/400 不产生任务、
  不消耗序号），供客户端防陈旧覆盖。
- 未知 `job_id` 的 status/result 请求返回 404；取消未知任务返回 HTTP 200 +
  `status:"not_found"`（取消请求本身被正常处理）。
- 结果可在任务运行中提前拉取：非终态只返回状态与阶段，不含几何。

### 2.1 `POST /plan/submit` 请求

```jsonc
{
  "request_id": "req-1",              // 必填，非空
  "map_id": "mapping_example",        // 必填，与已加载地图一致
  "map_version": "11561c…",           // 必填，OSM 字节 SHA256
  "start": {"x_m": 2.0, "y_m": 5.0, "yaw_rad": 0.0},   // 必填
  "goal":  {"x_m": 18.0, "y_m": 5.0, "yaw_rad": 0.0},  // 必填
  "vehicle_ref": "tests/fixtures/v3/vehicles/default.json", // 可选
  "obstacles": [                      // 可选（缺省空），与 web_scene 同 schema
    {"id": "obs-1", "type": "rectangle",
     "x_min_m": 9.0, "y_min_m": 4.0, "x_max_m": 11.0, "y_max_m": 6.0},
    {"id": "obs-2", "type": "convex_polygon",
     "vertices": [{"x_m": 9.5, "y_m": 3.5}, {"x_m": 12.5, "y_m": 4.5},
                  {"x_m": 12.0, "y_m": 7.5}, {"x_m": 9.0, "y_m": 6.5}]}
  ],
  "algorithm_params": {               // 可选
    "hybrid_backend": "kth_normalized",  // 可选，默认 kth_normalized
    "hybrid": {"heading_bins": 72},      // 可选，透传后端 configure() 键
    "time_budget_s": 5.0,                // 可选，默认 §7.2：5 s
    "node_budget": 1000000,              // 可选，默认 §7.2：1e6
    "corridor": {"sample_step_m": 0.25, "clearance_m": 0.05, "max_expansion_m": 4.0,
                 "construction_step_m": 0.05, "verification_step_m": 0.02,
                 "perturbation_yaw_rad": 0.017453292519943295,
                 "perturbation_position_m": 0.02},
    "dftpav": {
      "time_budget_s": 10.0,             // 可选，默认 §7.2：10 s（见 §4 取消语义）
      "limits": {"max_forward_velocity_mps": 5.0, "max_backward_velocity_mps": 2.0,
                 "max_forward_acceleration_mps2": 4.0, "max_backward_acceleration_mps2": 4.0,
                 "max_forward_curvature_inv_m": 0.1951, "max_backward_curvature_inv_m": 0.1951,
                 "max_lateral_acceleration_mps2": 5.0, "max_steering_rate_radps": 10000.0},
      "options": {"sample_dt_s": 0.05, "corridor_repair": true, "...": "OptimizeOptions 字段"}
    }
  }
}
```

- `map_id`/`map_version` 与已加载地图不匹配 → **409 `stale_map`**（不排队、不执行）。
- `vehicle_ref`：车辆 JSON 路径（字符串或 `{"ref","path"}` 对象；相对路径按
  服务工作目录解析），文档为 v3 车辆 schema（`wheelbase_m`、`width_m`、
  `front_overhang_m`、`rear_overhang_m`、`max_steer_rad`、`safety_margin_m`）；
  缺省为 §7.1 默认车辆（`tests/fixtures/v3/vehicles/default.json` 同值）。
  也可用内联 `vehicle` 对象（两者只能出现一个）。
- `obstacles`：矩形（`x_min_m<x_max_m`、`y_min_m<y_max_m`，倒置输入会被规范化，
  退化矩形报错）或凸多边形（≥3 顶点、有限、凸性由碰撞组件校验）。同一障碍
  同时进入碰撞场景（精确多边形）与栅格（中心点占据，供启发式）。
- `algorithm_params` 未知键（含 `corridor`/`dftpav`/`dftpav.limits`/`dftpav.options`
  子对象）→ 400，显式指出键名，绝不静默忽略；请求顶层未知字段容忍。
- 重复 `request_id` 不拒绝：每次提交生成新任务，由 `job_seq` 区分。

### 2.2 场景构造（服务侧约定）

- 服务只导入地图一次，用其 `map_id`/`map_version`（过期校验）、局部坐标系、
  地图边界（点层包围盒）与 lanelet 几何；启动后不再重读地图。
- 每个任务在 start/goal/障碍包围盒外扩「车辆外接圆半径 + 5 m」的局部窗口，
  窗口裁剪到地图边界，栅格分辨率默认 0.1 m；窗口超过 16M 格时分辨率按 2 的
  幂次上调，仍超限则任务失败 `window_too_large`。
- 场景域基础是地图的 **lanelet 可行驶域**（`Map::export_driveable_grid`）：
  窗口内与任一 lanelet 可行驶多边形相交（闭集，接触也算）的单元为空闲，
  其余为占据；请求障碍随后叠加为占据（「中心点在闭合多边形内即占据」，
  与固定夹具派生一致）。Lanelet2 area（停车场等）对象不参与首版可行驶域。
  栅格遵循 ROS OccupancyGrid 约定：origin 为窗口最小角（yaw 0），单元
  (r,c) 中心 = origin + ((c+0.5)·res, (r+0.5)·res)。
- start/goal 必须在地图边界内**且落在可行驶域单元上**，否则任务失败原因
  `start_outside_map`/`goal_outside_map`、`start_outside_driveable`/
  `goal_outside_driveable`。可行驶域检查先于障碍物栅格化，因此「障碍物压在
  起点上」仍按既有 Hybrid 阶段 `invalid_input` 上报。
- 规划（Hybrid A* → 走廊 → Dftpav → 独立验证）因此被约束在可行驶域内：
  越出车道的轨迹会被碰撞检查（栅格单元 + 精确障碍多边形）拒绝。
  剩余边界见 §6。
- X-M1（栅格快照协议 v1 §5）：场景组合由 `planner::scene` 统一执行，有效
  窗口/分辨率/障碍坐标按协议 §5.1 量化（1e-9）后作为唯一输入；`scene_id`
  由场景库规范序列化（键排序、定点 9 位小数、障碍按 id 排序）取 SHA-256，
  `scene_version="scene_v1"`。地图服务的 `/map/scene/preview` 调用同一组合
  与同一标识函数，因此同一地图版本/有效窗口/分辨率/障碍集合下预览与规划
  输入快照逐格一致（协议 §5 跨服务一致，见 §5 测试）。

### 2.3 `GET /plan/result/{job_id}` 响应

成功（`status:"succeeded"`）包含全部几何，类型使用公共 v1 编码：

```jsonc
{
  "schema_version": 1, "type": "plan_result",
  "job_id": "job-1", "request_id": "req-1", "job_seq": 1,
  "map_id": "mapping_example", "map_version": "11561c…", "status": "succeeded",
  "hybrid": {"backend": "kth_normalized",
             "segments": [{"points": [{"pose": {...}, "s_m": 0.0,
                                       "curvature_inv_m": 0.0, "direction": "forward"}]}],
             "statistics": { /* 后端统计原样透传 */ }},
  "corridor": {"id": "...", "map_id": "...", "map_version": "...", "frame_id": "map",
               "polygons": [{"s_start_m": 0.0, "s_end_m": 0.5,
                             "vertices": [{"x_m": ..., "y_m": ...}]}],
               "statistics": { /* CorridorStats */ }},
  "trajectory": [{"t_s": 0.0, "pose": {...}, "velocity_mps": 0.0,
                  "acceleration_mps2": 0.0, "curvature_inv_m": 0.0}],
  "validation": {"valid": true, "reason": "valid", "detail": "",
                 "first_failure": null, "checks": {...}},
  "statistics": {"hybrid_backend": "kth_normalized", "stage": "done",
                 "grid": {...}, "timings_s": {"map": ..., "hybrid": ..., "corridor": ...,
                 "optimize": ..., "validate": ..., "total": ...},
                 "path_length_m": ..., "trajectory_length_m": ..., "trajectory_points": ...,
                 "hybrid_budget_s": 5.0, "hybrid_node_budget": 1000000, "dftpav_budget_s": 10.0}
}
```

失败/取消只含 `status`、`stage`、`reason`、`detail`（可读）与部分 `statistics`
（计时与到达的阶段统计），**不含几何**；`reason` 为稳定代码：

| 来源 | `reason` |
| --- | --- |
| Hybrid 失败 | `invalid_input` / `no_path` / `timeout` / `cancelled` / `optimization_failed` / `validation_failed`（`detail` 为后端说明） |
| 参数/后端 | `unknown_hybrid_backend` / `invalid_hybrid_config` / `unsupported_hybrid_parameters` |
| 场景 | `start_outside_map` / `goal_outside_map` / `start_outside_driveable` / `goal_outside_driveable` / `window_outside_map` / `window_too_large` / `invalid_scene` |
| 走廊 | `corridor_invalid_input` / `corridor_blocked` / `corridor_narrow` / `corridor_no_corridor` |
| Dftpav | `solver_failed` / `cost_too_high` / `validation_<约束>` 等优化器稳定代码 |
| 服务门禁 | `validation_<约束>`（独立复跑 `planner::validation` 的结果） |
| 取消 | `cancelled` |

### 2.4 规划输入快照（X-M1，协议 §3.4）

- `map` 阶段场景构建成功后，任务记录 `scene_id`/`scene_version`：`GET /plan/status`
  与 `GET /plan/result` 在场景构建后（成功、失败、取消都）携带这两个字段；
  成功结果的既有 `statistics` 不变；`POST /plan/submit` 不变（提交时场景尚未
  构建）；`WS /plan/events` 的 `job_update`/`job_result` 在 map 阶段完成后
  可选携带 `scene_id`（HTTP 状态/结果为规范来源）。
- 服务把该次规划实际使用的 `grid_snapshot`（`source="planning_scene"`，字段
  与地图服务预览一致）保存在**内存 LRU**：容量 16、按 `scene_id` 去重（相同
  内容只存一份）、重启失效（纯内存，首版）。`GET /plan/scene/{scene_id}` 命中
  时按保存原文逐字节返回；未知或已淘汰返回 404 `not_found`/`unknown_scene`
  （客户端可凭有效窗口/分辨率/障碍重新预览；内容寻址保证同一输入重生成同一
  `scene_id`）。
- 容量档/大图不进快照：仅在线验收范围（单元数 ≤ 1024²）进入 LRU；2048²/4096²
  属离线容量档（扩展计划 §6.1/§6.3），服务不保留（对应 `scene_id` 的
  `GET /plan/scene` 返回 404）。规划窗口超过单元预算时任务本身以
  `window_too_large` 失败。
- 快照不携带 `request_seq`（提交请求无该字段）；`budget_cells` 回显生效预算
  （16,777,216）。

## 3. 推送契约（WebSocket `/plan/events`）

```jsonc
{"schema_version": 1, "type": "job_update", "job_id": "job-1", "request_id": "req-1",
 "job_seq": 1, "map_version": "11561c…", "status": "running", "stage": "corridor",
 "scene_id": "20821dc8…e776"}
{"schema_version": 1, "type": "job_result", "job_id": "job-1", "request_id": "req-1",
 "job_seq": 1, "map_version": "11561c…", "scene_id": "20821dc8…e776"}
```

- `job_update`：每次状态/阶段迁移推送一次（提交即 `queued`/`map`；执行中按
  `map`→`hybrid`→`corridor`→`optimize`→`validate`；终态 `succeeded|failed|cancelled`
  且 `stage:"done"`）。map 阶段完成后（作业携带场景标识起）可选携带
  `scene_id`；HTTP 状态/结果为规范来源。
- `job_result`：任务进入终态后推送（成功、失败、取消均推送），客户端据此拉取
  `GET /plan/result/{job_id}`（结果不放在推送里）；场景已构建时携带 `scene_id`。
  `job_seq` 供客户端丢弃陈旧任务的推送覆盖。
- 多订阅者广播；慢客户端队列上限 4096 条，溢出时服务端以关闭帧
  （PolicyViolation）终止该连接，客户端应重连并通过 HTTP 重新同步，不阻塞
  规划工作线程。服务停止时连接收到 GoingAway 关闭。

### 推送方式选择与偏差

- 已锁定依赖 cpp-httplib **v0.60.1 原生支持 WebSocket**（RFC 6455，含
  `Server::WebSocket`、`ws::WebSocketClient` 与心跳），因此按契约实现
  `ws://…/plan/events`，**未采用 SSE**，无需引入 Boost.Beast 等系统库；
  实现方式、选择与偏差记录于本 README 与运行报告。
- 库的上游说明明确其为阻塞式「每连接一线程」实现，适合本地中小规模连接；
  W5 为回环地址上的单机联调服务（地图服务同为回环 HTTP），该边界可接受；
  若未来需要大量并发连接，应更换非阻塞实现（届时按新依赖锁定流程评估）。
- 事件流为纯服务端推送；客户端消息被忽略。未实现事件重放：重连客户端用
  `GET /plan/status/{job_id}` / `GET /plan/result/{job_id}` 重新同步。

## 4. 管线与取消语义

单工作线程按提交顺序执行；阶段与事件一一对应：

1. **map**：构造可行驶域栅格（lanelet 车道导出）与碰撞场景（§2.2）。
2. **hybrid**：`algorithm_params.hybrid_backend`（默认 `kth_normalized`，
   可选 `ros_free`、`smac_normalized`、`zm0612_normalized` 等已注册后端）
   使用正式 `planner::rs` 与 `planner::collision`；预算默认 §7.2（5 s / 1e6
   节点）；检查取消回调与预算，失败给出稳定原因。
3. **corridor**：从 Hybrid 路径 + 请求障碍生成安全走廊（`corridor{...}`
   选项）；狭窄/断裂/受阻明确失败，不做静默回退。
4. **optimize**：Dftpav 以 Hybrid 路径为初值、走廊为约束求解时序轨迹；
   限值缺省按车辆推导（曲率 `tan(max_steer)/wheelbase`，速度/加速度取
   优化器默认），可由 `dftpav.limits/options` 覆盖；阶段耗时超过
   `dftpav.time_budget_s`（默认 10 s）按 `timeout` 失败。
5. **validate**：服务独立复跑 `planner::validation::validate_trajectory`
   （同一车辆/限值/终点/场景/走廊），**通过才返回成功**；失败返回
   `validation_<约束>` 与首个失败点。Dftpav 内部门禁通过但复检失败属于集成
   不一致，同样按失败上报。

取消语义：

- 排队中取消：立即转 `cancelled`（`stage:"done"`），工作线程跳过。
- 运行中取消：设置协作标志；Hybrid 在扩展循环检查，Dftpav 在求解的
  L-BFGS 迭代边界检查（`OptimizeOptions::cancel_check`，见 Src/dftpav
  README §1 与 PROVENANCE.md），阶段边界再次检查。
- Dftpav 阶段支持**协作式实时取消**（移植扩展）：粒度 = 求解前 + 每个
  L-BFGS 迭代/代价评估边界 + 求解后修复边界 + 验证前。取消在阶段内部生效，
  优化器返回 `kCancelled`/"cancelled" 且不返回部分轨迹，服务按既有取消语义
  以 `cancelled`/`stage:"done"` 收尾（响应不含任何几何）。
- Dftpav 的 `time_budget_s` 仍在阶段返回后判定（求解器没有时间回调），
  超预算按 `timeout` 上报。
- 终态任务再次取消返回 `already_finished`。

## 5. 测试与证据

- `planner_service_tests`（label `service`）：解析器与场景构造单测（默认值、
  车辆引用、障碍校验、栅格化、窗口裁剪/分辨率上调、地图边界拒绝），新增
  可行驶域用例：真实 v3 地图上「道路外单元 occupied、车道内 free、障碍叠加
  占据」，以及起点/终点在道路外时 `start_outside_driveable`/
  `goal_outside_driveable`（合成小地图 `tests/data/window_map.osm` 覆盖窗口
  裁剪到地图边界）；以及流水线取消单测 `test_pipeline_cancel.cpp`
  （不启动服务进程：用确定性轮询计数在 Dftpav 阶段内部触发取消，断言
  `result.optimize` 为 `kCancelled` 且无轨迹、无几何统计）。
- `planner_plan_service_tests`（label `service`）：以 `--port 0` 启动真实服务
  进程，覆盖 HTTP 成功链（submit→status→result、kth_normalized 默认后端与
  `ros_free` 通道场景）、`stale_map` 409、400/404、排队/运行取消与
  `already_finished`/`not_found`、失败原因（起点碰撞、道路外起点/终点
  `start_outside_driveable`/`goal_outside_driveable`、无几何）、`job_seq`
  单调，以及 WebSocket 事件顺序、阶段覆盖、终态通知与多订阅者广播。
  「运行中取消」的慢作业使用可行驶域内、被闭环障碍在精确几何上包围的目标
  （环墙 0.02 m 厚、薄于栅格单元，使栅格势场不提前判定不可达；远处填充
  障碍扩大局部窗口），不再依赖道路外自由空间。
- X-M1 测试：`planner_service_tests` 新增快照 LRU/去重/淘汰、`scene_id`
  冻结值（协议 §5.3 示例、v6 P11/P13）、定点格式、base64_u8 与 grid_snapshot
  组装、场景预览组合与窗口裁剪/量化/错误码；`planner_service_scene_tests`
  启动规划服务 + 地图服务，覆盖成功/失败任务的 `scene_id` 字段、快照 200 与
  逐字节一致、404 `unknown_scene`、17 场景 LRU 淘汰、WS `scene_id`，以及
  P11 跨服务逐格一致（预览与 `/plan/scene/{scene_id}` 的 `cells`、`scene_id`、
  宽高、分辨率、原点相等）。
- `planner_plan_service_example`（`examples/plan_example.cpp`）：不启动 HTTP 的
  独立管线示例（打印阶段、计时、段/多边形/轨迹点数与校验结果）；
  `--self-check` 作为 label `service` 的 CTest 用例运行 open 与通道两个场景。
- 运行方式：`python3 tools/task_runner.py test --component service --workers 2`；
  运行证据（stdout/资源/事件/JUnit）保存在 `artifacts/runs/<run_id>/`。
  本波次的 build/test run ID 与结果见交付报告（由集成负责人归档）。

## 6. 当前边界与遗留

- 可行驶域已由 lanelet 车道栅格化并作为场景域基础（§2.2）：地图组件提供
  `Map::export_driveable_grid`（窗口 + 分辨率 → `common::GridView`），服务
  在局部窗口内导出、叠加请求障碍并做起点/终点可行驶域校验。剩余边界：
  * 只有 lanelet 车道参与可行驶域，Lanelet2 area（停车场等）对象不参与；
  * 单元语义为「与车道多边形相交即空闲」，可行驶域相对车道几何外扩至多约
    一个单元；精确约束仍由碰撞检查（占据单元 + 精确障碍多边形）承担；
  * 局部窗口仍限制在 start/goal/障碍包围盒外扩边距后裁剪到地图边界，不会
    扩展到整张地图路网。
- Dftpav 阶段的**取消已实时化**（协作式，L-BFGS 迭代边界轮询
  `OptimizeOptions::cancel_check`；取消后不返回部分轨迹，见 §4）。剩余限制：
  `time_budget_s` 仍是阶段返回后的判定，不硬中断求解（求解器没有时间回调；
  超预算按 `timeout` 上报）。
- 单工作线程串行执行（契约要求顺序执行与 `job_seq` 防陈旧）；结果几何在
  成功响应中一次返回，未分页；超大轨迹的响应大小未做上限。
- X-M1 输入快照为进程内内存 LRU（16、按 `scene_id` 去重、重启失效、容量档
  ≤1024² 限制，见 §2.4）；无持久化、无跨进程共享，属首版约定。
- 事件队列溢出会主动断开慢客户端（需重连重同步），不做无限缓冲。
- `vehicle_ref` 相对路径按服务工作目录解析；经入口 `run` 时工作目录为仓库根。
