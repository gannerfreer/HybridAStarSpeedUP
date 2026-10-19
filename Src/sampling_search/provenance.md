# planner_sampling_search 来源与适配记录（X-S4）

- 日期：2026-10-12。
- 范围：`Src/sampling_search/`（库、适配器、测试、内部 CMake）的来源锁定、许可、
  适配补丁、原生偏差与验证证据。**不修改** `dependencies.lock.json`；锁条目与
  根构建接线由后续 wiring 代理统一完成（本文件记录待接线事实）。
- 依据：`Docs/expansion/candidate_registration.md`（S-06/S-09/S-10 与 §6/§7）、
  `Docs/expansion/probes/family_d_probe.md`、`supplement_ch0p1k3_probe.md`、
  `supplement_klampt_probe.md`、`Docs/component_expansion_plan.md` §4.1/§4.2。
- 构建/测试证据：`artifacts/runs/<run_id>/`（见 §7；命令均经
  `tools/task_runner.py exec --component sampling_search`，编译持全局编译锁
  `flock .git/planner-runtime/build.lock` 且 `-j4`）。

## 1. 公共契约与分组

- 统一输入/输出契约见 `include/planner/sampling_search/types.hpp`（二维边界、
  起终点、统一点/边有效性回调、显式沿边步长、种子、长度目标、预算；状态区分
  `kSuccess`/`kNoPath`/`kTimeout`/`kCancelled`/`kInvalidInput`）。
- **预算耗尽无解报 `kTimeout`，不作为不可达证明**；采样后端本轮不返回
  `kNoPath`。取消请求丢弃部分结果（`kCancelled` 且 path 为空）。
- 平凡情形：起终点重合（距离 ≤ 1e-9）时所有适配器经公共助手返回单点成功
  路径（cost 0），不调用后端；`stats.native_params` 标记 `trivial`。
- 返回路径统一经 `path_edges_valid` 复检；不满足统一沿边有效性或端点容差的
  原生解一律丢弃（按无解处理），不返回无效边。
- 分组（候选登记 §4.1）：连续空间组 = OMPL（RRT*/Informed 单列）、Ch0p1k3、
  Klamp't；栅格采样组 = vss2sn（与连续组分开评分）；ai-winter 为独立原生组
  （GPL 隔离、疑似派生，不参与公平横比）。
- 本轮只做正确性（X-S4 验收范围）；性能矩阵与预算 × 种子 0–29 正式基准属
  X-B1，不在本轮结果中。

## 2. OMPL 1.6.0（S-09，连续空间组）

| 项 | 值 |
| --- | --- |
| URL | https://github.com/ompl/ompl |
| 版本/commit | 1.6.0 / `e2994e580fcba0eb117e38bd1dc7439c4beb35ef`（tag 对象 89dcc934…） |
| 锁条目 | `dependencies.lock.json` → `dependencies.ompl`（复用，不静默升级） |
| 归档 | `https://github.com/ompl/ompl/archive/refs/tags/1.6.0.tar.gz`，SHA256 `f03daa95d2bbf1c21e91a38786242c245f4740f16aa9e9adbf7c7e0236e3c625`（探针复算与锁一致；本组件构建时由 `URL_HASH` 再次校验） |
| 许可证 | BSD-3-Clause，`ompl-1.6.0/LICENSE` |
| 接入方式 | FetchContent（`URL + URL_HASH`；`OMPL_BUILD_TESTS/DEMOS/PYBINDINGS/PYTESTS=OFF`、`OMPL_REGISTRATION=OFF`）；沿用 RS `planner_rs_ompl_reference` 私有适配器模式，公共 API 不暴露 OMPL 类型 |
| 适配面 | `StateValidityChecker`（点）、`MotionValidator`（沿边，`checkMotion` 两重载；lastValid 变体对不透明布尔回调做 40 步二分）、`timedPlannerTerminationCondition` + 取消条件组合、`RNG::setSeed`、中间解回调（首次解）、`PathLengthOptimizationObjective::setCostThreshold`（长度目标提前结束） |
| 变体 | RRTstar 与 InformedRRTstar 单列（`stats.variant`），同一来源不重复计数 |
| 原生偏差 | 无上游补丁；OMPL 路径点未简化（保持规划器返回）；种子为统一输入的编号映射（0→1，因为 OMPL `RNG::setSeed(0)` 被上游忽略），**不声称**与其他实现同种子同采样序列；`RNG::setSeed` 是进程全局单例，进程内首次使用 RNG 后再次 setSeed 被上游忽略（`native_params.rng_note` 记录）——正式基准须每 trial 独立进程；预算耗尽时 OMPL 可能上报"近似解"（末点未达目标），适配器只接受 `haveExactSolutionPath()`，近似解丢弃报 `kTimeout` |

## 3. Ch0p1k3（连续空间组补源，MIT）

| 项 | 值 |
| --- | --- |
| URL | https://github.com/Ch0p1k3/path-planning-algorithms-rrt-rrtstar |
| commit | `4ae8c935f985097e50976e75220c27dac57ffa61`（2023-04-22） |
| 许可证 | MIT；`LICENSE` SHA256 `5ccc37ecd73c384b567808acd569b0dca5b6c3ec17942ddd8b43a0b91478e368` |
| 获取方式 | **最小抽取副本** `third_party/ch0p1k3/`（上游含约 2.9 GB `tests/`，整仓 FetchContent 不可行；抽取集与探针 Gate 3 一致）。`src/` 23 文件，除 `rrt.hpp` 的 4 行种子钩子外与上游逐字节一致（抽取时逐文件 diff 核对） |
| 适配补丁 | 仅 `rrt.hpp` 种子钩子（`setSeed`，4 行，diff 见 `third_party/ch0p1k3/PATCHES.md`）；预算/取消为驱动层镜像上游 `RRTStarAlgorithm::launch` 循环（采样/最近邻/steer/近邻/选父/重连/插入/改边全部调用上游原语） |
| 依赖 | nanoflann v1.4.3（`third_party/nanoflann/nanoflann.hpp`，SHA256 `b63dac5a…`，与上游 pin 一致）；pugixml 用锁条目 **1.16**（上游 FetchContent pin 为 1.13；XML 解析 API 不变，版本差异记录在此）；SFML 仅可视化，用探针最小桩 `third_party/ch0p1k3/sfml_stub/`，永不调用 |
| 原生偏差 | ① 需整数边界（`Map` width/height 为 size_t），非整数边界返回 `kInvalidInput`；② 障碍经保守栅格化（单元封闭矩形与障碍相交 ⇒ 受阻；9 点 + 边界边采样），起点/终点所在单元强制解除以规避上游 `exit()`；返回路径仍经统一复检，复检失败按无解处理；③ 目标域语义为 `eps` 距离内到达，适配器按统一边有效补接精确终点并记录原生末点间距；④ steer 步长 = max(3×沿边步长, 1)、eps = max(沿边步长, 1)、gamma=100（探针口径；上游对 <1 值会自行改写） |
| 已知上游风险（不修补，披露） | ① `Tree::used` 去重表为死代码；② `obstacleFree` 半径搜索 `len²+1` 存在理论漏检窗口（探针 6/6 未触发；本轮返回路径统一复检拦截）；③ 上游构造器对输入错误/起终点在障碍时 `exit()`，适配器先做完整预检 |

## 4. Klamp't / KrisLibrary（连续空间组补源，BSD-3-Clause）

| 项 | 值 |
| --- | --- |
| URL | https://github.com/krishauser/KrisLibrary |
| commit | `f1c9ba76e287d9a3966b1ab3f870fa96e159ef7f`（2025-08-06，master） |
| 许可证 | BSD-3-Clause（Indiana University 2013）；`LICENSE` SHA256 `1fe78dbfd662971b898dfc85b54f36896e7f3051884e7b55c9b2d438631d5421`；`LICENSE.txt` SHA256 `f09bf96e59e8214cf5187ffecc0b9699978f13ce76559df821f1f39f1ac5408c` |
| 归档 | `https://github.com/krishauser/KrisLibrary/archive/f1c9ba76e287d9a3966b1ab3f870fa96e159ef7f.tar.gz`，SHA256 `e1b36dac98ff5df1fda5aa597559b6ff42373d8551bac99d22c358e7c04960e2`（2026-10-12 两次独立下载复算一致，2,158,334 B）；**暂无锁条目**，组件内固定 URL+SHA256，待 wiring 写入依赖锁；支持 `FETCHCONTENT_SOURCE_DIR_SAMPLING_SEARCH_KLAMPT` 离线源码树 |
| 获取方式 | FetchContent 仅拉取源码（`SOURCE_SUBDIR` 指向不存在的子目录，不构建上游工程）；48 文件最小构建清单（探针 Makefile 正式化为 CMake 目标；不含 GLdraw/robotics/meshing/image/ROS/OMPL；`nm`/`strings`/`ldd` 隔离检查见探针） |
| 适配补丁 | **无上游补丁**；仅公共虚接口适配：`CSpace::Sample/IsFeasible/PathChecker/Properties`、手动 `PlanMore()+IsSolved()` 循环、`Srand(seed)`、`MilestonePath` 输出转换 |
| 原生偏差 | ① 原生 `rrt*` = `PRMStarPlanner(rrg=true)`，属 **RRT*/RRG* 家族**：边维护为图式 + 最短路父指针更新，与 OMPL RRTstar 严格单父树语义**不同构**（横比须披露）；② `bidirectional` 默认 true，适配器固定 false 以与 OMPL RRTstar 对齐；③ 必须报告空间 `diameter`（否则连接半径退化为 1，探针实测 0.5 s 无解）；④ 无中间解回调，首次解时间由外层循环测量（与 OMPL 回调口径不同，正式基准需统一）；⑤ 点定位器为库默认（O(n) KNN），正式基准可固定为 KD 树/Ball 树（X-B1 决定）；⑥ 预算末解 = 循环退出时当前最优；固定迭代下逐位可复现（探针验证） |

## 5. vss2sn/path_planning（S-06，栅格采样组，BSD-3-Clause）

| 项 | 值 |
| --- | --- |
| URL | https://github.com/vss2sn/path_planning |
| commit | `d8f9edaeae064ab4665350bf820d1d5ebbbfcc45`（master） |
| 许可证 | BSD-3-Clause；`LICENSE.md` SHA256 `878e9cb40c3bccfaf4b15a4929a635eb4d725e75638bdaa5f1f77e274f4955ed` |
| 归档 | `https://github.com/vss2sn/path_planning/archive/d8f9edaeae064ab4665350bf820d1d5ebbbfcc45.tar.gz` SHA256 `25f9d462c9910c6d193b0f643fb136cb37fdbf685902509355aeecf5819f145e`（单次下载记录；仅作来源核对，本组件使用最小抽取副本） |
| 获取方式 | **最小抽取副本** `third_party/vss2sn/`：`rrt_star.hpp/.cpp`、`planner.hpp`、`utils/utils.hpp`、`utils.cpp`（上游 `include/`、`src/`、`lib/utils/`） |
| 适配补丁 | 种子（成员 `mt19937` + `SetSeed`）、预算/取消（`Plan` 主循环 stop predicate）、首次解回调、迭代计数；见 `third_party/vss2sn/PATCHES.md` 的 diff。最近邻/选父/重连/障碍判定未改。上游 pristine SHA256（对照）：`rrt_star.hpp` `77a0d5eb…`、`rrt_star.cpp` `f0e5f304…`；补丁后 `af8d8e77…` / `f5b6e90f…` |
| 原生偏差 | ① 只支持方阵栅格（非方阵返回 `kInvalidInput`）；② 栅格分辨率 = 统一 `edge_check_step`，边界须为其整数倍（否则 `kInvalidInput`）；③ 格点世界坐标 = `min + i·res`，受阻格点覆盖以格点为中心的封闭方格（原生障碍语义），保守栅格化保证障碍 ⊆ 受阻方格并集；④ 起终点按最近格点吸附，对齐输入端点精确；⑤ 预算耗尽或迭代上限耗尽无解均报 `kTimeout`；⑥ 上游 `Rewire` 为递归实现，大树深度风险由预算限制（披露）；⑦ 连接半径固定 threshold=2 格（上游示例口径）、`max_iter_x_factor=500`（构造默认）；最近邻为 O(N) 线性扫描，101² 开阔场景 1 s 预算下成功与否处于临界（demo 复跑曾出现 timeout，见 §8.9），X-B1 须记录并评估连接半径/点定位配置（变更需登记） |

## 6. ai-winter/ros_motion_planning（S-10，独立原生组，GPL-3.0）

| 项 | 值 |
| --- | --- |
| URL | https://github.com/ai-winter/ros_motion_planning（dev） |
| commit | `a6f21b0e7a5aa714bab204289f7a4835b2148fa4`（2026-03-28） |
| 许可证 | GPL-3.0（`LICENSE` SHA256 `fe3eea6c599e23a00c08c5f5cb2320c30adc8f8687db5fcec9b79a662c53ff6b`） |
| 独立性 | **疑似派生**（← S-06 vss2sn，2019；逐字注释、Node 类、"Heuritic" 拼写、辅助类型与时间线证据见 family_d_probe §3.2）→ 不参与公平横比 |
| 状态 | **适配延期**（本轮不构建、不下载、不链接）：CMake 开关 `PLANNER_SAMPLING_SEARCH_WITH_AIWINTER` 默认 OFF，显式置 ON 会报错并指向本文件；探针已覆盖可行性（桩环境 + 3 类补丁后 4/4 用例稳定）与缺陷证据（停滞死循环、Informed 越界）。后续接入须完成：种子注入、预算/取消与停滞上限、越界/负索引防护、costmap/配置适配、状态区分；GPL-3.0 隔离目标独立编译，保留版权头与许可证全文 |

## 7. 验证证据（run ID）

全部命令经 `tools/task_runner.py exec --component sampling_search --workers 1`；
编译持全局编译锁（`flock .git/planner-runtime/build.lock`）且 `--parallel 4`。
失败运行按 §5.4 保留，重试使用新 run ID 并记录 `retry_of`。

| 步骤 | run ID | 结果 |
| --- | --- | --- |
| 构建 1（首次 configure+build，全适配器） | `20261012T125116576198Z_sampling_search_85ab2141` | **失败保留**：`common::Point2d` 成员为 `x_m/y_m`（适配器误用 `.x/.y`）与 Ch0p1k3 适配器缺 pugixml 头路径；修复后重试 |
| 构建 2（重试构建 1，成功） | `20261012T125556171514Z_sampling_search_34b4075d` | success：OMPL 1.6.0 + Klamp't 48 文件 + Ch0p1k3（pugixml 1.16）+ vss2sn 全部编译，demo/tests 链接通过 |
| 测试 1（ctest -L sampling_search） | `20261012T125615034412Z_sampling_search_082b59aa` | **失败保留** 7/34：测试夹具与后端名不匹配、OMPL 近似解误判成功、OMPL seed 0 被忽略、起终点重合平凡情形；修复后重试 |
| 构建 3（修复后增量构建） | `20261012T125831623939Z_sampling_search_42784f1b` | success |
| 测试 2（重试测试 1） | `20261012T125843716816Z_sampling_search_f6fa1152` | success：34/34 通过（中间证据） |
| 演示自检（100×100 开阔，1 s） | `20261012T125917596352Z_sampling_search_acf77564` | success；但暴露 vss2sn 101² 场景临界（见 §8.9） |
| 构建 4（OMPL 元数据补充后） | `20261012T125942574157Z_sampling_search_fd97e424` | success |
| 测试 3 | `20261012T125949817951Z_sampling_search_98aa786f` | success：34/34 |
| 构建 5（OMPL 统计位置调整后） | `20261012T130131728012Z_sampling_search_0e533d2a` | success |
| **测试 4（最终源码）** | `20261012T130144209404Z_sampling_search_b6424fcb` | **success：34/34 通过（最终正确性证据）** |
| 演示自检（复跑，vss2sn 101² timeout） | `20261012T130217803107Z_sampling_search_4b39b86c` | **失败保留**：vss2sn 1 s 预算未解（性能临界证据，见 §8.9） |
| 构建 6（demo 场景改为 50×50 对齐） | `20261012T130352822656Z_sampling_search_50b345f7` | **失败保留**：demo 编辑误用 `.x/.y`；修复后重试 |
| 构建 7（重试构建 6） | `20261012T130417837261Z_sampling_search_06f60e61` | success |
| **演示自检（最终）** | `20261012T130445400115Z_sampling_search_685ba6d2` | **success**：5 个后端全部求解（cost 56.57–56.59，50×50 开阔，1 s） |
| 演示表格（50×50 障碍） | `20261012T130456818922Z_sampling_search_c28073c7` | success：全部求解；cost 60.33–65.94，首次解 0.0000–0.022 s |
| 独立开关验证（OMPL=OFF、Klamp't=OFF 的 standalone 构建 + 测试） | `20261012T130539588130Z_sampling_search_32cb854a` | success：34/34（6 个 OMPL/Klamp't 用例如实 Skipped）；`_deps` 无 ompl/klampt 目录，证明未启用不下载不链接 |
| X-B1 基准工具构建（bench 目标，最终二进制） | `20261012T135307472551Z_sampling_search_eda2798a` | success（硬超时 kill 回退 + `--help` 退出码修复后） |
| **X-B1 烟测（46 trial，每 trial 独立子进程；含容量档路径）** | **`20261012T135320409529Z_sampling_search_f8de5fdc`** | success：46/46 子进程、46/46 记录、0 SHA 失配、0 跳过，`harness.ok=true` |

测试用例构成（34 个 CTest 条目，来自 6 个 gtest 文件）：
- 核心契约 10：输入校验 4、沿边检查 3、路径助手 2、注册表一致性 1；
- 统一契约 9：成功边有效性/端点、探针场景（连续组）、多种子成功率（5 种子，
  要求 ≥4）、不可达报 timeout、预取消、线程取消、无效输入、起终点重合、统计；
- 适配器专项 15：OMPL 3（双变体、长度目标提前结束、不可达 timeout）、
  Ch0p1k3 4（探针场景、原生参数披露、非整数边界拒绝、取消）、
  Klamp't 3（探针场景、RRG* 披露、预算）、vss2sn 5（格点场景、栅格配置披露、
  非方阵拒绝、非整数倍拒绝、取消）。

## 8. 遗留风险与后续（X-S4 范围内）

1. **OMPL 归档稳定性**：构建按锁 `URL_HASH` 校验；若上游重新打包导致不匹配，
   按实际字节记录并交由 wiring/集成决定（不静默放宽）。
2. **Klamp't 锁条目缺失**：当前用组件内固定 URL+SHA256；wiring 阶段写入
   `dependencies.lock.json`（含许可证哈希与 48 文件清单）。
3. **Ch0p1k3/vss2sn 抽取副本**：wiring 阶段须把抽取文件级 SHA256、许可证与
   补丁写入锁/来源记录；本轮已在 §3/§5 与 `PATCHES.md` 记录。
4. **保守栅格化残余缺口**：Ch0p1k3/vss2sn 的障碍映射对"完全位于单元内部且
   不触碰任何采样点"的细障碍存在理论漏检；与夹具自身的沿边采样分辨率同源。
   返回路径统一复检，失败按无解处理；X-B1 大场景复核时须保留该披露。
5. **首次解口径**：OMPL 用中间解回调、Klamp't/Ch0p1k3 用外层循环、vss2sn 用
   补丁回调；口径差异须在 X-B1 正式基准统一（本轮只记录）。
6. **Klamp't RRG* 语义**：与 OMPL RRTstar 不同构，横比报告必须披露；若要求
   严格单父树语义需另立评估。
7. **ai-winter**：适配延期（§6）；独立性结论建议发布前用代码搜索服务复核。
8. **基准与容量档**：本轮无性能/容量档结果；按 benchmark_tiers §2.4/§3 由
   X-B1 执行（预算 0.1/0.5/1/5 s × 种子 0–29；容量档 2048²/4096² 与放大连续场景）。
9. **vss2sn 性能临界**：101² 开阔场景 1 s 预算下成功与否随实际迭代数波动
   （墙钟截断 + O(N) 最近邻）；50×50 对齐小场景在 0.5 s 内稳定求解（测试
   34/34 通过）。X-B1 须在正式矩阵中如实记录成功率与超时，不得以调参隐藏。
10. **OMPL 进程级种子**：`RNG::setSeed` 为进程全局单例，进程内首次使用后
    再播种被忽略；正式基准须每 trial 独立进程（或每进程只跑一个规划器），
    否则同进程后续 trial 不可复现（见 §2 原生偏差）。

## 9. X-B1 基准工具登记（2026-10-12 收口补充）

- 基准工具：`benchmarks/`（连续/栅格场景生成器 v1 + 每 trial 独立子进程运行器 +
  容量档列；注册片段已并入 `tools/components.json` →
  `components.sampling_search.benchmark_suites.v1`）；用法、矩阵与指标列见
  `benchmarks/README.md`。正式入口 `benchmark --component sampling_search
  --suite v1`（独占窗口）。
- 烟测 run **`20261012T135320409529Z_sampling_search_f8de5fdc`**：46/46 子进程、
  46/46 记录、0 SHA 失配、0 跳过，`harness.ok=true`（含容量档代码路径）；
  retry 链 `…c59f2a02` → `…2fd57cb1` 与首轮 `…73b7fb6d`（CSV 列数缺陷）
  按 §5.4 保留（明细见 benchmarks/README §7）。
- **OMPL 进程级种子方案（已落地）**：每 trial 独立子进程（`/proc/self/exe
  --single ...`），子进程只创建一次后端并调用一次 `plan()`；探针
  `20261012T134856320142Z_sampling_search_cec845b1` 验证同种子跨进程末解逐位
  一致（OMPL 113.14062321619376 ×2、Informed 113.13708498984485 ×2），迭代数/
  改进次数因墙钟预算截断有 ±0.1% 量级波动（时间预算语义，非播种失败）。
- **vss2sn 性能临界（如实记录）**：烟测中 128² open/maze、0.1/0.5 s 全部 timeout；
  与 §8.9 登记一致，正式矩阵不得以调参隐藏成功率/超时。
- **Ch0p1k3 窄道保守栅格化观察**：2.0 m 窄道场景 timeout——其 1 m 保守栅格化
  把通道两侧单元判为受阻（原生障碍映射语义），正式矩阵如实记录、不修补语义；
  容量档 2048² open 单次墙钟约 0.8 s、峰值 RSS 约 62 MiB（含保守栅格化）。
- 正式矩阵（连续组 2,880 + 栅格组 2,160 + 容量档 20 = 5,060 trial）与容量档属
  X-B1 独占窗口，不在本轮执行。
