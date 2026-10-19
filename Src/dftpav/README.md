# planner_dftpav：Dftpav 独立实验优化后端

> 状态：端到端链路交付。优化器核心由上游 Dftpav 固定版本抽离（GPL-3.0，
> 见 [PROVENANCE.md](PROVENANCE.md)）；对外有效轨迹必须通过
> `planner::validation`（V）校验；Hybrid A* `ros_free` 输出（公共
> `PathSegment`）可作为初值；公共安全走廊（`planner::common::Corridor`，
> planner::corridor 的输出）直接消费。

## 1. 功能与接口

- 公共头：`include/planner/dftpav/optimizer.hpp`
  - 输入：预制初始路径（`std::vector<planner::common::PathSegment>`，含前进/
    倒车分段，即 Hybrid A* ros_free 的输出类型）、公共走廊
    （`planner::common::Corridor`）、车辆模型、动力学限制、目标位姿
    （`goal`，缺省取路径末点位姿）与地图快照（`collision::Scene`，缺省为
    无栅格自由空间）。
  - 输出：`OptimizeResult`，含 `TrajectoryPoint` 时序轨迹、状态
    （`common::Status`）、稳定失败原因、`planner::validation::ValidationResult`
    判定与统计量。
- 时序轨迹语义：时间严格递增；速度为有符号量，换向处速度为 0（默认
  `direction_switch_velocity_mps = 0.0`）；加速度为纵向加速度；曲率满足
  `yaw_rate = velocity_mps * curvature_inv_m`；速度低于
  `curvature_report_min_speed_mps`（默认 0.24 m/s，对应上游速度奇异尺度）时
  报告曲率 0：MINCO 在起停边界（v=0、a=0）的极限曲率发散，该尺度以下的转向
  量在数值上无意义（上游 `non_sinv` 用同一尺度回避奇异区）。
- 失败原因（`OptimizeResult::reason`）：
  - 输入：`invalid_vehicle`、`invalid_limits`、`invalid_options`、
    `invalid_seed`、`empty_corridor`、`invalid_corridor`、`map_mismatch`、
    `corridor_coverage`；
  - 优化器：`solver_failed`、`cost_too_high`；
  - 取消（移植扩展）：`cancelled`（`common::Status::kCancelled`；仅由
    `OptimizeOptions::cancel_check` 触发，`trajectory` 为空）；
  - 临时守卫（方向相关的动力学诊断）：`provisional_velocity_limit`、
    `provisional_acceleration_limit`、`provisional_curvature_limit`、
    `provisional_lateral_acceleration_limit`、`provisional_direction_change_stop`、
    `provisional_non_finite`、`provisional_time_order`；
  - V 判定（权威，取 V 约束名并加前缀）：`validation_collision`、
    `validation_corridor_containment`、`validation_speed_limit`、
    `validation_curvature_limit` 等。
- 失败状态对相同输入确定性可复现；成功轨迹逐位确定。
- **协作式取消（移植扩展，登记于 PROVENANCE.md §2）**：
  `OptimizeOptions::cancel_check`（可选 `std::function<bool()>`，默认空 =
  永不取消）在求解前、L-BFGS 迭代边界（移植核心的代价回调与进度回调，以
  `LBFGS_CANCELED` 退出）、求解后修复边界与验证前被轮询。取消一旦被观察到，
  立即返回 `kCancelled` + `reason="cancelled"` 且 `trajectory` 为空：
  **绝不返回部分轨迹**，也绝不放宽 V/碰撞门禁（未判定的中间迭代一律丢弃，
  重试阶梯不再启动）。不设置该选项时行为与既有版本逐位一致。谓词由调用
  线程轮询，需非阻塞、不抛异常且线程安全（典型为读取 `std::atomic<bool>`）。

## 2. Validation 门禁与集成缓解

- **有效性由 V 决定**：每次尝试对采样轨迹调用
  `planner::validation::validate_trajectory`（同一车辆/限制/目标/场景/走廊，
  V 的固定容限，计划 §7.1）；只有 V 通过（且方向相关临时守卫通过）才返回
  `kSuccess`。V 的硬约束在适配层从不放宽。限制映射：V 每轴单值，速度/加速度/
  曲率取前后向较大者；方向相关限制由临时守卫精确执行。
- **软约束残差处理（收紧+重试）**：上游用平滑惩罚实现约束，活动约束上会留下
  残差。适配层确定性重试阶梯（限制/走廊残差类最多 4 次尝试；首次即碰撞失败
  时另加最多 3 次碰撞定向尝试，见第 5 条）：
  1. 内部动力学限制收紧（速度/加速度 5%，曲率 1%——Hybrid A* 路径常用满舵，
     曲率收得太紧会不可行）；
  2. 车体几何收紧 δ（0.01→0.03→0.05 m）：优化器车体 = 车辆保守足迹 + δ，
     V 检查的是保守足迹，因此 δ 吸收残差（V 足迹裕量 = δ − 残差）；
  3. 预制初值锚定（`anchor_weight`，默认 2000）：把内层路径点与换向点二次
     惩罚到初值，保持轨迹贴近预制路径（走廊区间按种子弧长参数化，轨迹漂移会
     使 V 的区间关联错位）；
  4. **细分段尝试（Wave 9）**：最后一次尝试把初值分段时间减半
     （`traj_piece_duration_s` × 0.5），denser 初值能贴紧紧走廊预制路径
     （1 s 网格在顶点处有分米级切角，事后修复无法恢复）；该尝试放在最后，
     已能通过粗网格的用例保持原轨迹。
  5. **碰撞驱动的定向重试（hybrid-006）**：仅当首次尝试即被 V 以
     `validation_collision` 拒绝（贴墙种子 × 优化偏离）时追加有界、确定性
     的尝试阶梯：0.5x 网格 + 2x 锚定 → 0.25x 网格 + 内部曲率收紧 1%→2%
     → 0.125x 网格 + 4x 可行性权重 + 2x 约束采样分辨率。每次尝试仍走
     完整 V 判定；全部失败时保留原失败结果（原因/详情/统计），绝不返回
     部分结果，也绝不放宽任何门禁（见 §6）。
- 已尝试且**无效**的杠杆（记录在案）：加大走廊权重（10×/100×）、单独加倍约束
  采样分辨率、加大锚定（过强时曲率不可行）——都会把优化器带入不同（通常
  更差的）局部极小。细分段仅在**重试**中启用（粗网格本身仍是默认起点）；
  碰撞驱动的定向重试（第 5 条）只在与细分段/可行性权重组合时才使用加倍
  分辨率（单独加倍对贴墙段无效）。
- **Wave 9（dftpav-002）结点空间可行恢复**（`corridor_repair`，默认开启）：
  软约束解在活动走廊约束上的残差为 1e-4–1e-3 m，V 容限 1e-6 m。恢复在
  MINCO 结点空间（内层结点位置 + 换向点位置，边界状态不变）做确定性硬约束
  可行化：
  - 关联与 V **逐项同款**：同样的存储采样网格/航向策略、同样的
    `chord·sinc(Δθ/2)` 弧长累积、同样的相邻存储点圆弧重建与 0.02 m 中间
    位姿（只查存储点会低估 V 实际检查的越界）；
  - 每个越界位姿给出一条线性化半平面约束
    `nᵀ(p + J δx − a) ≤ −(violation − target)`，`J` 为有限差分位置雅可比
    （中间位姿按两侧存储点雅可比线性混合）；
  - 循环投影求最小范数修正，回溯阻尼 + 外层迭代；评价函数
    `merit = max(包含残差, 曲率超限)`，避免用曲率违规换包含；边界相邻
    结点修正加权 0.25，防止在固定边界附近制造曲率尖峰；
  - 预处理：先评估向预制初值的混合（α∈{0.25,0.5,0.75,1}），取首个
    **无曲率超限**的最小混合作为起点（优化器局部极小可能在边界层带曲率
    残差，位置修正无法去除）；
  - 目标内裕量 −1e-4 m（只向内收紧，V/容限/安全裕量从不放宽）；统计见
    `corridor_repair_iterations` / `corridor_repair_worst_violation_m`。
    轨迹本已在走廊内（且曲率可行）时为 no-op。
- `heading_report_min_speed_mps`（默认 0.02）：存储航向在 |v| 高于该尺度时
  取解析运动方向，仅在真正静止时回退到携带/种子航向；若沿用曲率奇异尺度
  （0.24），加速穿越阈值时会产生航向台阶，V 会把它读成相邻存储点间的
  微小半径圆弧（中间位姿甩离真实路径）。
- 临时守卫只保留 V 无法表达的方向相关检查（逐方向速度/加速度/曲率、横向
  加速度 |v²κ|、换向停车）；走廊包含完全交给 V（V 用轨迹自身弧长关联，比
  适配层投影关联更权威）。

## 3. 构建与测试（只经统一入口）

```sh
python3 tools/task_runner.py build --component dftpav --profile release --parallel 4
python3 tools/task_runner.py test  --component dftpav --workers 1
```

- 库目标 `planner_dftpav`（别名 `planner::dftpav`），链接 `planner::common`、
  `planner::corridor`、`planner::collision`、`planner::validation`；测试 label
  `dftpav`。测试/示例另用 `planner::hybrid_astar`（端到端链路初值）。
- 依赖回退：若 `planner::validation` / `planner::hybrid_astar` 目标缺失，
  组件 CMake 会按各自组件的既有模式 `add_subdirectory` 引入（与 validation
  组件对 collision/corridor 的处理一致），因此从空构建目录执行
  `build --component dftpav` 也能工作。
- 系统 Eigen3（3.3/3.4 系列头文件）是唯一外部头文件依赖（上游优化器核心需要）；
  锁定参考版本为 `dependencies.lock.json` 的 `eigen` 条目（3.4.0），离线复现按该
  版本系列准备头文件；构建期不下载上游 Dftpav，不使用 ROS/OMPL/PCL/protobuf。

## 4. 端到端链路（Hybrid A* → 走廊 → Dftpav → V）

测试 `tests/test_e2e_chain.cpp` + `tests/dftpav_e2e_test_support.hpp` 消费共享
v2 夹具（`tests/fixtures/v2/paths/cases.json` 与 `vehicles/default.json`）：

| 用例 | 场景 | 结果 |
| --- | --- | --- |
| `OpenDirectSucceeds` | `open_101` 空旷直达 | Hybrid A* 路径 → 生成走廊 → 优化 → **V 通过** |
| `NarrowCorridorBetweenObstaclesSucceeds` | `corridor_111` 3 m 窄通道（两侧障碍墙） | 生成走廊直连，**V 通过** |
| `ReverseParkingSucceeds` | `deadend_124` 死胡同倒车（1 次换向） | 刀口走廊回退（局部车道走廊 0.8 m），**V 通过**，换向 |v|≤1e-3 |
| `CenterlineObstacleDetour106/109/110Succeeds` | `obstacle_106/109/110` 中心线单障碍 | 预制平滑绕行种子（fillet R=5.5，κ≤0.182）+ 生成走廊，**V 通过** |
| `TightCenterlineObstacleDetour106/109Succeeds` | 紧贴顶点版本（`obstacle_106` 顶点 7.8、`obstacle_109` 顶点 9.0；保守足迹距障碍 ≈0.18 m） | 粗网格残余 1.9 mm/59 µm → 细分段重试 + 结点空间恢复 → **V 通过**（残差 ≤0，`riter` 1–4） |
| `SmallFoldSeedsRosFreePassValidationButRemainInOutput` / `SmallFoldSeedsKthPassValidationButRemainInOutput` | `obstacle_107/109`、`deadend_122/125` 的原始 Hybrid A* 初值（hybrid-004 小折返；`ros_free` 与默认后端 `kth_normalized`）× 车道回退 0.8 m | 全部 **V 通过**；含折返初值的组合优化输出保留折返（显著非共线横切 1 处/例，横切段从初值 0.35–2.0 m 缩小到约 0.07–0.13 m；kth 的 obstacle 初值本无折返） |
| `SmallFoldSeedsSmacKnownState` / `SmallFoldSeedsZm0612KnownState` | 同四场景 × `smac_normalized`/`zm0612_normalized`（车道回退 0.8 m） | **全部 V 通过**且折返状态保持（hybrid-006 修复：贴墙种子的碰撞由碰撞驱动的定向重试消除，见 §6；V/碰撞门禁未放宽） |
| `CollisionRetryIsDeterministic` | `zm0612_normalized` × `deadend_125`（走满碰撞重试阶梯） | 同输入两次链路轨迹逐位一致（4 次尝试） |
| `ValidationRejectsCollision` | 走廊无该障碍、场景含路径障碍 | 优化器直行，V 判定 `validation_collision`，不返回有效 |
| `EmptyCorridorIsRejected` | 清空走廊 | `invalid_input`/`empty_corridor` |
| `CurvatureLimitBelowRequirementFails` | 0.02 1/m 曲率上限（路径需 ≈0.195） | V 拒绝（曲率或走廊），不返回有效 |
| `IsDeterministic` | 同输入重复链路 | 轨迹逐位一致 |

- 走廊生成失败时使用按种子保守足迹构造的局部车道走廊回退（测试内构造，
  `corridor_fallback=true` 标记），回退构造与原因在报告中记录。
- **换向点语义（docs-002 更正）**：hybrid-003 复核（4 个 normalized 后端 ×
  park_116–120、deadend_122/125 共 28 例）`max_gap ≤ 1e-9` 全部通过，可见行
  均为 `0.000e+00 m`，叠加既有 `PathQuality` 7 例共 35 条零断口证据——旧记载的
  “0.3–2.0 m 断口”不再复现（属陈旧构建/消费端语义）。消费端约定：**换向姿态
  在前后两段各出现一次（同一 `s_m`、方向字段相反）**；消费方按方向字段分段，
  不得把该重复姿态当作零长段或回环。适配层按此约定消费：`build_seed_line`
  在重复姿态处不累加弧长（几何弧长连续），换向点由优化器的共享换向变量统一。
- V 证据：链路测试在成功用例中断言 `result.validation.valid`、轨迹点数、
  走廊采样数与碰撞检查计数；证据位于 `artifacts/runs/<run_id>/`（stdout.log、
  junit.xml、metrics.csv），run ID 见交付报告。

## 5. 命令行示例

```sh
build/release/Src/dftpav/planner_dftpav_example --scenario straight
build/release/Src/dftpav/planner_dftpav_example --scenario reverse
build/release/Src/dftpav/planner_dftpav_example --scenario avoid
build/release/Src/dftpav/planner_dftpav_example --scenario infeasible
build/release/Src/dftpav/planner_dftpav_example --scenario empty-corridor
build/release/Src/dftpav/planner_dftpav_example --self-check
build/release/Src/dftpav/planner_dftpav_example --scenario straight --output /tmp/straight.json
```

示例场景为预制路径（非 Hybrid A* 链路）；Hybrid A* 链路见第 4 节测试。

## 6. 已知限制与遗留风险

- **软约束残差（Wave 7 结论 → Wave 9 缓解）**：活动走廊约束上的平滑惩罚残差
  为 1e-4–1e-3 m，而 V 对保守足迹的包含容限是 1e-6 m。Wave 9 已实现路径 (a)
  “结点空间 + V 同款弧长关联的硬约束可行恢复”（见 §2），紧贴顶点用例
  `obstacle_106` 顶点 7.8（粗网格 V 首违 1.9 mm、轨迹自身关联最大越界约
  0.39 m）与 `obstacle_109` 顶点 9.0（首违 59 µm）现在 **V 通过**
  （恢复后 `rworst ≤ 0`，`riter` 1–4）；关键是细分段重试把粗网格的分米级
  切角压到毫米级，恢复再做剩余可行化。剩余限制：恢复是事后修复，
  优化器内部的走廊约束仍按**初值参数化**关联（时间→站位），轨迹定时漂移
  仍会错位；根治方案仍是把关联改为轨迹自身弧长（成本函数内的非线性改动，
  超出本波次范围）。另：恢复只在包含/曲率上可行化，碰撞类失败（如刀口
  走廊）仍由回退走廊处理；不得单方面放宽 V。
- **Hybrid A* 换向点（docs-002 更正）**：换向断口**不复现**（hybrid-003：28/28
  `max_gap ≤ 1e-9`，可见值 `0.000e+00 m`；另有 7 条 `PathQuality` 零断口证据）。
  消费语义见 §4：换向姿态前后段各出现一次（同 `s_m`、方向相反），按方向字段
  分段，重复姿态不是零长段/回环。适配层仍保留 ≤0.75 m 的换向点吸附作为防御性
  输入卫生（当前后端不会触发）。残余：部分后端在 `obstacle_107/109`、
  `deadend_122/125` 出现每例 ≤1 处非共线小回环（`hybrid-004`，契约有效、
  无碰撞）；**定向核查（run `20261010T182505541183Z_dftpav_2c829c44`）表明
  优化/平滑阶段不消除该折返**：含折返初值且 V 通过的组合，输出仍保留显著
  非共线横切（每例 1 处，横切段缩小到约 0.07–0.13 m）；曾因贴墙被 V 拒绝的
  3 个死胡同组合已由 hybrid-006 修复（见下），回归见 `SmallFoldSeeds*`，
  `hybrid-004` 已按评估闭环（折返为
  通过性机动、非缺陷；三条消除路线均不采纳，见 hybrid 组件 README）。
  `obstacle_108` 目标足迹距地图边界
  0.1059 m（含 0.1 m 安全裕量），合法非缺陷。
- **hybrid-006（smac `deadend_125`、zm0612 `deadend_122/125` 的 V 碰撞拒绝）——已闭环（碰撞驱动的定向重试，Wave 10）**：
  分层定向核查（临时诊断用例，未入库；证据 run `20261011T021555211694Z_dftpav_1f42015d`
  （折返/同弧长站位对比）、`20261011T020956241634Z_dftpav_e5579b2b`（分层 + 生成器选项）、
  `20261011T021352972619Z_dftpav_d9f7d07a`（回退宽度/分段时间扫描））定位根因：
  - **种子（a）**：三个组合的初值经 `planner::collision` 检查器（0.1 m 安全裕量足迹，
    存储位姿 + 运动扫掠）全部 `clear`（合法），但贴墙：对墙体障碍的最小间隙为 smac
    `deadend_125` **0.0105 m**、zm0612 `deadend_122` **0.0006 m**、zm0612 `deadend_125`
    **0.0828 m**（V 通过的 ros_free/smac `deadend_122`、ros_free `deadend_125` 均为
    0.200 m）；
  - **走廊（b）**：0.8 m 车道回退不是原因（宽度扫 0.15–0.6 m、真实生成器选项均不能
    使失败组合通过；生成器默认选项对这三个种子 `blocked`）；
  - **优化（c）**：碰撞来自优化轨迹偏离初值（同弧长站位横向 1.2–5.2 cm、航向至 6.3°，
    扫掠穿透贴墙段；存储位姿均 `clear`）；smac125/zm125 初值无折返、zm0612_122 折返在
    s≈12 而失败在 s=2.3，与 hybrid-004 折返平滑无关。
  - **修复（适配层重试阶梯，未触碰移植上游数学）**：`OptimizeOptions::collision_retry`
    （默认开启）在首次 V 判定为 `validation_collision` 时按“改动从小到大”追加有界、
    确定性的尝试，每次仍走完整 V 判定：
    ① 0.5x 初值分段 + 2x 锚定 → zm0612 `deadend_122`（第 2 次尝试通过）；
    ② 0.25x 分段 + 内部曲率收紧 1%→2% → smac `deadend_125`（第 3 次）；
    ③ 0.125x 分段 + 4x 可行性权重 + 约束采样 32/64 → zm0612 `deadend_125`（第 4 次）。
    结果：三个组合全部 V 通过且折返状态保持（zm0612 `deadend_122` 输出保留 1 处显著
    非共线横切；smac/zm0612 `deadend_125` 无自交）；回归 `SmallFoldSeedsSmac/Zm0612KnownState`
    改为要求 V 通过（原 `expect_validation_rejects` 删除），新增
    `CollisionRetryIsDeterministic`；阻塞性反例 `ValidationRejectsCollision` 在全部阶梯
    尝试失败后仍返回原 `validation_collision`（门禁未放宽、无部分结果）。
    证据：分层核查 run 见上；修复参数扫描 run `20261011T064941134720Z_dftpav_22e32bb8`
    （网格/锚定/曲率/可行性/分辨率扩展扫描）与 `20261011T070701436823Z_dftpav_a2c41a17`
    （杠杆隔离）；修复后复验 run `20261011T075724823600Z_dftpav_730f3b55`（三组合逐次
    尝试与折返计数，全部 V 通过）；全量回归 test `20261011T083659306431Z_dftpav_e608f9e2`
    （40/40）；hybrid 侧未修改，回归 `20261011T084424753143Z_hybrid_astar_17288de0`
    （43/43）。
  - **剩余限制**：候选阶梯为实测固定参数（非通用障碍感知）；内部曲率收紧超过 2% 时
    曲率残差反向增大（实测），故上限保持 2%；关联/跟随的根本增强（障碍感知走廊、
    轨迹自身弧长关联）仍为后续方向。
- **刀口走廊**：tight 口袋内生成的走廊对 1° 航向扰动即产生厘米级越界
  （`deadend_121` 起点 5 cm）；这些夹具改用局部车道走廊回退。
- **锚定项**是移植代码的少量扩展（上游无此项），用于保持轨迹与预制路径的
  站位一致性；权重过强会与曲率可行性冲突，默认 2000 为实测折中。
- **协作式取消**同为移植扩展（上游无此项）：移植核心原有的进度回调机制被
  接入 `lbfgs_optimize`，并在代价回调/进度回调中轮询可选的取消谓词
  （见 §1 与 PROVENANCE.md §2）。它只影响提前退出与返回状态，不改变数学；
  未设置 `cancel_check` 时与旧行为逐位一致。
- **Eigen 依赖（`dftpav-001` 闭环，文档化决定）**：Eigen 已作为 `eigen` 条目
  （3.4.0）锁定在 `dependencies.lock.json`；本组件继续以系统 Eigen3 头文件作为
  构建提供者，离线复现按锁条目版本系列（3.3/3.4）准备，不再需要另做入锁处理。
- **components.json**：`dftpav` 条目当前只启用 `PLANNER_BUILD_DFTPAV` 与
  `PLANNER_BUILD_CORRIDOR`；建议补上 `PLANNER_BUILD_VALIDATION`、
  `PLANNER_BUILD_HYBRID_ASTAR`、`PLANNER_BUILD_COLLISION`（当前依赖 CMake
  回退可用，但注册表应显式声明）。
