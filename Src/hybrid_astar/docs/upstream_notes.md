# Hybrid A* 上游固定与对照笔记（Wave 4–6：调研固定与 7 后端交付）

本文件记录 Wave 4 调研时固定的三组上游版本、许可证、搜索结构与移植评估，
以及 Wave 5/6 按 §6.3 契约交付的 7 个后端（1 自研 normalized + 3 上游
native + 3 上游 normalized）。
上游完整源码克隆在仓库外的 `/tmp/opencode/upstreams/`，不进入本仓库；本仓库
不复制上游代码，只按概念重写（见各条“本项目采用方式”）。

## 1. 固定版本清单

| 上游 | 克隆目录 | 固定 commit | 提交日期 | 许可证 |
| --- | --- | --- | --- | --- |
| [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2)（`nav2_smac_planner`） | `/tmp/opencode/upstreams/nav2`（sparse：`nav2_smac_planner/`） | `bcc9c48cf468588559d01942b72d511a6b990891` | 2026-10-09 | 包内 `package.xml`: Apache-2.0；仓库含 LGPL-2.1-or-later 与 Apache-2.0 混合声明 |
| [karlkurzer/path_planner](https://github.com/karlkurzer/path_planner) | `/tmp/opencode/upstreams/kth_path_planner` | `ddf1a3afcafc323ef44d5e49d5cbb7b3c3f53665` | 2026-03-29 | BSD-3-Clause（`LICENSE.txt`，Copyright (c) 2017 Karl Kurzer） |
| [zm0612/Hybrid_A_Star](https://github.com/zm0612/Hybrid_A_Star) | `/tmp/opencode/upstreams/zm0612_Hybrid_A_Star` | `cf0cc997f285e05a1ff51f36b224d3040fc7708c` | 2024-06-16 | BSD-3-Clause（`LICENSE`，Copyright 2022 Zhang Zhimeng） |

固定方式：克隆后记录 `git rev-parse HEAD`；本波次不引入任何上游代码或构建依赖。
若后续适配器需要少量借鉴，必须在该适配器源码头部注明来源仓库、commit、许可证与
改写范围；Apache-2.0 文件保留 NOTICE 义务，BSD-3-Clause 保留版权声明与免责声明。

## 2. 对照要点

### 2.1 Nav2 Smac（`nav2_smac_planner`）

- 搜索结构：模板化标准 A*（`a_star_impl.hpp`），节点类型（2D / Hybrid / Lattice）
  提供邻居、代价与启发式；`NodeHybrid` 使用运动原语图；`AnalyticExpansion` 做
  Reeds-Shepp 终端连接；`GoalManager` 管理多目标与近似目标；主循环硬上限
  `max_iterations`（对比文档 §7.2 的 1,000,000 节点预算）。
- 碰撞/RS 依赖：`collision_checker.cpp` 基于 costmap footprint 检查与预计算
  footprint cost；解析连接复用同一碰撞检查。ROS/Nav2 依赖重（rviz、pluginlib、
  bt 行为树、costmap_2d、tf2 等）。
- 关键参数（README 默认）：`angle_quantization_bins: 64`、
  `analytic_expansion_ratio: 3.5`、`analytic_expansion_max_length: 3.0` m、
  `minimum_turning_radius`、`reverse_penalty: 2.1`、`change_penalty: 0.0`、
  `non_straight_penalty: 1.20`、`cost_penalty: 2.0`、`retrospective_penalty: 0.025`、
  `cache_obstacle_heuristic`（同目标重规划热启动缓存）。
- 启发式：动态规划障碍启发式（costmap DP 距离场，可缓存），面向通行带居中的
  `cost_penalty` 加权；路径平滑已移除，改为搜索内代价惩罚。
- 移植难度：高（ROS 2 框架耦合广、配置量大），但搜索/启发式/解析扩展的分层
  设计是本项目 normalized 后端的结构参考；仅作性能基线与机制参考。

### 2.2 KTH path_planner

- 搜索结构：经典 Hybrid A*（`algorithm.cpp::hybridAStar`），离散运动原语 +
  `Node3D` 状态格；`Constants::iterations = 30000` 上限；`dubinsShot` 解析连接
  （仅前进 Dubins，`dubinsStepSize = 1` m）；2D A*（`algorithm.cpp::aStar`）产生
  带障碍的距离启发式并在扩展时更新。
- 碰撞/RS 依赖：`collisiondetection` 基于栅格查找表（`positionResolution = 10`）；
  无 RS（Dijkstra 2D + Dubins shot）。ROS 1 / Boost / OpenCV 耦合。
- 关键参数：`headings = 72`、`cellSize = 1` m、`penaltyTurning = 1.05`、
  `penaltyReversing = 2.0`、`penaltyCOD = 2.0`、`dubinsShotDistance = 100`、
  `tieBreaker = 0.01`、`factor2D` 启发式放大因子。
- 移植难度：中（ROS 仅在外围订阅/可视化，搜索核心集中；Boost heap 与
  OpenCV 可视化可剥离），缺少倒车解析连接是其与本项目 RS 后端的主要差别。

### 2.3 zm0612/Hybrid_A_Star

- 搜索结构：单一 `HybridAStar` 类，RS 解析扩展（`rs_path.cpp` 自带实现），
  离散转向/步长（`steering_angle_discrete_num`、`segment_length_discrete_num`），
  `grid_size_phi = 72`，`shot_distance` 内尝试 RS 连接；代价项包含转向/倒车/
  转向变化惩罚；`trajectory_optimizer` 做后处理平滑（默认本项目不启用）。
- 碰撞/RS 依赖：costmap 订阅 + `CheckCollision`/`LineCheck` 遍历；Eigen、glog、
  ROS 1 依赖；无需 OMPL。
- 移植难度：中（类结构小、RS 内置，但碰撞基于 costmap 订阅且使用 Eigen/glog，
  需替换为公共碰撞与 RS 接口）。

## 3. 本项目采用方式（Wave 5/6 已交付）

- 共享搜索核心：所有后端共用同一 ROS-free 核心（离散状态 A* + 运动原语 +
  RS 解析连接），正式接入 `planner::rs`（解析连接 + RS 距离启发式）与
  `planner::collision`（整车运动扫掠检查）；换向点保留为 PathSegment 边界，
  且换向姿态按冻结契约**同时**作为前段末点与后段首点（hybrid-001 已修复）。
- 已交付 7 个后端（`src/backend.cpp` profiles）：`ros_free`（自研 normalized
  基线）；Smac / KTH / zm0612 各一 `native`（按固定 commit 的搜索结构/参数
  ROS-free 重写，仅参考，未复制上游代码）与一 `normalized`（正式组件 +
  冻结调优参数组，参与选型）。
- 上游参数名以别名映射到核心键（如 `angle_quantization_bins→heading_bins`、
  `minimum_turning_radius→min_turn_radius_m`、`cellSize→xy_resolution_m`、
  `penaltyTurning→turn_penalty`、`shot_distance→analytic_shot_distance_m`、
  `iterations→default_node_budget`）；无法映射的键记入 `unsupported` 并附
  原因，设置即 `invalid_input`，不静默忽略。
- 已记录偏差：KTH 去重网格 0.5 m（上游 cellSize 1.0 m + 同格 tie-breaker）、
  KTH Dubins 前向 shot 由完整 RS 解析替代、Smac 回顾/二次代价/原语插值未
  实现、上游 costmap/Eigen/glog/ROS 依赖由正式组件替代。
- 基准：`planner_hybrid_astar_bench --suite v1`（5 预热 + 30 测量、冷/热、
  5 s/1e6 预算）已产出 `selection.json`，按 §7.2 完整规则给出
  `qualified_candidates`/`default_backend`/`pending_optimization`；正式选型
  窗口与最终裁定属协调者。

## 4. 用户自研 HybridAstar（vendored，第 8/9 个后端）

- 来源与改写：`git@github.com:gannerfreer/HybridAstar.git`，提交
  `b6e5383a60c95ace77fe801856ef7cc18a9a4ab5`（作者本人代码，无 LICENSE，
  经作者指示复制用于内部对照）；源码位于
  `upstream/user_astar/`，逐项改写与编译范围见
  [PROVENANCE.md](../upstream/user_astar/PROVENANCE.md)。与前三组“概念重写、
  不复制上游代码”不同，这一组是**真实代码 vendoring + 最小钩子**。
- 两个后端：
  - `user_astar_raw`（kNative，参考项）：内部栅格碰撞（由请求 `GridView`
    栅格化障碍）、原始倒车运动学（镜像模型）、内置目标容差、无 RS 连接；
  - `user_astar_normalized`（kNormalized，参与选型）：修正倒车运动学
    （标准自行车模型，开关默认关闭以保持原行为）、公共
    `planner::collision` 位姿/运动检查、公共 `planner::rs` 目标连接（带
    节流/距离/长度限制）、时间/节点预算与取消、成功后端校验。
- 已记录偏差：① raw 倒车运动学与“后轴中心 + 带符号速度”约定不一致
  （yaw 变化方向与横向二阶项镜像），作为其原样行为保留并单列为参考；
  ② raw 的内部 OBB-栅格碰撞与公共碰撞检查结论可能不同（含起点/终点
  碰撞判定与负例可达性）；③ normalized 为满足统一契约新增了 RS 目标
  连接与预算/取消钩子（上游无此机制），原语曲率按弧长而非弦长换算；
  ④ 两后端均无跨调用缓存，`cache_mode` 恒为 `cold`；⑤ 倒车约束中的
  步长恒为 `motion_step_m`（上游前进/倒车共用 `delta_dist`），
  `reverse_motion_step_m` 未暴露。
- 正式基准命令已绑定 P 核（`taskset -c 0-7`，i7-13650HX，屏蔽 8 个 E 核
  的迁移干扰）；对照结果与选型影响见 `Docs/reports/hybrid_astar/` 的
  本轮报告。
