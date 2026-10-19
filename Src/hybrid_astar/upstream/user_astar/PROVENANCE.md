# user_astar 上游来源与改写说明

- 来源：`git@github.com:gannerfreer/HybridAstar.git`（作者本人代码）
- 固定提交：`b6e5383a60c95ace77fe801856ef7cc18a9a4ab5`（2026-07-23，Merge branch 'backend' into presentation）
- 复制时间：2026-10-11；复制范围：`include/astar/*.h` 与 `src/*.cpp`（作者指示用于本仓库内部对照基准）
- 许可证：上游仓库无 LICENSE 文件。按作者指示在本仓库内复制用于内部基准并保留来源与署名；
  如需对外分发，须先与作者确认许可。
- 编译范围：本组件仅编译 `src/hybrid_astar.cpp` 与 `src/node2d.cpp`；
  `grid_astar.*`、`planner_config.cpp` 原样收录以保持源码完整，不参与构建。

## 改写清单（最小钩子；未启用时与上游行为一致）

1. `HybridAStarConfig` 新增两个开关（默认均为原行为）：
   - `allow_reverse`（默认 `true`）：置 `false` 时不生成倒车原语；
   - `standard_reverse_kinematics`（默认 `false`）：置 `true` 时 `Propagate`
     的倒车分支使用标准自行车模型（yaw 变化 `-beta`、精确圆弧位移）；
     前进分支两种设置下完全一致。
2. 可选钩子（默认空）：
   - `SetAbortCallback`：搜索每次迭代开始检查（时间预算/取消）；返回 `true`
     时以消息 `"aborted"` 退出；
   - `SetPoseCollisionHook` / `SetMotionCollisionHook`：归一化后端接入公共
   碰撞检查；设置后分别取代 `IsCollision(vertex)` 与扩展循环内的位姿检查；
   - `SetGoalHook`：归一化后端接入公共 RS 解析连接，取代内置的
     距离/航向容差判据。
3. `AddBoundaryWorldPoint`：程序化边界栅格入口（与 CSV 入口等价，保留
   节点膨胀规则），供 raw 后端把公共 `GridView` 的占据栅格接入内部碰撞。

`src/hybrid_astar.cpp` 与上游的逐行差异仅限以上四点，均以
`[planner-adapter 扩展]` 注释标记。
