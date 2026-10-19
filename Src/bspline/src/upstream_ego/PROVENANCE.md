# planner_bspline 对照后端来源与许可证说明（EGO UniformBspline）

> 本文件记录 `Src/bspline/src/upstream_ego/` 内上游代码的来源、移植范围与
> GPL-3.0 隔离方式。该目录只由对照库 `planner_bspline_ego` 编译，链接进
> 测试与基准可执行文件；`planner_bspline` 主库不包含这些文件。

## 1. 上游固定信息

| 项 | 值 |
| --- | --- |
| 上游仓库 | <https://github.com/ZJU-FAST-Lab/ego-planner> |
| 固定 commit | `bfda51284c8c1b476043255a8145ef925a3778a5`（master 分支 HEAD，2025-03-08） |
| 源码归档 | `https://github.com/ZJU-FAST-Lab/ego-planner/archive/bfda51284c8c1b476043255a8145ef925a3778a5.tar.gz` |
| 归档 SHA256 | `848e6cbe403a4a0e0cf44a30704ad47535a076c958c77ca551391040fb6fb573`（17,534,024 B） |
| 依赖锁条目 | `dependencies.lock.json` → `dependencies.ego_planner`（`status: locked`） |
| 许可证 | GPL-3.0（上游根 `LICENSE`，SHA256 `3972dc9744f6499f0f9b2dbf76696f2ae7ad8af9b23dde66d6af86c9dfb36986`；本目录 `LICENSE-GPL-3.0.txt` 为同一文本） |

固定流程（2026-10-11 实际执行）：

1. 通过 GitHub API 解析 master HEAD commit 并固定完整 SHA；
2. 两次独立下载归档并复算 SHA256，一致（`848e6cbe…`）；
3. 解压后与 `raw.githubusercontent.com` 同 commit 的两个源文件逐一比对，
   SHA256 一致（见下表）；
4. 只从该固定副本复制评估所需的最小文件。

## 2. 复制文件清单与改动

| 本仓库文件 | 上游路径 | 上游 SHA256 | 改动 |
| --- | --- | --- | --- |
| `include/bspline_opt/uniform_bspline.h` | `src/planner/bspline_opt/include/bspline_opt/uniform_bspline.h` | `8760dbf7…` | 仅新增来源注释头；代码逐字节一致 |
| `src/uniform_bspline.cpp` | `src/planner/bspline_opt/src/uniform_bspline.cpp` | `8552c7ef…` | 仅新增来源注释头、删除未使用的 `#include <ros/ros.h>`；其余逐字节一致 |
| `LICENSE-GPL-3.0.txt` | `LICENSE` | `3972dc97…` | 无 |

目录布局保留上游 `include/bspline_opt/` + `src/` 结构，因此 cpp 的
`#include "bspline_opt/uniform_bspline.h"` 原样成立，无需改包含路径。

未复制的上游文件（对照范围外）：`bspline_optimizer.*`（轨迹优化器，
依赖 ROS 与可视化）、`lbfgs.hpp`、`gradient_descent_optimizer.*`、
`package.xml`/`CMakeLists.txt`（catkin 构建）。

构建说明：上游 `uniform_bspline.cpp` 的 `getMeanAndMaxVel/Acc` 在
`getTimeSpan` 失败路径上存在 GCC `-Wmaybe-uninitialized` 警告（tm/tmp 未
初始化，上游原有）；该文件逐字保留，仅在 CMake 中对这一个源文件抑制
`-Wno-maybe-uninitialized`（GCC 专属），不修改上游代码。

## 3. ROS-free 抽离结论

- 上游 `uniform_bspline.cpp` 中 ROS 的唯一引用是第 2 行
  `#include <ros/ros.h>`；删除后以 `grep -n "ros::\|ROS_\|ros/"` 复核，
  两个文件中无任何 ROS 符号（2026-10-11）。
- 头文件只依赖 Eigen（`#include <Eigen/Eigen>`）与标准库。
- 数值核心（`setUniformBspline`、`setKnot`、`evaluateDeBoor`、
  `getDerivative`）不依赖 ROS 时间/消息类型。

## 4. 许可证与隔离

- 上游 EGO-Planner 以 GPL-3.0 分发；本目录包含其复制代码，因此
  `planner_bspline_ego` 目标整体按 GPL-3.0 处理。
- **隔离方式**：`planner_bspline_ego` 是独立静态库，只被
  `planner_bspline_tests` 与 `planner_bspline_bench` 链接；
  `planner_bspline` 主库不编译、不链接该目录，通过运行时注册的工厂
  （`detail::set_ego_backend_factory`）按需获得后端实现。主库的默认后端
  与公开 API 语义不受影响。
- 该后端仅用于内部选型对照（正确性/性能），不得在未做许可证决策前链接进
  任何交付组件；若未来需要把 EGO 实现用于产品路径，必须单独评审 GPL 影响。

## 5. 复核方式

```sh
# 重新下载并校验固定归档（需网络）
curl -L -o /tmp/ego-planner.tar.gz \
  https://github.com/ZJU-FAST-Lab/ego-planner/archive/bfda51284c8c1b476043255a8145ef925a3778a5.tar.gz
sha256sum /tmp/ego-planner.tar.gz   # 应为 848e6cbe403a4a0e0cf44a30704ad47535a076c958c77ca551391040fb6fb573
# 与 raw.githubusercontent 同 commit 的文件对照（去注释头/去 ros include 后）
```
