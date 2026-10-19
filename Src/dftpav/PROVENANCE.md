# planner_dftpav 来源与许可证说明（PROVENANCE）

> 本文件是本组件（`Src/dftpav/`）上游来源、移植范围与许可证的权威记录。
> 组件内所有移植文件都在文件头保留了同样的来源说明。

## 1. 上游固定信息

| 项 | 值 |
| --- | --- |
| 上游仓库 | <https://github.com/ZJU-FAST-Lab/Dftpav> |
| 固定 commit | `87f63ad2f67506e090b36eac38f5d247c51a330e`（main 分支，2023-06-21，无 release/tag） |
| 源码归档 | `https://github.com/ZJU-FAST-Lab/Dftpav/archive/87f63ad2f67506e090b36eac38f5d247c51a330e.tar.gz` |
| 归档 SHA256 | `061dea3f948e86414a5dc2f3bd3475cb1cb2d34d567a2786d3b9dd2153f2d77a`（90,181,649 B） |
| 依赖锁条目 | `dependencies.lock.json` → `dependencies.dftpav`（`status: locked`） |
| 许可证 | GPL-3.0（上游根 `LICENSE` 全文；本组件 `LICENSE-GPL-3.0.txt` 为同一文本） |
| 本地固定副本 | `/tmp/opencode/upstreams/Dftpav-87f63ad2f67506e090b36eac38f5d247c51a330e/`（执行环境临时目录，不入仓库） |

固定流程（本波次实际执行）：

1. 从锁文件读取 commit 与归档 URL、SHA256；
2. 下载归档并复算 SHA256，与锁文件一致；
3. 解压后与同 commit 的 `git clone` 逐文件比对（`diff -rq` 无差异）；
4. 只从该固定副本移植文件，移植结果全部位于 `Src/dftpav/src/upstream/`。

构建时**不下载**上游归档：仓库内只保留必要的移植代码，上游没有任何 CMake
目标被消费（上游是 ROS/catkin 工作空间，无独立 CMake 目标）。

## 2. 移植文件清单与改动

| 本仓库文件 | 上游路径 | 移植方式 |
| --- | --- | --- |
| `src/upstream/lbfgs.hpp` | `src/Plan/traj_planner/include/geo_utils2d/lbfgs.hpp` | 逐字节复制，仅在文件头添加来源注释（算法源自 liblbfgs，MIT，Naoki Okazaki；上游以 GPL-3.0 分发该文件） |
| `src/upstream/poly_traj.hpp` | `src/Plan/traj_planner/include/plan_utils/poly_traj_utils.hpp` | 去 ROS/HKUST `common` 依赖；`common::VehicleParam` → `planner::dftpav::VehicleParam`；删除 `Trajectory::GetState`（HKUST `common::State`）与 `getdddSigma` 调试打印；类 `Piece`/`Trajectory`/`BandedSystem`/`MinJerkOpt` 数值逻辑不变 |
| `src/upstream/traj_optimizer.hpp` | `src/Plan/traj_planner/include/plan_manage/traj_optimizer.h` | 去 ROS/protobuf/可视化；protobuf `planning::minco::Config` → 纯值结构 `OptimizerConfig`（默认值取 `config/minco_config.pb.txt`）；删除动态避让（surround）与可视化成员；新增 `setVehicleParam` 与统计读取接口；**移植扩展**：`OptimizerConfig::wei_anchor`（内层路径点与换向点对预制初值的二次锚定，上游无此项，用于保持走廊站位一致性，见 README §2）；**移植扩展**：协作式取消钩子 `setCancelCheck`/`wasCancelled`（上游无此项，见 README §1） |
| `src/upstream/traj_optimizer.cpp` | `src/Plan/traj_planner/src/traj_optimizer.cpp` | 去 ROS（发布器、`ros::Time`、`ROS_*` 宏改为 stderr 输出）；删除动态避让/调试/可视化函数（`surroundGradCostP`、`debugGradCheck`、`dynamicObsGradCostP`、`dynamicObsCosCheck`、`log_sum_exp`、`display*`、未使用的 `positiveSmoothedL3`/`getBoundPts`）；`setParam` 改为读取 `OptimizerConfig`；`OptimizeTrajectory` 保留原数学与签名（`now` 默认 0.0），迭代上限与代价失败阈值改为可配置，并记录 L-BFGS 终止码与最终代价；**移植扩展**：在代价回调中加入 `wei_anchor` 锚定项（成本与梯度，见上），并在 `OptimizeTrajectory` 入口保存预制初值；**移植扩展**：把移植中处于休眠状态的上游进度回调 `earlyExitCallback` 接入 `lbfgs_optimize`，并在代价回调/进度回调中轮询可选的取消钩子（取消时求解以 `LBFGS_CANCELED` 停止，`OptimizeTrajectory` 报告失败而不暴露部分迭代；上游数学不变） |
| `include/planner/dftpav/vehicle_param.hpp` | `src/Sim/core/common/inc/common/basics/semantics.h`（`class VehicleParam`） | 抽取优化器实际使用的访问器，移入 `planner::dftpav` 命名空间，无 ROS 依赖 |
| `src/optimizer.cpp`、`include/planner/dftpav/optimizer.hpp`、`tests/`、`examples/` | 无 | 本项目原创适配层（预制路径/公共 Corridor → 上游优化器输入；输出采样、状态与失败原因、临时可行性检查），不是上游代码 |

未移植的上游功能（本波次范围外，见 README“已知限制”）：Kino A* 前端、地图
/语义地图适配、动态障碍（surround）、ROS 服务与可视化、protobuf 参数加载、
`minco_config.pb.txt` 之外的上游参数项。

移植扩展（相对上游的受控新增，均不改变优化数学；公共接口不泄漏上游类型）：

1. `wei_anchor` 锚定项（README §2）：把内层路径点与换向点二次锚定到预制
   初值，保持走廊站位一致性；上游无此项。
2. 协作式取消（README §1）：`PolyTrajOptimizer::setCancelCheck` /
   `wasCancelled`，在 `costFunctionCallback` 与 `earlyExitCallback` 中轮询
   调用方谓词。取消时求解在迭代边界停止（`LBFGS_CANCELED`），
   `OptimizeTrajectory` 返回 false 且 `wasCancelled()` 为真；适配层据此返回
   `kCancelled`/"cancelled"，**不采样、不返回部分迭代结果**。上游已有同一
   进度回调机制（`earlyExitCallback`），但移植版本此前把它作为 `proc_progress`
   传 `NULL`，且其声明仍保留 liblbfgs 风格的旧签名（与本仓库 `lbfgs.hpp` 的
   `lbfgs_progress_t` 不匹配）。本扩展把该回调签名对齐到移植版
   `lbfgs_progress_t` 并接入这一已有退出路径用于取消，没有新增求解路径。
   公共 API 通过 `OptimizeOptions::cancel_check`（`std::function<bool()>`）
   暴露；适配层用 C 风格回调 + 上下文桥接，谓词对象在调用期间保持有效。

适配层（`src/optimizer.cpp`、公共头、测试与示例）是本项目原创代码，不包含
上游数学：初值构造（梯形速度 + 等时长 MINCO 初值）、公共类型映射、限制收紧
与重试（含 hybrid-006 的碰撞驱动定向重试阶梯与 `collision_retry`/
`internal_curvature_tightening` 选项，均为适配层内部预算，不改变 V 门禁）、
临时可行性检查、输出采样与统计均为本项目实现；移植的上游文件只做
了上述“来源与改动”一节列出的删减与替换。

## 3. 移植补丁的应用方式

移植由 `traj_optimizer.cpp` 的一次性脚本完成（脚本本身只存在于执行环境，
不进入仓库）：脚本以文件内唯一文本锚点做定点删除/替换，并在结束时检查
`ros::`、`ROS_`、`surround_trajs_` 等符号无残留后才写出目标文件。复核方式：

```sh
# 重新下载并校验固定归档（需网络）
curl -L -o /tmp/dftpav.tar.gz \
  https://github.com/ZJU-FAST-Lab/Dftpav/archive/87f63ad2f67506e090b36eac38f5d247c51a330e.tar.gz
sha256sum /tmp/dftpav.tar.gz   # 应为 061dea3f948e86414a5dc2f3bd3475cb1cb2d34d567a2786d3b9dd2153f2d77a
# 解压后与本组件 src/upstream/ 中文件头注释列出的上游路径逐文件对照
```

## 4. 许可证与隔离

- 上游 Dftpav 以 GPL-3.0 分发；本组件包含其衍生代码，因此 `Src/dftpav/`
  整体按 GPL-3.0 处理，许可证全文见 `LICENSE-GPL-3.0.txt`。
- 本组件是隔离的实验后端（计划 §2/§3.5）：其他组件不依赖 `planner_dftpav`
  的公开接口，Dftpav 类型与原代码不泄漏到其他组件的公共 API。
- 上游 `thirdparty/odeint-v2` 等未移植目录的第三方许可证不适用于本组件移植
  文件（未使用）。
