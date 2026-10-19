#pragma once

// W5 服务的本地规划场景构建（X-A1-13：实现委托 planner::scene）。
//
// 该头文件保留 planner::service 的既有公开入口与类型（SceneBuildOptions /
// SceneBuildResult / build_scene），实现委托给 planner::scene：窗口裁剪、
// 可行驶域基础、障碍叠加、预算降级与稳定错误码都在 planner/scene/scene.hpp
// 中；本入口只做 ServiceObstacle -> collision::Obstacle 的类型转换，因此
// 既有消费者（流水线、示例、测试）与错误码不变。新增代码应直接使用
// planner::scene。
//
// 服务在由请求几何推导出的本地窗口上规划（起点、终点和障碍物包围盒，按
// 边距扩展并裁剪到导入的地图边界）。窗口的基础域不再是"全自由"：它由
// 地图的 lanelet 可行驶域栅格化得到（Map::export_driveable_grid）——与任一
// lanelet 可行驶多边形相交的单元为空闲，其余为占据；请求中的障碍物随后
// 叠加为占据（单元格中心位于闭合多边形内记为占据，即文档化的测试夹具
// 推导方式），并以精确凸多边形保留在 planner::collision::Scene 中，因此
// 网格势场和精确的整车检查都能感知它们。规划（Hybrid A* / 走廊 /
// Dftpav / 独立验证）因此被约束在可行驶域内。
//
// 起点与终点除必须位于地图边界内，还必须落在可行驶域单元上，否则场景
// 构建以稳定错误码 "start_outside_driveable" / "goal_outside_driveable"
// 失败（先于障碍物栅格化判定，因此障碍物压在起点上的既有
// "invalid_input" 行为不变）。
//
// 导入的 Lanelet2 地图提供地图标识/版本（stale_map 检查）、本地坐标系、
// 外层规划域与可行驶域几何；栅格化只使用 lanelet 车道（area 对象不参与，
// 见 README.md §6 的当前边界）。

#include <cstddef>
#include <string>
#include <vector>

#include "planner/collision/scene.hpp"
#include "planner/common/types.hpp"
#include "planner/map/map.hpp"
#include "planner/map/types.hpp"
#include "planner/service/types.hpp"

namespace planner::service {

struct SceneBuildOptions {
  // 请求的网格分辨率；当窗口会超出 `max_cells` 时按整数倍（2 的幂）增大。
  double resolution_m{0.1};
  // 裁剪到地图边界之前，围绕起点/终点/障碍物的额外边距。在其上再加上
  // 车辆外接圆半径，因此起点和终点的车身足迹始终位于网格内。
  double margin_m{5.0};
  // 网格单元格数上限（width * height）；16M 个单元格 = 64 MiB。
  std::size_t max_cells{16u * 1024u * 1024u};
};

struct SceneBuildResult {
  bool ok{false};
  // 稳定的失败错误码："start_outside_map"、"goal_outside_map"、
  // "start_outside_driveable"、"goal_outside_driveable"、
  // "window_outside_map"、"window_too_large"、"invalid_scene"。
  std::string reason;
  std::string detail;
  common::GridView grid;
  collision::Scene scene;
  // X-M1（协议 §5）：有效（量化后）窗口与分辨率、内容标识。失败结果中
  // scene_id/scene_version 为空。
  map::Bounds2d window;
  double requested_resolution_m{0.0};
  double effective_resolution_m{0.0};
  std::string scene_id;
  std::string scene_version;
};

// 针对已加载的地图为一个请求构建网格 + 碰撞场景（可行驶域基础 + 请求
// 障碍）。起点/终点必须位于地图边界内且落在可行驶域单元上。
// `map` 在调用期间只读；见 planner/map/map.hpp 的线程安全说明。
SceneBuildResult build_scene(const map::Map& map,
                             const common::VehicleModel& vehicle,
                             const common::Pose2d& start,
                             const common::Pose2d& goal,
                             const std::vector<ServiceObstacle>& obstacles,
                             const SceneBuildOptions& options = {});

}  // namespace planner::service
