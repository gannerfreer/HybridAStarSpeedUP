#pragma once

// planner_scene：地图窗口、可行驶域栅格与用户障碍的场景组合（X-A1-18）。
//
// 职责与依赖边界（冻结计划 §2.1）：
//   * 承载 W5 服务的本地场景构建（自 planner::service 的 src/scene.cpp 迁移，
//     见 X-A1-13）：由请求几何推导本地窗口（起点、终点和障碍物包围盒，按
//     边距扩展并裁剪到导入的地图边界），以地图 lanelet 可行驶域为栅格基础
//     （Map::export_driveable_grid），再叠加请求障碍；
//   * 依赖 map/grid/collision 的公共接口；不依赖规划器、HTTP 或显示；
//   * 错误通过稳定的字符串码报告，与迁移前的服务实现逐字一致。
//
// 窗口的基础域不再是"全自由"：与任一 lanelet 可行驶多边形相交的单元为空闲，
// 其余为占据；请求中的障碍物随后叠加为占据（单元格中心位于闭合多边形内记为
// 占据，即文档化的测试夹具推导方式），并以精确凸多边形保留在
// planner::collision::Scene 中，因此网格势场和精确的整车检查都能感知它们。
//
// 起点与终点除必须位于地图边界内，还必须落在可行驶域单元上，否则场景构建以
// 稳定错误码 "start_outside_driveable" / "goal_outside_driveable" 失败（先于
// 障碍物栅格化判定，因此障碍物压在起点上的既有 "invalid_input" 行为不变）。
//
// 障碍物以 collision::Obstacle（已校验的凸多边形；矩形用
// Obstacle::rectangle）传入；调用方负责在请求解析阶段完成验证，本函数对
// 顶点数不足的障碍物跳过栅格化并由 collision::Scene::valid 报告
// "invalid_scene"。

#include <cstddef>
#include <string>
#include <vector>

#include "planner/collision/scene.hpp"
#include "planner/common/types.hpp"
#include "planner/map/map.hpp"
#include "planner/map/types.hpp"

namespace planner::scene {

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
  // "window_outside_map"、"window_too_large"、"invalid_scene"；
  // 预览入口另有 "invalid_window"、"empty_window"。
  std::string reason;
  std::string detail;
  common::GridView grid;
  collision::Scene scene;
  // X-M1（协议 §5）：有效（量化后）窗口与分辨率、内容标识。失败结果中
  // scene_id/scene_version 为空；window/分辨率字段可能仍已填充。
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
                             const std::vector<collision::Obstacle>& obstacles,
                             const SceneBuildOptions& options = {});

// 显式窗口的场景预览（协议 §3.3，X-M1）：窗口裁剪到地图边界（空交集失败
// "window_outside_map"），随后与 build_scene 使用同一组合——可行驶域基础 +
// 障碍叠加（单元中心位于闭合多边形内即占据）——并计算同一 scene_id。不做
// 起点/终点校验（起终点不参与预览栅格内容）。
struct ScenePreviewOptions {
  // 请求窗口（可超出地图边界；服务回显裁剪后的有效窗口）。
  map::Bounds2d window;
  // 请求分辨率；超出 `max_cells` 时按 2 的幂上调（degraded 语义同协议 §4）。
  double resolution_m{0.1};
  // 网格单元格数上限（width * height）。
  std::size_t max_cells{16u * 1024u * 1024u};
};

// 失败原因："invalid_window"、"empty_window"、"window_outside_map"、
// "window_too_large"、"invalid_scene"。
SceneBuildResult build_scene_preview(const map::Map& map,
                                     const std::vector<collision::Obstacle>& obstacles,
                                     const ScenePreviewOptions& options = {});

}  // namespace planner::scene
