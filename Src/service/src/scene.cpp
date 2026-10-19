// 本地规划场景构建（见 scene.hpp）：委托 planner::scene（X-A1-13）。
//
// 迁移前位于本文件的窗口裁剪、可行驶域基础、障碍叠加与预算降级实现已移入
// Src/scene（planner::scene）；本文件只保留 planner::service 公开入口的
// 类型转换，错误码与结果字段不变。服务类型与场景类型相互独立，避免调用方
// 因参数类型（ADL）在 service/scene 两个 build_scene 之间产生歧义。

#include "planner/service/scene.hpp"

#include <utility>
#include <vector>

#include "planner/collision/scene.hpp"
#include "planner/scene/scene.hpp"

namespace planner::service {

SceneBuildResult build_scene(const map::Map& map,
                             const common::VehicleModel& vehicle,
                             const common::Pose2d& start,
                             const common::Pose2d& goal,
                             const std::vector<ServiceObstacle>& obstacles,
                             const SceneBuildOptions& options) {
  std::vector<collision::Obstacle> collision_obstacles;
  collision_obstacles.reserve(obstacles.size());
  for (const ServiceObstacle& obstacle : obstacles) {
    collision_obstacles.push_back(obstacle.to_collision());
  }

  scene::SceneBuildOptions scene_options;
  scene_options.resolution_m = options.resolution_m;
  scene_options.margin_m = options.margin_m;
  scene_options.max_cells = options.max_cells;

  scene::SceneBuildResult scene_result =
      scene::build_scene(map, vehicle, start, goal, collision_obstacles,
                         scene_options);

  SceneBuildResult result;
  result.ok = scene_result.ok;
  result.reason = std::move(scene_result.reason);
  result.detail = std::move(scene_result.detail);
  result.grid = std::move(scene_result.grid);
  result.scene = std::move(scene_result.scene);
  result.window = scene_result.window;
  result.requested_resolution_m = scene_result.requested_resolution_m;
  result.effective_resolution_m = scene_result.effective_resolution_m;
  result.scene_id = std::move(scene_result.scene_id);
  result.scene_version = std::move(scene_result.scene_version);
  return result;
}

}  // namespace planner::service
