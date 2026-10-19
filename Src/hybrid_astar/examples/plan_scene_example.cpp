// ros_free 归一化后端的命令行示例。
//
// 构建一个带一堵墙的 18 m x 10 m 合成场景（0.1 m 栅格），从左侧开阔区域
// 规划到墙后的目标点。`--self-check` 运行固定断言并注册到 CTest
//（标签 hybrid_astar）；默认模式打印路径摘要和统计 JSON。

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <string>

#include "planner/collision/scene.hpp"
#include "planner/hybrid_astar/ros_free_backend.hpp"

namespace {

using planner::collision::CheckResult;
using planner::collision::CollisionChecker;
using planner::collision::Obstacle;
using planner::collision::Scene;
using planner::common::GridView;
using planner::common::Pose2d;
using planner::common::Status;
using planner::common::VehicleModel;
using planner::hybrid_astar::PlannerRsProvider;
using planner::hybrid_astar::RosFreeBackend;

constexpr double kWallXMin = 8.0;
constexpr double kWallYMin = 0.0;
constexpr double kWallXMax = 10.0;
constexpr double kWallYMax = 6.5;
constexpr double kResolutionM = 0.1;
constexpr int kGridWidth = 180;
constexpr int kGridHeight = 140;

VehicleModel example_vehicle() {
  VehicleModel vehicle;
  vehicle.id = "example_vehicle";
  vehicle.wheelbase_m = 2.8;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.9;
  vehicle.rear_overhang_m = 1.0;
  vehicle.max_steer_rad = 0.5;
  vehicle.safety_margin_m = 0.1;
  return vehicle;
}

// 接触即占据：按测试夹具规则对轴对齐墙做栅格化。
void mark_wall(GridView* grid) {
  for (int row = 0; row < grid->height; ++row) {
    const double y0 = row * grid->resolution_m;
    const double y1 = y0 + grid->resolution_m;
    if (y1 < kWallYMin || y0 > kWallYMax) {
      continue;
    }
    for (int col = 0; col < grid->width; ++col) {
      const double x0 = col * grid->resolution_m;
      const double x1 = x0 + grid->resolution_m;
      if (x1 < kWallXMin || x0 > kWallXMax) {
        continue;
      }
      grid->cells[static_cast<std::size_t>(row) *
                      static_cast<std::size_t>(grid->width) +
                  static_cast<std::size_t>(col)] =
          planner::common::kCellOccupied;
    }
  }
}

int run(bool self_check) {
  const VehicleModel vehicle = example_vehicle();

  GridView grid;
  grid.map_id = "example_wall_map";
  grid.map_version = "v1";
  grid.resolution_m = kResolutionM;
  grid.width = kGridWidth;
  grid.height = kGridHeight;
  grid.origin = Pose2d{0.0, 0.0, 0.0};
  grid.cells.assign(
      static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height),
      planner::common::kCellFree);
  mark_wall(&grid);

  Scene scene;
  scene.grid = grid;
  scene.obstacles.push_back(Obstacle::rectangle("example_wall", kWallXMin,
                                                kWallYMin, kWallXMax,
                                                kWallYMax));
  const CollisionChecker checker(vehicle, scene);
  if (!checker.valid()) {
    std::fprintf(stderr, "scene is invalid: %s\n",
                 checker.invalid_reason().c_str());
    return 1;
  }

  planner::common::PlanRequest request;
  request.request_id = "example_request";
  request.map_id = grid.map_id;
  request.map_version = grid.map_version;
  request.vehicle = vehicle;
  request.start = Pose2d{1.5, 2.0, 0.0};
  request.goal = Pose2d{13.5, 8.0, 0.0};

  RosFreeBackend backend;
  const planner::hybrid_astar::ConfigReport report =
      backend.configure(nlohmann::json::object());
  if (!report.accepted) {
    std::fprintf(stderr, "configuration rejected: %s\n", report.reason.c_str());
    return 1;
  }
  PlannerRsProvider provider(vehicle);
  const planner::common::PlanResult result =
      backend.plan(request, grid, checker, provider);

  std::printf("backend: %s (%s)\n", backend.id().c_str(),
              planner::hybrid_astar::to_string(backend.kind()));
  std::printf("status: %s\n", planner::common::to_string(result.status).c_str());
  if (!result.reason.empty()) {
    std::printf("reason: %s\n", result.reason.c_str());
  }
  std::size_t point_count = 0;
  for (const planner::common::PathSegment& segment : result.path) {
    point_count += segment.points.size();
  }
  std::printf("segments: %zu, points: %zu\n", result.path.size(), point_count);
  if (result.status == Status::kSuccess) {
    std::printf("path_length_m: %.3f\n",
                result.statistics.at("path_length_m").get<double>());
    std::printf("gear_changes: %lld\n",
                static_cast<long long>(
                    result.statistics.at("gear_changes").get<std::int64_t>()));
  }

  if (!self_check) {
    std::printf("%s\n", result.statistics.dump(2).c_str());
    return result.status == Status::kSuccess ? 0 : 1;
  }

  int failures = 0;
  if (result.status != Status::kSuccess) {
    std::fprintf(stderr, "self-check: expected success, got %s (%s)\n",
                 planner::common::to_string(result.status).c_str(),
                 result.reason.c_str());
    ++failures;
  } else {
    const CheckResult recheck = checker.check_path(result.path);
    if (!recheck.is_clear()) {
      std::fprintf(stderr, "self-check: output path collides (%s)\n",
                   planner::collision::to_string(recheck.reason_code));
      ++failures;
    }
    if (result.path.empty() || result.path.back().points.empty()) {
      std::fprintf(stderr, "self-check: output path is empty\n");
      ++failures;
    } else {
      const Pose2d& last = result.path.back().points.back().pose;
      const double position_error =
          std::hypot(last.x_m - request.goal.x_m, last.y_m - request.goal.y_m);
      const double yaw_error = std::abs(planner::common::normalize_angle_rad(
          last.yaw_rad - request.goal.yaw_rad));
      if (position_error > 0.05 || yaw_error > 0.02) {
        std::fprintf(stderr,
                     "self-check: endpoint outside tolerance (%.6f m, %.6f "
                     "rad)\n",
                     position_error, yaw_error);
        ++failures;
      }
      const double max_curvature = 1.0 / vehicle.min_turn_radius_m();
      for (const planner::common::PathSegment& segment : result.path) {
        for (const planner::common::PathPoint& point : segment.points) {
          if (std::abs(point.curvature_inv_m) > max_curvature + 1e-9) {
            std::fprintf(stderr, "self-check: curvature limit exceeded\n");
            ++failures;
          }
        }
      }
    }
  }
  if (failures == 0) {
    std::printf("self-check: ok\n");
    return 0;
  }
  return 1;
}

}  // namespace

int main(int argc, char** argv) {
  bool self_check = false;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--self-check") {
      self_check = true;
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", argument.c_str());
      return 2;
    }
  }
  return run(self_check);
}
