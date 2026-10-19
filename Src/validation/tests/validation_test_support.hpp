#pragma once

// planner_validation GoogleTest 测试套件共用的构建函数：来自
// Docs/component_development_plan.md 第 7.1 节的默认车辆/限制、直线与
// 圆弧轨迹夹具、走廊矩形，以及通过所有检查的基础请求。

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "planner/collision/scene.hpp"
#include "planner/validation/validation.hpp"

namespace planner::validation::test_support {

using common::Corridor;
using common::CorridorPolygon;
using common::Point2d;
using common::Pose2d;
using common::TrajectoryPoint;
using common::VehicleModel;

inline VehicleModel default_vehicle() {
  VehicleModel vehicle;
  vehicle.id = "default_test_vehicle";
  vehicle.wheelbase_m = 2.8;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.9;
  vehicle.rear_overhang_m = 1.0;
  vehicle.max_steer_rad = 0.5;
  vehicle.safety_margin_m = 0.1;
  return vehicle;
}

inline VehicleLimits default_limits() {
  VehicleLimits limits;
  limits.max_speed_mps = 2.0;
  limits.max_acceleration_mps2 = 2.0;
  limits.max_deceleration_mps2 = 2.0;
  limits.max_curvature_inv_m = 0.2;
  return limits;
}

inline TrajectoryPoint point(double t_s, double x_m, double y_m,
                             double yaw_rad, double velocity_mps,
                             double acceleration_mps2,
                             double curvature_inv_m = 0.0) {
  TrajectoryPoint point;
  point.t_s = t_s;
  point.pose = Pose2d{x_m, y_m, yaw_rad};
  point.velocity_mps = velocity_mps;
  point.acceleration_mps2 = acceleration_mps2;
  point.curvature_inv_m = curvature_inv_m;
  return point;
}

// 沿 +x 从 0 到 3 m 的五点直线轨迹（加速、巡航、制动至停止）；总弧长
// 3.0 m。与示例数据一致。
inline std::vector<TrajectoryPoint> straight_trajectory() {
  return {
      point(0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
      point(1.0, 0.5, 0.0, 0.0, 1.0, 0.0),
      point(2.0, 1.5, 0.0, 0.0, 1.0, 0.0),
      point(3.0, 2.5, 0.0, 0.0, 1.0, -1.0),
      point(4.0, 3.0, 0.0, 0.0, 0.0, 0.0),
  };
}

inline CorridorPolygon rectangle_polygon(double s_start_m, double s_end_m,
                                         double x_min_m, double y_min_m,
                                         double x_max_m, double y_max_m) {
  CorridorPolygon polygon;
  polygon.s_start_m = s_start_m;
  polygon.s_end_m = s_end_m;
  polygon.vertices = {Point2d{x_min_m, y_min_m}, Point2d{x_max_m, y_min_m},
                      Point2d{x_max_m, y_max_m}, Point2d{x_min_m, y_max_m}};
  return polygon;
}

inline Corridor make_corridor(std::vector<CorridorPolygon> polygons,
                              const std::string& map_id = "test_map",
                              const std::string& map_version = "v1") {
  Corridor corridor;
  corridor.id = "corridor_test";
  corridor.map_id = map_id;
  corridor.map_version = map_version;
  corridor.frame_id = "map";
  corridor.polygons = std::move(polygons);
  return corridor;
}

// 从头到尾覆盖 3 m 直线轨迹的走廊。
inline Corridor straight_corridor() {
  return make_corridor({rectangle_polygon(0.0, 1.5, -2.0, -2.0, 6.0, 2.0),
                        rectangle_polygon(1.5, 3.0, 0.0, -2.0, 7.0, 2.0)});
}

// 自由空间场景（无栅格、无障碍物）。
inline collision::Scene empty_scene() { return collision::Scene{}; }

// 给定布局的全空闲栅格；单元格按行主序排列。
inline collision::Scene free_grid_scene(int width, int height,
                                        double resolution_m, double origin_x_m,
                                        double origin_y_m,
                                        const std::string& map_id = "test_map",
                                        const std::string& map_version = "v1") {
  collision::Scene scene;
  scene.grid.map_id = map_id;
  scene.grid.map_version = map_version;
  scene.grid.width = width;
  scene.grid.height = height;
  scene.grid.resolution_m = resolution_m;
  scene.grid.origin = Pose2d{origin_x_m, origin_y_m, 0.0};
  scene.grid.cells.assign(static_cast<std::size_t>(width) *
                              static_cast<std::size_t>(height),
                          0);
  return scene;
}

// 基础请求：直线轨迹位于自由空间中并有覆盖走廊；所有检查都必须通过。
inline ValidationRequest base_request() {
  ValidationRequest request;
  request.vehicle = default_vehicle();
  request.limits = default_limits();
  request.trajectory = straight_trajectory();
  request.goal = Pose2d{3.0, 0.0, 0.0};
  request.scene = empty_scene();
  request.corridor = straight_corridor();
  return request;
}

}  // namespace planner::validation::test_support
