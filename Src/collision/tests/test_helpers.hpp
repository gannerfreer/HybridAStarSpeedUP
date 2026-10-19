#pragma once

// planner_collision GoogleTest 套件共用的辅助函数。

#include <array>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "planner/collision/collision_checker.hpp"
#include "planner/collision/reference.hpp"
#include "planner/collision/scene.hpp"
#include "planner/common/serialization.hpp"
#include "planner/common/types.hpp"

namespace planner::collision::test {

inline constexpr double kPi = 3.14159265358979323846;

// §7.1 默认车辆，含安全余量。
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

// 与 2 m x 2 m 最小夹具栅格配合使用的小型车辆。
inline VehicleModel small_vehicle() {
  VehicleModel vehicle;
  vehicle.id = "small_test_vehicle";
  vehicle.wheelbase_m = 0.6;
  vehicle.width_m = 0.4;
  vehicle.front_overhang_m = 0.1;
  vehicle.rear_overhang_m = 0.1;
  vehicle.max_steer_rad = 0.5;
  vehicle.safety_margin_m = 0.05;
  return vehicle;
}

// 不含栅格与障碍物的空场景。
inline Scene empty_scene() { return Scene{}; }

// 覆盖 width_m x height_m 米、分辨率 0.1 m 的均匀自由栅格，原点位于 (0, 0, 0)。
inline Scene free_grid(double width_m, double height_m) {
  Scene scene;
  scene.grid.map_id = "test_grid";
  scene.grid.map_version = "v1";
  scene.grid.resolution_m = 0.1;
  scene.grid.width = static_cast<int>(std::lround(width_m / 0.1));
  scene.grid.height = static_cast<int>(std::lround(height_m / 0.1));
  scene.grid.origin = Pose2d{0.0, 0.0, 0.0};
  scene.grid.cells.assign(
      static_cast<std::size_t>(scene.grid.width) * scene.grid.height, 0);
  return scene;
}

// 把世界坐标矩形（原点坐标系）内的单元统一涂为占用或未知。
inline void paint_cells(GridView& grid, double x_min, double y_min,
                        double x_max, double y_max, int value) {
  const int col0 = static_cast<int>(std::floor(x_min / grid.resolution_m));
  const int col1 = static_cast<int>(std::ceil(x_max / grid.resolution_m)) - 1;
  const int row0 = static_cast<int>(std::floor(y_min / grid.resolution_m));
  const int row1 = static_cast<int>(std::ceil(y_max / grid.resolution_m)) - 1;
  for (int row = std::max(row0, 0); row <= std::min(row1, grid.height - 1);
       ++row) {
    for (int col = std::max(col0, 0); col <= std::min(col1, grid.width - 1);
         ++col) {
      grid.cells[static_cast<std::size_t>(row) * grid.width + col] = value;
    }
  }
}

// 复制自生产约定的稳定弧线积分；仅用于构造测试几何。
inline Pose2d pose_on_arc(const Pose2d& start, double s, double kappa) {
  const double b = kappa * s;
  const double cy = std::cos(start.yaw_rad);
  const double sy = std::sin(start.yaw_rad);
  const double q = 0.5 * b;
  const auto sinc = [](double x) {
    return std::abs(x) < 1e-8 ? 1.0 - x * x / 6.0 : std::sin(x) / x;
  };
  const double quartic = q * sinc(q) * sinc(q);
  const double dsin = cy * sinc(b) - sy * quartic;
  const double dcos = -sy * sinc(b) - cy * quartic;
  Pose2d out;
  out.x_m = start.x_m + s * dsin;
  out.y_m = start.y_m - s * dcos;
  out.yaw_rad = planner::common::normalize_angle_rad(start.yaw_rad + b);
  return out;
}

inline std::string read_text_file(const std::string& path) {
  std::ifstream stream(path);
  std::stringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

inline GridView load_fixture_grid(const std::string& relative) {
  return planner::common::decode_document<GridView>(
      read_text_file(std::string(PLANNER_COLLISION_FIXTURES_DIR) + "/" +
                     relative));
}

inline VehicleModel load_fixture_vehicle(const std::string& relative) {
  return planner::common::decode_document<VehicleModel>(
      read_text_file(std::string(PLANNER_COLLISION_FIXTURES_DIR) + "/" +
                     relative));
}

// 一段曲线运动以低于扫掠精度下限的 1e-8 m 间隙经过墙壁：生产检查器报告保守
// 碰撞（kPrecisionFloor），参考报告 inconclusive。两个调用方看到相同的输入。
struct SubPrecisionTangentCase {
  VehicleModel vehicle;
  Scene scene;
  MotionSegment motion;
};

inline SubPrecisionTangentCase make_sub_precision_tangent_case() {
  SubPrecisionTangentCase test_case;
  test_case.vehicle = default_vehicle();
  test_case.scene = free_grid(40.0, 30.0);
  test_case.motion.start = Pose2d{5.0, 10.0, 0.0};
  test_case.motion.length_m = 2.0;
  test_case.motion.direction = Direction::kForward;
  test_case.motion.curvature_inv_m = 0.2;
  const Footprint footprint = make_footprint(test_case.vehicle);
  const Pose2d end =
      pose_on_arc(test_case.motion.start, test_case.motion.length_m,
                  test_case.motion.curvature_inv_m);
  // 与终点位姿前表面平行的墙，间隙 1e-9 m：低于下限区间的矢高，但高于接触容差，
  // 因此参考在精度下限处无法断定。
  const double gap = 1e-9;
  const double near_x = footprint.x_max_m + gap;
  std::vector<Point2d> vertices;
  for (const Point2d& p : {Point2d{near_x, -4.0}, Point2d{near_x + 0.5, -4.0},
                           Point2d{near_x + 0.5, 4.0}, Point2d{near_x, 4.0}}) {
    vertices.push_back(transform_point(end, p));
  }
  test_case.scene.obstacles.push_back(
      Obstacle::convex_polygon("tangent_wall", std::move(vertices)));
  return test_case;
}

}  // namespace planner::collision::test
