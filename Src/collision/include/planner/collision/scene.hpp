#pragma once

// 碰撞组件的场景描述：矩形车辆足迹、多边形障碍物与只读占据栅格。
//
// GridView 语义（冻结的 v1 约定，tests/fixtures/v1/README.md）：
//   * `origin` 是单元 (0,0) 左下角的位姿；
//     单元 (r,c) 中心 = origin + ((c+0.5)*res, (r+0.5)*res)；
//   * 单元取值 0=空闲，1=占据，2=未知；
//   * 未知单元与地图矩形之外的一切区域，消费方均视为占据。
//
// 车辆足迹是车体坐标系中的整车矩形（后轴中心位姿）：x 在
// [-(rear_overhang + margin), wheelbase + front_overhang + margin]，
// y 在 [-width/2 - margin, width/2 + margin]。安全边距包含在内；因此 v1
// 默认车辆检查的是 4.9 m x 2.0 m 的包络。

#include <array>
#include <string>
#include <vector>

#include "planner/collision/geometry.hpp"

namespace planner::collision {

using planner::common::GridView;
using planner::common::VehicleModel;

// 地图局部坐标系中的凸多边形障碍物，逆时针。
struct Obstacle {
  std::string id;
  std::vector<Point2d> vertices;  // 闭合多边形，>= 3 个顶点

  // 轴对齐矩形，逆时针，闭集。输入无效（非有限 / 范围反向）时返回顶点
  // 为空的障碍物。
  static Obstacle rectangle(const std::string& id, double x_min_m,
                            double y_min_m, double x_max_m, double y_max_m);

  // 凸多边形；顶点归一化为逆时针。输入不是有效凸多边形时返回顶点为空的
  // 障碍物。
  static Obstacle convex_polygon(const std::string& id,
                                 std::vector<Point2d> vertices);

  // 验证顶点（见 convex_polygon_valid）；`reason` 可选。
  bool valid(std::string* reason = nullptr) const;

  Aabb2d aabb() const noexcept;
};

// 检查器消费的场景。宽度==0 且高度==0 的栅格表示“无栅格”；其他任何
// 零/负尺寸组合都是无效输入。
struct Scene {
  GridView grid;
  std::vector<Obstacle> obstacles;

  bool has_grid() const noexcept { return grid.width > 0 && grid.height > 0; }

  // 验证栅格与每个障碍物。`reason` 接收首个错误。
  bool valid(std::string* reason = nullptr) const;
};

// 由车辆模型导出的整车矩形足迹。
struct Footprint {
  double x_min_m{0.0};  // 车体坐标系（后轴在原点）
  double x_max_m{0.0};
  double y_min_m{0.0};
  double y_max_m{0.0};
  double corner_radius_m{0.0};   // 最大 |corner|，用于扫掠边界
  double half_width_m{0.0};      // (width + 2*margin) / 2
  double half_length_m{0.0};     // (length + 2*margin) / 2
  double center_x_m{0.0};        // 足迹中心在车体坐标系中的位置

  // 车体坐标系角点，从 (x_min, y_min) 起逆时针。
  std::array<Point2d, 4> body_corners() const noexcept;

  // 给定后轴中心位姿对应的世界坐标系角点。
  std::array<Point2d, 4> world_corners(const Pose2d& pose) const noexcept;
};

// 车辆足迹参数（轴距/宽度/前后悬/边距）必须有限且非负，且轴距与宽度为
// 正。max_steer_rad 不属于足迹，在此不作要求。
bool vehicle_footprint_valid(const VehicleModel& vehicle,
                             std::string* reason = nullptr);

// 构建足迹；假定车辆参数有效。
Footprint make_footprint(const VehicleModel& vehicle) noexcept;

// 原点坐标系中的栅格几何：单元 (r,c) 覆盖
// [c*res, (c+1)*res] x [r*res, (r+1)*res]；地图矩形为
// [0, width*res] x [0, height*res]。
struct GridFrame {
  double cos_yaw{1.0};
  double sin_yaw{0.0};
  double origin_x{0.0};
  double origin_y{0.0};
  double width_m{0.0};
  double height_m{0.0};
  double resolution_m{0.0};

  static GridFrame from_grid(const GridView& grid) noexcept;

  // 世界 -> 原点坐标系。
  Point2d to_grid_frame(const Point2d& world_point) const noexcept;

  // 点位于原点坐标系的闭地图矩形内时返回 true。
  bool inside_map(const Point2d& grid_point) const noexcept;

  // 内部点到地图矩形边界的距离（在外部为负）。供保守包络测试使用。
  double distance_to_boundary(const Point2d& grid_point) const noexcept;
};

}  // namespace planner::collision
