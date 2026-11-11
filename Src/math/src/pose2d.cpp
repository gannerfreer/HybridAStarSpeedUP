/**
 * @file pose2d.cpp
 * @brief 二维位姿坐标系变换实现
 */

#include "planner/math/pose2d.hpp"

#include <cmath>

namespace planner::math {

Point2d transform_point(const Pose2d& pose, const Point2d& body_point) noexcept {
  const double c = std::cos(pose.yaw_rad);
  const double s = std::sin(pose.yaw_rad);
  return Point2d{pose.x_m + c * body_point.x_m - s * body_point.y_m,
                 pose.y_m + s * body_point.x_m + c * body_point.y_m};
}

Point2d to_body_frame(const Pose2d& pose, const Point2d& world_point) noexcept {
  const double c = std::cos(pose.yaw_rad);
  const double s = std::sin(pose.yaw_rad);
  const double dx = world_point.x_m - pose.x_m;
  const double dy = world_point.y_m - pose.y_m;
  return Point2d{c * dx + s * dy, -s * dx + c * dy};
}

}  // namespace planner::math
