/**
 * @file pose2d.hpp
 * @brief 二维车辆位姿及世界/车体坐标系点变换
 */

#pragma once

#include "planner/math/vec2d.hpp"

namespace planner::math {

/**
 * @brief 车辆后轴中心在局部右手坐标系下的二维位姿
 * @details X 轴指向车体正前方，Y 轴指向车体左侧，航向角 yaw 沿逆时针方向为正（弧度）
 */
struct Pose2d {
  double x_m{0.0};      ///< 后轴中心 X 坐标（米）
  double y_m{0.0};      ///< 后轴中心 Y 坐标（米）
  double yaw_rad{0.0};  ///< 车身朝向角（弧度），归一化在 [-pi, pi) 区间
};

inline constexpr bool operator==(const Pose2d& lhs, const Pose2d& rhs) {
  return lhs.x_m == rhs.x_m && lhs.y_m == rhs.y_m && lhs.yaw_rad == rhs.yaw_rad;
}

/**
 * @brief 根据车体位姿，将车体局部坐标系下的点变换至世界坐标系
 * @param[in] pose 车体在世界系下的位姿
 * @param[in] body_point 车体局部坐标点
 * @return 变换后的世界坐标系点
 */
Point2d transform_point(const Pose2d& pose, const Point2d& body_point) noexcept;

/**
 * @brief 将世界坐标系下的点反算至车体局部坐标系
 * @param[in] pose 车体在世界系下的位姿
 * @param[in] world_point 世界坐标点
 * @return 车体局部坐标系下的点
 */
Point2d to_body_frame(const Pose2d& pose, const Point2d& world_point) noexcept;

}  // namespace planner::math
