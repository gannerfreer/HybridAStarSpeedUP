#pragma once

namespace planner::math {

/**
 * @brief 车辆后轴中心在局部右手坐标系下的二维位姿。
 * @details X 轴指向车体正前方，Y 轴指向车体左侧，航向角 yaw 沿逆时针方向为正（弧度）。
 */
struct Pose2d {
  double x_m{0.0};      ///< 后轴中心 X 坐标（米）。
  double y_m{0.0};      ///< 后轴中心 Y 坐标（米）。
  double yaw_rad{0.0};  ///< 车身朝向角（弧度），归一化在 [-pi, pi) 区间。
};

inline constexpr bool operator==(const Pose2d& lhs, const Pose2d& rhs) {
  return lhs.x_m == rhs.x_m && lhs.y_m == rhs.y_m && lhs.yaw_rad == rhs.yaw_rad;
}

}  // namespace planner::math
