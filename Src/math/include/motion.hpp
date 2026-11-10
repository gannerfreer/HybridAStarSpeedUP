#pragma once
#include "angle.hpp"
#include "motion_internal.hpp"
namespace planner::math {
// s 为有向弧长：前进正、倒车负。调用方负责检查输入有限性。
inline common::Pose2d integrate_arc(const common::Pose2d& start, double s, double curvature) noexcept {
  auto out = internal::advance_arc_trig_unchecked(start, s, curvature,
                                                 std::cos(start.yaw_rad), std::sin(start.yaw_rad));
  out.yaw_rad = normalize_angle_rad(out.yaw_rad);
  return out;
}
// 供扫掠递归复用起点三角函数值。
inline common::Pose2d integrate_arc_trig(const common::Pose2d& start, double s, double curvature,
                                        double cos_yaw, double sin_yaw) noexcept {
  auto out = internal::advance_arc_trig_unchecked(start, s, curvature, cos_yaw, sin_yaw);
  out.yaw_rad = normalize_angle_rad(out.yaw_rad);
  return out;
}
}
