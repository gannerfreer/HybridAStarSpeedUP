#pragma once

namespace planner::math {

/**
 * @brief 双精度圆周率常量，用于角度换算与航向角归一化。
 */
inline constexpr double kPi = 3.14159265358979323846;

// 有限输入归一化到 [-pi, pi)，非有限输入返回 NaN。
double normalize_angle_rad(double angle_rad);

}  // namespace planner::math
