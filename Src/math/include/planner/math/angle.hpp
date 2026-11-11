/**
 * @file angle.hpp
 * @brief 角度常量、归一化与单位换算辅助函数
 */

#pragma once

namespace planner::math {

/**
 * @brief 双精度圆周率常量，用于角度换算与航向角归一化
 */
inline constexpr double kPi = 3.14159265358979323846;

// 有限输入归一化到 [-pi, pi)，非有限输入返回 NaN
[[nodiscard]] double normalize_angle_rad(double angle_rad);

/**
 * @brief 计算两角之差并归一化到 [-pi, pi) 区间
 * @param[in] a_rad 被减角（弧度）
 * @param[in] b_rad 减数角（弧度）
 * @return 归一化后的角度差（弧度）；非有限输入返回 NaN
 */
[[nodiscard]] double angle_diff_rad(double a_rad, double b_rad) noexcept;

/**
 * @brief 将角度由度转换为弧度
 * @param[in] angle_deg 角度（度）
 * @return 对应弧度值
 */
inline constexpr double deg_to_rad(double angle_deg) noexcept { return angle_deg * (kPi / 180.0); }

/**
 * @brief 将角度由弧度转换为度
 * @param[in] angle_rad 角度（弧度）
 * @return 对应角度值（度）
 */
inline constexpr double rad_to_deg(double angle_rad) noexcept { return angle_rad * (180.0 / kPi); }

}  // namespace planner::math
