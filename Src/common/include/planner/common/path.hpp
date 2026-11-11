/**
 * @file path.hpp
 * @brief 路径行驶方向、采样点与连续路径段定义
 * @details 统一物理量单位规范：距离采用米 (m)，时间采用秒 (s)，角度采用弧度 (rad)
 *          局部笛卡尔坐标系统一采用右手系（x 轴朝前，y 轴朝左，逆时针航向角为正）
 */

#pragma once

#include <string_view>
#include <vector>

#include "planner/math/pose2d.hpp"

namespace planner::common {

using math::Pose2d;

/**
 * @brief 路径点或运动轨迹分段的行驶方向（前进/倒车）
 */
enum class Direction {
  kForward,  ///< 前进挡位
  kReverse,  ///< 倒车挡位
};

/**
 * @brief 将行驶方向枚举转换为易读的字符串形式
 * @param[in] direction 行驶方向枚举值
 * @return 对应的字符串视图（"forward" 或 "reverse"）；未知枚举值返回 "unknown"
 */
[[nodiscard]] std::string_view to_string(Direction direction) noexcept;

/**
 * @brief 车辆轨迹路径采样点
 * @details `s_m` 为从路径起点算起的非负累计弧长（米）
 *          `curvature_per_m` 为有符号曲率 \f$ \kappa = \tan(\delta) / L \f$，
 *          满足运动学关系 \f$ \dot{\psi} = v_{\text{signed}} \cdot \kappa \f$
 */
struct PathPoint {
  Pose2d pose;                  ///< 采样点处的车辆后轴中心位姿
  double s_m{0.0};              ///< 沿路径的累计弧长（米）
  double curvature_per_m{0.0};  ///< 有符号路径曲率（1/m，左转为正，右转为负）
  Direction direction{Direction::kForward};  ///< 该点对应的行驶方向（前进/倒车）
};

/**
 * @brief 具有统一行驶挡位的连续路径段
 * @details 挡位切换（折返尖点 cusp）处必定分割并启动新的路径段段内累计弧长 `s_m` 严格单调递增
 */
struct PathSegment {
  std::vector<PathPoint> points;  ///< 本段内有序排列的路径采样点序列
};

/**
 * @name 精确全等比较操作符
 * @brief 规划核心数据结构的值相等性判断（用于缓存校验、回放对比与结果校验）
 * @{
 */
bool operator==(const PathPoint& lhs, const PathPoint& rhs);
bool operator==(const PathSegment& lhs, const PathSegment& rhs);
/// @}

}  // namespace planner::common
