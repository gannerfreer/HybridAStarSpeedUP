/**
 * @file segment2d.cpp
 * @brief 二维线段长度与方向查询实现。
 */

#include "planner/math/segment2d.hpp"

namespace planner::math {

double Segment2d::length_sq() const noexcept {
  return (end - start).norm_sq();
}

double Segment2d::length() const noexcept { return (end - start).norm(); }

Vec2d Segment2d::unit_direction() const noexcept {
  return (end - start).normalized_or_zero();
}

}  // namespace planner::math
