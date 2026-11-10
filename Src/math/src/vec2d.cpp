/**
 * @file vec2d.cpp
 * @brief 二维向量基础代数实现。
 */

#include "planner/math/vec2d.hpp"

namespace planner::math {

double cross(const Point2d& a, const Point2d& b) noexcept {
  return a.cross(b);
}

double dot(const Point2d& a, const Point2d& b) noexcept { return a.dot(b); }

double norm(const Point2d& a) noexcept { return a.norm(); }

double distance(const Point2d& a, const Point2d& b) noexcept {
  return (a - b).norm();
}

}  // namespace planner::math
