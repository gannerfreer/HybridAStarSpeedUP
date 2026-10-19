#pragma once

// planner_bspline GoogleTest 测试套件的共享辅助函数。

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "planner/bspline/bspline.hpp"
#include "planner/common/types.hpp"

namespace planner::bspline::test {

inline bool near_value(double actual, double expected, double abs_tol = 1e-9,
                       double rel_tol = 1e-9) {
  return std::fabs(actual - expected) <=
         abs_tol + rel_tol * std::fabs(expected);
}

inline bool near_point(const common::Point2d& actual,
                       const common::Point2d& expected,
                       double abs_tol = 1e-9, double rel_tol = 1e-9) {
  return near_value(actual.x_m, expected.x_m, abs_tol, rel_tol) &&
         near_value(actual.y_m, expected.y_m, abs_tol, rel_tol);
}

inline Bspline require_curve(std::vector<common::Point2d> control_points,
                             std::vector<double> knots) {
  std::string message;
  std::optional<Bspline> curve =
      Bspline::create(std::move(control_points), std::move(knots), &message);
  if (!curve.has_value()) {
    throw std::runtime_error("curve construction failed: " + message);
  }
  return std::move(*curve);
}

inline Bspline require_time_curve(std::vector<common::Point2d> control_points,
                                  std::vector<double> knots, double start_s,
                                  double end_s) {
  std::string message;
  std::optional<Bspline> curve = Bspline::create_time_parameterized(
      std::move(control_points), std::move(knots), start_s, end_s, &message);
  if (!curve.has_value()) {
    throw std::runtime_error("time curve construction failed: " + message);
  }
  return std::move(*curve);
}

// 作为 B-spline 的夹持三次 Bezier（单区间）。
inline Bspline bezier_cubic(std::vector<common::Point2d> points) {
  return require_curve(std::move(points), {0.0, 0.0, 0.0, 0.0,
                                           1.0, 1.0, 1.0, 1.0});
}

// 两段三次 Bezier 共享 P3，在 0.5 处形成 C0 连接。
inline Bspline two_segment_c0() {
  return require_curve({{0.0, 0.0}, {1.0, 2.0}, {3.0, 2.0}, {4.0, 0.0},
                        {5.0, 2.0}, {7.0, 2.0}, {8.0, 0.0}},
                       {0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5,
                        1.0, 1.0, 1.0, 1.0});
}

// 两段独立的三次 Bezier 在 0.5 处存在位置间断
//（内部节点重数 4）。
inline Bspline two_segment_gap() {
  return require_curve({{0.0, 0.0}, {1.0, 2.0}, {3.0, 2.0}, {4.0, 0.0},
                        {5.0, 2.0}, {7.0, 2.0}, {8.0, 0.0}, {9.0, -3.0}},
                       {0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.5, 0.5,
                        1.0, 1.0, 1.0, 1.0});
}

#define EXPECT_POINT_NEAR(actual, expected, tol)  \
  EXPECT_NEAR((actual).x_m, (expected).x_m, tol); \
  EXPECT_NEAR((actual).y_m, (expected).y_m, tol)

}  // namespace planner::bspline::test
