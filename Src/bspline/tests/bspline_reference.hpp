#pragma once

// 仅供 planner_bspline 测试使用的独立参考实现。
// 它们不调用被测组件：采用右侧分段约定的 de Boor 求值、
// 基于 hodograph 的导数，以及对可分解为 Bezier 分段的曲线
// 进行解析 Bezier 求值。

#include <cmath>
#include <cstddef>
#include <vector>

#include "planner/common/types.hpp"

namespace planner::bspline::test {

inline common::Point2d add(const common::Point2d& a, const common::Point2d& b) {
  return common::Point2d{a.x_m + b.x_m, a.y_m + b.y_m};
}

inline common::Point2d sub(const common::Point2d& a, const common::Point2d& b) {
  return common::Point2d{a.x_m - b.x_m, a.y_m - b.y_m};
}

inline common::Point2d scale(const common::Point2d& a, double factor) {
  return common::Point2d{a.x_m * factor, a.y_m * factor};
}

inline double norm(const common::Point2d& a) {
  return std::hypot(a.x_m, a.y_m);
}

// 标准的夹持分段查找：在重复节点处返回右侧分段
//（NURBS 书中的 FindSpan，上界处返回最后一个分段）。
inline int reference_find_span(int degree, int control_point_count, double u,
                               const std::vector<double>& knots) {
  const int last = control_point_count - 1;
  if (u >= knots[static_cast<std::size_t>(control_point_count)]) {
    return last;
  }
  int low = degree;
  int high = control_point_count;
  int mid = (low + high) / 2;
  while (u < knots[static_cast<std::size_t>(mid)] ||
         u >= knots[static_cast<std::size_t>(mid + 1)]) {
    if (u < knots[static_cast<std::size_t>(mid)]) {
      high = mid;
    } else {
      low = mid;
    }
    mid = (low + high) / 2;
  }
  return mid;
}

inline common::Point2d reference_de_boor(
    const std::vector<common::Point2d>& control_points,
    const std::vector<double>& knots, int degree, double u) {
  const int count = static_cast<int>(control_points.size());
  const int span =
      reference_find_span(degree, count, u, knots);
  std::vector<common::Point2d> d(static_cast<std::size_t>(degree) + 1);
  for (int j = 0; j <= degree; ++j) {
    d[static_cast<std::size_t>(j)] =
        control_points[static_cast<std::size_t>(span - degree + j)];
  }
  for (int r = 1; r <= degree; ++r) {
    for (int j = degree; j >= r; --j) {
      const int base = span - degree + j;
      const double lower = knots[static_cast<std::size_t>(base)];
      const double upper =
          knots[static_cast<std::size_t>(base + degree - r + 1)];
      const double alpha =
          upper > lower ? (u - lower) / (upper - lower) : 0.0;
      d[static_cast<std::size_t>(j)] =
          add(scale(d[static_cast<std::size_t>(j - 1)], 1.0 - alpha),
              scale(d[static_cast<std::size_t>(j)], alpha));
    }
  }
  return d[static_cast<std::size_t>(degree)];
}

// u 处的 `order` 阶导数。hodograph 按标准公式构造；
// 零长度节点区间贡献为零（对连续曲线及 C0 连接处的右侧求值成立）。
inline common::Point2d reference_evaluate(
    const std::vector<common::Point2d>& control_points,
    const std::vector<double>& knots, int degree, double u, int order) {
  if (order < 0) {
    return common::Point2d{0.0, 0.0};
  }
  std::vector<common::Point2d> points = control_points;
  std::vector<double> current_knots = knots;
  int current_degree = degree;
  for (int step = 0; step < order; ++step) {
    if (current_degree <= 0) {
      return common::Point2d{0.0, 0.0};
    }
    std::vector<common::Point2d> derived(points.size() - 1);
    for (std::size_t i = 0; i + 1 < points.size(); ++i) {
      const double lower = current_knots[i + 1];
      const double upper =
          current_knots[i + static_cast<std::size_t>(current_degree) + 1];
      const double denom = upper - lower;
      if (denom > 0.0) {
        derived[i] = scale(sub(points[i + 1], points[i]),
                           static_cast<double>(current_degree) / denom);
      } else {
        derived[i] = common::Point2d{0.0, 0.0};
      }
    }
    current_knots.erase(current_knots.begin());
    current_knots.pop_back();
    points = std::move(derived);
    current_degree -= 1;
  }
  if (current_degree < 0) {
    return common::Point2d{0.0, 0.0};
  }
  return reference_de_boor(points, current_knots, current_degree, u);
}

// 局部参数 t（[0, 1]）处的解析 Bezier 值（de Casteljau 算法）。
inline common::Point2d bezier_value(
    const std::vector<common::Point2d>& control_points, double t) {
  std::vector<common::Point2d> d = control_points;
  for (std::size_t r = d.size(); r > 1; --r) {
    for (std::size_t i = 0; i + 1 < r; ++i) {
      d[i] = add(scale(d[i], 1.0 - t), scale(d[i + 1], t));
    }
  }
  return d.front();
}

// Bezier 分段在参数宽度为 `span` 的区间上的解析 order 阶导数，
// 在局部参数 t 处取值。
inline common::Point2d bezier_derivative(
    const std::vector<common::Point2d>& control_points, double t, int order,
    double span) {
  std::vector<common::Point2d> d = control_points;
  int degree = static_cast<int>(d.size()) - 1;
  for (int step = 0; step < order; ++step) {
    if (degree <= 0) {
      return common::Point2d{0.0, 0.0};
    }
    std::vector<common::Point2d> next(static_cast<std::size_t>(degree));
    for (int i = 0; i < degree; ++i) {
      next[static_cast<std::size_t>(i)] =
          scale(sub(d[static_cast<std::size_t>(i + 1)],
                    d[static_cast<std::size_t>(i)]),
                static_cast<double>(degree) / span);
    }
    d = std::move(next);
    degree -= 1;
  }
  return bezier_value(d, t);
}

}  // namespace planner::bspline::test
