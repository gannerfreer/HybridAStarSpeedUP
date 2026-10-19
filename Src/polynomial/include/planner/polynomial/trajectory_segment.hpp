#pragma once

// 二维轨迹分段：每个轴一个三次或五次多项式，在分段局部时间定义域
// [0, duration_s] 上求值（Docs/component_development_plan.md 第 6.1、6.2 节）。
//
// 位置和导数向量（速度、加速度）复用公开的
// planner::common::Point2d 载荷类型（x_m/y_m 是数值槽位；导数向量的单位
// 由请求的求导阶数决定）。

#include <cstddef>

#include "planner/common/types.hpp"
#include "planner/polynomial/polynomial.hpp"
#include "planner/polynomial/result.hpp"

namespace planner::polynomial {

// 三次分段边界条件（逐轴），与
// tests/fixtures/v1/playback/polynomial_curve.json 的字段名一致。
struct CubicBoundary2d {
  planner::common::Point2d p0_m;
  planner::common::Point2d v0_mps;
  planner::common::Point2d p1_m;
  planner::common::Point2d v1_mps;
};

// 五次分段边界条件（逐轴）。
struct QuinticBoundary2d {
  planner::common::Point2d p0_m;
  planner::common::Point2d v0_mps;
  planner::common::Point2d a0_mps2;
  planner::common::Point2d p1_m;
  planner::common::Point2d v1_mps;
  planner::common::Point2d a1_mps2;
};

class TrajectorySegment {
 public:
  // 空/无效分段；只有下面的工厂函数产生有效分段。
  TrajectorySegment() = default;

  // 用三次（位置/速度）边界条件求解两个轴。
  static Result<TrajectorySegment> cubic(const CubicBoundary2d& boundary,
                                         double duration_s);

  // 用五次（位置/速度/加速度）边界条件求解两个轴。
  static Result<TrajectorySegment> quintic(const QuinticBoundary2d& boundary,
                                           double duration_s);

  bool valid() const noexcept;
  double duration_s() const noexcept;

  // 三次为 3，五次为 5，空分段为 0。
  int degree() const noexcept;

  const Polynomial& x_polynomial() const noexcept;
  const Polynomial& y_polynomial() const noexcept;

  // 在局部时间 [0, duration_s] 内求第 derivative_order 阶时间导数。
  // 阶数 0 返回位置；阶数 1 返回速度向量；阶数 2 返回加速度向量。
  // 时间超出局部定义域时返回 kOutOfRange，不做外推；
  // 阶数超出第一版范围时返回 kUnsupportedDerivative。
  Result<planner::common::Point2d> evaluate(double local_time_s,
                                            int derivative_order = 0) const noexcept;

  // 批处理求值，使用调用方预分配的 `count` 个点的输出缓冲区。
  // 在第一个被拒绝的局部时间处停止并报告其索引。
  BatchResult evaluate_batch(const double* local_times_s, std::size_t count,
                             int derivative_order,
                             planner::common::Point2d* output) const noexcept;

  // 时间缩放：每个轴都满足 q(t) = p(t / time_scale)；分段时长乘以
  // time_scale，边界速度除以 time_scale。
  // 要求 time_scale 有限且 > 0。
  Result<TrajectorySegment> scaled_time(double time_scale) const;

 private:
  TrajectorySegment(Polynomial x, Polynomial y, double duration_s, int degree);

  Polynomial x_;
  Polynomial y_;
  double duration_s_{0.0};
  int degree_{0};
};

}  // namespace planner::polynomial
