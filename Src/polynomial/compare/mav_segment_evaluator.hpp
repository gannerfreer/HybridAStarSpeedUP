#pragma once

// planner_polynomial 的内部对照求值器（不随包安装、不进入公开 API）。
//
// 基于锁定的 mav_trajectory_generation（计划 §3.4 的“分段多项式表达、
// 求值与连续性处理参考”）提供与 planner::polynomial 相同的求值语义：
//   * 升幂系数、分段局部时间（秒）；
//   * 求导阶数 0..3；
//   * 时间超出 [0, duration_s] 拒绝，不隐式外推。
// 上游类型（Eigen / mav_trajectory_generation）由实现文件隔离。

#include <cstddef>
#include <memory>
#include <vector>

#include "planner/common/types.hpp"

namespace planner::polynomial::compare {

class MavSegmentEvaluator {
 public:
  // 由两个轴的升幂系数构造。系数数量必须一致且 >= 2；
  // duration_s 必须有限且为正；否则返回 false（不写入 out）。
  static bool create(const std::vector<double>& x_coefficients_ascending,
                     const std::vector<double>& y_coefficients_ascending,
                     double duration_s, MavSegmentEvaluator* out);

  bool valid() const noexcept;
  double duration_s() const noexcept;

  // 在局部时间 t_s ∈ [0, duration_s] 处求第 derivative_order 阶导数。
  // 越界时间或阶数超出 0..3 返回 false。
  bool evaluate(double t_s, int derivative_order,
                planner::common::Point2d* out) const;

  // 批量求值：等价于对每个时间点调用 evaluate。
  bool evaluate_batch(const double* times_s, std::size_t count,
                      int derivative_order,
                      planner::common::Point2d* out) const;

 private:
  struct Impl;
  std::shared_ptr<const Impl> impl_;
};

}  // namespace planner::polynomial::compare
