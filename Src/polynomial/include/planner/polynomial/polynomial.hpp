#pragma once

// 分段局部时间上的一维多项式，以升幂系数存储
// （Docs/component_development_plan.md 第 6.1、6.2 节，“多项式”行）：
//
//     p(t) = c0 + c1*t + c2*t^2 + ...      t 单位为秒，分段局部
//
// 系数相对于以秒为单位的局部时间轴保存；分段的 [0, duration] 定义域由
// TrajectorySegment 强制执行。这与
// tests/fixtures/v1/playback/polynomial_curve.json 一致，其中
// smoothstep 系数按 1/duration^3 缩放。
//
// 求值使用 Horner 法则。导数为解析形式：系数向量的 k 阶导数为
//     d[k]_i = (i+k)! / i! * c_{i+k}。

#include <cstddef>
#include <vector>

#include "planner/polynomial/result.hpp"

namespace planner::polynomial {

// 单个三次分段（单轴）的位置/速度边界条件。
struct CubicBoundary {
  double p0_m{0.0};
  double v0_mps{0.0};
  double p1_m{0.0};
  double v1_mps{0.0};
};

// 单个五次分段（单轴）的位置/速度/加速度边界条件。
struct QuinticBoundary {
  double p0_m{0.0};
  double v0_mps{0.0};
  double a0_mps2{0.0};
  double p1_m{0.0};
  double v1_mps{0.0};
  double a1_mps2{0.0};
};

class Polynomial {
 public:
  // 零多项式（系数 {0.0}）。系数列表会归一化：
  // 空输入向量存储为 {0.0}。
  Polynomial() = default;
  explicit Polynomial(std::vector<double> coefficients_ascending);

  // 求解 p(0)=p0, p'(0)=v0, p(T)=p1, p'(T)=v1，系数以秒为单位的
  // 局部时间 t 表示。时长为非正或非有限、或边界值非有限时
  // 返回 kInvalidInput。
  static Result<Polynomial> cubic(const CubicBoundary& boundary, double duration_s);

  // 求解 p(0)=p0, p'(0)=v0, p''(0)=a0, p(T)=p1, p'(T)=v1, p''(T)=a1。
  static Result<Polynomial> quintic(const QuinticBoundary& boundary, double duration_s);

  std::size_t degree() const noexcept;  // 零多项式为 0
  const std::vector<double>& coefficients() const noexcept;
  bool is_finite() const noexcept;

  // Horner 求值；零多项式在任意点求值均为 0。
  double evaluate(double t_s) const noexcept;

  // 解析导数多项式（阶数 0..kMaxDerivativeOrder）。
  // order > kMaxDerivativeOrder 返回 kUnsupportedDerivative；
  // 负阶数返回 kInvalidInput。
  Result<Polynomial> derivative(int order) const;

  // 对解析导数做 Horner 求值。结果为 t_s 处的 d^order p / dt^order。
  // order > kMaxDerivativeOrder 返回 kUnsupportedDerivative；
  // 负阶数或非有限时间返回 kInvalidInput。
  Result<double> evaluate_derivative(double t_s, int order) const noexcept;

  // 将导数批处理求值到调用方预分配的 `count` 个 double 缓冲区。
  // 按顺序求值所有输入，并在第一个被拒绝的值（非有限时间或
  // 不支持的阶数）处停止。
  BatchResult evaluate_batch(const double* times_s, std::size_t count,
                             int derivative_order, double* output) const noexcept;

  // 时间缩放：q(t) = p(t / time_scale)；q_i = c_i / time_scale^i。
  // 路径形状不变，时间轴按 `time_scale` 拉伸。
  // 要求 time_scale 有限且 > 0。
  Result<Polynomial> scaled_time(double time_scale) const;

 private:
  std::vector<double> coefficients_{0.0};
};

}  // namespace planner::polynomial
