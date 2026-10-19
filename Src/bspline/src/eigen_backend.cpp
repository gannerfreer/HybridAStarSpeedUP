// planner_bspline 的 Eigen Splines 后端实现（内部，对照候选）。
//
// 使用 Eigen unsupported 模块的 B-spline（显式节点向量 + 控制点）。
// Eigen 的 Span 选择（Piegl & Tiller A2.1，upper_bound）在内部节点处
// 返回右侧分段、在域末端返回最后分段，与组件契约一致；导数为对右侧
// 分段的单侧导数。
//
// 与 TinySpline 后端的差异：
//   * 无节点 epsilon 问题，仍沿用统一的缩放域以共享上层换算；
//   * derive/restrict 是零拷贝视图（累积导阶 + 子域），求值时按需
//     计算基函数导数，不做样条重参数化。

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <utility>

#include <unsupported/Eigen/Splines>

#include "backend.hpp"

namespace planner::bspline::detail {

namespace {

using EigenSpline = Eigen::Spline<double, 2>;  // 动态次数

// 视图句柄：同一 Eigen 样条 + 累积导阶 + 参数子域。
class EigenBackend final : public Backend {
 public:
  EigenBackend(std::shared_ptr<const EigenSpline> spline, int degree,
               int derivative_order, double domain_min, double domain_max)
      : spline_(std::move(spline)),
        degree_(degree),
        derivative_order_(derivative_order),
        domain_min_(domain_min),
        domain_max_(domain_max) {}

  BackendKind kind() const noexcept override {
    return BackendKind::kEigenSplines;
  }
  const char* name() const noexcept override { return "eigen_splines"; }

  bool eval(double u, bool select_right,
            common::Point2d* out) const override {
    // Eigen 的 Span 已保证右侧分段语义；select_right 不改变结果。
    (void)select_right;
    if (out == nullptr || !std::isfinite(u)) {
      return false;
    }
    const double tolerance =
        1e-9 * std::max(1.0, std::max(std::fabs(domain_min_),
                                      std::fabs(domain_max_)));
    if (u < domain_min_ - tolerance || u > domain_max_ + tolerance) {
      return false;
    }
    u = std::min(std::max(u, domain_min_), domain_max_);
    if (derivative_order_ > degree_) {
      *out = common::Point2d{0.0, 0.0};
      return true;
    }
    const auto derivative_values =
        spline_->derivatives(u, derivative_order_);
    if (derivative_values.cols() <= derivative_order_) {
      // 防御：Eigen 未给出请求阶数（不应发生，输入已校验）。
      *out = common::Point2d{0.0, 0.0};
      return true;
    }
    out->x_m = derivative_values(0, derivative_order_);
    out->y_m = derivative_values(1, derivative_order_);
    return true;
  }

  std::unique_ptr<Backend> derive(int order) const override {
    if (order < 1) {
      return nullptr;
    }
    return std::make_unique<EigenBackend>(spline_, degree_,
                                          derivative_order_ + order,
                                          domain_min_, domain_max_);
  }

  std::unique_ptr<Backend> restrict(double from, double to) const override {
    if (!(from < to)) {
      return nullptr;
    }
    const double domain_min = std::max(domain_min_, from);
    const double domain_max = std::min(domain_max_, to);
    if (!(domain_min < domain_max)) {
      return nullptr;
    }
    return std::make_unique<EigenBackend>(spline_, degree_, derivative_order_,
                                          domain_min, domain_max);
  }

  std::unique_ptr<Backend> clone() const override {
    return std::make_unique<EigenBackend>(spline_, degree_, derivative_order_,
                                          domain_min_, domain_max_);
  }

 private:
  std::shared_ptr<const EigenSpline> spline_;
  int degree_{0};
  int derivative_order_{0};
  double domain_min_{0.0};
  double domain_max_{0.0};
};

}  // namespace

std::shared_ptr<const Backend> make_eigen_backend(
    const std::vector<common::Point2d>& control_points,
    const std::vector<double>& scaled_knots, int degree) {
  const std::size_t control_point_count = control_points.size();
  if (degree < 1 || scaled_knots.size() != control_point_count +
                                            static_cast<std::size_t>(degree) +
                                            1) {
    return nullptr;
  }

  EigenSpline::KnotVectorType knots(
      static_cast<Eigen::Index>(scaled_knots.size()));
  for (std::size_t i = 0; i < scaled_knots.size(); ++i) {
    knots(static_cast<Eigen::Index>(i)) = scaled_knots[i];
  }

  EigenSpline::ControlPointVectorType ctrls(
      2, static_cast<Eigen::Index>(control_point_count));
  for (std::size_t i = 0; i < control_point_count; ++i) {
    ctrls(0, static_cast<Eigen::Index>(i)) = control_points[i].x_m;
    ctrls(1, static_cast<Eigen::Index>(i)) = control_points[i].y_m;
  }

  auto spline = std::make_shared<const EigenSpline>(knots, ctrls);
  const double domain_min =
      scaled_knots[static_cast<std::size_t>(degree)];
  const double domain_max = scaled_knots[control_point_count];
  return std::make_shared<EigenBackend>(std::move(spline), degree, 0,
                                        domain_min, domain_max);
}

}  // namespace planner::bspline::detail
