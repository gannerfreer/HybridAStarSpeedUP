// planner_bspline 的 EGO-Planner UniformBspline 对照后端（仅对照，GPL-3.0）。
//
// 本文件是原创适配层，包装上游 `Src/bspline/src/upstream_ego/` 的
// UniformBspline（来源与许可证见 PROVENANCE.md），使其满足内部 Backend
// 契约（显式节点、域内求值、右侧分段、解析导数句柄）。适配层与上游代码
// 一起只由对照库 planner_bspline_ego 编译，不进入 planner_bspline 主库。
//
// 与上游语义的差异及补偿（对照测试锁定，见 tests/test_backend_ego.cpp）：
//   * 上游 evaluateDeBoor 在内部节点处选择左侧分段（`u_(k+1) >= ub`），
//     本组件契约要求右侧分段。适配层仅在参数恰为内部节点时用
//     `nextafter(u, +inf)` 把上游的分段选择推到右侧；节点参数处的偏差为
//     1 ulp 量级（对照测试实测 ≤ 1.4e-13，容差 1e-9）。
//   * 上游对域外参数做夹持而非拒绝；适配层先做与 Eigen 后端一致的域检查。
//   * 上游构造函数只生成均匀节点，任意显式节点通过公开 setKnot 注入。
//   * 上游导数控制点在位置间断（重数 degree + 1）处含 inf 项，但求值的
//     分段选择不会引用它们；对照测试覆盖 gap/C0 用例。

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>

#include "backend.hpp"
#include "bspline_opt/uniform_bspline.h"

namespace planner::bspline::detail {

namespace {

using EgoSpline = ego_planner::UniformBspline;

// 上游 UniformBspline 的不可变视图：样条 + 累积导阶 + 参数子域。
// 上游 evaluateDeBoor 未标记 const，因此持有非 const 共享指针；句柄按约定
// 在构造后不再修改样条内容（对外仍表现为不可变句柄）。
class EgoBackend final : public Backend {
 public:
  EgoBackend(std::shared_ptr<EgoSpline> spline, int degree,
             int derivative_order, double domain_min, double domain_max,
             std::vector<double> explicit_knots)
      : spline_(std::move(spline)),
        degree_(degree),
        derivative_order_(derivative_order),
        domain_min_(domain_min),
        domain_max_(domain_max),
        explicit_knots_(std::move(explicit_knots)) {}

  BackendKind kind() const noexcept override {
    return BackendKind::kEgoUniform;
  }
  const char* name() const noexcept override { return "ego_uniform"; }

  bool eval(double u, bool select_right,
            common::Point2d* out) const override {
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
    // 右侧分段补偿：上游在参数恰为节点值时选择左侧分段；把求值参数推过
    // 该节点（1 ulp）即可让其选择右侧分段。仅处理严格位于视图域内的节点
    // 值；域右端点上游本身就取最后分段。
    double t = u;
    if (select_right && u < domain_max_ && is_explicit_knot(u)) {
      t = std::nextafter(u, std::numeric_limits<double>::infinity());
    }
    const Eigen::VectorXd value = spline_->evaluateDeBoor(t);
    if (value.size() < 2 || !std::isfinite(value(0)) ||
        !std::isfinite(value(1))) {
      return false;
    }
    out->x_m = value(0);
    out->y_m = value(1);
    return true;
  }

  std::unique_ptr<Backend> derive(int order) const override {
    if (order < 1 || derivative_order_ + order > degree_) {
      return nullptr;
    }
    auto spline =
        std::make_shared<EgoSpline>(*spline_);  // 复制当前（可导）句柄
    for (int i = 0; i < order; ++i) {
      spline = std::make_shared<EgoSpline>(spline->getDerivative());
    }
    return std::make_unique<EgoBackend>(std::move(spline), degree_,
                                        derivative_order_ + order, domain_min_,
                                        domain_max_, explicit_knots_);
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
    return std::make_unique<EgoBackend>(spline_, degree_, derivative_order_,
                                        domain_min, domain_max,
                                        explicit_knots_);
  }

  std::unique_ptr<Backend> clone() const override {
    return std::make_unique<EgoBackend>(spline_, degree_, derivative_order_,
                                        domain_min_, domain_max_,
                                        explicit_knots_);
  }

 private:
  // 参数是否恰为显式节点向量中的值（构造时传入的原始缩放节点）。
  bool is_explicit_knot(double u) const noexcept {
    for (const double knot : explicit_knots_) {
      if (knot == u) {
        return true;
      }
    }
    return false;
  }

  std::shared_ptr<EgoSpline> spline_;
  int degree_{0};
  int derivative_order_{0};
  double domain_min_{0.0};
  double domain_max_{0.0};
  // 原始显式节点向量（缩放单位）；导数字句沿用同一份节点做右侧判定。
  std::vector<double> explicit_knots_;
};

}  // namespace

std::shared_ptr<const Backend> make_ego_backend(
    const std::vector<common::Point2d>& control_points,
    const std::vector<double>& scaled_knots, int degree,
    std::string* error_message) {
  const std::size_t control_point_count = control_points.size();
  if (degree < 1 || scaled_knots.size() != control_point_count +
                                            static_cast<std::size_t>(degree) +
                                            1) {
    if (error_message != nullptr) {
      *error_message = "EGO UniformBspline backend received an invalid spline "
                       "description";
    }
    return nullptr;
  }

  Eigen::MatrixXd points(2, static_cast<Eigen::Index>(control_point_count));
  for (std::size_t i = 0; i < control_point_count; ++i) {
    points(0, static_cast<Eigen::Index>(i)) = control_points[i].x_m;
    points(1, static_cast<Eigen::Index>(i)) = control_points[i].y_m;
  }
  Eigen::VectorXd knots(static_cast<Eigen::Index>(scaled_knots.size()));
  for (std::size_t i = 0; i < scaled_knots.size(); ++i) {
    knots(static_cast<Eigen::Index>(i)) = scaled_knots[i];
  }

  // 上游构造函数只生成均匀节点；随后用公开 setKnot 注入显式节点向量
  //（两者规模一致：knots.size() == n_ctrlp + degree + 1）。
  EgoSpline spline(points, degree, 1.0);
  spline.setKnot(knots);

  const double domain_min =
      scaled_knots[static_cast<std::size_t>(degree)];
  const double domain_max = scaled_knots[control_point_count];
  return std::make_shared<EgoBackend>(
      std::make_shared<EgoSpline>(std::move(spline)), degree, 0, domain_min,
      domain_max, scaled_knots);
}

void install_ego_backend() {
  set_ego_backend_factory(&make_ego_backend);
}

}  // namespace planner::bspline::detail
