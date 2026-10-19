#include "planner/bspline/curve.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "backend.hpp"
#include "impl.hpp"

namespace planner::bspline {

namespace {

constexpr int kMaxDerivativeOrder = 3;
constexpr double kDomainRoundingTolerance = 1e-12;

bool is_finite_point(const common::Point2d& point) noexcept {
  return std::isfinite(point.x_m) && std::isfinite(point.y_m);
}

std::string describe_parameter_error(double parameter) {
  return "parameter " + std::to_string(parameter) +
         " is outside the explicit parameter domain";
}

// 将公开参数（几何节点参数或时间）映射为节点参数。
// 与参数域边界相差在极小相对舍入容差内的值会被夹到边界；
// 更远的值则被拒绝。
bool map_to_knot_parameter(const detail::SplineImpl& impl, double parameter,
                           double* knot_parameter, std::string* message) {
  if (!std::isfinite(parameter)) {
    if (message != nullptr) {
      *message = "parameter must be finite";
    }
    return false;
  }
  double lower = impl.knot_min;
  double upper = impl.knot_max;
  if (impl.parameter_kind == ParameterKind::kTime) {
    lower = impl.time_min;
    upper = impl.time_max;
  }
  const double tolerance =
      kDomainRoundingTolerance * std::max(1.0, std::max(std::fabs(lower),
                                                        std::fabs(upper)));
  if (parameter < lower - tolerance || parameter > upper + tolerance) {
    if (message != nullptr) {
      *message = describe_parameter_error(parameter);
    }
    return false;
  }
  const double clamped = std::min(std::max(parameter, lower), upper);
  double mapped = clamped;
  if (impl.parameter_kind == ParameterKind::kTime) {
    const double scale = (impl.knot_max - impl.knot_min) / (upper - lower);
    mapped = impl.knot_min + (clamped - lower) * scale;
  }
  *knot_parameter = std::min(std::max(mapped, impl.knot_min), impl.knot_max);
  return true;
}

// 从后端（缩放）参数到公开参数的链式法则：
//   几何：d^k/du^k = (du'/du)^k * d^k/du'^k, du'/du = knot_scale
//   时间：d^k/dt^k = (du'/dt)^k * d^k/du'^k
double public_derivative_scale(const detail::SplineImpl& impl, int order) {
  if (order <= 0) {
    return 1.0;
  }
  double du_backend_dparameter = impl.knot_scale;
  if (impl.parameter_kind == ParameterKind::kTime) {
    du_backend_dparameter *= (impl.knot_max - impl.knot_min) /
                             (impl.time_max - impl.time_min);
  }
  return std::pow(du_backend_dparameter, static_cast<double>(order));
}

// 在内部节点处使用右侧分段对后端（缩放）参数求值。
// 当求导句柄会在求导链中合并间断时，先把求值限制到包含该参数的分段上；
// 该补偿对两个后端都成立：限制后求导只在分段内进行。
bool evaluate_scaled_parameter(const detail::SplineImpl& impl, double scaled,
                               int order, common::Point2d* value) {
  if (impl.backend == nullptr) {
    return false;
  }
  if (order == 0) {
    return impl.backend->eval(scaled, true, value);
  }
  const bool needs_restriction = detail::merge_possible(impl, order);
  const double start =
      needs_restriction ? detail::span_left(impl, scaled) : impl.scaled_min;
  if (start <= impl.scaled_min) {
    const std::unique_ptr<detail::Backend> derived =
        impl.backend->derive(order);
    if (derived == nullptr) {
      return false;
    }
    return derived->eval(scaled, true, value);
  }
  const std::unique_ptr<detail::Backend> base =
      impl.backend->restrict(start, impl.scaled_max);
  if (base == nullptr) {
    return false;
  }
  const std::unique_ptr<detail::Backend> derived = base->derive(order);
  if (derived == nullptr) {
    return false;
  }
  return derived->eval(scaled, true, value);
}

}  // namespace

// ---------------------------------------------------------------------------
// 状态
// ---------------------------------------------------------------------------

const char* to_string(Status status) noexcept {
  switch (status) {
    case Status::kSuccess:
      return "success";
    case Status::kInvalidInput:
      return "invalid_input";
    case Status::kParameterOutOfDomain:
      return "parameter_out_of_domain";
    case Status::kDerivativeOrderUnsupported:
      return "derivative_order_unsupported";
    case Status::kDegenerateCurve:
      return "degenerate_curve";
    case Status::kArcLengthToleranceExceeded:
      return "arc_length_tolerance_exceeded";
    case Status::kInternalError:
      return "internal_error";
  }
  return "unknown";
}

// ---------------------------------------------------------------------------
// Bspline 生命周期
// ---------------------------------------------------------------------------

Bspline::Bspline(std::unique_ptr<detail::SplineImpl> impl)
    : impl_(std::move(impl)) {}

Bspline::~Bspline() = default;

Bspline::Bspline(Bspline&&) noexcept = default;
Bspline& Bspline::operator=(Bspline&&) noexcept = default;

Bspline::Bspline(const Bspline& other)
    : impl_(other.impl_ ? std::make_unique<detail::SplineImpl>() : nullptr) {
  if (impl_ == nullptr) {
    return;
  }
  impl_->backend = other.impl_->backend;  // 不可变共享句柄
  impl_->control_points = other.impl_->control_points;
  impl_->knots = other.impl_->knots;
  impl_->scaled_knots = other.impl_->scaled_knots;
  impl_->degree = other.impl_->degree;
  impl_->knot_min = other.impl_->knot_min;
  impl_->knot_max = other.impl_->knot_max;
  impl_->scaled_min = other.impl_->scaled_min;
  impl_->scaled_max = other.impl_->scaled_max;
  impl_->knot_scale = other.impl_->knot_scale;
  impl_->knot_scale_inverse = other.impl_->knot_scale_inverse;
  impl_->parameter_kind = other.impl_->parameter_kind;
  impl_->time_min = other.impl_->time_min;
  impl_->time_max = other.impl_->time_max;
  impl_->gap_knots = other.impl_->gap_knots;
}

Bspline& Bspline::operator=(const Bspline& other) {
  if (this == &other) {
    return *this;
  }
  Bspline copy(other);
  impl_ = std::move(copy.impl_);
  return *this;
}

// ---------------------------------------------------------------------------
// 构造
// ---------------------------------------------------------------------------

std::optional<Bspline> Bspline::create(std::vector<common::Point2d> control_points,
                                       std::vector<double> knots,
                                       std::string* error_message) {
  return create(std::move(control_points), std::move(knots), kDefaultBackend,
                error_message);
}

std::optional<Bspline> Bspline::create(std::vector<common::Point2d> control_points,
                                       std::vector<double> knots,
                                       BackendKind backend,
                                       std::string* error_message) {
  const auto fail = [error_message](const std::string& message)
      -> std::optional<Bspline> {
    if (error_message != nullptr) {
      *error_message = message;
    }
    return std::nullopt;
  };

  if (control_points.size() < 2) {
    return fail("at least two control points are required");
  }
  if (knots.size() < control_points.size() + 2) {
    return fail("knot vector is too short for degree >= 1");
  }
  const int degree = static_cast<int>(knots.size()) -
                     static_cast<int>(control_points.size()) - 1;
  if (degree < 1) {
    return fail("degree must be at least 1");
  }
  if (static_cast<std::size_t>(degree) >= control_points.size()) {
    return fail("insufficient control points: degree must be smaller than the "
                "control point count");
  }
  for (std::size_t i = 0; i < control_points.size(); ++i) {
    if (!is_finite_point(control_points[i])) {
      return fail("control point " + std::to_string(i) + " is not finite");
    }
  }
  for (std::size_t i = 0; i < knots.size(); ++i) {
    if (!std::isfinite(knots[i])) {
      return fail("knot " + std::to_string(i) + " is not finite");
    }
    if (i > 0 && knots[i] < knots[i - 1]) {
      return fail("knot vector must be non-decreasing");
    }
  }
  for (std::size_t i = 0; i < knots.size();) {
    std::size_t j = i;
    while (j < knots.size() && knots[j] == knots[i]) {
      ++j;
    }
    if (static_cast<int>(j - i) > degree + 1) {
      return fail("knot multiplicity must not exceed degree + 1");
    }
    i = j;
  }

  const double knot_min = knots[static_cast<std::size_t>(degree)];
  const double knot_max = knots[control_points.size()];
  if (!(knot_max > knot_min)) {
    return fail("parameter domain must have positive width");
  }

  auto impl = std::make_unique<detail::SplineImpl>();
  impl->degree = degree;
  impl->knot_min = knot_min;
  impl->knot_max = knot_max;
  impl->knots = std::move(knots);
  impl->control_points = std::move(control_points);

  // 后端节点向量：仿射映射到 [0, kScaledSpan]。不同节点在缩放后
  // 必须仍可区分（TinySpline 将相差小于 1e-4 的值视为相等），
  // 因此要求最小相对间隔 1e-8。
  const double domain_width = impl->knot_max - impl->knot_min;
  impl->knot_scale = detail::kScaledSpan / domain_width;
  impl->knot_scale_inverse = domain_width / detail::kScaledSpan;
  impl->scaled_knots.resize(impl->knots.size());
  for (std::size_t i = 0; i < impl->knots.size(); ++i) {
    impl->scaled_knots[i] = (impl->knots[i] - impl->knot_min) * impl->knot_scale;
  }
  for (std::size_t i = 1; i < impl->knots.size(); ++i) {
    const double separation =
        (impl->knots[i] - impl->knots[i - 1]) * impl->knot_scale;
    if (separation > 0.0 && separation < 1e-2) {
      return fail("distinct knots are too close relative to the parameter "
                  "domain");
    }
  }

  // 后端参数域与后端无关：标准 B-spline 定义域为
  // [knots[degree], knots[n_ctrl]]（TinySpline 与 Eigen 一致）。
  impl->scaled_min = impl->scaled_knots[static_cast<std::size_t>(degree)];
  impl->scaled_max = impl->scaled_knots[impl->control_points.size()];

  std::shared_ptr<const detail::Backend> geometry;
  if (backend == BackendKind::kTinyspline) {
    geometry = detail::make_tinyspline_backend(
        impl->control_points, impl->scaled_knots, degree, error_message);
    if (geometry == nullptr) {
      return std::nullopt;
    }
  } else if (backend == BackendKind::kEigenSplines) {
    geometry = detail::make_eigen_backend(impl->control_points,
                                          impl->scaled_knots, degree);
    if (geometry == nullptr) {
      return fail("Eigen Splines backend could not create the spline");
    }
  } else {
    // 对照后端（GPL-3.0 隔离库）：主库只持有运行时注册的工厂。
    const detail::EgoBackendFactory factory = detail::ego_backend_factory();
    if (factory == nullptr) {
      return fail(
          "ego_uniform backend is not linked in this build; link "
          "planner_bspline_ego and call "
          "planner::bspline::detail::install_ego_backend() first");
    }
    geometry = factory(impl->control_points, impl->scaled_knots, degree,
                       error_message);
    if (geometry == nullptr) {
      return std::nullopt;
    }
  }
  impl->backend = std::move(geometry);

  // 重数为 degree + 1 的内部节点是位置间断点：
  // 曲线在那里只在各分段内连续。
  for (std::size_t i = 0; i < impl->knots.size();) {
    std::size_t j = i;
    while (j < impl->knots.size() && impl->knots[j] == impl->knots[i]) {
      ++j;
    }
    const double value = impl->knots[i];
    if (static_cast<int>(j - i) == degree + 1 && value > impl->knot_min &&
        value < impl->knot_max) {
      impl->gap_knots.push_back(value);
    }
    i = j;
  }

  return Bspline(std::move(impl));
}

std::optional<Bspline> Bspline::create_time_parameterized(
    std::vector<common::Point2d> control_points, std::vector<double> knots,
    double start_time_s, double end_time_s, std::string* error_message) {
  return create_time_parameterized(std::move(control_points), std::move(knots),
                                   start_time_s, end_time_s, kDefaultBackend,
                                   error_message);
}

std::optional<Bspline> Bspline::create_time_parameterized(
    std::vector<common::Point2d> control_points, std::vector<double> knots,
    double start_time_s, double end_time_s, BackendKind backend,
    std::string* error_message) {
  if (!std::isfinite(start_time_s) || !std::isfinite(end_time_s) ||
      !(start_time_s < end_time_s)) {
    if (error_message != nullptr) {
      *error_message =
          "time interval must be finite with start_time_s < end_time_s";
    }
    return std::nullopt;
  }
  std::optional<Bspline> curve = create(std::move(control_points),
                                        std::move(knots), backend,
                                        error_message);
  if (!curve.has_value()) {
    return std::nullopt;
  }
  curve->impl_->parameter_kind = ParameterKind::kTime;
  curve->impl_->time_min = start_time_s;
  curve->impl_->time_max = end_time_s;
  return curve;
}

// ---------------------------------------------------------------------------
// 访问器
// ---------------------------------------------------------------------------

int Bspline::degree() const noexcept { return impl_->degree; }

std::size_t Bspline::control_point_count() const noexcept {
  return impl_->control_points.size();
}

const std::vector<common::Point2d>& Bspline::control_points() const noexcept {
  return impl_->control_points;
}

const std::vector<double>& Bspline::knots() const noexcept {
  return impl_->knots;
}

ParameterDomain Bspline::parameter_domain() const noexcept {
  ParameterDomain domain;
  domain.kind = impl_->parameter_kind;
  if (impl_->parameter_kind == ParameterKind::kTime) {
    domain.min_parameter = impl_->time_min;
    domain.max_parameter = impl_->time_max;
  } else {
    domain.min_parameter = impl_->knot_min;
    domain.max_parameter = impl_->knot_max;
  }
  return domain;
}

bool Bspline::is_geometric_parameter() const noexcept {
  return impl_->parameter_kind == ParameterKind::kGeometric;
}

BackendKind Bspline::backend() const noexcept {
  return impl_->backend->kind();
}

const char* Bspline::backend_name() const noexcept {
  return impl_->backend->name();
}

// ---------------------------------------------------------------------------
// 求值
// ---------------------------------------------------------------------------

EvaluationResult Bspline::evaluate(double parameter, int derivative_order) const {
  EvaluationResult result;
  result.parameter = parameter;
  result.derivative_order = derivative_order;

  if (derivative_order < 0 || derivative_order > kMaxDerivativeOrder) {
    result.status = Status::kDerivativeOrderUnsupported;
    result.message = "first version supports derivative orders 0..3";
    return result;
  }

  double u = 0.0;
  if (!map_to_knot_parameter(*impl_, parameter, &u, &result.message)) {
    result.status = Status::kParameterOutOfDomain;
    return result;
  }

  if (derivative_order > impl_->degree) {
    result.value = common::Point2d{0.0, 0.0};
    return result;
  }

  const double scaled = detail::to_scaled_parameter(*impl_, u);
  common::Point2d value;
  if (!evaluate_scaled_parameter(*impl_, scaled, derivative_order, &value)) {
    result.status = Status::kInternalError;
    result.message = "backend evaluation failed on a valid parameter";
    return result;
  }
  const double scale = public_derivative_scale(*impl_, derivative_order);
  result.value.x_m = value.x_m * scale;
  result.value.y_m = value.y_m * scale;
  return result;
}

Status Bspline::sample(const std::vector<double>& parameters,
                       std::vector<common::Point2d>& output_buffer,
                       int derivative_order) const {
  if (derivative_order < 0 || derivative_order > kMaxDerivativeOrder) {
    return Status::kDerivativeOrderUnsupported;
  }

  // 后端参数：把每个公开参数映射为节点参数并缩放到后端域。
  // 即使对几何曲线也需要缩放后的数组，因为后端域不是 [0, 1]。
  const std::size_t count = parameters.size();
  std::vector<double> scaled_parameters(count);
  for (std::size_t i = 0; i < count; ++i) {
    double u = 0.0;
    std::string message;
    if (!map_to_knot_parameter(*impl_, parameters[i], &u, &message)) {
      return Status::kParameterOutOfDomain;
    }
    scaled_parameters[i] = detail::to_scaled_parameter(*impl_, u);
  }
  const double* knot_parameters = scaled_parameters.data();

  output_buffer.resize(count);
  if (count == 0) {
    return Status::kSuccess;
  }
  if (derivative_order > impl_->degree) {
    for (common::Point2d& point : output_buffer) {
      point = common::Point2d{0.0, 0.0};
    }
    return Status::kSuccess;
  }

  const double scale = public_derivative_scale(*impl_, derivative_order);
  if (derivative_order == 0 || !detail::merge_possible(*impl_, derivative_order)) {
    // 批量快速路径：对（必要时一次求导得到的）句柄批量求值，
    // 再统一换算到公开参数的导数尺度。
    std::unique_ptr<detail::Backend> derived_storage;
    const detail::Backend* source = impl_->backend.get();
    if (derivative_order > 0) {
      derived_storage = impl_->backend->derive(derivative_order);
      if (derived_storage == nullptr) {
        return Status::kInternalError;
      }
      source = derived_storage.get();
    }
    if (!source->eval_batch(knot_parameters, count, output_buffer.data())) {
      return Status::kInternalError;
    }
    for (common::Point2d& point : output_buffer) {
      point.x_m *= scale;
      point.y_m *= scale;
    }

    // 后端的批量求值在间断参数处保留第一个结果；
    // 将这些条目替换为右侧分段的取值。
    for (std::size_t i = 0; i < count; ++i) {
      int multiplicity = 0;
      if (detail::interior_knot_multiplicity(*impl_, knot_parameters[i],
                                             &multiplicity) &&
          multiplicity > impl_->degree - derivative_order) {
        const EvaluationResult fixed =
            evaluate(parameters[i], derivative_order);
        if (fixed.status != Status::kSuccess) {
          return fixed.status;
        }
        output_buffer[i] = fixed.value;
      }
    }
    return Status::kSuccess;
  }

  // 合并间断会破坏求导得到的样条；
  // 改为把每个参数限制到各自所属的右侧分段。
  for (std::size_t i = 0; i < count; ++i) {
    const EvaluationResult single = evaluate(parameters[i], derivative_order);
    if (single.status != Status::kSuccess) {
      return single.status;
    }
    output_buffer[i] = single.value;
  }
  return Status::kSuccess;
}

// ---------------------------------------------------------------------------
// 节点辅助函数
// ---------------------------------------------------------------------------

std::vector<double> clamped_uniform_knots(std::size_t control_point_count,
                                          int degree, double domain_min,
                                          double domain_max) {
  std::vector<double> knots;
  if (degree < 1 ||
      control_point_count < static_cast<std::size_t>(degree) + 1 ||
      !std::isfinite(domain_min) || !std::isfinite(domain_max) ||
      !(domain_max > domain_min)) {
    return knots;
  }
  const std::size_t interior =
      control_point_count - static_cast<std::size_t>(degree) - 1;
  knots.reserve(control_point_count + static_cast<std::size_t>(degree) + 1);
  for (int i = 0; i < degree + 1; ++i) {
    knots.push_back(domain_min);
  }
  const double step =
      (domain_max - domain_min) / static_cast<double>(interior + 1);
  for (std::size_t i = 1; i <= interior; ++i) {
    knots.push_back(domain_min + step * static_cast<double>(i));
  }
  for (int i = 0; i < degree + 1; ++i) {
    knots.push_back(domain_max);
  }
  return knots;
}

}  // namespace planner::bspline
