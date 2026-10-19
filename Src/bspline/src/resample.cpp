// 弧长积分与近似等弧长重采样。
//
// 方法：每个参数段（按位置间断处切分）在其节点区间上采用自适应 8 点
// Gauss-Legendre 求积，得到单调的累积弧长表。目标弧长先用累积长度的
// 三次 Hermite 插值反求，再用对直接积分的 Newton 修正迭代；每个报告的
// 区间误差都通过直接数值积分测得，绝不使用 |target - target| 这样的差值。

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "backend.hpp"
#include "impl.hpp"
#include "planner/bspline/curve.hpp"

namespace planner::bspline {

namespace {

using detail::Backend;
using detail::SplineImpl;

// [-1, 1] 上的 8 点 Gauss-Legendre 节点/权重。
constexpr int kGl8Count = 8;
constexpr double kGl8Nodes[kGl8Count] = {
    -0.9602898564975363, -0.7966664774136267, -0.5255324099163290,
    -0.1834346424956498, 0.1834346424956498,  0.5255324099163290,
    0.7966664774136267,  0.9602898564975363};
constexpr double kGl8Weights[kGl8Count] = {
    0.1012285362903763, 0.2223810344533745, 0.3137066458778873,
    0.3626837833783620, 0.3626837833783620, 0.3137066458778873,
    0.2223810344533745, 0.1012285362903763};

constexpr double kDegenerateLengthEpsilon = 1e-12;
constexpr double kRemainderRelativeEpsilon = 1e-9;
constexpr int kMaxSubdivisionDoublings = 10;

// 速度 = 一阶导向量的模，通过后端句柄按右侧分段求值。
bool eval_speed(const Backend& speed_backend, double u, double* out) {
  common::Point2d velocity;
  if (!speed_backend.eval(u, true, &velocity)) {
    return false;
  }
  *out = std::hypot(velocity.x_m, velocity.y_m);
  return true;
}

double integrate_gl8(const Backend& speed_backend, double a, double b,
                     bool* ok) {
  *ok = true;
  const double half = 0.5 * (b - a);
  const double middle = 0.5 * (a + b);
  double sum = 0.0;
  for (int i = 0; i < kGl8Count; ++i) {
    double speed = 0.0;
    if (!eval_speed(speed_backend, middle + half * kGl8Nodes[i], &speed)) {
      *ok = false;
      return 0.0;
    }
    sum += kGl8Weights[i] * speed;
  }
  return sum * half;
}

// 把区间 [a, b] 均分为 `subdivisions` 段后的总积分。
bool integrate_span_total(const Backend& speed_backend, double a, double b,
                          int subdivisions, double* total) {
  const double step = (b - a) / static_cast<double>(subdivisions);
  double sum = 0.0;
  for (int i = 0; i < subdivisions; ++i) {
    bool ok = false;
    const double u0 = a + step * static_cast<double>(i);
    const double part = integrate_gl8(speed_backend, u0, u0 + step, &ok);
    if (!ok) {
      return false;
    }
    sum += part;
  }
  *total = sum;
  return true;
}

// 为区间 [a, b] 选择细分次数：不断加倍细化，直到积分值不再变化
//（相对容差 1e-11）。
bool choose_subdivisions(const Backend& speed_backend, double a, double b,
                         int* subdivisions) {
  int count = 1;
  double previous = 0.0;
  if (!integrate_span_total(speed_backend, a, b, count, &previous)) {
    return false;
  }
  for (int doubling = 0; doubling < kMaxSubdivisionDoublings; ++doubling) {
    const int candidate = count * 2;
    double current = 0.0;
    if (!integrate_span_total(speed_backend, a, b, candidate, &current)) {
      return false;
    }
    const double tolerance = 1e-11 * std::max(1.0, std::fabs(current));
    if (std::fabs(current - previous) <= tolerance) {
      *subdivisions = candidate;
      return true;
    }
    count = candidate;
    previous = current;
  }
  *subdivisions = count;
  return true;
}

// 曲线上的一段连续弧长分段：限制到 [a, b] 的求值句柄、其一阶导句柄
//（速度）与累积弧长表。
struct SegmentArcTable {
  std::unique_ptr<Backend> segment;
  std::unique_ptr<Backend> speed;
  std::vector<double> u;
  std::vector<double> s;
  double length{0.0};
  double param_begin{0.0};
  double param_end{1.0};
};

struct ArcModel {
  std::vector<SegmentArcTable> segments;
  std::vector<double> offsets;  // 大小 == segments.size() + 1
  double total_length{0.0};
};

// 原始节点向量在 [a, b] 范围内的正宽度区间。
std::vector<std::pair<double, double>> spans_within(
    const std::vector<double>& knots, double a, double b) {
  std::vector<std::pair<double, double>> spans;
  for (std::size_t i = 0; i + 1 < knots.size();) {
    std::size_t j = i;
    while (j + 1 < knots.size() && knots[j + 1] == knots[i]) {
      ++j;
    }
    const double next = knots[j + 1];
    if (next > knots[i]) {
      if (knots[i] >= a && next <= b) {
        spans.emplace_back(knots[i], next);
      }
    }
    i = j + 1;
  }
  return spans;
}

bool build_segment_table(const SplineImpl& impl, double a, double b,
                         SegmentArcTable* table) {
  table->segment = impl.backend->restrict(a, b);
  if (table->segment == nullptr) {
    return false;
  }
  table->speed = table->segment->derive(1);
  if (table->speed == nullptr) {
    return false;
  }
  table->param_begin = a;
  table->param_end = b;
  table->u.push_back(a);
  table->s.push_back(0.0);
  double cumulative = 0.0;
  for (const std::pair<double, double>& span :
       spans_within(impl.scaled_knots, a, b)) {
    int subdivisions = 2;
    if (!choose_subdivisions(*table->speed, span.first, span.second,
                             &subdivisions)) {
      return false;
    }
    const double step =
        (span.second - span.first) / static_cast<double>(subdivisions);
    for (int i = 0; i < subdivisions; ++i) {
      const double u0 = span.first + step * static_cast<double>(i);
      const double u1 = u0 + step;
      bool ok = false;
      const double part = integrate_gl8(*table->speed, u0, u1, &ok);
      if (!ok) {
        return false;
      }
      cumulative += part;
      table->u.push_back(u1);
      table->s.push_back(cumulative);
    }
  }
  table->length = cumulative;
  return true;
}

bool build_arc_model(const SplineImpl& impl, ArcModel* model) {
  // 后端（缩放）参数下的分段边界。
  std::vector<double> cuts;
  cuts.push_back(impl.scaled_min);
  for (const double gap : impl.gap_knots) {
    const double scaled_gap = detail::to_scaled_parameter(impl, gap);
    if (!cuts.empty() && scaled_gap == cuts.back()) {
      continue;
    }
    cuts.push_back(scaled_gap);
  }
  cuts.push_back(impl.scaled_max);
  if (!(cuts.size() >= 2)) {
    return false;
  }

  model->segments.clear();
  model->offsets.clear();
  model->offsets.push_back(0.0);
  double total = 0.0;
  for (std::size_t i = 0; i + 1 < cuts.size(); ++i) {
    const double a = cuts[i];
    const double b = cuts[i + 1];
    if (!(b > a)) {
      continue;
    }
    SegmentArcTable table;
    if (!build_segment_table(impl, a, b, &table)) {
      return false;
    }
    total += table.length;
    model->segments.push_back(std::move(table));
    model->offsets.push_back(total);
  }
  if (model->segments.empty()) {
    return false;
  }
  model->total_length = total;
  return true;
}

// 后端（缩放）参数 -> 公开参数（节点参数或时间）。
double public_parameter(const SplineImpl& impl, double scaled_parameter) {
  const double u = detail::to_original_parameter(impl, scaled_parameter);
  if (impl.parameter_kind != ParameterKind::kTime) {
    return u;
  }
  return impl.time_min +
         (u - impl.knot_min) * (impl.time_max - impl.time_min) /
             (impl.knot_max - impl.knot_min);
}

// 在单个分段内反求累积弧长 `target` 对应的参数。
bool invert_arc_length(const SegmentArcTable& table, double target, double* u_out,
                       double* arc_out) {
  const std::size_t grid = table.u.size();
  if (grid < 2) {
    return false;
  }
  if (target <= 0.0) {
    *u_out = table.u.front();
    *arc_out = 0.0;
    return true;
  }
  if (target >= table.length) {
    *u_out = table.u.back();
    *arc_out = table.length;
    return true;
  }

  const std::vector<double>::const_iterator upper =
      std::upper_bound(table.s.begin(), table.s.end(), target);
  std::size_t index = static_cast<std::size_t>(upper - table.s.begin());
  if (index == 0) {
    index = 1;
  }
  if (index > grid - 1) {
    index = grid - 1;
  }
  const double u0 = table.u[index - 1];
  const double u1 = table.u[index];
  const double s0 = table.s[index - 1];
  const double s1 = table.s[index];
  if (!(u1 > u0)) {
    *u_out = u0;
    *arc_out = s0;
    return true;
  }

  double v0 = 0.0;
  double v1 = 0.0;
  if (!eval_speed(*table.speed, u0, &v0) ||
      !eval_speed(*table.speed, u1, &v1)) {
    return false;
  }

  const double h = u1 - u0;
  double u = (s1 > s0) ? u0 + (target - s0) / (s1 - s0) * h : 0.5 * (u0 + u1);
  for (int iteration = 0; iteration < 6; ++iteration) {
    double t = (u - u0) / h;
    t = std::min(std::max(t, 0.0), 1.0);
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    const double h10 = t3 - 2.0 * t2 + t;
    const double h01 = -2.0 * t3 + 3.0 * t2;
    const double h11 = t3 - t2;
    const double interpolated =
        h00 * s0 + h10 * (h * v0) + h01 * s1 + h11 * (h * v1);
    const double derivative_t =
        (6.0 * t2 - 6.0 * t) * s0 + (3.0 * t2 - 4.0 * t + 1.0) * (h * v0) +
        (-6.0 * t2 + 6.0 * t) * s1 + (3.0 * t2 - 2.0 * t) * (h * v1);
    const double derivative_u = derivative_t / h;
    if (std::fabs(derivative_u) < 1e-14) {
      break;
    }
    u -= (interpolated - target) / derivative_u;
    u = std::min(std::max(u, u0), u1);
  }

  // 针对直接积分的 Newton 修正。
  bool ok = false;
  const double integral = integrate_gl8(*table.speed, u0, u, &ok);
  if (!ok) {
    return false;
  }
  const double residual = s0 + integral - target;
  double speed_u = 0.0;
  if (!eval_speed(*table.speed, u, &speed_u)) {
    return false;
  }
  if (speed_u > 1e-14) {
    u -= residual / speed_u;
    u = std::min(std::max(u, u0), u1);
  }

  const double final_integral =
      integrate_gl8(*table.speed, u0, u, &ok);
  if (!ok) {
    return false;
  }
  *u_out = u;
  *arc_out = s0 + final_integral;
  return true;
}

bool sample_at_arc_length(const SplineImpl& impl, const ArcModel& model,
                          double target, ArcLengthSample* sample) {
  std::size_t segment = 0;
  while (segment + 2 < model.offsets.size() &&
         target > model.offsets[segment + 1]) {
    ++segment;
  }
  const SegmentArcTable& table = model.segments[segment];
  const double local = std::min(std::max(target - model.offsets[segment], 0.0),
                                table.length);
  double u = 0.0;
  double arc = 0.0;
  if (!invert_arc_length(table, local, &u, &arc)) {
    return false;
  }
  common::Point2d position;
  if (!table.segment->eval(u, true, &position)) {
    return false;
  }
  sample->parameter = public_parameter(impl, u);
  sample->position = position;
  sample->arc_length_m = model.offsets[segment] + arc;
  return true;
}

bool valid_tolerance(double tolerance) {
  return std::isfinite(tolerance) && tolerance > 0.0;
}

}  // namespace

// ---------------------------------------------------------------------------
// 公开 API
// ---------------------------------------------------------------------------

double Bspline::total_length_m() const {
  ArcModel model;
  if (!build_arc_model(*impl_, &model)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return model.total_length;
}

ArcLengthResampleResult Bspline::sample_equal_arc_length(
    std::size_t point_count, std::vector<ArcLengthSample>& output_buffer,
    double interval_tolerance_m) const {
  ArcLengthResampleResult result;
  if (point_count < 1) {
    result.status = Status::kInvalidInput;
    result.message = "point_count must be at least 1";
    return result;
  }
  if (!valid_tolerance(interval_tolerance_m)) {
    result.status = Status::kInvalidInput;
    result.message = "interval_tolerance_m must be finite and positive";
    return result;
  }

  ArcModel model;
  if (!build_arc_model(*impl_, &model)) {
    result.status = Status::kInternalError;
    result.message = "failed to build the arc-length model";
    return result;
  }
  result.total_length_m = model.total_length;
  if (model.total_length <= kDegenerateLengthEpsilon) {
    result.status = Status::kDegenerateCurve;
    result.message = "arc-length resampling requires a curve with positive length";
    output_buffer.clear();
    return result;
  }

  const double nominal =
      point_count >= 2 ? model.total_length / static_cast<double>(point_count - 1)
                       : 0.0;
  result.nominal_interval_m = nominal;

  output_buffer.clear();
  output_buffer.resize(point_count);
  for (std::size_t j = 0; j < point_count; ++j) {
    double target = nominal * static_cast<double>(j);
    if (point_count >= 2 && j + 1 == point_count) {
      target = model.total_length;
    } else if (point_count == 1) {
      target = 0.0;
    }
    if (!sample_at_arc_length(*impl_, model, target, &output_buffer[j])) {
      result.status = Status::kInternalError;
      result.message = "arc-length inversion failed";
      output_buffer.clear();
      return result;
    }
  }

  double max_error = 0.0;
  double last_error = 0.0;
  for (std::size_t j = 0; j + 1 < point_count; ++j) {
    const double interval =
        output_buffer[j + 1].arc_length_m - output_buffer[j].arc_length_m;
    last_error = std::fabs(interval - nominal);
    max_error = std::max(max_error, last_error);
  }
  result.max_interval_error_m = max_error;
  result.last_interval_error_m = last_error;
  result.has_remainder = false;
  if (max_error > interval_tolerance_m) {
    result.status = Status::kArcLengthToleranceExceeded;
    result.message = "maximum arc-length interval error exceeds the tolerance";
  }
  return result;
}

ArcLengthResampleResult Bspline::sample_equal_arc_length(
    std::size_t point_count, double interval_tolerance_m) const {
  ArcLengthResampleResult result;
  std::vector<ArcLengthSample> buffer;
  result = sample_equal_arc_length(point_count, buffer, interval_tolerance_m);
  result.samples = std::move(buffer);
  return result;
}

ArcLengthResampleResult Bspline::resample_uniform_interval(
    double interval_m, std::vector<ArcLengthSample>& output_buffer,
    double interval_tolerance_m) const {
  ArcLengthResampleResult result;
  if (!std::isfinite(interval_m) || !(interval_m > 0.0)) {
    result.status = Status::kInvalidInput;
    result.message = "interval_m must be finite and positive";
    return result;
  }
  if (!valid_tolerance(interval_tolerance_m)) {
    result.status = Status::kInvalidInput;
    result.message = "interval_tolerance_m must be finite and positive";
    return result;
  }

  ArcModel model;
  if (!build_arc_model(*impl_, &model)) {
    result.status = Status::kInternalError;
    result.message = "failed to build the arc-length model";
    return result;
  }
  const double length = model.total_length;
  result.total_length_m = length;
  result.nominal_interval_m = interval_m;
  if (length <= kDegenerateLengthEpsilon) {
    result.status = Status::kDegenerateCurve;
    result.message = "arc-length resampling requires a curve with positive length";
    output_buffer.clear();
    return result;
  }

  const double remainder_epsilon =
      kRemainderRelativeEpsilon * std::max(1.0, length);
  std::size_t full_intervals =
      static_cast<std::size_t>(std::floor(length / interval_m));
  double remainder = length - static_cast<double>(full_intervals) * interval_m;
  bool has_remainder = false;
  if (remainder <= remainder_epsilon) {
    remainder = 0.0;
  } else if (remainder >= interval_m - remainder_epsilon) {
    full_intervals += 1;
    remainder = 0.0;
  } else {
    has_remainder = true;
  }
  result.has_remainder = has_remainder;

  const std::size_t point_count = full_intervals + 1 + (has_remainder ? 1 : 0);
  output_buffer.clear();
  output_buffer.resize(point_count);
  for (std::size_t j = 0; j <= full_intervals; ++j) {
    const double target = interval_m * static_cast<double>(j);
    if (!sample_at_arc_length(*impl_, model, target, &output_buffer[j])) {
      result.status = Status::kInternalError;
      result.message = "arc-length inversion failed";
      output_buffer.clear();
      return result;
    }
  }
  if (has_remainder) {
    if (!sample_at_arc_length(*impl_, model, length, &output_buffer.back())) {
      result.status = Status::kInternalError;
      result.message = "arc-length inversion failed";
      output_buffer.clear();
      return result;
    }
  }

  double max_error = 0.0;
  double last_error = 0.0;
  for (std::size_t j = 0; j < full_intervals; ++j) {
    const double interval =
        output_buffer[j + 1].arc_length_m - output_buffer[j].arc_length_m;
    max_error = std::max(max_error, std::fabs(interval - interval_m));
  }
  if (has_remainder) {
    last_error = std::fabs(
        (output_buffer[full_intervals + 1].arc_length_m -
         output_buffer[full_intervals].arc_length_m) -
        interval_m);
  } else if (full_intervals > 0) {
    last_error = std::fabs(
        (output_buffer[full_intervals].arc_length_m -
         output_buffer[full_intervals - 1].arc_length_m) -
        interval_m);
  } else {
    last_error = std::fabs(length - interval_m);
  }
  result.max_interval_error_m = max_error;
  result.last_interval_error_m = last_error;
  if (max_error > interval_tolerance_m) {
    result.status = Status::kArcLengthToleranceExceeded;
    result.message = "maximum arc-length interval error exceeds the tolerance";
  }
  return result;
}

ArcLengthResampleResult Bspline::resample_uniform_interval(
    double interval_m, double interval_tolerance_m) const {
  ArcLengthResampleResult result;
  std::vector<ArcLengthSample> buffer;
  result = resample_uniform_interval(interval_m, buffer, interval_tolerance_m);
  result.samples = std::move(buffer);
  return result;
}

}  // namespace planner::bspline
