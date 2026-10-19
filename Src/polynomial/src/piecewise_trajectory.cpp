// 分段二维多项式轨迹：首尾相接的分段、衔接点连续性验证、
// 右侧分段边界规则和时间缩放。

#include "planner/polynomial/piecewise_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <utility>

namespace planner::polynomial {
namespace {

using planner::common::Point2d;

// 衔接点的绝对加相对容差：
// |right - left| <= tolerance * (1 + max(|left|, |right|))。
bool value_continuous(double left, double right, double tolerance) noexcept {
  const double scale = 1.0 + std::max(std::fabs(left), std::fabs(right));
  return std::fabs(right - left) <= tolerance * scale;
}

// 返回给定策略和相邻分段阶数下，衔接点处必须连续的最高求导阶数。
int required_continuity_order(ContinuityPolicy policy, int left_degree,
                              int right_degree) noexcept {
  switch (policy) {
    case ContinuityPolicy::kPositionOnly:
      return 0;
    case ContinuityPolicy::kVelocity:
      return 1;
    case ContinuityPolicy::kAcceleration:
      return 2;
    case ContinuityPolicy::kAutomatic:
      return (left_degree == 5 || right_degree == 5) ? 2 : 1;
  }
  return 0;
}

}  // namespace

PiecewiseTrajectory2d::PiecewiseTrajectory2d(std::vector<TrajectorySegment> segments,
                                             std::vector<double> start_times_s,
                                             double duration_s)
    : segments_(std::move(segments)),
      start_times_s_(std::move(start_times_s)),
      duration_s_(duration_s) {}

Result<PiecewiseTrajectory2d> PiecewiseTrajectory2d::create(
    std::vector<TrajectorySegment> segments, ContinuityPolicy policy,
    double continuity_tolerance) {
  if (segments.empty()) {
    return Result<PiecewiseTrajectory2d>::failure(Code::kInvalidInput);
  }
  if (!std::isfinite(continuity_tolerance) || continuity_tolerance < 0.0) {
    return Result<PiecewiseTrajectory2d>::failure(Code::kInvalidInput);
  }
  for (const TrajectorySegment& segment : segments) {
    if (!segment.valid()) {
      return Result<PiecewiseTrajectory2d>::failure(Code::kInvalidInput);
    }
  }

  // 衔接点验证（"boundary mismatch" -> kInvalidInput）。
  for (std::size_t i = 0; i + 1 < segments.size(); ++i) {
    const TrajectorySegment& left = segments[i];
    const TrajectorySegment& right = segments[i + 1];
    const int required = required_continuity_order(policy, left.degree(), right.degree());
    for (int order = 0; order <= required; ++order) {
      const Result<Point2d> left_value = left.evaluate(left.duration_s(), order);
      const Result<Point2d> right_value = right.evaluate(0.0, order);
      if (!left_value.ok() || !right_value.ok()) {
        return Result<PiecewiseTrajectory2d>::failure(Code::kInvalidInput);
      }
      if (!value_continuous(left_value.value.x_m, right_value.value.x_m,
                            continuity_tolerance) ||
          !value_continuous(left_value.value.y_m, right_value.value.y_m,
                            continuity_tolerance)) {
        return Result<PiecewiseTrajectory2d>::failure(Code::kInvalidInput);
      }
    }
  }

  std::vector<double> start_times_s(segments.size(), 0.0);
  double duration_s = 0.0;
  for (std::size_t i = 0; i < segments.size(); ++i) {
    start_times_s[i] = duration_s;
    duration_s += segments[i].duration_s();
    if (!std::isfinite(duration_s)) {
      return Result<PiecewiseTrajectory2d>::failure(Code::kInvalidInput);
    }
  }
  return Result<PiecewiseTrajectory2d>::success(
      PiecewiseTrajectory2d(std::move(segments), std::move(start_times_s), duration_s));
}

std::size_t PiecewiseTrajectory2d::segment_count() const noexcept {
  return segments_.size();
}

bool PiecewiseTrajectory2d::empty() const noexcept {
  return segments_.empty();
}

double PiecewiseTrajectory2d::duration_s() const noexcept {
  return duration_s_;
}

double PiecewiseTrajectory2d::segment_start_time_s(std::size_t index) const {
  if (index >= start_times_s_.size()) {
    throw std::out_of_range("PiecewiseTrajectory2d::segment_start_time_s: index out of range");
  }
  return start_times_s_[index];
}

const TrajectorySegment& PiecewiseTrajectory2d::segment(std::size_t index) const {
  if (index >= segments_.size()) {
    throw std::out_of_range("PiecewiseTrajectory2d::segment: index out of range");
  }
  return segments_[index];
}

std::size_t PiecewiseTrajectory2d::segment_index_for_time(double t_s) const noexcept {
  // 第一个严格大于 t_s 的起始时间减一。因此内部边界选择
  // 右侧分段，最终端点选择最后一个分段。
  const auto it = std::upper_bound(start_times_s_.begin(), start_times_s_.end(), t_s);
  if (it == start_times_s_.begin()) {
    return 0;
  }
  return static_cast<std::size_t>(it - start_times_s_.begin() - 1);
}

Result<Point2d> PiecewiseTrajectory2d::evaluate(double t_s,
                                                int derivative_order) const noexcept {
  if (segments_.empty() || !std::isfinite(t_s)) {
    return Result<Point2d>::failure(Code::kInvalidInput);
  }
  if (derivative_order < 0) {
    return Result<Point2d>::failure(Code::kInvalidInput);
  }
  if (derivative_order > kMaxDerivativeOrder) {
    return Result<Point2d>::failure(Code::kUnsupportedDerivative);
  }
  if (t_s < 0.0 || t_s > duration_s_) {
    return Result<Point2d>::failure(Code::kOutOfRange);
  }
  const std::size_t index = segment_index_for_time(t_s);
  return segments_[index].evaluate(t_s - start_times_s_[index], derivative_order);
}

BatchResult PiecewiseTrajectory2d::evaluate_batch(const double* times_s, std::size_t count,
                                                  int derivative_order,
                                                  Point2d* output) const noexcept {
  BatchResult result;
  if (count == 0) {
    return result;
  }
  if (times_s == nullptr || output == nullptr) {
    result.code = Code::kInvalidInput;
    return result;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const Result<Point2d> value = evaluate(times_s[i], derivative_order);
    if (!value.ok()) {
      result.code = value.code;
      result.first_bad_index = i;
      return result;
    }
    output[i] = value.value;
    result.processed = i + 1;
  }
  return result;
}

Result<PiecewiseTrajectory2d> PiecewiseTrajectory2d::scaled_time(double time_scale) const {
  if (segments_.empty()) {
    return Result<PiecewiseTrajectory2d>::failure(Code::kInvalidInput);
  }
  if (!std::isfinite(time_scale) || time_scale <= 0.0) {
    return Result<PiecewiseTrajectory2d>::failure(Code::kInvalidInput);
  }
  std::vector<TrajectorySegment> scaled_segments;
  scaled_segments.reserve(segments_.size());
  for (const TrajectorySegment& segment : segments_) {
    const Result<TrajectorySegment> scaled = segment.scaled_time(time_scale);
    if (!scaled.ok()) {
      return Result<PiecewiseTrajectory2d>::failure(scaled.code);
    }
    scaled_segments.push_back(scaled.value);
  }
  // 所有时长均为正且有限；累积和保持有限，
  // 因为缩放后的总时长以 time_scale * duration_s() 为界。
  const double scaled_duration = duration_s_ * time_scale;
  if (!std::isfinite(scaled_duration) || scaled_duration <= 0.0) {
    return Result<PiecewiseTrajectory2d>::failure(Code::kInvalidInput);
  }
  std::vector<double> start_times_s(scaled_segments.size(), 0.0);
  double cursor = 0.0;
  for (std::size_t i = 0; i < scaled_segments.size(); ++i) {
    start_times_s[i] = cursor;
    cursor += scaled_segments[i].duration_s();
  }
  return Result<PiecewiseTrajectory2d>::success(
      PiecewiseTrajectory2d(std::move(scaled_segments), std::move(start_times_s), cursor));
}

}  // namespace planner::polynomial
