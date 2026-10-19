#pragma once

// 分段二维多项式轨迹：各分段在单一全局时间轴上首尾相接，
// 每个分段在自己的局部时间上求值
// （Docs/component_development_plan.md 第 6.1、6.2 节）。
//
// 边界规则（第 6.1 节）：内部分段边界选择右侧（较晚）的分段；
// 最终端点选择最后一个分段。求值超出 [0, duration_s] 时
// 返回 kOutOfRange，不做外推。

#include <cstddef>
#include <vector>

#include "planner/common/types.hpp"
#include "planner/polynomial/trajectory_segment.hpp"

namespace planner::polynomial {

// 构建分段轨迹时每个内部衔接点要求的连续性。
// kAutomatic 在两个三次分段之间要求 C1，任一侧为五次时要求 C2。
enum class ContinuityPolicy {
  kAutomatic,
  kPositionOnly,
  kVelocity,
  kAcceleration,
};

class PiecewiseTrajectory2d {
 public:
  // 空/无效轨迹（无分段）；只有 create() 产生有效轨迹。
  PiecewiseTrajectory2d() = default;

  // 从 t = 0 开始、由首尾相接的分段构建轨迹。
  // 拒绝空分段列表、无效分段，以及违反 `continuity_tolerance` 内
  // 所要求连续性的衔接点（绝对加相对：|right - left| <= tol * (1 + max(|left|,
  // |right|))），并返回 kInvalidInput（"boundary mismatch"）。
  static Result<PiecewiseTrajectory2d> create(
      std::vector<TrajectorySegment> segments,
      ContinuityPolicy policy = ContinuityPolicy::kAutomatic,
      double continuity_tolerance = 1e-9);

  std::size_t segment_count() const noexcept;
  bool empty() const noexcept;
  double duration_s() const noexcept;

  // 分段的累积起始时间。索引 >= segment_count() 时
  // 抛出 std::out_of_range。
  double segment_start_time_s(std::size_t index) const;
  const TrajectorySegment& segment(std::size_t index) const;

  // 全局时间求值。在内部衔接点选择右侧分段；
  // 在最终端点选择最后一个分段。时间超出 [0, duration_s] 时
  // 返回 kOutOfRange，不做外推。
  Result<planner::common::Point2d> evaluate(double t_s,
                                            int derivative_order = 0) const noexcept;

  // 批处理求值，使用调用方预分配的 `count` 个点的输出缓冲区。
  // 在第一个被拒绝的全局时间处停止并报告其索引。
  BatchResult evaluate_batch(const double* times_s, std::size_t count,
                             int derivative_order,
                             planner::common::Point2d* output) const noexcept;

  // 时间缩放：q(t) = p(t / time_scale)；每个分段时长和总时长
  // 都乘以 time_scale。要求 time_scale 有限且 > 0。
  Result<PiecewiseTrajectory2d> scaled_time(double time_scale) const;

 private:
  PiecewiseTrajectory2d(std::vector<TrajectorySegment> segments,
                        std::vector<double> start_times_s, double duration_s);

  // 拥有 t_s 的分段索引（t_s 必须位于 [0, duration_s] 内）。
  std::size_t segment_index_for_time(double t_s) const noexcept;

  std::vector<TrajectorySegment> segments_;
  std::vector<double> start_times_s_;
  double duration_s_{0.0};
};

}  // namespace planner::polynomial
