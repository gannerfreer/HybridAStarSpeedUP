#include "ompl_reference.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include <ompl/base/spaces/ReedsSheppStateSpace.h>

namespace planner::rs::ompl_reference {
namespace {

using ompl::base::ReedsSheppStateSpace;
using OmplState = ReedsSheppStateSpace::StateType;

// 对无效查询抛出与原生求解器相同的拒绝。
void require_valid(const common::Pose2d& start, const common::Pose2d& goal,
                   const common::VehicleModel& vehicle) {
  if (!is_valid_query(start, goal, vehicle)) {
    throw std::invalid_argument(
        "rs::ompl_reference: invalid input (finite poses, wheelbase_m > 0, "
        "0 < max_steer_rad < pi/2 required)");
  }
}

// 为一次查询管理两个 OMPL 状态。
class ScopedStates {
 public:
  ScopedStates(ReedsSheppStateSpace& space, const common::Pose2d& start,
               const common::Pose2d& goal)
      : space_(space), first_(space.allocState()), second_(space.allocState()) {
    auto* s1 = first_->as<OmplState>();
    s1->setX(start.x_m);
    s1->setY(start.y_m);
    s1->setYaw(start.yaw_rad);
    auto* s2 = second_->as<OmplState>();
    s2->setX(goal.x_m);
    s2->setY(goal.y_m);
    s2->setYaw(goal.yaw_rad);
  }

  ~ScopedStates() {
    space_.freeState(first_);
    space_.freeState(second_);
  }

  ScopedStates(const ScopedStates&) = delete;
  ScopedStates& operator=(const ScopedStates&) = delete;

  ompl::base::State* first() { return first_; }
  ompl::base::State* second() { return second_; }

 private:
  ReedsSheppStateSpace& space_;
  ompl::base::State* first_;
  ompl::base::State* second_;
};

// 沿用与原生转换器相同的遍历规则，将一个 OMPL 词转换为 planner::rs 表示
// （带符号长度 -> 挡位，L/R -> 物理转向曲率，相邻相同原始段合并）。
RsPath convert(const common::Pose2d& start, const common::Pose2d& goal,
               double radius_m, const ReedsSheppStateSpace::ReedsSheppPath& raw) {
  RsPath out;
  out.start = start;
  out.goal = goal;
  common::Pose2d cursor = start;
  double total = 0.0;
  for (int i = 0; i < 5; ++i) {
    const ReedsSheppStateSpace::ReedsSheppPathSegmentType type = raw.type_[i];
    if (type == ReedsSheppStateSpace::RS_NOP) {
      continue;
    }
    const double signed_length = raw.length_[i];
    // 与原生转换器完全一致地丢弃舍入幻影原始段。
    if (!(std::fabs(signed_length) > 1e-14)) {
      continue;
    }
    const double length_m = std::fabs(signed_length) * radius_m;
    RsSegment segment;
    segment.direction = (signed_length >= 0.0) ? common::Direction::kForward
                                               : common::Direction::kReverse;
    segment.length_m = length_m;
    switch (type) {
      case ReedsSheppStateSpace::RS_LEFT:
        segment.curvature_inv_m = 1.0 / radius_m;
        break;
      case ReedsSheppStateSpace::RS_RIGHT:
        segment.curvature_inv_m = -1.0 / radius_m;
        break;
      case ReedsSheppStateSpace::RS_STRAIGHT:
        segment.curvature_inv_m = 0.0;
        break;
      case ReedsSheppStateSpace::RS_NOP:
      default:
        continue;
    }
    total += segment.length_m;
    cursor = integrate_segment(cursor, segment, segment.length_m);
    if (!out.segments.empty()) {
      RsSegment& last = out.segments.back();
      if (last.direction == segment.direction &&
          last.curvature_inv_m == segment.curvature_inv_m) {
        last.length_m += segment.length_m;
        continue;
      }
    }
    out.segments.push_back(segment);
  }
  out.length_m = total;
  return out;
}

}  // namespace

double distance(const common::Pose2d& start, const common::Pose2d& goal,
                const common::VehicleModel& vehicle) {
  require_valid(start, goal, vehicle);
  const double radius = min_turn_radius_m(vehicle);
  ReedsSheppStateSpace space(radius);
  ScopedStates states(space, start, goal);
  return space.distance(states.first(), states.second());
}

RsPath path(const common::Pose2d& start, const common::Pose2d& goal,
            const common::VehicleModel& vehicle) {
  require_valid(start, goal, vehicle);
  const double radius = min_turn_radius_m(vehicle);
  ReedsSheppStateSpace space(radius);
  ScopedStates states(space, start, goal);
  const ReedsSheppStateSpace::ReedsSheppPath raw =
      space.reedsShepp(states.first(), states.second());
  return convert(start, goal, radius, raw);
}

}  // namespace planner::rs::ompl_reference
