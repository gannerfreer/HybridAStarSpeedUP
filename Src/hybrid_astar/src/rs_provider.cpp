// RS 提供者实现（见 rs_provider.hpp）。

#include "planner/hybrid_astar/rs_provider.hpp"

#include <string>
#include <utility>

namespace planner::hybrid_astar {

PlannerRsProvider::PlannerRsProvider(const common::VehicleModel& vehicle)
    : vehicle_(vehicle) {}

const common::VehicleModel& PlannerRsProvider::vehicle() const {
  return vehicle_;
}

common::Status PlannerRsProvider::distance(const common::Pose2d& from,
                                           const common::Pose2d& to,
                                           double* distance_m,
                                           std::string* reason) const {
  if (distance_m == nullptr) {
    if (reason != nullptr) {
      *reason = "distance output pointer is null";
    }
    return common::Status::kInvalidInput;
  }
  const rs::RsPlan solved = rs::plan(from, to, vehicle_);
  if (solved.status != common::Status::kSuccess) {
    if (reason != nullptr) {
      *reason = solved.reason;
    }
    return solved.status;
  }
  *distance_m = solved.path.length_m;
  return common::Status::kSuccess;
}

rs::RsPlan PlannerRsProvider::plan(const common::Pose2d& start,
                                   const common::Pose2d& goal) const {
  return rs::plan(start, goal, vehicle_);
}

}  // namespace planner::hybrid_astar
