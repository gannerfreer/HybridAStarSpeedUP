// 距离场容器与请求校验实现。

#include "planner/grid_search/distance_field.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "planner/grid_search/neighborhood.hpp"

namespace planner::grid_search {

double DistanceField::at(int row, int col) const {
  if (!in_bounds(row, col)) {
    throw std::out_of_range("DistanceField::at: cell outside the field");
  }
  return distances[static_cast<std::size_t>(row) *
                       static_cast<std::size_t>(width) +
                   static_cast<std::size_t>(col)];
}

bool DistanceField::reachable(int row, int col) const {
  return std::isfinite(at(row, col));
}

std::int64_t DistanceField::unreachable_count() const noexcept {
  std::int64_t count = 0;
  for (double distance : distances) {
    if (!std::isfinite(distance)) {
      ++count;
    }
  }
  return count;
}

double DistanceField::max_reachable_m() const noexcept {
  double maximum = 0.0;
  for (double distance : distances) {
    if (std::isfinite(distance) && distance > maximum) {
      maximum = distance;
    }
  }
  return maximum;
}

common::Status validate_distance_field_request(
    const DistanceFieldRequest& request, std::string* reason) {
  if (request.grid == nullptr) {
    if (reason != nullptr) {
      *reason = "grid must not be null";
    }
    return common::Status::kInvalidInput;
  }
  GridSearchRequest probe;
  probe.grid = request.grid;
  probe.start = request.source;
  probe.goal = request.source;
  probe.neighborhood = request.neighborhood;
  probe.time_budget_s = request.time_budget_s;
  probe.node_budget = request.node_budget;
  probe.cancelled = request.cancelled;
  return validate_request(probe, reason);
}

}  // namespace planner::grid_search
