// GridSearchBackend 默认实现。

#include "planner/grid_search/backend.hpp"

namespace planner::grid_search {

DistanceFieldResult GridSearchBackend::distance_field(
    const DistanceFieldRequest& request) {
  DistanceFieldResult result;
  result.status = common::Status::kInvalidInput;
  result.reason = "backend '" + name() +
                  "' does not provide a distance-field role "
                  "(check supports_distance_field())";
  (void)request;
  return result;
}

}  // namespace planner::grid_search
