// 服务值类型和小型辅助函数。

#include "planner/service/types.hpp"

namespace planner::service {

const char* to_string(JobState state) noexcept {
  switch (state) {
    case JobState::kQueued:
      return "queued";
    case JobState::kRunning:
      return "running";
    case JobState::kSucceeded:
      return "succeeded";
    case JobState::kFailed:
      return "failed";
    case JobState::kCancelled:
      return "cancelled";
  }
  return "failed";
}

bool is_terminal(JobState state) noexcept {
  return state == JobState::kSucceeded || state == JobState::kFailed ||
         state == JobState::kCancelled;
}

void ServiceObstacle::bounds(double* min_x, double* min_y, double* max_x,
                             double* max_y) const {
  if (kind == Kind::kRectangle) {
    *min_x = x_min_m;
    *min_y = y_min_m;
    *max_x = x_max_m;
    *max_y = y_max_m;
    return;
  }
  double lo_x = vertices.empty() ? 0.0 : vertices.front().x_m;
  double lo_y = vertices.empty() ? 0.0 : vertices.front().y_m;
  double hi_x = lo_x;
  double hi_y = lo_y;
  for (const common::Point2d& vertex : vertices) {
    lo_x = std::min(lo_x, vertex.x_m);
    lo_y = std::min(lo_y, vertex.y_m);
    hi_x = std::max(hi_x, vertex.x_m);
    hi_y = std::max(hi_y, vertex.y_m);
  }
  *min_x = lo_x;
  *min_y = lo_y;
  *max_x = hi_x;
  *max_y = hi_y;
}

collision::Obstacle ServiceObstacle::to_collision() const {
  if (kind == Kind::kRectangle) {
    return collision::Obstacle::rectangle(id, x_min_m, y_min_m, x_max_m, y_max_m);
  }
  return collision::Obstacle::convex_polygon(id, vertices);
}

corridor::Obstacle ServiceObstacle::to_corridor() const {
  if (kind == Kind::kRectangle) {
    return corridor::make_rectangle_obstacle(id, x_min_m, y_min_m, x_max_m, y_max_m);
  }
  return corridor::make_convex_obstacle(id, vertices);
}

common::VehicleModel default_vehicle() {
  common::VehicleModel vehicle;
  vehicle.id = "default_test_vehicle";
  vehicle.wheelbase_m = 2.8;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.9;
  vehicle.rear_overhang_m = 1.0;
  vehicle.max_steer_rad = 0.5;
  vehicle.safety_margin_m = 0.1;
  return vehicle;
}

}  // namespace planner::service
