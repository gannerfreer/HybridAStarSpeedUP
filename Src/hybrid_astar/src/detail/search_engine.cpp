/**
 * @file search_engine.cpp
 * @brief Hybrid A* 路径搜索算法及动力学状态外扩的具体实现
 */

#include "planner/hybrid_astar/detail/search_engine.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "planner/math/kinematics/arc.hpp"
#include "planner/math/kinematics/bicycle_model.hpp"

namespace planner::hybrid_astar::detail {

using math::kPi;

SearchEngine::SearchEngine(const common::VehicleModel& vehicle,
                           const common::UserAstarConfig& config) {
  configure(vehicle, config);
}

void SearchEngine::configure(const common::VehicleModel& vehicle,
                             const common::UserAstarConfig& config) {
  vehicle_ = vehicle;
  config_ = config;
  heading_bin_rad_ = 2.0 * kPi / static_cast<double>(config_.heading_bins);

  const int steering_num = std::max(2, config_.steering_samples);
  const double steer_min_rad = -vehicle_.max_steer_rad;
  const double delta_steer_rad =
      (vehicle_.max_steer_rad - steer_min_rad) / static_cast<double>(steering_num - 1);
  curvature_samples_.resize(steering_num);
  for (int i = 0; i < steering_num; ++i) {
    const double steering = steer_min_rad + static_cast<double>(i) * delta_steer_rad;
    curvature_samples_[i] = math::kinematics::curvature_from_steer(steering, vehicle_.wheelbase_m);
  }
}

// ---------------------------------------------------------------------------
// 搜索钩子
// ---------------------------------------------------------------------------
void SearchEngine::set_abort_callback(std::function<bool()> callback) {
  abort_callback_ = std::move(callback);
}

void SearchEngine::set_pose_collision_hook(std::function<bool(const math::Pose2d&)> hook) {
  pose_collision_hook_ = std::move(hook);
}

void SearchEngine::set_motion_collision_hook(
    std::function<bool(const SearchState&, const SearchState&)> hook) {
  motion_collision_hook_ = std::move(hook);
}

void SearchEngine::set_goal_hook(std::function<bool(const SearchState&)> hook) {
  goal_hook_ = std::move(hook);
}

void SearchEngine::clear_hooks() {
  abort_callback_ = nullptr;
  pose_collision_hook_ = nullptr;
  motion_collision_hook_ = nullptr;
  goal_hook_ = nullptr;
}

void SearchEngine::reset_state() {
  open_set_.clear();
  closed_set_.clear();
  best_nodes_.clear();
  open_queue_.clear();
}

SearchResult SearchEngine::search(const math::Pose2d& start, const math::Pose2d& goal,
                                  std::int64_t node_budget) {
  SearchResult result;
  result.status = common::Status::kInvalidInput;

  if (!pose_collision_hook_ || !motion_collision_hook_ || !goal_hook_) {
    result.reason = "required collision or goal hook is missing";
    return result;
  }
  if (!std::isfinite(start.x_m) || !std::isfinite(start.y_m) || !std::isfinite(start.yaw_rad) ||
      !std::isfinite(goal.x_m) || !std::isfinite(goal.y_m) || !std::isfinite(goal.yaw_rad)) {
    result.reason = "start or goal is not finite";
    return result;
  }

  reset_state();

  Vertex3D start_forward;
  start_forward.state.pose.x_m = start.x_m;
  start_forward.state.pose.y_m = start.y_m;
  start_forward.state.pose.yaw_rad = math::normalize_angle_rad(start.yaw_rad);
  start_forward.state.direction = common::Direction::kForward;
  start_forward.parent_id = kInvalidNodeId;
  start_forward.id = vertex_hash(start_forward);
  start_forward.g = 0.0;
  start_forward.h = heuristic(start_forward, goal);
  start_forward.f = start_forward.g + start_forward.h;

  Vertex3D start_backward = start_forward;
  start_backward.state.direction = common::Direction::kReverse;
  start_backward.id = vertex_hash(start_backward);
  start_backward.f = start_backward.h;

  Vertex3D goal_check;
  goal_check.state.pose.x_m = goal.x_m;
  goal_check.state.pose.y_m = goal.y_m;
  goal_check.state.pose.yaw_rad = math::normalize_angle_rad(goal.yaw_rad);
  if (is_collision(start_forward) || is_collision(goal_check)) {
    result.reason = "start or goal in collision";
    return result;
  }

  open_set_[start_forward.id] = start_forward;
  best_nodes_[start_forward.id] = start_forward;
  open_queue_.push(OpenEntry{start_forward.f, start_forward.id});

  open_set_[start_backward.id] = start_backward;
  best_nodes_[start_backward.id] = start_backward;
  open_queue_.push(OpenEntry{start_backward.f, start_backward.id});

  const std::int64_t max_iterations = std::max<std::int64_t>(1, node_budget);
  std::uint64_t final_id = kInvalidNodeId;
  bool found = false;

  for (std::int64_t iter = 0; iter < max_iterations; ++iter) {
    if (abort_callback_ && abort_callback_()) {
      result.status = common::Status::kCancelled;
      result.reason = "aborted";
      return result;
    }
    result.iterations = iter + 1;
    if (open_queue_.empty()) {
      result.status = common::Status::kNoPath;
      result.reason = "no feasible path";
      return result;
    }

    const OpenEntry top = open_queue_.top();
    open_queue_.pop();

    auto open_it = open_set_.find(top.id);
    if (open_it == open_set_.end()) {
      continue;
    }
    if (std::fabs(open_it->second.f - top.f) > 1e-9) {
      continue;
    }

    const Vertex3D current = open_it->second;
    open_set_.erase(open_it);
    closed_set_.insert(current.id);

    if (is_goal(current)) {
      found = true;
      final_id = current.id;
      result.cost = current.g;
      break;
    }

    expand_node(current, start, goal);
  }

  if (!found) {
    if (open_queue_.empty()) {
      result.status = common::Status::kNoPath;
      result.reason = "no feasible path";
    } else {
      result.status = common::Status::kTimeout;
      result.reason = "node budget of " + std::to_string(max_iterations) + " exhausted";
    }
    return result;
  }

  if (!reconstruct_path(final_id, &result.path)) {
    result.status = common::Status::kValidationFailed;
    result.reason = "failed to reconstruct path";
    return result;
  }

  result.status = common::Status::kSuccess;
  result.reason.clear();
  return result;
}

bool SearchEngine::is_collision(const Vertex3D& node) const {
  return pose_collision_hook_(node.state.pose);
}

bool SearchEngine::is_goal(const Vertex3D& node) const { return goal_hook_(node.state); }

std::uint64_t SearchEngine::vertex_hash(const Vertex3D& node) const {
  auto x_bin = static_cast<int32_t>(std::floor(node.state.pose.x_m / config_.xy_resolution_m));
  auto y_bin = static_cast<int32_t>(std::floor(node.state.pose.y_m / config_.xy_resolution_m));
  auto angle_bin = static_cast<int32_t>(
      std::floor((math::normalize_angle_rad(node.state.pose.yaw_rad) + kPi) / heading_bin_rad_));
  if (angle_bin >= config_.heading_bins) {
    angle_bin %= config_.heading_bins;
  }
  if (angle_bin < 0) {
    angle_bin = (angle_bin % config_.heading_bins + config_.heading_bins) % config_.heading_bins;
  }

  std::uint64_t hash = 0U;
  if (node.state.direction == common::Direction::kReverse) {
    hash |= 0x8000000000000000ULL;
  }
  if (x_bin < 0) {
    hash |= 0x2000000000000000ULL;
    x_bin = -x_bin;
  }
  if (y_bin < 0) {
    hash |= 0x1000000000000000ULL;
    y_bin = -y_bin;
  }
  hash |= (static_cast<std::uint64_t>(angle_bin) & 0xFFFULL) << 48U;
  hash |= (static_cast<std::uint64_t>(x_bin) & 0xFFFFFFULL) << 24U;
  hash |= static_cast<std::uint64_t>(y_bin) & 0xFFFFFFULL;
  // kInvalidNodeId 保留为根节点父驱/重建终止哨兵，哈希不得占用该值
  if (hash == kInvalidNodeId) {
    hash &= ~0x1ULL;
  }
  return hash;
}

void SearchEngine::expand_node(const Vertex3D& current, const math::Pose2d& start,
                               const math::Pose2d& goal) {
  bool allow_backward = config_.allow_reverse;
  if (allow_backward && config_.backward_search_range_m > 0.0) {
    const double dist2_start_sq =
        (current.state.pose.x_m - start.x_m) * (current.state.pose.x_m - start.x_m) +
        (current.state.pose.y_m - start.y_m) * (current.state.pose.y_m - start.y_m);
    const double range_sq = config_.backward_search_range_m * config_.backward_search_range_m;
    if (dist2_start_sq > range_sq) {
      allow_backward = false;
    }
  }

  const double dx_start = current.state.pose.x_m - start.x_m;
  const double dy_start = current.state.pose.y_m - start.y_m;
  const double dist_to_start_sq = dx_start * dx_start + dy_start * dy_start;
  const double coeff = (dist_to_start_sq > config_.linear_preferred_distance_m2) ? 1.0 : 5.0;

  const double cos_yaw = std::cos(current.state.pose.yaw_rad);
  const double sin_yaw = std::sin(current.state.pose.yaw_rad);

  const common::Direction directions[2] = {common::Direction::kForward,
                                           common::Direction::kReverse};
  const int direction_count = allow_backward ? 2 : 1;

  for (const double curvature : curvature_samples_) {
    for (int d = 0; d < direction_count; ++d) {
      Vertex3D succ = propagate(current, directions[d], curvature, cos_yaw, sin_yaw);
      succ.id = vertex_hash(succ);
      if (closed_set_.find(succ.id) != closed_set_.end()) {
        continue;
      }
      succ.g = transition_cost(current, succ, coeff);

      auto open_it = open_set_.find(succ.id);
      if (open_it != open_set_.end() && succ.g >= open_it->second.g) {
        continue;
      }
      if (motion_collision_hook_(current.state, succ.state)) {
        continue;
      }

      succ.h = heuristic(succ, goal);
      succ.f = succ.g + succ.h;

      open_set_[succ.id] = succ;
      best_nodes_[succ.id] = succ;
      open_queue_.push(OpenEntry{succ.f, succ.id});
    }
  }
}

SearchEngine::Vertex3D SearchEngine::propagate(const Vertex3D& start, common::Direction direction,
                                               double curvature_per_m, double cos_yaw,
                                               double sin_yaw) const {
  Vertex3D end;
  end.parent_id = start.id;
  end.state.direction = direction;
  end.state.curvature_per_m = curvature_per_m;
  end.state.arc_length_m = config_.motion_step_m;
  const double sign = direction == common::Direction::kForward ? 1.0 : -1.0;
  end.state.pose = math::integrate_arc_trig(start.state.pose, sign * end.state.arc_length_m,
                                            curvature_per_m, cos_yaw, sin_yaw);
  return end;
}

double SearchEngine::transition_cost(const Vertex3D& from, const Vertex3D& to, double coeff) const {
  const double heading_change =
      std::fabs(math::normalize_angle_rad(to.state.pose.yaw_rad - from.state.pose.yaw_rad));
  const double direction_cost = (to.state.direction == common::Direction::kReverse)
                                    ? config_.reverse_cost
                                    : config_.forward_cost;
  const double switch_cost =
      (to.state.direction == from.state.direction) ? 0.0 : config_.switch_cost;

  return from.g + config_.motion_step_m * direction_cost +
         coeff * config_.turn_penalty * heading_change + switch_cost;
}

double SearchEngine::heuristic(const Vertex3D& node, const math::Pose2d& goal) const {
  const double euclid = std::hypot(node.state.pose.x_m - goal.x_m, node.state.pose.y_m - goal.y_m);
  const double yaw_diff =
      std::fabs(math::normalize_angle_rad(node.state.pose.yaw_rad - goal.yaw_rad));
  return euclid * std::max(1.0, config_.forward_cost) + yaw_diff * config_.turn_penalty;
}

bool SearchEngine::reconstruct_path(std::uint64_t final_id, std::vector<SearchState>* path) const {
  if (path == nullptr || final_id == kInvalidNodeId) {
    return false;
  }
  std::vector<SearchState> reversed;
  std::uint64_t current_id = final_id;
  while (current_id != kInvalidNodeId) {
    const auto it = best_nodes_.find(current_id);
    if (it == best_nodes_.end()) {
      return false;
    }
    reversed.push_back(it->second.state);
    current_id = it->second.parent_id;
  }
  std::reverse(reversed.begin(), reversed.end());
  *path = std::move(reversed);
  return !path->empty();
}

}  // namespace planner::hybrid_astar::detail
