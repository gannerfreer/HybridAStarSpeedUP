/**
 * @file hybrid_astar.cpp
 * @brief Hybrid A* 路径搜索算法及动力学状态外扩的具体实现。
 */

#include "motion.hpp"
#include "hybrid_astar.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace astar {

using planner::common::kPi;

HybridAStar::HybridAStar(const PlannerConfig& config) : config_(config) {
    SetConfig(config);
}

void HybridAStar::SetConfig(const PlannerConfig& config) {
    config_ = config;
    grid_dist_ = (config_.hybrid_astar.grid_dist > 0.0) ? config_.hybrid_astar.grid_dist : 1.0;

    const double grid_angle_deg = (config_.hybrid_astar.grid_angle > 0.0) ? config_.hybrid_astar.grid_angle : 1.0;
    grid_angle_rad_ = grid_angle_deg * kPi / 180.0;
    angle_bin_count_ = std::max(8, static_cast<int32_t>(std::round(2.0 * kPi / grid_angle_rad_)));

}

const PlannerConfig& HybridAStar::GetConfig() const {
    return config_;
}

// ---------------------------------------------------------------------------
// 搜索钩子
// ---------------------------------------------------------------------------
void HybridAStar::SetAbortCallback(std::function<bool()> callback) {
    abort_callback_ = std::move(callback);
}

void HybridAStar::SetPoseCollisionHook(std::function<bool(const Pose2D&)> hook) {
    pose_collision_hook_ = std::move(hook);
}

void HybridAStar::SetMotionCollisionHook(
    std::function<bool(const Pose2D&, const Pose2D&)> hook) {
    motion_collision_hook_ = std::move(hook);
}

void HybridAStar::SetGoalHook(std::function<bool(const Pose2D&)> hook) {
    goal_hook_ = std::move(hook);
}

bool HybridAStar::Search(const Pose2D& start, const Pose2D& goal, HybridSearchResult* result) {
    if (result == nullptr) {
        return false;
    }
    result->success = false;
    result->cost = 0.0;
    result->iterations = 0;
    result->path.clear();
    result->message.clear();

    if (!pose_collision_hook_ || !motion_collision_hook_ || !goal_hook_) {
        result->message = "required collision or goal hook is missing";
        return false;
    }
    if (!std::isfinite(start.x) || !std::isfinite(start.y) || !std::isfinite(start.yaw) ||
        !std::isfinite(goal.x) || !std::isfinite(goal.y) || !std::isfinite(goal.yaw)) {
        result->message = "start or goal is not finite";
        return false;
    }

    while (!open_queue_.empty()) {
        open_queue_.pop();
    }
    open_set_.clear();
    closed_set_.clear();
    best_nodes_.clear();
    search_start_pose_ = start;

    Vertex3D start_forward;
    start_forward.x = start.x;
    start_forward.y = start.y;
    start_forward.yaw = planner::math::normalize_angle_rad(start.yaw);
    start_forward.direction = MotionDirection::Forward;
    start_forward.parent_id = 0;
    start_forward.id = VertexHash(start_forward);
    start_forward.g = 0.0;
    start_forward.h = Heuristic(start_forward, goal);
    start_forward.f = start_forward.g + start_forward.h;

    Vertex3D start_backward = start_forward;
    start_backward.direction = MotionDirection::Backward;
    start_backward.id = VertexHash(start_backward);
    start_backward.f = start_backward.h;

    Vertex3D goal_check;
    goal_check.x = goal.x;
    goal_check.y = goal.y;
    goal_check.yaw = planner::math::normalize_angle_rad(goal.yaw);
    goal_check.direction = goal.direction;
    if (IsCollision(start_forward) || IsCollision(goal_check)) {
        result->message = "start or goal in collision";
        return false;
    }

    open_set_[start_forward.id] = start_forward;
    best_nodes_[start_forward.id] = start_forward;
    open_queue_.push(OpenEntry{start_forward.f, start_forward.id});

    open_set_[start_backward.id] = start_backward;
    best_nodes_[start_backward.id] = start_backward;
    open_queue_.push(OpenEntry{start_backward.f, start_backward.id});

    const int32_t max_iterations = std::max(1, static_cast<int32_t>(config_.hybrid_astar.max_iterations_astar));
    uint64_t      final_id = 0;
    bool          found = false;

    for (int32_t iter = 0; iter < max_iterations; ++iter) {
        if (abort_callback_ && abort_callback_()) {
            result->message = "aborted";
            return false;
        }
        result->iterations = iter + 1;
        if (open_queue_.empty()) {
            break;
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
        closed_set_[current.id] = current;

        if (IsGoal(current, goal)) {
            found = true;
            final_id = current.id;
            result->cost = current.g;
            break;
        }

        ExpandNode(current, start, goal);
    }

    if (!found) {
        result->message = "no feasible path";
        return false;
    }

    if (!ReconstructPath(final_id, result)) {
        result->message = "failed to reconstruct path";
        return false;
    }

    result->success = true;
    return true;
}

bool HybridAStar::IsCollision(const Vertex3D& node) const {
    return pose_collision_hook_(Pose2D{node.x, node.y, node.yaw, node.direction});
}

bool HybridAStar::IsGoal(const Vertex3D& node, const Pose2D&) const {
    return goal_hook_(Pose2D{node.x, node.y, node.yaw, node.direction});
}

uint64_t HybridAStar::VertexHash(const Vertex3D& node) const {
    int32_t x_bin = static_cast<int32_t>(std::floor(node.x / grid_dist_));
    int32_t y_bin = static_cast<int32_t>(std::floor(node.y / grid_dist_));
    int32_t angle_bin = static_cast<int32_t>(std::floor((planner::math::normalize_angle_rad(node.yaw) + kPi) / grid_angle_rad_));
    if (angle_bin >= angle_bin_count_) {
        angle_bin %= angle_bin_count_;
    }
    if (angle_bin < 0) {
        angle_bin = (angle_bin % angle_bin_count_ + angle_bin_count_) % angle_bin_count_;
    }

    uint64_t hash = 0U;
    if (node.direction == MotionDirection::Backward) {
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
    hash |= (static_cast<uint64_t>(angle_bin) & 0xFFFULL) << 48U;
    hash |= (static_cast<uint64_t>(x_bin) & 0xFFFFFFULL) << 24U;
    hash |= static_cast<uint64_t>(y_bin) & 0xFFFFFFULL;
    return hash;
}

void HybridAStar::ExpandNode(const Vertex3D& current, const Pose2D& start, const Pose2D& goal) {
    const int32_t steering_num = std::max(2, static_cast<int32_t>(std::round(config_.hybrid_astar.angle_discrete_num)));
    const double min_steering_rad = config_.hybrid_astar.min_steering * kPi / 180.0;
    const double max_steering_rad = config_.hybrid_astar.max_steering * kPi / 180.0;
    const double delta_steering = (max_steering_rad - min_steering_rad) / static_cast<double>(steering_num - 1);

    bool allow_backward = config_.hybrid_astar.allow_reverse;
    if (allow_backward && config_.hybrid_astar.backward_search_range > 0.0) {
        const double dist2_start_sq = (current.x - start.x) * (current.x - start.x) + (current.y - start.y) * (current.y - start.y);
        const double range_sq = config_.hybrid_astar.backward_search_range * config_.hybrid_astar.backward_search_range;
        if (dist2_start_sq > range_sq) {
            allow_backward = false;
        }
    }

    for (int32_t i = 0; i < steering_num; ++i) {
        const double steering = min_steering_rad + static_cast<double>(i) * delta_steering;
        const MotionDirection begin_dir = MotionDirection::Forward;
        const MotionDirection end_dir = allow_backward ? MotionDirection::Backward : MotionDirection::Forward;

        for (int32_t d = static_cast<int32_t>(begin_dir); d <= static_cast<int32_t>(end_dir); ++d) {
            const MotionDirection direction = static_cast<MotionDirection>(d);
            Vertex3D succ = Propagate(current, direction, steering);
            succ.id = VertexHash(succ);
            if (closed_set_.find(succ.id) != closed_set_.end()) {
                continue;
            }
            if (motion_collision_hook_) {
                Pose2D from;
                from.x = current.x;
                from.y = current.y;
                from.yaw = current.yaw;
                from.direction = current.direction;
                Pose2D to;
                to.x = succ.x;
                to.y = succ.y;
                to.yaw = succ.yaw;
                to.direction = succ.direction;
                if (motion_collision_hook_(from, to)) {
                    continue;
                }
            }

            succ.g = TransitionCost(current, succ);
            succ.h = Heuristic(succ, goal);
            succ.f = succ.g + succ.h;

            auto open_it = open_set_.find(succ.id);
            if (open_it == open_set_.end() || succ.g < open_it->second.g) {
                open_set_[succ.id] = succ;
                best_nodes_[succ.id] = succ;
                open_queue_.push(OpenEntry{succ.f, succ.id});
            }
        }
    }
}

HybridAStar::Vertex3D HybridAStar::Propagate(const Vertex3D& start, MotionDirection direction, double steering_rad) const {
    Vertex3D end;
    end.parent_id = start.id;
    end.direction = direction;
    const double sign = direction == MotionDirection::Forward ? 1.0 : -1.0;
    const double curvature = std::abs(steering_rad) < 1e-6 ? 0.0 :
        std::tan(steering_rad) / std::max(1e-3, config_.vehicle.wheel_base);
    const auto pose = planner::math::integrate_arc({start.x, start.y, start.yaw},
                                                   sign * config_.hybrid_astar.delta_dist, curvature);
    end.x = pose.x_m;
    end.y = pose.y_m;
    end.yaw = pose.yaw_rad;
    return end;
}

double HybridAStar::TransitionCost(const Vertex3D& from, const Vertex3D& to) const {
    const double dx_start = from.x - search_start_pose_.x;
    const double dy_start = from.y - search_start_pose_.y;
    const double dist_to_start_sq = dx_start * dx_start + dy_start * dy_start;
    const double coeff = (dist_to_start_sq > config_.hybrid_astar.linear_preferred_distance_square) ? 1.0 : 5.0;
    const double heading_change = std::fabs(planner::math::normalize_angle_rad(to.yaw - from.yaw));
    const double direction_cost =
        (to.direction == MotionDirection::Backward) ? config_.hybrid_astar.backward_penalty : config_.hybrid_astar.forward_penalty;
    const double switch_cost =
        (to.direction == from.direction) ? 0.0 : config_.hybrid_astar.switch_penalty;

    return from.g + config_.hybrid_astar.delta_dist * direction_cost +
           coeff * config_.hybrid_astar.turnning_penalty * heading_change + switch_cost;
}

double HybridAStar::Heuristic(const Vertex3D& node, const Pose2D& goal) const {
    const double euclid = std::hypot(node.x - goal.x, node.y - goal.y);
    const double yaw_diff = std::fabs(planner::math::normalize_angle_rad(node.yaw - goal.yaw));
    return euclid * std::max(1.0, config_.hybrid_astar.forward_penalty) +
           yaw_diff * config_.hybrid_astar.turnning_penalty;
}

bool HybridAStar::ReconstructPath(uint64_t final_id, HybridSearchResult* result) const {
    std::vector<Pose2D> reversed;
    uint64_t current_id = final_id;
    while (true) {
        const auto it = best_nodes_.find(current_id);
        if (it == best_nodes_.end()) {
            return false;
        }
        Pose2D pose;
        pose.x = it->second.x;
        pose.y = it->second.y;
        pose.yaw = it->second.yaw;
        pose.direction = it->second.direction;
        reversed.push_back(pose);

        if (it->second.parent_id == 0 || it->second.parent_id == current_id) {
            break;
        }
        current_id = it->second.parent_id;
    }
    std::reverse(reversed.begin(), reversed.end());
    result->path = reversed;
    return !result->path.empty();
}

} // namespace astar
