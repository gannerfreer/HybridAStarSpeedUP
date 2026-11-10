/**
 * @file planner_config.cpp
 * @brief Hybrid A* 规划器配置参数合法性校验的具体实现。
 */

#include "planner/common/planner_config.hpp"

#include <cmath>
#include <string>

namespace planner::common {

namespace {

bool finite_positive(double value, const char* name, std::string* reason) {
  if (!std::isfinite(value) || !(value > 0.0)) {
    if (reason) {
      *reason = std::string("planner config ") + name + " must be finite and > 0";
    }
    return false;
  }
  return true;
}

bool finite_nonnegative(double value, const char* name, std::string* reason) {
  if (!std::isfinite(value) || !(value >= 0.0)) {
    if (reason) {
      *reason = std::string("planner config ") + name + " must be finite and >= 0";
    }
    return false;
  }
  return true;
}

}  // namespace

bool planner_config_valid(const UserAstarConfig& config, std::string* reason) {
  if (!finite_positive(config.xy_resolution_m, "xy_resolution_m", reason)) {
    return false;
  }
  if (!finite_positive(config.motion_step_m, "motion_step_m", reason)) {
    return false;
  }
  if (!finite_positive(config.goal_capture_radius_m, "goal_capture_radius_m",
                       reason)) {
    return false;
  }
  if (!std::isfinite(config.default_time_budget_s) ||
      !(config.default_time_budget_s > 0.0) ||
      config.default_time_budget_s > 1e9) {
    if (reason) {
      *reason = "planner config default_time_budget_s must be finite and in (0, 1e9]";
    }
    return false;
  }
  if (!finite_nonnegative(config.forward_cost, "forward_cost", reason)) {
    return false;
  }
  if (!finite_nonnegative(config.reverse_cost, "reverse_cost", reason)) {
    return false;
  }
  if (!finite_nonnegative(config.turn_penalty, "turn_penalty", reason)) {
    return false;
  }
  if (!finite_nonnegative(config.switch_cost, "switch_cost", reason)) {
    return false;
  }
  if (!finite_nonnegative(config.backward_search_range_m,
                          "backward_search_range_m", reason)) {
    return false;
  }
  if (!finite_nonnegative(config.linear_preferred_distance_m2,
                          "linear_preferred_distance_m2", reason)) {
    return false;
  }
  if (!finite_nonnegative(config.analytic_shot_distance_m,
                          "analytic_shot_distance_m", reason)) {
    return false;
  }
  if (!finite_nonnegative(config.analytic_expansion_max_length_m,
                          "analytic_expansion_max_length_m", reason)) {
    return false;
  }
  if (!finite_nonnegative(config.min_connector_segment_m,
                          "min_connector_segment_m", reason)) {
    return false;
  }
  if (config.heading_bins < 8) {
    if (reason) *reason = "planner config heading_bins must be >= 8";
    return false;
  }
  if (config.steering_samples < 2) {
    if (reason) *reason = "planner config steering_samples must be >= 2";
    return false;
  }
  if (config.analytic_expansion_ratio < 1) {
    if (reason) *reason = "planner config analytic_expansion_ratio must be >= 1";
    return false;
  }
  if (config.default_node_budget < 1) {
    if (reason) *reason = "planner config default_node_budget must be >= 1";
    return false;
  }
  return true;
}

}  // namespace planner::common
