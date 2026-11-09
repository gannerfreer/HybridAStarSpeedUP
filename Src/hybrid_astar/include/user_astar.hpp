#pragma once

#include <functional>
#include <memory>
#include "types.hpp"
#include "collision_checker.hpp"

namespace planner::hybrid_astar {

struct UserAstarConfig {
  double xy_resolution_m = 0.5;
  int heading_bins = 72;
  double motion_step_m = 1.0;
  int steering_samples = 5;
  bool allow_reverse = true;
  double forward_cost = 1.0;
  double reverse_cost = 2.0;
  double turn_penalty = 1.0;
  double switch_cost = 2.0;
  double backward_search_range_m = 0.0;  // 0 = 不限制
  double linear_preferred_distance_square = 0.0;
  double goal_yaw_tolerance_deg = 1.0;
  int analytic_expansion_ratio = 3;
  double analytic_shot_distance_m = 0.0;        // 0 = 无距离限制
  // RS 连接长度上限（默认 11.8 m）：拒绝 park_116/118 的贪心长连接
  // （12.0–12.65 m）并迫使搜索靠近后再连接，同时保留 ≤11.65 m 的次优
  // 连接；0 = 无长度限制（完整性优先）。
  double analytic_expansion_max_length_m = 12.0;
  double goal_capture_radius_m = 8.0;
  double min_connector_segment_m = 0.15;  // RS 连接中低于该长度的分段视为数值
                                          // 退化（会制造虚假换向），拒绝后重试
  double default_time_budget_s = 5.0;
  std::int64_t default_node_budget = 1000000;
};

// 单个实例不可并发调用。地图、车辆和碰撞检测器在 plan 期间须保持不变。
class UserAstar {
 public:
  explicit UserAstar(const UserAstarConfig& config = {});
  ~UserAstar();
  UserAstar(const UserAstar&) = delete;
  UserAstar& operator=(const UserAstar&) = delete;
  // 非法配置抛出 std::invalid_argument；失败时保留原配置。
  void configure(const UserAstarConfig& config);
  const UserAstarConfig& config() const;
  common::PlanResult plan(const common::PlanRequest& request,
                          const common::GridView& grid,
                          const collision::CollisionChecker& collision_checker);
  void reset_cache();
  void set_cancel_callback(std::function<bool()> callback);
 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace planner::hybrid_astar
