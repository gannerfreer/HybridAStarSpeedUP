/**
 * @file user_astar.hpp
 * @brief High-level Hybrid A* path planner interface integrating collision checking and analytic expansions.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>

#include "collision_checker.hpp"
#include "types.hpp"

namespace planner::hybrid_astar {

/**
 * @brief Configuration parameters for the Hybrid A* planner.
 */
struct UserAstarConfig {
  double xy_resolution_m = 0.5;                   ///< Spatial grid cell resolution for state-space discretization (m).
  int heading_bins = 72;                          ///< Number of discrete heading bins across [0, 2*pi).
  double motion_step_m = 1.0;                     ///< Step distance per kinodynamic expansion arc (m).
  int steering_samples = 5;                       ///< Number of discrete steering angles evaluated per expansion.
  bool allow_reverse = true;                      ///< Whether reverse driving motions are permitted.
  double forward_cost = 1.0;                      ///< Cost weight for forward driving motion.
  double reverse_cost = 2.0;                      ///< Cost weight for reverse driving motion.
  double turn_penalty = 1.0;                      ///< Penalty multiplier for steering away from straight.
  double switch_cost = 2.0;                       ///< Penalty cost for direction changes (forward <-> reverse).
  double backward_search_range_m = 0.0;           ///< Maximum travel distance allowed in reverse; 0 means unconstrained.
  double linear_preferred_distance_square = 0.0;  ///< Squared distance threshold favoring straight paths near start.
  double goal_yaw_tolerance_deg = 1.0;            ///< Heading angle tolerance at goal pose (deg).
  int analytic_expansion_ratio = 3;               ///< Frequency ratio for attempting Reeds-Shepp analytic expansions.
  double analytic_shot_distance_m = 0.0;          ///< Maximum Euclidean distance to goal to attempt analytic expansion; 0 means unlimited.

  /// Maximum allowed Reeds-Shepp connector path length (m).
  /// Rejects overly long or greedy connections, encouraging the search to approach closer before connecting.
  /// 0 means unlimited connector length.
  double analytic_expansion_max_length_m = 12.0;

  double goal_capture_radius_m = 8.0;             ///< Distance threshold to goal within which direct capture is enabled (m).

  /// Minimum segment length in Reeds-Shepp curves (m).
  /// Sub-segments below this threshold are treated as numerical degeneracies (causing spurious gear switches) and rejected.
  double min_connector_segment_m = 0.15;

  double default_time_budget_s = 5.0;             ///< Default maximum planning time budget (s).
  std::int64_t default_node_budget = 1000000;     ///< Default maximum number of node expansions.
};

/**
 * @brief Thread-compatible Hybrid A* planner instance.
 * @note Individual instances are not thread-safe and must not be called concurrently.
 *       The map grid, vehicle model, and collision checker must remain immutable throughout plan().
 */
class UserAstar {
 public:
  /**
   * @brief Constructs a UserAstar planner with optional initial configuration.
   * @param[in] config Planner configuration parameters.
   */
  explicit UserAstar(const UserAstarConfig& config = {});

  /**
   * @brief Destructor.
   */
  ~UserAstar();

  UserAstar(const UserAstar&) = delete;
  UserAstar& operator=(const UserAstar&) = delete;

  /**
   * @brief Updates the planner configuration.
   * @param[in] config New configuration to apply.
   * @throw std::invalid_argument If configuration parameters are invalid (preserves previous config).
   */
  void configure(const UserAstarConfig& config);

  /**
   * @brief Retrieves the active planner configuration.
   * @return Const reference to active configuration.
   */
  const UserAstarConfig& config() const;

  /**
   * @brief Plans a collision-free kinodynamic path from start to goal.
   * @param[in] request Planning query containing start/goal poses and bounds.
   * @param[in] grid Occupancy grid view for obstacle detection.
   * @param[in] collision_checker Checker for swept vehicle footprint evaluation.
   * @return Result structure containing planning status, trajectory, and statistics.
   */
  common::PlanResult plan(const common::PlanRequest& request,
                          const common::GridView& grid,
                          const collision::CollisionChecker& collision_checker);

  /**
   * @brief Clears cached heuristic grids and internal lookup tables.
   */
  void reset_cache();

  /**
   * @brief Sets an external cancellation callback polled during search.
   * @param[in] callback Function returning true when planning should abort.
   */
  void set_cancel_callback(std::function<bool()> callback);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace planner::hybrid_astar
