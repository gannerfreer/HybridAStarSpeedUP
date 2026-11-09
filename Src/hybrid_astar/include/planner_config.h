/**
 * @file planner_config.h
 * @brief Configuration structures for vehicle dimensions, Hybrid A* parameters, and planning problems.
 */

#ifndef astar_PLANNER_CONFIG_H
#define astar_PLANNER_CONFIG_H

#include <string>

namespace astar {

/**
 * @brief Vehicle geometric dimensions and collision safety margins.
 */
struct VehicleConfig {
  double center2front = 1.0;   ///< Distance from vehicle reference center to front bumper (m).
  double center2rear = 1.0;    ///< Distance from vehicle reference center to rear bumper (m).
  double center2side = 0.5;    ///< Distance from vehicle reference center to left/right side (m).
  double safe_margin = 0.0;    ///< Extra safety margin around vehicle body (m).
  double wheel_base = 1.0;     ///< Wheelbase between front and rear axles (m).

  /// Extra map boundary inflation in meters. A value >= 0 applies this margin directly;
  /// negative values indicate no additional inflation.
  double boundary_inflation_meter = -1.0;
};

/**
 * @brief Search parameters and cost coefficients for Hybrid A*.
 */
struct HybridAStarConfig {
  double min_steering = -30.0;                   ///< Minimum steering angle (deg).
  double max_steering = 30.0;                    ///< Maximum steering angle (deg).
  double max_iterations_astar = 100000.0;        ///< Maximum number of node expansions before termination.
  double angle_discrete_num = 8.0;               ///< Number of steering angle discretization levels.
  double delta_dist = 1.0;                       ///< Step distance per expansion arc (m).
  double grid_dist = 1.0;                        ///< Spatial resolution of the 3D state-space grid (m).
  double grid_angle = 1.0;                       ///< Angular resolution of the 3D state-space grid (rad or deg).
  double goal_yaw_tolerance_deg = 1.0;           ///< Heading angle tolerance at goal pose (deg).
  double forward_penalty = 1.0;                  ///< Cost penalty factor for forward driving motion.
  double backward_penalty = 1.0;                 ///< Cost penalty factor for reverse driving motion.
  double turnning_penalty = 1.0;                 ///< Cost penalty factor for steering motion.
  double switch_penalty = 1.0;                   ///< Cost penalty factor for gear switching (forward <-> reverse).
  double backward_search_range = 0.0;            ///< Maximum distance allowed for reverse expansion (m).
  double linear_preferred_distance_square = 0.0; ///< Squared distance threshold favoring straight paths.

  /// Whether reverse driving motions are permitted during node expansion.
  bool allow_reverse = true;

  /// When true, reverse kinematics adheres to the standard bicycle model
  /// (yaw delta = -beta, position updated along circular arc).
  bool standard_reverse_kinematics = false;
};

/**
 * @brief Complete configuration bundling vehicle geometry, search parameters, and boundary conditions.
 */
struct PlannerConfig {
  VehicleConfig vehicle;            ///< Vehicle geometric parameters.
  HybridAStarConfig hybrid_astar;   ///< Hybrid A* search parameters.

  /**
   * @brief Initial and target poses and travel directions.
   */
  struct StartGoalConfig {
    double start_x = 4.0;           ///< Start position X (m).
    double start_y = 4.0;           ///< Start position Y (m).
    double start_yaw_deg = 0.0;     ///< Start heading angle (deg).
    int start_direction = 0;        ///< Initial driving direction (0: unspecified/forward).
    double goal_x = 6.0;            ///< Target position X (m).
    double goal_y = 6.0;            ///< Target position Y (m).
    double goal_yaw_deg = 90.0;     ///< Target heading angle (deg).
    int goal_direction = 0;         ///< Target driving direction.
  } start_goal;
};

}  // namespace astar

#endif  // astar_PLANNER_CONFIG_H
