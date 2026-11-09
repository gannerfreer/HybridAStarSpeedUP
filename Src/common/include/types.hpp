/**
 * @file types.hpp
 * @brief Common data structures, geometric primitives, and types for motion planning.
 * @details Defines standardized units: meters for distances, seconds for time, and radians for angles.
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace planner::common {

/**
 * @brief Double-precision literal of pi, used for angular conversions and normalization.
 */
inline constexpr double kPi = 3.14159265358979323846;

/**
 * @name Grid Cell States
 * @brief Integer values representing occupancy status in GridView::cells (row-major order).
 * @{
 */
inline constexpr int kCellFree = 0;      ///< Cell is traversable and free of obstacles.
inline constexpr int kCellOccupied = 1;  ///< Cell is occupied by an obstacle.
inline constexpr int kCellUnknown = 2;   ///< Cell state is unknown (treated conservatively as occupied).
/// @}

/**
 * @brief Normalizes an angle in radians to the half-open interval [-pi, pi).
 * @param[in] angle_rad Input angle in radians. Must be a finite number.
 * @return Normalized angle in [-pi, pi) radians.
 * @note The caller is responsible for ensuring the input value is finite.
 */
double normalize_angle_rad(double angle_rad);

/**
 * @brief Driving direction / gear of a path point or trajectory segment.
 */
enum class Direction {
  kForward,  ///< Forward driving gear.
  kReverse,  ///< Reverse driving gear.
};

/**
 * @brief Execution status codes for planning and geometric solver routines.
 */
enum class Status {
  kSuccess,             ///< Operation completed successfully.
  kInvalidInput,        ///< Input arguments were invalid or out of range.
  kNoPath,              ///< No collision-free path could be found.
  kTimeout,             ///< Planning terminated due to exceeding time budget.
  kCancelled,           ///< Planning was cancelled by client request or callback.
  kOptimizationFailed,  ///< Numerical solver or trajectory optimization failed.
  kValidationFailed,    ///< Candidate path failed post-search safety validation.
};

/**
 * @brief Converts a Direction enumerator to a human-readable string.
 * @param[in] direction Driving direction.
 * @return String representation ("forward" or "reverse").
 * @throw std::invalid_argument If the enum value is unrecognized.
 */
std::string to_string(Direction direction);

/**
 * @brief Converts a Status enumerator to a human-readable string.
 * @param[in] status Execution status code.
 * @return String representation of the status code.
 * @throw std::invalid_argument If the enum value is unrecognized.
 */
std::string to_string(Status status);

/**
 * @brief 2D Cartesian point in a local right-handed coordinate frame (x forward, y left).
 */
struct Point2d {
  double x_m{0.0};  ///< X coordinate in meters.
  double y_m{0.0};  ///< Y coordinate in meters.
};

/**
 * @brief 2D pose representing the vehicle's rear-axle center in a local right-handed frame.
 * @details X points forward, Y points to the left, and yaw is counter-clockwise positive in radians.
 */
struct Pose2d {
  double x_m{0.0};      ///< X coordinate of rear-axle center in meters.
  double y_m{0.0};      ///< Y coordinate of rear-axle center in meters.
  double yaw_rad{0.0};  ///< Heading angle (yaw) in radians, normalized in [-pi, pi).
};

/**
 * @brief Kinematic and geometric parameter model for a rectangular vehicle footprint.
 * @details Minimum turning radius is derived from wheelbase and maximum steering angle.
 */
struct VehicleModel {
  std::string id;               ///< Vehicle model identifier.
  double wheelbase_m{0.0};       ///< Wheelbase (distance between front and rear axles) in meters.
  double width_m{0.0};           ///< Vehicle body width in meters.
  double front_overhang_m{0.0};  ///< Distance from front axle to front bumper in meters.
  double rear_overhang_m{0.0};   ///< Distance from rear axle to rear bumper in meters.
  double max_steer_rad{0.0};     ///< Maximum front-wheel steering angle in radians.
  double safety_margin_m{0.0};   ///< Additional safety margin added around the vehicle perimeter in meters.

  /**
   * @brief Computes the minimum turning radius at the rear-axle center.
   * @return Minimum turning radius in meters: wheelbase / tan(max_steer).
   */
  double min_turn_radius_m() const { return wheelbase_m / std::tan(max_steer_rad); }
};

/**
 * @brief Read-only 2D occupancy grid view.
 * @details Represents an occupancy grid stored in row-major order. Cell states are:
 *          0 = Free, 1 = Occupied, 2 = Unknown. Unknown and out-of-bounds cells are treated
 *          conservatively as occupied by consumers.
 *
 *          `origin` specifies the pose of the grid's lower-left corner (row=0, col=0)
 *          in the local map coordinate frame. Cell (r, c) center is located at:
 *          \f$ \text{origin} + ((c + 0.5) \cdot \text{res}, (r + 0.5) \cdot \text{res}) \f$.
 */
struct GridView {
  std::string map_id;       ///< Unique map identifier.
  std::string map_version;  ///< Opaque map version string for equality comparisons.
  int width{0};             ///< Grid width (number of columns along x-axis).
  int height{0};            ///< Grid height (number of rows along y-axis).
  double resolution_m{0.0}; ///< Side length of each square grid cell in meters.
  Pose2d origin;            ///< Pose of the grid origin (cell [0,0] bottom-left corner).
  std::vector<int> cells;   ///< Cell occupancy states in row-major order.

  /**
   * @brief Checks if the given row and column indices are within grid bounds.
   * @param[in] row Row index.
   * @param[in] col Column index.
   * @return True if 0 <= row < height and 0 <= col < width, false otherwise.
   */
  bool in_bounds(int row, int col) const {
    return row >= 0 && row < height && col >= 0 && col < width;
  }

  /**
   * @brief Accesses the occupancy state at cell (row, col).
   * @param[in] row Row index.
   * @param[in] col Column index.
   * @return Cell occupancy state value.
   * @throw std::out_of_range If indices are out of bounds.
   */
  int cell(int row, int col) const;
};

/**
 * @brief Waypoint along a vehicle path.
 * @details `s_m` is the non-negative accumulated arc length from path start.
 *          `curvature_inv_m` is the signed curvature defined as \f$ \tan(\delta) / L \f$,
 *          satisfying \f$ \dot{\psi} = v_{\text{signed}} \cdot \kappa \f$.
 */
struct PathPoint {
  Pose2d pose;                  ///< Rear-axle center pose.
  double s_m{0.0};              ///< Accumulated arc length along the path in meters.
  double curvature_inv_m{0.0};  ///< Signed path curvature in 1/m (positive for left turn).
  Direction direction{Direction::kForward};  ///< Motion direction gear.
};

/**
 * @brief Continuous path segment with uniform driving direction.
 * @details Direction switches (cusps) always initiate a new segment.
 *          `s_m` strictly increases within each segment.
 */
struct PathSegment {
  std::vector<PathPoint> points;  ///< Ordered sequence of waypoints along this segment.
};

/**
 * @brief Planning request payload.
 * @details Specifies start/goal poses, vehicle model, and optional computation budgets.
 *          Unset budgets fall back to the planner's configured defaults.
 */
struct PlanRequest {
  std::string request_id;                 ///< Unique identifier for tracking this planning request.
  std::string map_id;                     ///< Target map identifier.
  std::string map_version;                ///< Expected map version string.
  VehicleModel vehicle;                   ///< Vehicle geometric and kinematic parameters.
  Pose2d start;                           ///< Initial vehicle pose (rear-axle center).
  Pose2d goal;                            ///< Target vehicle pose (rear-axle center).
  std::optional<double> time_budget_s;    ///< Optional CPU time limit in seconds.
  std::optional<std::int64_t> node_budget;///< Optional maximum expanded node budget.
  bool cancelled{false};                  ///< Cancellation flag.
};

/**
 * @brief Planning result payload.
 * @details Callers should consume `path` only when `status` is Status::kSuccess.
 */
struct PlanResult {
  std::string request_id;         ///< Echo of the corresponding request ID.
  Status status{Status::kInvalidInput};  ///< Planning execution outcome status.
  std::string reason;             ///< Human-readable diagnostic or error message.
  std::vector<PathSegment> path;  ///< Resulting path divided into directional segments.
};

/**
 * @name Value Equality Operators
 * @brief Strict equality comparisons for planning structures (used for caching and validation).
 * @{
 */
bool operator==(const Point2d& lhs, const Point2d& rhs);
bool operator==(const Pose2d& lhs, const Pose2d& rhs);
bool operator==(const VehicleModel& lhs, const VehicleModel& rhs);
bool operator==(const GridView& lhs, const GridView& rhs);
bool operator==(const PathPoint& lhs, const PathPoint& rhs);
bool operator==(const PathSegment& lhs, const PathSegment& rhs);
/// @}

}  // namespace planner::common
