/**
 * @file reeds_shepp.hpp
 * @brief Reeds-Shepp (RS) analytic shortest path solver, segmentation, and waypoint sampling.
 * @details Implements the native 48-curve analytic Reeds-Shepp model for car-like vehicles
 *          capable of moving both forwards and backwards with bounded turning radius.
 *
 * Geometric Conventions:
 * - Pose2d represents the vehicle's rear-axle center (x forward, y left, yaw counter-clockwise positive).
 * - Path curvature is defined as \f$ \kappa = \tan(\delta) / L \f$, satisfying
 *   \f$ \dot{\psi} = v_{\text{signed}} \cdot \kappa \f$. Curvature sign denotes physical steering
 *   direction (positive for left turn) and is independent of driving gear.
 * - Every path produced satisfies \f$ |\kappa| \le 1 / R_{\min} \f$.
 */

#pragma once

#include <string>
#include <vector>

#include "types.hpp"

namespace planner::rs {

/**
 * @name Endpoint Numerical Tolerances
 * @{
 */
inline constexpr double kEndpointPositionToleranceM = 1e-6;  ///< Endpoint position tolerance in meters.
inline constexpr double kEndpointYawToleranceRad = 1e-6;     ///< Endpoint heading tolerance in radians.
/// @}

/**
 * @brief Single constant-curvature segment of a Reeds-Shepp path.
 */
struct RsSegment {
  common::Direction direction{common::Direction::kForward};  ///< Motion direction / gear (forward or reverse).
  double length_m{0.0};         ///< Traversed arc length in meters (always >= 0).
  double curvature_inv_m{0.0};  ///< Signed curvature \f$ \tan(\delta)/L \f$ in 1/m (0 for straight lines, \f$\pm 1/R_{\min}\f$ for arcs).
};

/**
 * @brief Shortest Reeds-Shepp path between two vehicle poses.
 * @details Zero-distance queries (start == goal) result in an empty `segments` vector.
 */
struct RsPath {
  common::Pose2d start;             ///< Echo of the query start pose (normalized).
  common::Pose2d goal;              ///< Echo of the query goal pose (normalized).
  double length_m{0.0};             ///< Total path length in meters (sum of segment lengths).
  std::vector<RsSegment> segments;  ///< Sequence of motion segments ordered from start to goal.
};

/**
 * @brief Non-throwing result structure for Reeds-Shepp path planning queries.
 */
struct RsPlan {
  common::Status status{common::Status::kInvalidInput};  ///< Outcome status code (kSuccess, kInvalidInput, or kOptimizationFailed).
  std::string reason;   ///< Diagnostic description if planning failed.
  RsPath path;          ///< Planned shortest path (valid if status == Status::kSuccess).
};

/**
 * @brief Validates inputs for a Reeds-Shepp query.
 * @param[in] start Start pose.
 * @param[in] goal Goal pose.
 * @param[in] vehicle Vehicle model parameters.
 * @return True if poses are finite, wheelbase > 0, 0 < max_steer < pi/2, and min turn radius is positive.
 */
bool is_valid_query(const common::Pose2d& start, const common::Pose2d& goal,
                    const common::VehicleModel& vehicle) noexcept;

/**
 * @brief Computes minimum turning radius from vehicle wheelbase and maximum steer angle.
 * @param[in] vehicle Vehicle model.
 * @return Minimum turning radius in meters: wheelbase / tan(max_steer).
 * @throw std::invalid_argument If vehicle parameters are invalid.
 */
double min_turn_radius_m(const common::VehicleModel& vehicle);

/**
 * @brief Computes the shortest Reeds-Shepp path between two poses (non-throwing).
 * @param[in] start Start pose of the vehicle rear-axle center.
 * @param[in] goal Goal pose of the vehicle rear-axle center.
 * @param[in] vehicle Vehicle kinematic model.
 * @return RsPlan containing status, error message (if any), and computed path.
 */
RsPlan plan(const common::Pose2d& start, const common::Pose2d& goal,
            const common::VehicleModel& vehicle);

/**
 * @brief Computes the shortest Reeds-Shepp distance between two poses (throwing wrapper).
 * @param[in] start Start pose.
 * @param[in] goal Goal pose.
 * @param[in] vehicle Vehicle kinematic model.
 * @return Shortest path length in meters.
 * @throw std::invalid_argument If inputs are invalid.
 * @throw std::runtime_error If path generation or validation failed.
 */
double distance(const common::Pose2d& start, const common::Pose2d& goal,
                const common::VehicleModel& vehicle);

/**
 * @brief Computes the shortest Reeds-Shepp path between two poses (throwing wrapper).
 * @param[in] start Start pose.
 * @param[in] goal Goal pose.
 * @param[in] vehicle Vehicle kinematic model.
 * @return Planned shortest Reeds-Shepp path.
 * @throw std::invalid_argument If inputs are invalid.
 * @throw std::runtime_error If path generation or validation failed.
 */
RsPath path(const common::Pose2d& start, const common::Pose2d& goal,
            const common::VehicleModel& vehicle);

/**
 * @brief Integrates a pose along a constant-curvature segment by arc length ds_m.
 * @param[in] start Initial pose at segment start.
 * @param[in] segment Motion segment defining curvature and gear.
 * @param[in] ds_m Distance to advance in meters (must satisfy 0 <= ds_m <= segment.length_m).
 * @return Integrated pose at distance ds_m.
 * @throw std::invalid_argument If ds_m is negative or exceeds segment length.
 */
common::Pose2d integrate_segment(const common::Pose2d& start,
                                 const RsSegment& segment, double ds_m);

/**
 * @brief Samples a Reeds-Shepp path at roughly uniform arc length intervals.
 * @details Guarantees that:
 *          - First waypoint is exactly `path.start` with s_m = 0;
 *          - Final waypoint is exactly `path.goal` with s_m = path.length_m;
 *          - All segment boundaries (cusps) are preserved as exact waypoints;
 *          - Spacing between adjacent points is at most `ds_m`.
 * @param[in] path Reeds-Shepp path to sample.
 * @param[in] ds_m Nominal sample step size in meters (> 0).
 * @return Vector of sampled PathPoints.
 * @throw std::invalid_argument If ds_m <= 0, non-finite, or resulting sample count exceeds safety bounds.
 */
std::vector<common::PathPoint> sample(const RsPath& path, double ds_m);

/**
 * @brief Evaluates an exact waypoint at cumulative arc length `s_m`.
 * @param[in] path Reeds-Shepp path.
 * @param[in] s_m Target arc length along path in meters, in [0, path.length_m].
 * @return Waypoint at distance s_m.
 * @throw std::out_of_range If s_m < 0 or s_m > path.length_m.
 * @throw std::invalid_argument If s_m is non-finite or path is malformed.
 */
common::PathPoint sample_at_s(const RsPath& path, double s_m);

}  // namespace planner::rs
