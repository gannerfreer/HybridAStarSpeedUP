/**
 * @file collision_checker.hpp
 * @brief Continuous and discrete collision checking for vehicle poses, circular motion segments, and paths.
 * @details Core collision invariants:
 *          - Closed set geometry: contact / tangency counts as collision.
 *          - Out-of-bounds map regions and unknown grid cells (kCellUnknown) are treated conservatively as occupied.
 *          - Continuous motion segments are verified via adaptive bisection with conservative displacement
 *            bounding envelopes. Unresolved intervals down to `sweep_precision_m` (default 1e-3 m) are reported
 *            as collision.
 *          - Multi-waypoint paths are checked continuously by reconstructing circular transition arcs between
 *            adjacent points from curvature, distance, and direction gear.
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "scene.hpp"

namespace planner::collision {

using planner::common::PathSegment;

/**
 * @brief High-level outcome status of a collision query.
 */
enum class CheckStatus {
  kClear,        ///< No collision detected across the evaluated pose or trajectory.
  kCollision,    ///< Collision or contact detected (see reason_code and detail).
  kInvalidInput  ///< Input parameters rejected; no collision evaluation performed.
};

/**
 * @brief Specific cause of a detected collision or query rejection.
 */
enum class CollisionReason {
  kNone,            ///< No collision occurred.
  kObstacle,        ///< Overlap or contact with a convex polygonal obstacle.
  kGridOccupied,    ///< Overlap with an occupied grid cell (kCellOccupied).
  kGridUnknown,     ///< Overlap with an unknown grid cell (kCellUnknown, treated as occupied).
  kOutsideMap,      ///< Vehicle footprint partially or completely extends beyond map boundary.
  kPrecisionFloor,  ///< Motion bounding envelope remained unresolved at sweep precision threshold.
  kInvalidInput     ///< Input parameters were non-finite or structurally invalid.
};

/**
 * @brief Converts a CheckStatus enum to a human-readable string.
 * @param[in] status Check status enum value.
 * @return String literal representation.
 */
const char* to_string(CheckStatus status) noexcept;

/**
 * @brief Converts a CollisionReason enum to a human-readable string.
 * @param[in] reason Collision reason enum value.
 * @return String literal representation.
 */
const char* to_string(CollisionReason reason) noexcept;

/**
 * @brief Detailed outcome of a single collision check query.
 */
struct CheckResult {
  CheckStatus status{CheckStatus::kClear};            ///< Overall query status (kClear, kCollision, or kInvalidInput).
  CollisionReason reason_code{CollisionReason::kNone};///< Identified cause of collision.
  std::string detail;                                 ///< Human-readable diagnostic details.
  Pose2d first_pose;                                  ///< First rear-axle center pose where contact was detected.
  double first_s_m{0.0};                              ///< Arc length along segment/path to the first contact point in meters.
  int path_segment_index{-1};                         ///< Index of the failing path segment (for multi-segment path queries).

  /**
   * @brief Checks if a collision occurred.
   * @return True if status == CheckStatus::kCollision.
   */
  bool collided() const noexcept { return status == CheckStatus::kCollision; }

  /**
   * @brief Checks if the query was completely collision-free.
   * @return True if status == CheckStatus::kClear.
   */
  bool is_clear() const noexcept { return status == CheckStatus::kClear; }

  /**
   * @brief Checks if inputs were valid and a check was performed.
   * @return True if status != CheckStatus::kInvalidInput.
   */
  bool is_valid() const noexcept { return status != CheckStatus::kInvalidInput; }
};

/**
 * @brief Configuration options controlling collision checking precision and acceleration.
 */
struct CheckOptions {
  /**
   * @brief Adaptive bisection precision floor in meters (default: 1e-3 m).
   * @details Upper bound on the maximum displacement of any point on the vehicle body
   *          within a single leaf swept interval.
   */
  double sweep_precision_m{1e-3};

  /**
   * @brief Enables spatial bounding-volume acceleration indices.
   * @details When disabled, pure geometric tests are run directly.
   */
  bool enable_acceleration{false};
};

/**
 * @brief Constant-curvature motion primitive segment.
 * @details Defined by start pose, arc length, gear direction, and signed curvature.
 *          Satisfies \f$ \Delta\psi = \kappa \cdot s_{\text{signed}} \f$.
 */
struct MotionSegment {
  Pose2d start;                             ///< Start pose of the vehicle rear-axle center.
  double length_m{0.0};                     ///< Arc length traveled in meters (>= 0).
  Direction direction{Direction::kForward}; ///< Motion gear (forward or reverse).
  double curvature_inv_m{0.0};              ///< Signed curvature \f$ \kappa = \tan(\delta)/L \f$ in 1/m.
};

/**
 * @brief Checks static collision of a vehicle footprint at a single pose.
 * @param[in] pose Vehicle rear-axle pose.
 * @param[in] vehicle Vehicle geometric model.
 * @param[in] scene Environment scene (grid and obstacles).
 * @param[in] options Collision check options.
 * @return Query result indicating clear or first contact cause.
 */
CheckResult check_pose(const Pose2d& pose, const VehicleModel& vehicle,
                       const Scene& scene, const CheckOptions& options = {});

/**
 * @brief Checks continuous collision along a constant-curvature circular motion segment.
 * @param[in] segment Circular motion segment.
 * @param[in] vehicle Vehicle geometric model.
 * @param[in] scene Environment scene.
 * @param[in] options Collision check options.
 * @return Query result indicating clear or first contact pose and arc length.
 */
CheckResult check_motion(const MotionSegment& segment, const VehicleModel& vehicle,
                         const Scene& scene, const CheckOptions& options = {});

/**
 * @brief Checks continuous collision along a single directional path segment.
 * @param[in] segment Waypoint sequence forming a continuous path segment.
 * @param[in] vehicle Vehicle geometric model.
 * @param[in] scene Environment scene.
 * @param[in] options Collision check options.
 * @return Query result.
 */
CheckResult check_path(const PathSegment& segment, const VehicleModel& vehicle,
                       const Scene& scene, const CheckOptions& options = {});

/**
 * @brief Checks continuous collision along an ordered sequence of path segments.
 * @param[in] path Multi-segment path.
 * @param[in] vehicle Vehicle geometric model.
 * @param[in] scene Environment scene.
 * @param[in] options Collision check options.
 * @return Query result with path_segment_index indicating the first failing segment.
 */
CheckResult check_path(const std::vector<PathSegment>& path,
                       const VehicleModel& vehicle, const Scene& scene,
                       const CheckOptions& options = {});

struct CollisionCheckerImpl;

/**
 * @brief Reusable collision checker caching vehicle footprint and optional scene acceleration structures.
 */
class CollisionChecker {
 public:
  /**
   * @brief Constructs and validates a reusable collision checker instance.
   * @param[in] vehicle Vehicle geometric model.
   * @param[in] scene Environment scene snapshot.
   * @param[in] options Collision checking configuration options.
   */
  CollisionChecker(const VehicleModel& vehicle, const Scene& scene,
                   CheckOptions options = {});

  /**
   * @brief Returns true if inputs were validated successfully.
   * @return True if valid, false if rejected.
   */
  bool valid() const noexcept;

  /**
   * @brief Returns error diagnostic message if construction validation failed.
   * @return Diagnostic string.
   */
  const std::string& invalid_reason() const noexcept;

  const VehicleModel& vehicle() const noexcept;
  const Scene& scene() const noexcept;
  const CheckOptions& options() const noexcept;

  CheckResult check_pose(const Pose2d& pose) const;
  CheckResult check_motion(const MotionSegment& segment) const;
  CheckResult check_path(const PathSegment& segment) const;
  CheckResult check_path(const std::vector<PathSegment>& path) const;

 private:
  std::shared_ptr<const CollisionCheckerImpl> state_;
};

}  // namespace planner::collision
