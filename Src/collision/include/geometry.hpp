/**
 * @file geometry.hpp
 * @brief 2D computational geometry primitives, Separating Axis Theorem (SAT), and distance queries.
 * @details Core conventions:
 *          - All geometric shapes are treated as closed sets: contact (tangency) is classified as intersection.
 *          - Convex obstacle polygons are ordered counter-clockwise (CCW).
 *          - Numerical evaluations are conservative: small scale-dependent tolerances ensure floating-point
 *            rounding never masks contact.
 */

#pragma once

#include <string>
#include <vector>

#include "types.hpp"

namespace planner::collision {

using planner::common::Direction;
using planner::common::Point2d;
using planner::common::Pose2d;

/**
 * @name Basic 2D Vector Algebra
 * @{
 */

/**
 * @brief Computes 2D vector cross product (z-component of 3D cross product): \f$ a_x b_y - a_y b_x \f$.
 * @param[in] a First vector.
 * @param[in] b Second vector.
 * @return Scalar cross product.
 */
double cross(const Point2d& a, const Point2d& b) noexcept;

/**
 * @brief Computes 2D vector dot product: \f$ a_x b_x + a_y b_y \f$.
 * @param[in] a First vector.
 * @param[in] b Second vector.
 * @return Scalar dot product.
 */
double dot(const Point2d& a, const Point2d& b) noexcept;

Point2d operator+(const Point2d& a, const Point2d& b) noexcept;
Point2d operator-(const Point2d& a, const Point2d& b) noexcept;
Point2d operator*(const Point2d& a, double s) noexcept;

/**
 * @brief Computes Euclidean norm (length) of a 2D point / vector.
 * @param[in] a Input vector.
 * @return Length of vector.
 */
double norm(const Point2d& a) noexcept;

/**
 * @brief Computes Euclidean distance between two 2D points.
 * @param[in] a First point.
 * @param[in] b Second point.
 * @return Distance in meters.
 */
double distance(const Point2d& a, const Point2d& b) noexcept;

/**
 * @brief Transforms a point from vehicle body frame to world frame given a pose.
 * @param[in] pose Vehicle pose in world frame.
 * @param[in] body_point Point coordinates in body frame.
 * @return Point coordinates in world frame.
 */
Point2d transform_point(const Pose2d& pose, const Point2d& body_point) noexcept;

/**
 * @brief Inverse transforms a point from world frame to vehicle body frame.
 * @param[in] pose Vehicle pose in world frame.
 * @param[in] world_point Point coordinates in world frame.
 * @return Point coordinates in vehicle body frame.
 */
Point2d to_body_frame(const Pose2d& pose, const Point2d& world_point) noexcept;
/// @}

/**
 * @name Scale-Dependent Numerical Tolerances
 * @{
 */

/**
 * @brief Computes scale-dependent numerical tolerance for contact classification.
 * @details Two shapes are considered in contact if their minimum Euclidean distance is <= contact_tolerance(scale).
 * @param[in] scale_m Coordinate magnitude scale in meters.
 * @return Numerical contact tolerance in meters (up to 1e-12 * scale).
 */
double contact_tolerance(double scale_m) noexcept;
/// @}

/**
 * @name 2D Axis-Aligned Bounding Box (AABB)
 * @{
 */

/**
 * @brief 2D axis-aligned bounding box.
 */
struct Aabb2d {
  double min_x{0.0};  ///< Minimum x coordinate.
  double min_y{0.0};  ///< Minimum y coordinate.
  double max_x{0.0};  ///< Maximum x coordinate.
  double max_y{0.0};  ///< Maximum y coordinate.

  /**
   * @brief Tests if point p is contained within or on the boundary of the AABB.
   * @param[in] p Query point.
   * @return True if point is inside or on boundary.
   */
  bool contains(const Point2d& p) const noexcept;

  /**
   * @brief Tests if this AABB intersects another closed AABB (touching counts as intersection).
   * @param[in] other Other AABB.
   * @return True if boxes overlap or touch.
   */
  bool intersects(const Aabb2d& other) const noexcept;

  /**
   * @brief Expands the bounding box outward by margin on all sides.
   * @param[in] margin Expansion distance in meters.
   */
  void expand(double margin) noexcept;

  /**
   * @brief Expands this AABB to enclose point p.
   * @param[in] p Point to merge.
   */
  void merge_point(const Point2d& p) noexcept;

  /**
   * @brief Computes the maximum absolute coordinate value of the box corners.
   * @return Maximum absolute coordinate.
   */
  double max_abs_coordinate() const noexcept;
};

/**
 * @brief Computes tight axis-aligned bounding box enclosing a set of 2D points.
 * @param[in] points Point set.
 * @return Enclosing Aabb2d.
 */
Aabb2d aabb_of(const std::vector<Point2d>& points) noexcept;

/**
 * @brief Computes Euclidean distance between two closed AABBs (0 if overlapping or touching).
 * @param[in] a First AABB.
 * @param[in] b Second AABB.
 * @return Distance in meters.
 */
double aabb_distance(const Aabb2d& a, const Aabb2d& b) noexcept;

/**
 * @brief Fast square-domain reachability filter between two AABBs.
 * @details Equivalent to `aabb_distance(a, b) <= reach` without computing square roots.
 * @param[in] a First AABB.
 * @param[in] b Second AABB.
 * @param[in] reach Reach threshold distance in meters.
 * @return True if distance <= reach.
 */
bool aabb_within(const Aabb2d& a, const Aabb2d& b, double reach) noexcept;

/**
 * @brief Distance from point to closed AABB (0 if inside or on boundary).
 * @param[in] p Query point.
 * @param[in] box Target AABB.
 * @return Distance in meters.
 */
double point_aabb_distance(const Point2d& p, const Aabb2d& box) noexcept;

/**
 * @brief Squared distance from point to closed AABB.
 * @param[in] p Query point.
 * @param[in] box Target AABB.
 * @return Squared distance in m^2.
 */
double point_aabb_distance_sq(const Point2d& p, const Aabb2d& box) noexcept;
/// @}

/**
 * @name Convex Polygons and Separating Axis Theorem (SAT)
 * @{
 */

/**
 * @brief Computes twice the signed area of a polygon (positive for counter-clockwise orientation).
 * @param[in] polygon Sequence of polygon vertices.
 * @return Twice the signed area in m^2.
 */
double signed_area_twice(const std::vector<Point2d>& polygon) noexcept;

/**
 * @brief Validates if a polygon is non-degenerate and convex.
 * @param[in] polygon Sequence of vertices.
 * @param[out] reason Optional diagnostic failure reason.
 * @return True if polygon has >= 3 distinct finite vertices, non-zero area, and is convex.
 */
bool convex_polygon_valid(const std::vector<Point2d>& polygon,
                          std::string* reason = nullptr);

/**
 * @brief Checks if polygon vertices are ordered counter-clockwise.
 * @param[in] polygon Vertex sequence.
 * @return True if CCW, false if CW or degenerate.
 */
bool is_counter_clockwise(const std::vector<Point2d>& polygon) noexcept;

/**
 * @brief In-place normalizes polygon vertex order to counter-clockwise.
 * @param[in,out] polygon Polygon whose vertices will be reversed if clockwise.
 */
void normalize_counter_clockwise(std::vector<Point2d>& polygon);

/**
 * @brief Constructs a counter-clockwise convex polygon from axis-aligned rectangle coordinates.
 * @param[in] x_min Minimum x.
 * @param[in] y_min Minimum y.
 * @param[in] x_max Maximum x.
 * @param[in] y_max Maximum y.
 * @param[out] reason Optional failure reason.
 * @return Vector of 4 CCW corner points.
 */
std::vector<Point2d> rectangle_polygon(double x_min, double y_min,
                                       double x_max, double y_max,
                                       std::string* reason = nullptr);

/**
 * @brief Computes 2D convex hull of a point set using Monotone Chain algorithm.
 * @param[in] points Input point cloud.
 * @return CCW convex hull polygon vertices.
 */
std::vector<Point2d> convex_hull(std::vector<Point2d> points);

/**
 * @brief Tests intersection between two closed convex polygons using Separating Axis Theorem (SAT).
 * @param[in] a First convex polygon.
 * @param[in] b Second convex polygon.
 * @param[in] tol Distance tolerance within which shapes are considered touching/intersecting.
 * @return True if polygons intersect or touch within tolerance.
 */
bool convex_polygons_intersect(const std::vector<Point2d>& a,
                               const std::vector<Point2d>& b,
                               double tol = 0.0) noexcept;

/**
 * @brief Tests intersection between a convex polygon and an AABB using SAT.
 * @param[in] polygon Convex polygon.
 * @param[in] box Axis-aligned bounding box.
 * @param[in] tol Distance tolerance.
 * @return True if polygon and box intersect or touch within tolerance.
 */
bool convex_polygon_intersects_aabb(const std::vector<Point2d>& polygon,
                                    const Aabb2d& box,
                                    double tol = 0.0) noexcept;
/// @}

/**
 * @name Exact Geometric Distance Calculations
 * @{
 */

/**
 * @brief Computes Euclidean distance from a point to a closed convex polygon (0 if inside/on boundary).
 * @param[in] p Query point.
 * @param[in] polygon Convex polygon.
 * @return Minimum Euclidean distance in meters.
 */
double point_polygon_distance(const Point2d& p,
                              const std::vector<Point2d>& polygon) noexcept;

/**
 * @brief Squared Euclidean distance from a point to a closed convex polygon.
 * @param[in] p Query point.
 * @param[in] polygon Convex polygon.
 * @return Squared distance in m^2.
 */
double point_polygon_distance_sq(const Point2d& p,
                                 const std::vector<Point2d>& polygon) noexcept;

/**
 * @brief Computes distance from a point to a line segment [a, b].
 * @param[in] p Query point.
 * @param[in] a Segment start endpoint.
 * @param[in] b Segment end endpoint.
 * @return Distance in meters.
 */
double point_segment_distance(const Point2d& p, const Point2d& a,
                              const Point2d& b) noexcept;

/**
 * @brief Squared distance from a point to a line segment [a, b].
 * @param[in] p Query point.
 * @param[in] a Segment start endpoint.
 * @param[in] b Segment end endpoint.
 * @return Squared distance in m^2.
 */
double point_segment_distance_sq(const Point2d& p, const Point2d& a,
                                 const Point2d& b) noexcept;

/**
 * @brief Computes minimum distance between two line segments [a0, a1] and [b0, b1].
 * @param[in] a0 First segment start.
 * @param[in] a1 First segment end.
 * @param[in] b0 Second segment start.
 * @param[in] b1 Second segment end.
 * @return Minimum distance in meters.
 */
double segment_segment_distance(const Point2d& a0, const Point2d& a1,
                                const Point2d& b0, const Point2d& b1) noexcept;

/**
 * @brief Squared minimum distance between two line segments.
 * @param[in] a0 First segment start.
 * @param[in] a1 First segment end.
 * @param[in] b0 Second segment start.
 * @param[in] b1 Second segment end.
 * @return Squared distance in m^2.
 */
double segment_segment_distance_sq(const Point2d& a0, const Point2d& a1,
                                   const Point2d& b0,
                                   const Point2d& b1) noexcept;

/**
 * @brief Computes Euclidean distance between two closed convex polygons (0 if intersecting or touching).
 * @param[in] a First convex polygon.
 * @param[in] b Second convex polygon.
 * @return Minimum Euclidean distance in meters.
 */
double convex_polygon_distance(const std::vector<Point2d>& a,
                               const std::vector<Point2d>& b) noexcept;

/**
 * @brief Squared Euclidean distance between two closed convex polygons.
 * @param[in] a First convex polygon.
 * @param[in] b Second convex polygon.
 * @return Squared distance in m^2.
 */
double convex_polygon_distance_sq(const std::vector<Point2d>& a,
                                  const std::vector<Point2d>& b) noexcept;

/**
 * @brief Minimum Euclidean distance between a convex polygon and a closed AABB (0 if touching/overlapping).
 * @param[in] polygon Convex polygon.
 * @param[in] box Target AABB.
 * @return Distance in meters.
 */
double polygon_aabb_distance(const std::vector<Point2d>& polygon,
                             const Aabb2d& box) noexcept;

/**
 * @brief Squared Euclidean distance between a convex polygon and a closed AABB.
 * @param[in] polygon Convex polygon.
 * @param[in] box Target AABB.
 * @return Squared distance in m^2.
 */
double polygon_aabb_distance_sq(const std::vector<Point2d>& polygon,
                                const Aabb2d& box) noexcept;
/// @}

}  // namespace planner::collision
