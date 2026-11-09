/**
 * @file scene.hpp
 * @brief Scene representation: vehicle footprints, convex obstacles, and occupancy grid frames.
 * @details Defines the collision environment:
 *          - Rectangular vehicle footprint derived from kinematic wheelbase, overhangs, width, and safety margins.
 *          - Convex polygonal obstacles in the local map frame (oriented counter-clockwise).
 *          - Occupancy grid coordinate frames and transformations.
 */

#pragma once

#include <array>
#include <string>
#include <vector>

#include "geometry.hpp"

namespace planner::collision {

using planner::common::GridView;
using planner::common::VehicleModel;

/**
 * @brief Convex polygonal obstacle in the local map coordinate frame.
 */
struct Obstacle {
  std::string id;                 ///< Unique obstacle identifier.
  std::vector<Point2d> vertices;  ///< Closed convex polygon vertices ordered counter-clockwise (>= 3 vertices).

  /**
   * @brief Factory creating an axis-aligned closed rectangular obstacle.
   * @param[in] id Obstacle identifier.
   * @param[in] x_min_m Minimum x in meters.
   * @param[in] y_min_m Minimum y in meters.
   * @param[in] x_max_m Maximum x in meters.
   * @param[in] y_max_m Maximum y in meters.
   * @return Obstacle with 4 CCW vertices, or empty vertices if input is invalid.
   */
  static Obstacle rectangle(const std::string& id, double x_min_m,
                            double y_min_m, double x_max_m, double y_max_m);

  /**
   * @brief Factory creating a convex polygon obstacle normalized to CCW order.
   * @param[in] id Obstacle identifier.
   * @param[in] vertices Polygon vertices.
   * @return Obstacle with CCW vertices, or empty vertices if polygon is non-convex/degenerate.
   */
  static Obstacle convex_polygon(const std::string& id,
                                 std::vector<Point2d> vertices);

  /**
   * @brief Validates obstacle vertex coordinates, count, area, and convexity.
   * @param[out] reason Optional failure description.
   * @return True if obstacle is a valid convex polygon.
   */
  bool valid(std::string* reason = nullptr) const;

  /**
   * @brief Computes axis-aligned bounding box enclosing the obstacle.
   * @return Bounding box Aabb2d.
   */
  Aabb2d aabb() const noexcept;
};

/**
 * @brief Planning and collision checking scene environment.
 */
struct Scene {
  GridView grid;                    ///< 2D occupancy grid view.
  std::vector<Obstacle> obstacles;  ///< Collection of convex polygon obstacles.

  /**
   * @brief Tests whether the scene contains a non-empty grid.
   * @return True if grid.width > 0 and grid.height > 0.
   */
  bool has_grid() const noexcept { return grid.width > 0 && grid.height > 0; }

  /**
   * @brief Validates grid dimensions, resolution, cell buffer, and all obstacles.
   * @param[out] reason Optional failure description.
   * @return True if the entire scene is valid.
   */
  bool valid(std::string* reason = nullptr) const;
};

/**
 * @brief Vehicle rectangular bounding footprint expanded by safety margin.
 * @details Coordinate system: Vehicle body frame with origin at the rear-axle center,
 *          X pointing forward along the longitudinal centerline, Y pointing left.
 */
struct Footprint {
  double x_min_m{0.0};          ///< Body-frame minimum longitudinal extent: -(rear_overhang + margin).
  double x_max_m{0.0};          ///< Body-frame maximum longitudinal extent: wheelbase + front_overhang + margin.
  double y_min_m{0.0};          ///< Body-frame minimum lateral extent: -(width/2 + margin).
  double y_max_m{0.0};          ///< Body-frame maximum lateral extent: +(width/2 + margin).
  double corner_radius_m{0.0};  ///< Maximum corner radius from rear axle, used for bounding swept spheres.
  double half_width_m{0.0};     ///< Half of total vehicle width including safety margin: (width + 2*margin)/2.
  double half_length_m{0.0};    ///< Half of total vehicle length including safety margin.
  double center_x_m{0.0};       ///< Center of the footprint rectangle in body coordinates.

  /**
   * @brief Computes body-frame corners ordered counter-clockwise starting from (x_min, y_min).
   * @return Array of 4 body-frame corner coordinates.
   */
  std::array<Point2d, 4> body_corners() const noexcept;

  /**
   * @brief Computes world-frame corners for a given rear-axle pose.
   * @param[in] pose Vehicle rear-axle pose in world frame.
   * @return Array of 4 world-frame corner coordinates.
   */
  std::array<Point2d, 4> world_corners(const Pose2d& pose) const noexcept;
};

/**
 * @brief Validates vehicle model dimensions (wheelbase, width, overhangs, margin).
 * @param[in] vehicle Vehicle model parameters.
 * @param[out] reason Optional diagnostic failure reason.
 * @return True if dimensions are finite, positive/non-negative as required.
 */
bool vehicle_footprint_valid(const VehicleModel& vehicle,
                             std::string* reason = nullptr);

/**
 * @brief Constructs a Footprint from a VehicleModel.
 * @param[in] vehicle Validated vehicle parameters.
 * @return Rectangular footprint structure.
 */
Footprint make_footprint(const VehicleModel& vehicle) noexcept;

/**
 * @brief Coordinate frame geometry and fast transformations for 2D GridView.
 * @details Cell (r, c) spans [c * res, (c + 1) * res] x [r * res, (r + 1) * res] in grid frame.
 */
struct GridFrame {
  double cos_yaw{1.0};       ///< Cosine of grid origin orientation.
  double sin_yaw{0.0};       ///< Sine of grid origin orientation.
  double origin_x{0.0};      ///< Grid origin X in world coordinates.
  double origin_y{0.0};      ///< Grid origin Y in world coordinates.
  double width_m{0.0};       ///< Total grid width in meters: width * resolution_m.
  double height_m{0.0};      ///< Total grid height in meters: height * resolution_m.
  double resolution_m{0.0};  ///< Side length of square cell in meters.

  /**
   * @brief Constructs GridFrame transformation cache from a GridView.
   * @param[in] grid Occupancy grid view.
   * @return Initialized GridFrame.
   */
  static GridFrame from_grid(const GridView& grid) noexcept;

  /**
   * @brief Transforms point from world coordinates to grid origin frame.
   * @param[in] world_point Point in world frame.
   * @return Point in grid frame.
   */
  Point2d to_grid_frame(const Point2d& world_point) const noexcept;

  /**
   * @brief Checks if a point in grid frame is inside the map rectangle [0, width_m] x [0, height_m].
   * @param[in] grid_point Point in grid coordinates.
   * @return True if inside or on map boundary.
   */
  bool inside_map(const Point2d& grid_point) const noexcept;

  /**
   * @brief Computes signed distance from a point to the map rectangular boundary (negative if outside).
   * @param[in] grid_point Point in grid coordinates.
   * @return Distance to boundary in meters (positive inside, 0 on boundary, negative outside).
   */
  double distance_to_boundary(const Point2d& grid_point) const noexcept;
};

}  // namespace planner::collision
