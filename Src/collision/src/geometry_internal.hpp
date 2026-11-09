/**
 * @file geometry_internal.hpp
 * @brief Internal fast-path geometry data structures and precomputed SAT caches.
 * @details Optimizes collision query hot paths by:
 *          - Using unnormalized edge normal axes and comparing projection gap squared:
 *            `d > 0 && d * d > tol * tol * axis_len_sq`, eliminating per-axis hypot and division.
 *          - Precomputing and caching polygon projection intervals along edge normal axes.
 *          - Falling back to the normalized SAT implementation when coordinates or axis lengths
 *            exceed safe numerical domains (|coord| > 1e75, len_sq > 1e300).
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "geometry.hpp"

namespace planner::collision::internal {

/**
 * @brief Fallback normalized intersection routine for extreme coordinates or large polygons.
 * @param[in] polygon Array of polygon vertices.
 * @param[in] count Number of vertices.
 * @param[in] box Target AABB.
 * @param[in] tol Distance tolerance.
 * @return True if polygon intersects or touches box within tolerance.
 */
bool convex_polygon_intersects_aabb_normalized(const Point2d* polygon,
                                               std::size_t count,
                                               const Aabb2d& box,
                                               double tol) noexcept;

/**
 * @brief Precomputed Separating Axis Theorem (SAT) cache for convex polygon vs. AABB tests.
 * @tparam Capacity Maximum number of polygon vertices supported in fixed-size storage.
 */
template <std::size_t Capacity>
struct PolygonSatCache {
  static constexpr std::size_t kCapacity = Capacity;

  /// Maximum coordinate and axis length limits for the fast unnormalized squared-distance domain.
  static constexpr double kFastMaxAbsCoordinate = 1.0e75;
  static constexpr double kFastMaxAxisLenSq = 1.0e300;

  const Point2d* points{nullptr};  ///< Non-owning pointer to vertex buffer.
  std::size_t count{0};            ///< Number of vertices in the polygon.
  bool usable{false};              ///< False if count > Capacity (caller must fallback).
  bool fast_domain{false};         ///< True if all coordinates and axis lengths are within safe fast bounds.
  double min_x{0.0};
  double max_x{0.0};
  double min_y{0.0};
  double max_y{0.0};
  std::array<Point2d, Capacity> raw_axes{};       ///< Unnormalized edge normal axes for edge (i, i+1).
  std::array<double, Capacity> axis_len_sq{};     ///< Squared length |axis|^2.
  std::array<double, Capacity> poly_lo{};         ///< Minimum polygon projection along raw_axes[i].
  std::array<double, Capacity> poly_hi{};         ///< Maximum polygon projection along raw_axes[i].
  std::array<std::uint8_t, Capacity> axis_ok{};   ///< 1 if normal axis is non-degenerate, 0 otherwise.

  /**
   * @brief Precomputes bounding box, edge normals, squared lengths, and projection intervals.
   * @param[in] polygon Array of polygon vertices.
   * @param[in] n Vertex count.
   */
  void build(const Point2d* polygon, std::size_t n) noexcept {
    points = polygon;
    count = n;
    usable = n <= Capacity;
    fast_domain = false;
    if (!usable || n == 0) {
      return;  // Empty polygons are valid (never intersect); capacity overflow triggers fallback.
    }
    min_x = std::numeric_limits<double>::infinity();
    max_x = -std::numeric_limits<double>::infinity();
    min_y = std::numeric_limits<double>::infinity();
    max_y = -std::numeric_limits<double>::infinity();
    bool coordinates_finite = true;
    for (std::size_t i = 0; i < n; ++i) {
      const Point2d& p = polygon[i];
      if (!std::isfinite(p.x_m) || !std::isfinite(p.y_m) ||
          std::abs(p.x_m) > kFastMaxAbsCoordinate ||
          std::abs(p.y_m) > kFastMaxAbsCoordinate) {
        coordinates_finite = false;
      }
      min_x = std::min(min_x, p.x_m);
      max_x = std::max(max_x, p.x_m);
      min_y = std::min(min_y, p.y_m);
      max_y = std::max(max_y, p.y_m);
    }
    fast_domain = coordinates_finite;
    for (std::size_t i = 0; i < n; ++i) {
      const Point2d& p0 = polygon[i];
      const Point2d& p1 = polygon[(i + 1) % n];
      const Point2d axis{-(p1.y_m - p0.y_m), p1.x_m - p0.x_m};
      const double len_sq = axis.x_m * axis.x_m + axis.y_m * axis.y_m;
      if (!(len_sq > 0.0)) {
        axis_ok[i] = 0;  // Degenerate axis: cannot separate
        raw_axes[i] = Point2d{0.0, 0.0};
        axis_len_sq[i] = 0.0;
        poly_lo[i] = 0.0;
        poly_hi[i] = 0.0;
        continue;
      }
      if (!std::isfinite(len_sq) || len_sq > kFastMaxAxisLenSq) {
        fast_domain = false;
      }
      axis_ok[i] = 1;
      raw_axes[i] = axis;
      axis_len_sq[i] = len_sq;
      double lo = std::numeric_limits<double>::infinity();
      double hi = -std::numeric_limits<double>::infinity();
      for (std::size_t j = 0; j < n; ++j) {
        const double v =
            polygon[j].x_m * axis.x_m + polygon[j].y_m * axis.y_m;
        lo = std::min(lo, v);
        hi = std::max(hi, v);
      }
      poly_lo[i] = lo;
      poly_hi[i] = hi;
    }
  }

  /**
   * @brief Fast closed-set intersection test between polygon and AABB.
   * @param[in] box Target AABB.
   * @param[in] tol Distance tolerance.
   * @return True if polygon overlaps or touches box within tolerance.
   */
  bool maybe_intersects(const Aabb2d& box, double tol) const noexcept {
    if (count == 0) {
      return false;
    }
    if (!usable) {
      return true;  // Conservative: caller falls back to full test.
    }
    if (!fast_domain || !box_in_fast_domain(box) || !(tol >= 0.0) ||
        !std::isfinite(tol)) {
      return convex_polygon_intersects_aabb_normalized(points, count, box, tol);
    }
    // Fast rejection via bounding boxes.
    if (max_x < box.min_x - tol || min_x > box.max_x + tol ||
        max_y < box.min_y - tol || min_y > box.max_y + tol) {
      return false;
    }
    const double tol_sq = tol * tol;
    for (std::size_t i = 0; i < count; ++i) {
      if (axis_ok[i] == 0) {
        continue;
      }
      const Point2d& axis = raw_axes[i];
      const double p00 = box.min_x * axis.x_m + box.min_y * axis.y_m;
      const double p10 = box.max_x * axis.x_m + box.min_y * axis.y_m;
      const double p01 = box.min_x * axis.x_m + box.max_y * axis.y_m;
      const double p11 = box.max_x * axis.x_m + box.max_y * axis.y_m;
      const double box_lo = std::min(std::min(p00, p10), std::min(p01, p11));
      const double box_hi = std::max(std::max(p00, p10), std::max(p01, p11));
      const double gap_above = box_lo - poly_hi[i];
      if (gap_above > 0.0 &&
          gap_above * gap_above > tol_sq * axis_len_sq[i]) {
        return false;
      }
      const double gap_below = poly_lo[i] - box_hi;
      if (gap_below > 0.0 &&
          gap_below * gap_below > tol_sq * axis_len_sq[i]) {
        return false;
      }
    }
    return true;
  }

 private:
  static bool box_in_fast_domain(const Aabb2d& box) noexcept {
    return std::isfinite(box.min_x) && std::isfinite(box.max_x) &&
           std::isfinite(box.min_y) && std::isfinite(box.max_y) &&
           std::abs(box.min_x) <= kFastMaxAbsCoordinate &&
           std::abs(box.max_x) <= kFastMaxAbsCoordinate &&
           std::abs(box.min_y) <= kFastMaxAbsCoordinate &&
           std::abs(box.max_y) <= kFastMaxAbsCoordinate;
  }
};

using PolygonSatCache8 = PolygonSatCache<8>;

}  // namespace planner::collision::internal
