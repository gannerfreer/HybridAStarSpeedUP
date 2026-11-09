/**
 * @file grid_potential.hpp
 * @brief Internal 8-connected Dijkstra potential field computation on 2D occupancy grids.
 *
 * Computes shortest-path Euclidean-weighted distances on free cells (cells == kCellFree)
 * originating from the goal cell. Occupied, unknown, and out-of-map cells are treated as blocked.
 * Used as an obstacle-aware heuristic in combination with Reeds-Shepp analytic expansion,
 * and as a connectivity filter for early reachability verification.
 */

#pragma once

#include <cstdint>
#include <functional>
#include <limits>
#include <vector>

#include "types.hpp"

namespace planner::hybrid_astar::internal {

/// Represents an infinite potential value indicating unreachable cells.
inline constexpr float kPotentialInfinity = std::numeric_limits<float>::infinity();

/**
 * @brief 2D potential field map storing shortest-path distances to the goal.
 */
struct GridPotential {
  int width{0};                   ///< Grid width (columns).
  int height{0};                  ///< Grid height (rows).
  double resolution_m{0.0};       ///< Cell edge resolution in meters.
  std::vector<float> distance_m;  ///< Distance map in row-major order (m); infinity if unreachable.
  bool aborted{false};            ///< True if potential computation was aborted due to timeout.
};

/**
 * @brief Generates a 64-bit hash key covering grid contents, geometry, and goal coordinates.
 * @param[in] grid Grid view referencing occupancy cells and dimensions.
 * @param[in] goal_row Target cell row index.
 * @param[in] goal_col Target cell column index.
 * @return 64-bit cache key.
 */
std::uint64_t grid_cache_key(const common::GridView& grid, int goal_row, int goal_col);

/**
 * @brief Computes Dijkstra potential field from the target cell over free cells.
 * @param[in] grid Occupancy grid view.
 * @param[in] goal_row Target cell row index.
 * @param[in] goal_col Target cell column index.
 * @param[in] deadline_exceeded Optional callback returning true if the computation deadline is exceeded.
 * @return Computed potential field; if aborted, aborted flag is set to true.
 */
GridPotential compute_grid_potential(const common::GridView& grid, int goal_row,
                                     int goal_col,
                                     const std::function<bool()>& deadline_exceeded);

}  // namespace planner::hybrid_astar::internal
