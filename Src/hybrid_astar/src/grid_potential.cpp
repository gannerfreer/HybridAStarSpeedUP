// 内部目标势场计算（见 grid_potential.hpp）。

#include "grid_potential.hpp"

#include <cmath>
#include <cstring>
#include <queue>
#include <utility>

#include "planner/common/types.hpp"

namespace planner::hybrid_astar::internal {

namespace {

void hash_byte_block(std::uint64_t* hash, const void* data, std::size_t size) {
  const unsigned char* bytes = static_cast<const unsigned char*>(data);
  for (std::size_t i = 0; i < size; ++i) {
    *hash ^= static_cast<std::uint64_t>(bytes[i]);
    *hash *= 1099511628211ull;
  }
}

void hash_u64(std::uint64_t* hash, std::uint64_t value) {
  hash_byte_block(hash, &value, sizeof(value));
}

void hash_double(std::uint64_t* hash, double value) {
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value), "double must be 64-bit");
  std::memcpy(&bits, &value, sizeof(bits));
  hash_u64(hash, bits);
}

}  // namespace

std::uint64_t grid_cache_key(const common::GridView& grid, int goal_row,
                             int goal_col) {
  std::uint64_t hash = 14695981039346656037ull;  // FNV-1a 偏移基准
  hash_byte_block(&hash, grid.map_id.data(), grid.map_id.size());
  hash_byte_block(&hash, grid.map_version.data(), grid.map_version.size());
  hash_u64(&hash, static_cast<std::uint64_t>(grid.width));
  hash_u64(&hash, static_cast<std::uint64_t>(grid.height));
  hash_double(&hash, grid.resolution_m);
  hash_double(&hash, grid.origin.x_m);
  hash_double(&hash, grid.origin.y_m);
  hash_double(&hash, grid.origin.yaw_rad);
  hash_u64(&hash, static_cast<std::uint64_t>(grid.cells.size()));
  for (const int cell : grid.cells) {
    hash_u64(&hash, static_cast<std::uint64_t>(cell + 1));
  }
  hash_u64(&hash, static_cast<std::uint64_t>(goal_row));
  hash_u64(&hash, static_cast<std::uint64_t>(goal_col));
  return hash;
}

GridPotential compute_grid_potential(
    const common::GridView& grid, int goal_row, int goal_col,
    const std::function<bool()>& deadline_exceeded) {
  GridPotential potential;
  potential.width = grid.width;
  potential.height = grid.height;
  potential.resolution_m = grid.resolution_m;

  const int cell_count = grid.width * grid.height;
  if (cell_count <= 0 || !grid.in_bounds(goal_row, goal_col) ||
      grid.cells.size() != static_cast<std::size_t>(cell_count)) {
    return potential;
  }
  const int goal_index = goal_row * grid.width + goal_col;
  if (grid.cells[static_cast<std::size_t>(goal_index)] != common::kCellFree) {
    return potential;
  }

  potential.distance_m.assign(static_cast<std::size_t>(cell_count),
                              kPotentialInfinity);

  const float step_m = static_cast<float>(grid.resolution_m);
  const float diagonal_step_m = step_m * static_cast<float>(std::sqrt(2.0));

  struct QueueItem {
    float distance;
    int index;
  };
  struct GreaterDistance {
    bool operator()(const QueueItem& a, const QueueItem& b) const {
      return a.distance > b.distance;
    }
  };
  std::priority_queue<QueueItem, std::vector<QueueItem>, GreaterDistance> queue;

  potential.distance_m[static_cast<std::size_t>(goal_index)] = 0.0f;
  queue.push(QueueItem{0.0f, goal_index});

  static const int kRowDelta[8] = {-1, -1, -1, 0, 0, 1, 1, 1};
  static const int kColDelta[8] = {-1, 0, 1, -1, 1, -1, 0, 1};

  std::uint64_t processed = 0;
  while (!queue.empty()) {
    if (deadline_exceeded != nullptr && ((++processed & 8191u) == 0u) &&
        deadline_exceeded()) {
      potential.aborted = true;
      return potential;
    }
    const QueueItem top = queue.top();
    queue.pop();
    if (top.distance >
        potential.distance_m[static_cast<std::size_t>(top.index)]) {
      continue;  // 过期堆条目
    }
    const int row = top.index / grid.width;
    const int col = top.index % grid.width;
    for (int k = 0; k < 8; ++k) {
      const int next_row = row + kRowDelta[k];
      const int next_col = col + kColDelta[k];
      if (!grid.in_bounds(next_row, next_col)) {
        continue;
      }
      const int next_index = next_row * grid.width + next_col;
      if (grid.cells[static_cast<std::size_t>(next_index)] != common::kCellFree) {
        continue;
      }
      const float edge =
          (kRowDelta[k] != 0 && kColDelta[k] != 0) ? diagonal_step_m : step_m;
      const float next_distance = top.distance + edge;
      if (next_distance <
          potential.distance_m[static_cast<std::size_t>(next_index)]) {
        potential.distance_m[static_cast<std::size_t>(next_index)] =
            next_distance;
        queue.push(QueueItem{next_distance, next_index});
      }
    }
  }
  return potential;
}

}  // namespace planner::hybrid_astar::internal
