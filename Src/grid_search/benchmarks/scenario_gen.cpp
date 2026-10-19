// grid_scenario_gen v1 生成规则实现（确定性；规则见 benchmarks/README.md）。

#include "scenario_gen.hpp"

#include <algorithm>
#include <deque>
#include <stdexcept>
#include <utility>

#include "sha256.hpp"

namespace planner::grid_search::bench {
namespace {

constexpr int kDirections[4][2] = {{0, 1}, {1, 0}, {0, -1}, {-1, 0}};

// 与 tests/reference/grid_reference.py 相同的自带 LCG（跨版本稳定）。
class Lcg {
 public:
  explicit Lcg(std::uint64_t seed)
      : state_(static_cast<std::uint32_t>(
            (seed * 2654435761ULL + 1013904223ULL) & 0x7FFFFFFFULL)) {}

  std::uint32_t next() {
    state_ = static_cast<std::uint32_t>(
        (1103515245ULL * state_ + 12345ULL) & 0x7FFFFFFFULL);
    return state_;
  }

  std::vector<int> permutation(std::vector<int> items) {
    for (int index = static_cast<int>(items.size()) - 1; index > 0; --index) {
      const int swap =
          static_cast<int>(next() % static_cast<std::uint32_t>(index + 1));
      std::swap(items[static_cast<std::size_t>(index)],
                items[static_cast<std::size_t>(swap)]);
    }
    return items;
  }

 private:
  std::uint32_t state_;
};

std::uint8_t& cell_at(ScenarioInstance& instance, int row, int col) {
  return instance.cells[static_cast<std::size_t>(row) *
                            static_cast<std::size_t>(instance.width) +
                        static_cast<std::size_t>(col)];
}

std::uint8_t cell_at(const ScenarioInstance& instance, int row, int col) {
  return instance.cells[static_cast<std::size_t>(row) *
                            static_cast<std::size_t>(instance.width) +
                        static_cast<std::size_t>(col)];
}

void fill_rect(ScenarioInstance& instance, int row0, int col0, int row1,
               int col1, std::uint8_t value) {
  for (int row = row0; row <= row1; ++row) {
    for (int col = col0; col <= col1; ++col) {
      cell_at(instance, row, col) = value;
    }
  }
}

void carve_interior(ScenarioInstance& instance) {
  fill_rect(instance, 1, 1, instance.height - 2, instance.width - 2, 0);
}

// ---------------------------------------------------------------------------
// 逐类生成（冻结解释见 benchmarks/README.md“生成器规则 v1”）
// ---------------------------------------------------------------------------

void make_open(ScenarioInstance& instance) {
  const int scale = instance.scale;
  carve_interior(instance);
  const int margin = 2 + (instance.seed % 3);
  instance.start = GridCell{margin, margin};
  instance.goal = GridCell{scale - 1 - margin, scale - 1 - margin};
  instance.distance_source = instance.start;
  instance.expected = "success";
}

void make_maze(ScenarioInstance& instance) {
  const int scale = instance.scale;
  const int pitch = std::max(8, scale / 16);   // 房间间距 max(8, 规模/16)
  const int rooms = scale / pitch;             // 16（128²–4096² 一致）
  const int wall = 1 + (instance.seed % 2);    // 墙厚 1–2 格
  const auto interior_lo = [wall, pitch](int index) {
    return index * pitch + wall;
  };
  const auto interior_hi = [wall, pitch, scale](int index) {
    return std::min(index * pitch + pitch - 1, scale - 1 - wall);
  };

  // 全占据起步；开出房间内部（通道宽 = 房间内部尺寸 ≥ 4 ≥ 3）。
  for (int ri = 0; ri < rooms; ++ri) {
    for (int rj = 0; rj < rooms; ++rj) {
      fill_rect(instance, interior_lo(ri), interior_lo(rj), interior_hi(ri),
                interior_hi(rj), 0);
    }
  }

  // 递归回溯（栈式 DFS；房间访问顺序由房间种子 LCG 决定）。
  std::vector<std::vector<bool>> visited(
      static_cast<std::size_t>(rooms),
      std::vector<bool>(static_cast<std::size_t>(rooms), false));
  std::vector<std::pair<int, int>> stack;
  stack.emplace_back(0, 0);
  visited[0][0] = true;
  while (!stack.empty()) {
    const auto [ri, rj] = stack.back();
    Lcg lcg(static_cast<std::uint64_t>(instance.seed) * 1000003ULL +
            static_cast<std::uint64_t>(ri) * 131ULL +
            static_cast<std::uint64_t>(rj) * 17ULL);
    const std::vector<int> order = lcg.permutation({0, 1, 2, 3});
    bool moved = false;
    for (int direction : order) {
      const int nr = ri + kDirections[direction][0];
      const int nc = rj + kDirections[direction][1];
      if (nr < 0 || nr >= rooms || nc < 0 || nc >= rooms || visited[nr][nc]) {
        continue;
      }
      if (kDirections[direction][0] == 0) {
        // 水平相邻：打通垂直墙（列区间 [wall_lo, wall_lo+wall-1]）。
        const int wall_lo = kDirections[direction][1] == 1
                                ? rj * pitch + pitch
                                : nc * pitch + pitch;
        const int lo = interior_lo(ri);
        const int hi = interior_hi(ri);
        const int center = (lo + hi) / 2;
        fill_rect(instance, center - 1, wall_lo, center + 1,
                  wall_lo + wall - 1, 0);
      } else {
        // 垂直相邻：打通水平墙（行区间 [wall_lo, wall_lo+wall-1]）。
        const int wall_lo = kDirections[direction][0] == 1
                                ? ri * pitch + pitch
                                : nr * pitch + pitch;
        const int lo = interior_lo(rj);
        const int hi = interior_hi(rj);
        const int center = (lo + hi) / 2;
        fill_rect(instance, wall_lo, center - 1, wall_lo + wall - 1,
                  center + 1, 0);
      }
      visited[nr][nc] = true;
      stack.emplace_back(nr, nc);
      moved = true;
      break;
    }
    if (!moved) {
      stack.pop_back();
    }
  }

  instance.start = GridCell{(interior_lo(0) + interior_hi(0)) / 2,
                            (interior_lo(0) + interior_hi(0)) / 2};
  instance.goal = GridCell{(interior_lo(rooms - 1) + interior_hi(rooms - 1)) / 2,
                           (interior_lo(rooms - 1) + interior_hi(rooms - 1)) / 2};
  instance.distance_source = instance.start;
  instance.expected = "success";
}

void make_corridor(ScenarioInstance& instance) {
  const int scale = instance.scale;
  carve_interior(instance);
  const int center_row =
      scale / 2 + (instance.seed - 2) * std::max(1, scale / 64);
  const int span_lo = scale / 8;
  const int span_hi = scale - scale / 8 - 1;  // 贯穿约 75% 边长
  fill_rect(instance, center_row - 2, span_lo, center_row - 2, span_hi, 1);
  fill_rect(instance, center_row + 2, span_lo, center_row + 2, span_hi, 1);
  instance.start = GridCell{center_row, 2};
  instance.goal = GridCell{center_row, scale - 3};
  instance.distance_source = instance.start;
  instance.expected = "success";
}

void make_deadend(ScenarioInstance& instance) {
  const int scale = instance.scale;
  carve_interior(instance);
  const int center_row =
      scale / 2 + (instance.seed - 2) * std::max(1, scale / 64);
  const int depth = std::max(10, scale / 16) + instance.seed;  // 袋深 ≥10
  const int x0 = scale / 8;
  const int x1 = x0 + depth - 1;
  fill_rect(instance, center_row - 2, x0, center_row - 2, x1 + 1, 1);
  fill_rect(instance, center_row + 2, x0, center_row + 2, x1 + 1, 1);
  fill_rect(instance, center_row - 2, x1 + 1, center_row + 2, x1 + 1, 1);
  instance.start = GridCell{center_row, 2};
  instance.goal = GridCell{center_row, x1};
  instance.distance_source = instance.start;
  instance.expected = "success";
}

void make_unreachable(ScenarioInstance& instance) {
  const int scale = instance.scale;
  carve_interior(instance);
  const int center_row =
      scale / 2 + (instance.seed - 2) * std::max(1, scale / 32);
  const int center_col = scale / 2;
  const int half = std::max(4, scale / 64);
  // 封闭腔：墙厚 1、无开口（内部保持空闲）。
  fill_rect(instance, center_row - half, center_col - half, center_row + half,
            center_col - half, 1);
  fill_rect(instance, center_row - half, center_col + half, center_row + half,
            center_col + half, 1);
  fill_rect(instance, center_row - half, center_col - half, center_row - half,
            center_col + half, 1);
  fill_rect(instance, center_row + half, center_col - half, center_row + half,
            center_col + half, 1);
  instance.start = GridCell{2, 2};
  instance.goal = GridCell{center_row, center_col};
  instance.distance_source = instance.start;
  instance.expected = "no_path";
}

void make_border_corner(ScenarioInstance& instance) {
  const int scale = instance.scale;
  carve_interior(instance);
  const int pairs = std::max(3, scale / 64) + instance.seed;  // ≥3 处对角对
  for (int index = 0; index < pairs; ++index) {
    const int base = 1 + 3 * index;
    cell_at(instance, base, base + 1) = 1;
    cell_at(instance, base + 1, base) = 1;
  }
  // 贴边通道：墙沿右边界内侧，底部留出入口。
  const int wall_col = scale - 5;
  fill_rect(instance, scale / 8, wall_col, scale - 2, wall_col, 1);
  instance.start = GridCell{2, 2};
  instance.goal = GridCell{scale - 3, scale - 3};
  instance.distance_source = instance.start;
  instance.expected = "success";
}

// ---------------------------------------------------------------------------
// 独立 BFS（仅生成器自检使用；不调用被测搜索库）
// ---------------------------------------------------------------------------
bool bfs_reachable(const ScenarioInstance& instance, bool eight,
                   const GridCell& from, const GridCell& to) {
  const int width = instance.width;
  const int height = instance.height;
  std::vector<std::uint8_t> visited(static_cast<std::size_t>(width) *
                                        static_cast<std::size_t>(height),
                                    0);
  std::deque<GridCell> queue;
  queue.push_back(from);
  visited[static_cast<std::size_t>(from.row) * static_cast<std::size_t>(width) +
          static_cast<std::size_t>(from.col)] = 1;
  while (!queue.empty()) {
    const GridCell current = queue.front();
    queue.pop_front();
    if (current.row == to.row && current.col == to.col) {
      return true;
    }
    const int step_count = eight ? 8 : 4;
    static const int diagonal[4][2] = {{1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    for (int step = 0; step < step_count; ++step) {
      const int d_row = step < 4 ? kDirections[step][0] : diagonal[step - 4][0];
      const int d_col = step < 4 ? kDirections[step][1] : diagonal[step - 4][1];
      const int nr = current.row + d_row;
      const int nc = current.col + d_col;
      if (nr < 0 || nr >= height || nc < 0 || nc >= width) {
        continue;
      }
      if (cell_at(instance, nr, nc) != 0) {
        continue;
      }
      if (d_row != 0 && d_col != 0) {
        if (cell_at(instance, current.row, nc) != 0 ||
            cell_at(instance, nr, current.col) != 0) {
          continue;  // 禁对角穿角（统一契约）
        }
      }
      const std::size_t index = static_cast<std::size_t>(nr) *
                                    static_cast<std::size_t>(width) +
                                static_cast<std::size_t>(nc);
      if (visited[index] != 0) {
        continue;
      }
      visited[index] = 1;
      queue.push_back(GridCell{nr, nc});
    }
  }
  return false;
}

void add_problem(InstanceValidation& validation, const std::string& problem) {
  validation.ok = false;
  if (validation.problems.size() < 32) {
    validation.problems.push_back(problem);
  }
}

}  // namespace

const std::vector<std::string>& all_scenario_classes() {
  static const std::vector<std::string> classes = {
      "open",   "maze",         "corridor",
      "deadend", "unreachable", "border_corner"};
  return classes;
}

bool is_valid_scenario_class(const std::string& name) {
  const auto& classes = all_scenario_classes();
  return std::find(classes.begin(), classes.end(), name) != classes.end();
}

bool is_valid_scale(int scale) {
  return scale == 128 || scale == 512 || scale == 1024 || scale == 2048 ||
         scale == 4096;
}

std::string instance_id_for(const std::string& scenario_class, int scale,
                            int seed) {
  return "grid-" + scenario_class + "-" + std::to_string(scale) + "-s" +
         std::to_string(seed);
}

ScenarioInstance generate_instance(const std::string& scenario_class, int scale,
                                   int seed) {
  if (!is_valid_scenario_class(scenario_class)) {
    throw std::invalid_argument("unknown scenario class: " + scenario_class);
  }
  if (!is_valid_scale(scale)) {
    throw std::invalid_argument("unsupported scale: " + std::to_string(scale));
  }
  if (seed < 0 || seed > 4) {
    throw std::invalid_argument("seed out of range [0,4]: " +
                                std::to_string(seed));
  }
  ScenarioInstance instance;
  instance.instance_id = instance_id_for(scenario_class, scale, seed);
  instance.scenario_class = scenario_class;
  instance.scale = scale;
  instance.seed = seed;
  instance.width = scale;
  instance.height = scale;
  instance.resolution_m = kBenchResolutionM;
  instance.cells.assign(static_cast<std::size_t>(scale) *
                            static_cast<std::size_t>(scale),
                        1);
  if (scenario_class == "open") {
    make_open(instance);
  } else if (scenario_class == "maze") {
    make_maze(instance);
  } else if (scenario_class == "corridor") {
    make_corridor(instance);
  } else if (scenario_class == "deadend") {
    make_deadend(instance);
  } else if (scenario_class == "unreachable") {
    make_unreachable(instance);
  } else {
    make_border_corner(instance);
  }
  return instance;
}

InstanceValidation validate_instance(const ScenarioInstance& instance) {
  InstanceValidation validation;
  const int scale = instance.scale;
  if (instance.width != scale || instance.height != scale) {
    add_problem(validation, "size mismatch");
  }
  if (instance.resolution_m != kBenchResolutionM) {
    add_problem(validation, "resolution mismatch");
  }
  if (instance.cells.size() != static_cast<std::size_t>(scale) *
                                     static_cast<std::size_t>(scale)) {
    add_problem(validation, "cells size mismatch");
    return validation;
  }
  const auto free_cell = [&](const GridCell& cell) {
    return cell.row >= 0 && cell.row < instance.height && cell.col >= 0 &&
           cell.col < instance.width &&
           cell_at(instance, cell.row, cell.col) == 0;
  };
  if (!free_cell(instance.start)) {
    add_problem(validation, "start not free");
  }
  if (!free_cell(instance.goal)) {
    add_problem(validation, "goal not free");
  }
  if (!free_cell(instance.distance_source)) {
    add_problem(validation, "distance source not free");
  }
  for (int col = 0; col < instance.width; ++col) {
    if (cell_at(instance, 0, col) == 0 ||
        cell_at(instance, instance.height - 1, col) == 0) {
      add_problem(validation, "top/bottom border not closed");
      break;
    }
  }
  for (int row = 0; row < instance.height; ++row) {
    if (cell_at(instance, row, 0) == 0 ||
        cell_at(instance, row, instance.width - 1) == 0) {
      add_problem(validation, "left/right border not closed");
      break;
    }
  }
  const bool expected_success = instance.expected == "success";
  if (expected_success == bfs_reachable(instance, false, instance.start,
                                        instance.goal)) {
    // ok
  } else {
    add_problem(validation, "four-neighborhood reachability != expected");
  }
  if (expected_success ==
      bfs_reachable(instance, true, instance.start, instance.goal)) {
    // ok
  } else {
    add_problem(validation, "eight-neighborhood reachability != expected");
  }

  // 逐类不变量。
  if (instance.scenario_class == "maze") {
    const int pitch = std::max(8, scale / 16);
    if (scale % pitch != 0) {
      add_problem(validation, "maze scale not divisible by pitch");
    }
    const int wall = 1 + (instance.seed % 2);
    if (wall < 1 || wall > 2) {
      add_problem(validation, "maze wall thickness outside 1-2");
    }
  } else if (instance.scenario_class == "corridor") {
    const int center_row =
        scale / 2 + (instance.seed - 2) * std::max(1, scale / 64);
    const int span_lo = scale / 8;
    const int span_hi = scale - scale / 8 - 1;
    for (int row = center_row - 1; row <= center_row + 1; ++row) {
      if (cell_at(instance, row, (span_lo + span_hi) / 2) != 0) {
        add_problem(validation, "corridor interior not free (width < 3)");
        break;
      }
    }
    if (cell_at(instance, center_row - 2, (span_lo + span_hi) / 2) == 0 ||
        cell_at(instance, center_row + 2, (span_lo + span_hi) / 2) == 0) {
      add_problem(validation, "corridor walls missing");
    }
  } else if (instance.scenario_class == "deadend") {
    const int depth = std::max(10, scale / 16) + instance.seed;
    if (depth < 10) {
      add_problem(validation, "deadend depth < 10");
    }
  } else if (instance.scenario_class == "border_corner") {
    int diagonal_pairs = 0;
    for (int row = 1; row < scale - 1; ++row) {
      for (int col = 1; col < scale - 1; ++col) {
        if (cell_at(instance, row, col) != 0 &&
            cell_at(instance, row + 1, col + 1) != 0) {
          ++diagonal_pairs;
        }
        if (cell_at(instance, row, col + 1) != 0 &&
            cell_at(instance, row + 1, col) != 0) {
          ++diagonal_pairs;
        }
      }
    }
    if (diagonal_pairs < 3) {
      add_problem(validation, "fewer than 3 diagonal-adjacent obstacle pairs");
    }
  } else if (instance.scenario_class == "unreachable") {
    if (bfs_reachable(instance, true, instance.start, instance.goal)) {
      add_problem(validation, "unreachable goal is reachable");
    }
  }
  return validation;
}

std::string instance_cells_sha256(const ScenarioInstance& instance) {
  return sha256_hex(instance.cells);
}

}  // namespace planner::grid_search::bench
