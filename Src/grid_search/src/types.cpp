// 统一契约实现：请求校验、路径校验、代价重算与容差。

#include "planner/grid_search/types.hpp"

#include <cmath>
#include <string>

#include "planner/grid_search/neighborhood.hpp"

namespace planner::grid_search {
namespace {

bool finite(double value) { return std::isfinite(value); }

// 栅格自身校验：尺寸、分辨率、单元长度与三态取值。
bool valid_grid(const common::GridView& grid, std::string* reason) {
  if (grid.width <= 0 || grid.height <= 0) {
    if (reason != nullptr) {
      *reason = "grid has non-positive size (" + std::to_string(grid.width) +
                "x" + std::to_string(grid.height) + ")";
    }
    return false;
  }
  if (!finite(grid.resolution_m) || grid.resolution_m <= 0.0) {
    if (reason != nullptr) {
      *reason = "grid resolution must be finite and positive";
    }
    return false;
  }
  const std::size_t expected =
      static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height);
  if (grid.cells.size() != expected) {
    if (reason != nullptr) {
      *reason = "grid cells size " + std::to_string(grid.cells.size()) +
                " does not match width*height " + std::to_string(expected);
    }
    return false;
  }
  for (std::size_t index = 0; index < grid.cells.size(); ++index) {
    const int value = grid.cells[index];
    if (value != common::kCellFree && value != common::kCellOccupied &&
        value != common::kCellUnknown) {
      if (reason != nullptr) {
        *reason = "grid cell " + std::to_string(index) +
                  " has invalid state " + std::to_string(value);
      }
      return false;
    }
  }
  return true;
}

bool valid_budget(const std::optional<double>& time_budget_s,
                  const std::optional<std::int64_t>& node_budget,
                  std::string* reason) {
  if (time_budget_s.has_value()) {
    const double value = *time_budget_s;
    if (!finite(value) || value <= 0.0) {
      if (reason != nullptr) {
        *reason = "time_budget_s must be finite and positive";
      }
      return false;
    }
  }
  if (node_budget.has_value() && *node_budget <= 0) {
    if (reason != nullptr) {
      *reason = "node_budget must be positive";
    }
    return false;
  }
  return true;
}

common::Status validate_cell(const common::GridView& grid, const GridCell& cell,
                             const char* what, std::string* reason) {
  if (!grid.in_bounds(cell.row, cell.col)) {
    if (reason != nullptr) {
      *reason = std::string(what) + " cell (" + std::to_string(cell.row) + "," +
                std::to_string(cell.col) + ") is outside the grid";
    }
    return common::Status::kInvalidInput;
  }
  if (cell_blocked(grid, cell.row, cell.col)) {
    if (reason != nullptr) {
      *reason = std::string(what) + " cell (" + std::to_string(cell.row) + "," +
                std::to_string(cell.col) +
                ") is blocked (occupied, unknown or outside)";
    }
    return common::Status::kInvalidInput;
  }
  return common::Status::kSuccess;
}

}  // namespace

bool operator==(const GridCell& lhs, const GridCell& rhs) noexcept {
  return lhs.row == rhs.row && lhs.col == rhs.col;
}

bool operator!=(const GridCell& lhs, const GridCell& rhs) noexcept {
  return !(lhs == rhs);
}

std::string to_string(Neighborhood neighborhood) {
  return neighborhood == Neighborhood::kFour ? "four" : "eight";
}

std::optional<Neighborhood> neighborhood_from_string(const std::string& text) {
  if (text == "four") {
    return Neighborhood::kFour;
  }
  if (text == "eight") {
    return Neighborhood::kEight;
  }
  return std::nullopt;
}

std::string status_name(common::Status status) {
  return common::to_string(status);
}

common::Status validate_request(const GridSearchRequest& request,
                                std::string* reason) {
  if (request.grid == nullptr) {
    if (reason != nullptr) {
      *reason = "grid must not be null";
    }
    return common::Status::kInvalidInput;
  }
  if (!valid_grid(*request.grid, reason)) {
    return common::Status::kInvalidInput;
  }
  if (!valid_budget(request.time_budget_s, request.node_budget, reason)) {
    return common::Status::kInvalidInput;
  }
  common::Status status =
      validate_cell(*request.grid, request.start, "start", reason);
  if (status != common::Status::kSuccess) {
    return status;
  }
  return validate_cell(*request.grid, request.goal, "goal", reason);
}

std::string validate_path(const common::GridView& grid,
                          const GridSearchRequest& request,
                          const std::vector<GridCell>& path,
                          bool enforce_corner_rule) {
  if (path.empty()) {
    return "path is empty";
  }
  if (path.front() != request.start) {
    return "path does not start at the requested start cell";
  }
  if (path.back() != request.goal) {
    return "path does not end at the requested goal cell";
  }
  for (std::size_t index = 0; index < path.size(); ++index) {
    if (!grid.in_bounds(path[index].row, path[index].col)) {
      return "path cell " + std::to_string(index) + " is outside the grid";
    }
    if (cell_blocked(grid, path[index].row, path[index].col)) {
      return "path cell " + std::to_string(index) + " is blocked";
    }
    if (index == 0) {
      continue;
    }
    const int d_row = path[index].row - path[index - 1].row;
    const int d_col = path[index].col - path[index - 1].col;
    bool matched = false;
    for (const NeighborStep& step : neighborhood_steps(request.neighborhood)) {
      if (step.d_row != d_row || step.d_col != d_col) {
        continue;
      }
      matched = true;
      if (enforce_corner_rule &&
          !diagonal_corner_allowed(grid, path[index - 1].row,
                                   path[index - 1].col, step)) {
        return "path step " + std::to_string(index) +
               " cuts a blocked diagonal corner";
      }
      break;
    }
    if (!matched) {
      return "path step " + std::to_string(index) +
             " does not match the requested neighborhood";
    }
  }
  return std::string();
}

double path_cost_m(const common::GridView& grid, Neighborhood neighborhood,
                   const std::vector<GridCell>& path) {
  double cost_cells = 0.0;
  for (std::size_t index = 1; index < path.size(); ++index) {
    const int d_row = path[index].row - path[index - 1].row;
    const int d_col = path[index].col - path[index - 1].col;
    double step_cost = std::sqrt(static_cast<double>(d_row * d_row + d_col * d_col));
    for (const NeighborStep& step : neighborhood_steps(neighborhood)) {
      if (step.d_row == d_row && step.d_col == d_col) {
        step_cost = step.cost;
        break;
      }
    }
    cost_cells += step_cost;
  }
  return cost_cells * grid.resolution_m;
}

bool costs_match(double actual, double reference) noexcept {
  return std::fabs(actual - reference) <= 1e-8 + 1e-8 * std::fabs(reference);
}

}  // namespace planner::grid_search
