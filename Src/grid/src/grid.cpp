// planner_grid 实现（见 grid.hpp）。

#include "planner/grid/grid.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>

namespace planner::grid {
namespace {

bool finite(double value) noexcept { return std::isfinite(value); }

// int 的可表示区间（double 形式），避免 floor/ceil 结果转换时的未定义行为。
constexpr double kIntMin = -2147483648.0;
constexpr double kIntMax = 2147483647.0;

// ceil(span/res) 的 double 结果；参数假定已校验为正且有限。
double span_cell_count_double(double span_m, double resolution_m) noexcept {
  return std::ceil(span_m / resolution_m);
}

// ceil(width/res) * ceil(height/res) 的 double 结果。
double window_cell_count_double(double width_m, double height_m,
                                double resolution_m) noexcept {
  return span_cell_count_double(width_m, resolution_m) *
         span_cell_count_double(height_m, resolution_m);
}

bool valid_resolution(double resolution_m) noexcept {
  return finite(resolution_m) && resolution_m > 0.0;
}

}  // namespace

bool operator==(const CellIndex& lhs, const CellIndex& rhs) noexcept {
  return lhs.row == rhs.row && lhs.col == rhs.col;
}

bool operator!=(const CellIndex& lhs, const CellIndex& rhs) noexcept {
  return !(lhs == rhs);
}

std::optional<CellIndex> world_to_cell(const common::GridView& grid,
                                       double x_m, double y_m) noexcept {
  if (!finite(x_m) || !finite(y_m)) {
    return std::nullopt;
  }
  if (grid.width <= 0 || grid.height <= 0) {
    return std::nullopt;
  }
  if (!valid_resolution(grid.resolution_m)) {
    return std::nullopt;
  }
  if (!finite(grid.origin.x_m) || !finite(grid.origin.y_m) ||
      !finite(grid.origin.yaw_rad)) {
    return std::nullopt;
  }
  // 世界 -> 原点坐标系（逆旋转 origin.yaw_rad）。
  const double dx = x_m - grid.origin.x_m;
  const double dy = y_m - grid.origin.y_m;
  const double cos_yaw = std::cos(grid.origin.yaw_rad);
  const double sin_yaw = std::sin(grid.origin.yaw_rad);
  const double local_x = cos_yaw * dx + sin_yaw * dy;
  const double local_y = -sin_yaw * dx + cos_yaw * dy;
  const double col = std::floor(local_x / grid.resolution_m);
  const double row = std::floor(local_y / grid.resolution_m);
  if (!finite(col) || !finite(row) || col < kIntMin || col > kIntMax ||
      row < kIntMin || row > kIntMax) {
    return std::nullopt;
  }
  return CellIndex{static_cast<int>(row), static_cast<int>(col)};
}

CellIndex world_to_cell_clamped(const common::GridView& grid, double x_m,
                                double y_m) noexcept {
  CellIndex cell;
  if (grid.width <= 0 || grid.height <= 0) {
    return cell;
  }
  const std::optional<CellIndex> raw = world_to_cell(grid, x_m, y_m);
  if (!raw) {
    return cell;
  }
  cell.col = std::max(0, std::min(raw->col, grid.width - 1));
  cell.row = std::max(0, std::min(raw->row, grid.height - 1));
  return cell;
}

common::Point2d cell_center(const common::GridView& grid, int row,
                            int col) noexcept {
  const double local_x = (static_cast<double>(col) + 0.5) * grid.resolution_m;
  const double local_y = (static_cast<double>(row) + 0.5) * grid.resolution_m;
  const double cos_yaw = std::cos(grid.origin.yaw_rad);
  const double sin_yaw = std::sin(grid.origin.yaw_rad);
  return common::Point2d{grid.origin.x_m + cos_yaw * local_x - sin_yaw * local_y,
                         grid.origin.y_m + sin_yaw * local_x + cos_yaw * local_y};
}

int ceil_cell_count(double span_m, double resolution_m) noexcept {
  if (!finite(span_m) || !(span_m > 0.0) || !valid_resolution(resolution_m)) {
    return 0;
  }
  const double count = span_cell_count_double(span_m, resolution_m);
  if (!(count >= 1.0) || count > kIntMax) {
    return 0;
  }
  return static_cast<int>(count);
}

std::optional<WindowGridSpec> window_grid_spec(double width_m, double height_m,
                                               double resolution_m) noexcept {
  const int width = ceil_cell_count(width_m, resolution_m);
  const int height = ceil_cell_count(height_m, resolution_m);
  if (width <= 0 || height <= 0) {
    return std::nullopt;
  }
  WindowGridSpec spec;
  spec.width = width;
  spec.height = height;
  spec.cells = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  return spec;
}

CellBudgetFit fit_resolution_to_cell_budget(double width_m, double height_m,
                                            double resolution_m,
                                            std::size_t max_cells) noexcept {
  CellBudgetFit fit;
  fit.resolution_m = resolution_m;
  if (!finite(width_m) || !(width_m > 0.0) || !finite(height_m) ||
      !(height_m > 0.0) || !valid_resolution(resolution_m)) {
    return fit;
  }
  const double budget =
      static_cast<double>(std::max<std::size_t>(max_cells, std::size_t{1}));
  int guard = 0;
  while (window_cell_count_double(width_m, height_m, fit.resolution_m) > budget &&
         guard < 32) {
    fit.resolution_m *= 2.0;
    ++guard;
  }
  const double cells = window_cell_count_double(width_m, height_m, fit.resolution_m);
  fit.within_budget = finite(cells) && cells <= budget;
  // 预算内且可安全转换时给出整数单元数；极端参数下保持 0（within_budget
  // 仍然有效，调用方以最终分辨率与栅格导出结果为准）。
  if (fit.within_budget && cells <= 9007199254740992.0) {  // 2^53
    fit.cells = static_cast<std::size_t>(cells);
  }
  return fit;
}

bool cell_state_valid(int value) noexcept {
  return value == common::kCellFree || value == common::kCellOccupied ||
         value == common::kCellUnknown;
}

bool valid_grid(const common::GridView& grid, std::string* reason) {
  const bool no_grid = grid.width == 0 && grid.height == 0;
  if (!no_grid) {
    if (grid.width <= 0 || grid.height <= 0) {
      if (reason) {
        *reason = "grid needs both width and height positive (or both zero)";
      }
      return false;
    }
    if (!valid_resolution(grid.resolution_m)) {
      if (reason) {
        *reason = "grid resolution must be finite and positive";
      }
      return false;
    }
    if (!finite(grid.origin.x_m) || !finite(grid.origin.y_m) ||
        !finite(grid.origin.yaw_rad)) {
      if (reason) {
        *reason = "grid origin is not finite";
      }
      return false;
    }
    const std::size_t expected =
        static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height);
    if (grid.cells.size() != expected) {
      if (reason) {
        *reason = "grid cell count does not match width * height";
      }
      return false;
    }
    for (std::size_t i = 0; i < grid.cells.size(); ++i) {
      if (!cell_state_valid(grid.cells[i])) {
        if (reason) {
          *reason = "grid cell " + std::to_string(i) +
                    " has a value outside {0,1,2}";
        }
        return false;
      }
    }
  }
  return true;
}

}  // namespace planner::grid
