/**
 * @file grid_view.cpp
 * @brief 栅格视图单元查询、相等性比较与栅格坐标系变换的具体实现
 */

#include "planner/common/grid_view.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <optional>
#include <stdexcept>

namespace planner::common {

int GridView::cell(int row, int col) const {
  if (!in_bounds(row, col)) {
    throw std::out_of_range("GridView::cell: row/col out of bounds");
  }
  const auto index = static_cast<std::size_t>(row) * static_cast<std::size_t>(width) +
                     static_cast<std::size_t>(col);
  return cells[index];
}

std::optional<int> GridView::try_cell(int row, int col) const noexcept {
  if (!in_bounds(row, col)) {
    return std::nullopt;
  }
  const auto index = static_cast<std::size_t>(row) * static_cast<std::size_t>(width) +
                     static_cast<std::size_t>(col);
  return cells[index];
}

bool operator==(const GridView& lhs, const GridView& rhs) {
  return lhs.map_id == rhs.map_id && lhs.map_version == rhs.map_version && lhs.width == rhs.width &&
         lhs.height == rhs.height && lhs.resolution_m == rhs.resolution_m &&
         lhs.origin == rhs.origin && lhs.cells == rhs.cells;
}

GridFrame GridFrame::from_grid(const GridView& grid) noexcept {
  GridFrame frame;
  frame.cos_yaw = std::cos(grid.origin.yaw_rad);
  frame.sin_yaw = std::sin(grid.origin.yaw_rad);
  frame.origin_x = grid.origin.x_m;
  frame.origin_y = grid.origin.y_m;
  frame.width_m = grid.width * grid.resolution_m;
  frame.height_m = grid.height * grid.resolution_m;
  frame.resolution_m = grid.resolution_m;
  return frame;
}

Point2d GridFrame::to_grid_frame(const Point2d& world_point) const noexcept {
  const double dx = world_point.x_m - origin_x;
  const double dy = world_point.y_m - origin_y;
  return Point2d{cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy};
}

bool GridFrame::inside_map(const Point2d& grid_point) const noexcept {
  return grid_point.x_m >= 0.0 && grid_point.x_m <= width_m && grid_point.y_m >= 0.0 &&
         grid_point.y_m <= height_m;
}

double GridFrame::distance_to_boundary(const Point2d& grid_point) const noexcept {
  const double d = std::min(std::min(grid_point.x_m, width_m - grid_point.x_m),
                            std::min(grid_point.y_m, height_m - grid_point.y_m));
  return d;  // 若在地图外部则为负值
}

}  // namespace planner::common
