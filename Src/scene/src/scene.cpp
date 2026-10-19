// 本地规划场景构建（见 scene.hpp）。
//
// 实现自 planner::service 的 W5 场景构建器迁移（X-A1-18）；除障碍物输入类型
// 改为 collision::Obstacle 外，窗口推导、可行驶域基础、预算降级与错误码保持
// 逐字一致。X-M1 增加：有效窗口/分辨率按协议 §5.1 量化（1e-9）、统一
// scene_id（content_id.hpp）与显式窗口预览入口 build_scene_preview；两个
// 入口共用同一组合尾部（可行驶域基础 + 障碍叠加 + 校验 + 标识）。

#include "planner/scene/scene.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "planner/collision/scene.hpp"
#include "planner/grid/grid.hpp"
#include "planner/map/map.hpp"
#include "planner/scene/content_id.hpp"

namespace planner::scene {
namespace {

// 凸多边形点包含测试（含边界），与多边形绕向无关。
bool point_in_convex_polygon(const std::vector<common::Point2d>& polygon,
                             const common::Point2d& point) {
  const std::size_t count = polygon.size();
  if (count < 3) {
    return false;
  }
  double signed_area = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const common::Point2d& a = polygon[i];
    const common::Point2d& b = polygon[(i + 1) % count];
    signed_area += a.x_m * b.y_m - b.x_m * a.y_m;
  }
  const bool ccw = signed_area >= 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const common::Point2d& a = polygon[i];
    const common::Point2d& b = polygon[(i + 1) % count];
    const double cross =
        (b.x_m - a.x_m) * (point.y_m - a.y_m) - (b.y_m - a.y_m) * (point.x_m - a.x_m);
    if (ccw ? cross < 0.0 : cross > 0.0) {
      return false;
    }
  }
  return true;
}

// 文档化的测试夹具推导方式：单元格中心位于闭合障碍物多边形内时记为占据
// （接触也算）。精确凸多边形保留在碰撞场景中；此处只是使网格势场更精细。
// 顶点数不足 3 的障碍物不参与栅格化，随后由 collision::Scene::valid 报告
// invalid_scene（与迁移前"先栅格化、再整体校验"的可观察行为一致）。
void rasterize_obstacles(common::GridView* grid,
                         const std::vector<collision::Obstacle>& obstacles) {
  const collision::GridFrame frame = collision::GridFrame::from_grid(*grid);
  for (const collision::Obstacle& obstacle : obstacles) {
    if (obstacle.vertices.size() < 3) {
      continue;
    }
    std::vector<common::Point2d> polygon;
    polygon.reserve(obstacle.vertices.size());
    for (const common::Point2d& corner : obstacle.vertices) {
      polygon.push_back(frame.to_grid_frame(corner));
    }
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const common::Point2d& point : polygon) {
      min_x = std::min(min_x, point.x_m);
      max_x = std::max(max_x, point.x_m);
      min_y = std::min(min_y, point.y_m);
      max_y = std::max(max_y, point.y_m);
    }
    const int col_begin =
        std::max(0, static_cast<int>(std::floor(min_x / grid->resolution_m)) - 1);
    const int col_end = std::min(grid->width - 1,
                                 static_cast<int>(std::ceil(max_x / grid->resolution_m)));
    const int row_begin =
        std::max(0, static_cast<int>(std::floor(min_y / grid->resolution_m)) - 1);
    const int row_end = std::min(grid->height - 1,
                                 static_cast<int>(std::ceil(max_y / grid->resolution_m)));
    for (int row = row_begin; row <= row_end; ++row) {
      for (int col = col_begin; col <= col_end; ++col) {
        const common::Point2d center{(col + 0.5) * grid->resolution_m,
                                     (row + 0.5) * grid->resolution_m};
        if (point_in_convex_polygon(polygon, center)) {
          grid->cells[static_cast<std::size_t>(row) *
                          static_cast<std::size_t>(grid->width) +
                      static_cast<std::size_t>(col)] = common::kCellOccupied;
        }
      }
    }
  }
}

// 协议 §5.1：窗口坐标与障碍坐标按 1e-9 量化后作为唯一有效输入。
map::Bounds2d quantize_window(const map::Bounds2d& window) {
  map::Bounds2d out;
  out.min_x_m = quantize_scene_value(window.min_x_m);
  out.min_y_m = quantize_scene_value(window.min_y_m);
  out.max_x_m = quantize_scene_value(window.max_x_m);
  out.max_y_m = quantize_scene_value(window.max_y_m);
  return out;
}

void quantize_obstacles(std::vector<collision::Obstacle>* obstacles) {
  for (collision::Obstacle& obstacle : *obstacles) {
    for (common::Point2d& vertex : obstacle.vertices) {
      vertex.x_m = quantize_scene_value(vertex.x_m);
      vertex.y_m = quantize_scene_value(vertex.y_m);
    }
  }
}

// 在有效（量化后）窗口上导出可行驶域基础并做单元预算降级。失败时填充
// reason/detail；reason 沿用稳定码。
bool fit_and_export(const map::Map& map, const map::Bounds2d& window,
                    double requested_resolution_m, std::size_t max_cells,
                    common::GridView* grid, std::string* reason,
                    std::string* detail) {
  double resolution = requested_resolution_m;
  if (!(resolution > 0.0)) {
    resolution = 0.1;
  }
  resolution = quantize_scene_value(resolution);
  const grid::CellBudgetFit fit = grid::fit_resolution_to_cell_budget(
      window.max_x_m - window.min_x_m, window.max_y_m - window.min_y_m, resolution,
      max_cells);
  if (!fit.within_budget) {
    *reason = "window_too_large";
    *detail = "the local planning window exceeds the configured grid budget";
    return false;
  }
  // 场景域基础：lanelet 可行驶域（窗口内与车道多边形相交的单元为自由，
  // 其余为占据）。窗口/分辨率已经过校验与单元预算调整，这里的导出失败
  // 只可能是集成不一致。
  const map::DriveableGridResult driveable =
      map.export_driveable_grid(window, fit.resolution_m);
  if (!driveable.ok()) {
    *reason = driveable.reason == "grid_too_large" ? "window_too_large" : "invalid_scene";
    *detail = "driveable grid export failed: " + driveable.reason;
    return false;
  }
  *grid = driveable.grid;
  return true;
}

// 组合尾部（两入口共用）：量化障碍、叠加占据、校验碰撞场景并计算 scene_id。
// 失败时填充 result->reason/detail 并返回 false。
bool finish_composition(const map::Map& map, const map::Bounds2d& window,
                        double requested_resolution_m,
                        std::vector<collision::Obstacle> obstacles,
                        common::GridView grid, SceneBuildResult* result) {
  quantize_obstacles(&obstacles);
  rasterize_obstacles(&grid, obstacles);

  collision::Scene scene;
  scene.grid = grid;
  scene.obstacles = std::move(obstacles);
  std::string reason;
  if (!scene.valid(&reason)) {
    result->reason = "invalid_scene";
    result->detail = reason;
    return false;
  }

  result->ok = true;
  result->grid = std::move(grid);
  result->scene = std::move(scene);
  result->window = window;
  result->requested_resolution_m = requested_resolution_m;
  result->effective_resolution_m = result->grid.resolution_m;
  result->scene_version = kSceneVersion;
  // scene_id 统一由场景库计算（协议 §5；地图服务预览与规划服务共用）。
  result->scene_id = scene_content_id(
      map.info().map_id, map.info().map_version, window, result->grid.resolution_m,
      result->grid.width, result->grid.height, result->grid.origin,
      result->scene.obstacles);
  return true;
}

}  // namespace

SceneBuildResult build_scene(const map::Map& map,
                             const common::VehicleModel& vehicle,
                             const common::Pose2d& start,
                             const common::Pose2d& goal,
                             const std::vector<collision::Obstacle>& obstacles,
                             const SceneBuildOptions& options) {
  SceneBuildResult result;
  const map::MapInfo& info = map.info();
  const map::Bounds2d& map_bounds = info.bounds_m;

  if (!map_bounds.contains(start.x_m, start.y_m)) {
    result.reason = "start_outside_map";
    result.detail = "start pose is outside the imported map bounds";
    return result;
  }
  if (!map_bounds.contains(goal.x_m, goal.y_m)) {
    result.reason = "goal_outside_map";
    result.detail = "goal pose is outside the imported map bounds";
    return result;
  }

  double min_x = std::min(start.x_m, goal.x_m);
  double max_x = std::max(start.x_m, goal.x_m);
  double min_y = std::min(start.y_m, goal.y_m);
  double max_y = std::max(start.y_m, goal.y_m);
  for (const collision::Obstacle& obstacle : obstacles) {
    const collision::Aabb2d box = obstacle.aabb();
    min_x = std::min(min_x, box.min_x);
    max_x = std::max(max_x, box.max_x);
    min_y = std::min(min_y, box.min_y);
    max_y = std::max(max_y, box.max_y);
  }

  // 起点/终点处整个保守车辆足迹必须位于网格内：按足迹外接圆半径加上
  // 请求的边距进行扩展。
  const double margin = vehicle.safety_margin_m;
  const double x_min_body = -(vehicle.rear_overhang_m + margin);
  const double x_max_body = vehicle.wheelbase_m + vehicle.front_overhang_m + margin;
  const double y_half = 0.5 * vehicle.width_m + margin;
  const double circumradius =
      std::hypot(std::max(std::abs(x_min_body), std::abs(x_max_body)), y_half);
  const double expand = options.margin_m + circumradius;
  min_x -= expand;
  max_x += expand;
  min_y -= expand;
  max_y += expand;

  min_x = std::max(min_x, map_bounds.min_x_m);
  max_x = std::min(max_x, map_bounds.max_x_m);
  min_y = std::max(min_y, map_bounds.min_y_m);
  max_y = std::min(max_y, map_bounds.max_y_m);
  if (!(min_x < max_x) || !(min_y < max_y)) {
    result.reason = "window_outside_map";
    result.detail = "the local planning window does not intersect the map bounds";
    return result;
  }

  // 协议 §5.1：量化后的窗口/分辨率是唯一有效输入，快照与 scene_id 均以
  // 量化值为准。
  const map::Bounds2d window = quantize_window(map::Bounds2d{min_x, min_y, max_x, max_y});
  if (!(window.min_x_m < window.max_x_m) || !(window.min_y_m < window.max_y_m)) {
    result.reason = "window_outside_map";
    result.detail = "the local planning window does not intersect the map bounds";
    return result;
  }

  double requested_resolution = options.resolution_m;
  if (!(requested_resolution > 0.0)) {
    requested_resolution = 0.1;
  }
  requested_resolution = quantize_scene_value(requested_resolution);

  common::GridView grid;
  std::string reason;
  std::string detail;
  if (!fit_and_export(map, window, requested_resolution, options.max_cells, &grid,
                      &reason, &detail)) {
    result.reason = reason;
    result.detail = detail;
    return result;
  }

  // 起点/终点必须落在可行驶域单元上。该检查发生在障碍物栅格化之前，
  // 因此"障碍物压住起点"仍按既有的 Hybrid 阶段 invalid_input 上报。
  const auto driveable_cell = [&](const common::Pose2d& pose, int* row, int* col) {
    const grid::CellIndex cell =
        grid::world_to_cell_clamped(grid, pose.x_m, pose.y_m);
    *row = cell.row;
    *col = cell.col;
  };
  int start_row = 0;
  int start_col = 0;
  driveable_cell(start, &start_row, &start_col);
  if (grid.cell(start_row, start_col) != common::kCellFree) {
    result.reason = "start_outside_driveable";
    result.detail = "start pose is outside the lanelet driveable area";
    return result;
  }
  int goal_row = 0;
  int goal_col = 0;
  driveable_cell(goal, &goal_row, &goal_col);
  if (grid.cell(goal_row, goal_col) != common::kCellFree) {
    result.reason = "goal_outside_driveable";
    result.detail = "goal pose is outside the lanelet driveable area";
    return result;
  }

  if (!finish_composition(map, window, requested_resolution, obstacles, std::move(grid),
                          &result)) {
    return result;
  }
  return result;
}

SceneBuildResult build_scene_preview(const map::Map& map,
                                     const std::vector<collision::Obstacle>& obstacles,
                                     const ScenePreviewOptions& options) {
  SceneBuildResult result;
  const map::Bounds2d& requested = options.window;
  if (!std::isfinite(requested.min_x_m) || !std::isfinite(requested.min_y_m) ||
      !std::isfinite(requested.max_x_m) || !std::isfinite(requested.max_y_m)) {
    result.reason = "invalid_window";
    result.detail = "preview window coordinates must be finite";
    return result;
  }
  if (!(requested.min_x_m < requested.max_x_m) ||
      !(requested.min_y_m < requested.max_y_m)) {
    result.reason = "empty_window";
    result.detail = "preview window is empty: min_x_m < max_x_m and min_y_m < max_y_m required";
    return result;
  }

  // 协议 §3.3 语义 1：窗口裁剪到地图边界；不相交失败 window_outside_map。
  const map::Bounds2d& map_bounds = map.info().bounds_m;
  map::Bounds2d clipped{
      std::max(requested.min_x_m, map_bounds.min_x_m),
      std::max(requested.min_y_m, map_bounds.min_y_m),
      std::min(requested.max_x_m, map_bounds.max_x_m),
      std::min(requested.max_y_m, map_bounds.max_y_m)};
  if (!(clipped.min_x_m < clipped.max_x_m) || !(clipped.min_y_m < clipped.max_y_m)) {
    result.reason = "window_outside_map";
    result.detail = "the preview window does not intersect the map bounds";
    return result;
  }
  const map::Bounds2d window = quantize_window(clipped);
  if (!(window.min_x_m < window.max_x_m) || !(window.min_y_m < window.max_y_m)) {
    result.reason = "window_outside_map";
    result.detail = "the preview window does not intersect the map bounds";
    return result;
  }

  double requested_resolution = options.resolution_m;
  if (!(requested_resolution > 0.0)) {
    requested_resolution = 0.1;
  }
  requested_resolution = quantize_scene_value(requested_resolution);

  common::GridView grid;
  std::string reason;
  std::string detail;
  if (!fit_and_export(map, window, requested_resolution, options.max_cells, &grid,
                      &reason, &detail)) {
    result.reason = reason;
    result.detail = detail;
    return result;
  }

  if (!finish_composition(map, window, requested_resolution, obstacles, std::move(grid),
                          &result)) {
    return result;
  }
  return result;
}

}  // namespace planner::scene
