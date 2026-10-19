// vss2sn/path_planning（BSD-3-Clause，commit d8f9eda）栅格 Dijkstra / A* 适配器。
//
// 适配补丁（见 provenance.md）：
//   * P1：GetMotion(bool) 增加八邻域运动原语（上游仅四邻域，对角项被注释）；
//   * P2：Planner::SetMotion / SetCancelChecker（上游无取消/超时钩子）；
//   * P3：Plan 循环使用可配置运动、禁对角穿角、边界检查前移；Dijkstra 增加
//         PlanDistanceField（上游无距离场 API）；
//   * P4：A* 八邻域 octile 启发式（四邻域保持 Manhattan）。
//
// 原生偏差（适配层补偿）：
//   * 上游 Planner 假定 n x n 方形栅格：非方形输入按 max(width,height) 填充
//     为阻塞（转换成本单列）；
//   * 上游目标判定先于边界/障碍检查：适配层预校验起终点，拒绝阻塞/越界单元；
//   * 上游返回路径为 goal -> start，适配层反转为 start -> goal；
//   * 上游不暴露展开节点计数（stats.expanded_nodes 保持 0）。

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "path_planning/a_star.hpp"
#include "path_planning/dijkstra.hpp"
#include "utils/utils.hpp"

#include "planner/grid_search/adapters/adapters.hpp"
#include "planner/grid_search/distance_field.hpp"

#include "adapter_util.hpp"

namespace planner::grid_search {
namespace {

// GridView -> 上游内部栅格 [x][y]（0=空闲、1=阻塞）；unknown 与地图外按阻塞。
// 非方形输入填充为 n x n（n = max(width, height)），填充单元 = 阻塞。
std::vector<std::vector<int>> to_upstream_grid(const common::GridView& grid,
                                               int* n_out) {
  const int n = std::max(grid.width, grid.height);
  std::vector<std::vector<int>> cells(
      static_cast<std::size_t>(n), std::vector<int>(static_cast<std::size_t>(n), 1));
  for (int row = 0; row < grid.height; ++row) {
    for (int col = 0; col < grid.width; ++col) {
      cells[static_cast<std::size_t>(col)][static_cast<std::size_t>(row)] =
          grid.cell(row, col) == common::kCellFree ? 0 : 1;
    }
  }
  *n_out = n;
  return cells;
}

Node make_upstream_node(int col, int row, int n) {
  Node node(col, row, 0, 0, 0, 0);
  node.id_ = col * n + row;
  node.pid_ = node.id_;
  return node;
}

std::vector<Node> upstream_motion(Neighborhood neighborhood) {
  return GetMotion(neighborhood == Neighborhood::kEight);
}

class Vss2snDijkstraBackend final : public GridSearchBackend {
 public:
  std::string name() const override { return "vss2sn_dijkstra"; }
  bool supports_distance_field() const override { return true; }

  GridSearchResult search(const GridSearchRequest& request) override {
    std::string reason;
    const common::Status status = detail::precheck(request, &reason);
    if (status != common::Status::kSuccess) {
      return detail::make_failure(status, reason);
    }

    int n = 0;
    Dijkstra planner(to_upstream_grid(*request.grid, &n));
    planner.SetMotion(upstream_motion(request.neighborhood));
    detail::BudgetTracker tracker(request.time_budget_s, request.cancelled);
    planner.SetCancelChecker([&tracker]() { return tracker.should_stop(); });

    const Node start = make_upstream_node(request.start.col, request.start.row, n);
    const Node goal = make_upstream_node(request.goal.col, request.goal.row, n);
    const auto [found, raw_path] = planner.Plan(start, goal);

    GridSearchStats stats;
    stats.upstream_status = found ? "success" : "no_path";
    stats.upstream_cost = std::numeric_limits<double>::quiet_NaN();
    if (!found) {
      if (tracker.should_stop()) {
        return detail::make_failure(tracker.stop_status(),
                                    tracker.stop_reason(), stats);
      }
      return detail::make_failure(common::Status::kNoPath,
                                  "no path found by vss2sn Dijkstra", stats);
    }

    // 上游返回 goal -> start；反转为统一 start -> goal。
    std::vector<GridCell> path;
    path.reserve(raw_path.size());
    for (auto iterator = raw_path.rbegin(); iterator != raw_path.rend();
         ++iterator) {
      path.push_back(GridCell{iterator->y_, iterator->x_});
    }
    return detail::make_path_result(request, std::move(path), stats);
  }

  DistanceFieldResult distance_field(
      const DistanceFieldRequest& request) override {
    DistanceFieldResult result;
    std::string reason;
    const common::Status status =
        validate_distance_field_request(request, &reason);
    if (status != common::Status::kSuccess) {
      result.status = status;
      result.reason = reason;
      return result;
    }
    if (request.cancelled) {
      result.status = common::Status::kCancelled;
      result.reason = "cancelled by caller";
      return result;
    }

    int n = 0;
    Dijkstra planner(to_upstream_grid(*request.grid, &n));
    planner.SetMotion(upstream_motion(request.neighborhood));
    detail::BudgetTracker tracker(request.time_budget_s, request.cancelled);
    planner.SetCancelChecker([&tracker]() { return tracker.should_stop(); });

    const Node source =
        make_upstream_node(request.source.col, request.source.row, n);
    const std::vector<std::vector<double>> upstream =
        planner.PlanDistanceField(source);

    result.stats.upstream_status = "success";
    result.stats.upstream_cost = std::numeric_limits<double>::quiet_NaN();
    if (tracker.should_stop()) {
      result.status = tracker.stop_status();
      result.reason = tracker.stop_reason();
      return result;
    }

    // 上游为 [x][y] 单元步长；转换为行主序米制，不可达保持 +infinity。
    result.field.width = request.grid->width;
    result.field.height = request.grid->height;
    result.field.distances.assign(
        static_cast<std::size_t>(request.grid->width) *
            static_cast<std::size_t>(request.grid->height),
        kUnreachableDistance);
    for (int row = 0; row < request.grid->height; ++row) {
      for (int col = 0; col < request.grid->width; ++col) {
        const double distance =
            upstream[static_cast<std::size_t>(col)][static_cast<std::size_t>(row)];
        if (std::isfinite(distance)) {
          result.field.distances[static_cast<std::size_t>(row) *
                                     static_cast<std::size_t>(request.grid->width) +
                                 static_cast<std::size_t>(col)] =
              distance * request.grid->resolution_m;
        }
      }
    }
    result.status = common::Status::kSuccess;
    return result;
  }
};

class Vss2snAstarBackend final : public GridSearchBackend {
 public:
  std::string name() const override { return "vss2sn_astar"; }

  GridSearchResult search(const GridSearchRequest& request) override {
    std::string reason;
    const common::Status status = detail::precheck(request, &reason);
    if (status != common::Status::kSuccess) {
      return detail::make_failure(status, reason);
    }

    int n = 0;
    AStar planner(to_upstream_grid(*request.grid, &n));
    planner.SetMotion(upstream_motion(request.neighborhood));
    detail::BudgetTracker tracker(request.time_budget_s, request.cancelled);
    planner.SetCancelChecker([&tracker]() { return tracker.should_stop(); });

    const Node start = make_upstream_node(request.start.col, request.start.row, n);
    const Node goal = make_upstream_node(request.goal.col, request.goal.row, n);
    const auto [found, raw_path] = planner.Plan(start, goal);

    GridSearchStats stats;
    stats.upstream_status = found ? "success" : "no_path";
    stats.upstream_cost = std::numeric_limits<double>::quiet_NaN();
    if (!found) {
      if (tracker.should_stop()) {
        return detail::make_failure(tracker.stop_status(),
                                    tracker.stop_reason(), stats);
      }
      return detail::make_failure(common::Status::kNoPath,
                                  "no path found by vss2sn A*", stats);
    }

    std::vector<GridCell> path;
    path.reserve(raw_path.size());
    for (auto iterator = raw_path.rbegin(); iterator != raw_path.rend();
         ++iterator) {
      path.push_back(GridCell{iterator->y_, iterator->x_});
    }
    return detail::make_path_result(request, std::move(path), stats);
  }
};

}  // namespace

std::unique_ptr<GridSearchBackend> make_vss2sn_dijkstra_backend() {
  return std::make_unique<Vss2snDijkstraBackend>();
}

std::unique_ptr<GridSearchBackend> make_vss2sn_astar_backend() {
  return std::make_unique<Vss2snAstarBackend>();
}

}  // namespace planner::grid_search
