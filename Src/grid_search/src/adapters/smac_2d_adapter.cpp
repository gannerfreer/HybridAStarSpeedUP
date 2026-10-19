// Nav2 SmacPlanner2D（Apache-2.0，自 navigation2 @ 3286119753ce1fab9d2ff411ea6d439b823fe18d
// 抽取）栅格 A* 适配器。
//
// 解耦方式（家族 C 探针已实证，见 provenance.md）：
//   * 抽取 AStarAlgorithm<Node2D> 与 Node2D 搜索核心 + 最小桩集（costmap/
//     异常/lifecycle/碰撞检查索引路径/空 Hybrid/Lattice）；
//   * 适配层固定 allow_unknown=false（unknown 按阻塞）与 tolerance=0
//     （精确最短路，关闭最接近路径回退）；
//   * 适配补丁 P2（getNeighbors 禁对角穿角）与 P4（四邻域选择）已应用于
//     抽取源码；
//   * 代价感知模式（cost_penalty>0 / cell_costs）单列，不进入统一排名。

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "nav2_core/planner_exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_smac_planner/a_star.hpp"
#include "nav2_smac_planner/constants.hpp"
#include "nav2_smac_planner/node_2d.hpp"
#include "nav2_smac_planner/types.hpp"

#include "planner/grid_search/adapters/adapters.hpp"

#include "adapter_util.hpp"

namespace planner::grid_search {
namespace {

constexpr int kCostAwareMaxCellCost = 252;  // 上游 MAX_NON_OBSTACLE_COST

class Smac2dBackend final : public GridSearchBackend {
 public:
  explicit Smac2dBackend(Smac2dOptions options) : options_(std::move(options)) {}

  std::string name() const override { return "smac_2d_astar"; }

  GridSearchResult search(const GridSearchRequest& request) override {
    std::string reason;
    const common::Status status = detail::precheck(request, &reason);
    if (status != common::Status::kSuccess) {
      return detail::make_failure(status, reason);
    }
    if (!options_.cell_costs.empty() &&
        options_.cell_costs.size() != request.grid->cells.size()) {
      return detail::make_failure(
          common::Status::kInvalidInput,
          "smac cost layer size does not match width*height");
    }
    for (double cost : options_.cell_costs) {
      if (!std::isfinite(cost) || cost < 0.0 ||
          cost > static_cast<double>(kCostAwareMaxCellCost)) {
        return detail::make_failure(
            common::Status::kInvalidInput,
            "smac cost layer values must be finite and within [0, 252]");
      }
    }

    // 原生偏差登记：上游 createPath 在 start == goal 时因起始节点没有父节点
    // 而 backtracePath 失败（返回 false）。适配层短路为单单元成功结果。
    if (request.start == request.goal) {
      GridSearchStats stats;
      stats.upstream_status = "start_equals_goal_adapter_shortcut";
      return detail::make_path_result(request, {request.start}, stats);
    }

    // GridView 三态 -> 桩 Costmap2D：free -> 0（或代价层）、occupied -> 254、
    // unknown -> 255；allow_unknown=false 时 255 按阻塞（契约）。
    nav2_costmap_2d::Costmap2D costmap(
        static_cast<unsigned int>(request.grid->width),
        static_cast<unsigned int>(request.grid->height),
        request.grid->resolution_m, 0.0, 0.0, 0);
    for (int row = 0; row < request.grid->height; ++row) {
      for (int col = 0; col < request.grid->width; ++col) {
        const std::size_t index = static_cast<std::size_t>(row) *
                                      static_cast<std::size_t>(request.grid->width) +
                                  static_cast<std::size_t>(col);
        const int value = request.grid->cells[index];
        unsigned char cost = 0;
        if (value == common::kCellOccupied) {
          cost = 254;
        } else if (value == common::kCellUnknown) {
          cost = 255;
        } else if (!options_.cell_costs.empty()) {
          cost = static_cast<unsigned char>(
              std::lround(options_.cell_costs[index]));
        }
        costmap.setCost(static_cast<unsigned int>(col),
                        static_cast<unsigned int>(row), cost);
      }
    }

    nav2_smac_planner::GridCollisionChecker collision_checker(&costmap);
    nav2_smac_planner::SearchInfo search_info;
    search_info.cost_penalty = static_cast<float>(options_.cost_penalty);
    search_info.use_four_neighborhood =
        request.neighborhood == Neighborhood::kFour;  // 补丁 P4

    nav2_smac_planner::AStarAlgorithm<nav2_smac_planner::Node2D> astar(
        nav2_smac_planner::MotionModel::TWOD, search_info);

    int max_iterations =
        request.node_budget.has_value()
            ? static_cast<int>(*request.node_budget)
            : std::numeric_limits<int>::max();
    const double max_planning_time =
        request.time_budget_s.has_value()
            ? *request.time_budget_s
            : std::numeric_limits<double>::max();
    astar.initialize(false /*allow_unknown*/, max_iterations,
                     0 /*max_on_approach_iterations*/,
                     1 /*terminal_checking_interval*/, max_planning_time,
                     1.0f /*lookup_table_size*/, 1u /*dim_3_size*/);
    astar.setCollisionChecker(&collision_checker);
    astar.setStart(static_cast<float>(request.start.col),
                   static_cast<float>(request.start.row), 0);
    astar.setGoal(static_cast<float>(request.goal.col),
                  static_cast<float>(request.goal.row), 0);

    detail::BudgetTracker tracker(request.time_budget_s, request.cancelled);
    nav2_smac_planner::Node2D::CoordinateVector raw_path;
    int iterations = 0;
    bool found = false;
    try {
      found = astar.createPath(
          raw_path, iterations, 0.0f /*tolerance=0：精确最短路*/,
          [&tracker]() { return tracker.should_stop(); });
    } catch (const nav2_core::PlannerCancelled&) {
      GridSearchStats stats;
      stats.expanded_nodes = iterations;
      stats.iterations = iterations;
      stats.upstream_status = "cancelled";
      return detail::make_failure(tracker.stop_status(),
                                  tracker.stop_reason(), stats);
    } catch (const nav2_core::GoalOccupied&) {
      return detail::make_failure(common::Status::kInvalidInput,
                                  "smac rejected the goal cell");
    }

    GridSearchStats stats;
    stats.expanded_nodes = iterations;
    stats.iterations = iterations;
    stats.upstream_status = found ? "success" : "no_path";
    stats.upstream_cost = std::numeric_limits<double>::quiet_NaN();

    if (!found) {
      if (tracker.should_stop()) {
        return detail::make_failure(tracker.stop_status(),
                                    tracker.stop_reason(), stats);
      }
      if (request.node_budget.has_value() && iterations >= max_iterations) {
        stats.upstream_status = "node_budget_exhausted";
        return detail::make_failure(common::Status::kTimeout,
                                    "node budget exhausted before finding a "
                                    "path (smac)", stats);
      }
      return detail::make_failure(common::Status::kNoPath,
                                  "no path found by SmacPlanner2D A*", stats);
    }

    // 上游返回 goal -> start 的单元链（float 单元索引），反转为统一顺序。
    std::vector<GridCell> path;
    path.reserve(raw_path.size());
    for (auto iterator = raw_path.rbegin(); iterator != raw_path.rend();
         ++iterator) {
      path.push_back(GridCell{
          static_cast<int>(std::lround(iterator->y)),
          static_cast<int>(std::lround(iterator->x))});
    }
    return detail::make_path_result(request, std::move(path), stats);
  }

 private:
  Smac2dOptions options_;
};

}  // namespace

std::unique_ptr<GridSearchBackend> make_smac_2d_backend(
    const Smac2dOptions& options) {
  return std::make_unique<Smac2dBackend>(options);
}

}  // namespace planner::grid_search
