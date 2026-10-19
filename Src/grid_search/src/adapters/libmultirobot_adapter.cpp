// libMultiRobotPlanning（MIT，commit 4c75fa20）栅格 Environment 适配。
//
// 复用图版 A* 模板核心（include/libMultiRobotPlanning/a_star.hpp 三头文件）；
// 栅格 Environment 由适配层提供：四/八邻域、八邻域禁对角穿角、unknown 与
// 地图外按阻塞。上游无取消/超时钩子，按探针结论在 onExpandNode 中检查
// 预算/取消并抛异常中止（适配补丁，见 provenance.md）。

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include "libMultiRobotPlanning/a_star.hpp"

#include "planner/grid_search/adapters/adapters.hpp"
#include "planner/grid_search/neighborhood.hpp"

#include "adapter_util.hpp"

namespace planner::grid_search {
namespace {

// onExpandNode 中抛出的协作终止信号（穿过上游模板后被适配层捕获）。
struct SearchStopped {};

class GridEnvironment {
 public:
  using State = int;
  using Action = int;
  using Cost = double;
  using Neighbor = libMultiRobotPlanning::Neighbor<State, Action, Cost>;

  GridEnvironment(const GridSearchRequest& request, detail::BudgetTracker& tracker)
      : request_(request), tracker_(tracker) {}

  double admissibleHeuristic(const State& state) const {
    const int row = state / request_.grid->width;
    const int col = state % request_.grid->width;
    return detail::neighborhood_heuristic(request_.neighborhood,
                                          request_.goal.row - row,
                                          request_.goal.col - col);
  }

  bool isSolution(const State& state) const { return state == goal_index_; }

  void getNeighbors(const State& state,
                    std::vector<Neighbor>& neighbors) const {
    const int row = state / request_.grid->width;
    const int col = state % request_.grid->width;
    const std::vector<GridCell> next =
        planner::grid_search::neighbors(*request_.grid, row, col,
                                        request_.neighborhood);
    for (std::size_t index = 0; index < next.size(); ++index) {
      const GridCell& cell = next[index];
      const int d_row = cell.row - row;
      const int d_col = cell.col - col;
      const double cost = (d_row != 0 && d_col != 0) ? std::sqrt(2.0) : 1.0;
      neighbors.emplace_back(cell.row * request_.grid->width + cell.col,
                             static_cast<int>(index), cost);
    }
  }

  void onExpandNode(const State&, double, double) const {
    ++expanded_;
    if (tracker_.should_stop()) {
      throw SearchStopped{};
    }
  }

  void onDiscover(const State&, double, double) const { ++generated_; }

  void set_goal_index(int index) { goal_index_ = index; }

  std::int64_t expanded() const { return expanded_; }
  std::int64_t generated() const { return generated_; }

 private:
  const GridSearchRequest& request_;
  detail::BudgetTracker& tracker_;
  int goal_index_{0};
  mutable std::int64_t expanded_{0};
  mutable std::int64_t generated_{0};
};

class LibMultiRobotAstarBackend : public GridSearchBackend {
 public:
  std::string name() const override { return "libmultirobot_astar"; }

  GridSearchResult search(const GridSearchRequest& request) override {
    std::string reason;
    const common::Status status = detail::precheck(request, &reason);
    if (status != common::Status::kSuccess) {
      return detail::make_failure(status, reason);
    }

    detail::BudgetTracker tracker(request.time_budget_s, request.cancelled);
    GridEnvironment environment(request, tracker);
    environment.set_goal_index(request.goal.row * request.grid->width +
                               request.goal.col);

    libMultiRobotPlanning::AStar<int, int, double, GridEnvironment> astar(
        environment);
    libMultiRobotPlanning::PlanResult<int, int, double> solution;
    bool found = false;
    try {
      found = astar.search(request.start.row * request.grid->width +
                               request.start.col,
                           solution);
    } catch (const SearchStopped&) {
      GridSearchStats stats;
      stats.expanded_nodes = environment.expanded();
      stats.generated_nodes = environment.generated();
      stats.upstream_status = "stopped_by_adapter";
      return detail::make_failure(tracker.stop_status(), tracker.stop_reason(),
                                  stats);
    }

    GridSearchStats stats;
    stats.expanded_nodes = environment.expanded();
    stats.generated_nodes = environment.generated();
    stats.iterations = environment.expanded();
    stats.upstream_status = found ? "success" : "no_path";

    if (!found) {
      return detail::make_failure(common::Status::kNoPath,
                                  "no path found by libMultiRobotPlanning A*",
                                  stats);
    }

    std::vector<GridCell> path;
    path.reserve(solution.states.size());
    for (const auto& entry : solution.states) {
      path.push_back(GridCell{entry.first / request.grid->width,
                              entry.first % request.grid->width});
    }
    stats.upstream_cost = solution.cost;  // 单元步长（double）
    return detail::make_path_result(request, std::move(path), stats);
  }
};

}  // namespace

std::unique_ptr<GridSearchBackend> make_libmultirobot_astar_backend() {
  return std::make_unique<LibMultiRobotAstarBackend>();
}

}  // namespace planner::grid_search
