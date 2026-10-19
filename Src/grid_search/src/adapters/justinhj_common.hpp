#pragma once

// justinhj/astar-algorithm-cpp（MIT，commit 875bbd8，tag v1.4.2）栅格 UserState
// 与后端实现模板。
//
// 内存池开/关是编译期宏（stlastar.h 的 USE_FSA_MEMORY），因此两个配置放在
// 两个翻译单元中编译：justinhj_adapter.cpp（池开）与
// justinhj_nopool_adapter.cpp（池关，USE_FSA_MEMORY=0）。为避免 ODR 冲突，
// 两个配置使用不同的 UserState 标签类型（GridState<PoolTag> /
// GridState<NoPoolTag>），模板实例互不相同。
//
// 原生偏差（见 provenance.md）：
//   * 代价与启发式为 float32（上游 g/h/f 为 float）；统一 cost_m 由适配层按
//     返回路径以 double 重算，上游 float32 自报值放 stats.upstream_cost；
//   * 取消与“无路径”在上游同为 SEARCH_STATE_FAILED，由适配层自行区分；
//   * 内存池容量必须按规模设置（MaxNodes），耗尽返回 OUT_OF_MEMORY。

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "stlastar.h"

#include "planner/grid_search/adapters/adapters.hpp"
#include "planner/grid_search/neighborhood.hpp"

#include "adapter_util.hpp"

namespace planner::grid_search {
namespace justinhj_detail {

struct PoolTag {};
struct NoPoolTag {};

template <typename Tag>
class GridState {
 public:
  GridState() = default;
  GridState(const common::GridView* grid, const GridSearchRequest* request,
            int row, int col)
      : grid_(grid), request_(request), row_(row), col_(col) {}

  // 上游接口：启发式（float32）、目标判定、后继、步代价、去重与哈希。
  float GoalDistanceEstimate(const GridState& goal) const {
    return static_cast<float>(detail::neighborhood_heuristic(
        request_->neighborhood, row_ - goal.row_, col_ - goal.col_));
  }

  bool IsGoal(const GridState& goal) const {
    return row_ == goal.row_ && col_ == goal.col_;
  }

  bool GetSuccessors(AStarSearch<GridState<Tag>>* search,
                     GridState<Tag>* /*parent*/) const {
    for (const GridCell& cell :
         planner::grid_search::neighbors(*grid_, row_, col_,
                                         request_->neighborhood)) {
      GridState successor(grid_, request_, cell.row, cell.col);
      if (!search->AddSuccessor(successor)) {
        return false;  // 内存池耗尽（或分配失败）
      }
    }
    return true;
  }

  float GetCost(const GridState& successor) const {
    const int d_row = successor.row_ - row_;
    const int d_col = successor.col_ - col_;
    return (d_row != 0 && d_col != 0) ? 1.4142135623730951f : 1.0f;
  }

  bool IsSameState(const GridState& rhs) const {
    return row_ == rhs.row_ && col_ == rhs.col_;
  }

  std::size_t Hash() const {
    return std::hash<int>()(row_) * 1000003u + std::hash<int>()(col_);
  }

  int row() const { return row_; }
  int col() const { return col_; }

 private:
  const common::GridView* grid_{nullptr};
  const GridSearchRequest* request_{nullptr};
  int row_{0};
  int col_{0};
};

// 与上游语义一致的搜索执行与结果封装。Tag 决定模板实例，UsePool 只影响
// 文档/容量策略（宏本身在翻译单元内定义）。
template <typename Tag>
class JustinhjBackend final : public GridSearchBackend {
 public:
  using State = GridState<Tag>;
  using Search = AStarSearch<State>;

  explicit JustinhjBackend(std::string name) : name_(std::move(name)) {}

  std::string name() const override { return name_; }

  GridSearchResult search(const GridSearchRequest& request) override {
    std::string reason;
    const common::Status status = detail::precheck(request, &reason);
    if (status != common::Status::kSuccess) {
      return detail::make_failure(status, reason);
    }

    detail::BudgetTracker tracker(request.time_budget_s, request.cancelled);
    State start(request.grid, &request, request.start.row, request.start.col);
    State goal(request.grid, &request, request.goal.row, request.goal.col);

    const std::int64_t cells =
        static_cast<std::int64_t>(request.grid->width) *
        static_cast<std::int64_t>(request.grid->height);
    const std::int64_t capacity =
        cells + 64 > 1024 ? cells + 64 : 1024;  // 池容量按规模设置
    Search search(static_cast<int>(capacity));
    search.SetStartAndGoalStates(start, goal);

    unsigned int final_state = Search::SEARCH_STATE_SEARCHING;
    while (true) {
      if (tracker.should_stop()) {
        search.CancelSearch();
        search.SearchStep();  // 消费取消：释放节点并返回 FAILED
        final_state = Search::SEARCH_STATE_FAILED;
        break;
      }
      final_state = search.SearchStep();
      if (final_state == Search::SEARCH_STATE_SUCCEEDED ||
          final_state == Search::SEARCH_STATE_FAILED ||
          final_state == Search::SEARCH_STATE_OUT_OF_MEMORY) {
        break;
      }
    }

    GridSearchStats stats;
    stats.iterations = search.GetStepCount();
    stats.upstream_status = state_name(final_state);
    stats.expanded_nodes = stats.iterations;

    if (final_state == Search::SEARCH_STATE_OUT_OF_MEMORY) {
      return detail::make_failure(
          common::Status::kTimeout,
          "justinhj memory pool exhausted (MaxNodes=" +
              std::to_string(capacity) + ")",
          stats);
    }
    if (final_state != Search::SEARCH_STATE_SUCCEEDED) {
      if (tracker.should_stop()) {
        return detail::make_failure(tracker.stop_status(),
                                    tracker.stop_reason(), stats);
      }
      return detail::make_failure(common::Status::kNoPath,
                                  "no path found by justinhj A*", stats);
    }

    std::vector<GridCell> path;
    for (State* state = search.GetSolutionStart(); state != nullptr;
         state = search.GetSolutionNext()) {
      path.push_back(GridCell{state->row(), state->col()});
    }
    stats.upstream_cost = static_cast<double>(search.GetSolutionCost());
    search.FreeSolutionNodes();
    return detail::make_path_result(request, std::move(path), stats);
  }

 private:
  static std::string state_name(unsigned int state) {
    switch (state) {
      case Search::SEARCH_STATE_SUCCEEDED:
        return "success";
      case Search::SEARCH_STATE_FAILED:
        return "failed";
      case Search::SEARCH_STATE_OUT_OF_MEMORY:
        return "out_of_memory";
      case Search::SEARCH_STATE_SEARCHING:
        return "searching";
      default:
        return "invalid";
    }
  }

  std::string name_;
};

}  // namespace justinhj_detail
}  // namespace planner::grid_search
