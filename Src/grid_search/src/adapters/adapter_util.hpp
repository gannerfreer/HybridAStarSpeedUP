#pragma once

// 内部适配工具（仅组件实现使用，不进入公共头文件）：
//   * BudgetTracker：统一时间预算/取消标志跟踪；
//   * 请求预检与统一结果封装（路径校验、代价重算、状态区分）。

#include <chrono>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "planner/grid_search/backend.hpp"
#include "planner/grid_search/neighborhood.hpp"
#include "planner/grid_search/types.hpp"

namespace planner::grid_search::detail {

// 时间预算与协作取消。后端在搜索循环中轮询 should_stop()；终止后由适配层
// 结合 cancelled()/expired() 区分 kCancelled 与 kTimeout（上游通常无法区分）。
class BudgetTracker {
 public:
  using Clock = std::chrono::steady_clock;

  BudgetTracker(const std::optional<double>& time_budget_s, bool cancelled)
      : time_budget_s_(time_budget_s),
        cancelled_(cancelled),
        start_(Clock::now()) {}

  bool cancelled() const noexcept { return cancelled_; }

  bool expired() const noexcept {
    if (!time_budget_s_.has_value()) {
      return false;
    }
    return elapsed_s() >= *time_budget_s_;
  }

  bool should_stop() const noexcept { return cancelled_ || expired(); }

  double elapsed_s() const noexcept {
    return std::chrono::duration<double>(Clock::now() - start_).count();
  }

  // 预算/取消终止的最终状态：取消优先于超时。
  common::Status stop_status() const noexcept {
    return cancelled_ ? common::Status::kCancelled : common::Status::kTimeout;
  }

  std::string stop_reason() const {
    return cancelled_ ? "cancelled by caller" : "time budget exhausted";
  }

 private:
  std::optional<double> time_budget_s_;
  bool cancelled_{false};
  Clock::time_point start_;
};

// 请求预检：校验 + 立即取消。返回 kSuccess 表示可以继续；
// 否则已填充 `reason`（kInvalidInput / kCancelled）。
inline common::Status precheck(const GridSearchRequest& request,
                               std::string* reason) {
  const common::Status status = validate_request(request, reason);
  if (status != common::Status::kSuccess) {
    return status;
  }
  if (request.cancelled) {
    if (reason != nullptr) {
      *reason = "cancelled by caller";
    }
    return common::Status::kCancelled;
  }
  return common::Status::kSuccess;
}

inline GridSearchResult make_failure(common::Status status, std::string reason,
                                     GridSearchStats stats = GridSearchStats{}) {
  GridSearchResult result;
  result.status = status;
  result.reason = std::move(reason);
  result.stats = std::move(stats);
  return result;
}

// 由后端返回的 start->goal 路径构造统一成功结果：
//   * 结构校验失败返回 kValidationFailed（不得静默接受违约路径）；
//   * `enforce_corner_rule=false` 仅供单列变体（JPS 原生允许对角穿角）；
//   * cost_m 按路径以 double 重算（上游 float32 自报值放 stats.upstream_cost）。
inline GridSearchResult make_path_result(const GridSearchRequest& request,
                                         std::vector<GridCell> path,
                                         GridSearchStats stats,
                                         bool enforce_corner_rule = true) {
  const std::string problem = validate_path(
      *request.grid, request, path, enforce_corner_rule);
  if (!problem.empty()) {
    GridSearchResult result;
    result.status = common::Status::kValidationFailed;
    result.reason = "adapter produced an invalid path: " + problem;
    result.stats = std::move(stats);
    return result;
  }
  GridSearchResult result;
  result.status = common::Status::kSuccess;
  result.cost_m =
      path_cost_m(*request.grid, request.neighborhood, path);
  result.path = std::move(path);
  result.stats = std::move(stats);
  return result;
}

// 邻域启发式（单元步长，可采纳）：
//   * 八邻域：octile = max + (sqrt(2)-1)*min；
//   * 四邻域：Manhattan。
inline double neighborhood_heuristic(Neighborhood neighborhood, int d_row,
                                     int d_col) {
  const double abs_row = std::abs(static_cast<double>(d_row));
  const double abs_col = std::abs(static_cast<double>(d_col));
  if (neighborhood == Neighborhood::kFour) {
    return abs_row + abs_col;
  }
  const double maximum = abs_row > abs_col ? abs_row : abs_col;
  const double minimum = abs_row > abs_col ? abs_col : abs_row;
  return maximum + (std::sqrt(2.0) - 1.0) * minimum;
}

}  // namespace planner::grid_search::detail
