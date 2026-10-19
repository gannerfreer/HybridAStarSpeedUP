#pragma once

// 距离场接口：与栅格对齐的距离（米）及不可达标记。
//
// 契约（计划 §4.2）：距离场返回与栅格对齐的距离及不可达标记。
//   * `distances` 行主序，尺寸 width x height，与输入 GridView 逐单元对齐；
//   * 源点距离为 0；不可达单元距离为 +infinity（std::numeric_limits<double>::infinity()）；
//   * 阻塞单元（占据/未知）恒为不可达；
//   * 距离单位为米（单元步长正交 1、对角 sqrt(2)，乘以 resolution_m）；
//   * 邻域与禁穿角规则与点到点搜索一致（四/八邻域分组）。

#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "planner/common/types.hpp"
#include "planner/grid_search/types.hpp"

namespace planner::grid_search {

// 不可达标记。
inline constexpr double kUnreachableDistance =
    std::numeric_limits<double>::infinity();

struct DistanceField {
  int width{0};
  int height{0};
  std::vector<double> distances;  // 行主序；不可达为 +infinity

  bool empty() const noexcept { return distances.empty(); }
  bool in_bounds(int row, int col) const noexcept {
    return row >= 0 && row < height && col >= 0 && col < width;
  }
  // 越界抛出 std::out_of_range。
  double at(int row, int col) const;
  bool reachable(int row, int col) const;
  // 不可达单元数（含阻塞单元）。
  std::int64_t unreachable_count() const noexcept;
  // 可达单元中的最大距离（米）；全部不可达时返回 0。
  double max_reachable_m() const noexcept;
};

struct DistanceFieldRequest {
  const common::GridView* grid{nullptr};
  GridCell source{};
  Neighborhood neighborhood{Neighborhood::kEight};
  std::optional<double> time_budget_s{};
  std::optional<std::int64_t> node_budget{};
  bool cancelled{false};
};

struct DistanceFieldResult {
  common::Status status{common::Status::kInvalidInput};
  std::string reason;
  DistanceField field;
  GridSearchStats stats;
};

// 距离场请求校验（同点到点请求，起终点换为源点）。
common::Status validate_distance_field_request(
    const DistanceFieldRequest& request, std::string* reason);

}  // namespace planner::grid_search
