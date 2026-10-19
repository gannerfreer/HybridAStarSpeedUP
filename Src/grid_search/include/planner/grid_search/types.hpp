#pragma once

// planner::grid_search：统一栅格搜索契约（X-S3 栅格 A* / X-S2 栅格 Dijkstra）。
//
// 契约来源（已冻结）：
//   * Docs/component_expansion_plan.md §4.2（适配契约）；
//   * Docs/expansion/candidate_registration.md S-02/S-03/S-06/S-07/S-08；
//   * Docs/expansion/benchmark_tiers.md §2.2（栅格矩阵，规模分层）。
//
// 统一语义（所有后端必须遵守，适配偏差记录在 provenance.md）：
//   * 输入为只读 common::GridView、单元起终点（row/col，行主序）与邻域策略；
//   * 四邻域（kFour）与八邻域（kEight）分组；
//   * 八邻域禁止对角穿角：对角移动要求两个正交邻格均不阻塞；
//   * 未知（2）与地图外按阻塞；占据（1）按阻塞；
//   * 输出状态区分 success / no_path / timeout / cancelled / invalid_input；
//   * 代价单位为米（单元步长正交 1、对角 sqrt(2)，乘以 resolution_m）。
//
// 本头文件只依赖 planner::common，不暴露任何上游类型。

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "planner/common/types.hpp"

namespace planner::grid_search {

// 邻域策略。分组口径见契约：四邻域为对照组，八邻域为主组。
enum class Neighborhood {
  kFour,
  kEight,
};

std::string to_string(Neighborhood neighborhood);

// 解析 "four"/"eight"；未知输入返回 std::nullopt。
std::optional<Neighborhood> neighborhood_from_string(const std::string& text);

// 栅格单元索引（row 沿 +y、col 沿 +x，(0,0) 为左下角，与 GridView 一致）。
struct GridCell {
  int row{0};
  int col{0};
};

bool operator==(const GridCell& lhs, const GridCell& rhs) noexcept;
bool operator!=(const GridCell& lhs, const GridCell& rhs) noexcept;

// 点到点栅格搜索请求。`grid` 只读，生命周期由调用方保证。
// `time_budget_s` / `node_budget` 未设置表示无限制；预算耗尽按 timeout 报告，
// 不得与 no_path 混淆。`cancelled` 为调用方设置的协作取消标志（运行中由后端
// 轮询；未实现轮询的后端在启动时检查并立即返回 cancelled）。
struct GridSearchRequest {
  const common::GridView* grid{nullptr};
  GridCell start{};
  GridCell goal{};
  Neighborhood neighborhood{Neighborhood::kEight};
  std::optional<double> time_budget_s{};
  std::optional<std::int64_t> node_budget{};
  bool cancelled{false};
};

// 后端自报统计（未暴露的项保持 0；`upstream_cost` 未暴露时为 NaN）。
// `upstream_cost` 为上游原生代价（单元步长，可能为 float32 精度）；
// 统一 `GridSearchResult::cost_m` 一律由适配层按返回路径以 double 重算。
struct GridSearchStats {
  std::int64_t expanded_nodes{0};
  std::int64_t generated_nodes{0};
  std::int64_t iterations{0};
  std::int64_t collision_checks{0};
  double upstream_cost{0.0};
  std::string upstream_status;
};

// 点到点搜索结果。`path` 为 start -> goal 的单元序列（含两端）；
// 失败时为空。`cost_m` 与 `path` 一致（由路径重算，非上游自报值）。
struct GridSearchResult {
  common::Status status{common::Status::kInvalidInput};
  std::string reason;
  std::vector<GridCell> path;
  double cost_m{0.0};
  GridSearchStats stats;
};

// 请求校验（与后端无关）：栅格有效、起终点在界内且不被阻塞、预算合法。
// 返回 kSuccess 或 kInvalidInput（`reason` 可选接收说明）。
// 注意：目标不可达（合法输入）不属于 invalid_input。
common::Status validate_request(const GridSearchRequest& request,
                                std::string* reason);

// 路径结构校验（供测试与消费方使用）：起点/终点匹配、单元在界内且不被阻塞、
// 相邻步符合邻域与禁穿角规则。返回空字符串表示通过。
// `enforce_corner_rule=false` 仅供单列变体（jps3d JPS 原生允许对角穿角）使用；
// 统一排名口径必须保持 true（默认）。
std::string validate_path(const common::GridView& grid,
                          const GridSearchRequest& request,
                          const std::vector<GridCell>& path,
                          bool enforce_corner_rule = true);

// 按邻域策略重算路径代价（米）。调用方保证路径合法。
double path_cost_m(const common::GridView& grid, Neighborhood neighborhood,
                   const std::vector<GridCell>& path);

// 精确最短路组的统一比较容差：1e-8 + 1e-8 * |reference|（计划 §6.2）。
bool costs_match(double actual, double reference) noexcept;

// 供报告与日志使用：状态名（复用 common::to_string）。
std::string status_name(common::Status status);

}  // namespace planner::grid_search
