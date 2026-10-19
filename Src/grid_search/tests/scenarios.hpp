#pragma once

// 测试夹具加载：场景（scenarios.json）与独立 Python 参考期望值
// （expected_costs.json）。两个文件均由 tests/reference/grid_reference.py
// 确定性生成，CTest 中的 *fixture_check* 测试负责复现校验。

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "planner/common/types.hpp"
#include "planner/grid_search/types.hpp"

namespace planner::grid_search::test {

struct Scenario {
  std::string id;
  common::GridView grid;
  GridCell start;
  GridCell goal;
  GridCell distance_source;
};

struct ExpectedNeighborhood {
  bool reachable{false};
  double cost_m{0.0};
  std::vector<double> distance_field_m;  // 行主序；不可达 = +inf
  std::int64_t unreachable_cells{0};
  double max_distance_m{0.0};
};

struct ExpectedScenario {
  std::string id;
  ExpectedNeighborhood four;
  ExpectedNeighborhood eight;
  ExpectedNeighborhood eight_corner_cut_allowed;
};

// 夹具目录（由 CMake 以编译定义注入）。
std::string fixture_dir();

// 全部场景（顺序与夹具一致）。
std::vector<Scenario> load_scenarios();

// 期望值（按场景 id）。
std::map<std::string, ExpectedScenario> load_expected();

// 取期望（neighborhood 决定 four/eight；JPS 单列用 cut_allowed）。
const ExpectedNeighborhood& expected_for(const ExpectedScenario& scenario,
                                         Neighborhood neighborhood,
                                         bool corner_cut_allowed = false);

// 由场景构造请求。
GridSearchRequest make_request(const Scenario& scenario,
                               Neighborhood neighborhood);

std::string context_of(const std::string& backend, const Scenario& scenario,
                       Neighborhood neighborhood);

}  // namespace planner::grid_search::test
