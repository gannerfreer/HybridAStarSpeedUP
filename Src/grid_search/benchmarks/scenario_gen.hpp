#pragma once

// grid_scenario_gen v1：正式档/容量档栅格场景的确定性生成规则。
//
// 规则依据：Docs/expansion/benchmark_tiers.md §2.2（6 类场景生成规则）、
// §3（容量档 2048²/4096² 单次、种子 0）与本文档 benchmarks/README.md 的
// “生成器规则 v1”（冻结解释与逐类公式）。
//
// 约定：
//   * 栅格 S×S，分辨率 0.1 m/格；0 = 空闲、1 = 占据（打包 uint8，行主序）；
//   * 每实例固定 1 对起终点（点到点）与 1 个距离场源点；
//   * 期望 success / no_path 由生成器用独立 BFS 自检（四/八邻域均验证）；
//   * 实例 ID：grid-<class>-<scale>-s<seed>；内容寻址为打包字节 SHA256。

#include <cstdint>
#include <string>
#include <vector>

#include "planner/grid_search/types.hpp"

namespace planner::grid_search::bench {

inline constexpr int kGeneratorVersion = 1;
inline constexpr double kBenchResolutionM = 0.1;
inline constexpr std::int64_t kArchiveThresholdBytes = 10 * 1024 * 1024;

struct ScenarioInstance {
  std::string instance_id;
  std::string scenario_class;  // open/maze/corridor/deadend/unreachable/border_corner
  int scale{0};                // S（128/512/1024/2048/4096）
  int seed{0};
  int width{0};
  int height{0};
  double resolution_m{kBenchResolutionM};
  GridCell start{};
  GridCell goal{};
  GridCell distance_source{};
  std::string expected;  // "success" / "no_path"
  std::vector<std::uint8_t> cells;  // 行主序；0 空闲、1 占据
};

// 冻结的类别与规模集合。
const std::vector<std::string>& all_scenario_classes();
bool is_valid_scenario_class(const std::string& name);
bool is_valid_scale(int scale);  // {128, 512, 1024, 2048, 4096}

std::string instance_id_for(const std::string& scenario_class, int scale,
                            int seed);

// 确定性生成；参数非法时抛出 std::invalid_argument。
ScenarioInstance generate_instance(const std::string& scenario_class, int scale,
                                   int seed);

struct InstanceValidation {
  bool ok{true};
  std::vector<std::string> problems;
};

// 生成器自检：尺寸/分辨率/边界墙、起终点与源点空闲、四/八邻域可达性与
// expected 一致（独立 BFS，不调用被测搜索库）。
InstanceValidation validate_instance(const ScenarioInstance& instance);

// 打包字节 SHA256（64 位小写十六进制）。
std::string instance_cells_sha256(const ScenarioInstance& instance);

}  // namespace planner::grid_search::bench
