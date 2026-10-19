#pragma once

// X-B1 采样搜索基准场景生成器（连续组 + 栅格组 + 容量档）。
//
// 规则来源（冻结）：
// - 连续组：Docs/expansion/benchmark_tiers.md §2.4（6 场景，100×100 m；容量档
//   为“更大边界 + 更多障碍”放大，§3）；
// - 栅格组：benchmark_tiers.md §2.2（6 类 × 3 规模，128²/512²/1024²，0.1 m；
//   几何实例取 seed 0）。`grid_scenario_gen v1` 尚未实现，本文件提供
//   `sampling_search_grid_gen v1`（暂定实现，规则按 §2.2 表），跨族一致性在
//   后续阶段与栅格族生成器交叉校验（见 benchmarks/README.md §3）。
//
// 确定性：所有实例由 id 完全确定（几何种子固定为 0；稀疏障碍/迷宫由
// SplitMix64 派生，逐位可复现）。实例的规范 JSON 与其 SHA256 由本模块提供，
// 写入 run 的 outputs/instances.json 与 outputs/instances.sha256.txt。
//
// 有效性语义：
// - 连续组：点有效性 = 界内且不在任何矩形障碍内（闭区间包含）；
//   边有效性 = 精确线段-矩形相交（Liang–Barsky，闭区间；与沿边步长无关）。
//   精确检查用于可靠探测 0.2 m 薄壁（不可达场景），避免采样步长漏检；
//   Problem::edge_check_step 仍显式声明（0.1 m）供需要显式步长的后端使用，
//   并进入结果元数据。
// - 栅格组：点有效性 = 按 resolution_m 取所在单元，单元受阻则无效；
//   边有效性 = 按 resolution 沿边采样（≥1 单元厚的障碍不会被跨越漏检）。

#include <cstdint>
#include <string>
#include <vector>

#include <planner/sampling_search/types.hpp>

namespace planner::sampling_search::bench {

// 轴对齐矩形障碍（闭区间）。
struct RectObstacle {
  double x0{0.0};
  double y0{0.0};
  double x1{0.0};
  double y1{0.0};
};

// 连续组实例（formal 6 个 + capacity 2 个）。
struct ContinuousInstance {
  std::string id;
  std::string class_name;  // open/sparse/narrow/trap/edge/unreachable
  bool capacity{false};
  double size_m{100.0};
  std::uint64_t geometry_seed{0};
  Bounds2d bounds{};
  Point2d start{};
  Point2d goal{};
  double edge_check_step{0.1};
  std::vector<RectObstacle> obstacles;
};

// 栅格组实例（formal 6 类 × 3 规模 + capacity 6 类 × 2 规模）。
struct GridInstance {
  std::string id;
  std::string class_name;  // open/maze/narrow/deadend/unreachable/border_corner
  bool capacity{false};
  int size{0};  // 边长（单元数），128/512/1024 或 2048/4096
  double resolution_m{0.1};
  std::uint64_t geometry_seed{0};
  // 单元占据：row-major，row 0 为底行（+y 向上），1 = occupied。
  std::vector<std::uint8_t> cells;
  int start_row{0};
  int start_col{0};
  int goal_row{0};
  int goal_col{0};

  std::uint8_t at(int row, int col) const {
    return cells[static_cast<std::size_t>(row) * static_cast<std::size_t>(size) +
                 static_cast<std::size_t>(col)];
  }
};

// 实例 id 清单（顺序稳定，供矩阵枚举与实例表）。
std::vector<std::string> continuous_instance_ids(bool include_capacity);
std::vector<std::string> grid_instance_ids(bool include_capacity);

// 由 id 确定性构造实例；未知 id 抛 std::runtime_error。
ContinuousInstance make_continuous_instance(const std::string& id);
GridInstance make_grid_instance(const std::string& id);

// 实例 → 统一 Problem（回调按值捕获实例几何）。
Problem make_problem(const ContinuousInstance& instance, double budget_s,
                     std::uint64_t seed);
Problem make_problem(const GridInstance& instance, double budget_s,
                     std::uint64_t seed);

// 规范 JSON（键排序、固定字段；不含 sha256 字段本身）与 SHA256。
std::string canonical_instance_json(const ContinuousInstance& instance);
std::string canonical_instance_json(const GridInstance& instance);
std::string instance_sha256(const ContinuousInstance& instance);
std::string instance_sha256(const GridInstance& instance);

// 分组标签：continuous / grid / capacity_continuous / capacity_grid。
std::string group_of(const std::string& instance_id);

// 生成器名与版本（写入实例表与 summary）。
inline constexpr char kContinuousGenerator[] =
    "sampling_search_continuous_gen v1";
inline constexpr char kGridGenerator[] = "sampling_search_grid_gen v1";

// 几何谓词（夹具与自检共用；精确连续语义见文件头）。
bool continuous_point_valid(const ContinuousInstance& instance,
                            const Point2d& point);
bool continuous_edge_valid(const ContinuousInstance& instance,
                           const Point2d& a, const Point2d& b);
bool grid_point_valid(const GridInstance& instance, const Point2d& point);

// 生成器自检（不依赖任何后端）：对每个实例核对
// ① 起终点有效；② 稀疏场景保守粗网格可达；③ 不可达场景保守粗网格不可达；
// ④ 迷宫生成规则保证连通（房间生成树）。返回空字符串表示通过，否则返回
// 首个失败原因。
std::string generator_self_check(bool include_capacity);

}  // namespace planner::sampling_search::bench
