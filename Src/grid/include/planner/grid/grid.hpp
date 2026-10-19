#pragma once

// planner_grid：与显示和地图实现无关的通用栅格数学与校验（X-A1-17）。
//
// 边界（冻结计划 §2.1）：
//   * 只依赖 planner::common（GridView 类型按冻结补充暂留 common）；
//   * 不暴露 Lanelet2 或任何上游类型，不运行任何搜索；
//   * 本库以自由函数提供：栅格/世界坐标换算、窗口单元数/预算降级数学、
//     栅格数据校验（三态取值与长度）。
//
// 坐标约定沿用 common::GridView 的冻结 v1 契约（与 ROS OccupancyGrid 一致）：
// `origin` 是单元 (0,0) 左下角的位姿；单元 (r,c) 的中心 = origin 沿其 +x
// 方向平移 (c + 0.5)*resolution_m、沿 +y 方向平移 (r + 0.5)*resolution_m
// 的位置（origin.yaw_rad 参与旋转）。单元取值 0=空闲、1=占据、2=未知；
// 消费方把未知与地图外视为占据。

#include <cstddef>
#include <optional>
#include <string>

#include "planner/common/types.hpp"

namespace planner::grid {

// 行列索引（行沿 +y、列沿 +x，(0,0) 为左下角）。
struct CellIndex {
  int row{0};
  int col{0};
};

bool operator==(const CellIndex& lhs, const CellIndex& rhs) noexcept;
bool operator!=(const CellIndex& lhs, const CellIndex& rhs) noexcept;

// 世界坐标 -> 包含该点的单元索引（floor 语义，不夹取；边界点属于上侧单元）。
// 非有限输入、空栅格、非有限或非正分辨率、索引超出 int 可表示范围时返回
// std::nullopt。
std::optional<CellIndex> world_to_cell(const common::GridView& grid,
                                       double x_m, double y_m) noexcept;

// 同 world_to_cell，但把结果夹取到 [0, width-1] x [0, height-1]。
// 空栅格或输入不可映射时返回 {0, 0}（调用方负责先校验位姿在窗口内）。
CellIndex world_to_cell_clamped(const common::GridView& grid,
                                double x_m, double y_m) noexcept;

// 单元中心的世界坐标（含 origin.yaw_rad 旋转；不做范围检查）。
common::Point2d cell_center(const common::GridView& grid, int row,
                            int col) noexcept;

// 覆盖长度 `span_m` 所需的单元数 ceil(span/resolution)。参数非有限或非正
// 时返回 0；结果超出 int 可表示范围时也返回 0（由调用方按预算拒绝）。
int ceil_cell_count(double span_m, double resolution_m) noexcept;

// 轴对齐窗口的栅格尺寸（ceil 语义，与 GridView 约定一致）。
struct WindowGridSpec {
  int width{0};
  int height{0};
  std::size_t cells{0};  // width * height
};

// 参数非有限或非正、或尺寸无法用 int/size_t 表示时返回 std::nullopt。
std::optional<WindowGridSpec> window_grid_spec(double width_m, double height_m,
                                               double resolution_m) noexcept;

// 单元预算降级结果：分辨率按 2 的幂增大，直至单元数不超过预算。
struct CellBudgetFit {
  double resolution_m{0.0};  // 请求值（无法降级时原样返回）
  std::size_t cells{0};      // 最终单元数；无法表示时为 0
  bool within_budget{false}; // 最终分辨率是否满足预算
};

// 最多翻倍 32 次（与场景构建器的既有降级上限一致）。`max_cells` 为 0 时
// 按 1 处理。参数无效时返回 within_budget=false 且 resolution 为请求值。
CellBudgetFit fit_resolution_to_cell_budget(double width_m, double height_m,
                                            double resolution_m,
                                            std::size_t max_cells) noexcept;

// 三态取值校验（0=空闲、1=占据、2=未知）。
bool cell_state_valid(int value) noexcept;

// 栅格数据校验：
//   * width/height 必须都为零（无栅格）或都为正；
//   * resolution 必须有限且为正；
//   * origin 的三个分量必须有限；
//   * cells 长度必须等于 width * height；
//   * 每个单元取值必须在 {0,1,2}。
// `reason` 可选地接收首个错误说明。空栅格（0x0，cells 为空）视为有效。
bool valid_grid(const common::GridView& grid, std::string* reason = nullptr);

}  // namespace planner::grid
