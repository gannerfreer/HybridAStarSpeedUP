/**
 * @file grid_view.hpp
 * @brief 二维栅格地图占用网格只读视图与栅格坐标系变换定义。
 */

#pragma once

#include <optional>
#include <string>
#include <vector>

#include "planner/math/pose2d.hpp"

namespace planner::common {

using math::Point2d;
using math::Pose2d;

/**
 * @name 栅格单元占用状态
 * @brief GridView::cells 中以行主序存储的单元占用状态整数值。
 * @{
 */
inline constexpr int kCellFree = 0;      ///< 自由空闲栅格，无障碍物且可安全通行。
inline constexpr int kCellOccupied = 1;  ///< 被障碍物占用的栅格。
inline constexpr int kCellUnknown = 2;   ///< 未知栅格（碰撞检测与规划保守视作占用）。
/// @}

/**
 * @brief 二维栅格地图占用网格只读视图。
 * @details 栅格单元状态以行主序平铺存储：0 = Free，1 = Occupied，2 = Unknown。
 *          超出地图边界的区域在规划与碰撞检测中均保守视作障碍物。
 *          `origin` 为栅格原点（对应第 0 行第 0 列单元的左下角）在局部坐标系中的位姿。
 *          单元格 (r, c) 的中心坐标为：
 *          \f$ \text{origin} + ((c + 0.5) \cdot \text{res}, (r + 0.5) \cdot \text{res}) \f$。
 */
struct GridView {
  std::string map_id;       ///< 地图唯一标识符。
  std::string map_version;  ///< 地图版本字符串，用于校验快照一致性。
  int width{0};             ///< 栅格列数（沿局部 X 轴方向）。
  int height{0};            ///< 栅格行数（沿局部 Y 轴方向）。
  double resolution_m{0.0}; ///< 正方形栅格单元边长分辨率（米）。
  Pose2d origin;            ///< 栅格地图原点位姿（[0,0] 单元左下角顶点）。
  std::vector<int> cells;   ///< 行主序存储的栅格单元占用状态数组。

  /**
   * @brief 检查给定的行列索引是否位于网格有效边界内。
   * @param[in] row 目标栅格行索引。
   * @param[in] col 目标栅格列索引。
   * @return 若 0 <= row < height 且 0 <= col < width 则返回 true，否则返回 false。
   */
  bool in_bounds(int row, int col) const {
    return row >= 0 && row < height && col >= 0 && col < width;
  }

  /**
   * @brief 查询指定 (row, col) 处的单元格占用状态。
   * @param[in] row 目标栅格行索引。
   * @param[in] col 目标栅格列索引。
   * @return 对应的占用状态整数值。
   * @throw std::out_of_range 若索引超出栅格边界。
   */
  int cell(int row, int col) const;

  /**
   * @brief 查询指定 (row, col) 处的单元格占用状态，越界时返回空值。
   * @param[in] row 目标栅格行索引。
   * @param[in] col 目标栅格列索引。
   * @return 若索引位于有效边界内则返回对应占用状态，否则返回 std::nullopt。
   */
  std::optional<int> try_cell(int row, int col) const noexcept;
};

/**
 * @name 精确全等比较操作符
 * @brief 规划核心数据结构的值相等性判断（用于缓存校验、回放对比与结果校验）。
 * @{
 */
bool operator==(const GridView& lhs, const GridView& rhs);
/// @}

/**
 * @brief 二维栅格地图坐标系几何定义与快速坐标变换缓存。
 * @details 网格坐标系中，单元 (r, c) 占据的区间为 [c*res, (c+1)*res] x [r*res, (r+1)*res]。
 */
struct GridFrame {
  double cos_yaw{1.0};       ///< 栅格原点航向角余弦值。
  double sin_yaw{0.0};       ///< 栅格原点航向角正弦值。
  double origin_x{0.0};      ///< 栅格原点在世界系下的 X 坐标（米）。
  double origin_y{0.0};      ///< 栅格原点在世界系下的 Y 坐标（米）。
  double width_m{0.0};       ///< 地图总物理宽度：width * resolution_m（米）。
  double height_m{0.0};      ///< 地图总物理高度：height * resolution_m（米）。
  double resolution_m{0.0};  ///< 正方形网格单边分辨率（米）。

  /**
   * @brief 从 GridView 提取并预计算 GridFrame 坐标变换结构。
   * @param[in] grid 输入栅格地图视图。
   * @return 初始化完成的 GridFrame。
   */
  static GridFrame from_grid(const GridView& grid) noexcept;

  /**
   * @brief 将世界坐标系下的点变换至栅格局部坐标系。
   * @param[in] world_point 世界系点。
   * @return 栅格局部系点。
   */
  Point2d to_grid_frame(const Point2d& world_point) const noexcept;

  /**
   * @brief 判定栅格坐标系下的点是否位于地图矩形 [0, width_m] x [0, height_m] 内。
   * @param[in] grid_point 栅格系点。
   * @return 若在地图内或边界上返回 true。
   */
  bool inside_map(const Point2d& grid_point) const noexcept;

  /**
   * @brief 计算栅格坐标点到地图矩形边界的有向最短距离（在外部为负，在内部为正）。
   * @param[in] grid_point 栅格系点。
   * @return 到边界的欧氏距离（米）。
   */
  double distance_to_boundary(const Point2d& grid_point) const noexcept;
};

}  // namespace planner::common
