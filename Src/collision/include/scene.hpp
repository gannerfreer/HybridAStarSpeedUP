/**
 * @file scene.hpp
 * @brief 碰撞检测场景表征：车辆足迹包络、凸多边形障碍物与栅格地图坐标系。
 * @details 碰撞环境的核心建模：
 *          - 车辆足迹模型 (Footprint)：由轴距、前后悬、车宽及安全裕量构建的刚体矩形包络。
 *          - 凸多边形障碍物 (Obstacle)：局部世界坐标系下的逆时针闭合凸多边形集合。
 *          - 栅格地图坐标系 (GridFrame)：支持世界坐标系与栅格局部坐标系之间的快速变换及地图边界判定。
 */

#pragma once

#include <array>
#include <cmath>
#include <string>
#include <vector>

#include "planner/math/aabb2d.hpp"
#include "planner/math/box2d.hpp"
#include "planner/math/polygon2d.hpp"
#include "types.hpp"

namespace planner::collision {
using common::Point2d;
using common::Pose2d;
using common::Direction;

using planner::common::GridView;
using planner::common::VehicleModel;

/**
 * @brief 局部世界坐标系下的凸多边形静态障碍物。
 */
struct Obstacle {
  std::string id;          ///< 障碍物唯一标识符。
  math::Polygon2d polygon; ///< 按逆时针 (CCW) 排列的闭合凸多边形（顶点数 >= 3）。

  /**
   * @brief 创建轴对齐矩形凸障碍物的工厂方法。
   * @param[in] id 障碍物标识符。
   * @param[in] x_min_m X 轴下界（米）。
   * @param[in] y_min_m Y 轴下界（米）。
   * @param[in] x_max_m X 轴上界（米）。
   * @param[in] y_max_m Y 轴上界（米）。
   * @return 包含 4 个 CCW 顶点的 Obstacle，若参数非法则返回空多边形。
   */
  static Obstacle rectangle(const std::string& id, double x_min_m,
                            double y_min_m, double x_max_m, double y_max_m);

  /**
   * @brief 创建通用凸多边形障碍物的工厂方法（自动归一化为逆时针）。
   * @param[in] id 障碍物标识符。
   * @param[in] vertices 多边形顶点数组。
   * @return 包含逆时针顶点的 Obstacle，若非凸或退化则返回空多边形。
   */
  static Obstacle convex_polygon(const std::string& id,
                                 std::vector<Point2d> vertices);

  /**
   * @brief 校验障碍物顶点坐标、数量、面积及凸性是否合法。
   * @param[out] reason 失败时的诊断说明文本指针。
   * @return 若为有效凸多边形则返回 true。
   */
  bool valid(std::string* reason = nullptr) const;

  /**
   * @brief 获取逆时针顶点序列。
   * @return 顶点序列只读引用。
   */
  const std::vector<Point2d>& vertices() const noexcept {
    return polygon.vertices();
  }

  /**
   * @brief 获取顶点连续存储首地址。
   * @return 顶点数组指针。
   */
  const Point2d* data() const noexcept { return polygon.data(); }

  /**
   * @brief 获取顶点数量。
   * @return 顶点数量。
   */
  std::size_t size() const noexcept { return polygon.size(); }

  /**
   * @brief 判定障碍物多边形是否为空。
   * @return 若无顶点返回 true。
   */
  bool empty() const noexcept { return polygon.empty(); }

  /**
   * @brief 获取构造时缓存的紧密轴对齐包围盒。
   * @return 对应的 math::Aabb2d。
   */
  const math::Aabb2d& aabb() const noexcept { return polygon.aabb(); }
};

/**
 * @brief 规划与碰撞检测场景的综合环境模型。
 */
struct Scene {
  GridView grid;                    ///< 二维占用栅格地图视图。
  std::vector<Obstacle> obstacles;  ///< 凸多边形障碍物列表。

  /**
   * @brief 检查场景是否包含非空栅格地图。
   * @return 若栅格宽高均大于 0 返回 true。
   */
  bool has_grid() const noexcept { return grid.width > 0 && grid.height > 0; }

  /**
   * @brief 完整校验栅格尺寸、分辨率、单元缓冲区及所有障碍物的合法性。
   * @param[out] reason 失败时的诊断说明文本指针。
   * @return 若场景完全合法则返回 true。
   */
  bool valid(std::string* reason = nullptr) const;
};

/**
 * @brief 包含额外安全裕量的车体矩形足迹几何包络。
 * @details 坐标系规范：车体局部坐标系，原点位于后轴中心，X 轴沿车身中心线朝前，Y 轴朝左。
 */
struct Footprint {
  double x_min_m{0.0};          ///< 车体系纵向后向极限：-(rear_overhang + margin)（米）。
  double x_max_m{0.0};          ///< 车体系纵向前向极限：wheelbase + front_overhang + margin（米）。
  double y_min_m{0.0};          ///< 车体系横向右侧极限：-(width/2 + margin)（米）。
  double y_max_m{0.0};          ///< 车体系横向左侧极限：+(width/2 + margin)（米）。
  double corner_radius_m{0.0};  ///< 各角点相对后轴中心的最大回转半径，用于扫掠包围球上界估计（米）。
  double half_width_m{0.0};     ///< 包含安全裕量的车身总半宽（米）。
  double half_length_m{0.0};    ///< 包含安全裕量的车身总半长（米）。
  double center_x_m{0.0};       ///< 足迹矩形几何中心在车体系下的 X 坐标（米）。

  /**
   * @brief 计算车体系下的 4 个角点坐标，从 (x_min, y_min) 起沿逆时针排列。
   * @return 包含 4 个车体系角点的定长数组。
   */
  std::array<Point2d, 4> body_corners() const noexcept {
    return {Point2d{x_min_m, y_min_m}, Point2d{x_max_m, y_min_m},
            Point2d{x_max_m, y_max_m}, Point2d{x_min_m, y_max_m}};
  }

  /**
   * @brief 构造车体系下的足迹有向包围盒。
   * @return 中心位于足迹几何中心、航向角为 0 的 Box2d。
   */
  math::Box2d body_box() const noexcept {
    return math::Box2d{math::Vec2d{center_x_m, 0.0}, x_max_m - x_min_m,
                       y_max_m - y_min_m, 0.0, 1.0, 0.0,
                       math::Vec2d{0.0, 0.0}, body_corners()};
  }

  /**
   * @brief 构造车辆在指定世界位姿下的足迹有向包围盒。
   * @param[in] pose 车辆后轴中心在世界系下的位姿。
   * @return 世界坐标系下的 Box2d。
   */
  math::Box2d world_box(const Pose2d& pose) const noexcept {
    // 单次求正余弦并变换足迹中心；Box2d 复用缓存的正余弦与车体角点批量生成世界系角点。
    const double c = std::cos(pose.yaw_rad);
    const double s = std::sin(pose.yaw_rad);
    const math::Vec2d center{pose.x_m + c * center_x_m,
                             pose.y_m + s * center_x_m};
    return math::Box2d{center, x_max_m - x_min_m, y_max_m - y_min_m,
                       pose.yaw_rad, c, s, math::Vec2d{pose.x_m, pose.y_m},
                       body_corners()};
  }

  /**
   * @brief 计算车辆在指定世界位姿下的 4 个世界系角点坐标。
   * @param[in] pose 车辆后轴中心在世界系下的位姿。
   * @return 包含 4 个世界系角点的定长数组。
   */
  std::array<Point2d, 4> world_corners(const Pose2d& pose) const noexcept {
    return world_box(pose).corners();
  }
};

/**
 * @brief 校验车辆模型几何尺寸（轴距、车宽、前后悬、安全裕量）的合法性。
 * @param[in] vehicle 车辆模型参数。
 * @param[out] reason 失败时的诊断说明文本。
 * @return 若所有尺寸均为有限正数/非负数则返回 true。
 */
bool vehicle_footprint_valid(const VehicleModel& vehicle,
                             std::string* reason = nullptr);

/**
 * @brief 根据车辆模型参数构造对应的矩形足迹模型。
 * @param[in] vehicle 经校验合法的车辆参数。
 * @return 构造生成的 Footprint 结构。
 */
Footprint make_footprint(const VehicleModel& vehicle) noexcept;

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

}  // namespace planner::collision
