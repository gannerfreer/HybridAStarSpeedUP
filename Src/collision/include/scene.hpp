/**
 * @file scene.hpp
 * @brief 碰撞检测场景表征：车辆足迹包络、凸多边形障碍物与栅格地图坐标系
 * @details 碰撞环境的核心建模：
 *          - 车辆足迹模型 (Footprint)：由轴距、前后悬、车宽及安全裕量构建的刚体矩形包络
 *          - 凸多边形障碍物 (Obstacle)：局部世界坐标系下的逆时针闭合凸多边形集合
 *          - 栅格地图视图 (GridView) 与坐标系变换 (GridFrame)：由通用 common 模块提供
 */

#pragma once

#include <array>
#include <cmath>
#include <string>
#include <string_view>
#include <vector>

#include "planner/common/grid_view.hpp"
#include "planner/common/scenario_config.hpp"
#include "planner/common/vehicle.hpp"
#include "planner/math/box2d.hpp"
#include "planner/math/polygon2d.hpp"

namespace planner::collision {
using common::Point2d;
using common::Pose2d;

using planner::common::GridView;
using planner::common::VehicleModel;

/**
 * @brief 局部世界坐标系下的凸多边形静态障碍物
 */
struct Obstacle {
  std::string id;           ///< 障碍物唯一标识符
  math::Polygon2d polygon;  ///< 按逆时针 (CCW) 排列的闭合凸多边形（顶点数 >= 3）

  /**
   * @brief 创建轴对齐矩形凸障碍物的工厂方法
   * @param[in] id 障碍物标识符
   * @param[in] x_min_m X 轴下界（米）
   * @param[in] y_min_m Y 轴下界（米）
   * @param[in] x_max_m X 轴上界（米）
   * @param[in] y_max_m Y 轴上界（米）
   * @return 包含 4 个 CCW 顶点的 Obstacle，若参数非法则返回空多边形
   */
  static Obstacle rectangle(std::string_view id, double x_min_m, double y_min_m, double x_max_m,
                            double y_max_m);

  /**
   * @brief 创建通用凸多边形障碍物的工厂方法（自动归一化为逆时针）
   * @param[in] id 障碍物标识符
   * @param[in] vertices 多边形顶点数组
   * @return 包含逆时针顶点的 Obstacle，若非凸或退化则返回空多边形
   */
  static Obstacle convex_polygon(std::string_view id, std::vector<Point2d> vertices);
};

/**
 * @brief 规划与碰撞检测场景的综合环境模型
 */
struct Scene {
  GridView grid;                    ///< 二维占用栅格地图视图
  std::vector<Obstacle> obstacles;  ///< 凸多边形障碍物列表

  /**
   * @brief 检查场景是否包含非空栅格地图
   * @return 若栅格宽高均大于 0 返回 true
   */
  bool has_grid() const noexcept { return grid.width > 0 && grid.height > 0; }

  /**
   * @brief 完整校验栅格尺寸、分辨率、单元缓冲区及所有障碍物的合法性
   * @param[out] reason 失败时的诊断说明文本指针
   * @return 若场景完全合法则返回 true
   */
  bool valid(std::string* reason = nullptr) const;
};

/**
 * @brief 根据纯数据场景配置构建碰撞检测场景
 * @details 栅格地图标识固定为 "scenario"/"1"，所有栅格默认为自由单元；
 *          障碍物由配置顶点构造为凸多边形输入非法时返回的 Scene::valid() 为 false
 * @param[in] scenario 场景配置数据
 * @return 构建得到的场景快照
 */
Scene make_scene(const common::ScenarioConfig& scenario);

/**
 * @brief 包含额外安全裕量的车体矩形足迹几何包络
 * @details 坐标系规范：车体局部坐标系，原点位于后轴中心，X 轴沿车身中心线朝前，Y 轴朝左
 */
struct Footprint {
  double x_min_m{0.0};  ///< 车体系纵向后向极限：-(rear_overhang + margin)（米）
  double x_max_m{0.0};  ///< 车体系纵向前向极限：wheelbase + front_overhang + margin（米）
  double y_min_m{0.0};  ///< 车体系横向右侧极限：-(width/2 + margin)（米）
  double y_max_m{0.0};  ///< 车体系横向左侧极限：+(width/2 + margin)（米）
  double corner_radius_m{0.0};  ///< 各角点相对后轴中心的最大回转半径，用于扫掠包围球上界估计（米）
  double half_width_m{0.0};   ///< 包含安全裕量的车身总半宽（米）
  double half_length_m{0.0};  ///< 包含安全裕量的车身总半长（米）
  double center_x_m{0.0};     ///< 足迹矩形几何中心在车体系下的 X 坐标（米）

  /**
   * @brief 计算车体系下的 4 个角点坐标，从 (x_min, y_min) 起沿逆时针排列
   * @return 包含 4 个车体系角点的定长数组
   */
  std::array<Point2d, 4> body_corners() const noexcept {
    return {Point2d{x_min_m, y_min_m}, Point2d{x_max_m, y_min_m}, Point2d{x_max_m, y_max_m},
            Point2d{x_min_m, y_max_m}};
  }

  /**
   * @brief 构造车体系下的足迹有向包围盒
   * @return 中心位于足迹几何中心、航向角为 0 的 Box2d
   */
  math::Box2d body_box() const noexcept {
    return math::Box2d{
        math::Vec2d{center_x_m, 0.0}, x_max_m - x_min_m, y_max_m - y_min_m, 0.0, 1.0, 0.0,
        math::Vec2d{0.0, 0.0},        body_corners()};
  }

  /**
   * @brief 构造车辆在指定世界位姿下的足迹有向包围盒
   * @param[in] pose 车辆后轴中心在世界系下的位姿
   * @return 世界坐标系下的 Box2d
   */
  math::Box2d world_box(const Pose2d& pose) const noexcept {
    // 单次求正余弦并批量变换 4 个角点，数值上与分别调用 transform_point 逐位一致
    // Box2d 复用缓存的正余弦与车体角点批量生成世界系角点
    const double c = std::cos(pose.yaw_rad);
    const double s = std::sin(pose.yaw_rad);
    const math::Vec2d center{pose.x_m + c * center_x_m, pose.y_m + s * center_x_m};
    return math::Box2d{center,
                       x_max_m - x_min_m,
                       y_max_m - y_min_m,
                       pose.yaw_rad,
                       c,
                       s,
                       math::Vec2d{pose.x_m, pose.y_m},
                       body_corners()};
  }

  /**
   * @brief 计算车辆在指定世界位姿下的 4 个世界系角点坐标
   * @param[in] pose 车辆后轴中心在世界系下的位姿
   * @return 包含 4 个世界系角点的定长数组
   */
  std::array<Point2d, 4> world_corners(const Pose2d& pose) const noexcept {
    return world_box(pose).corners();
  }
};

/**
 * @brief 校验车辆模型几何尺寸与最大前轮转角的合法性
 * @details 委托 common::vehicle_valid 执行统一校验，保留本接口以兼容既有调用方
 * @param[in] vehicle 车辆模型参数
 * @param[out] reason 失败时的诊断说明文本
 * @return 若所有尺寸与转角参数均为合法值则返回 true
 */
bool vehicle_footprint_valid(const VehicleModel& vehicle, std::string* reason = nullptr);

/**
 * @brief 根据车辆模型参数构造对应的矩形足迹模型
 * @param[in] vehicle 经校验合法的车辆参数
 * @return 构造生成的 Footprint 结构
 */
Footprint make_footprint(const VehicleModel& vehicle) noexcept;

}  // namespace planner::collision
