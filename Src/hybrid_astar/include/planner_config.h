/**
 * @file planner_config.h
 * @brief 规划器配置参数结构体定义文件，涵盖车辆几何外形尺寸、Hybrid A* 核心搜索参数以及起止位姿配置。
 */

#ifndef astar_PLANNER_CONFIG_H
#define astar_PLANNER_CONFIG_H

#include <string>

namespace astar {

/**
 * @brief 车辆几何几何外形尺寸与碰撞安全膨胀裕度配置。
 */
struct VehicleConfig {
  double center2front = 1.0;   ///< 车辆参考中心至前保险杠的距离（米，m）。
  double center2rear = 1.0;    ///< 车辆参考中心至后保险杠的距离（米，m）。
  double center2side = 0.5;    ///< 车辆参考中心至左/右侧边缘的半宽（米，m）。
  double safe_margin = 0.0;    ///< 车体外围额外安全膨胀裕度（米，m）。
  double wheel_base = 1.0;     ///< 前后车轴轴距（米，m）。

  /// 地图边界额外安全膨胀距离（米，m）。当数值 >= 0 时直接应用该边界裕度；
  /// 负值表示不对地图外边界施加额外膨胀。
  double boundary_inflation_meter = -1.0;
};

/**
 * @brief Hybrid A* 算法搜索参数与代价惩罚系数配置。
 */
struct HybridAStarConfig {
  double min_steering = -30.0;                   ///< 最小前轮转向角（度，deg）。
  double max_steering = 30.0;                    ///< 最大前轮转向角（度，deg）。
  double max_iterations_astar = 100000.0;        ///< A* 节点扩展的最大迭代步数上限。
  double angle_discrete_num = 8.0;               ///< 前轮转向角离散化档位数量。
  double delta_dist = 1.0;                       ///< 单次外扩圆弧运动步长的弧长（米，m）。
  double grid_dist = 1.0;                        ///< 三维状态空间栅格（x, y, yaw）的空间位置分辨率（米，m）。
  double grid_angle = 1.0;                       ///< 三维状态空间栅格的航向角分辨率（弧度或度）。
  double goal_yaw_tolerance_deg = 1.0;           ///< 目标位姿航向角容差阈值（度，deg）。
  double forward_penalty = 1.0;                  ///< 前进驱动运动代价惩罚权重系数。
  double backward_penalty = 1.0;                 ///< 倒车驱动运动代价惩罚权重系数。
  double turnning_penalty = 1.0;                 ///< 转向弯道运动代价惩罚权重系数。
  double switch_penalty = 1.0;                   ///< 换向挡位切换（前进 <-> 倒车）代价惩罚权重系数。
  double backward_search_range = 0.0;            ///< 允许倒车外扩的最大搜索距离（米，m）。
  double linear_preferred_distance_square = 0.0; ///< 偏好直线行驶的平方距离阈值。

  /// 节点外扩搜索过程中是否允许倒车运动。
  bool allow_reverse = true;

  /// 当为 true 时，倒车运动严格遵循标准运动学自行车模型
  /// （航向角增量 delta_yaw = -beta，位置沿对应圆弧更新）。
  bool standard_reverse_kinematics = false;
};

/**
 * @brief 规划问题全局配置结构体，整合车辆尺寸、搜索参数以及起止位姿条件。
 */
struct PlannerConfig {
  VehicleConfig vehicle;            ///< 车辆几何外形参数。
  HybridAStarConfig hybrid_astar;   ///< Hybrid A* 算法搜索参数。

  /**
   * @brief 起始与目标位姿及行驶方向配置结构体。
   */
  struct StartGoalConfig {
    double start_x = 4.0;           ///< 起点 X 轴坐标（米，m）。
    double start_y = 4.0;           ///< 起点 Y 轴坐标（米，m）。
    double start_yaw_deg = 0.0;     ///< 起点航向角（度，deg）。
    int start_direction = 0;        ///< 起始行驶方向（0：未指定/前进）。
    double goal_x = 6.0;            ///< 目标点 X 轴坐标（米，m）。
    double goal_y = 6.0;            ///< 目标点 Y 轴坐标（米，m）。
    double goal_yaw_deg = 90.0;     ///< 目标点航向角（度，deg）。
    int goal_direction = 0;         ///< 目标行驶方向。
  } start_goal;
};

}  // namespace astar

#endif  // astar_PLANNER_CONFIG_H
