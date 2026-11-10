/**
 * @file planner_config.h
 * @brief 规划器配置参数结构体定义文件，涵盖轴距、Hybrid A* 核心搜索参数。
 */

#ifndef astar_PLANNER_CONFIG_H
#define astar_PLANNER_CONFIG_H

#include <string>

namespace astar {

/**
 * @brief 车辆运动学参数。
 */
struct VehicleConfig {
  double wheel_base = 1.0;  ///< 轴距（米）。
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
  double grid_angle = 1.0;                       ///< 三维状态空间栅格的航向角分辨率（度）。
  double forward_penalty = 1.0;                  ///< 前进驱动运动代价惩罚权重系数。
  double backward_penalty = 1.0;                 ///< 倒车驱动运动代价惩罚权重系数。
  double turnning_penalty = 1.0;                 ///< 转向弯道运动代价惩罚权重系数。
  double switch_penalty = 1.0;                   ///< 换向挡位切换（前进 <-> 倒车）代价惩罚权重系数。
  double backward_search_range = 0.0;            ///< 允许倒车外扩的最大搜索距离（米，m）。
  double linear_preferred_distance_square = 0.0; ///< 偏好直线行驶的平方距离阈值。

  /// 节点外扩搜索过程中是否允许倒车运动。
  bool allow_reverse = true;

};

/**
 * @brief 规划问题配置结构体，整合轴距与搜索参数。
 */
struct PlannerConfig {
  VehicleConfig vehicle;            ///< 车辆几何外形参数。
  HybridAStarConfig hybrid_astar;   ///< Hybrid A* 算法搜索参数。

};

}  // namespace astar

#endif  // astar_PLANNER_CONFIG_H
