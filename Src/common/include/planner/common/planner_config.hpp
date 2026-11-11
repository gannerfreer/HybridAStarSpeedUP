/**
 * @file planner_config.hpp
 * @brief Hybrid A* 规划器配置参数结构体定义与合法性校验
 */

#pragma once

#include <cstdint>
#include <string>

namespace planner::common {

/**
 * @brief Hybrid A* 规划器配置参数结构体
 */
struct UserAstarConfig {
  double xy_resolution_m = 0.5;  ///< 状态空间离散化的位置栅格分辨率（米，m）
  int heading_bins = 72;         ///< 覆盖区间 [0, 2*pi) 的离散航向角分桶数量
  double motion_step_m = 1.0;    ///< 单次动力学外扩圆弧步长的弧长（米，m）
  int steering_samples = 5;      ///< 每次外扩时评估的离散前轮转向角采样数量
  bool allow_reverse = true;     ///< 节点外扩搜索中是否允许倒车动作
  double forward_cost = 1.0;     ///< 前进驱动运动代价权重系数
  double reverse_cost = 2.0;     ///< 倒车驱动运动代价权重系数
  double turn_penalty = 1.0;     ///< 弯道转向脱离直行时的代价惩罚乘数
  double switch_cost = 2.0;      ///< 换向挡位切换（前进 <-> 倒车）代价惩罚
  double backward_search_range_m = 0.0;  ///< 允许倒车外扩的最大行驶距离（米，m）；0 表示不受限制
  double linear_preferred_distance_m2 = 0.0;  ///< 起点附近偏好直行路径的平方距离阈值
  int analytic_expansion_ratio = 3;  ///< 尝试 Reeds-Shepp 解析展开的节点外扩频率比率
  double analytic_shot_distance_m =
      0.0;  ///< 允许尝试解析展开的最大欧氏目标距离（米，m）；0 表示不受限制

  /// Reeds-Shepp 连接路径的最大允许长度（米，m）
  /// 剔除过长或过于贪心的连接曲线，促使搜索更靠近目标时再行解析连接
  /// 0 表示对连接路径长度不设上限
  double analytic_expansion_max_length_m = 12.0;

  double goal_capture_radius_m = 8.0;  ///< 开启目标直接捕获的距离阈值（米，m）

  /// Reeds-Shepp 曲线中各微段的最小长度（米，m）
  /// 低于该阈值的微段被视为数值退化微段（会导致虚假换向尖点），予以平滑剔除
  double min_connector_segment_m = 0.15;

  double default_time_budget_s = 5.0;  ///< 默认单次规划的最大时间预算上限（秒，s）
  std::int64_t default_node_budget = 1000000;  ///< 默认允许外扩的节点总数上限
};

/**
 * @brief 校验 Hybrid A* 规划器配置参数的合法性与数值范围
 * @details 分辨率、运动步长与目标捕获半径须为有限正数；默认时间预算须位于 (0, 1e9] 秒内；
 *          各代价/惩罚系数、搜索范围与连接参数须为有限非负数；
 *          航向分桶数须 >= 8、转向采样数须 >= 2、解析展开频率比率须 >= 1、
 *          默认节点预算须 >= 1
 * @param[in] config 待校验的规划器配置参数
 * @param[out] reason 失败时的诊断说明文本指针，可为 nullptr
 * @return 若配置全部合法则返回 true
 */
[[nodiscard]] bool planner_config_valid(const UserAstarConfig& config,
                                        std::string* reason = nullptr);

}  // namespace planner::common
