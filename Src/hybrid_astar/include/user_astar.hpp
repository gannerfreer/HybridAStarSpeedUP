/**
 * @file user_astar.hpp
 * @brief 集成精确碰撞检测与 Reeds-Shepp 解析展开的高层 Hybrid A* 路径规划器接口头文件。
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>

#include "collision_checker.hpp"
#include "types.hpp"

namespace planner::hybrid_astar {

/**
 * @brief Hybrid A* 规划器配置参数结构体。
 */
struct UserAstarConfig {
  double xy_resolution_m = 0.5;                   ///< 状态空间离散化的位置栅格分辨率（米，m）。
  int heading_bins = 72;                          ///< 覆盖区间 [0, 2*pi) 的离散航向角分桶数量。
  double motion_step_m = 1.0;                     ///< 单次动力学外扩圆弧步长的弧长（米，m）。
  int steering_samples = 5;                       ///< 每次外扩时评估的离散前轮转向角采样数量。
  bool allow_reverse = true;                      ///< 节点外扩搜索中是否允许倒车动作。
  double forward_cost = 1.0;                      ///< 前进驱动运动代价权重系数。
  double reverse_cost = 2.0;                      ///< 倒车驱动运动代价权重系数。
  double turn_penalty = 1.0;                      ///< 弯道转向脱离直行时的代价惩罚乘数。
  double switch_cost = 2.0;                       ///< 换向挡位切换（前进 <-> 倒车）代价惩罚。
  double backward_search_range_m = 0.0;           ///< 允许倒车外扩的最大行驶距离（米，m）；0 表示不受限制。
  double linear_preferred_distance_square = 0.0;  ///< 起点附近偏好直行路径的平方距离阈值。
  double goal_yaw_tolerance_deg = 1.0;            ///< 目标位姿航向角容差阈值（度，deg）。
  int analytic_expansion_ratio = 3;               ///< 尝试 Reeds-Shepp 解析展开的节点外扩频率比率。
  double analytic_shot_distance_m = 0.0;          ///< 允许尝试解析展开的最大欧氏目标距离（米，m）；0 表示不受限制。

  /// Reeds-Shepp 连接路径的最大允许长度（米，m）。
  /// 剔除过长或过于贪心的连接曲线，促使搜索更靠近目标时再行解析连接。
  /// 0 表示对连接路径长度不设上限。
  double analytic_expansion_max_length_m = 12.0;

  double goal_capture_radius_m = 8.0;             ///< 开启目标直接捕获的距离阈值（米，m）。

  /// Reeds-Shepp 曲线中各微段的最小长度（米，m）。
  /// 低于该阈值的微段被视为数值退化微段（会导致虚假换向尖点），予以平滑剔除。
  double min_connector_segment_m = 0.15;

  double default_time_budget_s = 5.0;             ///< 默认单次规划的最大时间预算上限（秒，s）。
  std::int64_t default_node_budget = 1000000;     ///< 默认允许外扩的节点总数上限。
};

/**
 * @brief 线程兼容的 Hybrid A* 路径规划器类。
 * @note 单个规划器实例非线程安全，禁止在多线程间并发调用。
 *       在 plan() 执行过程中，输入的占据栅格、车辆模型与碰撞检测器必须保持只读不可变。
 */
class UserAstar {
 public:
  /**
   * @brief 使用指定的初始配置构造 UserAstar 规划器实例。
   * @param[in] config 规划器配置参数。
   */
  explicit UserAstar(const UserAstarConfig& config = {});

  /**
   * @brief 析构函数。
   */
  ~UserAstar();

  UserAstar(const UserAstar&) = delete;
  UserAstar& operator=(const UserAstar&) = delete;

  /**
   * @brief 更新规划器的配置参数。
   * @param[in] config 待应用的新配置参数。
   * @throw std::invalid_argument 若配置参数校验非法则抛出异常（同时安全保留原配置不变）。
   */
  void configure(const UserAstarConfig& config);

  /**
   * @brief 获取当前生效的规划器配置参数。
   * @return 当前配置参数的常量引用。
   */
  const UserAstarConfig& config() const;

  /**
   * @brief 规划从起点到目标位姿的无碰撞动力学可行路径。
   * @param[in] request 规划请求对象，包含起始/目标位姿以及时间与节点预算等边界约束。
   * @param[in] grid 用于障碍物感知的占据栅格地图视图。
   * @param[in] collision_checker 用于评估连续车辆扫掠轮廓的碰撞检测器。
   * @return 包含规划状态、生成的轨迹路径以及统计指标的规划结果对象。
   */
  common::PlanResult plan(const common::PlanRequest& request,
                          const common::GridView& grid,
                          const collision::CollisionChecker& collision_checker);

  /**
   * @brief 清空内部缓存的启发式势场网格与离散查找表。
   */
  void reset_cache();

  /**
   * @brief 注册外部取消回调函数，在搜索外扩循环中定期轮询。
   * @param[in] callback 当规划需要提前中止退出时返回 true 的函数回调。
   */
  void set_cancel_callback(std::function<bool()> callback);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace planner::hybrid_astar
