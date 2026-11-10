/**
 * @file user_astar.hpp
 * @brief 集成精确碰撞检测与 Reeds-Shepp 解析展开的高层 Hybrid A* 路径规划器接口头文件。
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>

#include "collision_checker.hpp"
#include "planner/common/plan.hpp"
#include "planner/common/planner_config.hpp"

namespace planner::hybrid_astar {

using common::UserAstarConfig;

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
