/**
 * @file planner.hpp
 * @author vss2sn
 * @brief Contains the abstract planner class
 */

#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <functional>
#include <iostream>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "utils/utils.hpp"

/**
 *  Abstract class that is inherited by concerete implementaions of planner
 *  classes. The Plan function is a pure virtual funciton that is overloaded
 */
class Planner {
 public:
  /**
   * @brief Constructor
   * @param grid the grid on which the planner is to plan
   * @return no return value
   */
  Planner(std::vector<std::vector<int>> grid)
      : original_grid_(std::move(grid)), n_(original_grid_.size()){};

  /**
   * @brief Copy constructor
   * @return No return value
   * @details default
   */
  Planner(const Planner&) = default;

  /**
   * @brief Move constructor
   * @return No return value
   * @details default
   */
  Planner(Planner&&) = default;

  /**
   * @brief Copy assignment
   * @return No return value
   * @details default
   */
  Planner& operator=(const Planner&) = default;

  /**
   * @brief Move assignment
   * @return No return value
   * @details default
   */
  Planner& operator=(Planner&&) = default;

  /**
   * @brief Virtual destructor
   * @return No return value
   * @details default
   */

  virtual ~Planner() = default;

  /**
   * @brief Pure virtual function that is overloadde by planner implementations
   * @param start start node
   * @param goal goal node
   * @return tuple contatining a bool as to whether a path was found, and the
   * path
   */
  virtual std::tuple<bool, std::vector<Node>> Plan(const Node& start,
                                                   const Node& goal) = 0;

  /**
   * @brief [grid_search 适配补丁 P2] 设置运动原语集合（四/八邻域）
   * @param motion 运动原语；默认 = 上游 GetMotion()（四邻域）
   * @details 供适配层在八邻域模式下注入 GetMotion(true)；默认行为不变。
   */
  void SetMotion(std::vector<Node> motion) { motion_ = std::move(motion); }

  /**
   * @brief [grid_search 适配补丁 P2] 协作取消/超时钩子
   * @param checker 返回 true 时搜索循环尽快返回 {false, {}}（不区分取消与
   *        无路径，由适配层结合自身标志区分）
   * @details 上游搜索循环没有取消/超时钩子；补丁只在循环头增加一次检查，
   * 不改变正常搜索语义。
   */
  void SetCancelChecker(std::function<bool()> checker) {
    cancel_checker_ = std::move(checker);
  }

  /**
   * @brief [grid_search 适配补丁 P2] 查询取消/超时钩子
   * @return checker 已设置且返回 true
   */
  bool ShouldCancel() const { return cancel_checker_ && cancel_checker_(); }

  /**
   * @brief Sets the time discovered obstacles and the option that allows
            the creation of random obstacles
   * @param create_random_obstacles should random obstacles be created during
   *        execution
   * @param time_discovered_obstacles obstacles to be discovered at specific
   *        times
   * @return void
   * @details Set separately from the plan function to allow this to persist
              between calls to Plan()
   */
  virtual void SetDynamicObstacles(
      const bool create_random_obstacles = false,
      const std::unordered_map<int, std::vector<Node>>&
          time_discovered_obstacles = {}) {
    std::cout << "Please implement this function for the planner" << '\n';
    std::cout << "Value attempted to be set: " << '\n';
    std::cout << "Create random obstacles: " << create_random_obstacles << '\n';
    std::cout << "Number of time discovered obstacles: "
              << time_discovered_obstacles.size() << '\n';
  };

 protected:
  std::vector<std::vector<int>> grid_ = {};
  const std::vector<std::vector<int>> original_grid_;
  const int n_;
  // [grid_search 适配补丁 P2] 运动原语与取消/超时钩子；默认值保持上游行为。
  std::vector<Node> motion_ = GetMotion();
  std::function<bool()> cancel_checker_;
};

#endif  // PLANNER_HPP
