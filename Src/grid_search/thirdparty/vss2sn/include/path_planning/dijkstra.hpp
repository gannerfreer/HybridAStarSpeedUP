/**
 * @file dijkstra.hpp
 * @author vss2sn
 * @brief Contains the Dijkstra class
 */

#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <queue>

#include "path_planning/planner.hpp"
#include "utils/utils.hpp"

/**
 * @brief Class for objects that plan using the Dijkstra algorithm
 */
class Dijkstra : public Planner {
 public:
  /**
   * @brief Constructor
   * @param grid the grid on which the planner is to plan
   * @return no return value
   */
  explicit Dijkstra(std::vector<std::vector<int>> grid)
      : Planner(std::move(grid)) {}

  /**
   * @brief Dijkstra algorithm implementation
   * @param start start node
   * @param goal goal node
   * @return tuple contatining a bool as to whether a path was found, and the
   * path
   */
  std::tuple<bool, std::vector<Node>> Plan(const Node& start,
                                           const Node& goal) override;

  /**
   * @brief [grid_search 适配补丁 P3] 完整距离场（上游无此 API）
   * @param start 源点（必须空闲；适配层负责预校验）
   * @return n x n 距离矩阵，索引 [x][y]（与内部栅格一致）；单元步长
   *         （正交 1、对角 sqrt(2)）；不可达为 +infinity
   * @details 与 Plan 使用相同的运动原语（SetMotion）、禁对角穿角规则与
   * 代价语义，完整展开全部可达单元；取消/超时钩子命中时提前返回部分距离场
   * （适配层据 ShouldCancel 判定并丢弃）。距离单位为单元步长。
   */
  std::vector<std::vector<double>> PlanDistanceField(const Node& start);

 private:
  std::vector<Node> ConvertClosedListToPath(
      std::unordered_set<Node, NodeIdAsHash, compare_coordinates>& closed_list,
      const Node& start, const Node& goal);
};

#endif  // DIJKSTRA_H
