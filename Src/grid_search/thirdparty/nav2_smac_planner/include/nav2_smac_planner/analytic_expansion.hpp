// [grid_search 适配桩，非上游源码]
//
// 上游 nav2_smac_planner/analytic_expansion.{hpp,_impl.hpp}（navigation2 @
// 3286119753ce1fab9d2ff411ea6d439b823fe18d）在 Node2D 实例化时，
// tryAnalyticExpansion 的 `if constexpr (std::is_base_of_v<Node2D, NodeT>)`
// 分支无条件返回 nullptr（家族 C 探针实证），但会无条件包含 OMPL 头文件
// （仅解析、不链接）。为保持 grid_search 组件独立可配置（Smac 适配器开关默认
// ON，不引入 OMPL 获取/配置成本），这里以等价的 Node2D 专用桩替换：
//   * 接口与 a_star_impl.hpp 的调用点一致（构造/ setCollisionChecker /
//     setContext / tryAnalyticExpansion）；
//   * 语义 = 上游 Node2D 分支（返回 nullptr，不影响 A* 搜索行为）。
// 上游两个文件不进入抽取清单；SHA256 见 Src/grid_search/provenance.md。

#ifndef NAV2_SMAC_PLANNER__ANALYTIC_EXPANSION_HPP_
#define NAV2_SMAC_PLANNER__ANALYTIC_EXPANSION_HPP_

#include <functional>
#include <vector>

#include "nav2_smac_planner/constants.hpp"
#include "nav2_smac_planner/types.hpp"

namespace nav2_smac_planner
{

class GridCollisionChecker;

template<typename NodeT>
class AnalyticExpansion
{
public:
  typedef NodeT * NodePtr;
  typedef std::vector<NodePtr> NodeVector;
  typedef typename NodeT::CoordinateVector CoordinateVector;
  using NodeContext = typename NodeT::NodeContext;
  typedef std::function<bool (const uint64_t &, NodeT * &)> NodeGetter;

  AnalyticExpansion(
    const MotionModel & /*motion_model*/,
    const SearchInfo & /*search_info*/,
    const bool & /*traverse_unknown*/,
    const unsigned int & /*dim_3_size*/)
  {
  }

  void setCollisionChecker(GridCollisionChecker * /*collision_checker*/) {}
  void setContext(NodeContext * /*ctx*/) {}

  NodePtr tryAnalyticExpansion(
    const NodePtr & /*current_node*/,
    const NodeVector & /*coarse_check_goals*/,
    const NodeVector & /*fine_check_goals*/,
    const CoordinateVector & /*goals_coords*/,
    const NodeGetter & /*getter*/,
    int & /*iterations*/,
    int & /*closest_distance*/)
  {
    return nullptr;
  }
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__ANALYTIC_EXPANSION_HPP_
