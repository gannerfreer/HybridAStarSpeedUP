// Minimal stub of nav2_smac_planner::NodeHybrid for the SmacPlanner2D decoupling probe.
// (family C / X-S S-07 ad-hoc feasibility probe; NOT upstream code)
//
// Why this stub exists: node_2d.hpp / node_basic.hpp / goal_manager.hpp / a_star.hpp
// include node_hybrid.hpp unconditionally, and several `if constexpr` branches test
// std::is_base_of_v<NodeHybrid, NodeT>. For a Node2D-only extraction those branches are
// discarded at instantiation, but the class type must still be complete for is_base_of.
// The real NodeHybrid pulls in OMPL SE2 spaces, Costmap2DROS, inflation layer and
// obstacle/distance heuristics - all outside the Node2D search path.

#ifndef NAV2_SMAC_PLANNER__NODE_HYBRID_HPP_
#define NAV2_SMAC_PLANNER__NODE_HYBRID_HPP_

namespace nav2_smac_planner
{

class NodeHybrid
{
public:
  NodeHybrid() = default;
  virtual ~NodeHybrid() = default;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__NODE_HYBRID_HPP_
