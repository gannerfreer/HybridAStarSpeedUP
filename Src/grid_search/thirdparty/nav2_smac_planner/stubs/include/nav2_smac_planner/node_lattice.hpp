// Minimal stub of nav2_smac_planner::NodeLattice for the SmacPlanner2D decoupling probe.
// (family C / X-S S-07 ad-hoc feasibility probe; NOT upstream code)
//
// Same rationale as node_hybrid.hpp: the class type must be complete for
// std::is_base_of_v<NodeLattice, NodeT> checks, but all NodeLattice branches are
// discarded for a Node2D-only instantiation. The real NodeLattice depends on motion
// primitive JSON files, OMPL, Costmap2DROS and utils.hpp.

#ifndef NAV2_SMAC_PLANNER__NODE_LATTICE_HPP_
#define NAV2_SMAC_PLANNER__NODE_LATTICE_HPP_

namespace nav2_smac_planner
{

class NodeLattice
{
public:
  NodeLattice() = default;
  virtual ~NodeLattice() = default;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__NODE_LATTICE_HPP_
