// Minimal stub of nav2_smac_planner::GridCollisionChecker for the SmacPlanner2D
// decoupling probe.
// (family C / X-S S-07 ad-hoc feasibility probe; NOT upstream code)
//
// Upstream GridCollisionChecker derives from
// nav2_costmap_2d::FootprintCollisionChecker<Costmap2D *> and needs Costmap2DROS,
// rclcpp_lifecycle and geometry_msgs. The Node2D search path only ever calls the
// index-based overload:
//
//   inCollision(index, traverse_unknown)  ->  center_cost_ = costmap_->getCost(index);
//                                              if (center_cost_ == UNKNOWN_COST && traverse_unknown)
//                                                return false;
//                                              return center_cost_ >= INSCRIBED_COST;
//   getCost()                             ->  returns center_cost_
//   getCostmap()                          ->  Costmap2D * used by AStarAlgorithm
//
// This stub reproduces exactly that logic (copied semantics from upstream
// src/collision_checker.cpp lines 172-189) and omits the footprint/pose overloads that
// the Node2D path does not use. If a future adapter uses footprints, the full checker
// must be extracted instead.

#ifndef NAV2_SMAC_PLANNER__COLLISION_CHECKER_HPP_
#define NAV2_SMAC_PLANNER__COLLISION_CHECKER_HPP_

#include <memory>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_smac_planner/constants.hpp"

namespace nav2_smac_planner
{

class GridCollisionChecker
{
public:
  explicit GridCollisionChecker(nav2_costmap_2d::Costmap2D * costmap)
  : costmap_(costmap)
  {
  }

  bool inCollision(const unsigned int & i, const bool & traverse_unknown)
  {
    center_cost_ = costmap_->getCost(i);
    if (center_cost_ == UNKNOWN_COST && traverse_unknown) {
      return false;
    }
    return center_cost_ >= INSCRIBED_COST;
  }

  float getCost()
  {
    // Assumes inCollision called prior (upstream contract)
    return static_cast<float>(center_cost_);
  }

  nav2_costmap_2d::Costmap2D * getCostmap()
  {
    return costmap_;
  }

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> getCostmapROS()
  {
    // Not available in the decoupled probe; only used by SE2 goal paths.
    return nullptr;
  }

  std::vector<float> & getPrecomputedAngles()
  {
    return angles_;
  }

protected:
  nav2_costmap_2d::Costmap2D * costmap_;
  float center_cost_{0.0f};
  std::vector<float> angles_;
};

}  // namespace nav2_smac_planner

#endif  // NAV2_SMAC_PLANNER__COLLISION_CHECKER_HPP_
