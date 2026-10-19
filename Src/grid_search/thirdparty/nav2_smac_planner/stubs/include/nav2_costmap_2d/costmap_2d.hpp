// Minimal stub of nav2_costmap_2d::Costmap2D for the SmacPlanner2D decoupling probe.
// (family C / X-S S-07 ad-hoc feasibility probe; NOT upstream code)
//
// Scope: only the API surface used by the extracted Node2D / AStarAlgorithm subset:
//   getSizeInCellsX/Y, getOriginX/Y, getResolution, getCost(index), getCost(mx,my),
//   mapToWorld, plus test-only setCost/setCostIndex.
// The real nav2 Costmap2D additionally provides update mechanics, parameter handling,
// layered plugins, world<->map conversions and bounds checks; none of those are needed
// to run the plain 2D A* search on a static grid.

#ifndef NAV2_COSTMAP_2D__COSTMAP_2D_HPP_
#define NAV2_COSTMAP_2D__COSTMAP_2D_HPP_

#include <cstddef>
#include <vector>

namespace nav2_costmap_2d
{

class Costmap2DROS;  // forward declaration only (not used by the Node2D search path)

class Costmap2D
{
public:
  Costmap2D(
    unsigned int size_x, unsigned int size_y, double resolution,
    double origin_x, double origin_y, unsigned char default_value = 0)
  : size_x_(size_x), size_y_(size_y), resolution_(resolution),
    origin_x_(origin_x), origin_y_(origin_y),
    costmap_(static_cast<std::size_t>(size_x) * size_y, default_value)
  {
  }

  unsigned int getSizeInCellsX() const {return size_x_;}
  unsigned int getSizeInCellsY() const {return size_y_;}
  double getOriginX() const {return origin_x_;}
  double getOriginY() const {return origin_y_;}
  double getResolution() const {return resolution_;}

  unsigned char getCost(unsigned int mx, unsigned int my) const
  {
    return costmap_[static_cast<std::size_t>(my) * size_x_ + mx];
  }

  unsigned char getCost(unsigned int index) const
  {
    return costmap_[index];
  }

  void setCost(unsigned int mx, unsigned int my, unsigned char cost)
  {
    costmap_[static_cast<std::size_t>(my) * size_x_ + mx] = cost;
  }

  void setCostIndex(unsigned int index, unsigned char cost)
  {
    costmap_[index] = cost;
  }

  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
  {
    wx = origin_x_ + (static_cast<double>(mx) + 0.5) * resolution_;
    wy = origin_y_ + (static_cast<double>(my) + 0.5) * resolution_;
  }

private:
  unsigned int size_x_;
  unsigned int size_y_;
  double resolution_;
  double origin_x_;
  double origin_y_;
  std::vector<unsigned char> costmap_;
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__COSTMAP_2D_HPP_
