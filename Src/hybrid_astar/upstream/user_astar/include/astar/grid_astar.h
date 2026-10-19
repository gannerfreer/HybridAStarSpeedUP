#ifndef astar_GRID_ASTAR_H
#define astar_GRID_ASTAR_H

#include <cstdint>
#include <set>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "astar/node2d.h"
#include "astar/planner_config.h"

namespace astar {

struct IntCoordinate {
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;
};

class GridAStar {
  public:
    explicit GridAStar(double grid_resolution = 1.0);
    explicit GridAStar(const PlannerConfig& config);

    void SetConfig(const PlannerConfig& config);

    void ClearBoundaries();
    bool LoadBoundaryFromCsv(const std::string& csv_path, bool close_loop);
    std::size_t BoundaryCount() const;
    const PlannerConfig& GetConfig() const;

    bool Search(const IntCoordinate& start, const IntCoordinate& goal, std::vector<IntCoordinate>* path, float* cost);

  private:
    uint32_t Coordinate2Hash(const IntCoordinate& point) const;
    IntCoordinate Hash2Coordinate(uint32_t hash) const;
    bool IsBoundGrid(const IntCoordinate& point) const;
    bool ParseCsvPoint(const std::string& line, double* x, double* y) const;
    void RasterizeLine(const IntCoordinate& start, const IntCoordinate& end);
    void AddBoundaryPoint(const IntCoordinate& point);
    float ComputeStepCost(int32_t move_index) const;

    static uint64_t OpenHash(const Node2D& node);
    static std::string Trim(const std::string& text);

    PlannerConfig                config_;
    double                       grid_resolution_;
    int32_t                      inflation_cells_ = 0;
    std::unordered_set<uint32_t> boundary_set_;
    std::unordered_map<uint32_t, Node2D> nodes2d_;
    std::set<uint64_t>                  open2d_;
    std::unordered_map<uint32_t, uint32_t> parent_;
};

} // namespace astar

#endif // astar_GRID_ASTAR_H
