/**
 * @file hybrid_astar.h
 * @brief Hybrid A* kinodynamic path planner for car-like vehicles.
 */

#ifndef astar_HYBRID_ASTAR_H
#define astar_HYBRID_ASTAR_H

#include <cstdint>
#include <functional>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "planner_config.h"

namespace astar {

/**
 * @brief Discrete vehicle gear / motion direction.
 */
enum class MotionDirection : int32_t {
  Forward = 0,   ///< Vehicle travels forward.
  Backward = 1   ///< Vehicle travels in reverse.
};

/**
 * @brief 2D vehicle pose with orientation and travel direction.
 */
struct Pose2D {
  double x = 0.0;                                       ///< X position coordinate (m).
  double y = 0.0;                                       ///< Y position coordinate (m).
  double yaw = 0.0;                                     ///< Heading angle (rad).
  MotionDirection direction = MotionDirection::Forward; ///< Travel direction associated with this pose.
};

/**
 * @brief Search outcome containing status, path, and statistics.
 */
struct HybridSearchResult {
  bool success = false;          ///< True if a feasible path to the goal was found.
  double cost = 0.0;             ///< Total accumulated path cost (g-value).
  int32_t iterations = 0;        ///< Number of node expansions executed during search.
  std::vector<Pose2D> path;      ///< Sequence of waypoints forming the planned trajectory.
  std::string message;           ///< Diagnostic message or termination reason.
};

/**
 * @brief Hybrid A* path planning engine with kinodynamic node expansion.
 */
class HybridAStar {
 public:
  /**
   * @brief Constructs a Hybrid A* planner instance with the specified configuration.
   * @param[in] config Planner settings including vehicle geometry and cost weights.
   */
  explicit HybridAStar(const PlannerConfig& config);

  /**
   * @brief Updates the planner configuration.
   * @param[in] config New configuration to apply.
   */
  void SetConfig(const PlannerConfig& config);

  /**
   * @brief Retrieves the active planner configuration.
   * @return Const reference to current configuration.
   */
  const PlannerConfig& GetConfig() const;

  /**
   * @brief Clears all ingested boundary obstacle points.
   */
  void ClearBoundaries();

  /**
   * @brief Loads obstacle boundary polyline points from a CSV file.
   * @param[in] csv_path File system path to the CSV file.
   * @param[in] close_loop Whether to connect the last point back to the first point.
   * @return True if parsing succeeded and points were loaded.
   */
  bool LoadBoundaryFromCsv(const std::string& csv_path, bool close_loop);

  /**
   * @brief Returns the number of rasterized boundary obstacle cells currently stored.
   * @return Count of occupied boundary grid cells.
   */
  std::size_t BoundaryCount() const;

  /** @name Search Hooks and Boundary Ingestion
   *  Optional callbacks and programmatic boundary injection. When unset, defaults
   *  to internal grid collision and Euclidean/yaw goal tolerances.
   *  @{
   */

  /**
   * @brief Registers an early termination callback evaluated at the beginning of each expansion iteration.
   * @param[in] callback Function returning true if search should be cancelled immediately.
   */
  void SetAbortCallback(std::function<bool()> callback);

  /**
   * @brief Overrides the default grid collision test with a custom pose collision hook.
   * @param[in] hook Function returning true if the vehicle pose is in collision.
   */
  void SetPoseCollisionHook(std::function<bool(const Pose2D&)> hook);

  /**
   * @brief Overrides default motion segment checking during node propagation.
   * @param[in] hook Function returning true if motion between two poses collides.
   */
  void SetMotionCollisionHook(
      std::function<bool(const Pose2D&, const Pose2D&)> hook);

  /**
   * @brief Overrides the default distance/heading goal criteria.
   * @param[in] hook Function returning true if the evaluated pose satisfies the goal condition.
   */
  void SetGoalHook(std::function<bool(const Pose2D&)> hook);

  /**
   * @brief Programmatically rasterizes a world-frame obstacle point into the internal boundary grid.
   * @param[in] x World X coordinate (m).
   * @param[in] y World Y coordinate (m).
   */
  void AddBoundaryWorldPoint(double x, double y);

  /** @} */

  /**
   * @brief Executes Hybrid A* search between start and goal poses.
   * @param[in] start Initial vehicle pose.
   * @param[in] goal Target vehicle pose.
   * @param[out] result Output search statistics and planned trajectory.
   * @return True if a valid path was found.
   */
  bool Search(const Pose2D& start, const Pose2D& goal, HybridSearchResult* result);

 private:
  /**
   * @brief Internal 3D graph search node (x, y, yaw).
   */
  struct Vertex3D {
    uint64_t id = 0;                                       ///< Unique discrete state key.
    uint64_t parent_id = 0;                                ///< Key of the predecessor node.
    double x = 0.0;                                        ///< Continuous X position (m).
    double y = 0.0;                                        ///< Continuous Y position (m).
    double yaw = 0.0;                                      ///< Continuous heading (rad).
    MotionDirection direction = MotionDirection::Forward;  ///< Gear direction entering this state.
    double g = 0.0;                                        ///< Accumulated path cost from start.
    double h = 0.0;                                        ///< Estimated heuristic cost to goal.
    double f = 0.0;                                        ///< Total priority score (f = g + h).
  };

  /**
   * @brief Priority queue entry tracking node priority and key.
   */
  struct OpenEntry {
    double f = 0.0;       ///< Total cost priority.
    uint64_t id = 0;      ///< Node identifier.
  };

  /**
   * @brief Min-heap comparator for OpenEntry ordering by lowest f-cost.
   */
  struct OpenEntryGreater {
    bool operator()(const OpenEntry& lhs, const OpenEntry& rhs) const {
      if (lhs.f == rhs.f) {
        return lhs.id > rhs.id;
      }
      return lhs.f > rhs.f;
    }
  };

  /**
   * @brief Discrete 2D integer cell index.
   */
  struct GridPoint {
    int32_t x = 0;  ///< Column index.
    int32_t y = 0;  ///< Row index.
  };

  bool ParseCsvPoint(const std::string& line, double* x, double* y) const;
  void RasterizeLine(const GridPoint& start, const GridPoint& end);
  void AddBoundaryPoint(const GridPoint& p);

  bool IsBoundGrid(const GridPoint& p) const;
  bool IsCollision(const Vertex3D& node) const;
  bool IsGoal(const Vertex3D& node, const Pose2D& goal) const;

  uint64_t CoordinateHash(const GridPoint& p) const;
  uint64_t VertexHash(const Vertex3D& node) const;
  static double NormalizeAngle(double angle);
  static std::string Trim(const std::string& text);

  void ExpandNode(const Vertex3D& current, const Pose2D& start, const Pose2D& goal);
  Vertex3D Propagate(const Vertex3D& start, MotionDirection direction, double steering_rad) const;
  double TransitionCost(const Vertex3D& from, const Vertex3D& to) const;
  double Heuristic(const Vertex3D& node, const Pose2D& goal) const;
  bool ReconstructPath(uint64_t final_id, HybridSearchResult* result) const;

  PlannerConfig config_;
  double grid_dist_ = 1.0;
  double grid_angle_rad_ = 0.017453292519943295;
  double collision_front_meter_ = 1.0;
  double collision_rear_meter_ = 1.0;
  double collision_half_width_meter_ = 0.5;
  double collision_safe_margin_meter_ = 0.0;
  int32_t inflation_cells_ = 0;
  int32_t angle_bin_count_ = 360;
  Pose2D search_start_pose_;

  std::unordered_set<uint64_t> boundaries_;

  std::unordered_map<uint64_t, Vertex3D> open_set_;
  std::unordered_map<uint64_t, Vertex3D> closed_set_;
  std::unordered_map<uint64_t, Vertex3D> best_nodes_;
  std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryGreater> open_queue_;

  std::function<bool()> abort_callback_;
  std::function<bool(const Pose2D&)> pose_collision_hook_;
  std::function<bool(const Pose2D&, const Pose2D&)> motion_collision_hook_;
  std::function<bool(const Pose2D&)> goal_hook_;
};

}  // namespace astar

#endif  // astar_HYBRID_ASTAR_H
