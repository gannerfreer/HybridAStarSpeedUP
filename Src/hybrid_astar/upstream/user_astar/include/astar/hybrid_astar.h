#ifndef astar_HYBRID_ASTAR_H
#define astar_HYBRID_ASTAR_H

#include <cstdint>
#include <functional>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "astar/planner_config.h"

namespace astar {

enum class MotionDirection : int32_t {
    Forward = 0,
    Backward = 1
};

struct Pose2D {
    double          x = 0.0;
    double          y = 0.0;
    double          yaw = 0.0;
    MotionDirection direction = MotionDirection::Forward;
};

struct HybridSearchResult {
    bool                success = false;
    double              cost = 0.0;
    int32_t             iterations = 0;
    std::vector<Pose2D> path;
    std::string         message;
};

class HybridAStar {
  public:
    explicit HybridAStar(const PlannerConfig& config);

    void SetConfig(const PlannerConfig& config);
    const PlannerConfig& GetConfig() const;

    void ClearBoundaries();
    bool LoadBoundaryFromCsv(const std::string& csv_path, bool close_loop);
    std::size_t BoundaryCount() const;

    // [planner-adapter 扩展] 以下钩子与入口不改变搜索语义；未设置时
    // 行为与上游一致（内部栅格碰撞 + 内置目标容差）。
    // 中止回调在每个搜索迭代开始时检查；返回 true 时搜索以 "aborted" 退出。
    void SetAbortCallback(std::function<bool()> callback);
    // true = 碰撞。设置后取代内部栅格碰撞（IsCollision）。
    void SetPoseCollisionHook(std::function<bool(const Pose2D&)> hook);
    // true = 运动段碰撞。设置后在扩展时取代 IsCollision(succ)。
    void SetMotionCollisionHook(
        std::function<bool(const Pose2D&, const Pose2D&)> hook);
    // true = 到达目标。设置后取代内置的距离/航向容差判据。
    void SetGoalHook(std::function<bool(const Pose2D&)> hook);
    // 供栅格化障碍输入：把世界坐标点加入内部边界栅格（原 CSV 入口的
    // 程序化等价形式，保留相同膨胀规则）。
    void AddBoundaryWorldPoint(double x, double y);

    bool Search(const Pose2D& start, const Pose2D& goal, HybridSearchResult* result);

    double CalculateTransitionCostForTest(const Pose2D& from, const Pose2D& to) const;

  private:
    struct Vertex3D {
        uint64_t         id = 0;
        uint64_t         parent_id = 0;
        double           x = 0.0;
        double           y = 0.0;
        double           yaw = 0.0;
        MotionDirection  direction = MotionDirection::Forward;
        double           g = 0.0;
        double           h = 0.0;
        double           f = 0.0;
    };

    struct OpenEntry {
        double   f = 0.0;
        uint64_t id = 0;
    };

    struct OpenEntryGreater {
        bool operator()(const OpenEntry& lhs, const OpenEntry& rhs) const {
            if (lhs.f == rhs.f) {
                return lhs.id > rhs.id;
            }
            return lhs.f > rhs.f;
        }
    };

    struct GridPoint {
        int32_t x = 0;
        int32_t y = 0;
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
    double        grid_dist_ = 1.0;
    double        grid_angle_rad_ = 0.017453292519943295;
    double        collision_front_meter_ = 1.0;
    double        collision_rear_meter_ = 1.0;
    double        collision_half_width_meter_ = 0.5;
    double        collision_safe_margin_meter_ = 0.0;
    int32_t       inflation_cells_ = 0;
    int32_t       angle_bin_count_ = 360;
    Pose2D        search_start_pose_;

    std::unordered_set<uint64_t> boundaries_;

    std::unordered_map<uint64_t, Vertex3D> open_set_;
    std::unordered_map<uint64_t, Vertex3D> closed_set_;
    std::unordered_map<uint64_t, Vertex3D> best_nodes_;
    std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryGreater> open_queue_;

    // [planner-adapter 扩展] 可选钩子；为空时保持上游行为。
    std::function<bool()>                    abort_callback_;
    std::function<bool(const Pose2D&)>       pose_collision_hook_;
    std::function<bool(const Pose2D&, const Pose2D&)> motion_collision_hook_;
    std::function<bool(const Pose2D&)>       goal_hook_;
};

} // namespace astar

#endif // astar_HYBRID_ASTAR_H
