/**
 * @file hybrid_astar.h
 * @brief 针对类车（Car-like）机器人的 Hybrid A* 动力学路径规划器头文件。
 */

#ifndef astar_HYBRID_ASTAR_H
#define astar_HYBRID_ASTAR_H

#include <cstdint>
#include <functional>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

#include "planner_config.h"

namespace astar {

/**
 * @brief 离散车辆挡位与运动方向枚举。
 */
enum class MotionDirection : int32_t {
  Forward = 0,   ///< 车辆前进运动。
  Backward = 1   ///< 车辆倒车运动。
};

/**
 * @brief 包含二维连续坐标、航向角及行驶方向的车辆位姿。
 */
struct Pose2D {
  double x = 0.0;                                       ///< X 轴位置坐标（米，m）。
  double y = 0.0;                                       ///< Y 轴位置坐标（米，m）。
  double yaw = 0.0;                                     ///< 航向角（弧度，rad）。
  MotionDirection direction = MotionDirection::Forward; ///< 该位姿对应的车辆运动方向。
};

/**
 * @brief Hybrid A* 路径搜索结果结构体，记录搜索状态、轨迹与统计指标。
 */
struct HybridSearchResult {
  bool success = false;          ///< 搜索是否成功找到到达目标的无碰撞可行路径。
  double cost = 0.0;             ///< 规划轨迹的总累积代价（g 值）。
  int32_t iterations = 0;        ///< 搜索过程中执行的节点外扩迭代次数。
  std::vector<Pose2D> path;      ///< 规划生成的离散位姿路点轨迹序列。
  std::string message;           ///< 终止原因诊断或状态描述信息。
};

/**
 * @brief 基于运动学自行车模型外扩的 Hybrid A* 路径规划引擎。
 */
class HybridAStar {
 public:
  /**
   * @brief 使用指定的规划配置构造 Hybrid A* 规划器实例。
   * @param[in] config 规划器配置参数，包含车辆尺寸、代价权重与搜索边界。
   */
  explicit HybridAStar(const PlannerConfig& config);

  /**
   * @brief 更新规划器的配置参数。
   * @param[in] config 待应用的新配置参数。
   */
  void SetConfig(const PlannerConfig& config);

  /**
   * @brief 获取当前生效的规划器配置参数。
   * @return 当前配置参数的常量引用。
   */
  const PlannerConfig& GetConfig() const;

  /** @name 搜索钩子接口
   *  位姿碰撞、运动碰撞与目标钩子必须设置，否则搜索拒绝运行。
   *  @{
   */

  /**
   * @brief 注册提前中止回调函数，在每次节点外扩迭代开始时调用评估。
   * @param[in] callback 返回 true 表示应立即中止搜索的回调函数。
   */
  void SetAbortCallback(std::function<bool()> callback);

  /**
   * @brief 注册必需的位姿碰撞检测钩子。
   * @param[in] hook 判定给定车辆位姿是否发生碰撞的回调函数（返回 true 表示碰撞）。
   */
  void SetPoseCollisionHook(std::function<bool(const Pose2D&)> hook);

  /**
   * @brief 注册必需的连续运动碰撞检测钩子。
   * @param[in] hook 判定两个位姿间的运动轨迹是否发生碰撞的回调函数（返回 true 表示碰撞）。
   */
  void SetMotionCollisionHook(
      std::function<bool(const Pose2D&, const Pose2D&)> hook);

  /**
   * @brief 注册必需的目标达成判定钩子。
   * @param[in] hook 判定当前位姿是否满足目标条件的回调函数（返回 true 表示达成目标）。
   */
  void SetGoalHook(std::function<bool(const Pose2D&)> hook);

  /** @} */

  /**
   * @brief 执行起点与目标位姿间的 Hybrid A* 路径搜索。
   * @param[in] start 车辆初始位姿。
   * @param[in] goal 车辆目标位姿。
   * @param[out] result 输出参数，存储搜索统计指标及规划轨迹路径。
   * @return 若成功规划出可行路径则返回 true，否则返回 false。
   */
  bool Search(const Pose2D& start, const Pose2D& goal, HybridSearchResult* result);

 private:
  /**
   * @brief 内部三维图搜索状态节点（x, y, yaw）。
   */
  struct Vertex3D {
    uint64_t id = 0;                                       ///< 离散状态唯一哈希标识键。
    uint64_t parent_id = 0;                                ///< 父驱前驱节点的唯一标识键。
    double x = 0.0;                                        ///< 连续 X 轴坐标（米，m）。
    double y = 0.0;                                        ///< 连续 Y 轴坐标（米，m）。
    double yaw = 0.0;                                      ///< 连续航向角（弧度，rad）。
    MotionDirection direction = MotionDirection::Forward;  ///< 进入当前状态时所采用的行驶方向。
    double g = 0.0;                                        ///< 从起点到当前状态的累积实际代价。
    double h = 0.0;                                        ///< 到目标状态的启发式预估代价。
    double f = 0.0;                                        ///< 节点综合优先级评分（f = g + h）。
  };

  /**
   * @brief 优先队列条目，记录节点优先级评分与哈希标识。
   */
  struct OpenEntry {
    double f = 0.0;       ///< 节点综合代价值（f 值）。
    uint64_t id = 0;      ///< 节点唯一哈希标识。
  };

  /**
   * @brief 优先队列最小堆比较仿函数，优先弹出最小 f 值的条目。
   */
  struct OpenEntryGreater {
    bool operator()(const OpenEntry& lhs, const OpenEntry& rhs) const {
      if (lhs.f == rhs.f) {
        return lhs.id > rhs.id;
      }
      return lhs.f > rhs.f;
    }
  };

  bool IsCollision(const Vertex3D& node) const;
  bool IsGoal(const Vertex3D& node, const Pose2D& goal) const;

  uint64_t VertexHash(const Vertex3D& node) const;

  void ExpandNode(const Vertex3D& current, const Pose2D& start, const Pose2D& goal);
  Vertex3D Propagate(const Vertex3D& start, MotionDirection direction, double steering_rad) const;
  double TransitionCost(const Vertex3D& from, const Vertex3D& to) const;
  double Heuristic(const Vertex3D& node, const Pose2D& goal) const;
  bool ReconstructPath(uint64_t final_id, HybridSearchResult* result) const;

  PlannerConfig config_;
  double grid_dist_ = 1.0;
  double grid_angle_rad_ = 0.017453292519943295;

  int32_t angle_bin_count_ = 360;
  Pose2D search_start_pose_;

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
