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
#include <unordered_set>
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

  /**
   * @brief 清除当前已加载的所有障碍物边界点。
   */
  void ClearBoundaries();

  /**
   * @brief 从 CSV 文件加载障碍物多段线边界点。
   * @param[in] csv_path CSV 文件的系统绝对或相对路径。
   * @param[in] close_loop 是否将最后一个顶点与第一个顶点相连以形成闭合环路。
   * @return 解析成功并完成加载返回 true，否则返回 false。
   */
  bool LoadBoundaryFromCsv(const std::string& csv_path, bool close_loop);

  /**
   * @brief 获取当前内部存储的光栅化边界障碍物像元总数。
   * @return 被占用的边界栅格像元数量。
   */
  std::size_t BoundaryCount() const;

  /** @name 搜索钩子与边界注入接口
   *  可选回调函数与可编程边界导入接口。若未设置，则默认采用内部栅格碰撞检测以及欧氏/航向容差判定。
   *  @{
   */

  /**
   * @brief 注册提前中止回调函数，在每次节点外扩迭代开始时调用评估。
   * @param[in] callback 返回 true 表示应立即中止搜索的回调函数。
   */
  void SetAbortCallback(std::function<bool()> callback);

  /**
   * @brief 覆盖默认的栅格位姿碰撞检测逻辑，使用自定义位姿碰撞检测钩子。
   * @param[in] hook 判定给定车辆位姿是否发生碰撞的回调函数（返回 true 表示碰撞）。
   */
  void SetPoseCollisionHook(std::function<bool(const Pose2D&)> hook);

  /**
   * @brief 覆盖默认的节点外扩运动线段碰撞检测逻辑。
   * @param[in] hook 判定两个位姿间的运动轨迹是否发生碰撞的回调函数（返回 true 表示碰撞）。
   */
  void SetMotionCollisionHook(
      std::function<bool(const Pose2D&, const Pose2D&)> hook);

  /**
   * @brief 覆盖默认的距离与航向角目标达成判定准则。
   * @param[in] hook 判定当前位姿是否满足目标条件的回调函数（返回 true 表示达成目标）。
   */
  void SetGoalHook(std::function<bool(const Pose2D&)> hook);

  /**
   * @brief 以编程方式将世界坐标系下的障碍物点光栅化导入内部边界栅格。
   * @param[in] x 世界坐标系 X 坐标（米，m）。
   * @param[in] y 世界坐标系 Y 坐标（米，m）。
   */
  void AddBoundaryWorldPoint(double x, double y);

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

  /**
   * @brief 二维离散整数栅格索引。
   */
  struct GridPoint {
    int32_t x = 0;  ///< 栅格列索引。
    int32_t y = 0;  ///< 栅格行索引。
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
