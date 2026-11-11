/**
 * @file search_engine.hpp
 * @brief 针对类车（Car-like）机器人的内部 Hybrid A* 动力学搜索引擎头文件
 */

#pragma once

#include <ankerl/unordered_dense.h>

#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <string>
#include <vector>

#include "planner/common/path.hpp"
#include "planner/common/planner_config.hpp"
#include "planner/common/status.hpp"
#include "planner/common/vehicle.hpp"
#include "planner/math/pose2d.hpp"

namespace planner::hybrid_astar::detail {

/// 无效节点标识：根节点的父驱标识，同时用作路径重建的终止哨兵
inline constexpr std::uint64_t kInvalidNodeId = std::numeric_limits<std::uint64_t>::max();

/**
 * @brief 内部搜索状态：连续位姿、行驶方向以及进入该状态的常曲率运动基元参数
 */
struct SearchState {
  math::Pose2d pose;                                         ///< 车辆后轴中心位姿
  common::Direction direction{common::Direction::kForward};  ///< 进入当前状态时所采用的行驶方向
  double curvature_per_m{0.0};  ///< 从父驱状态进入当前状态的路径曲率（1/m）
  double arc_length_m{0.0};     ///< 从父驱状态进入当前状态的圆弧弧长（米，m）
};

/**
 * @brief Hybrid A* 搜索结果，记录搜索状态、轨迹与统计指标
 */
struct SearchResult {
  common::Status status{common::Status::kInvalidInput};  ///< 搜索终止状态码
  std::string reason;             ///< 终止原因诊断或状态描述信息
  double cost{0.0};               ///< 规划轨迹的总累积代价（g 值）
  std::int64_t iterations{0};     ///< 搜索过程中执行的节点外扩迭代次数
  std::vector<SearchState> path;  ///< 规划生成的离散状态轨迹序列
};

/**
 * @brief 基于运动学自行车模型外扩的 Hybrid A* 路径搜索引擎
 */
class SearchEngine {
 public:
  /**
   * @brief 使用车辆运动学模型与 SI 单位规划配置构造搜索引擎实例
   * @param[in] vehicle 车辆运动学模型参数
   * @param[in] config 规划器 SI 单位配置参数
   */
  explicit SearchEngine(const common::VehicleModel& vehicle, const common::UserAstarConfig& config);

  /**
   * @brief 更新搜索引擎的车辆模型与规划配置参数
   * @param[in] vehicle 待应用的车辆运动学模型参数
   * @param[in] config 待应用的新配置参数
   */
  void configure(const common::VehicleModel& vehicle, const common::UserAstarConfig& config);

  /** @name 搜索钩子接口
   *  位姿碰撞、运动碰撞与目标钩子必须设置，否则搜索拒绝运行
   *  @{
   */

  /**
   * @brief 注册提前中止回调函数，在每次节点外扩迭代开始时调用评估
   * @param[in] callback 返回 true 表示应立即中止搜索的回调函数
   */
  void set_abort_callback(std::function<bool()> callback);

  /**
   * @brief 注册必需的位姿碰撞检测钩子
   * @param[in] hook 判定给定车辆位姿是否发生碰撞的回调函数（返回 true 表示碰撞）
   */
  void set_pose_collision_hook(std::function<bool(const math::Pose2d&)> hook);

  /**
   * @brief 注册必需的连续运动碰撞检测钩子
   * @param[in] hook 判定父驱状态到后继状态的运动轨迹是否发生碰撞的回调函数
   *             （返回 true 表示碰撞）
   */
  void set_motion_collision_hook(
      std::function<bool(const SearchState& from, const SearchState& to)> hook);

  /**
   * @brief 注册必需的目标达成判定钩子
   * @param[in] hook 判定当前状态是否满足目标条件的回调函数（返回 true 表示达成目标）
   */
  void set_goal_hook(std::function<bool(const SearchState&)> hook);

  /**
   * @brief 清空全部搜索钩子，避免复用引擎时残留上一轮规划的捕获引用
   */
  void clear_hooks();

  /** @} */

  /**
   * @brief 执行起点与目标位姿间的 Hybrid A* 路径搜索
   * @param[in] start 车辆初始位姿
   * @param[in] goal 车辆目标位姿
   * @param[in] node_budget 允许执行的最大节点外扩迭代次数
   * @return 包含终止状态、规划轨迹与统计指标的搜索结果对象
   */
  SearchResult search(const math::Pose2d& start, const math::Pose2d& goal,
                      std::int64_t node_budget);

  /**
   * @brief 清空开/闭集合、最优节点表与优先队列，但保留容器已分配的容量
   */
  void reset_state();

 private:
  /**
   * @brief 内部三维图搜索状态节点（x, y, yaw）
   */
  struct Vertex3D {
    std::uint64_t id{kInvalidNodeId};         ///< 离散状态唯一哈希标识键
    std::uint64_t parent_id{kInvalidNodeId};  ///< 父驱前驱节点的唯一标识键
    SearchState state;  ///< 连续位姿、挡位与进入该状态的常曲率运动基元参数
    double g{0.0};      ///< 从起点到当前状态的累积实际代价
    double h{0.0};      ///< 到目标状态的启发式预估代价
    double f{0.0};      ///< 节点综合优先级评分（f = g + h）
  };

  /**
   * @brief 优先队列条目，记录节点优先级评分与哈希标识
   */
  struct OpenEntry {
    double f{0.0};        ///< 节点综合代价值（f 值）
    std::uint64_t id{0};  ///< 节点唯一哈希标识
  };

  /**
   * @brief 优先队列最小堆比较仿函数，优先弹出最小 f 值的条目
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
   * @brief 支持保留底层存储容量的最小堆优先队列
   */
  class OpenQueue
      : public std::priority_queue<OpenEntry, std::vector<OpenEntry>, OpenEntryGreater> {
   public:
    /// 清空队列元素但保留 vector 已分配的容量
    void clear() { this->c.clear(); }
  };

  bool is_collision(const Vertex3D& node) const;
  bool is_goal(const Vertex3D& node) const;

  std::uint64_t vertex_hash(const Vertex3D& node) const;

  void expand_node(const Vertex3D& current, const math::Pose2d& start, const math::Pose2d& goal);
  Vertex3D propagate(const Vertex3D& start, common::Direction direction, double curvature_per_m,
                     double cos_yaw, double sin_yaw) const;
  double transition_cost(const Vertex3D& from, const Vertex3D& to, double coeff) const;
  double heuristic(const Vertex3D& node, const math::Pose2d& goal) const;
  bool reconstruct_path(std::uint64_t final_id, std::vector<SearchState>* path) const;

  common::VehicleModel vehicle_;
  common::UserAstarConfig config_;
  std::vector<double> curvature_samples_;
  double heading_bin_rad_{0.0};

  ankerl::unordered_dense::map<std::uint64_t, Vertex3D> open_set_;
  ankerl::unordered_dense::set<std::uint64_t> closed_set_;
  ankerl::unordered_dense::map<std::uint64_t, Vertex3D> best_nodes_;
  OpenQueue open_queue_;

  std::function<bool()> abort_callback_;
  std::function<bool(const math::Pose2d&)> pose_collision_hook_;
  std::function<bool(const SearchState&, const SearchState&)> motion_collision_hook_;
  std::function<bool(const SearchState&)> goal_hook_;
};

}  // namespace planner::hybrid_astar::detail
