#pragma once

// 公开碰撞检测接口（Docs/component_development_plan.md §6.1/§6.2）：
// check_pose / check_motion / check_path 返回车辆足迹是否碰撞、首个接触
// 位姿以及原因。
//
// 此处实现的共识规则：
//   * 接触（相切）计为碰撞；
//   * 未知栅格单元与地图矩形之外的一切区域均视为占据；
//   * 运动分段使用自适应细分与保守的位移包络进行检查；当包络无法证明
//     无碰撞时，检查器继续细分，而在整个车辆精度下限以下（车辆任一点的
//     位移上限 1e-3 m，即 §7.1 默认值），未解决的区间报告为碰撞
//     （保守）。
//
// check_path 还会在相邻路径点之间扫掠：每个过渡段是由首个路径点存储的
// 曲率、弧长差与分段方向重建的常曲率圆弧；存储的点位姿始终精确检查。
// 仅检查离散路径点永远不够（必须发现采样点之间的薄障碍物）。

#include <memory>
#include <string>
#include <vector>

#include "planner/collision/scene.hpp"

namespace planner::collision {

using planner::common::PathSegment;

enum class CheckStatus {
  kClear,        // 未发现碰撞
  kCollision,    // 碰撞（含接触）；见 reason_code/detail
  kInvalidInput  // 输入被拒绝；无判定结果
};

enum class CollisionReason {
  kNone,
  kObstacle,       // 多边形障碍物重叠
  kGridOccupied,   // 占据栅格单元重叠
  kGridUnknown,    // 未知栅格单元重叠（视为占据）
  kOutsideMap,     // 足迹的一部分位于栅格地图之外
  kPrecisionFloor, // 运动包络在扫掠精度下限处未解决
  kInvalidInput
};

const char* to_string(CheckStatus status) noexcept;
const char* to_string(CollisionReason reason) noexcept;

// 单次碰撞检测的结果。`first_pose` 是首个检测到接触的后轴中心位姿：
//   * check_pose：被检查的位姿，first_s_m = 0；
//   * check_motion：圆弧上距起点弧长为 first_s_m 的位姿；
//   * check_path：失败路径分段上的位姿；first_s_m 是从该分段起点算起的
//     弧长，path_segment_index 标识该分段（从 0 开始）。
struct CheckResult {
  CheckStatus status{CheckStatus::kClear};
  CollisionReason reason_code{CollisionReason::kNone};
  std::string detail;
  Pose2d first_pose;
  double first_s_m{0.0};
  int path_segment_index{-1};

  bool collided() const noexcept { return status == CheckStatus::kCollision; }
  bool is_clear() const noexcept { return status == CheckStatus::kClear; }
  bool is_valid() const noexcept { return status != CheckStatus::kInvalidInput; }
};

// 所有检查共用的选项。
struct CheckOptions {
  // 细分下限：单个扫掠区间内车辆任一点的最大位移（米）。§7.1 默认
  // 1e-3 m。
  double sweep_precision_m{1e-3};

  // 可选的包围体 / 空间索引加速。默认禁用：纯几何路径是可信基准。启用后
  // 结果与纯路径完全一致（由差分对比测试验证），因为每个被跳过的对象都
  // 可证明比查询包络更远。
  bool enable_acceleration{false};
};

// 常曲率运动基元：起始位姿、弧长、挡位与带符号曲率。
// yaw_change = curvature * signed_length，其中前进时
// signed_length = +length、倒车时 -length（按 v1 约定，
// yaw_rate = signed_speed * curvature）。
struct MotionSegment {
  Pose2d start;
  double length_m{0.0};
  Direction direction{Direction::kForward};
  double curvature_inv_m{0.0};
};

// 检查单个位姿。
CheckResult check_pose(const Pose2d& pose, const VehicleModel& vehicle,
                       const Scene& scene, const CheckOptions& options = {});

// 检查一个常曲率运动分段。
CheckResult check_motion(const MotionSegment& segment, const VehicleModel& vehicle,
                         const Scene& scene, const CheckOptions& options = {});

// 检查一个方向一致的路径分段。
CheckResult check_path(const PathSegment& segment, const VehicleModel& vehicle,
                       const Scene& scene, const CheckOptions& options = {});

// 检查有序的路径分段列表。结果中的 `path_segment_index` 标识失败的分段。
CheckResult check_path(const std::vector<PathSegment>& path,
                       const VehicleModel& vehicle, const Scene& scene,
                       const CheckOptions& options = {});

// 不可变检查器状态的前向声明（定义在 src/collision_checker.cpp）。
struct CollisionCheckerImpl;

// 可复用的检查器：对场景验证一次，并可选地预计算加速结构。每个规划器
// 实例持有自己的检查器；检查器使用期间不得修改输入快照。
class CollisionChecker {
 public:
  CollisionChecker(const VehicleModel& vehicle, const Scene& scene,
                   CheckOptions options = {});

  // 构造函数拒绝输入时为 false；`reason` 说明原因，且所有检查都返回
  // kInvalidInput。
  bool valid() const noexcept;
  const std::string& invalid_reason() const noexcept;

  const VehicleModel& vehicle() const noexcept;
  const Scene& scene() const noexcept;
  const CheckOptions& options() const noexcept;

  CheckResult check_pose(const Pose2d& pose) const;
  CheckResult check_motion(const MotionSegment& segment) const;
  CheckResult check_path(const PathSegment& segment) const;
  CheckResult check_path(const std::vector<PathSegment>& path) const;

 private:
  std::shared_ptr<const CollisionCheckerImpl> state_;
};

}  // namespace planner::collision
