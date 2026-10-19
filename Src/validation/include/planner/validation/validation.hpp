#pragma once

// 轨迹验证 V（Docs/component_development_plan.md 第 2、6.2 和 7.1 节）：
// 对采样轨迹做独立于优化器的后验检查。
//
// 输入：轨迹（公开类型 planner::common::TrajectoryPoint 序列）、车辆及其
// 限制、地图快照（planner::collision::Scene：只读 GridView 加凸障碍物）
// 与安全走廊（planner::common::Corridor）。必需输入缺失或不可用会返回
// `common::Status::kInvalidInput`；任何约束违规都会返回
// `common::Status::kValidationFailed`，并附带首次失败时间、被违反的约束
// 以及实际值/限制值对。
//
// 检查的约束（用于处理同一时间并列的固定顺序）：
//   * 每个存储值有限；时间严格递增；
//   * 换向必须经过零速，|v| <= 1e-3 m/s；
//   * 速度/加速度/减速度/曲率限制，数值容差 1e-6 * max(1, limit)；
//   * 沿轨迹稠密重采样，检查整车足迹的走廊包含性（绝不只检查中心线）；
//   * 沿轨迹的静态碰撞：每个存储位姿都精确检查，每相邻一对用由轨迹
//     重建的 MotionSegment（长度/方向/曲率）扫掠，复用
//     planner::collision 的 check_motion；仅有离散位姿永远不够；
//   * 端点位置 <= 0.05 m、航向 <= 0.02 rad（路径标准）。
//
// 报告按时间取首次失败：每项检查报告其最早的违规时刻，全局结果保留
// 最小时间；时间完全并列时按上述检查顺序解决。组件绝不放松硬安全约束：
// 违规一律报告为无效，只有在对应输入缺失时才能省略检查，而这本身也是
// `invalid_input` 失败。
//
// 该组件独立于任何优化器（不依赖 Dftpav），只复用公开的
// planner::collision / planner::corridor 接口。

#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "planner/collision/collision_checker.hpp"
#include "planner/common/types.hpp"

namespace planner::validation {

// 来自 Docs/component_development_plan.md 第 7.1 节的固定验收容差。
// 它们是组件契约的一部分，不可配置。
inline constexpr double kEndpointPositionToleranceM = 0.05;
inline constexpr double kEndpointHeadingToleranceRad = 0.02;
inline constexpr double kDirectionChangeSpeedToleranceMps = 1e-3;

// 应用于速度/加速度/曲率限制的数值容差：
// allowed = limit + kLimitRelativeTolerance * max(1, limit)。
inline constexpr double kLimitRelativeTolerance = 1e-6;

// ValidationResult::reason 与 ValidationViolation::constraint 使用的稳定
// 约束名（to_string 返回规范字符串 "input"、"finite_values"、
// "time_monotonicity"、"direction_change_stop"、"speed_limit"、
// "acceleration_limit"、"deceleration_limit"、"curvature_limit"、
// "endpoint_position"、"endpoint_heading"、"corridor_containment" 和
// "collision"；kNone 映射为 "none"）。
enum class Constraint {
  kNone,
  kInput,                 // 必需输入无效或缺失
  kFiniteValues,          // 存储的轨迹值含 NaN / Infinity
  kTimeMonotonicity,      // t_s 非严格递增
  kDirectionChangeStop,   // 换挡未经过零速
  kSpeedLimit,            // |velocity_mps| 超过 max_speed_mps
  kAccelerationLimit,     // acceleration_mps2 超过 max_acceleration_mps2
  kDecelerationLimit,     // acceleration_mps2 低于 -max_deceleration_mps2
  kCurvatureLimit,        // |curvature_inv_m| 超过 max_curvature_inv_m
  kEndpointPosition,      // 最终位姿距目标超过 0.05 m
  kEndpointHeading,       // 最终航向距目标超过 0.02 rad
  kCorridorContainment,   // 整车不在覆盖的走廊多边形内
  kCollision,             // 静态碰撞（位姿或重建运动）
};

const char* to_string(Constraint constraint) noexcept;

// 验证器要求的车辆动力学限制。所有值必须有限且严格为正；限制为幅值
// （减速度应用于有符号加速度的负侧）。
struct VehicleLimits {
  double max_speed_mps{0.0};
  double max_acceleration_mps2{0.0};
  double max_deceleration_mps2{0.0};
  double max_curvature_inv_m{0.0};
};

// 完整的验证输入。trajectory / vehicle / limits 是普通值；
// goal / scene / corridor 为可选项，使缺失的必需输入可以被表示并报告为
// `invalid_input`，而不是被静默跳过。无栅格的场景（width == height == 0）
// 是有效的自由空间场景；空走廊永远无法覆盖非空轨迹，会包含性检查失败。
struct ValidationRequest {
  std::vector<common::TrajectoryPoint> trajectory;  // 必需，非空
  common::VehicleModel vehicle;                     // 必需，足迹有效
  VehicleLimits limits;                             // 必需，全部 > 0
  std::optional<common::Pose2d> goal;               // 必需，有限
  std::optional<collision::Scene> scene;            // 必需，结构有效
  std::optional<common::Corridor> corridor;         // 必需，结构有效
};

// 一处检测到的违规。约束没有数值对（如碰撞）时 `actual` / `limit` 为 NaN；
// 失败与轨迹采样点无关（所有输入错误）时 `time_s` 为 NaN。
struct ValidationViolation {
  Constraint constraint{Constraint::kNone};
  double time_s{std::numeric_limits<double>::quiet_NaN()};
  int point_index{-1};
  double actual{std::numeric_limits<double>::quiet_NaN()};
  double limit{std::numeric_limits<double>::quiet_NaN()};
  std::string detail;
};

// 不改变各项检查自身容差的选项。
struct ValidationOptions {
  // 走廊包含性检查的稠密重采样步长（米）。
  double corridor_sample_step_m{0.02};
  // 走廊包含允许的数值亏空（米）。仅作浮点保护量，比安全裕度小几个
  // 数量级。
  double containment_tolerance_m{1e-6};
  // 原样转发给 planner::collision 检查的选项。
  collision::CheckOptions collision;
};

// 验证判定。`valid` 仅在 Status::kSuccess 时为 true；每个缺失输入映射为
// Status::kInvalidInput，每个约束违规映射为 Status::kValidationFailed。
// `reason` 是稳定代码（"valid"、如 "missing_goal" 的缺失输入代码，或被
// 违反的约束名），`detail` 为人类可读文本。
struct ValidationResult {
  common::Status status{common::Status::kInvalidInput};
  bool valid{false};
  std::string reason{"missing_input"};
  std::string detail;
  ValidationViolation first_failure;

  // 实际完成的工作量（失败结果也会填充）。
  std::size_t trajectory_points{0};
  std::size_t corridor_samples{0};
  std::size_t collision_pose_checks{0};
  std::size_t collision_motion_checks{0};

  bool ok() const noexcept { return valid; }
};

// 验证轨迹。必需但缺失的输入报告为 Status::kInvalidInput（reason 为
// "missing_trajectory"、"invalid_vehicle"、"invalid_limits"、"missing_goal"、
// "invalid_goal"、"missing_scene"、"invalid_scene"、"missing_corridor"、
// "invalid_corridor"、"map_mismatch"、"invalid_options" 或
// "invalid_motion_reconstruction"）；约束违规报告为
// Status::kValidationFailed，reason 等于 to_string(first_failure.constraint)。
ValidationResult validate_trajectory(const ValidationRequest& request,
                                     const ValidationOptions& options = {});

}  // namespace planner::validation
