// 轨迹验证 V 实现。
//
// 验证器按固定、确定的顺序执行 validation.hpp 中记录的检查。属于结构性
// 前提的检查（输入有效性、有限值、时间严格递增）会立即失败；其余检查
// 全部执行，各自报告其最早的违规时刻，全局结果保留最小时间（完全并列时
// 保留文档顺序中靠前的检查）。碰撞检测复用 planner::collision
// （CollisionChecker），走廊包含性测试复用 planner::corridor 几何；
// 不涉及任何优化器代码。

#include "planner/validation/validation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "planner/collision/geometry.hpp"
#include "planner/corridor/geometry.hpp"

namespace planner::validation {

namespace {

using common::Corridor;
using common::Direction;
using common::kPi;
using common::normalize_angle_rad;
using common::Point2d;
using common::Pose2d;
using common::TrajectoryPoint;

using collision::CheckStatus;
using collision::CollisionChecker;
using collision::MotionSegment;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// 走廊 s 覆盖闭端测试的容差：最后一个轨迹采样点可以正好落在多边形的
// s_end_m 上（走廊区间其余情况下为半开区间 [s_start_m, s_end_m)）。
constexpr double kCorridorSCoverageEps = 1e-9;

// 相邻位姿距离小于该值时视为零长度步；位姿仍会被精确检查。
constexpr double kMinChordM = 1e-12;

// planner::collision 支持 |curvature| <= 1e6 1/m（半径 >= 1 um）。
// 重建出的更弯弧段长度不足 4 微米，由两端点的精确位姿检查覆盖。
constexpr double kMaxReconstructedCurvatureInvM = 1.0e6;

double limit_slack(double limit) {
  return kLimitRelativeTolerance * std::max(1.0, limit);
}

std::string format_double(double value) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.12g", value);
  return std::string(buffer);
}

ValidationResult invalid_result(const std::string& reason,
                                const std::string& detail) {
  ValidationResult result;
  result.status = common::Status::kInvalidInput;
  result.valid = false;
  result.reason = reason;
  result.detail = detail;
  result.first_failure.constraint = Constraint::kInput;
  return result;
}

ValidationResult failed_result(const ValidationViolation& violation) {
  ValidationResult result;
  result.status = common::Status::kValidationFailed;
  result.valid = false;
  result.reason = to_string(violation.constraint);
  result.detail = violation.detail;
  result.first_failure = violation;
  return result;
}

bool pose_finite(const Pose2d& pose) {
  return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) &&
         std::isfinite(pose.yaw_rad);
}

const char* non_finite_trajectory_field(const TrajectoryPoint& point) {
  if (!std::isfinite(point.t_s)) return "t_s";
  if (!std::isfinite(point.pose.x_m)) return "pose.x_m";
  if (!std::isfinite(point.pose.y_m)) return "pose.y_m";
  if (!std::isfinite(point.pose.yaw_rad)) return "pose.yaw_rad";
  if (!std::isfinite(point.velocity_mps)) return "velocity_mps";
  if (!std::isfinite(point.acceleration_mps2)) return "acceleration_mps2";
  if (!std::isfinite(point.curvature_inv_m)) return "curvature_inv_m";
  return nullptr;
}

std::string limits_error(const VehicleLimits& limits) {
  const auto positive = [](double value) {
    return std::isfinite(value) && value > 0.0;
  };
  if (!positive(limits.max_speed_mps)) {
    return "max_speed_mps must be finite and > 0";
  }
  if (!positive(limits.max_acceleration_mps2)) {
    return "max_acceleration_mps2 must be finite and > 0";
  }
  if (!positive(limits.max_deceleration_mps2)) {
    return "max_deceleration_mps2 must be finite and > 0";
  }
  if (!positive(limits.max_curvature_inv_m)) {
    return "max_curvature_inv_m must be finite and > 0";
  }
  return "";
}

std::string corridor_structure_error(const Corridor& corridor) {
  if (corridor.id.empty()) return "corridor id is empty";
  if (corridor.map_id.empty()) return "corridor map_id is empty";
  if (corridor.map_version.empty()) return "corridor map_version is empty";
  if (corridor.frame_id.empty()) return "corridor frame_id is empty";
  double previous_start = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < corridor.polygons.size(); ++i) {
    const common::CorridorPolygon& polygon = corridor.polygons[i];
    const std::string prefix = "polygon " + std::to_string(i) + ": ";
    if (!std::isfinite(polygon.s_start_m) || !(polygon.s_start_m >= 0.0)) {
      return prefix + "s_start_m must be finite and >= 0";
    }
    if (!std::isfinite(polygon.s_end_m) ||
        !(polygon.s_end_m > polygon.s_start_m)) {
      return prefix + "s_end_m must be finite and greater than s_start_m";
    }
    if (polygon.s_start_m < previous_start) {
      return prefix + "polygons must be ordered by non-decreasing s_start_m";
    }
    previous_start = polygon.s_start_m;
    if (!corridor::is_convex_ccw_polygon(polygon.vertices)) {
      return prefix +
             "first-version corridor polygons must be strictly convex, "
             "counter-clockwise, finite and have at least three vertices";
    }
  }
  return "";
}

// 对一对相邻轨迹点做常曲率圆弧重建。`arc_m` 为弧长（>= 弦长），
// `direction` 选择连接两个存储位姿的分支（由几何驱动，因此即使输入
// 不一致，扫掠弧也总能覆盖实际几何路径），`curvature_inv_m` 带符号，使
// yaw_change = curvature * signed_length，其中
// signed_length = +arc（前进）/ -arc（倒车）。零长度或低于容差（比
// 1e6 1/m 更弯）的步 `valid == false`；此时两个存储位姿仍总是被精确检查。
struct ReconstructedMotion {
  bool valid{false};
  double arc_m{0.0};
  Direction direction{Direction::kForward};
  double curvature_inv_m{0.0};
};

double sinc(double x) {
  if (std::abs(x) < 1e-8) {
    const double x2 = x * x;
    return 1.0 - x2 / 6.0 + x2 * x2 / 120.0;
  }
  return std::sin(x) / x;
}

Pose2d pose_on_arc(const Pose2d& start, double sigma, double kappa) {
  const double b = kappa * sigma;
  const double cy = std::cos(start.yaw_rad);
  const double sy = std::sin(start.yaw_rad);
  const double half = 0.5 * b;
  const double sb = sinc(b);
  const double h = half * sinc(half) * sinc(half);
  const double dsin = cy * sb - sy * h;   // (sin(y+b) - sin y) / b
  const double dcos = -sy * sb - cy * h;  // (cos(y+b) - cos y) / b
  Pose2d out;
  out.x_m = start.x_m + sigma * dsin;
  out.y_m = start.y_m - sigma * dcos;
  out.yaw_rad = normalize_angle_rad(start.yaw_rad + b);
  return out;
}

ReconstructedMotion reconstruct_motion(const Pose2d& from, const Pose2d& to) {
  ReconstructedMotion motion;
  const double dx = to.x_m - from.x_m;
  const double dy = to.y_m - from.y_m;
  const double chord = std::hypot(dx, dy);
  if (!(chord > kMinChordM)) {
    return motion;  // 零长度步
  }
  const double dtheta = normalize_angle_rad(to.yaw_rad - from.yaw_rad);
  double arc = chord;
  if (std::abs(dtheta) > 1e-12) {
    const double half = 0.5 * dtheta;
    arc = chord * half / std::sin(half);
  }
  const double chord_angle = std::atan2(dy, dx);
  const double forward_error =
      normalize_angle_rad(chord_angle - (from.yaw_rad + 0.5 * dtheta));
  const Direction direction = std::abs(forward_error) <= 0.5 * kPi
                                  ? Direction::kForward
                                  : Direction::kReverse;
  const double sigma = direction == Direction::kForward ? arc : -arc;
  const double curvature = dtheta / sigma;
  if (!(arc > 0.0) || !std::isfinite(curvature) ||
      std::abs(curvature) > kMaxReconstructedCurvatureInvM) {
    return motion;
  }
  motion.valid = true;
  motion.arc_m = arc;
  motion.direction = direction;
  motion.curvature_inv_m = curvature;
  return motion;
}

}  // namespace

const char* to_string(Constraint constraint) noexcept {
  switch (constraint) {
    case Constraint::kNone:
      return "none";
    case Constraint::kInput:
      return "input";
    case Constraint::kFiniteValues:
      return "finite_values";
    case Constraint::kTimeMonotonicity:
      return "time_monotonicity";
    case Constraint::kDirectionChangeStop:
      return "direction_change_stop";
    case Constraint::kSpeedLimit:
      return "speed_limit";
    case Constraint::kAccelerationLimit:
      return "acceleration_limit";
    case Constraint::kDecelerationLimit:
      return "deceleration_limit";
    case Constraint::kCurvatureLimit:
      return "curvature_limit";
    case Constraint::kEndpointPosition:
      return "endpoint_position";
    case Constraint::kEndpointHeading:
      return "endpoint_heading";
    case Constraint::kCorridorContainment:
      return "corridor_containment";
    case Constraint::kCollision:
      return "collision";
  }
  return "none";
}

ValidationResult validate_trajectory(const ValidationRequest& request,
                                     const ValidationOptions& options) {
  const std::vector<TrajectoryPoint>& trajectory = request.trajectory;

  std::size_t corridor_samples = 0;
  std::size_t collision_pose_checks = 0;
  std::size_t collision_motion_checks = 0;
  const auto finalize = [&](ValidationResult result) {
    result.trajectory_points = trajectory.size();
    result.corridor_samples = corridor_samples;
    result.collision_pose_checks = collision_pose_checks;
    result.collision_motion_checks = collision_motion_checks;
    return result;
  };

  // -------------------------------------------------------------------------
  // 1. 必需输入与选项。
  // -------------------------------------------------------------------------
  if (!std::isfinite(options.corridor_sample_step_m) ||
      !(options.corridor_sample_step_m > 0.0)) {
    return finalize(invalid_result("invalid_options",
                                   "corridor_sample_step_m must be finite "
                                   "and > 0"));
  }
  if (!std::isfinite(options.containment_tolerance_m) ||
      !(options.containment_tolerance_m >= 0.0)) {
    return finalize(invalid_result("invalid_options",
                                   "containment_tolerance_m must be finite "
                                   "and >= 0"));
  }
  if (!std::isfinite(options.collision.sweep_precision_m) ||
      !(options.collision.sweep_precision_m > 0.0)) {
    return finalize(invalid_result(
        "invalid_options", "collision.sweep_precision_m must be finite and > 0"));
  }
  if (trajectory.empty()) {
    return finalize(invalid_result(
        "missing_trajectory",
        "trajectory is a required input and must contain at least one point"));
  }
  {
    std::string reason;
    if (!collision::vehicle_footprint_valid(request.vehicle, &reason)) {
      return finalize(invalid_result("invalid_vehicle", reason));
    }
  }
  {
    const std::string reason = limits_error(request.limits);
    if (!reason.empty()) {
      return finalize(invalid_result("invalid_limits", reason));
    }
  }
  if (!request.goal.has_value()) {
    return finalize(invalid_result(
        "missing_goal", "goal is a required input for the endpoint check"));
  }
  if (!pose_finite(*request.goal)) {
    return finalize(invalid_result("invalid_goal",
                                   "goal pose must be finite (x_m, y_m, yaw_rad)"));
  }
  if (!request.scene.has_value()) {
    return finalize(invalid_result(
        "missing_scene",
        "scene (GridView and/or obstacles) is a required input for the "
        "collision check; use a scene without a grid for free space"));
  }
  {
    std::string reason;
    if (!request.scene->valid(&reason)) {
      return finalize(invalid_result("invalid_scene", reason));
    }
  }
  if (!request.corridor.has_value()) {
    return finalize(invalid_result(
        "missing_corridor",
        "corridor is a required input for the containment check"));
  }
  {
    const std::string reason = corridor_structure_error(*request.corridor);
    if (!reason.empty()) {
      return finalize(invalid_result("invalid_corridor", reason));
    }
  }
  if (request.scene->has_grid()) {
    const common::GridView& grid = request.scene->grid;
    const Corridor& corridor = *request.corridor;
    if (!grid.map_id.empty() && corridor.map_id != grid.map_id) {
      return finalize(invalid_result(
          "map_mismatch", "corridor map_id '" + corridor.map_id +
                              "' does not match scene grid map_id '" +
                              grid.map_id + "'"));
    }
    if (!grid.map_version.empty() && corridor.map_version != grid.map_version) {
      return finalize(invalid_result(
          "map_mismatch", "corridor map_version '" + corridor.map_version +
                              "' does not match scene grid map_version '" +
                              grid.map_version + "'"));
    }
  }

  // -------------------------------------------------------------------------
  // 2. 有限值（后续所有比较的前提）。
  // -------------------------------------------------------------------------
  for (std::size_t i = 0; i < trajectory.size(); ++i) {
    const char* field = non_finite_trajectory_field(trajectory[i]);
    if (field != nullptr) {
      ValidationViolation violation;
      violation.constraint = Constraint::kFiniteValues;
      violation.time_s = std::isfinite(trajectory[i].t_s) ? trajectory[i].t_s
                                                          : kNaN;
      violation.point_index = static_cast<int>(i);
      violation.actual = kNaN;
      violation.limit = kNaN;
      violation.detail = "trajectory point " + std::to_string(i) +
                         " has a non-finite value in field " + field;
      return finalize(failed_result(violation));
    }
  }

  // -------------------------------------------------------------------------
  // 3. 时间严格递增（按时间顺序报告的前提）。
  // -------------------------------------------------------------------------
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    if (!(trajectory[i].t_s > trajectory[i - 1].t_s)) {
      ValidationViolation violation;
      violation.constraint = Constraint::kTimeMonotonicity;
      violation.time_s = trajectory[i].t_s;
      violation.point_index = static_cast<int>(i);
      violation.actual = trajectory[i].t_s;
      violation.limit = trajectory[i - 1].t_s;
      violation.detail =
          "t_s must be strictly increasing: point " + std::to_string(i) +
          " has t_s = " + format_double(trajectory[i].t_s) +
          " after t_s = " + format_double(trajectory[i - 1].t_s);
      return finalize(failed_result(violation));
    }
  }

  // -------------------------------------------------------------------------
  // 4. 其余检查。每项报告其最早的违规时刻；结果保留最小时间（完全并列时
  //    保留本顺序中靠前的检查）。
  // -------------------------------------------------------------------------
  std::optional<ValidationViolation> first_failure;
  const auto consider = [&first_failure](ValidationViolation violation) {
    if (!first_failure.has_value() ||
        violation.time_s < first_failure->time_s) {
      first_failure = std::move(violation);
    }
  };

  // 4a. 换向必须经过零速（|v| <= 1e-3 m/s）。
  for (std::size_t i = 0; i + 1 < trajectory.size(); ++i) {
    const double v0 = trajectory[i].velocity_mps;
    const double v1 = trajectory[i + 1].velocity_mps;
    const bool opposite = (v0 > 0.0 && v1 < 0.0) || (v0 < 0.0 && v1 > 0.0);
    if (!opposite) {
      continue;
    }
    const double closest_to_zero = std::min(std::abs(v0), std::abs(v1));
    if (closest_to_zero > kDirectionChangeSpeedToleranceMps) {
      ValidationViolation violation;
      violation.constraint = Constraint::kDirectionChangeStop;
      violation.time_s = trajectory[i + 1].t_s;
      violation.point_index = static_cast<int>(i + 1);
      violation.actual = closest_to_zero;
      violation.limit = kDirectionChangeSpeedToleranceMps;
      violation.detail =
          "gear change between trajectory points " + std::to_string(i) +
          " and " + std::to_string(i + 1) +
          " without passing through zero speed: min(|v|) = " +
          format_double(closest_to_zero) + " m/s > " +
          format_double(kDirectionChangeSpeedToleranceMps) + " m/s";
      consider(std::move(violation));
      break;
    }
  }

  // 4b. 速度限制。
  {
    const double limit = request.limits.max_speed_mps;
    const double allowed = limit + limit_slack(limit);
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
      const double speed = std::abs(trajectory[i].velocity_mps);
      if (speed > allowed) {
        ValidationViolation violation;
        violation.constraint = Constraint::kSpeedLimit;
        violation.time_s = trajectory[i].t_s;
        violation.point_index = static_cast<int>(i);
        violation.actual = speed;
        violation.limit = limit;
        violation.detail = "|velocity_mps| = " + format_double(speed) +
                           " m/s at point " + std::to_string(i) +
                           " exceeds max_speed_mps = " + format_double(limit);
        consider(std::move(violation));
        break;
      }
    }
  }

  // 4c. 加速度限制。
  {
    const double limit = request.limits.max_acceleration_mps2;
    const double allowed = limit + limit_slack(limit);
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
      const double accel = trajectory[i].acceleration_mps2;
      if (accel > allowed) {
        ValidationViolation violation;
        violation.constraint = Constraint::kAccelerationLimit;
        violation.time_s = trajectory[i].t_s;
        violation.point_index = static_cast<int>(i);
        violation.actual = accel;
        violation.limit = limit;
        violation.detail = "acceleration_mps2 = " + format_double(accel) +
                           " m/s^2 at point " + std::to_string(i) +
                           " exceeds max_acceleration_mps2 = " +
                           format_double(limit);
        consider(std::move(violation));
        break;
      }
    }
  }

  // 4d. 减速度限制（有符号加速度低于 -limit）。
  {
    const double limit = request.limits.max_deceleration_mps2;
    const double allowed = -(limit + limit_slack(limit));
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
      const double accel = trajectory[i].acceleration_mps2;
      if (accel < allowed) {
        ValidationViolation violation;
        violation.constraint = Constraint::kDecelerationLimit;
        violation.time_s = trajectory[i].t_s;
        violation.point_index = static_cast<int>(i);
        violation.actual = accel;
        violation.limit = -limit;
        violation.detail = "acceleration_mps2 = " + format_double(accel) +
                           " m/s^2 at point " + std::to_string(i) +
                           " exceeds max_deceleration_mps2 = " +
                           format_double(limit);
        consider(std::move(violation));
        break;
      }
    }
  }

  // 4e. 曲率限制。
  {
    const double limit = request.limits.max_curvature_inv_m;
    const double allowed = limit + limit_slack(limit);
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
      const double curvature = std::abs(trajectory[i].curvature_inv_m);
      if (curvature > allowed) {
        ValidationViolation violation;
        violation.constraint = Constraint::kCurvatureLimit;
        violation.time_s = trajectory[i].t_s;
        violation.point_index = static_cast<int>(i);
        violation.actual = curvature;
        violation.limit = limit;
        violation.detail = "|curvature_inv_m| = " + format_double(curvature) +
                           " 1/m at point " + std::to_string(i) +
                           " exceeds max_curvature_inv_m = " +
                           format_double(limit);
        consider(std::move(violation));
        break;
      }
    }
  }

  // 为每相邻一对重建一个 MotionSegment，供走廊重采样和碰撞扫掠使用。
  std::vector<ReconstructedMotion> motions;
  motions.reserve(trajectory.size() - 1);
  for (std::size_t i = 0; i + 1 < trajectory.size(); ++i) {
    motions.push_back(reconstruct_motion(trajectory[i].pose,
                                         trajectory[i + 1].pose));
  }

  // 4f. 走廊包含性：沿轨迹稠密重采样的整个保守足迹必须位于 s 区间
  //     覆盖该采样点的走廊多边形内。缺少覆盖多边形属于覆盖失败；
  //     仅有中心线永远不够。
  {
    const collision::Footprint footprint = collision::make_footprint(request.vehicle);
    double s_cursor = 0.0;
    bool failed = false;
    const auto evaluate_sample = [&](double s_m, const Pose2d& pose,
                                     double time_s, int point_index) {
      if (failed) {
        return;
      }
      ++corridor_samples;
      const std::array<Point2d, 4> corner_array = footprint.world_corners(pose);
      const std::vector<Point2d> corners(corner_array.begin(),
                                         corner_array.end());
      bool contained = false;
      bool covered = false;
      double best_outside_m = std::numeric_limits<double>::infinity();
      int best_polygon = -1;
      for (std::size_t p = 0; p < request.corridor->polygons.size(); ++p) {
        const common::CorridorPolygon& polygon = request.corridor->polygons[p];
        if (s_m < polygon.s_start_m - kCorridorSCoverageEps ||
            s_m > polygon.s_end_m + kCorridorSCoverageEps) {
          continue;
        }
        covered = true;
        if (corridor::convex_polygon_contains(
                polygon.vertices, corners, options.containment_tolerance_m)) {
          contained = true;
          break;
        }
        double outside_m = 0.0;
        for (const Point2d& corner : corners) {
          outside_m = std::max(
              outside_m, collision::point_polygon_distance(corner, polygon.vertices));
        }
        if (outside_m < best_outside_m) {
          best_outside_m = outside_m;
          best_polygon = static_cast<int>(p);
        }
      }
      if (contained) {
        return;
      }
      failed = true;
      ValidationViolation violation;
      violation.constraint = Constraint::kCorridorContainment;
      violation.time_s = time_s;
      violation.point_index = point_index;
      if (!covered) {
        violation.actual = s_m;
        violation.limit = kNaN;
        violation.detail =
            "no corridor polygon covers the trajectory arc length s = " +
            format_double(s_m) + " m (point " + std::to_string(point_index) +
            ")";
      } else {
        violation.actual = best_outside_m;
        violation.limit = options.containment_tolerance_m;
        violation.detail =
            "whole vehicle footprint is outside every covering corridor "
            "polygon at s = " + format_double(s_m) + " m (point " +
            std::to_string(point_index) + "); best covering polygon " +
            std::to_string(best_polygon) + " is exceeded by " +
            format_double(best_outside_m) + " m";
      }
      consider(std::move(violation));
    };

    evaluate_sample(0.0, trajectory.front().pose, trajectory.front().t_s, 0);
    for (std::size_t i = 0; i + 1 < trajectory.size() && !failed; ++i) {
      const ReconstructedMotion& motion = motions[i];
      if (motion.valid) {
        const std::size_t steps = std::max<std::size_t>(
            1, static_cast<std::size_t>(
                   std::ceil(motion.arc_m / options.corridor_sample_step_m)));
        const double sigma_sign =
            motion.direction == Direction::kForward ? 1.0 : -1.0;
        for (std::size_t k = 1; k < steps && !failed; ++k) {
          const double fraction = static_cast<double>(k) /
                                  static_cast<double>(steps);
          const Pose2d pose = pose_on_arc(
              trajectory[i].pose, sigma_sign * motion.arc_m * fraction,
              motion.curvature_inv_m);
          const double time_s =
              trajectory[i].t_s +
              (trajectory[i + 1].t_s - trajectory[i].t_s) * fraction;
          evaluate_sample(s_cursor + motion.arc_m * fraction, pose, time_s,
                          static_cast<int>(i));
        }
        s_cursor += motion.arc_m;
      }
      // 存储位姿总会被精确检查；重建只填充其间的区间。
      evaluate_sample(s_cursor, trajectory[i + 1].pose,
                      trajectory[i + 1].t_s, static_cast<int>(i + 1));
    }
  }

  // 4g. 静态碰撞：精确位姿检查加上对每个重建段的保守运动扫掠
  // （仅有离散位姿永远不够）。
  {
    CollisionChecker checker(request.vehicle, *request.scene,
                             options.collision);
    if (!checker.valid()) {
      return finalize(invalid_result(
          "invalid_scene",
          "collision checker rejected the scene: " + checker.invalid_reason()));
    }
    bool failed = false;
    for (std::size_t i = 0; i < trajectory.size() && !failed; ++i) {
      ++collision_pose_checks;
      const collision::CheckResult pose_result =
          checker.check_pose(trajectory[i].pose);
      if (pose_result.collided()) {
        failed = true;
        ValidationViolation violation;
        violation.constraint = Constraint::kCollision;
        violation.time_s = trajectory[i].t_s;
        violation.point_index = static_cast<int>(i);
        violation.actual = kNaN;
        violation.limit = kNaN;
        violation.detail =
            "static collision at trajectory point " + std::to_string(i) +
            " (" + collision::to_string(pose_result.reason_code) + "): " +
            pose_result.detail;
        consider(std::move(violation));
        break;
      }
      if (i + 1 >= trajectory.size() || !motions[i].valid) {
        continue;
      }
      ++collision_motion_checks;
      MotionSegment segment;
      segment.start = trajectory[i].pose;
      segment.length_m = motions[i].arc_m;
      segment.direction = motions[i].direction;
      segment.curvature_inv_m = motions[i].curvature_inv_m;
      const collision::CheckResult motion_result = checker.check_motion(segment);
      if (motion_result.status == CheckStatus::kInvalidInput) {
        return finalize(invalid_result(
            "invalid_motion_reconstruction",
            "collision checker rejected the reconstructed motion segment "
            "between points " +
                std::to_string(i) + " and " + std::to_string(i + 1) + ": " +
                motion_result.detail));
      }
      if (motion_result.collided()) {
        failed = true;
        const double fraction =
            motions[i].arc_m > 0.0
                ? std::min(1.0, motion_result.first_s_m / motions[i].arc_m)
                : 0.0;
        ValidationViolation violation;
        violation.constraint = Constraint::kCollision;
        violation.time_s =
            trajectory[i].t_s +
            (trajectory[i + 1].t_s - trajectory[i].t_s) * fraction;
        violation.point_index = static_cast<int>(i);
        violation.actual = motion_result.first_s_m;
        violation.limit = kNaN;
        violation.detail =
            "static collision on the reconstructed motion between points " +
            std::to_string(i) + " and " + std::to_string(i + 1) + " (" +
            collision::to_string(motion_result.reason_code) + "): " +
            motion_result.detail;
        consider(std::move(violation));
        break;
      }
    }
  }

  // 4h. 端点：最终位姿对目标（路径标准容差）。
  {
    const Pose2d& last = trajectory.back().pose;
    const Pose2d& goal = *request.goal;
    const double position_error =
        std::hypot(last.x_m - goal.x_m, last.y_m - goal.y_m);
    if (position_error > kEndpointPositionToleranceM) {
      ValidationViolation violation;
      violation.constraint = Constraint::kEndpointPosition;
      violation.time_s = trajectory.back().t_s;
      violation.point_index = static_cast<int>(trajectory.size() - 1);
      violation.actual = position_error;
      violation.limit = kEndpointPositionToleranceM;
      violation.detail = "endpoint position error = " +
                         format_double(position_error) +
                         " m exceeds 0.05 m";
      consider(std::move(violation));
    }
    const double heading_error =
        std::abs(normalize_angle_rad(last.yaw_rad - goal.yaw_rad));
    if (heading_error > kEndpointHeadingToleranceRad) {
      ValidationViolation violation;
      violation.constraint = Constraint::kEndpointHeading;
      violation.time_s = trajectory.back().t_s;
      violation.point_index = static_cast<int>(trajectory.size() - 1);
      violation.actual = heading_error;
      violation.limit = kEndpointHeadingToleranceRad;
      violation.detail = "endpoint heading error = " +
                         format_double(heading_error) +
                         " rad exceeds 0.02 rad";
      consider(std::move(violation));
    }
  }

  if (first_failure.has_value()) {
    return finalize(failed_result(*first_failure));
  }
  ValidationResult result;
  result.status = common::Status::kSuccess;
  result.valid = true;
  result.reason = "valid";
  result.detail.clear();
  return finalize(result);
}

}  // namespace planner::validation
