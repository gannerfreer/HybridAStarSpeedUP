/**
 * @file user_astar.cpp
 * @brief 集成标准运动学倒车自行车模型、扫掠碰撞检测、Reeds-Shepp 解析展开以及 Dijkstra 目标势场启发式的 Hybrid A* 规划器实现文件。
 */

#include "planner/math/kinematics/arc.hpp"
#include "user_astar.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "hybrid_astar.h"

#include "grid_potential.hpp"
#include "scene.hpp"
#include "reeds_shepp.hpp"

namespace planner::hybrid_astar {

namespace {

using Clock = std::chrono::steady_clock;

using math::kPi;
constexpr double kStartPoseToleranceM = 1e-6;
constexpr double kStartYawToleranceRad = 1e-6;
constexpr double kEndpointPositionToleranceM = 0.05;  ///< 终点位置容差阈值（米，m）。
constexpr double kEndpointYawToleranceRad = 0.02;     ///< 终点航向角容差阈值（弧度，rad）。
constexpr double kCurvatureTolerance = 1e-9;

bool finite_pose(const common::Pose2d& pose) {
  return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) &&
         std::isfinite(pose.yaw_rad);
}

bool same_vehicle(const common::VehicleModel& a, const common::VehicleModel& b) {
  return a.wheelbase_m == b.wheelbase_m && a.width_m == b.width_m &&
         a.front_overhang_m == b.front_overhang_m &&
         a.rear_overhang_m == b.rear_overhang_m &&
         a.max_steer_rad == b.max_steer_rad &&
         a.safety_margin_m == b.safety_margin_m;
}

common::Pose2d to_common(const astar::Pose2D& pose) {
  common::Pose2d out;
  out.x_m = pose.x;
  out.y_m = pose.y;
  out.yaw_rad = math::normalize_angle_rad(pose.yaw);
  return out;
}

common::Direction to_common(astar::MotionDirection direction) {
  return direction == astar::MotionDirection::Backward
             ? common::Direction::kReverse
             : common::Direction::kForward;
}

std::string collision_detail(const collision::CheckResult& check) {
  std::string text = collision::to_string(check.reason_code);
  if (!check.detail.empty()) {
    text += ": " + check.detail;
  }
  return text;
}

/// 校验两个栅格视图是否拥有相同的快照标识、维度尺寸与单元格占据内容。
bool grids_equal(const common::GridView& a, const common::GridView& b) {
  return a.map_id == b.map_id && a.map_version == b.map_version &&
         a.width == b.width && a.height == b.height &&
         a.resolution_m == b.resolution_m && a.origin == b.origin &&
         a.cells == b.cells;
}

/// 从 Reeds-Shepp 连接路径中平滑剔除数值退化微段（亚毫米级微小位移）。
/// 这些微小位移源自词边界（word boundary）浮点极值情况，不具备几何意义，
/// 却会产生虚假换向尖点（cusp）并污染终点位姿。
rs::RsPlan clean_connector(rs::RsPlan plan) {
  constexpr double kDegenerateSegmentM = 1e-3;
  rs::RsPlan cleaned;
  cleaned.status = plan.status;
  cleaned.reason = std::move(plan.reason);
  cleaned.path.start = plan.path.start;
  cleaned.path.goal = plan.path.goal;
  cleaned.path.length_m = 0.0;
  for (const rs::RsSegment& segment : plan.path.segments) {
    if (segment.length_m < kDegenerateSegmentM) {
      continue;
    }
    cleaned.path.length_m += segment.length_m;
    cleaned.path.segments.push_back(segment);
  }
  return cleaned;
}

/// 根据相邻节点的位姿推导恒定基元曲率：曲率 = delta_yaw / signed_arc。
/// 底层 Propagate 对前进和倒车运动均将圆弧长度严格固定为 delta_dist，
/// 因此必须使用弧长而非弦长进行推导（使用弦长会引入 beta^2/24 的几何畸变）。
double primitive_curvature(const astar::Pose2D& from,
                           const astar::Pose2D& to, double arc_length_m) {
  if (arc_length_m <= 1e-12) {
    return 0.0;
  }
  const double dyaw = math::normalize_angle_rad(to.yaw - from.yaw);
  const double signed_arc = to.direction == astar::MotionDirection::Forward
                                ? arc_length_m
                                : -arc_length_m;
  return dyaw / signed_arc;
}

}  // namespace

struct UserAstar::Impl {
  UserAstarConfig config;
  std::function<bool()> cancel_callback;
  std::int64_t expanded_nodes{0};
  bool potential_valid{false};
  std::uint64_t potential_key{0};
  internal::GridPotential potential;
};

UserAstar::UserAstar(const UserAstarConfig& config) : impl_(std::make_unique<Impl>()) {
  configure(config);
}
UserAstar::~UserAstar() = default;
const UserAstarConfig& UserAstar::config() const { return impl_->config; }
void UserAstar::configure(const UserAstarConfig& config) {
  if (!std::isfinite(config.xy_resolution_m) || !(config.xy_resolution_m > 0.0))
    throw std::invalid_argument("invalid xy_resolution_m");
  if (!std::isfinite(config.motion_step_m) || !(config.motion_step_m > 0.0))
    throw std::invalid_argument("invalid motion_step_m");
  if (!std::isfinite(config.goal_capture_radius_m) || !(config.goal_capture_radius_m > 0.0))
    throw std::invalid_argument("invalid goal_capture_radius_m");
  if (!std::isfinite(config.default_time_budget_s) || !(config.default_time_budget_s > 0.0) || config.default_time_budget_s > 1e9)
    throw std::invalid_argument("invalid default_time_budget_s");
  if (!std::isfinite(config.forward_cost) || !(config.forward_cost >= 0.0))
    throw std::invalid_argument("invalid forward_cost");
  if (!std::isfinite(config.reverse_cost) || !(config.reverse_cost >= 0.0))
    throw std::invalid_argument("invalid reverse_cost");
  if (!std::isfinite(config.turn_penalty) || !(config.turn_penalty >= 0.0))
    throw std::invalid_argument("invalid turn_penalty");
  if (!std::isfinite(config.switch_cost) || !(config.switch_cost >= 0.0))
    throw std::invalid_argument("invalid switch_cost");
  if (!std::isfinite(config.backward_search_range_m) || !(config.backward_search_range_m >= 0.0))
    throw std::invalid_argument("invalid backward_search_range_m");
  if (!std::isfinite(config.linear_preferred_distance_square) || !(config.linear_preferred_distance_square >= 0.0))
    throw std::invalid_argument("invalid linear_preferred_distance_square");
  if (!std::isfinite(config.analytic_shot_distance_m) || !(config.analytic_shot_distance_m >= 0.0))
    throw std::invalid_argument("invalid analytic_shot_distance_m");
  if (!std::isfinite(config.analytic_expansion_max_length_m) || !(config.analytic_expansion_max_length_m >= 0.0))
    throw std::invalid_argument("invalid analytic_expansion_max_length_m");
  if (!std::isfinite(config.min_connector_segment_m) || !(config.min_connector_segment_m >= 0.0))
    throw std::invalid_argument("invalid min_connector_segment_m");
  if (config.heading_bins < 8 || config.steering_samples < 2 ||
      config.analytic_expansion_ratio < 1 || config.default_node_budget < 1)
    throw std::invalid_argument("invalid discrete search configuration");
  impl_->config = config;
  reset_cache();
}

void UserAstar::set_cancel_callback(std::function<bool()> callback) {
  impl_->cancel_callback = std::move(callback);
}

void UserAstar::reset_cache() {
  // 使目标势场缓存失效。
  impl_->potential_valid = false;
  impl_->potential_key = 0;
  impl_->potential = internal::GridPotential{};
  impl_->expanded_nodes = 0;
}

common::PlanResult UserAstar::plan(
    const common::PlanRequest& request, const common::GridView& grid,
    const collision::CollisionChecker& collision_checker) {
  common::PlanResult result;
  result.request_id = request.request_id;

  const auto fail = [&result](common::Status status, const std::string& reason) {
    result.status = status;
    result.reason = reason;
    return result;
  };

  if (request.cancelled) {
    return fail(common::Status::kCancelled, "request was cancelled");
  }
  if (request.map_id != grid.map_id ||
      request.map_version != grid.map_version) {
    return fail(common::Status::kInvalidInput,
                "request map_id/map_version does not match the grid snapshot");
  }
  if (!same_vehicle(request.vehicle, collision_checker.vehicle())) {
    return fail(common::Status::kInvalidInput,
                "request vehicle does not match collision checker vehicle");
  }
  if (!collision_checker.valid()) {
    return fail(common::Status::kInvalidInput,
                "collision checker is invalid: " +
                    collision_checker.invalid_reason());
  }
  if (!finite_pose(request.start) || !finite_pose(request.goal)) {
    return fail(common::Status::kInvalidInput, "start/goal pose is not finite");
  }
  if (grid.cells.size() !=
      static_cast<std::size_t>(grid.width) *
          static_cast<std::size_t>(grid.height)) {
    return fail(common::Status::kInvalidInput,
                "grid cell count does not match width*height");
  }
  if (request.vehicle.wheelbase_m <= 0.0 || request.vehicle.max_steer_rad <= 0.0) {
    return fail(common::Status::kInvalidInput,
                "vehicle wheelbase/max steer must be positive");
  }

  if ((request.time_budget_s && (!std::isfinite(*request.time_budget_s) || *request.time_budget_s <= 0.0 || *request.time_budget_s > 1e9)) ||
      (request.node_budget && *request.node_budget < 1)) {
    return fail(common::Status::kInvalidInput, "invalid time or node budget");
  }

  impl_->expanded_nodes = 0;

  const Clock::time_point plan_start = Clock::now();
  const double time_budget_s =
      request.time_budget_s.value_or(impl_->config.default_time_budget_s);
  const std::int64_t node_budget =
      request.node_budget.value_or(impl_->config.default_node_budget);
  const Clock::time_point deadline =
      plan_start + std::chrono::duration_cast<Clock::duration>(
                       std::chrono::duration<double>(time_budget_s));

  bool potential_reused = false;
  const auto early_failure = [&](common::Status status, const std::string& reason) {
    return fail(status, reason);
  };

  // ------------------------------------------------------------------
  // 底层配置参数映射与转换
  // ------------------------------------------------------------------
  astar::PlannerConfig config;
  config.vehicle.wheel_base = request.vehicle.wheelbase_m;
  const double steer_limit_deg =
      request.vehicle.max_steer_rad * 180.0 / kPi;
  config.hybrid_astar.min_steering = -steer_limit_deg;
  config.hybrid_astar.max_steering = steer_limit_deg;
  config.hybrid_astar.angle_discrete_num =
      static_cast<double>(impl_->config.steering_samples);
  config.hybrid_astar.delta_dist = impl_->config.motion_step_m;
  config.hybrid_astar.grid_dist = impl_->config.xy_resolution_m;
  config.hybrid_astar.grid_angle =
      360.0 / static_cast<double>(impl_->config.heading_bins);
  config.hybrid_astar.max_iterations_astar =
      static_cast<double>(node_budget);
  config.hybrid_astar.forward_penalty = impl_->config.forward_cost;
  config.hybrid_astar.backward_penalty = impl_->config.reverse_cost;
  config.hybrid_astar.turnning_penalty = impl_->config.turn_penalty;
  config.hybrid_astar.switch_penalty = impl_->config.switch_cost;
  config.hybrid_astar.backward_search_range =
      impl_->config.backward_search_range_m;
  config.hybrid_astar.linear_preferred_distance_square =
      impl_->config.linear_preferred_distance_square;
  config.hybrid_astar.allow_reverse = impl_->config.allow_reverse;

  // ------------------------------------------------------------------
  // 目标势场连通性检验与不可达状态预剪枝。
  // 起点与目标位姿首先由完整的车辆多边形轮廓执行精确碰撞检验。
  // ------------------------------------------------------------------
  bool potential_usable = false;
  collision::GridFrame potential_frame = collision::GridFrame::from_grid(grid);
  const internal::GridPotential* potential = nullptr;
  const auto cell_of_pose = [&](const common::Pose2d& pose, int* row, int* col) {
    const common::Point2d grid_point =
        potential_frame.to_grid_frame(common::Point2d{pose.x_m, pose.y_m});
    const int c = static_cast<int>(std::floor(grid_point.x_m / grid.resolution_m));
    const int r = static_cast<int>(std::floor(grid_point.y_m / grid.resolution_m));
    if (!grid.in_bounds(r, c)) {
      return false;
    }
    *row = r;
    *col = c;
    return true;
  };

  if (!grids_equal(collision_checker.scene().grid, grid)) {
    return early_failure(
        common::Status::kInvalidInput,
        "the collision checker scene grid does not match the grid argument");
  }
  const collision::CheckResult start_pose_check =
      collision_checker.check_pose(request.start);
  if (start_pose_check.status == collision::CheckStatus::kInvalidInput) {
    return early_failure(common::Status::kInvalidInput,
                         "start pose was rejected: " +
                             collision_detail(start_pose_check));
  }
  if (start_pose_check.collided()) {
    return early_failure(common::Status::kInvalidInput,
                         "start pose is in collision (" +
                             collision_detail(start_pose_check) + ")");
  }
  const collision::CheckResult goal_pose_check =
      collision_checker.check_pose(request.goal);
  if (goal_pose_check.status == collision::CheckStatus::kInvalidInput) {
    return early_failure(common::Status::kInvalidInput,
                         "goal pose was rejected: " +
                             collision_detail(goal_pose_check));
  }
  if (goal_pose_check.collided()) {
    return early_failure(common::Status::kInvalidInput,
                         "goal pose is in collision (" +
                             collision_detail(goal_pose_check) + ")");
  }

  int start_row = 0;
  int start_col = 0;
  int goal_row = 0;
  int goal_col = 0;
  if (!cell_of_pose(request.start, &start_row, &start_col) ||
      !cell_of_pose(request.goal, &goal_row, &goal_col)) {
    return early_failure(common::Status::kInvalidInput,
                         "start or goal rear-axle position is outside the "
                         "grid");
  }

  potential_usable = std::isfinite(grid.resolution_m) &&
                     grid.resolution_m > 0.0;
  if (potential_usable) {
    const std::uint64_t potential_key =
        internal::grid_cache_key(grid, goal_row, goal_col);
    potential_reused =
        impl_->potential_valid && impl_->potential_key == potential_key;
    if (!potential_reused) {
      const auto deadline_exceeded = [&]() { return Clock::now() >= deadline; };
      internal::GridPotential computed = internal::compute_grid_potential(
          grid, goal_row, goal_col, deadline_exceeded);
      if (computed.aborted) {
        return early_failure(
            common::Status::kTimeout,
            "time budget of " + std::to_string(time_budget_s) +
                " s exhausted while computing the goal potential");
      }
      impl_->potential = std::move(computed);
      impl_->potential_key = potential_key;
      impl_->potential_valid = true;
    }
    potential = &impl_->potential;
    potential_usable =
        potential->distance_m.size() ==
        static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height);
  }
  if (potential_usable) {
    const int start_index = start_row * grid.width + start_col;
    if (!std::isfinite(
            potential->distance_m[static_cast<std::size_t>(start_index)])) {
      return early_failure(
          common::Status::kNoPath,
          "no path: start and goal are disconnected in the free-grid "
          "relaxation");
    }
  }

  astar::HybridAStar planner(config);

  // Reeds-Shepp 目标连接路径缓存（由目标达成钩子计算并填充）。
  bool connector_valid = false;
  rs::RsPath connector;
  std::int64_t goal_hook_calls = 0;

  // 车辆多边形轮廓位姿与运动线段碰撞检测钩子。
  planner.SetPoseCollisionHook([&](const astar::Pose2D& pose) {
    const collision::CheckResult check =
        collision_checker.check_pose(to_common(pose));
    return !check.is_clear();
  });
  planner.SetMotionCollisionHook(
      [&](const astar::Pose2D& from, const astar::Pose2D& to) {
        // 基于连通性松弛的预剪枝（当目标势场可用时）：
        // 在空闲栅格松弛图上均不可达的后继节点不可能构成任何无碰撞可行路径，
        // 应当立即予以剪枝剔除（详见 grid_potential.hpp）。
        if (potential_usable) {
          int row = 0;
          int col = 0;
          if (!cell_of_pose(to_common(to), &row, &col) ||
              !std::isfinite(potential->distance_m[
                  static_cast<std::size_t>(row) *
                      static_cast<std::size_t>(grid.width) +
                  static_cast<std::size_t>(col)])) {
            return true;
          }
        }
        const double arc_length = impl_->config.motion_step_m;
        collision::MotionSegment segment;
        segment.start = to_common(from);
        segment.length_m = arc_length;
        segment.direction = to_common(to.direction);
        segment.curvature_inv_m =
            primitive_curvature(from, to, arc_length);
        const collision::CheckResult check =
            collision_checker.check_motion(segment);
        if (!check.is_clear()) {
          return true;
        }
        return false;
      });
  planner.SetGoalHook([&](const astar::Pose2D& pose) {
    const double distance =
        std::hypot(pose.x - request.goal.x_m, pose.y - request.goal.y_m);
    if (distance > impl_->config.goal_capture_radius_m) {
      return false;
    }
    ++goal_hook_calls;
    const bool immediate = distance <= 2.0 * impl_->config.motion_step_m;
    if (!immediate &&
        goal_hook_calls % impl_->config.analytic_expansion_ratio != 0) {
      return false;
    }
    if (impl_->config.analytic_shot_distance_m > 0.0 &&
        distance > impl_->config.analytic_shot_distance_m) {
      return false;
    }
    rs::RsPlan solved = rs::plan(to_common(pose), request.goal, request.vehicle);
    if (solved.status != common::Status::kSuccess) {
      return false;
    }
    // 在对剩余段进行碰撞检测前，剔除退化的亚毫米微段。
    solved = clean_connector(std::move(solved));
    if (impl_->config.analytic_expansion_max_length_m > 0.0 &&
        solved.path.length_m >
            impl_->config.analytic_expansion_max_length_m) {
      return false;
    }
    // 短于 min_connector_segment_m 的子基元微段：仅当与相邻段构成虚假换向尖点时剔除连接，
    // 从而引导搜索继续外扩以寻找更平滑清晰的连接。
    if (impl_->config.min_connector_segment_m > 0.0) {
      const std::vector<rs::RsSegment>& segments = solved.path.segments;
      for (std::size_t i = 0; i + 1 < segments.size(); ++i) {
        if (segments[i].direction == segments[i + 1].direction) {
          continue;
        }
        if (segments[i].length_m < impl_->config.min_connector_segment_m ||
            segments[i + 1].length_m <
                impl_->config.min_connector_segment_m) {
          return false;
        }
      }
    }
    // 直接检查 RS 原始常曲率段，length_m 是完整弧长，不能用采样点间弦长代替。
    common::Pose2d cursor = solved.path.start;
    try {
      for (const rs::RsSegment& rs_segment : solved.path.segments) {
        collision::MotionSegment segment;
        segment.start = cursor;
        segment.length_m = rs_segment.length_m;
        segment.direction = rs_segment.direction;
        segment.curvature_inv_m = rs_segment.curvature_inv_m;
        if (!collision_checker.check_motion(segment).is_clear()) {
          return false;
        }
        cursor = rs::integrate_segment(cursor, rs_segment, rs_segment.length_m);
      }
    } catch (const std::exception&) {
      return false;
    }
    connector_valid = true;
    connector = solved.path;
    return true;
  });

  bool aborted = false;
  bool timed_out = false;
  planner.SetAbortCallback([&]() {
    if (request.cancelled ||
        (impl_->cancel_callback && impl_->cancel_callback())) {
      aborted = true;
      return true;
    }
    if (Clock::now() >= deadline) {
      timed_out = true;
      return true;
    }
    return false;
  });

  // ------------------------------------------------------------------
  // 执行 Hybrid A* 搜索
  // ------------------------------------------------------------------
  astar::Pose2D start_pose;
  start_pose.x = request.start.x_m;
  start_pose.y = request.start.y_m;
  start_pose.yaw = request.start.yaw_rad;
  start_pose.direction = astar::MotionDirection::Forward;
  astar::Pose2D goal_pose;
  goal_pose.x = request.goal.x_m;
  goal_pose.y = request.goal.y_m;
  goal_pose.yaw = request.goal.yaw_rad;
  goal_pose.direction = astar::MotionDirection::Forward;

  astar::HybridSearchResult search;
  const bool found = planner.Search(start_pose, goal_pose, &search);

  impl_->expanded_nodes = search.iterations;

  if (!found) {
    if (aborted) {
      return fail(common::Status::kCancelled, "request was cancelled");
    }
    if (timed_out) {
      return fail(common::Status::kTimeout,
                  "time budget of " + std::to_string(time_budget_s) +
                      " s exhausted");
    }
    if (impl_->expanded_nodes >= node_budget) {
      return fail(common::Status::kTimeout,
                  "node budget of " + std::to_string(node_budget) +
                      " exhausted");
    }
    if (search.message == "start or goal in collision") {
      return fail(common::Status::kInvalidInput,
                  "start or goal pose is in collision");
    }
    return fail(common::Status::kNoPath, search.message.empty()
                                             ? "no feasible path"
                                             : search.message);
  }

  // ------------------------------------------------------------------
  // 路径轨迹拼装（整合 Reeds-Shepp 连接曲线）与严格后置校验
  // ------------------------------------------------------------------
  std::vector<common::PathPoint> flat;
  flat.reserve(search.path.size() + 64);
  const std::size_t node_count = search.path.size();
  double s_cum = 0.0;
  for (std::size_t i = 0; i < node_count; ++i) {
    common::PathPoint point;
    point.pose = to_common(search.path[i]);
    point.s_m = s_cum;
    if (i + 1 < node_count) {
      point.direction = to_common(search.path[i + 1].direction);
      point.curvature_inv_m = primitive_curvature(
          search.path[i], search.path[i + 1], impl_->config.motion_step_m);
    } else if (connector_valid && !connector.segments.empty()) {
      point.direction = connector.segments.front().direction;
      point.curvature_inv_m = connector.segments.front().curvature_inv_m;
    } else if (i > 0) {
      point.direction = to_common(search.path[i].direction);
      point.curvature_inv_m = primitive_curvature(
          search.path[i - 1], search.path[i], impl_->config.motion_step_m);
    } else {
      point.direction = common::Direction::kForward;
      point.curvature_inv_m = 0.0;
    }
    flat.push_back(point);
    if (i + 1 < node_count) {
      // 底层运动基元的圆弧长度对前进和倒车均固定为 delta_dist。
      s_cum += impl_->config.motion_step_m;
    }
  }
  if (connector_valid && connector.length_m > 0.0) {
    std::vector<common::PathPoint> samples;
    try {
      samples = rs::sample(connector, std::max(0.2, 0.5 * impl_->config.motion_step_m));
    } catch (const std::exception& error) {
      return fail(common::Status::kValidationFailed,
                  std::string("RS sampling error: ") + error.what());
    }
    for (std::size_t k = 1; k < samples.size(); ++k) {
      common::PathPoint point = samples[k];
      point.s_m += s_cum;
      flat.push_back(point);
    }
  }

  std::vector<common::PathSegment> path;
  for (const common::PathPoint& point : flat) {
    if (path.empty()) {
      path.emplace_back();
      path.back().points.push_back(point);
      continue;
    }
    if (path.back().points.back().direction != point.direction) {
      common::PathPoint cusp = point;
      cusp.direction = path.back().points.back().direction;
      cusp.curvature_inv_m = path.back().points.back().curvature_inv_m;
      path.back().points.push_back(cusp);
      path.emplace_back();
      path.back().points.push_back(point);
    } else {
      path.back().points.push_back(point);
    }
  }

  result.path = path;
  double max_abs_curvature = 0.0;
  bool structural_error = false;
  std::string structural_reason;
  if (path.empty() || path.front().points.empty()) {
    structural_error = true;
    structural_reason = "output path is empty";
  }
  if (!structural_error) {
    for (std::size_t segment_index = 0; segment_index < path.size();
         ++segment_index) {
      const common::PathSegment& segment = path[segment_index];
      for (std::size_t i = 0; i < segment.points.size(); ++i) {
        const common::PathPoint& point = segment.points[i];
        if (!std::isfinite(point.s_m) ||
            !std::isfinite(point.curvature_inv_m) ||
            !finite_pose(point.pose)) {
          structural_error = true;
          structural_reason = "output path contains a non-finite value";
          break;
        }
        max_abs_curvature =
            std::max(max_abs_curvature, std::abs(point.curvature_inv_m));
        if (segment_index == 0 && i == 0) {
          const double position_error =
              std::hypot(point.pose.x_m - request.start.x_m,
                         point.pose.y_m - request.start.y_m);
          const double yaw_error = std::abs(math::normalize_angle_rad(
              point.pose.yaw_rad - request.start.yaw_rad));
          if (position_error > kStartPoseToleranceM ||
              yaw_error > kStartYawToleranceRad) {
            structural_error = true;
            structural_reason =
                "output path does not begin at the requested start pose";
            break;
          }
        }
      }
      if (structural_error) {
        break;
      }
    }
  }

  const common::Pose2d endpoint = path.empty() || path.back().points.empty()
                                      ? common::Pose2d{}
                                      : path.back().points.back().pose;
  const double endpoint_position_error =
      std::hypot(endpoint.x_m - request.goal.x_m,
                 endpoint.y_m - request.goal.y_m);
  const double endpoint_yaw_error = std::abs(math::normalize_angle_rad(
      endpoint.yaw_rad - request.goal.yaw_rad));

  if (structural_error) {
    return fail(common::Status::kValidationFailed, structural_reason);
  }

  const double max_curvature = 1.0 / request.vehicle.min_turn_radius_m();
  if (max_abs_curvature > max_curvature + kCurvatureTolerance) {
    double worst_curvature = 0.0;
    double worst_s = 0.0;
    for (const common::PathSegment& segment : path) {
      for (const common::PathPoint& point : segment.points) {
        if (std::abs(point.curvature_inv_m) > worst_curvature) {
          worst_curvature = std::abs(point.curvature_inv_m);
          worst_s = point.s_m;
        }
      }
    }
    return fail(common::Status::kValidationFailed,
                "output curvature " + std::to_string(worst_curvature) +
                    " 1/m at s=" + std::to_string(worst_s) +
                    " exceeds the vehicle limit " +
                    std::to_string(max_curvature));
  }
  if (endpoint_position_error > kEndpointPositionToleranceM) {
    return fail(common::Status::kValidationFailed,
                "endpoint position error exceeds 0.05 m");
  }
  if (endpoint_yaw_error > kEndpointYawToleranceRad) {
    return fail(common::Status::kValidationFailed,
                "endpoint yaw error exceeds 0.02 rad");
  }
  if (!collision_checker.check_path(result.path).is_clear()) {
    return fail(common::Status::kValidationFailed,
                "output path fails the common collision re-check");
  }

  result.status = common::Status::kSuccess;
  result.reason.clear();
  return result;
}

}  // namespace planner::hybrid_astar
