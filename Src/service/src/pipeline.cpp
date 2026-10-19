// W5 规划流水线（见 pipeline.hpp）。

#include "planner/service/pipeline.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "planner/hybrid_astar/backend.hpp"
#include "planner/hybrid_astar/rs_provider.hpp"

namespace planner::service {
namespace {

using nlohmann::json;

json finite_or_null(double value) {
  return std::isfinite(value) ? json(value) : json(nullptr);
}

json corridor_stats_json(const corridor::CorridorStats& stats) {
  return json{{"seed_points", stats.seed_points},
              {"construction_poses", stats.construction_poses},
              {"polygon_count", stats.polygon_count},
              {"path_length_m", finite_or_null(stats.path_length_m)},
              {"min_polygon_width_m", finite_or_null(stats.min_polygon_width_m)},
              {"min_obstacle_clearance_m", finite_or_null(stats.min_obstacle_clearance_m)},
              {"obstacle_limited_edges", stats.obstacle_limited_edges},
              {"robustness_margin_m", finite_or_null(stats.robustness_margin_m)}};
}

json optimize_stats_json(const dftpav::OptimizeStats& stats) {
  return json{{"direction_changes", stats.direction_changes},
              {"total_pieces", stats.total_pieces},
              {"lbfgs_iterations", stats.lbfgs_iterations},
              {"lbfgs_result", stats.lbfgs_result},
              {"final_cost", finite_or_null(stats.final_cost)},
              {"solve_time_s", finite_or_null(stats.solve_time_s)},
              {"trajectory_duration_s", finite_or_null(stats.trajectory_duration_s)},
              {"trajectory_points", stats.trajectory_points},
              {"corridor_constraint_points", stats.corridor_constraint_points},
              {"max_speed_mps", finite_or_null(stats.max_speed_mps)},
              {"max_acceleration_mps2", finite_or_null(stats.max_acceleration_mps2)},
              {"max_curvature_inv_m", finite_or_null(stats.max_curvature_inv_m)},
              {"max_lateral_acceleration_mps2",
               finite_or_null(stats.max_lateral_acceleration_mps2)},
              {"tightening_attempts", stats.tightening_attempts},
              {"corridor_repair_iterations", stats.corridor_repair_iterations},
              {"corridor_repair_worst_violation_m",
               finite_or_null(stats.corridor_repair_worst_violation_m)},
              {"attempt_reasons", stats.attempt_reasons}};
}

json validation_json(const validation::ValidationResult& validation) {
  json first_failure = nullptr;
  if (!validation.valid) {
    const validation::ValidationViolation& failure = validation.first_failure;
    first_failure = json{{"constraint", validation::to_string(failure.constraint)},
                         {"time_s", finite_or_null(failure.time_s)},
                         {"point_index", failure.point_index},
                         {"actual", finite_or_null(failure.actual)},
                         {"limit", finite_or_null(failure.limit)},
                         {"detail", failure.detail}};
  }
  return json{{"valid", validation.valid},
              {"reason", validation.reason},
              {"detail", validation.detail},
              {"first_failure", std::move(first_failure)},
              {"checks", json{{"trajectory_points", validation.trajectory_points},
                              {"corridor_samples", validation.corridor_samples},
                              {"collision_pose_checks", validation.collision_pose_checks},
                              {"collision_motion_checks", validation.collision_motion_checks}}}};
}

double polyline_length(const std::vector<common::PathSegment>& path) {
  double total = 0.0;
  for (const common::PathSegment& segment : path) {
    for (std::size_t i = 1; i < segment.points.size(); ++i) {
      total += std::hypot(segment.points[i].pose.x_m - segment.points[i - 1].pose.x_m,
                          segment.points[i].pose.y_m - segment.points[i - 1].pose.y_m);
    }
  }
  return total;
}

double trajectory_length(const std::vector<common::TrajectoryPoint>& trajectory) {
  double total = 0.0;
  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    total += std::hypot(trajectory[i].pose.x_m - trajectory[i - 1].pose.x_m,
                        trajectory[i].pose.y_m - trajectory[i - 1].pose.y_m);
  }
  return total;
}

json grid_json(const common::GridView& grid) {
  return json{{"map_id", grid.map_id},
              {"map_version", grid.map_version},
              {"width", grid.width},
              {"height", grid.height},
              {"resolution_m", grid.resolution_m},
              {"origin", json{{"x_m", grid.origin.x_m},
                              {"y_m", grid.origin.y_m},
                              {"yaw_rad", grid.origin.yaw_rad}}}};
}

json build_statistics(const PipelineResult& result, const ServicePlanRequest& request,
                      const common::GridView& grid) {
  json stats = json::object();
  stats["hybrid_backend"] = result.hybrid_backend;
  stats["stage"] = result.stage;
  if (!result.reason.empty()) {
    stats["reason"] = result.reason;
  }
  stats["grid"] = grid_json(grid);
  stats["timings_s"] = json{{"hybrid", finite_or_null(result.hybrid_time_s)},
                            {"corridor", finite_or_null(result.corridor_time_s)},
                            {"optimize", finite_or_null(result.optimize_time_s)},
                            {"validate", finite_or_null(result.validation_time_s)}};
  if (result.hybrid_ran) {
    stats["hybrid"] = result.hybrid.statistics;
  }
  if (result.corridor_ran) {
    stats["corridor"] = corridor_stats_json(result.corridor.stats);
  }
  if (result.optimize_ran) {
    stats["optimize"] = optimize_stats_json(result.optimize.stats);
  }
  if (result.validation_ran) {
    stats["validation"] = validation_json(result.validation);
  }
  if (result.hybrid_ran && !result.hybrid.path.empty()) {
    stats["path_length_m"] = finite_or_null(polyline_length(result.hybrid.path));
  }
  if (result.optimize_ran && !result.optimize.trajectory.empty()) {
    stats["trajectory_length_m"] = finite_or_null(trajectory_length(result.optimize.trajectory));
    stats["trajectory_points"] = result.optimize.trajectory.size();
  }
  stats["hybrid_budget_s"] = finite_or_null(request.params.hybrid_time_budget_s);
  stats["hybrid_node_budget"] = request.params.hybrid_node_budget;
  stats["dftpav_budget_s"] = finite_or_null(request.params.dftpav_time_budget_s);
  return stats;
}

}  // namespace

dftpav::OptimizeLimits limits_for_vehicle(const common::VehicleModel& vehicle) {
  dftpav::OptimizeLimits limits;
  limits.max_forward_velocity_mps = 5.0;
  limits.max_backward_velocity_mps = 2.0;
  limits.max_forward_acceleration_mps2 = 4.0;
  limits.max_backward_acceleration_mps2 = 4.0;
  const double max_curvature = std::tan(vehicle.max_steer_rad) / vehicle.wheelbase_m;
  limits.max_forward_curvature_inv_m = max_curvature;
  limits.max_backward_curvature_inv_m = max_curvature;
  limits.max_lateral_acceleration_mps2 = 5.0;
  limits.max_steering_rate_radps = 10000.0;
  return limits;
}

PipelineResult run_pipeline(const ServicePlanRequest& request,
                            const common::GridView& grid,
                            const collision::Scene& scene,
                            const StageCallback& on_stage,
                            const CancelCallback& cancelled) {
  PipelineResult result;
  result.hybrid_backend = request.params.hybrid_backend;

  const auto now = [] { return std::chrono::steady_clock::now(); };
  const auto elapsed_s = [](const auto& from, const auto& to) {
    return std::chrono::duration<double>(to - from).count();
  };
  const auto is_cancelled = [&]() { return cancelled && cancelled(); };
  const auto announce = [&](const char* stage_name) {
    result.stage = stage_name;
    if (on_stage) {
      on_stage(stage_name);
    }
  };
  const auto finish = [&]() -> PipelineResult {
    result.statistics = build_statistics(result, request, grid);
    return std::move(result);
  };
  const auto cancel_result = [&](const char* detail) -> PipelineResult {
    result.status = common::Status::kCancelled;
    result.reason = "cancelled";
    result.detail = detail;
    return finish();
  };

  // -------------------------------------------------------------------------
  // Hybrid A*。
  // -------------------------------------------------------------------------
  announce(stage::kHybrid);
  if (is_cancelled()) {
    return cancel_result("cancelled before the hybrid stage");
  }
  std::unique_ptr<hybrid_astar::HybridAStarBackend> backend =
      hybrid_astar::create_backend(request.params.hybrid_backend);
  if (!backend) {
    result.status = common::Status::kInvalidInput;
    result.reason = "unknown_hybrid_backend";
    result.detail = "no hybrid backend with id '" + request.params.hybrid_backend + "'";
    return finish();
  }
  const hybrid_astar::ConfigReport config_report =
      backend->configure(request.params.hybrid_config);
  if (!config_report.accepted) {
    result.status = common::Status::kInvalidInput;
    result.reason = "invalid_hybrid_config";
    result.detail = config_report.reason;
    return finish();
  }
  if (!config_report.unsupported.empty()) {
    std::string joined;
    for (const std::string& key : config_report.unsupported) {
      if (!joined.empty()) {
        joined += ", ";
      }
      joined += key;
    }
    result.status = common::Status::kInvalidInput;
    result.reason = "unsupported_hybrid_parameters";
    result.detail = "backend '" + request.params.hybrid_backend +
                    "' cannot map: " + joined;
    return finish();
  }
  backend->set_cancel_callback(cancelled);

  collision::CollisionChecker checker(request.vehicle, scene);
  if (!checker.valid()) {
    result.status = common::Status::kInvalidInput;
    result.reason = "invalid_scene";
    result.detail = checker.invalid_reason();
    return finish();
  }
  hybrid_astar::PlannerRsProvider rs_provider(request.vehicle);

  common::PlanRequest plan_request;
  plan_request.request_id = request.request_id;
  plan_request.map_id = request.map_id;
  plan_request.map_version = request.map_version;
  plan_request.vehicle = request.vehicle;
  plan_request.start = request.start;
  plan_request.goal = request.goal;
  plan_request.config = request.params.hybrid_config;
  plan_request.time_budget_s = request.params.hybrid_time_budget_s;
  plan_request.node_budget = request.params.hybrid_node_budget;
  plan_request.cancelled = is_cancelled();

  const auto hybrid_start = now();
  result.hybrid = backend->plan(plan_request, grid, checker, rs_provider);
  result.hybrid_time_s = elapsed_s(hybrid_start, now());
  result.hybrid_ran = true;
  if (result.hybrid.status != common::Status::kSuccess) {
    result.status = result.hybrid.status;
    switch (result.hybrid.status) {
      case common::Status::kNoPath:
        result.reason = "no_path";
        break;
      case common::Status::kTimeout:
        result.reason = "timeout";
        break;
      case common::Status::kCancelled:
        result.reason = "cancelled";
        break;
      case common::Status::kInvalidInput:
        result.reason = "invalid_input";
        break;
      case common::Status::kOptimizationFailed:
        result.reason = "optimization_failed";
        break;
      case common::Status::kValidationFailed:
        result.reason = "validation_failed";
        break;
      case common::Status::kSuccess:
        break;
    }
    result.detail = result.hybrid.reason;
    return finish();
  }
  if (result.hybrid.path.empty()) {
    result.status = common::Status::kNoPath;
    result.reason = "no_path";
    result.detail = "the hybrid backend reported success with an empty path";
    return finish();
  }
  if (is_cancelled()) {
    return cancel_result("cancelled after the hybrid stage");
  }

  // -------------------------------------------------------------------------
  // 安全走廊。
  // -------------------------------------------------------------------------
  announce(stage::kCorridor);
  if (is_cancelled()) {
    return cancel_result("cancelled before the corridor stage");
  }
  corridor::CorridorRequest corridor_request;
  corridor_request.request_id = request.request_id;
  corridor_request.map_id = request.map_id;
  corridor_request.map_version = request.map_version;
  corridor_request.frame_id = "map";
  corridor_request.vehicle = request.vehicle;
  corridor_request.seed = result.hybrid.path;
  corridor_request.obstacles.reserve(request.obstacles.size());
  for (const ServiceObstacle& obstacle : request.obstacles) {
    corridor_request.obstacles.push_back(obstacle.to_corridor());
  }
  corridor_request.options = request.params.corridor;

  const auto corridor_start = now();
  result.corridor = corridor::generate_corridor(corridor_request);
  result.corridor_time_s = elapsed_s(corridor_start, now());
  result.corridor_ran = true;
  if (!result.corridor.ok()) {
    result.status = result.corridor.status;
    result.reason = "corridor_" + result.corridor.reason;
    result.detail = result.corridor.detail;
    return finish();
  }
  if (is_cancelled()) {
    return cancel_result("cancelled after the corridor stage");
  }

  // -------------------------------------------------------------------------
  // Dftpav 优化。
  // -------------------------------------------------------------------------
  announce(stage::kOptimize);
  if (is_cancelled()) {
    return cancel_result("cancelled before the optimize stage");
  }
  dftpav::OptimizeRequest optimize_request;
  optimize_request.request_id = request.request_id;
  optimize_request.map_id = request.map_id;
  optimize_request.map_version = request.map_version;
  optimize_request.vehicle = request.vehicle;
  optimize_request.seed = result.hybrid.path;
  optimize_request.corridor = result.corridor.corridor;
  optimize_request.goal = request.goal;
  optimize_request.scene = scene;
  optimize_request.limits =
      request.params.dftpav_limits.value_or(limits_for_vehicle(request.vehicle));
  optimize_request.options = request.params.dftpav_options;
  // 协作式取消（dftpav 移植扩展）：把作业取消状态传入优化器，使取消在
  // L-BFGS 迭代边界生效，而不必等待阶段返回；优化器在取消时不返回部分
  // 轨迹，V/碰撞门禁不受影响。
  if (cancelled) {
    optimize_request.options.cancel_check = cancelled;
  }

  const auto optimize_start = now();
  result.optimize = dftpav::optimize_trajectory(optimize_request);
  result.optimize_time_s = elapsed_s(optimize_start, now());
  result.optimize_ran = true;
  // 优化器内部观察到取消（与作业取消同源；此处也覆盖自定义取消谓词），
  // 或阶段返回后才发现取消：按既有取消语义收尾，不返回任何几何。
  if (result.optimize.status == common::Status::kCancelled || is_cancelled()) {
    return cancel_result("cancelled during the optimize stage");
  }
  if (result.optimize_time_s > request.params.dftpav_time_budget_s) {
    result.status = common::Status::kTimeout;
    result.reason = "timeout";
    result.detail = "the dftpav stage took " + std::to_string(result.optimize_time_s) +
                    " s and exceeded its " +
                    std::to_string(request.params.dftpav_time_budget_s) +
                    " s budget (the budget is evaluated when the stage returns; "
                    "only cooperative cancellation interrupts the solver)";
    return finish();
  }
  if (!result.optimize.ok()) {
    result.status = result.optimize.status;
    result.reason = result.optimize.reason;
    result.detail = result.optimize.detail;
    return finish();
  }

  // -------------------------------------------------------------------------
  // 独立验证门禁。
  // -------------------------------------------------------------------------
  announce(stage::kValidate);
  if (is_cancelled()) {
    return cancel_result("cancelled before the validation stage");
  }
  validation::ValidationRequest validation_request;
  validation_request.trajectory = result.optimize.trajectory;
  validation_request.vehicle = request.vehicle;
  validation_request.limits.max_speed_mps =
      std::max(optimize_request.limits.max_forward_velocity_mps,
               optimize_request.limits.max_backward_velocity_mps);
  validation_request.limits.max_acceleration_mps2 =
      std::max(optimize_request.limits.max_forward_acceleration_mps2,
               optimize_request.limits.max_backward_acceleration_mps2);
  validation_request.limits.max_deceleration_mps2 =
      std::max(optimize_request.limits.max_forward_acceleration_mps2,
               optimize_request.limits.max_backward_acceleration_mps2);
  validation_request.limits.max_curvature_inv_m =
      std::max(optimize_request.limits.max_forward_curvature_inv_m,
               optimize_request.limits.max_backward_curvature_inv_m);
  validation_request.goal = request.goal;
  validation_request.scene = scene;
  validation_request.corridor = result.corridor.corridor;

  const auto validation_start = now();
  result.validation = validation::validate_trajectory(validation_request);
  result.validation_time_s = elapsed_s(validation_start, now());
  result.validation_ran = true;
  if (is_cancelled()) {
    return cancel_result("cancelled after the validation stage");
  }
  if (!result.validation.valid) {
    result.status = common::Status::kValidationFailed;
    result.reason = "validation_" + result.validation.reason;
    result.detail = result.validation.detail;
    return finish();
  }

  result.status = common::Status::kSuccess;
  result.reason.clear();
  result.detail.clear();
  result.stage = stage::kDone;
  return finish();
}

}  // namespace planner::service
