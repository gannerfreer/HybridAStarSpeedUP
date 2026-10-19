// 后端注册表：共享搜索核心的配置定义。
//
// 各配置复现锁定的上游搜索结构/参数（docs/upstream_notes.md；ROS-free
// 重写，`native` 类别仅供参考），以及遵循统一契约、使用正式 planner::rs +
// planner::collision 组件的归一化候选。

#include "planner/hybrid_astar/backend.hpp"

#include <string>
#include <utility>
#include <vector>

#include "planner/hybrid_astar/ros_free_backend.hpp"
#include "planner/hybrid_astar/user_astar_backend.hpp"

namespace planner::hybrid_astar {

const char* to_string(BackendKind kind) noexcept {
  switch (kind) {
    case BackendKind::kNative:
      return "native";
    case BackendKind::kNormalized:
      return "normalized";
  }
  return "unknown";
}

namespace {

// 添加一个参数表条目；`target` 非空时该键同时是映射到该规范核心键的
// 受支持别名。
void add_parameter(BackendProfile* profile, const std::string& name,
                   const std::string& note, const std::string& target = "") {
  const bool supported = !target.empty();
  profile->parameters.push_back(ParameterSupport{name, supported, note});
  if (supported) {
    profile->aliases.emplace_back(name, target);
  }
}

BackendProfile smac_profile(BackendKind kind) {
  const bool native = kind == BackendKind::kNative;
  BackendProfile profile;
  profile.id = native ? "smac_native" : "smac_normalized";
  profile.kind = kind;
  profile.description =
      native
          ? "Nav2 Smac (nav2_smac_planner @ bcc9c48) structural reproduction "
            "(ROS-free rewrite; reference only): 64 heading bins, chord "
            "primitives from the minimum turning radius, penalties "
            "2.1/1.2/0.0, analytic ratio 3.5 and 3 m length cap"
          : "Smac-style normalized candidate: upstream structure with formal "
            "planner::rs + planner::collision and the frozen normalized "
            "parameter group";
  profile.defaults.xy_resolution_m = 0.5;
  profile.defaults.heading_bins = 64;
  profile.defaults.motion_step_m = 1.415;        // R = 5.125 m 的弦长
  profile.defaults.reverse_motion_step_m = 1.415;
  profile.defaults.steer_fractions =
      native ? std::vector<double>{-1.0, 0.0, 1.0}
             : std::vector<double>{-1.0, -0.5, 0.0, 0.5, 1.0};
  profile.defaults.reverse_cost = 2.1;
  profile.defaults.turn_penalty = 1.2;
  profile.defaults.gear_change_cost = 0.0;
  profile.defaults.steer_cost = 0.0;
  profile.defaults.forward_cost = 1.0;
  profile.defaults.grid_weight = 1.0;
  profile.defaults.analytic_expansion_ratio = native ? 3 : 1;
  profile.defaults.analytic_expansion_max_length_m = native ? 3.0 : 50.0;
  profile.defaults.analytic_shot_distance_m = 0.0;
  profile.defaults.default_node_budget = 1000000;
  profile.parameters = core_parameter_support();
  add_parameter(&profile, "angle_quantization_bins",
                "Smac heading bins; maps to heading_bins", "heading_bins");
  add_parameter(&profile, "minimum_turning_radius",
                "Smac primitive radius; maps to min_turn_radius_m (must not "
                "be below the request vehicle limit)",
                "min_turn_radius_m");
  add_parameter(&profile, "reverse_penalty",
                "Smac reverse penalty; maps to reverse_cost", "reverse_cost");
  add_parameter(&profile, "change_penalty",
                "Smac gear-change penalty; maps to gear_change_cost",
                "gear_change_cost");
  add_parameter(&profile, "non_straight_penalty",
                "Smac non-straight penalty; maps to turn_penalty",
                "turn_penalty");
  add_parameter(&profile, "cost_penalty",
                "Smac obstacle-heuristic cost weight; maps to grid_weight",
                "grid_weight");
  add_parameter(&profile, "analytic_expansion_ratio",
                "Smac analytic ratio (rounded to the nearest integer); maps "
                "to analytic_expansion_ratio",
                "analytic_expansion_ratio");
  add_parameter(&profile, "analytic_expansion_max_length",
                "Smac analytic length cap; maps to "
                "analytic_expansion_max_length_m",
                "analytic_expansion_max_length_m");
  add_parameter(&profile, "max_iterations",
                "Smac search iteration cap; maps to default_node_budget",
                "default_node_budget");
  add_parameter(&profile, "retrospective_penalty",
                "not implemented in this core (no retrospective travel cost)");
  add_parameter(&profile, "use_quadratic_cost_penalty",
                "not implemented; the grid term is linear in distance");
  add_parameter(&profile, "cache_obstacle_heuristic",
                "always cached per (map, goal) epoch via reset_cache()");
  add_parameter(&profile, "allow_primitive_interpolation",
                "not implemented; primitives use fixed steering grades");
  profile.notes.push_back(
      "reference-only reproduction; upstream collision/RS are replaced by the "
      "formal project components (ROS-free rewrite, no upstream code copied)");
  return profile;
}

BackendProfile kth_profile(BackendKind kind) {
  const bool native = kind == BackendKind::kNative;
  BackendProfile profile;
  profile.id = native ? "kth_native" : "kth_normalized";
  profile.kind = kind;
  profile.description =
      native
          ? "KTH path_planner @ ddf1a3a structural reproduction (ROS-free "
            "rewrite; reference only): 72 headings, 0.707 m primitives with "
            "R = 6 m, penalties 1.05/2.0/2.0, 30000-iteration cap"
          : "KTH-style normalized candidate: upstream structure with formal "
            "planner::rs + planner::collision and the frozen normalized "
            "parameter group";
  profile.defaults.xy_resolution_m = 0.5;  // KTH cellSize 已减小（见说明）
  profile.defaults.heading_bins = 72;
  profile.defaults.motion_step_m = 0.7068582;
  profile.defaults.reverse_motion_step_m = 0.7068582;
  profile.defaults.steer_fractions =
      native ? std::vector<double>{-1.0, 0.0, 1.0}
             : std::vector<double>{-1.0, -0.5, 0.0, 0.5, 1.0};
  // Native 复现上游固定的 R = 6 m；归一化候选使用请求车辆自身的最小
  // 转弯半径。
  profile.defaults.min_turn_radius_m = native ? 6.0 : 0.0;
  profile.defaults.turn_penalty = 1.05;
  profile.defaults.reverse_cost = 2.0;
  profile.defaults.gear_change_cost = 2.0;
  profile.defaults.steer_cost = 0.0;
  profile.defaults.forward_cost = 1.0;
  profile.defaults.grid_weight = 1.0;
  profile.defaults.analytic_expansion_ratio = 1;
  profile.defaults.analytic_expansion_max_length_m = native ? 10.0 : 50.0;
  profile.defaults.analytic_shot_distance_m = native ? 10.0 : 0.0;
  profile.defaults.default_node_budget = native ? 30000 : 1000000;
  profile.parameters = core_parameter_support();
  add_parameter(&profile, "cellSize",
                "KTH 2D cell size; maps to xy_resolution_m (the reproduction "
                "uses 0.5 m instead of the upstream 1.0 m cell because the "
                "shared core requires primitives to leave the dedup cell; "
                "upstream handles same-cell successors with a tie-breaker)",
                "xy_resolution_m");
  add_parameter(&profile, "headings", "KTH heading bins; maps to heading_bins",
                "heading_bins");
  add_parameter(&profile, "penaltyTurning",
                "KTH turning penalty; maps to turn_penalty", "turn_penalty");
  add_parameter(&profile, "penaltyReversing",
                "KTH reversing penalty; maps to reverse_cost", "reverse_cost");
  add_parameter(&profile, "penaltyCOD",
                "KTH change-of-direction penalty; additive approximation maps "
                "to gear_change_cost",
                "gear_change_cost");
  add_parameter(&profile, "iterations",
                "KTH iteration cap; maps to default_node_budget",
                "default_node_budget");
  add_parameter(&profile, "factor2D",
                "KTH 2D heuristic factor; plain multiplier maps to grid_weight",
                "grid_weight");
  add_parameter(&profile, "dubinsShotDistance",
                "KTH forward-shot gate interpreted as an RS connection "
                "distance gate; maps to analytic_shot_distance_m",
                "analytic_shot_distance_m");
  add_parameter(&profile, "dubinsShot",
                "upstream forward-only Dubins shot; this core uses full RS "
                "analytic connections instead (documented deviation)");
  add_parameter(&profile, "dubins",
                "upstream Dubins heuristic toggle; this core always uses the "
                "RS distance plus grid potential");
  add_parameter(&profile, "dubinsLookup",
                "not implemented (upstream lookup heuristic)");
  add_parameter(&profile, "dubinsStepSize",
                "upstream Dubins sampling step; RS sampling uses the core "
                "sample step");
  add_parameter(&profile, "twoD",
                "upstream 2D heuristic toggle; set grid_weight to 0 instead");
  add_parameter(&profile, "bloating",
                "vehicle margin comes from the request VehicleModel");
  add_parameter(&profile, "width", "vehicle parameters come from PlanRequest");
  add_parameter(&profile, "length", "vehicle parameters come from PlanRequest");
  add_parameter(&profile, "r", "vehicle parameters come from PlanRequest");
  add_parameter(&profile, "tieBreaker",
                "not parameterized; the frontier tie-breaks on h");
  profile.notes.push_back(
      "reference-only reproduction; upstream collision/Dubins are replaced by "
      "the formal project components (ROS-free rewrite, no upstream code "
      "copied)");
  profile.notes.push_back(
      "dedup cell 0.5 m (upstream cellSize 1.0 m): the shared core requires "
      "primitive endpoints to leave the dedup cell, while upstream handles "
      "same-cell successors with a tie-breaker");
  return profile;
}

BackendProfile zm0612_profile(BackendKind kind) {
  const bool native = kind == BackendKind::kNative;
  BackendProfile profile;
  profile.id = native ? "zm0612_native" : "zm0612_normalized";
  profile.kind = kind;
  profile.description =
      native
          ? "zm0612/Hybrid_A_Star @ cf0cc99 structural reproduction (ROS-free "
            "rewrite; reference only): 72 headings, 3 steering grades, 1.6 m "
            "segments, RS shot <= 5 m, penalties 1.05/2.0/1.5"
          : "zm0612-style normalized candidate: upstream structure with "
            "formal planner::rs + planner::collision and the frozen "
            "normalized parameter group";
  profile.defaults.xy_resolution_m = 0.5;
  profile.defaults.heading_bins = 72;
  profile.defaults.motion_step_m = native ? 1.6 : 2.0;
  profile.defaults.reverse_motion_step_m = native ? 1.6 : 1.5;
  profile.defaults.steer_fractions =
      native ? std::vector<double>{-1.0, 0.0, 1.0}
             : std::vector<double>{-1.0, -0.75, -0.5, -0.25, 0.0,
                                   0.25, 0.5, 0.75, 1.0};
  profile.defaults.turn_penalty = 1.05;
  profile.defaults.reverse_cost = 2.0;
  profile.defaults.steer_change_cost = 1.5;
  profile.defaults.gear_change_cost = 0.0;
  profile.defaults.steer_cost = 0.0;
  profile.defaults.forward_cost = 1.0;
  profile.defaults.grid_weight = 1.0;
  profile.defaults.analytic_expansion_ratio = 1;
  profile.defaults.analytic_expansion_max_length_m = native ? 5.0 : 50.0;
  profile.defaults.analytic_shot_distance_m = native ? 5.0 : 0.0;
  profile.defaults.default_node_budget = 1000000;
  profile.parameters = core_parameter_support();
  add_parameter(&profile, "segment_length",
                "zm0612 segment length; maps to motion_step_m (the reverse "
                "step keeps its own key)",
                "motion_step_m");
  add_parameter(&profile, "steering_penalty",
                "zm0612 steering penalty; maps to turn_penalty",
                "turn_penalty");
  add_parameter(&profile, "reversing_penalty",
                "zm0612 reversing penalty; maps to reverse_cost",
                "reverse_cost");
  add_parameter(&profile, "steering_change_penalty",
                "zm0612 steering-change penalty; maps to steer_change_cost",
                "steer_change_cost");
  add_parameter(&profile, "shot_distance",
                "zm0612 RS shot distance; maps to analytic_shot_distance_m",
                "analytic_shot_distance_m");
  add_parameter(&profile, "state_grid_resolution",
                "zm0612 state grid resolution; maps to xy_resolution_m",
                "xy_resolution_m");
  add_parameter(&profile, "grid_size_phi",
                "zm0612 heading discretization; maps to heading_bins",
                "heading_bins");
  add_parameter(&profile, "wheel_base",
                "vehicle parameters come from PlanRequest");
  add_parameter(&profile, "steering_angle",
                "vehicle parameters come from PlanRequest");
  add_parameter(&profile, "steering_angle_discrete_num",
                "steering grades are set via steer_fractions");
  add_parameter(&profile, "segment_length_discrete_num",
                "integration substeps are not parameterized; primitives are "
                "swept-checked at the frozen collision precision");
  profile.notes.push_back(
      "reference-only reproduction; upstream costmap checks are replaced by "
      "the formal project components (ROS-free rewrite, no upstream code "
      "copied)");
  if (!native) {
    profile.notes.push_back(
        "Wave 6 frozen-config change for path quality: 9 steering samples and "
        "2.0/1.5 m steps (was 5 samples, 1.6/1.6 m); deadend_124 ratio "
        "1.181 -> 1.000; obstacle_109 stays at 1.105 (>1.10) and is recorded "
        "as pending optimization in selection.json");
  }
  return profile;
}

std::vector<BackendProfile> all_backend_profiles() {
  return {ros_free_profile(),
          smac_profile(BackendKind::kNative),
          smac_profile(BackendKind::kNormalized),
          kth_profile(BackendKind::kNative),
          kth_profile(BackendKind::kNormalized),
          zm0612_profile(BackendKind::kNative),
          zm0612_profile(BackendKind::kNormalized)};
}

}  // namespace

std::vector<BackendDescriptor> list_backends() {
  std::vector<BackendDescriptor> backends;
  for (const BackendProfile& profile : all_backend_profiles()) {
    backends.push_back(
        BackendDescriptor{profile.id, profile.kind, profile.description});
  }
  // 用户自研 HybridAstar（vendored；见 upstream/user_astar/PROVENANCE.md）。
  backends.push_back(BackendDescriptor{
      "user_astar_raw", BackendKind::kNative,
      "user HybridAstar (author repo b6e5383a) raw behaviour: internal grid "
      "collision, original reverse kinematics, upstream goal tolerance, no RS "
      "connection; reference only"});
  backends.push_back(BackendDescriptor{
      "user_astar_normalized", BackendKind::kNormalized,
      "user HybridAstar (author repo b6e5383a) normalized: standard reverse "
      "kinematics + planner::collision checks + planner::rs goal connection + "
      "budgets/cancellation"});
  return backends;
}

std::unique_ptr<HybridAStarBackend> create_backend(const std::string& id) {
  if (id == "user_astar_raw") {
    return std::make_unique<UserAstarBackend>(BackendKind::kNative);
  }
  if (id == "user_astar_normalized") {
    return std::make_unique<UserAstarBackend>(BackendKind::kNormalized);
  }
  for (const BackendProfile& profile : all_backend_profiles()) {
    if (profile.id == id) {
      return std::unique_ptr<HybridAStarBackend>(new RosFreeBackend(profile));
    }
  }
  return nullptr;
}

}  // namespace planner::hybrid_astar
