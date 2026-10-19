// 用户自研 HybridAstar 适配后端实现（上游来源与改写见
// upstream/user_astar/PROVENANCE.md）。
//
//   * user_astar_raw        (kNative)     - 原样行为：内部栅格碰撞（由请求
//     栅格栅格化）、原始倒车运动学、内置目标容差、无 RS 连接；参考项。
//   * user_astar_normalized (kNormalized) - 修正倒车运动学 + 公共
//     planner::collision 位姿/运动检查 + 公共 planner::rs 目标连接 +
//     预算/取消 + 成功后端校验；参与选型。

#include "planner/hybrid_astar/user_astar_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <astar/hybrid_astar.h>

#include "grid_potential.hpp"
#include "planner/collision/scene.hpp"
#include "planner/rs/reeds_shepp.hpp"

namespace planner::hybrid_astar {

namespace {

using Clock = std::chrono::steady_clock;

constexpr double kPi = 3.14159265358979323846;
constexpr double kStartPoseToleranceM = 1e-6;
constexpr double kStartYawToleranceRad = 1e-6;
constexpr double kEndpointPositionToleranceM = 0.05;  // §7.1
constexpr double kEndpointYawToleranceRad = 0.02;     // §7.1
constexpr double kCurvatureTolerance = 1e-9;
constexpr double kPoseEqualityTolerance = 1e-9;

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
  out.yaw_rad = common::normalize_angle_rad(pose.yaw);
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

// 栅格快照完全一致（标识、几何、单元值）时返回 true。归一化模式用它确认
// 目标势场使用的栅格就是碰撞检查器的场景栅格（与共享核心同一前置条件）。
bool grids_equal(const common::GridView& a, const common::GridView& b) {
  return a.map_id == b.map_id && a.map_version == b.map_version &&
         a.width == b.width && a.height == b.height &&
         a.resolution_m == b.resolution_m && a.origin == b.origin &&
         a.cells == b.cells;
}

// 剔除 RS 连接中的数值退化分段（远小于毫米级）：这些微米级位移来自 RS
// 字分解的边界情况，没有几何意义，却会制造虚假换向并扰动端点。长度按
// 剩余分段重算，start/goal 回显保持不变。
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

// 由相邻节点位姿反推常曲率基元的曲率：curvature = Δyaw / signed_arc。
// 上游 `Propagate` 的每段基元弧长恒为 delta_dist（前进/倒车相同），
// 因此必须按弧长而不是弦长换算（弦长会把曲率系统性放大 β²/24）。
// 原代码的基元归属一致（方向取到达节点的方向）；倒车在 raw 模式下与
// 原实现一致（Δyaw 为正、signed_arc 为负，因此曲率符号镜像），在
// normalized 模式下为修正后的标准自行车模型。
double primitive_curvature(const astar::Pose2D& from,
                           const astar::Pose2D& to, double arc_length_m) {
  if (arc_length_m <= 1e-12) {
    return 0.0;
  }
  const double dyaw = common::normalize_angle_rad(to.yaw - from.yaw);
  const double signed_arc = to.direction == astar::MotionDirection::Forward
                                ? arc_length_m
                                : -arc_length_m;
  return dyaw / signed_arc;
}

// ---------------------------------------------------------------------------
// 配置（规范键 + 上游 ini 键别名）。
// ---------------------------------------------------------------------------

struct UserAstarConfig {
  double xy_resolution_m = 0.5;
  int heading_bins = 72;
  double motion_step_m = 1.0;
  int steering_samples = 5;
  bool allow_reverse = true;
  double forward_cost = 1.0;
  double reverse_cost = 2.0;
  double turn_penalty = 1.0;
  double switch_cost = 2.0;
  double backward_search_range_m = 0.0;  // 0 = 不限制
  double linear_preferred_distance_square = 0.0;
  double goal_yaw_tolerance_deg = 1.0;
  int analytic_expansion_ratio = 3;
  double analytic_shot_distance_m = 0.0;        // 0 = 无距离限制
  // RS 连接长度上限（默认 11.8 m）：拒绝 park_116/118 的贪心长连接
  // （12.0–12.65 m）并迫使搜索靠近后再连接，同时保留 ≤11.65 m 的次优
  // 连接；0 = 无长度限制（完整性优先）。
  double analytic_expansion_max_length_m = 11.8;
  double goal_capture_radius_m = 8.0;
  double min_connector_segment_m = 0.15;  // RS 连接中低于该长度的分段视为数值
                                          // 退化（会制造虚假换向），拒绝后重试
  double default_time_budget_s = 5.0;
  std::int64_t default_node_budget = 1000000;
  bool smooth = false;
};

enum class Field {
  kXyResolution,
  kHeadingBins,
  kMotionStep,
  kSteeringSamples,
  kAllowReverse,
  kForwardCost,
  kReverseCost,
  kTurnPenalty,
  kSwitchCost,
  kBackwardRange,
  kLinearPreferred,
  kGoalYawTolerance,
  kAnalyticRatio,
  kAnalyticShot,
  kAnalyticMaxLength,
  kGoalCapture,
  kMinConnectorSegment,
  kDefaultTimeBudget,
  kDefaultNodeBudget,
  kSmooth,
};

struct ParameterRow {
  std::string name;
  bool supported{false};
  std::string note;
};

const std::vector<std::pair<std::string, Field>>& field_table() {
  static const std::vector<std::pair<std::string, Field>> table = {
      {"xy_resolution_m", Field::kXyResolution},
      {"heading_bins", Field::kHeadingBins},
      {"motion_step_m", Field::kMotionStep},
      {"steering_samples", Field::kSteeringSamples},
      {"allow_reverse", Field::kAllowReverse},
      {"forward_cost", Field::kForwardCost},
      {"reverse_cost", Field::kReverseCost},
      {"turn_penalty", Field::kTurnPenalty},
      {"switch_cost", Field::kSwitchCost},
      {"backward_search_range_m", Field::kBackwardRange},
      {"linear_preferred_distance_square", Field::kLinearPreferred},
      {"goal_yaw_tolerance_deg", Field::kGoalYawTolerance},
      {"analytic_expansion_ratio", Field::kAnalyticRatio},
      {"analytic_shot_distance_m", Field::kAnalyticShot},
      {"analytic_expansion_max_length_m", Field::kAnalyticMaxLength},
      {"goal_capture_radius_m", Field::kGoalCapture},
      {"min_connector_segment_m", Field::kMinConnectorSegment},
      {"default_time_budget_s", Field::kDefaultTimeBudget},
      {"default_node_budget", Field::kDefaultNodeBudget},
      {"smooth", Field::kSmooth}};
  return table;
}

const std::vector<std::pair<std::string, std::string>>& alias_table() {
  // 上游 ini 键 -> 规范键（同单位/同义）。
  static const std::vector<std::pair<std::string, std::string>> table = {
      {"grid_dist", "xy_resolution_m"},
      {"delta_dist", "motion_step_m"},
      {"angle_discrete_num", "steering_samples"},
      {"max_iterations_astar", "default_node_budget"},
      {"forward_penalty", "forward_cost"},
      {"backward_penalty", "reverse_cost"},
      {"turnning_penalty", "turn_penalty"},
      {"switch_penalty", "switch_cost"},
      {"backward_search_range", "backward_search_range_m"}};
  return table;
}

std::vector<ParameterRow> build_parameter_rows(bool raw_mode) {
  std::vector<ParameterRow> rows;
  const auto note_for = [raw_mode](const std::string& key) {
    if (key == "xy_resolution_m") {
      return std::string(
          "状态去重栅格；raw 模式同时用作内部碰撞栅格的格子边长");
    }
    if (key == "heading_bins") {
      return std::string("航向离散份数（上游 grid_angle 的等价表达）");
    }
    if (key == "motion_step_m") {
      return std::string("前进/倒车基元步长（上游两向共用 delta_dist）");
    }
    if (key == "steering_samples") {
      return std::string(
          "转向角采样数；转向范围由 PlanRequest 车辆的 max_steer_rad 决定");
    }
    if (key == "allow_reverse") {
      return std::string("false 时不生成倒车原语");
    }
    if (key == "backward_search_range_m") {
      return std::string(">0 时仅在起点该半径内生成倒车原语；0 = 不限制");
    }
    if (key == "linear_preferred_distance_square") {
      return std::string("起点附近转向代价放大区的距离平方阈值 [m^2]");
    }
    if (key == "goal_yaw_tolerance_deg") {
      return raw_mode
                 ? std::string("raw 内置目标航向容差")
                 : std::string("normalized 使用 RS 目标连接，本键不参与判定");
    }
    if (key == "analytic_expansion_ratio") {
      return std::string("normalized 每 N 次目标钩子调用尝试一次 RS 连接");
    }
    if (key == "analytic_shot_distance_m") {
      return std::string(">0 时仅在该距离内尝试 RS 连接；0 = 不限制");
    }
    if (key == "analytic_expansion_max_length_m") {
      return std::string(
          "RS 连接长度上限（默认 11.8 m，质量/完整性折衷）；0 = 不限制");
    }
    if (key == "goal_capture_radius_m") {
      return std::string("normalized 进入该半径后允许尝试 RS 目标连接");
    }
    if (key == "min_connector_segment_m") {
      return raw_mode
                 ? std::string("raw 模式无 RS 连接，本键不参与")
                 : std::string(
                       "RS 连接中短于该长度的分段视为数值退化（虚假换向），"
                       "拒绝该连接并继续搜索；0 = 不拒绝");
    }
    if (key == "smooth") {
      return std::string("上游无平滑；只接受 false");
    }
    return std::string();
  };
  for (const auto& entry : field_table()) {
    rows.push_back(ParameterRow{entry.first, true, note_for(entry.first)});
  }
  for (const auto& alias : alias_table()) {
    rows.push_back(ParameterRow{alias.first, true,
                                "上游别名，映射到 " + alias.second});
  }
  // 明确不支持的键（设置即 invalid_input，不静默忽略）。
  rows.push_back(ParameterRow{"grid_angle", true,
                              "上游角度分辨率（度）；映射到 heading_bins "
                              "= round(360 / grid_angle)"});
  rows.push_back(ParameterRow{
      "min_steering", false, "转向范围来自 PlanRequest 车辆的 max_steer_rad"});
  rows.push_back(ParameterRow{
      "max_steering", false, "转向范围来自 PlanRequest 车辆的 max_steer_rad"});
  rows.push_back(ParameterRow{"wheel_base", false,
                              "车辆参数来自 PlanRequest",
                              });
  rows.push_back(ParameterRow{
      "boundary_inflation_meter", false,
      "障碍来自 PlanRequest 栅格/公共碰撞检查，无独立边界膨胀"});
  rows.push_back(ParameterRow{"safe_margin", false,
                              "车辆安全裕量来自 PlanRequest"});
  rows.push_back(ParameterRow{"center2front", false,
                              "车辆参数来自 PlanRequest"});
  rows.push_back(ParameterRow{"center2rear", false,
                              "车辆参数来自 PlanRequest"});
  rows.push_back(ParameterRow{"center2side", false,
                              "车辆参数来自 PlanRequest"});
  return rows;
}

bool json_number(const nlohmann::json& value, double* out) {
  if (value.is_number()) {
    *out = value.get<double>();
    return true;
  }
  return false;
}

bool json_integer(const nlohmann::json& value, std::int64_t* out) {
  if (value.is_number_integer()) {
    *out = value.get<std::int64_t>();
    return true;
  }
  if (value.is_number()) {
    const double as_double = value.get<double>();
    if (std::isfinite(as_double) &&
        as_double == std::floor(as_double)) {
      *out = static_cast<std::int64_t>(as_double);
      return true;
    }
  }
  return false;
}

}  // namespace

// ---------------------------------------------------------------------------
// Impl
// ---------------------------------------------------------------------------

struct UserAstarBackend::Impl {
  explicit Impl(BackendKind backend_kind) : kind(backend_kind), raw(backend_kind == BackendKind::kNative) {
    parameters = build_parameter_rows(raw);
    effective = effective_json();
  }

  nlohmann::json effective_json() const {
    nlohmann::json out = nlohmann::json::object();
    out["xy_resolution_m"] = config.xy_resolution_m;
    out["heading_bins"] = config.heading_bins;
    out["motion_step_m"] = config.motion_step_m;
    out["steering_samples"] = config.steering_samples;
    out["allow_reverse"] = config.allow_reverse;
    out["forward_cost"] = config.forward_cost;
    out["reverse_cost"] = config.reverse_cost;
    out["turn_penalty"] = config.turn_penalty;
    out["switch_cost"] = config.switch_cost;
    out["backward_search_range_m"] = config.backward_search_range_m;
    out["linear_preferred_distance_square"] =
        config.linear_preferred_distance_square;
    out["goal_yaw_tolerance_deg"] = config.goal_yaw_tolerance_deg;
    out["analytic_expansion_ratio"] = config.analytic_expansion_ratio;
    out["analytic_shot_distance_m"] = config.analytic_shot_distance_m;
    out["analytic_expansion_max_length_m"] =
        config.analytic_expansion_max_length_m;
    out["goal_capture_radius_m"] = config.goal_capture_radius_m;
    out["min_connector_segment_m"] = config.min_connector_segment_m;
    out["default_time_budget_s"] = config.default_time_budget_s;
    out["default_node_budget"] = config.default_node_budget;
    out["smooth"] = config.smooth;
    return out;
  }

  BackendKind kind{BackendKind::kNormalized};
  bool raw{false};
  UserAstarConfig config;
  std::vector<ParameterRow> parameters;
  nlohmann::json effective = nlohmann::json::object();
  ConfigReport report;
  std::string unsupported_reason;
  std::function<bool()> cancel_callback;

  // 每次 plan 调用的计数（统计输出）。
  std::int64_t expanded_nodes{0};
  std::int64_t collision_checks{0};
  std::int64_t motion_primitives{0};
  std::int64_t motion_rejections{0};
  std::int64_t rs_connection_attempts{0};
  std::int64_t pruned_unreachable{0};

  // 目标势场缓存（仅 normalized 模式；与共享核心同一封装）。键覆盖栅格
  // 标识/几何/单元值/目标单元；configure()/reset_cache() 使其失效。
  bool potential_valid{false};
  std::uint64_t potential_key{0};
  internal::GridPotential potential;
};

std::string UserAstarBackend::id() const {
  return impl_->raw ? "user_astar_raw" : "user_astar_normalized";
}

UserAstarBackend::UserAstarBackend(BackendKind kind)
    : impl_(std::make_unique<Impl>(kind)) {}

UserAstarBackend::~UserAstarBackend() = default;

BackendKind UserAstarBackend::kind() const noexcept { return impl_->kind; }

BackendInfo UserAstarBackend::info() const {
  BackendInfo info;
  info.id = id();
  info.kind = impl_->kind;
  info.description =
      impl_->raw
          ? "user HybridAstar (author repo b6e5383a): raw behaviour "
            "(internal grid collision, original reverse kinematics, upstream "
            "goal tolerance); reference only"
          : "user HybridAstar (author repo b6e5383a): normalized (standard "
            "reverse kinematics + planner::collision + planner::rs goal "
            "connection + budgets/cancellation)";
  for (const ParameterRow& row : impl_->parameters) {
    info.parameters.push_back(ParameterSupport{row.name, row.supported, row.note});
  }
  const auto stat = [](const std::string& name, bool supported,
                       const std::string& note) {
    return ParameterSupport{name, supported, note};
  };
  info.statistics = {
      stat("expanded_nodes", true, "搜索迭代数（上游 iterations）"),
      stat("collision_checks", true,
           impl_->raw ? "raw 模式使用内部栅格碰撞，不计数公共检查"
                      : "公共位姿/运动检查次数"),
      stat("motion_primitives", !impl_->raw,
           impl_->raw ? "raw 模式不单独计数"
                      : "运动原语检查次数"),
      stat("motion_rejections", !impl_->raw,
           impl_->raw ? "raw 模式不单独计数" : "被碰撞检查拒绝的原语数"),
      stat("rs_connection_attempts", !impl_->raw,
           impl_->raw ? "raw 模式无 RS 连接"
                      : "RS 目标连接尝试次数"),
      stat("pruned_unreachable_states", !impl_->raw,
           impl_->raw
               ? "raw 模式无势能剪枝"
               : "被自由栅格可达性松弛剪枝的后继状态数"),
      stat("search_time_ms", true, "搜索挂钟时间"),
      stat("postprocess_time_ms", true, "路径构建/校验时间"),
      stat("total_time_ms", true, "单次 plan 总时间"),
      stat("path_length_m", true, "输出路径总长"),
      stat("gear_changes", true, "换向次数"),
      stat("endpoint_position_error_m", true, "终点位置误差"),
      stat("endpoint_yaw_error_rad", true, "终点航向误差"),
      stat("cache_mode", true,
           impl_->raw
               ? "raw 模式无跨调用缓存；恒为 cold"
               : "目标势能跨调用缓存：cold = 本次调用计算，warm = 复用")};
  return info;
}

ConfigReport UserAstarBackend::configure(const nlohmann::json& config) {
  ConfigReport report;
  report.accepted = true;
  impl_->unsupported_reason.clear();
  impl_->config = UserAstarConfig{};  // 上游默认冻结值

  const auto fail = [&report](const std::string& reason) {
    report.accepted = false;
    report.reason = reason;
  };
  const auto apply_double = [&](Field field, double value) {
    switch (field) {
      case Field::kXyResolution:
        if (!(value > 0.0)) return false;
        impl_->config.xy_resolution_m = value;
        return true;
      case Field::kMotionStep:
        if (!(value > 0.0)) return false;
        impl_->config.motion_step_m = value;
        return true;
      case Field::kForwardCost:
        if (!(value >= 0.0)) return false;
        impl_->config.forward_cost = value;
        return true;
      case Field::kReverseCost:
        if (!(value >= 0.0)) return false;
        impl_->config.reverse_cost = value;
        return true;
      case Field::kTurnPenalty:
        if (!(value >= 0.0)) return false;
        impl_->config.turn_penalty = value;
        return true;
      case Field::kSwitchCost:
        if (!(value >= 0.0)) return false;
        impl_->config.switch_cost = value;
        return true;
      case Field::kBackwardRange:
        if (!(value >= 0.0)) return false;
        impl_->config.backward_search_range_m = value;
        return true;
      case Field::kLinearPreferred:
        if (!(value >= 0.0)) return false;
        impl_->config.linear_preferred_distance_square = value;
        return true;
      case Field::kGoalYawTolerance:
        if (!(value > 0.0)) return false;
        impl_->config.goal_yaw_tolerance_deg = value;
        return true;
      case Field::kAnalyticShot:
        if (!(value >= 0.0)) return false;
        impl_->config.analytic_shot_distance_m = value;
        return true;
      case Field::kAnalyticMaxLength:
        if (!(value >= 0.0)) return false;
        impl_->config.analytic_expansion_max_length_m = value;
        return true;
      case Field::kGoalCapture:
        if (!(value > 0.0)) return false;
        impl_->config.goal_capture_radius_m = value;
        return true;
      case Field::kMinConnectorSegment:
        if (!(value >= 0.0)) return false;
        impl_->config.min_connector_segment_m = value;
        return true;
      case Field::kDefaultTimeBudget:
        if (!(value > 0.0)) return false;
        impl_->config.default_time_budget_s = value;
        return true;
      default:
        return false;
    }
  };
  const auto apply_int = [&](Field field, std::int64_t value) {
    switch (field) {
      case Field::kHeadingBins:
        if (value < 8) return false;
        impl_->config.heading_bins = static_cast<int>(value);
        return true;
      case Field::kSteeringSamples:
        if (value < 2) return false;
        impl_->config.steering_samples = static_cast<int>(value);
        return true;
      case Field::kAnalyticRatio:
        if (value < 1) return false;
        impl_->config.analytic_expansion_ratio = static_cast<int>(value);
        return true;
      case Field::kDefaultNodeBudget:
        if (value < 1) return false;
        impl_->config.default_node_budget = value;
        return true;
      default:
        return false;
    }
  };

  if (!config.is_object()) {
    fail("configuration must be a JSON object");
    report.effective_config = impl_->effective_json();
    return report;
  }

  const auto field_for = [](const std::string& key) -> const Field* {
    for (const auto& entry : field_table()) {
      if (entry.first == key) {
        return &entry.second;
      }
    }
    return nullptr;
  };
  const auto alias_for = [](const std::string& key) -> const std::string* {
    for (const auto& entry : alias_table()) {
      if (entry.first == key) {
        return &entry.second;
      }
    }
    return nullptr;
  };

  bool smooth_requested = false;
  for (auto it = config.begin(); it != config.end(); ++it) {
    const std::string key = it.key();
    if (key == "grid_angle") {
      double degrees = 0.0;
      if (!json_number(it.value(), &degrees) || !(degrees > 0.0) ||
          degrees > 180.0) {
        fail("invalid value for key 'grid_angle'");
        break;
      }
      impl_->config.heading_bins =
          std::max(8, static_cast<int>(std::lround(360.0 / degrees)));
      report.parameters.push_back(ParameterSupport{
          "grid_angle", true, "上游别名，映射到 heading_bins"});
      continue;
    }
    std::string canonical = key;
    if (const std::string* target = alias_for(key); target != nullptr) {
      canonical = *target;
    }
    const Field* field = field_for(canonical);
    if (field == nullptr) {
      report.unsupported.push_back(key);
      continue;
    }
    bool ok = false;
    switch (*field) {
      case Field::kAllowReverse:
      case Field::kSmooth: {
        if (!it.value().is_boolean()) {
          fail("invalid value for key '" + canonical + "'");
          break;
        }
        const bool value = it.value().get<bool>();
        if (*field == Field::kAllowReverse) {
          impl_->config.allow_reverse = value;
          ok = true;
        } else {
          impl_->config.smooth = value;
          if (value) {
            smooth_requested = true;
          }
          ok = true;
        }
        break;
      }
      case Field::kHeadingBins:
      case Field::kSteeringSamples:
      case Field::kAnalyticRatio:
      case Field::kDefaultNodeBudget: {
        std::int64_t value = 0;
        if (!json_integer(it.value(), &value)) {
          fail("invalid value for key '" + canonical + "'");
          break;
        }
        ok = apply_int(*field, value);
        if (!ok && report.accepted) {
          fail("value out of range for key '" + canonical + "'");
        }
        break;
      }
      default: {
        double value = 0.0;
        if (!json_number(it.value(), &value)) {
          fail("invalid value for key '" + canonical + "'");
          break;
        }
        ok = apply_double(*field, value);
        if (!ok && report.accepted) {
          fail("value out of range for key '" + canonical + "'");
        }
        break;
      }
    }
    if (!report.accepted) {
      break;
    }
    (void)ok;
  }
  if (smooth_requested) {
    report.unsupported.push_back("smooth");
  }
  report.effective_config = impl_->effective_json();
  impl_->effective = report.effective_config;
  for (const ParameterRow& row : impl_->parameters) {
    bool already = false;
    for (const ParameterSupport& existing : report.parameters) {
      if (existing.name == row.name) {
        already = true;
        break;
      }
    }
    if (!already) {
      report.parameters.push_back(
          ParameterSupport{row.name, row.supported, row.note});
    }
  }
  if (!report.unsupported.empty()) {
    std::string joined;
    for (const std::string& key : report.unsupported) {
      if (!joined.empty()) {
        joined += ", ";
      }
      joined += key;
    }
    impl_->unsupported_reason = "unsupported configuration keys: " + joined;
  }
  impl_->report = report;
  reset_cache();
  return report;
}

const nlohmann::json& UserAstarBackend::effective_config() const {
  return impl_->effective;
}

void UserAstarBackend::set_cancel_callback(std::function<bool()> callback) {
  impl_->cancel_callback = std::move(callback);
}

void UserAstarBackend::reset_cache() {
  // 清除目标势能缓存；报告的 cache_mode 在 normalized 模式下按本次调用
  // 是否复用势能给出 cold/warm（raw 模式不使用势能，恒为 cold）。
  impl_->potential_valid = false;
  impl_->potential_key = 0;
  impl_->potential = internal::GridPotential{};
  impl_->expanded_nodes = 0;
  impl_->collision_checks = 0;
  impl_->motion_primitives = 0;
  impl_->motion_rejections = 0;
  impl_->rs_connection_attempts = 0;
  impl_->pruned_unreachable = 0;
}

common::PlanResult UserAstarBackend::plan(
    const common::PlanRequest& request, const common::GridView& grid,
    const collision::CollisionChecker& collision_checker,
    RsProvider& rs_provider) {
  common::PlanResult result;
  result.request_id = request.request_id;

  const auto fail = [&result](common::Status status, const std::string& reason) {
    result.status = status;
    result.reason = reason;
    return result;
  };

  if (!impl_->unsupported_reason.empty()) {
    return fail(common::Status::kInvalidInput, impl_->unsupported_reason);
  }
  if (request.cancelled) {
    return fail(common::Status::kCancelled, "request was cancelled");
  }
  if (request.map_id != grid.map_id ||
      request.map_version != grid.map_version) {
    return fail(common::Status::kInvalidInput,
                "request map_id/map_version does not match the grid snapshot");
  }
  if (!same_vehicle(request.vehicle, collision_checker.vehicle()) ||
      !same_vehicle(request.vehicle, rs_provider.vehicle())) {
    return fail(common::Status::kInvalidInput,
                "request vehicle does not match collision checker / RS "
                "provider vehicle");
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

  impl_->expanded_nodes = 0;
  impl_->collision_checks = 0;
  impl_->motion_primitives = 0;
  impl_->motion_rejections = 0;
  impl_->rs_connection_attempts = 0;
  impl_->pruned_unreachable = 0;

  const Clock::time_point plan_start = Clock::now();
  const double time_budget_s =
      request.time_budget_s.value_or(impl_->config.default_time_budget_s);
  const std::int64_t node_budget =
      request.node_budget.value_or(impl_->config.default_node_budget);
  const Clock::time_point deadline =
      plan_start + std::chrono::duration_cast<Clock::duration>(
                       std::chrono::duration<double>(time_budget_s));

  const auto elapsed_ms = [](Clock::time_point from, Clock::time_point to) {
    return std::chrono::duration<double, std::milli>(to - from).count();
  };

  // 目标势能缓存状态（normalized 模式；raw 恒为 cold 且不使用势能）。
  bool potential_reused = false;

  // 统计公共字段；搜索耗时由 search_ms_value 提供。预检提前返回时该值
  // 记为 plan_start 到返回的耗时（与共享核心的失败路径一致）。
  double search_ms_value = 0.0;
  const auto add_common_statistics = [&](nlohmann::json* stats) {
    (*stats)["expanded_nodes"] = impl_->expanded_nodes;
    if (!impl_->raw) {
      (*stats)["collision_checks"] = impl_->collision_checks;
      (*stats)["motion_primitives"] = impl_->motion_primitives;
      (*stats)["motion_rejections"] = impl_->motion_rejections;
      (*stats)["rs_connection_attempts"] = impl_->rs_connection_attempts;
      (*stats)["pruned_unreachable_states"] = impl_->pruned_unreachable;
    }
    (*stats)["search_time_ms"] = search_ms_value;
    (*stats)["cache_mode"] = potential_reused ? "warm" : "cold";
    (*stats)["reverse_kinematics"] =
        impl_->raw ? "upstream_mirrored" : "standard_bicycle";
    (*stats)["goal_mode"] =
        impl_->raw ? "upstream_tolerance" : "planner_rs_connection";
    (*stats)["collision_mode"] =
        impl_->raw ? "internal_grid_obb" : "planner_collision";
  };
  const auto early_failure = [&](common::Status status,
                                 const std::string& reason) {
    search_ms_value = elapsed_ms(plan_start, Clock::now());
    nlohmann::json stats = nlohmann::json::object();
    add_common_statistics(&stats);
    stats["total_time_ms"] = search_ms_value;
    result.statistics = std::move(stats);
    return fail(status, reason);
  };

  // ------------------------------------------------------------------
  // 上游配置映射。
  // ------------------------------------------------------------------
  astar::PlannerConfig config;
  config.vehicle.center2front =
      request.vehicle.wheelbase_m + request.vehicle.front_overhang_m;
  config.vehicle.center2rear = request.vehicle.rear_overhang_m;
  config.vehicle.center2side = 0.5 * request.vehicle.width_m;
  config.vehicle.safe_margin = request.vehicle.safety_margin_m;
  config.vehicle.wheel_base = request.vehicle.wheelbase_m;
  config.vehicle.boundary_inflation_meter = -1.0;
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
  config.hybrid_astar.goal_yaw_tolerance_deg =
      impl_->config.goal_yaw_tolerance_deg;
  config.hybrid_astar.forward_penalty = impl_->config.forward_cost;
  config.hybrid_astar.backward_penalty = impl_->config.reverse_cost;
  config.hybrid_astar.turnning_penalty = impl_->config.turn_penalty;
  config.hybrid_astar.switch_penalty = impl_->config.switch_cost;
  config.hybrid_astar.backward_search_range =
      impl_->config.backward_search_range_m;
  config.hybrid_astar.linear_preferred_distance_square =
      impl_->config.linear_preferred_distance_square;
  config.hybrid_astar.allow_reverse = impl_->config.allow_reverse;
  config.hybrid_astar.standard_reverse_kinematics = !impl_->raw;

  // ------------------------------------------------------------------
  // 归一化模式：目标势场连通性预检 + 可达性松弛剪枝（复用共享核心的
  // grid_potential 封装，与 ros_free 同一模型）：
  //   * 起/终点位姿先做公共碰撞检查（非法输入优先于 no_path）；
  //   * 势场为仅自由单元上的 8 连通 Dijkstra，按 grid+goal 单元缓存；
  //   * 起点单元在自由栅格图上不可达时直接 no_path（0 扩展、0 原语）；
  //   * 搜索内用同一势场剪枝不可达后继（绝不放松碰撞门槛）。
  // raw 模式保持上游原样行为：不预检、不剪枝、无缓存。
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
  if (!impl_->raw) {
    if (!grids_equal(collision_checker.scene().grid, grid)) {
      return early_failure(
          common::Status::kInvalidInput,
          "the collision checker scene grid does not match the grid argument");
    }
    const collision::CheckResult start_pose_check =
        collision_checker.check_pose(request.start);
    ++impl_->collision_checks;
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
    ++impl_->collision_checks;
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
  }

  astar::HybridAStar planner(config);

  // RS 目标连接的缓存（归一化模式；由 goal hook 填充）。
  bool connector_valid = false;
  rs::RsPath connector;
  std::int64_t goal_hook_calls = 0;

  if (impl_->raw) {
    // 原样模式：把请求栅格的占据/未知单元栅格化进入内部边界集合。
    planner.ClearBoundaries();
    const double cos_origin = std::cos(grid.origin.yaw_rad);
    const double sin_origin = std::sin(grid.origin.yaw_rad);
    for (int row = 0; row < grid.height; ++row) {
      for (int col = 0; col < grid.width; ++col) {
        const int cell = grid.cell(row, col);
        if (cell == common::kCellFree) {
          continue;
        }
        const double local_x = (static_cast<double>(col) + 0.5) * grid.resolution_m;
        const double local_y = (static_cast<double>(row) + 0.5) * grid.resolution_m;
        const double world_x = grid.origin.x_m + cos_origin * local_x -
                               sin_origin * local_y;
        const double world_y = grid.origin.y_m + sin_origin * local_x +
                               cos_origin * local_y;
        planner.AddBoundaryWorldPoint(world_x, world_y);
      }
    }
  } else {
    // 归一化模式：公共碰撞检查接入（位姿 + 运动段）。
    planner.SetPoseCollisionHook([&](const astar::Pose2D& pose) {
      ++impl_->collision_checks;
      const collision::CheckResult check =
          collision_checker.check_pose(to_common(pose));
      return !check.is_clear();
    });
    planner.SetMotionCollisionHook(
        [&](const astar::Pose2D& from, const astar::Pose2D& to) {
          // 可达性松弛剪枝（仅势能可用时）：自由栅格图上不可达的后继单元
          // 不可能是碰撞自由路径的一部分，直接拒绝（见 grid_potential.hpp）。
          if (potential_usable) {
            int row = 0;
            int col = 0;
            if (!cell_of_pose(to_common(to), &row, &col) ||
                !std::isfinite(potential->distance_m[
                    static_cast<std::size_t>(row) *
                        static_cast<std::size_t>(grid.width) +
                    static_cast<std::size_t>(col)])) {
              ++impl_->pruned_unreachable;
              return true;
            }
          }
          ++impl_->motion_primitives;
          const double arc_length = impl_->config.motion_step_m;
          collision::MotionSegment segment;
          segment.start = to_common(from);
          segment.length_m = arc_length;
          segment.direction = to_common(to.direction);
          segment.curvature_inv_m =
              primitive_curvature(from, to, arc_length);
          ++impl_->collision_checks;
          const collision::CheckResult check =
              collision_checker.check_motion(segment);
          if (!check.is_clear()) {
            ++impl_->motion_rejections;
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
      ++impl_->rs_connection_attempts;
      rs::RsPlan solved = rs_provider.plan(to_common(pose), request.goal);
      if (solved.status != common::Status::kSuccess) {
        return false;
      }
      // 数值退化分段（微米级位移）没有几何意义，却会制造虚假换向并污染
      // 端点；先剔除并重算长度，剩余分段仍逐一做完整运动碰撞检查。
      solved = clean_connector(std::move(solved));
      if (impl_->config.analytic_expansion_max_length_m > 0.0 &&
          solved.path.length_m >
              impl_->config.analytic_expansion_max_length_m) {
        return false;
      }
      // 亚原语分段（小于 min_connector_segment_m）视为连接退化：仅当它
      // 与相邻分段构成换向（虚假 cusp）时拒绝该连接，让搜索继续寻找更
      // 干净的目标连接（目标连接策略，非门槛放宽）；同向的短分段无害。
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
      std::vector<common::PathPoint> samples;
      const double sample_step =
          std::max(0.2, 0.5 * impl_->config.motion_step_m);
      try {
        samples = rs::sample(solved.path, sample_step);
      } catch (const std::exception&) {
        return false;
      }
      for (std::size_t k = 0; k + 1 < samples.size(); ++k) {
        const double step = std::hypot(samples[k + 1].pose.x_m - samples[k].pose.x_m,
                                       samples[k + 1].pose.y_m - samples[k].pose.y_m);
        if (step <= 1e-12) {
          continue;
        }
        collision::MotionSegment segment;
        segment.start = samples[k].pose;
        segment.length_m = step;
        segment.direction = samples[k].direction;
        segment.curvature_inv_m = samples[k].curvature_inv_m;
        ++impl_->collision_checks;
        const collision::CheckResult check =
            collision_checker.check_motion(segment);
        if (!check.is_clear()) {
          return false;
        }
      }
      connector_valid = true;
      connector = solved.path;
      return true;
    });
  }

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
  // 搜索。
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
  const Clock::time_point search_start = Clock::now();
  const bool found = planner.Search(start_pose, goal_pose, &search);
  const Clock::time_point search_end = Clock::now();

  impl_->expanded_nodes = search.iterations;
  search_ms_value = elapsed_ms(search_start, search_end);

  if (!found) {
    nlohmann::json stats = nlohmann::json::object();
    add_common_statistics(&stats);
    result.statistics = std::move(stats);
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
  // 路径构建（含 RS 连接）与输出校验。
  // ------------------------------------------------------------------
  const Clock::time_point post_start = Clock::now();
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
      // 上游基元弧长恒为 delta_dist（前进/倒车相同）。
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
  const double path_length = flat.empty() ? 0.0 : flat.back().s_m;
  std::int64_t gear_changes = 0;
  common::Direction previous_direction = common::Direction::kForward;
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
          const double yaw_error = std::abs(common::normalize_angle_rad(
              point.pose.yaw_rad - request.start.yaw_rad));
          if (position_error > kStartPoseToleranceM ||
              yaw_error > kStartYawToleranceRad) {
            structural_error = true;
            structural_reason =
                "output path does not begin at the requested start pose";
            break;
          }
          previous_direction = point.direction;
        } else if (point.direction != previous_direction) {
          ++gear_changes;
          previous_direction = point.direction;
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
  const double endpoint_yaw_error = std::abs(common::normalize_angle_rad(
      endpoint.yaw_rad - request.goal.yaw_rad));

  nlohmann::json stats = nlohmann::json::object();
  add_common_statistics(&stats);
  stats["postprocess_time_ms"] = elapsed_ms(post_start, Clock::now());
  stats["total_time_ms"] = elapsed_ms(plan_start, Clock::now());
  stats["path_length_m"] = path_length;
  stats["gear_changes"] = gear_changes;
  stats["endpoint_position_error_m"] = endpoint_position_error;
  stats["endpoint_yaw_error_rad"] = endpoint_yaw_error;
  result.statistics = std::move(stats);

  if (structural_error) {
    return fail(common::Status::kValidationFailed, structural_reason);
  }

  if (!impl_->raw) {
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
  }

  result.status = common::Status::kSuccess;
  result.reason.clear();
  return result;
}

}  // namespace planner::hybrid_astar
