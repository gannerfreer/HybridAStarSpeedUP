// ros_free 归一化 Hybrid A* 后端（见 ros_free_backend.hpp）。
//
// 搜索结构（与上游参考共享）：
//   * 在 (x, y, heading) 状态单元上的离散状态 A*；
//   * 运动基元由车辆最大转向角生成，并用 planner::rs 辅助函数积分，使
//     基元、RS 连接和输出路径共享冻结的曲率/方向约定；
//   * 启发式函数 h = max(RS 距离, 目标势场距离)，加权的；
//   * 周期性尝试解析 RS 连接（用整车运动扫掠做完整检查），并在节点耗尽
//     后再尝试一次；
//   * 尖点保留为 PathSegment 边界；
//   * 后处理（重建 + 端点/曲率/碰撞复检）计时并计入报告的总时间。

#include "planner/hybrid_astar/ros_free_backend.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "grid_potential.hpp"
#include "planner/collision/scene.hpp"
#include "planner/common/types.hpp"

namespace planner::hybrid_astar {

namespace {

using common::Direction;
using common::PlanRequest;
using common::PlanResult;
using common::Pose2d;
using common::Status;
using common::VehicleModel;

using Clock = std::chrono::steady_clock;

constexpr double kEndpointPositionToleranceM = 0.05;   // Docs §7.1
constexpr double kEndpointYawToleranceRad = 0.02;      // Docs §7.1
constexpr double kCurvatureTolerance = 1e-9;
constexpr double kStartPoseToleranceM = 1e-6;
constexpr double kStartYawToleranceRad = 1e-6;
constexpr std::uint32_t kNoParent = 0xffffffffu;

double elapsed_ms_since(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

bool pose_is_finite(const Pose2d& pose) {
  return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) &&
         std::isfinite(pose.yaw_rad);
}

bool vehicles_equal(const VehicleModel& a, const VehicleModel& b) {
  return a.wheelbase_m == b.wheelbase_m && a.width_m == b.width_m &&
         a.front_overhang_m == b.front_overhang_m &&
         a.rear_overhang_m == b.rear_overhang_m &&
         a.max_steer_rad == b.max_steer_rad &&
         a.safety_margin_m == b.safety_margin_m;
}

bool vehicle_model_valid(const VehicleModel& vehicle) {
  return std::isfinite(vehicle.wheelbase_m) && vehicle.wheelbase_m > 0.0 &&
         std::isfinite(vehicle.width_m) && vehicle.width_m > 0.0 &&
         std::isfinite(vehicle.front_overhang_m) &&
         vehicle.front_overhang_m >= 0.0 &&
         std::isfinite(vehicle.rear_overhang_m) &&
         vehicle.rear_overhang_m >= 0.0 &&
         std::isfinite(vehicle.max_steer_rad) && vehicle.max_steer_rad > 0.0 &&
         vehicle.max_steer_rad < common::kPi / 2.0 &&
         std::isfinite(vehicle.safety_margin_m) &&
         vehicle.safety_margin_m >= 0.0;
}

bool grid_valid(const common::GridView& grid) {
  if (grid.width <= 0 || grid.height <= 0) {
    return false;
  }
  if (!(std::isfinite(grid.resolution_m) && grid.resolution_m > 0.0)) {
    return false;
  }
  if (!pose_is_finite(grid.origin)) {
    return false;
  }
  if (grid.cells.size() !=
      static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height)) {
    return false;
  }
  for (const int value : grid.cells) {
    if (value != common::kCellFree && value != common::kCellOccupied &&
        value != common::kCellUnknown) {
      return false;
    }
  }
  return true;
}

bool grids_equal(const common::GridView& a, const common::GridView& b) {
  return a.map_id == b.map_id && a.map_version == b.map_version &&
         a.width == b.width && a.height == b.height &&
         a.resolution_m == b.resolution_m && a.origin == b.origin &&
         a.cells == b.cells;
}

std::string collision_detail(const collision::CheckResult& check) {
  std::string text = collision::to_string(check.reason_code);
  if (!check.detail.empty()) {
    text += ": " + check.detail;
  }
  return text;
}


std::vector<ParameterSupport> statistic_support() {
  return {
      {"cache_mode", true,
       "cold when the goal potential was computed in this call, warm when "
       "reused from the instance cache"},
      {"expanded_nodes", true, "nodes popped and expanded from the frontier"},
      {"generated_nodes", true, "successor states accepted into the frontier"},
      {"closed_states", true, "unique discretized states marked closed"},
      {"collision_checks", true,
       "whole-vehicle collision checks performed by this backend"},
      {"motion_rejections", true,
       "motion primitives rejected by the collision checker"},
      {"pruned_unreachable_states", true,
       "states pruned by the free-grid reachability relaxation"},
      {"stale_frontier_entries", true,
       "frontier entries skipped because their state was already closed"},
      {"rs_distance_calls", true, "RS distance/heuristic solves"},
      {"rs_connection_attempts", true, "RS analytic-connection attempts"},
      {"search_time_ms", true, "expansion loop wall time in milliseconds"},
      {"postprocess_time_ms", true,
       "path rebuild + re-validation wall time in milliseconds"},
      {"total_time_ms", true, "whole plan() call wall time in milliseconds"},
      {"path_length_m", true, "output path arc length (success only)"},
      {"gear_changes", true,
       "direction changes kept as PathSegment boundaries (success only)"},
      {"endpoint_position_error_m", true,
       "output endpoint position error (success only)"},
      {"endpoint_yaw_error_rad", true,
       "output endpoint yaw error (success only)"},
      {"cpu_ms", false,
       "not instrumented; CPU time is collected by the benchmark harness"},
      {"peak_rss_bytes", false,
       "process-level metric sampled by the task runner resource monitor"},
      {"smooth_time_ms", false, "this backend implements no smoothing"},
  };
}

nlohmann::json unsupported_statistics_json() {
  nlohmann::json entries = nlohmann::json::array();
  for (const ParameterSupport& support : statistic_support()) {
    if (!support.supported) {
      entries.push_back(
          nlohmann::json{{"name", support.name}, {"reason", support.note}});
    }
  }
  return entries;
}

PlanResult base_result(const BackendProfile& profile,
                       const std::string& request_id, Status status,
                       std::string reason) {
  PlanResult result;
  result.request_id = request_id;
  result.status = status;
  result.reason = std::move(reason);
  result.statistics = nlohmann::json{
      {"backend_id", profile.id},
      {"backend_kind", to_string(profile.kind)},
      {"unsupported_statistics", unsupported_statistics_json()}};
  return result;
}

struct Primitive {
  double steer_fraction{0.0};
  double curvature_inv_m{0.0};
  double step_m{0.0};
  Direction direction{Direction::kForward};
};

rs::RsSegment to_rs_segment(const Primitive& primitive) {
  rs::RsSegment segment;
  segment.direction = primitive.direction;
  segment.length_m = primitive.step_m;
  segment.curvature_inv_m = primitive.curvature_inv_m;
  return segment;
}

struct SearchNode {
  Pose2d pose;
  double cost{0.0};        // 加权 A* 的 g 值
  double length_m{0.0};    // 已行驶弧长（非负）
  std::uint32_t parent{kNoParent};
  int primitive{-1};       // 产生该节点的基元；根节点为 -1
  std::uint64_t key{0};    // 离散化状态键
};

struct QueueEntry {
  double f{0.0};
  double h{0.0};
  std::uint32_t node{0};
};

struct QueueEntryGreater {
  bool operator()(const QueueEntry& a, const QueueEntry& b) const {
    if (a.f != b.f) {
      return a.f > b.f;
    }
    return a.h > b.h;
  }
};

// 一次规划运行：持有统计信息、节点存储和搜索循环。
class RosFreeSearch {
 public:
  RosFreeSearch(const RosFreeConfig& config, const PlanRequest& request,
                const common::GridView& grid,
                const collision::CollisionChecker& checker, RsProvider& rs,
                const internal::GridPotential& potential,
                bool potential_reused, double min_turn_radius_m,
                double curvature_scale, std::string backend_id,
                std::string backend_kind, Clock::time_point plan_start,
                const std::function<bool()>& cancel_callback)
      : config_(config),
        request_(request),
        grid_(grid),
        checker_(checker),
        rs_(rs),
        potential_(potential),
        potential_reused_(potential_reused),
        min_turn_radius_m_(min_turn_radius_m),
        curvature_scale_(curvature_scale),
        backend_id_(std::move(backend_id)),
        backend_kind_(std::move(backend_kind)),
        cancel_callback_(cancel_callback),
        plan_start_(plan_start),
        grid_frame_(collision::GridFrame::from_grid(grid)),
        time_budget_s_(request.time_budget_s.value_or(config.default_time_budget_s)),
        node_budget_(request.node_budget.value_or(config.default_node_budget)) {}

  PlanResult run();

 private:
  double elapsed_ms() const { return elapsed_ms_since(plan_start_); }
  bool deadline_exceeded() const { return elapsed_ms() > time_budget_s_ * 1000.0; }

  int heading_bin(double yaw_rad) const {
    const double wrapped = common::normalize_angle_rad(yaw_rad);
    const double span = 2.0 * common::kPi / static_cast<double>(config_.heading_bins);
    int bin = static_cast<int>(std::floor((wrapped + common::kPi) / span));
    bin = std::max(0, std::min(config_.heading_bins - 1, bin));
    return bin;
  }

  std::uint64_t state_key(const Pose2d& pose) const {
    const std::int64_t cell_x =
        static_cast<std::int64_t>(std::floor(pose.x_m / config_.xy_resolution_m));
    const std::int64_t cell_y =
        static_cast<std::int64_t>(std::floor(pose.y_m / config_.xy_resolution_m));
    const std::uint64_t ux = static_cast<std::uint64_t>(cell_x) & 0x1FFFFFull;
    const std::uint64_t uy = static_cast<std::uint64_t>(cell_y) & 0x1FFFFFull;
    const std::uint64_t hb =
        static_cast<std::uint64_t>(heading_bin(pose.yaw_rad)) & 0xFFFFull;
    return (ux << 43) | (uy << 22) | hb;
  }

  bool cell_of_pose(const Pose2d& pose, int* row, int* col) const {
    const common::Point2d grid_point =
        grid_frame_.to_grid_frame(common::Point2d{pose.x_m, pose.y_m});
    const int c = static_cast<int>(std::floor(grid_point.x_m / grid_.resolution_m));
    const int r = static_cast<int>(std::floor(grid_point.y_m / grid_.resolution_m));
    if (!grid_.in_bounds(r, c)) {
      return false;
    }
    *row = r;
    *col = c;
    return true;
  }

  bool cell_reachable(const Pose2d& pose) const {
    int row = 0;
    int col = 0;
    if (!cell_of_pose(pose, &row, &col)) {
      return false;
    }
    return std::isfinite(
        potential_.distance_m[static_cast<std::size_t>(row) *
                                  static_cast<std::size_t>(potential_.width) +
                              static_cast<std::size_t>(col)]);
  }

  double heuristic(const Pose2d& pose) {
    double rs_distance = 0.0;
    std::string reason;
    ++rs_calls_;
    if (rs_.distance(pose, request_.goal, &rs_distance, &reason) !=
        Status::kSuccess) {
      rs_distance = 0.0;  // 保守回退；栅格项仍起引导作用
    }
    double grid_distance = 0.0;
    int row = 0;
    int col = 0;
    if (cell_of_pose(pose, &row, &col)) {
      const float distance =
          potential_.distance_m[static_cast<std::size_t>(row) *
                                    static_cast<std::size_t>(potential_.width) +
                                static_cast<std::size_t>(col)];
      if (std::isfinite(distance)) {
        const double slack = 0.5 * std::sqrt(2.0) * potential_.resolution_m;
        grid_distance = std::max(0.0, static_cast<double>(distance) - slack);
      }
    }
    return std::max(rs_distance, config_.grid_weight * grid_distance);
  }

  std::vector<Primitive> build_primitives() const {
    std::vector<Primitive> primitives;
    primitives.reserve(config_.steer_fractions.size() *
                       (config_.allow_reverse ? 2u : 1u));
    for (const double fraction : config_.steer_fractions) {
      const double steer = fraction * request_.vehicle.max_steer_rad;
      double curvature = 0.0;
      if (std::abs(steer) > 1e-12) {
        curvature = std::tan(steer) / request_.vehicle.wheelbase_m;
        curvature *= curvature_scale_;
      }
      primitives.push_back(Primitive{fraction, curvature, config_.motion_step_m,
                                     Direction::kForward});
      if (config_.allow_reverse) {
        primitives.push_back(Primitive{fraction, curvature,
                                       config_.reverse_motion_step_m,
                                       Direction::kReverse});
      }
    }
    return primitives;
  }

  static double cross2d(const common::Point2d& a, const common::Point2d& b,
                        const common::Point2d& c) {
    return (b.x_m - a.x_m) * (c.y_m - a.y_m) -
           (b.y_m - a.y_m) * (c.x_m - a.x_m);
  }

  static bool point_on_segment(const common::Point2d& a,
                               const common::Point2d& b,
                               const common::Point2d& p) {
    const double tolerance = 1e-9;
    return std::min(a.x_m, b.x_m) - tolerance <= p.x_m &&
           p.x_m <= std::max(a.x_m, b.x_m) + tolerance &&
           std::min(a.y_m, b.y_m) - tolerance <= p.y_m &&
           p.y_m <= std::max(a.y_m, b.y_m) + tolerance;
  }

  static bool segments_intersect(const common::Point2d& a,
                                 const common::Point2d& b,
                                 const common::Point2d& c,
                                 const common::Point2d& d) {
    const double tolerance = 1e-9;
    const auto sign = [tolerance](double value) {
      return value > tolerance ? 1 : (value < -tolerance ? -1 : 0);
    };
    const double o1 = cross2d(a, b, c);
    const double o2 = cross2d(a, b, d);
    const double o3 = cross2d(c, d, a);
    const double o4 = cross2d(c, d, b);
    if (sign(o1) != sign(o2) && sign(o3) != sign(o4)) {
      return true;
    }
    if (sign(o1) == 0 && point_on_segment(a, b, c)) return true;
    if (sign(o2) == 0 && point_on_segment(a, b, d)) return true;
    if (sign(o3) == 0 && point_on_segment(c, d, a)) return true;
    if (sign(o4) == 0 && point_on_segment(c, d, b)) return true;
    return false;
  }

  // 候选基元的弦（父位姿 -> 下一位姿）穿过某个先前链路分段时返回 true；
  // 不含终点为父节点、共享端点的相邻段。弦近似：仅用于软代价启发，
  // 绝不作为拒绝条件。
  bool primitive_crosses_chain(std::uint32_t parent_index,
                               const Pose2d& next) const {
    if (config_.loop_penalty <= 0.0 || config_.loop_guard_segments <= 0) {
      return false;
    }
    std::array<common::Point2d, 65> points{};
    int count = 0;
    std::uint32_t index = parent_index;
    while (index != kNoParent && count < config_.loop_guard_segments + 1) {
      points[static_cast<std::size_t>(count)] =
          common::Point2d{nodes_[index].pose.x_m, nodes_[index].pose.y_m};
      ++count;
      index = nodes_[index].parent;
    }
    const common::Point2d start{nodes_[parent_index].pose.x_m,
                                nodes_[parent_index].pose.y_m};
    const common::Point2d end{next.x_m, next.y_m};
    for (int t = 1; t + 1 < count; ++t) {
      if (segments_intersect(points[static_cast<std::size_t>(t)],
                             points[static_cast<std::size_t>(t + 1)], start,
                             end)) {
        return true;
      }
    }
    return false;
  }

  double primitive_cost(const Primitive& primitive, int parent_primitive) const {
    const double direction_cost =
        primitive.direction == Direction::kForward ? config_.forward_cost
                                                    : config_.reverse_cost;
    double cost = primitive.step_m * direction_cost;
    if (std::abs(primitive.steer_fraction) > 1e-12) {
      cost *= config_.turn_penalty;
    }
    cost += config_.steer_cost * std::abs(primitive.steer_fraction) *
            primitive.step_m;
    if (parent_primitive >= 0) {
      const Primitive& parent =
          primitives_[static_cast<std::size_t>(parent_primitive)];
      if (std::abs(parent.steer_fraction - primitive.steer_fraction) > 1e-12) {
        cost += config_.steer_change_cost;
      }
      if (parent.direction != primitive.direction) {
        cost += config_.gear_change_cost;
      }
    }
    return cost;
  }

  // 对从 `node` 到请求目标的 RS 连接做完整检查（逐 RS 分段整车扫掠）。
  bool try_connection(const SearchNode& node, rs::RsPlan* connection) {
    rs::RsPlan solved = rs_.plan(node.pose, request_.goal);
    ++rs_calls_;
    if (solved.status != Status::kSuccess) {
      return false;
    }
    if (config_.analytic_expansion_max_length_m > 0.0 &&
        solved.path.length_m > config_.analytic_expansion_max_length_m) {
      return false;
    }
    if (config_.analytic_shot_distance_m > 0.0 &&
        solved.path.length_m > config_.analytic_shot_distance_m) {
      return false;
    }
    Pose2d cursor = node.pose;
    for (const rs::RsSegment& segment : solved.path.segments) {
      if (!(segment.length_m > 0.0)) {
        continue;
      }
      collision::MotionSegment motion;
      motion.start = cursor;
      motion.length_m = segment.length_m;
      motion.direction = segment.direction;
      motion.curvature_inv_m = segment.curvature_inv_m;
      const collision::CheckResult check = checker_.check_motion(motion);
      ++collision_checks_;
      if (!check.is_clear()) {
        return false;
      }
      cursor = rs::integrate_segment(cursor, segment, segment.length_m);
    }
    *connection = std::move(solved);
    return true;
  }

  nlohmann::json make_statistics() const {
    nlohmann::json stats = nlohmann::json::object();
    stats["backend_id"] = backend_id_;
    stats["backend_kind"] = backend_kind_;
    stats["cache_mode"] = potential_reused_ ? "warm" : "cold";
    stats["heuristic_cache_reused"] = potential_reused_;
    stats["expanded_nodes"] = expanded_;
    stats["generated_nodes"] = generated_;
    stats["closed_states"] = closed_states_;
    stats["collision_checks"] = collision_checks_;
    stats["motion_rejections"] = motion_rejections_;
    stats["pruned_unreachable_states"] = pruned_unreachable_;
    stats["stale_frontier_entries"] = stale_pops_;
    stats["rs_distance_calls"] = rs_calls_;
    stats["rs_connection_attempts"] = analytic_attempts_;
    stats["motion_primitives"] = static_cast<std::int64_t>(primitives_.size());
    stats["search_time_ms"] = search_ms_;
    stats["postprocess_time_ms"] = postprocess_ms_;
    stats["time_budget_s"] = time_budget_s_;
    stats["node_budget"] = node_budget_;
    stats["unsupported_statistics"] = unsupported_statistics_json();
    return stats;
  }

  PlanResult failure(Status status, const std::string& reason) {
    search_ms_ = elapsed_ms();
    postprocess_ms_ = 0.0;
    PlanResult result;
    result.request_id = request_.request_id;
    result.status = status;
    result.reason = reason;
    result.statistics = make_statistics();
    result.statistics["total_time_ms"] = elapsed_ms();
    return result;
  }

  // 重建扁平路径（节点链 + RS 连接），在尖点处切分，再次验证并返回最终
  // PlanResult。
  PlanResult build_success(std::uint32_t final_node, const rs::RsPath& connection);

  const RosFreeConfig& config_;
  const PlanRequest& request_;
  const common::GridView& grid_;
  const collision::CollisionChecker& checker_;
  RsProvider& rs_;
  const internal::GridPotential& potential_;
  bool potential_reused_;
  double min_turn_radius_m_;
  double curvature_scale_;
  std::string backend_id_;
  std::string backend_kind_;
  const std::function<bool()>& cancel_callback_;
  Clock::time_point plan_start_;
  collision::GridFrame grid_frame_;
  double time_budget_s_;
  std::int64_t node_budget_;

  std::vector<Primitive> primitives_;
  std::vector<SearchNode> nodes_;
  std::unordered_map<std::uint64_t, double> best_cost_;
  std::unordered_map<std::uint64_t, std::uint32_t> closed_;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryGreater>
      frontier_;

  std::int64_t expanded_{0};
  std::int64_t generated_{0};
  std::int64_t closed_states_{0};
  std::int64_t collision_checks_{0};
  std::int64_t rs_calls_{0};
  std::int64_t analytic_attempts_{0};
  std::int64_t stale_pops_{0};
  std::int64_t pruned_unreachable_{0};
  std::int64_t motion_rejections_{0};
  double search_ms_{0.0};
  double postprocess_ms_{0.0};
  std::uint32_t best_node_{0};
  double best_node_h_{std::numeric_limits<double>::infinity()};
};

PlanResult RosFreeSearch::run() {
  primitives_ = build_primitives();
  if (primitives_.empty()) {
    return failure(Status::kInvalidInput,
                   "invalid input: configuration produced no motion primitives");
  }

  SearchNode root;
  root.pose = request_.start;
  root.cost = 0.0;
  root.length_m = 0.0;
  root.parent = kNoParent;
  root.primitive = -1;
  root.key = state_key(root.pose);
  nodes_.push_back(root);
  best_cost_[root.key] = 0.0;
  const double root_h = heuristic(root.pose);
  best_node_ = 0;
  best_node_h_ = root_h;
  frontier_.push(QueueEntry{config_.heuristic_weight * root_h, root_h, 0});

  while (!frontier_.empty()) {
    if (cancel_callback_ && cancel_callback_()) {
      return failure(Status::kCancelled,
                     "cancelled during search before expansion " +
                         std::to_string(expanded_));
    }
    if (deadline_exceeded()) {
      return failure(
          Status::kTimeout,
          "timeout: time budget " + std::to_string(time_budget_s_) +
              " s exceeded after " + std::to_string(expanded_) + " expansions");
    }
    if (expanded_ >= node_budget_) {
      return failure(
          Status::kTimeout,
          "timeout: node budget " + std::to_string(node_budget_) +
              " exhausted after " + std::to_string(expanded_) + " expansions");
    }

    const QueueEntry top = frontier_.top();
    frontier_.pop();
    const std::uint32_t index = top.node;
    if (closed_.find(nodes_[index].key) != closed_.end()) {
      ++stale_pops_;
      continue;  // 过期前沿条目
    }
    closed_.emplace(nodes_[index].key, index);
    ++expanded_;
    closed_states_ = static_cast<std::int64_t>(closed_.size());

    const bool attempt_analytic =
        (expanded_ == 1) ||
        (config_.analytic_expansion_ratio > 0 &&
         (expanded_ % config_.analytic_expansion_ratio) == 0);
    if (attempt_analytic) {
      ++analytic_attempts_;
      rs::RsPlan connection;
      if (try_connection(nodes_[index], &connection)) {
        return build_success(index, connection.path);
      }
    }

    const Pose2d parent_pose = nodes_[index].pose;
    const double parent_cost = nodes_[index].cost;
    const double parent_length = nodes_[index].length_m;
    const int parent_primitive = nodes_[index].primitive;

    for (std::size_t p = 0; p < primitives_.size(); ++p) {
      const Primitive& primitive = primitives_[p];
      Pose2d next;
      try {
        next = rs::integrate_segment(parent_pose, to_rs_segment(primitive),
                                     primitive.step_m);
      } catch (const std::invalid_argument&) {
        continue;  // 被拒绝的基元直接不作为候选
      }
      const std::uint64_t next_key = state_key(next);
      if (closed_.find(next_key) != closed_.end()) {
        continue;
      }
      if (!cell_reachable(next)) {
        ++pruned_unreachable_;
        continue;  // 无法经由该单元无碰撞到达
      }
      collision::MotionSegment motion;
      motion.start = parent_pose;
      motion.length_m = primitive.step_m;
      motion.direction = primitive.direction;
      motion.curvature_inv_m = primitive.curvature_inv_m;
      const collision::CheckResult check = checker_.check_motion(motion);
      ++collision_checks_;
      if (!check.is_clear()) {
        ++motion_rejections_;
        continue;
      }
      double cost = parent_cost + primitive_cost(primitive, parent_primitive);
      if (primitive_crosses_chain(index, next)) {
        cost += config_.loop_penalty;
      }
      const auto known = best_cost_.find(next_key);
      if (known != best_cost_.end() && known->second <= cost + 1e-12) {
        continue;
      }
      best_cost_[next_key] = cost;
      SearchNode child;
      child.pose = next;
      child.cost = cost;
      child.length_m = parent_length + primitive.step_m;
      child.parent = index;
      child.primitive = static_cast<int>(p);
      child.key = next_key;
      nodes_.push_back(child);
      ++generated_;
      const double h = heuristic(next);
      if (h < best_node_h_) {
        best_node_h_ = h;
        best_node_ = static_cast<std::uint32_t>(nodes_.size() - 1);
      }
      frontier_.push(
          QueueEntry{cost + config_.heuristic_weight * h, h,
                     static_cast<std::uint32_t>(nodes_.size() - 1)});
    }
  }

  // 前沿耗尽：离散可达状态空间已完全探索。从最近的已扩展状态做最后一次
  // 解析尝试。
  if (best_node_ != 0) {
    ++analytic_attempts_;
    rs::RsPlan connection;
    if (try_connection(nodes_[best_node_], &connection)) {
      return build_success(best_node_, connection.path);
    }
  }
  return failure(Status::kNoPath,
                 "no path: search space exhausted after " +
                     std::to_string(expanded_) + " expansions");
}

PlanResult RosFreeSearch::build_success(std::uint32_t final_node,
                                        const rs::RsPath& connection) {
  search_ms_ = elapsed_ms();
  const Clock::time_point post_start = Clock::now();

  std::vector<std::uint32_t> chain;
  for (std::uint32_t index = final_node; index != kNoParent;
       index = nodes_[index].parent) {
    chain.push_back(index);
  }
  std::reverse(chain.begin(), chain.end());

  std::vector<common::PathPoint> flat;
  flat.reserve(chain.size() + 64);
  double s_m = 0.0;
  for (std::size_t i = 0; i < chain.size(); ++i) {
    const SearchNode& node = nodes_[chain[i]];
    common::PathPoint point;
    point.pose = node.pose;
    point.s_m = s_m;
    // 边界点携带从该点开始的运动的方向/曲率（与 planner::rs 采样相同的
    // 归属规则）。
    const Primitive* beginning = nullptr;
    if (i + 1 < chain.size()) {
      beginning =
          &primitives_[static_cast<std::size_t>(nodes_[chain[i + 1]].primitive)];
    }
    if (beginning != nullptr) {
      point.direction = beginning->direction;
      point.curvature_inv_m = beginning->curvature_inv_m;
    } else if (!connection.segments.empty()) {
      point.direction = connection.segments.front().direction;
      point.curvature_inv_m = connection.segments.front().curvature_inv_m;
    } else if (i > 0) {
      const Primitive& incoming =
          primitives_[static_cast<std::size_t>(node.primitive)];
      point.direction = incoming.direction;
      point.curvature_inv_m = incoming.curvature_inv_m;
    } else {
      point.direction = Direction::kForward;
      point.curvature_inv_m = 0.0;
    }
    flat.push_back(point);
    if (i + 1 < chain.size()) {
      const SearchNode& next = nodes_[chain[i + 1]];
      const Primitive& incoming =
          primitives_[static_cast<std::size_t>(next.primitive)];
      s_m += incoming.step_m;
    }
  }

  if (!connection.segments.empty()) {
    const double sample_step_m = std::max(0.25, 0.5 * config_.motion_step_m);
    std::vector<common::PathPoint> samples;
    try {
      samples = rs::sample(connection, sample_step_m);
    } catch (const std::exception& error) {
      return failure(Status::kValidationFailed,
                     std::string("validation failed: RS sampling error: ") +
                         error.what());
    }
    for (std::size_t k = 1; k < samples.size(); ++k) {
      common::PathPoint point = samples[k];
      point.s_m += s_m;
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
      // 冻结契约（序列化 README + 回放样例）：尖点位姿既是前一分段的最后
      // 一个点，也是后一分段的第一个点；direction 字段区分挡位。没有这份
      // 重复拷贝，消费方会在每个尖点看到一段基元长度的空隙。
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

  // ------------------------------------------------------------------
  // 后处理验证（硬性检查；未通过绝不报告成功）。计时计入总时间。
  // ------------------------------------------------------------------
  std::string failure_reason;
  double endpoint_position_error = 0.0;
  double endpoint_yaw_error = 0.0;
  if (!path.empty() && !path.front().points.empty()) {
    const common::PathPoint& first = path.front().points.front();
    const double position_error =
        std::hypot(first.pose.x_m - request_.start.x_m,
                   first.pose.y_m - request_.start.y_m);
    const double yaw_error = std::abs(common::normalize_angle_rad(
        first.pose.yaw_rad - request_.start.yaw_rad));
    if (position_error > kStartPoseToleranceM ||
        yaw_error > kStartYawToleranceRad) {
      failure_reason = "output path does not begin at the requested start pose";
    }
  }
  if (failure_reason.empty() && path.empty()) {
    failure_reason = "output path is empty";
  }

  double max_abs_curvature = 0.0;
  double previous_s = -1.0;
  if (failure_reason.empty()) {
    for (const common::PathSegment& segment : path) {
      if (segment.points.empty()) {
        failure_reason = "output path contains an empty segment";
        break;
      }
      for (std::size_t i = 0; i < segment.points.size(); ++i) {
        const common::PathPoint& point = segment.points[i];
        if (!pose_is_finite(point.pose) || !std::isfinite(point.s_m) ||
            !std::isfinite(point.curvature_inv_m)) {
          failure_reason = "output path contains a non-finite value";
          break;
        }
        if (i == 0) {
          // 后续分段的第一个点复制了前一分段的最后一个点（尖点归属），
          // 因此那里允许相等；分段内部 s_m 必须严格递增。
          if (previous_s >= 0.0 && point.s_m < previous_s) {
            failure_reason = "arc length decreases between segments";
            break;
          }
        } else if (!(point.s_m > segment.points[i - 1].s_m)) {
          failure_reason = "arc length is not strictly increasing";
          break;
        }
        max_abs_curvature =
            std::max(max_abs_curvature, std::abs(point.curvature_inv_m));
        previous_s = point.s_m;
      }
      if (!failure_reason.empty()) {
        break;
      }
    }
  }

  if (failure_reason.empty()) {
    const double max_curvature = 1.0 / min_turn_radius_m_;
    if (max_abs_curvature > max_curvature + kCurvatureTolerance) {
      failure_reason =
          "output path curvature exceeds the vehicle steering limit";
    }
  }

  const common::PathPoint* last_point = nullptr;
  if (!path.empty() && !path.back().points.empty()) {
    last_point = &path.back().points.back();
  }
  if (failure_reason.empty() && last_point != nullptr) {
    endpoint_position_error =
        std::hypot(last_point->pose.x_m - request_.goal.x_m,
                   last_point->pose.y_m - request_.goal.y_m);
    endpoint_yaw_error = std::abs(common::normalize_angle_rad(
        last_point->pose.yaw_rad - request_.goal.yaw_rad));
    if (endpoint_position_error > kEndpointPositionToleranceM ||
        endpoint_yaw_error > kEndpointYawToleranceRad) {
      failure_reason = "output endpoint exceeds the 0.05 m / 0.02 rad tolerance";
    }
  }

  bool recheck_clear = true;
  if (failure_reason.empty()) {
    const collision::CheckResult check = checker_.check_path(path);
    ++collision_checks_;
    recheck_clear = check.is_clear();
    if (!recheck_clear) {
      failure_reason =
          "whole-vehicle re-check of the output path failed: " +
          collision_detail(check);
    }
  }

  if (!failure_reason.empty() || !recheck_clear) {
    return failure(Status::kValidationFailed,
                   "validation failed: " +
                       (failure_reason.empty() ? std::string("unknown")
                                               : failure_reason));
  }

  postprocess_ms_ = elapsed_ms_since(post_start);
  PlanResult result;
  result.request_id = request_.request_id;
  result.status = Status::kSuccess;
  result.reason.clear();
  result.path = std::move(path);
  const double path_length_m =
      result.path.empty() ? 0.0 : result.path.back().points.back().s_m;
  const std::int64_t gear_changes =
      result.path.empty() ? 0 : static_cast<std::int64_t>(result.path.size()) - 1;
  result.statistics = make_statistics();
  result.statistics["path_length_m"] = path_length_m;
  result.statistics["gear_changes"] = gear_changes;
  result.statistics["endpoint_position_error_m"] = endpoint_position_error;
  result.statistics["endpoint_yaw_error_rad"] = endpoint_yaw_error;
  result.statistics["postprocess_time_ms"] = postprocess_ms_;
  result.statistics["total_time_ms"] = elapsed_ms();
  return result;
}

}  // namespace

nlohmann::json config_to_json(const RosFreeConfig& config) {
  nlohmann::json value = nlohmann::json::object();
  value["xy_resolution_m"] = config.xy_resolution_m;
  value["heading_bins"] = config.heading_bins;
  value["motion_step_m"] = config.motion_step_m;
  value["reverse_motion_step_m"] = config.reverse_motion_step_m;
  value["steer_fractions"] = config.steer_fractions;
  value["allow_reverse"] = config.allow_reverse;
  value["min_turn_radius_m"] = config.min_turn_radius_m;
  value["forward_cost"] = config.forward_cost;
  value["reverse_cost"] = config.reverse_cost;
  value["steer_cost"] = config.steer_cost;
  value["gear_change_cost"] = config.gear_change_cost;
  value["turn_penalty"] = config.turn_penalty;
  value["steer_change_cost"] = config.steer_change_cost;
  value["heuristic_weight"] = config.heuristic_weight;
  value["grid_weight"] = config.grid_weight;
  value["loop_penalty"] = config.loop_penalty;
  value["loop_guard_segments"] = config.loop_guard_segments;
  value["analytic_expansion_ratio"] = config.analytic_expansion_ratio;
  value["analytic_expansion_max_length_m"] =
      config.analytic_expansion_max_length_m;
  value["analytic_shot_distance_m"] = config.analytic_shot_distance_m;
  value["smooth"] = config.smooth;
  value["default_time_budget_s"] = config.default_time_budget_s;
  value["default_node_budget"] = config.default_node_budget;
  return value;
}

std::vector<ParameterSupport> core_parameter_support() {
  return {
      {"xy_resolution_m", true,
       "state-grid cell for closed-set dedup; 0 < value <= 10 [m]"},
      {"heading_bins", true,
       "heading discretization of the state grid; integer in [8, 3600]"},
      {"motion_step_m", true,
       "forward motion-primitive length; 0 < value <= 50 [m]"},
      {"reverse_motion_step_m", true,
       "reverse motion-primitive length; 0 < value <= 50 [m]"},
      {"steer_fractions", true,
       "steering samples as fractions of max_steer_rad, each in [-1, 1]"},
      {"allow_reverse", true, "enables reverse motion primitives"},
      {"min_turn_radius_m", true,
       "primitive curvature radius override; 0 = derive from the request "
       "vehicle; a set value must not be below the vehicle limit [m]"},
      {"forward_cost", true, "cost per forward meter; > 0"},
      {"reverse_cost", true, "cost per reverse meter; > 0"},
      {"steer_cost", true,
       "extra cost per |steer fraction| per meter; >= 0"},
      {"gear_change_cost", true, "extra cost at a direction change; >= 0"},
      {"turn_penalty", true,
       "multiplier on the step cost when steering != 0; > 0"},
      {"steer_change_cost", true,
       "extra cost when the steering grade changes; >= 0"},
      {"heuristic_weight", true, "weighted-A* heuristic factor; > 0"},
      {"grid_weight", true,
       "weight of the grid potential term in the heuristic; >= 0"},
      {"loop_penalty", true,
       "extra cost when a primitive crosses a recent chain segment; >= 0 "
       "(0 disables the loop guard)"},
      {"loop_guard_segments", true,
       "number of previous chain segments tested by the loop guard; integer "
       "in [0, 64]"},
      {"analytic_expansion_ratio", true,
       "attempt an RS connection every N node expansions; integer >= 1"},
      {"analytic_expansion_max_length_m", true,
       "reject RS connections longer than this; 0 = unlimited [m]"},
      {"analytic_shot_distance_m", true,
       "attempt RS connections only within this distance; 0 = no gate [m]"},
      {"smooth", true,
       "this backend implements no smoothing; only false is accepted"},
      {"default_time_budget_s", true,
       "time budget used when the request does not set one; > 0 [s]"},
      {"default_node_budget", true,
       "node budget used when the request does not set one; >= 1"},
  };
}

BackendProfile ros_free_profile() {
  BackendProfile profile;
  profile.id = "ros_free";
  profile.kind = BackendKind::kNormalized;
  profile.description =
      "ROS-free normalized backend: discrete-state Hybrid A* over "
      "planner::rs analytic connections and whole-vehicle planner::collision "
      "checks";
  profile.defaults = RosFreeConfig{};
  profile.parameters = core_parameter_support();
  profile.notes.push_back(
      "no upstream-specific aliases; parameter names are the frozen core "
      "contract");
  return profile;
}

// ---------------------------------------------------------------------------
// 配置解析
// ---------------------------------------------------------------------------

ConfigReport configure_ros_free(const nlohmann::json& config,
                                const BackendProfile& profile,
                                RosFreeConfig* out) {
  ConfigReport report;
  report.parameters = profile.parameters;
  report.effective_config = config_to_json(profile.defaults);
  if (!config.is_object()) {
    report.accepted = false;
    report.reason = "config must be a JSON object";
    return report;
  }

  RosFreeConfig parsed = profile.defaults;
  bool accepted = true;
  std::string reason;

  const auto fail = [&](const std::string& key, const std::string& message) {
    accepted = false;
    reason = "parameter '" + key + "': " + message;
  };
  const auto number_in_range = [](const nlohmann::json& value, double minimum,
                                  double maximum) {
    if (!value.is_number()) {
      return false;
    }
    const double parsed_value = value.get<double>();
    return std::isfinite(parsed_value) && parsed_value > minimum &&
           parsed_value <= maximum;
  };
  const auto non_negative_number = [](const nlohmann::json& value) {
    if (!value.is_number()) {
      return false;
    }
    const double parsed_value = value.get<double>();
    return std::isfinite(parsed_value) && parsed_value >= 0.0;
  };
  const auto canonical = [&profile](const std::string& key) {
    for (const auto& alias : profile.aliases) {
      if (alias.first == key) {
        return alias.second;
      }
    }
    return key;
  };

  for (auto it = config.begin(); it != config.end() && accepted; ++it) {
    const std::string key = it.key();
    const std::string mapped = canonical(key);
    const nlohmann::json& value = it.value();
    if (mapped == "xy_resolution_m") {
      if (!number_in_range(value, 0.0, 10.0)) {
        fail(key, "expected a number in (0, 10] meters");
      } else {
        parsed.xy_resolution_m = value.get<double>();
      }
    } else if (mapped == "heading_bins") {
      if (!value.is_number_integer()) {
        fail(key, "expected an integer in [8, 3600]");
      } else {
        parsed.heading_bins = value.get<int>();
        if (parsed.heading_bins < 8 || parsed.heading_bins > 3600) {
          fail(key, "expected an integer in [8, 3600]");
        }
      }
    } else if (mapped == "motion_step_m") {
      if (!number_in_range(value, 0.0, 50.0)) {
        fail(key, "expected a number in (0, 50] meters");
      } else {
        parsed.motion_step_m = value.get<double>();
      }
    } else if (mapped == "reverse_motion_step_m") {
      if (!number_in_range(value, 0.0, 50.0)) {
        fail(key, "expected a number in (0, 50] meters");
      } else {
        parsed.reverse_motion_step_m = value.get<double>();
      }
    } else if (mapped == "steer_fractions") {
      if (!value.is_array() || value.empty() || value.size() > 33) {
        fail(key, "expected an array of 1..33 numbers in [-1, 1]");
      } else {
        std::vector<double> fractions;
        fractions.reserve(value.size());
        for (const nlohmann::json& element : value) {
          if (!element.is_number()) {
            fail(key, "every steering fraction must be a number");
            break;
          }
          const double fraction = element.get<double>();
          if (!std::isfinite(fraction) || fraction < -1.0 || fraction > 1.0) {
            fail(key, "steering fractions must be finite and in [-1, 1]");
            break;
          }
          fractions.push_back(fraction);
        }
        if (accepted) {
          parsed.steer_fractions = std::move(fractions);
        }
      }
    } else if (mapped == "allow_reverse") {
      if (!value.is_boolean()) {
        fail(key, "expected a boolean");
      } else {
        parsed.allow_reverse = value.get<bool>();
      }
    } else if (mapped == "min_turn_radius_m") {
      if (!non_negative_number(value)) {
        fail(key, "expected a non-negative number (0 = derive from vehicle)");
      } else {
        parsed.min_turn_radius_m = value.get<double>();
      }
    } else if (mapped == "forward_cost" || mapped == "reverse_cost") {
      if (!number_in_range(value, 0.0, 1000.0)) {
        fail(key, "expected a number in (0, 1000]");
      } else if (mapped == "forward_cost") {
        parsed.forward_cost = value.get<double>();
      } else {
        parsed.reverse_cost = value.get<double>();
      }
    } else if (mapped == "loop_penalty") {
      if (!non_negative_number(value)) {
        fail(key, "expected a non-negative number");
      } else {
        parsed.loop_penalty = value.get<double>();
      }
    } else if (mapped == "loop_guard_segments") {
      if (!value.is_number_integer()) {
        fail(key, "expected an integer in [0, 64]");
      } else {
        parsed.loop_guard_segments = value.get<int>();
        if (parsed.loop_guard_segments < 0 || parsed.loop_guard_segments > 64) {
          fail(key, "expected an integer in [0, 64]");
        }
      }
    } else if (mapped == "steer_cost" || mapped == "gear_change_cost" ||
               mapped == "steer_change_cost" || mapped == "grid_weight") {
      if (!non_negative_number(value)) {
        fail(key, "expected a non-negative number");
      } else if (mapped == "steer_cost") {
        parsed.steer_cost = value.get<double>();
      } else if (mapped == "gear_change_cost") {
        parsed.gear_change_cost = value.get<double>();
      } else if (mapped == "steer_change_cost") {
        parsed.steer_change_cost = value.get<double>();
      } else {
        parsed.grid_weight = value.get<double>();
      }
    } else if (mapped == "turn_penalty") {
      if (!number_in_range(value, 0.0, 1000.0)) {
        fail(key, "expected a number in (0, 1000]");
      } else {
        parsed.turn_penalty = value.get<double>();
      }
    } else if (mapped == "heuristic_weight") {
      if (!number_in_range(value, 0.0, 1000.0)) {
        fail(key, "expected a number in (0, 1000]");
      } else {
        parsed.heuristic_weight = value.get<double>();
      }
    } else if (mapped == "analytic_expansion_ratio") {
      if (!value.is_number()) {
        fail(key, "expected a number in [1, 100000]");
      } else {
        const double raw = value.get<double>();
        const int rounded = static_cast<int>(std::lround(raw));
        if (!std::isfinite(raw) || rounded < 1 || rounded > 100000) {
          fail(key, "expected a number in [1, 100000]");
        } else {
          parsed.analytic_expansion_ratio = rounded;
        }
      }
    } else if (mapped == "analytic_expansion_max_length_m" ||
               mapped == "analytic_shot_distance_m") {
      if (!non_negative_number(value)) {
        fail(key, "expected a non-negative number (0 = disabled)");
      } else if (mapped == "analytic_expansion_max_length_m") {
        parsed.analytic_expansion_max_length_m = value.get<double>();
      } else {
        parsed.analytic_shot_distance_m = value.get<double>();
      }
    } else if (mapped == "smooth") {
      if (!value.is_boolean()) {
        fail(key, "expected a boolean");
      } else if (value.get<bool>()) {
        report.unsupported.push_back(
            "smooth (upstream extra smoothing is not implemented; only false "
            "is accepted)");
      }
      parsed.smooth = false;
    } else if (mapped == "default_time_budget_s") {
      if (!number_in_range(value, 0.0, 100000.0)) {
        fail(key, "expected a number in (0, 100000] seconds");
      } else {
        parsed.default_time_budget_s = value.get<double>();
      }
    } else if (mapped == "default_node_budget") {
      if (!value.is_number_integer()) {
        fail(key, "expected an integer >= 1");
      } else {
        parsed.default_node_budget = value.get<std::int64_t>();
        if (parsed.default_node_budget < 1) {
          fail(key, "expected an integer >= 1");
        }
      }
    } else if (mapped == key) {
      bool documented = false;
      for (const ParameterSupport& support : profile.parameters) {
        if (support.name == key && !support.supported) {
          report.unsupported.push_back(key + " (" + support.note + ")");
          documented = true;
          break;
        }
      }
      if (!documented) {
        report.unsupported.push_back(key + " (unknown parameter)");
      }
    } else {
      report.unsupported.push_back(
          key + " (alias '" + mapped + "' is not mappable for this backend)");
    }
  }

  if (!accepted) {
    report.accepted = false;
    report.reason = reason;
    return report;
  }

  report.effective_config = config_to_json(parsed);
  report.accepted = true;
  if (out != nullptr) {
    *out = std::move(parsed);
  }
  return report;
}

// ---------------------------------------------------------------------------
// RosFreeBackend
// ---------------------------------------------------------------------------

struct RosFreeBackend::Cache {
  bool valid{false};
  std::uint64_t key{0};
  internal::GridPotential potential;
};

RosFreeBackend::RosFreeBackend() : RosFreeBackend(ros_free_profile()) {}

RosFreeBackend::RosFreeBackend(BackendProfile profile)
    : profile_(std::move(profile)), cache_(new Cache()) {
  const ConfigReport report = configure(nlohmann::json::object());
  configured_ = report.accepted;
}

RosFreeBackend::~RosFreeBackend() = default;

std::string RosFreeBackend::id() const { return profile_.id; }

BackendKind RosFreeBackend::kind() const noexcept { return profile_.kind; }

BackendInfo RosFreeBackend::info() const {
  BackendInfo info;
  info.id = profile_.id;
  info.kind = profile_.kind;
  info.description = profile_.description;
  info.parameters = profile_.parameters;
  info.statistics = statistic_support();
  return info;
}

ConfigReport RosFreeBackend::configure(const nlohmann::json& config) {
  RosFreeConfig parsed;
  const ConfigReport report = configure_ros_free(config, profile_, &parsed);
  report_ = report;
  configured_ = report.accepted;
  if (report.accepted) {
    config_ = parsed;
    unsupported_reason_.clear();
    for (const std::string& name : report.unsupported) {
      if (!unsupported_reason_.empty()) {
        unsupported_reason_ += ", ";
      }
      unsupported_reason_ += name;
    }
  } else {
    unsupported_reason_.clear();
  }
  reset_cache();
  return report_;
}

const nlohmann::json& RosFreeBackend::effective_config() const {
  return report_.effective_config;
}

void RosFreeBackend::reset_cache() { cache_->valid = false; }

void RosFreeBackend::set_cancel_callback(std::function<bool()> callback) {
  cancel_callback_ = std::move(callback);
}

common::PlanResult RosFreeBackend::plan(
    const common::PlanRequest& request, const common::GridView& grid,
    const collision::CollisionChecker& collision_checker,
    RsProvider& rs_provider) {
  return run_plan(request, grid, collision_checker, rs_provider);
}

common::PlanResult RosFreeBackend::run_plan(
    const common::PlanRequest& request, const common::GridView& grid,
    const collision::CollisionChecker& collision_checker,
    RsProvider& rs_provider) {
  const std::string& request_id = request.request_id;
  if (!configured_) {
    return base_result(profile_, request_id, Status::kInvalidInput,
                       "invalid input: backend configuration was rejected: " +
                           report_.reason);
  }
  if (!unsupported_reason_.empty()) {
    return base_result(profile_, request_id, Status::kInvalidInput,
                       "invalid input: unsupported parameters are set: " +
                           unsupported_reason_);
  }
  if (request.cancelled) {
    return base_result(profile_, request_id, Status::kCancelled,
                       "cancelled: request.cancelled is set before planning");
  }
  if (!pose_is_finite(request.start) || !pose_is_finite(request.goal)) {
    return base_result(profile_, request_id, Status::kInvalidInput,
                       "invalid input: start or goal pose is not finite");
  }
  if (!vehicle_model_valid(request.vehicle)) {
    return base_result(profile_, request_id, Status::kInvalidInput,
                       "invalid input: vehicle model parameters are invalid");
  }
  if (request.time_budget_s.has_value() && !(*request.time_budget_s > 0.0)) {
    return base_result(profile_, request_id, Status::kInvalidInput,
                       "invalid input: time_budget_s must be positive");
  }
  if (request.node_budget.has_value() && *request.node_budget < 1) {
    return base_result(profile_, request_id, Status::kInvalidInput,
                       "invalid input: node_budget must be at least 1");
  }
  if (!grid_valid(grid)) {
    return base_result(profile_, request_id, Status::kInvalidInput,
                       "invalid input: grid dimensions, resolution or cells "
                       "are invalid");
  }
  if (!request.map_id.empty() && request.map_id != grid.map_id) {
    return base_result(profile_, request_id, Status::kInvalidInput,
                       "invalid input: request map_id does not match the grid");
  }
  if (!request.map_version.empty() &&
      request.map_version != grid.map_version) {
    return base_result(profile_, request_id, Status::kInvalidInput,
                       "invalid input: request map_version does not match the "
                       "grid");
  }
  if (!collision_checker.valid()) {
    return base_result(profile_, request_id, Status::kInvalidInput,
                       "invalid input: collision checker is invalid: " +
                           collision_checker.invalid_reason());
  }
  if (!vehicles_equal(request.vehicle, collision_checker.vehicle())) {
    return base_result(profile_,
        request_id, Status::kInvalidInput,
        "invalid input: request vehicle does not match the collision checker "
        "vehicle");
  }
  if (!vehicles_equal(request.vehicle, rs_provider.vehicle())) {
    return base_result(profile_,
        request_id, Status::kInvalidInput,
        "invalid input: request vehicle does not match the RS provider "
        "vehicle");
  }
  if (!grids_equal(collision_checker.scene().grid, grid)) {
    return base_result(profile_,
        request_id, Status::kInvalidInput,
        "invalid input: the collision checker scene grid does not match the "
        "grid argument");
  }

  double min_turn_radius_m = 0.0;
  try {
    min_turn_radius_m = rs::min_turn_radius_m(request.vehicle);
  } catch (const std::exception& error) {
    return base_result(profile_, request_id, Status::kInvalidInput,
                       std::string("invalid input: min turn radius: ") +
                           error.what());
  }
  double curvature_scale = 1.0;
  if (config_.min_turn_radius_m > 0.0) {
    if (config_.min_turn_radius_m < min_turn_radius_m) {
      return base_result(profile_, request_id, Status::kInvalidInput,
          "invalid input: min_turn_radius_m is below the request vehicle's "
          "minimum turning radius");
    }
    curvature_scale = min_turn_radius_m / config_.min_turn_radius_m;
  }

  const collision::CheckResult start_check =
      collision_checker.check_pose(request.start);
  if (start_check.status == collision::CheckStatus::kInvalidInput) {
    return base_result(profile_, request_id, Status::kInvalidInput,
                       "invalid input: start pose was rejected: " +
                           collision_detail(start_check));
  }
  if (start_check.collided()) {
    return base_result(profile_, request_id, Status::kInvalidInput,
                       "invalid input: start pose is in collision (" +
                           collision_detail(start_check) + ")");
  }
  const collision::CheckResult goal_check =
      collision_checker.check_pose(request.goal);
  if (goal_check.status == collision::CheckStatus::kInvalidInput) {
    return base_result(profile_, request_id, Status::kInvalidInput,
                       "invalid input: goal pose was rejected: " +
                           collision_detail(goal_check));
  }
  if (goal_check.collided()) {
    return base_result(profile_, request_id, Status::kInvalidInput,
                       "invalid input: goal pose is in collision (" +
                           collision_detail(goal_check) + ")");
  }

  const collision::GridFrame grid_frame =
      collision::GridFrame::from_grid(grid);
  const auto cell_of = [&](const Pose2d& pose, int* row, int* col) {
    const common::Point2d grid_point =
        grid_frame.to_grid_frame(common::Point2d{pose.x_m, pose.y_m});
    const int c =
        static_cast<int>(std::floor(grid_point.x_m / grid.resolution_m));
    const int r =
        static_cast<int>(std::floor(grid_point.y_m / grid.resolution_m));
    if (!grid.in_bounds(r, c)) {
      return false;
    }
    *row = r;
    *col = c;
    return true;
  };
  int start_row = 0;
  int start_col = 0;
  int goal_row = 0;
  int goal_col = 0;
  if (!cell_of(request.start, &start_row, &start_col)) {
    return base_result(profile_, request_id, Status::kInvalidInput,
                       "invalid input: start rear-axle position is outside the "
                       "grid");
  }
  if (!cell_of(request.goal, &goal_row, &goal_col)) {
    return base_result(profile_, request_id, Status::kInvalidInput,
                       "invalid input: goal rear-axle position is outside the "
                       "grid");
  }

  const Clock::time_point plan_start = Clock::now();
  const std::uint64_t potential_key =
      internal::grid_cache_key(grid, goal_row, goal_col);
  const bool reused = cache_->valid && cache_->key == potential_key;
  if (!reused) {
    const double time_budget_s =
        request.time_budget_s.value_or(config_.default_time_budget_s);
    const auto deadline = [&]() {
      return elapsed_ms_since(plan_start) > time_budget_s * 1000.0;
    };
    internal::GridPotential potential =
        internal::compute_grid_potential(grid, goal_row, goal_col, deadline);
    if (potential.aborted) {
      PlanResult result = base_result(profile_,
          request_id, Status::kTimeout,
          "timeout: time budget " + std::to_string(time_budget_s) +
              " s exceeded while computing the goal potential");
      result.statistics["cache_mode"] = "cold";
      result.statistics["total_time_ms"] = elapsed_ms_since(plan_start);
      return result;
    }
    cache_->potential = std::move(potential);
    cache_->key = potential_key;
    cache_->valid = true;
  }

  const internal::GridPotential& potential = cache_->potential;
  const int start_index = start_row * grid.width + start_col;
  if (!std::isfinite(
          potential.distance_m[static_cast<std::size_t>(start_index)])) {
    PlanResult result = base_result(profile_,
        request_id, Status::kNoPath,
        "no path: start and goal are disconnected in the free-grid "
        "relaxation");
    result.statistics["cache_mode"] = reused ? "warm" : "cold";
    result.statistics["heuristic_cache_reused"] = reused;
    result.statistics["total_time_ms"] = elapsed_ms_since(plan_start);
    return result;
  }

  RosFreeSearch search(config_, request, grid, collision_checker, rs_provider,
                       potential, reused, min_turn_radius_m, curvature_scale,
                       profile_.id, to_string(profile_.kind), plan_start,
                       cancel_callback_);
  return search.run();
}

}  // namespace planner::hybrid_astar
