// planner_dftpav：预制路径 + 公共走廊 -> Dftpav 优化器。
//
// This adapter is original code of the NewStructPlanner project (see
// Src/dftpav/PROVENANCE.md); it drives the ported GPL-3.0 optimizer core and
// does not modify its math.
//
// 流程：
//   1. 校验请求（车辆、限制、初值、走廊）；
//   2. 把初值展平为带全局弧长的单条折线，并按恒定方向切分为若干运行段
//      （前进/后退，上游 `singul`）；
//   3. 构建上游输入元组：每段的 MINCO 边界状态、等时长的初始路径点，
//      以及每个约束点的走廊半平面（由 planner::common::Corridor 多边形转换）；
//   4. 调用移植的 PolyTrajOptimizer；
//   5. 把分段 Bezier 结果采样为 common::TrajectoryPoint，并运行临时性的
//      可行性检查。

#include "planner/dftpav/optimizer.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigen>

#include "planner/common/types.hpp"
#include "planner/dftpav/vehicle_param.hpp"

#include "traj_optimizer.hpp"

namespace planner::dftpav {
namespace {

namespace ported = planner::dftpav::ported;
using ported::plan_manage::OptimizerConfig;
using ported::plan_manage::PolyTrajOptimizer;
using ported::plan_utils::MinJerkOpt;
using ported::plan_utils::Trajectory;

using common::Direction;
using common::Point2d;
using common::Pose2d;
using common::TrajectoryPoint;

constexpr double kPositionEps = 1e-9;
constexpr double kProjectionEps = 1e-12;

bool finite(double value) { return std::isfinite(value); }

// 协作式取消（移植扩展，见 PROVENANCE.md 与公共头 OptimizeOptions::cancel_check）：
// 适配层把可选的 std::function 谓词桥接为移植核心的 C 风格回调 + 上下文。
// 上下文指向 `request.options.cancel_check`，在整个 optimize_trajectory 调用
// 期间有效；核心只调用它（const 调用运算符），从不修改。
bool ported_cancel_check(void* context) {
  const auto* check = static_cast<const std::function<bool()>*>(context);
  return check != nullptr && static_cast<bool>(*check) && (*check)();
}

// 适配层自身的取消轮询（空谓词 = 永不取消）。
bool cancel_requested(const OptimizeOptions& options) {
  return static_cast<bool>(options.cancel_check) && options.cancel_check();
}

double point_distance(const Point2d& a, const Point2d& b) {
  return std::hypot(a.x_m - b.x_m, a.y_m - b.y_m);
}

std::string format_double(double value) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.9g", value);
  return std::string(buffer);
}

// ---------------------------------------------------------------------------
// 初值展平。
// ---------------------------------------------------------------------------

struct SeedLine {
  std::vector<Point2d> points;  // 去重后的折线（全局坐标系）
  std::vector<double> s_m;      // 每个点的全局弧长
  double total_length_m{0.0};
};

struct DirectionRun {
  int singul{1};  // +1 前进，-1 后退
  std::size_t first_seed_segment{0};
  double s_begin_m{0.0};
  double s_end_m{0.0};
  Point2d begin;
  Point2d end;
};

bool is_strictly_convex_ccw(const std::vector<Point2d>& polygon) {
  const std::size_t n = polygon.size();
  if (n < 3) {
    return false;
  }
  for (std::size_t i = 0; i < n; ++i) {
    const Point2d& a = polygon[i];
    const Point2d& b = polygon[(i + 1) % n];
    const Point2d& c = polygon[(i + 2) % n];
    const double cross = (b.x_m - a.x_m) * (c.y_m - b.y_m) - (b.y_m - a.y_m) * (c.x_m - b.x_m);
    if (!(cross > 0.0)) {
      return false;
    }
  }
  return true;
}

Eigen::MatrixXd polygon_halfplanes(const std::vector<Point2d>& polygon,
                                   double inward_tightening_m = 0.0) {
  const std::size_t n = polygon.size();
  Eigen::MatrixXd h(4, static_cast<Eigen::Index>(n));
  for (std::size_t i = 0; i < n; ++i) {
    const Point2d& a = polygon[i];
    const Point2d& b = polygon[(i + 1) % n];
    const double ex = b.x_m - a.x_m;
    const double ey = b.y_m - a.y_m;
    const double length = std::hypot(ex, ey);
    const double nx = ey / length;
    const double ny = -ex / length;
    h(0, static_cast<Eigen::Index>(i)) = nx;
    h(1, static_cast<Eigen::Index>(i)) = ny;
    h(2, static_cast<Eigen::Index>(i)) = a.x_m - inward_tightening_m * nx;
    h(3, static_cast<Eigen::Index>(i)) = a.y_m - inward_tightening_m * ny;
  }
  return h;
}

// 把 `q` 最近点投影到折线上；返回插值得到的全局弧长。
double project_arc_length(const SeedLine& line, const Point2d& q) {
  double best_distance = std::numeric_limits<double>::infinity();
  double best_s = 0.0;
  for (std::size_t i = 0; i + 1 < line.points.size(); ++i) {
    const Point2d& a = line.points[i];
    const Point2d& b = line.points[i + 1];
    const double ex = b.x_m - a.x_m;
    const double ey = b.y_m - a.y_m;
    const double length2 = ex * ex + ey * ey;
    double t = 0.0;
    if (length2 > kProjectionEps) {
      t = ((q.x_m - a.x_m) * ex + (q.y_m - a.y_m) * ey) / length2;
      t = std::min(1.0, std::max(0.0, t));
    }
    const double px = a.x_m + t * ex;
    const double py = a.y_m + t * ey;
    const double distance = std::hypot(q.x_m - px, q.y_m - py);
    if (distance < best_distance) {
      best_distance = distance;
      best_s = line.s_m[i] + t * (line.s_m[i + 1] - line.s_m[i]);
    }
  }
  return best_s;
}

Point2d point_at_arc_length(const SeedLine& line, double s) {
  if (line.points.empty()) {
    return {};
  }
  s = std::min(std::max(s, 0.0), line.total_length_m);
  for (std::size_t i = 0; i + 1 < line.points.size(); ++i) {
    if (s <= line.s_m[i + 1]) {
      const double ds = line.s_m[i + 1] - line.s_m[i];
      const double t = ds > kProjectionEps ? (s - line.s_m[i]) / ds : 0.0;
      return Point2d{line.points[i].x_m + t * (line.points[i + 1].x_m - line.points[i].x_m),
                     line.points[i].y_m + t * (line.points[i + 1].y_m - line.points[i].y_m)};
    }
  }
  return line.points.back();
}

// 第一个 s 区间包含 s 的多边形；没有则为 -1。
int polygon_index_for_s(const common::Corridor& corridor, double s, double tolerance_m) {
  for (std::size_t i = 0; i < corridor.polygons.size(); ++i) {
    const common::CorridorPolygon& polygon = corridor.polygons[i];
    if (s >= polygon.s_start_m - tolerance_m && s <= polygon.s_end_m + tolerance_m) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

// ---------------------------------------------------------------------------
// 验证辅助函数。
// ---------------------------------------------------------------------------

struct ValidationError {
  common::Status status{common::Status::kInvalidInput};
  std::string reason;
  std::string detail;
};

bool validate_vehicle(const common::VehicleModel& vehicle, std::string* detail) {
  if (!(finite(vehicle.wheelbase_m) && vehicle.wheelbase_m > 0.0) ||
      !(finite(vehicle.width_m) && vehicle.width_m > 0.0) ||
      !(finite(vehicle.front_overhang_m) && vehicle.front_overhang_m >= 0.0) ||
      !(finite(vehicle.rear_overhang_m) && vehicle.rear_overhang_m >= 0.0) ||
      !(finite(vehicle.safety_margin_m) && vehicle.safety_margin_m >= 0.0) ||
      !(finite(vehicle.max_steer_rad) && vehicle.max_steer_rad > 0.0 &&
        vehicle.max_steer_rad < common::kPi * 0.5)) {
    *detail = "vehicle geometry invalid: wheelbase/width positive, overhangs and margin "
              "non-negative, max_steer_rad in (0, pi/2) are required";
    return false;
  }
  return true;
}

bool validate_limits(const OptimizeLimits& limits, std::string* detail) {
  const double values[] = {limits.max_forward_velocity_mps,
                           limits.max_backward_velocity_mps,
                           limits.max_forward_acceleration_mps2,
                           limits.max_backward_acceleration_mps2,
                           limits.max_forward_curvature_inv_m,
                           limits.max_backward_curvature_inv_m,
                           limits.max_lateral_acceleration_mps2};
  const char* names[] = {"max_forward_velocity_mps",
                         "max_backward_velocity_mps",
                         "max_forward_acceleration_mps2",
                         "max_backward_acceleration_mps2",
                         "max_forward_curvature_inv_m",
                         "max_backward_curvature_inv_m",
                         "max_lateral_acceleration_mps2"};
  for (std::size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
    if (!(finite(values[i]) && values[i] > 0.0)) {
      *detail = std::string(names[i]) + " must be finite and positive";
      return false;
    }
  }
  if (!(finite(limits.max_steering_rate_radps) && limits.max_steering_rate_radps > 0.0)) {
    *detail = "max_steering_rate_radps must be finite and positive";
    return false;
  }
  return true;
}

bool validate_options(const OptimizeOptions& options, std::string* detail) {
  if (options.traj_resolution < 1 || options.destraj_resolution < 1) {
    *detail = "traj_resolution and destraj_resolution must be >= 1";
    return false;
  }
  if (!(finite(options.static_obstacle_weight) && options.static_obstacle_weight >= 0.0) ||
      !(finite(options.feasibility_weight) && options.feasibility_weight >= 0.0) ||
      !(finite(options.time_weight) && options.time_weight >= 0.0)) {
    *detail = "cost weights must be finite and non-negative";
    return false;
  }
  if (!(finite(options.traj_piece_duration_s) && options.traj_piece_duration_s > 0.0) ||
      !(finite(options.min_piece_duration_s) && options.min_piece_duration_s > 0.0) ||
      !(finite(options.sample_dt_s) && options.sample_dt_s > 0.0)) {
    *detail = "piece/sample durations must be finite and positive";
    return false;
  }
  if (options.lbfgs_memsize < 1 || options.lbfgs_max_iterations < 1) {
    *detail = "lbfgs_memsize and lbfgs_max_iterations must be >= 1";
    return false;
  }
  if (!(finite(options.cost_fail_threshold) && options.cost_fail_threshold > 0.0)) {
    *detail = "cost_fail_threshold must be finite and positive";
    return false;
  }
  if (!(finite(options.anchor_weight) && options.anchor_weight >= 0.0)) {
    *detail = "anchor_weight must be finite and non-negative";
    return false;
  }
  if (!(finite(options.corridor_tightening_m) && options.corridor_tightening_m >= 0.0)) {
    *detail = "corridor_tightening_m must be finite and non-negative";
    return false;
  }
  if (!(finite(options.internal_curvature_tightening) &&
        options.internal_curvature_tightening >= 0.0 &&
        options.internal_curvature_tightening <= 0.1)) {
    *detail = "internal_curvature_tightening must be finite and within [0, 0.1]";
    return false;
  }
  if (options.hull_margin_m.has_value() &&
      !(finite(*options.hull_margin_m) && *options.hull_margin_m >= 0.0)) {
    *detail = "hull_margin_m must be finite and non-negative when set";
    return false;
  }
  if (!(finite(options.direction_switch_velocity_mps) &&
        options.direction_switch_velocity_mps >= 0.0)) {
    *detail = "direction_switch_velocity_mps must be finite and non-negative";
    return false;
  }
  if (!(finite(options.constraint_tolerance) && options.constraint_tolerance > 0.0)) {
    *detail = "constraint_tolerance must be finite and positive";
    return false;
  }
  if (!(finite(options.heading_report_min_speed_mps) &&
        options.heading_report_min_speed_mps >= 0.0)) {
    *detail = "heading_report_min_speed_mps must be finite and non-negative";
    return false;
  }
  if (!(finite(options.curvature_report_min_speed_mps) &&
        options.curvature_report_min_speed_mps >= 0.0)) {
    *detail = "curvature_report_min_speed_mps must be finite and non-negative";
    return false;
  }
  return true;
}

// 把初值段展平进 `line` 并构建方向运行段。
// 遇到错误输入时向 `detail` 追加错误消息并返回 false。
bool build_seed_line(const std::vector<common::PathSegment>& seed, SeedLine* line,
                     std::vector<DirectionRun>* runs, std::string* detail) {
  if (seed.empty()) {
    *detail = "seed is empty";
    return false;
  }
  line->points.clear();
  line->s_m.clear();
  runs->clear();

  // 全局弧长按几何方式累加（点到点距离）。上游生产者可能把每段的 `s_m`
  // 设为从零重新开始或全局连续（Hybrid A* ros_free 路径使用全局值），
  // 因此 `s_m` 只检查每段内的单调性，不用于折线参数化。
  double global_s = 0.0;
  Point2d previous_point{};
  bool have_previous = false;

  for (std::size_t segment_index = 0; segment_index < seed.size(); ++segment_index) {
    const common::PathSegment& segment = seed[segment_index];
    if (segment.points.size() < 2) {
      *detail = "seed segment " + std::to_string(segment_index) + " has fewer than 2 points";
      return false;
    }
    double previous_local_s = -1.0;
    bool first_of_segment = true;
    Point2d segment_begin{};
    Point2d segment_end{};
    double segment_s_begin = global_s;
    double segment_s_end = global_s;
    for (std::size_t k = 0; k < segment.points.size(); ++k) {
      const common::PathPoint& path_point = segment.points[k];
      const Pose2d& pose = path_point.pose;
      if (!(finite(pose.x_m) && finite(pose.y_m) && finite(pose.yaw_rad) &&
            finite(path_point.s_m) && finite(path_point.curvature_inv_m))) {
        *detail = "seed contains non-finite values (segment " + std::to_string(segment_index) +
                  ", point " + std::to_string(k) + ")";
        return false;
      }
      if (path_point.s_m < previous_local_s) {
        *detail = "seed point arc length not non-decreasing (segment " +
                  std::to_string(segment_index) + ")";
        return false;
      }
      previous_local_s = path_point.s_m;
      const Point2d point{pose.x_m, pose.y_m};
      if (first_of_segment) {
        segment_begin = point;
        segment_s_begin = global_s;
        first_of_segment = false;
        if (have_previous) {
          // 分段之间的连接点。当前 Hybrid A* 后端输出的零间隙路径中，
          // 换向位姿会在相邻两段中各出现一次（s_m 相同、direction 字段相反；
          // hybrid-003 证据）；重复点处的几何弧长保持连续。优化器在相邻
          // 运行段之间共享一个连接点变量（第 i 段的尾位置成为第 i+1 段的头）。
          // 下面 <= 0.75 m 的吸附仅是防御性的输入清理；更大的间隙
          // 按畸形输入拒绝。
          const double gap = point_distance(point, previous_point);
          if (gap > 0.75) {
            *detail = "seed segments are not geometrically continuous (gap " +
                      format_double(gap) + " m at segment " +
                      std::to_string(segment_index) + ")";
            return false;
          }
        }
      } else {
        const double step = point_distance(point, previous_point);
        if (step > kPositionEps) {
          global_s += step;
        } else if (step < kPositionEps) {
          continue;  // 完全重复的点
        }
      }
      segment_end = point;
      segment_s_end = global_s;
      line->points.push_back(point);
      line->s_m.push_back(global_s);
      previous_point = point;
      have_previous = true;
    }
    // 结束/延长当前运行段。
    const int singul = segment.points.front().direction == Direction::kForward ? 1 : -1;
    if (!runs->empty() && runs->back().singul == singul &&
        runs->back().s_end_m == segment_s_begin) {
      runs->back().s_end_m = segment_s_end;
      runs->back().end = segment_end;
    } else {
      runs->push_back(DirectionRun{singul, segment_index, segment_s_begin, segment_s_end,
                                   segment_begin, segment_end});
    }
  }
  line->total_length_m = line->s_m.empty() ? 0.0 : line->s_m.back();
  if (line->points.size() < 2 || line->total_length_m <= 1e-6) {
    *detail = "seed path has zero length";
    return false;
  }
  return true;
}

}  // namespace

// ===========================================================================
// 公开入口。
// ===========================================================================

namespace {

// 保守整车足迹的角点（validation/碰撞检测约定：
// 车辆矩形 + 每边 safety_margin_m）。
std::array<Point2d, 4> conservative_footprint(const common::VehicleModel& vehicle,
                                              const Pose2d& pose) {
  const double margin = vehicle.safety_margin_m;
  const double length =
      vehicle.wheelbase_m + vehicle.front_overhang_m + vehicle.rear_overhang_m;
  const double d_cr =
      (vehicle.front_overhang_m + vehicle.wheelbase_m - vehicle.rear_overhang_m) / 2.0;
  const double half_length = 0.5 * (length + 2.0 * margin);
  const double half_width = 0.5 * (vehicle.width_m + 2.0 * margin);
  const double cos_yaw = std::cos(pose.yaw_rad);
  const double sin_yaw = std::sin(pose.yaw_rad);
  const double body[4][2] = {{d_cr + half_length, half_width},
                             {d_cr + half_length, -half_width},
                             {d_cr - half_length, -half_width},
                             {d_cr - half_length, half_width}};
  std::array<Point2d, 4> corners;
  for (int i = 0; i < 4; ++i) {
    corners[i] = Point2d{pose.x_m + cos_yaw * body[i][0] - sin_yaw * body[i][1],
                         pose.y_m + sin_yaw * body[i][0] + cos_yaw * body[i][1]};
  }
  return corners;
}

// 足迹相对单个逆时针凸多边形的最大半平面违规量；
// 同时写出最严重违规边的外法线。
double footprint_polygon_violation(const std::vector<Point2d>& polygon,
                                   const std::array<Point2d, 4>& corners,
                                   Point2d* outward_normal) {
  double worst = -std::numeric_limits<double>::infinity();
  Point2d worst_normal{0.0, 0.0};
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const Point2d& a = polygon[i];
    const Point2d& b = polygon[(i + 1) % polygon.size()];
    const double ex = b.x_m - a.x_m;
    const double ey = b.y_m - a.y_m;
    const double length = std::hypot(ex, ey);
    if (length <= kPositionEps) {
      continue;
    }
    const double nx = ey / length;
    const double ny = -ex / length;
    for (const Point2d& corner : corners) {
      const double violation = nx * (corner.x_m - a.x_m) + ny * (corner.y_m - a.y_m);
      if (violation > worst) {
        worst = violation;
        worst_normal = Point2d{nx, ny};
      }
    }
  }
  if (outward_normal != nullptr) {
    *outward_normal = worst_normal;
  }
  return worst;
}

struct CorridorRepairResult {
  std::vector<Trajectory> trajectories;
  double worst_violation{std::numeric_limits<double>::infinity()};
  int iterations{0};
};

// 确定性的求解后投影修复（Wave 7，dftpav-002）。
//
// 上游优化器用平滑惩罚施加走廊约束，因此活跃边界会保留 1e-4..1e-3 m 的
// 残差，而 planner::validation（对保守足迹的容差为 1e-6 m）会拒绝它。
// 本流程保持 MINCO 表示：提取每个运行段的节点位置与边界状态，用轨迹自身的
// 累计弧长（validation 所用的关联方式）测量保守足迹相对走廊多边形的违规，
// 沿被违反的半平面法线向内移动相邻节点并重新生成轨迹
// （MinJerkOpt::generate）。最多迭代 `kMaxIterations` 次并返回最优迭代结果。
// 最终判定仍属于 planner::validation；本修复从不放宽它，也从不移动固定的
// 起点/终点位置。
// sin(x)/x，与 planner::validation 的 sinc 一致（供 pose_on_arc 使用）。
double sinc_ratio(double x) {
  return std::abs(x) < 1e-8 ? 1.0 : std::sin(x) / x;
}

// planner::validation 对单个存储步长的圆弧重建。
struct ReconstructedStep {
  bool valid{false};
  double arc_m{0.0};
  int sign{1};
  double curvature_inv_m{0.0};
};

ReconstructedStep reconstruct_step(const Pose2d& from, const Pose2d& to) {
  ReconstructedStep step;
  const double dx = to.x_m - from.x_m;
  const double dy = to.y_m - from.y_m;
  const double chord = std::hypot(dx, dy);
  if (!(chord > 1e-12)) {
    return step;
  }
  const double dtheta = common::normalize_angle_rad(to.yaw_rad - from.yaw_rad);
  double arc = chord;
  if (std::abs(dtheta) > 1e-12) {
    const double half = 0.5 * dtheta;
    arc = chord * half / std::sin(half);
  }
  const double chord_angle = std::atan2(dy, dx);
  const double forward_error = common::normalize_angle_rad(
      chord_angle - (from.yaw_rad + 0.5 * dtheta));
  const bool forward = std::abs(forward_error) <= 0.5 * common::kPi;
  const double sigma = forward ? arc : -arc;
  const double curvature = dtheta / sigma;
  if (!(arc > 0.0) || !std::isfinite(curvature) ||
      std::abs(curvature) > 1.0e6) {
    return step;
  }
  step.valid = true;
  step.arc_m = arc;
  step.sign = forward ? 1 : -1;
  step.curvature_inv_m = curvature;
  return step;
}

// planner::validation 的 pose_on_arc。
Pose2d pose_on_arc(const Pose2d& start, double sigma, double kappa) {
  const double b = kappa * sigma;
  const double cy = std::cos(start.yaw_rad);
  const double sy = std::sin(start.yaw_rad);
  const double half = 0.5 * b;
  const double sb = sinc_ratio(b);
  const double h = half * sinc_ratio(half) * sinc_ratio(half);
  const double dsin = cy * sb - sy * h;
  const double dcos = -sy * sb - cy * h;
  Pose2d out;
  out.x_m = start.x_m + sigma * dsin;
  out.y_m = start.y_m - sigma * dcos;
  out.yaw_rad = common::normalize_angle_rad(start.yaw_rad + b);
  return out;
}

// 确定性的节点空间可行性恢复（Wave 9，dftpav-002）。
//
// 软约束解会在活跃走廊约束上保留 1e-4..1e-3 m 的残差（在粗参数化下，
// 相对预制初值还会有分米级的路径偏差），planner::validation（对保守足迹
// 1e-6 m）会拒绝它。本流程在 MINCO 节点空间中恢复可行性：
//   * 变量是内部节点位置与连接点位置；边界状态（固定起点/终点、边界导数）
//     保持不变；
//   * 约束关联与 planner::validation 完全一致：相同的存储采样网格与航向
//     策略、相同的 chord*sinc 弧长累加，以及相邻存储采样点之间相同的
//     圆弧插值位姿（只检查存储采样点会低估 validation 看到的违规量）；
//   * 违规位姿贡献线性化的半平面约束
//     n^T (p + J dx - a) <= -(violation - target)，其中 J 是相对节点变量的
//     有限差分位置雅可比（中间位姿混合相邻两个存储采样的雅可比）；
//   * 循环投影求解给出最小范数修正，配合回溯阻尼与外层迭代施加；评价
//     指标把包含违规量与曲率超限量合并，因此绝不会用包含性换取曲率
//     违规；保留最优迭代结果。
// 最终判定仍属 planner::validation；不放宽任何容差或安全余量
// （目标为 1e-4 m 的内部余量）。
CorridorRepairResult repair_corridor_containment(
    const OptimizeRequest& request, const std::vector<Trajectory>& input,
    const std::vector<Trajectory>& initial_guess, const std::vector<int>& singuls,
    double target_margin_m) {
  CorridorRepairResult result;
  result.trajectories = input;
  const std::size_t runs = input.size();
  if (runs == 0 || request.corridor.polygons.empty()) {
    result.worst_violation = 0.0;
    return result;
  }

  struct Shape {
    std::vector<int> pieces;
    std::vector<double> dts;
    std::vector<Eigen::MatrixXd> heads;
    std::vector<Eigen::MatrixXd> tails;
    std::vector<Eigen::MatrixXd> knots;
    std::vector<Eigen::Vector2d> boundaries;
  };
  Shape base;
  base.pieces.resize(runs);
  base.dts.resize(runs);
  base.heads.resize(runs);
  base.tails.resize(runs);
  base.knots.resize(runs);
  base.boundaries.resize(runs + 1);
  for (std::size_t j = 0; j < runs; ++j) {
    base.pieces[j] = input[j].getPieceNum();
    if (base.pieces[j] < 2) {
      result.worst_violation = 0.0;
      return result;
    }
    base.dts[j] = input[j][0].getDuration();
    const double total = base.dts[j] * base.pieces[j];
    base.heads[j].resize(2, 3);
    base.tails[j].resize(2, 3);
    base.heads[j].col(0) = input[j].getPos(0.0);
    base.heads[j].col(1) = input[j].getdSigma(0.0);
    base.heads[j].col(2) = input[j].getddSigma(0.0);
    base.tails[j].col(0) = input[j].getPos(total);
    base.tails[j].col(1) = input[j].getdSigma(total);
    base.tails[j].col(2) = input[j].getddSigma(total);
    base.knots[j].resize(2, base.pieces[j] - 1);
    for (int i = 1; i < base.pieces[j]; ++i) {
      base.knots[j].col(i - 1) = input[j].getPos(i * base.dts[j]);
    }
    if (j == 0) {
      base.boundaries[0] = base.heads[0].col(0);
    }
    base.boundaries[j + 1] = base.tails[j].col(0);
  }

  std::vector<int> knot_offset(runs, 0);
  int variable_count = 0;
  for (std::size_t j = 0; j < runs; ++j) {
    knot_offset[j] = variable_count;
    variable_count += 2 * (base.pieces[j] - 1);
  }
  const int junction_offset = variable_count;
  variable_count += 2 * (static_cast<int>(runs) - 1);
  if (variable_count == 0) {
    result.worst_violation = 0.0;
    return result;
  }

  Eigen::VectorXd x = Eigen::VectorXd::Zero(variable_count);
  for (std::size_t j = 0; j < runs; ++j) {
    for (int i = 0; i < base.pieces[j] - 1; ++i) {
      x(knot_offset[j] + 2 * i) = base.knots[j](0, i);
      x(knot_offset[j] + 2 * i + 1) = base.knots[j](1, i);
    }
  }
  for (std::size_t j = 1; j < runs; ++j) {
    x(junction_offset + 2 * (static_cast<int>(j) - 1)) = base.boundaries[j](0);
    x(junction_offset + 2 * (static_cast<int>(j) - 1) + 1) = base.boundaries[j](1);
  }

  // 初值变量向量（与节点/连接点布局相同）：边界层预条件器的混合目标。
  Eigen::VectorXd x_initial = x;
  if (initial_guess.size() == runs) {
    for (std::size_t j = 0; j < runs; ++j) {
      if (initial_guess[j].getPieceNum() != base.pieces[j]) {
        continue;
      }
      // 初值有自己的分段时长；按其总时长的同一比例采样，使混合目标复现
      // 初值路径而非扭曲它。
      const double initial_total =
          initial_guess[j][0].getDuration() * initial_guess[j].getPieceNum();
      const double optimized_total = base.dts[j] * base.pieces[j];
      for (int i = 1; i < base.pieces[j]; ++i) {
        const double fraction =
            (static_cast<double>(i) * base.dts[j]) / optimized_total;
        const Eigen::Vector2d position = initial_guess[j].getPos(fraction * initial_total);
        x_initial(knot_offset[j] + 2 * (i - 1)) = position(0);
        x_initial(knot_offset[j] + 2 * (i - 1) + 1) = position(1);
      }
      if (j > 0) {
        const Eigen::Vector2d head_position = initial_guess[j].getPos(0.0);
        x_initial(junction_offset + 2 * (static_cast<int>(j) - 1)) = head_position(0);
        x_initial(junction_offset + 2 * (static_cast<int>(j) - 1) + 1) = head_position(1);
      }
      if (j + 1 < runs) {
        const Eigen::Vector2d tail_position = initial_guess[j].getPos(initial_total);
        x_initial(junction_offset + 2 * static_cast<int>(j)) = tail_position(0);
        x_initial(junction_offset + 2 * static_cast<int>(j) + 1) = tail_position(1);
      }
    }
  }

  const auto regenerate = [&](const Eigen::VectorXd& vars,
                              std::vector<Trajectory>* out) {
    out->resize(runs);
    for (std::size_t j = 0; j < runs; ++j) {
      Eigen::MatrixXd knots(2, base.pieces[j] - 1);
      for (int i = 0; i < base.pieces[j] - 1; ++i) {
        knots(0, i) = vars(knot_offset[j] + 2 * i);
        knots(1, i) = vars(knot_offset[j] + 2 * i + 1);
      }
      Eigen::MatrixXd head = base.heads[j];
      Eigen::MatrixXd tail = base.tails[j];
      if (j == 0) {
        head.col(0) = base.boundaries[0];
      } else {
        head.col(0) = Eigen::Vector2d(vars(junction_offset + 2 * (static_cast<int>(j) - 1)),
                                      vars(junction_offset + 2 * (static_cast<int>(j) - 1) + 1));
      }
      if (j + 1 == runs) {
        tail.col(0) = base.boundaries[runs];
      } else {
        tail.col(0) = Eigen::Vector2d(vars(junction_offset + 2 * static_cast<int>(j)),
                                      vars(junction_offset + 2 * static_cast<int>(j) + 1));
      }
      MinJerkOpt min_jerk;
      min_jerk.reset(base.pieces[j]);
      min_jerk.generate(knots, base.dts[j], head, tail);
      (*out)[j] = min_jerk.getTraj(singuls[j]);
    }
  };

  // 与最终采样器相同的存储采样网格与航向策略。
  const double sample_dt = request.options.sample_dt_s;
  const Pose2d seed_start = request.seed.front().points.front().pose;
  const Pose2d seed_end = request.seed.back().points.back().pose;
  struct Sample {
    int run;
    double t;
    Pose2d pose;
  };
  const auto build_samples = [&](const std::vector<Trajectory>& trajectories) {
    std::vector<Sample> samples;
    double previous_yaw = 0.0;
    bool have_previous_yaw = false;
    double previous_global_time = -1.0;
    double run_start_time = 0.0;
    for (std::size_t j = 0; j < runs; ++j) {
      const Trajectory& trajectory = trajectories[j];
      const double total = base.dts[j] * base.pieces[j];
      std::size_t k = 0;
      while (true) {
        double t_local = static_cast<double>(k) * sample_dt;
        if (t_local >= total) {
          t_local = total;
        }
        ++k;
        const double t_global = run_start_time + t_local;
        if (t_global <= previous_global_time) {
          if (t_local >= total) {
            break;
          }
          continue;
        }
        const bool last_sample = (j + 1 == runs) && t_local >= total;
        const double velocity = trajectory.getVel(t_local);
        const Eigen::Vector2d position = trajectory.getPos(t_local);
        Pose2d pose;
        pose.x_m = position(0);
        pose.y_m = position(1);
        if (std::abs(velocity) > request.options.heading_report_min_speed_mps) {
          pose.yaw_rad = common::normalize_angle_rad(trajectory.getAngle(t_local));
          have_previous_yaw = true;
        } else if (last_sample) {
          pose.yaw_rad = seed_end.yaw_rad;
          have_previous_yaw = true;
        } else if (have_previous_yaw) {
          pose.yaw_rad = previous_yaw;
        } else {
          pose.yaw_rad = seed_start.yaw_rad;
          have_previous_yaw = true;
        }
        previous_yaw = pose.yaw_rad;
        previous_global_time = t_global;
        samples.push_back({static_cast<int>(j), t_local, pose});
        if (t_local >= total) {
          break;
        }
      }
      run_start_time += total;
    }
    return samples;
  };

  // 与 planner::validation 相同的弧长关联：只有有效的重建步长才推进游标。
  const auto cumulative_s = [&](const std::vector<Sample>& samples) {
    std::vector<double> s(samples.size(), 0.0);
    for (std::size_t i = 1; i < samples.size(); ++i) {
      const ReconstructedStep step =
          reconstruct_step(samples[i - 1].pose, samples[i].pose);
      s[i] = s[i - 1] + (step.valid ? step.arc_m : 0.0);
    }
    return s;
  };

  // 一个违规位姿：存储采样点（b < 0），或重建步长 a -> b 在 `fraction` 处的
  // 中间位姿。
  struct CheckPose {
    int sample_a{0};
    int sample_b{-1};
    double fraction{0.0};
    Eigen::Vector2d normal{Eigen::Vector2d::Zero()};
    double violation{0.0};
  };

  const auto evaluate = [&](const std::vector<Trajectory>& trajectories,
                            std::vector<Sample>* samples,
                            std::vector<double>* s_values,
                            std::vector<CheckPose>* checks,
                            double* curvature_excess) {
    *samples = build_samples(trajectories);
    *s_values = cumulative_s(*samples);
    checks->clear();
    *curvature_excess = 0.0;
    double worst = -std::numeric_limits<double>::infinity();
    const auto check_pose = [&](const Pose2d& pose, double s, int sample_a,
                                int sample_b, double fraction) {
      const std::array<Point2d, 4> corners =
          conservative_footprint(request.vehicle, pose);
      bool covered = false;
      double best_violation = std::numeric_limits<double>::infinity();
      Point2d best_normal{0.0, 0.0};
      for (const common::CorridorPolygon& polygon : request.corridor.polygons) {
        if (s < polygon.s_start_m - 1e-9 || s > polygon.s_end_m + 1e-9) {
          continue;
        }
        covered = true;
        Point2d normal{0.0, 0.0};
        const double violation =
            footprint_polygon_violation(polygon.vertices, corners, &normal);
        if (violation < best_violation) {
          best_violation = violation;
          best_normal = normal;
        }
      }
      if (!covered) {
        return;
      }
      worst = std::max(worst, best_violation);
      if (best_violation > target_margin_m) {
        CheckPose check;
        check.sample_a = sample_a;
        check.sample_b = sample_b;
        check.fraction = fraction;
        check.normal = Eigen::Vector2d(best_normal.x_m, best_normal.y_m);
        check.violation = best_violation;
        checks->push_back(check);
      }
    };
    for (std::size_t i = 0; i < samples->size(); ++i) {
      const Sample& sample = (*samples)[i];
      const Trajectory& trajectory = trajectories[static_cast<std::size_t>(sample.run)];
      const double velocity = trajectory.getVel(sample.t);
      if (std::abs(velocity) > request.options.curvature_report_min_speed_mps) {
        const double curvature = std::abs(trajectory.getCurv(sample.t));
        const double limit = singuls[static_cast<std::size_t>(sample.run)] > 0
                                 ? request.limits.max_forward_curvature_inv_m
                                 : request.limits.max_backward_curvature_inv_m;
        *curvature_excess = std::max(*curvature_excess, curvature - limit);
      }
      check_pose(sample.pose, (*s_values)[i], static_cast<int>(i), -1, 0.0);
    }
    // 中间位姿与 planner::validation 的采样方式完全一致。
    // planner::validation 的默认走廊重采样步长；适配器
    // 不覆盖 validation 选项。
    constexpr double kValidationCorridorSampleStepM = 0.02;
    const double step_m = kValidationCorridorSampleStepM;
    for (std::size_t i = 0; i + 1 < samples->size(); ++i) {
      const ReconstructedStep step =
          reconstruct_step((*samples)[i].pose, (*samples)[i + 1].pose);
      if (!step.valid) {
        continue;
      }
      const std::size_t steps = std::max<std::size_t>(
          1, static_cast<std::size_t>(std::ceil(step.arc_m / step_m)));
      const double sigma_sign = step.sign > 0 ? 1.0 : -1.0;
      for (std::size_t k = 1; k < steps; ++k) {
        const double fraction =
            static_cast<double>(k) / static_cast<double>(steps);
        const Pose2d pose = pose_on_arc((*samples)[i].pose,
                                        sigma_sign * step.arc_m * fraction,
                                        step.curvature_inv_m);
        check_pose(pose, (*s_values)[i] + step.arc_m * fraction,
                   static_cast<int>(i), static_cast<int>(i + 1), fraction);
      }
    }
    return worst;
  };

  const auto merit = [](double containment_worst, double curvature_excess) {
    return std::max(containment_worst, curvature_excess);
  };

  std::vector<Sample> samples;
  std::vector<double> s_values;
  std::vector<CheckPose> checks;
  double current_excess = 0.0;
  std::vector<Trajectory> current = input;
  double current_worst =
      evaluate(current, &samples, &s_values, &checks, &current_excess);
  std::vector<Trajectory> best = current;
  double best_worst = current_worst;
  double best_excess = current_excess;
  if (best_worst <= target_margin_m && best_excess <= 0.0) {
    result.trajectories = std::move(best);
    result.worst_violation = best_worst;
    return result;
  }

  // 预条件：优化器的局部极小值可能在固定边界状态附近带有曲率残差，任何
  // 位置修正都无法消除；把节点向初值（它沿预制路径，不带此类残差）混合，
  // 能给恢复过程一个更干净的起点。取无曲率超限的最小混合系数。
  {
    double chosen_alpha = 0.0;
    std::vector<Trajectory> chosen;
    std::vector<Sample> chosen_samples;
    std::vector<double> chosen_s;
    std::vector<CheckPose> chosen_checks;
    double chosen_worst = current_worst;
    double chosen_excess = current_excess;
    const double blends[] = {0.25, 0.5, 0.75, 1.0};
    for (double alpha : blends) {
      const Eigen::VectorXd x_blend = (1.0 - alpha) * x + alpha * x_initial;
      std::vector<Trajectory> candidate;
      regenerate(x_blend, &candidate);
      std::vector<Sample> candidate_samples;
      std::vector<double> candidate_s;
      std::vector<CheckPose> candidate_checks;
      double candidate_excess = 0.0;
      const double candidate_worst = evaluate(candidate, &candidate_samples, &candidate_s,
                                              &candidate_checks, &candidate_excess);
      if (candidate_excess <= 0.0) {
        chosen_alpha = alpha;
        chosen = std::move(candidate);
        chosen_samples = std::move(candidate_samples);
        chosen_s = std::move(candidate_s);
        chosen_checks = std::move(candidate_checks);
        chosen_worst = candidate_worst;
        chosen_excess = candidate_excess;
        break;
      }
    }
    if (chosen_alpha > 0.0) {
      x = (1.0 - chosen_alpha) * x + chosen_alpha * x_initial;
      current = std::move(chosen);
      current_worst = chosen_worst;
      current_excess = chosen_excess;
      samples = std::move(chosen_samples);
      s_values = std::move(chosen_s);
      checks = std::move(chosen_checks);
    }
    if (current_worst < best_worst ||
        (current_worst == best_worst && current_excess < best_excess)) {
      best = current;
      best_worst = current_worst;
      best_excess = current_excess;
    }
  }

  constexpr double kFiniteDifferenceStep = 1e-4;
  constexpr int kMaxIterations = 12;
  constexpr int kProjectionSweeps = 120;
  constexpr std::size_t kMaxChecks = 400;
  // 每个运行段首/末节点的修正会被阻尼：移动贴近边界的节点正是把包含性
  // 修正变成固定起点/终点边界状态附近曲率尖峰的原因。
  Eigen::VectorXd variable_weight = Eigen::VectorXd::Ones(variable_count);
  for (std::size_t j = 0; j < runs; ++j) {
    if (base.pieces[j] >= 2) {
      variable_weight(knot_offset[j]) = 0.25;
      variable_weight(knot_offset[j] + 1) = 0.25;
      variable_weight(knot_offset[j] + 2 * (base.pieces[j] - 2)) = 0.25;
      variable_weight(knot_offset[j] + 2 * (base.pieces[j] - 2) + 1) = 0.25;
    }
  }
  for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
    if (cancel_requested(request.options)) {
      // 协作式取消（移植扩展）：修复是有界循环，但每次迭代仍要能及早退出；
      // 调用方（run_attempt）在取消时丢弃这里的结果。
      break;
    }
    if (current_worst <= target_margin_m && current_excess <= 0.0) {
      break;
    }
    if (checks.empty()) {
      break;
    }
    // 检查过多时保留最严重的（确定性顺序）。
    if (checks.size() > kMaxChecks) {
      std::partial_sort(checks.begin(), checks.begin() + kMaxChecks, checks.end(),
                        [](const CheckPose& l, const CheckPose& r) {
                          return l.violation > r.violation;
                        });
      checks.resize(kMaxChecks);
    }
    // 所有存储采样点的有限差分位置雅可比。
    std::vector<Eigen::Vector2d> base_positions(samples.size());
    for (std::size_t i = 0; i < samples.size(); ++i) {
      base_positions[i] = Eigen::Vector2d(samples[i].pose.x_m, samples[i].pose.y_m);
    }
    const int sample_count = static_cast<int>(samples.size());
    Eigen::MatrixXd jacobian(2 * sample_count, variable_count);
    jacobian.setZero();
    std::vector<Trajectory> perturbed;
    for (int v = 0; v < variable_count; ++v) {
      Eigen::VectorXd x_perturbed = x;
      x_perturbed(v) += kFiniteDifferenceStep;
      regenerate(x_perturbed, &perturbed);
      const std::vector<Sample> perturbed_samples = build_samples(perturbed);
      if (perturbed_samples.size() != samples.size()) {
        continue;
      }
      for (int i = 0; i < sample_count; ++i) {
        jacobian(2 * i, v) =
            (perturbed_samples[static_cast<std::size_t>(i)].pose.x_m - base_positions[i](0)) /
            kFiniteDifferenceStep;
        jacobian(2 * i + 1, v) =
            (perturbed_samples[static_cast<std::size_t>(i)].pose.y_m - base_positions[i](1)) /
            kFiniteDifferenceStep;
      }
    }
    Eigen::MatrixXd a(static_cast<int>(checks.size()), variable_count);
    Eigen::VectorXd b(static_cast<int>(checks.size()));
    for (std::size_t r = 0; r < checks.size(); ++r) {
      const CheckPose& check = checks[r];
      // 位置雅可比行：n^T J，中间位姿时混合两个存储采样点。
      Eigen::RowVectorXd jx = jacobian.row(2 * check.sample_a);
      Eigen::RowVectorXd jy = jacobian.row(2 * check.sample_a + 1);
      if (check.sample_b >= 0) {
        jx = (1.0 - check.fraction) * jx + check.fraction * jacobian.row(2 * check.sample_b);
        jy = (1.0 - check.fraction) * jy +
             check.fraction * jacobian.row(2 * check.sample_b + 1);
      }
      a.row(static_cast<int>(r)) = check.normal(0) * jx + check.normal(1) * jy;
      b(static_cast<int>(r)) = -(check.violation - target_margin_m);
    }
    // 向线性化半平面做循环投影（最小范数修正）。
    Eigen::VectorXd delta = Eigen::VectorXd::Zero(variable_count);
    for (int sweep = 0; sweep < kProjectionSweeps; ++sweep) {
      for (int r = 0; r < a.rows(); ++r) {
        const double norm2 = a.row(r).squaredNorm();
        if (norm2 <= 1e-12) {
          continue;
        }
        const double slack = a.row(r).dot(delta) - b(r);
        if (slack > 0.0) {
          delta -= (slack / norm2) * a.row(r).transpose();
        }
      }
    }
    // 带阻尼的施加，并对评价指标做回溯。
    delta = delta.cwiseProduct(variable_weight);
    bool improved = false;
    double alpha = 1.0;
    const double current_merit = merit(current_worst, current_excess);
    for (int attempt = 0; attempt < 4; ++attempt) {
      Eigen::VectorXd x_candidate = x + alpha * delta;
      std::vector<Trajectory> candidate;
      regenerate(x_candidate, &candidate);
      std::vector<Sample> candidate_samples;
      std::vector<double> candidate_s;
      std::vector<CheckPose> candidate_checks;
      double candidate_excess = 0.0;
      const double candidate_worst =
          evaluate(candidate, &candidate_samples, &candidate_s, &candidate_checks,
                   &candidate_excess);
      if (merit(candidate_worst, candidate_excess) < current_merit) {
        x = x_candidate;
        current = std::move(candidate);
        current_worst = candidate_worst;
        current_excess = candidate_excess;
        samples = std::move(candidate_samples);
        s_values = std::move(candidate_s);
        checks = std::move(candidate_checks);
        improved = true;
        break;
      }
      alpha *= 0.5;
    }
    ++result.iterations;
    if (merit(current_worst, current_excess) < merit(best_worst, best_excess)) {
      best_worst = current_worst;
      best_excess = current_excess;
      best = current;
    }
    if (!improved) {
      break;
    }
  }
  result.trajectories = std::move(best);
  result.worst_violation = best_worst;
  return result;
}

// 基础尝试与重试阶梯统一使用的内部限制收紧（5 %，只收紧内部软约束预算）。
constexpr double kBaseLimitTightening = 5e-2;

// 单次优化尝试的适配层调参。所有字段只影响优化器的内部软约束预算或初值
// 参数化（从不改变 planner::validation 的硬门禁）：
//   * `tightening`：速度/加速度/横向加速度内部限制的相对收紧；
//   * `piece_duration_s`：初值分段时长（越小初值网格越密）；
//   * `geometry_tightening_m`：内部外壳相对验证足迹的额外膨胀；
//   * `corridor_weight_scale`：走廊软约束权重倍率；
//   * `anchor_weight_scale`：初值锚定权重倍率（越大越贴紧预制初值）；
//   * `curvature_tightening`：内部曲率限制的相对收紧（越大越紧）；
//   * `feasibility_weight_scale`：可行性惩罚权重倍率（越大越贴近约束）；
//   * `traj_resolution`/`destraj_resolution`：>0 时作为走廊约束采样分辨率的
//     下限（只升不降，避免覆盖调用方更高的设置）。
struct AttemptTuning {
  double tightening{kBaseLimitTightening};
  double piece_duration_s{1.0};
  double geometry_tightening_m{0.01};
  double corridor_weight_scale{1.0};
  double anchor_weight_scale{1.0};
  double curvature_tightening{0.01};
  double feasibility_weight_scale{1.0};
  int traj_resolution{0};
  int destraj_resolution{0};
};

// 在给定调参下的一次优化器尝试。
// `request` 已经过校验，`line`/`runs` 已由
// optimize_trajectory 构建；以下全部运行在移植的核心上。
OptimizeResult run_attempt(const OptimizeRequest& request, const SeedLine& line,
                           const std::vector<DirectionRun>& runs,
                           const AttemptTuning& tuning) {
  OptimizeResult result;
  const OptimizeOptions& options = request.options;
  const OptimizeLimits& limits = request.limits;
  result.stats.direction_changes =
      runs.empty() ? 0 : static_cast<int>(runs.size()) - 1;

  auto fail = [&result](common::Status status, const std::string& reason,
                        const std::string& detail) -> OptimizeResult {
    result.status = status;
    result.reason = reason;
    result.detail = detail;
    return result;
  };

  // ---------- 2. 车辆 / 优化器配置 --------------------------
  const double vehicle_length =
      request.vehicle.wheelbase_m + request.vehicle.front_overhang_m +
      request.vehicle.rear_overhang_m;
  const double d_cr =
      (request.vehicle.front_overhang_m + request.vehicle.wheelbase_m -
       request.vehicle.rear_overhang_m) /
      2.0;
  const double hull_margin =
      options.hull_margin_m.value_or(request.vehicle.safety_margin_m);

  VehicleParam vehicle_param;
  vehicle_param.set_width(request.vehicle.width_m);
  vehicle_param.set_length(vehicle_length);
  vehicle_param.set_wheel_base(request.vehicle.wheelbase_m);
  vehicle_param.set_front_suspension(request.vehicle.front_overhang_m);
  vehicle_param.set_rear_suspension(request.vehicle.rear_overhang_m);
  vehicle_param.set_d_cr(d_cr);

  // 软惩罚收紧：上游优化器用平滑惩罚项施加动力学限制，因此收敛轨迹可能
  // 略微高于限制一个小残差。收缩内部限制可让输出落在请求的限制内
  // （由临时性检查核对）。当限制残差仍超出请求界限时，
  // optimize_trajectory 会以更大的 `tightening` 重试。
  const double tight = 1.0 - tuning.tightening;

  // 走廊约束采样分辨率与传给移植核心的配置必须一致（上游按
  // `(pieces-2)*(traj_resolution+1) + 2*(destraj_resolution+1)` 校验约束点
  // 数量）。重试阶梯的覆盖值只作下限（只升不降）。
  const int traj_resolution = std::max(tuning.traj_resolution, options.traj_resolution);
  const int destraj_resolution =
      std::max(tuning.destraj_resolution, options.destraj_resolution);

  OptimizerConfig config;
  config.traj_resolution = traj_resolution;
  config.destraj_resolution = destraj_resolution;
  config.wei_sta_obs = options.static_obstacle_weight * tuning.corridor_weight_scale;
  config.wei_feas = options.feasibility_weight * tuning.feasibility_weight_scale;
  config.wei_time = options.time_weight;
  config.wei_anchor = options.anchor_weight * tuning.anchor_weight_scale;
  // 优化器外壳在验证足迹之外按 `geometry_tightening_m` 膨胀：让膨胀后的
  // 外壳保持在走廊内，就为平滑惩罚残差留出了同样多的余量，使（含余量的）
  // 验证足迹在 validation 的 1e-6 m 容差内仍位于走廊内。
  // 重试阶梯会在各次尝试中逐步加大它。
  config.half_margin =
      std::max(hull_margin, request.vehicle.safety_margin_m) + tuning.geometry_tightening_m;
  config.max_forward_vel = limits.max_forward_velocity_mps * tight;
  config.max_backward_vel = limits.max_backward_velocity_mps * tight;
  config.max_forward_acc = limits.max_forward_acceleration_mps2 * tight;
  config.max_backward_acc = limits.max_backward_acceleration_mps2 * tight;
  // 曲率收紧得少得多：预制 Hybrid A* 路径常常用满转向范围
  // （kappa == 运动学极限），5 % 的收紧会让路径不可行，惩罚将持续存在。
  // 1 % 的余量对典型残差能让平衡点落在请求限制内，同时对最小半径圆弧
  // 保持可行。碰撞驱动的定向重试可略微加大它（仍只为内部预算）。
  const double curvature_tight = 1.0 - tuning.curvature_tightening;
  config.max_forward_cur = limits.max_forward_curvature_inv_m * curvature_tight;
  config.max_backward_cur = limits.max_backward_curvature_inv_m * curvature_tight;
  config.max_latacc = limits.max_lateral_acceleration_mps2 * tight;
  config.max_phidot = limits.max_steering_rate_radps;
  config.non_sinv = options.direction_switch_velocity_mps;
  config.gearopt = true;
  config.lbfgs_memsize = options.lbfgs_memsize;
  config.lbfgs_past = options.lbfgs_past;
  config.lbfgs_delta = options.lbfgs_delta;
  config.mini_t = options.min_piece_duration_s;
  config.lbfgs_max_iterations = options.lbfgs_max_iterations;
  // 重试时走廊权重可能加大；让失败阈值随之缩放，
  // 以保持门槛的相对含义不变。
  config.cost_fail_threshold = options.cost_fail_threshold * tuning.corridor_weight_scale;

  // ---------- 3. 初值与走廊约束 -------------------
  std::vector<Eigen::MatrixXd> head_states;
  std::vector<Eigen::MatrixXd> tail_states;
  std::vector<Eigen::MatrixXd> inner_points;
  std::vector<double> total_durations;
  std::vector<std::vector<Eigen::MatrixXd>> corridor_constraints;
  std::vector<int> singuls;
  std::vector<int> piece_counts;
  std::vector<Trajectory> initial_trajectories;

  const std::size_t run_count = runs.size();
  head_states.reserve(run_count);
  tail_states.reserve(run_count);
  inner_points.reserve(run_count);
  total_durations.reserve(run_count);
  corridor_constraints.reserve(run_count);
  singuls.reserve(run_count);
  piece_counts.reserve(run_count);

  std::size_t constraint_point_count = 0;
  for (std::size_t run_index = 0; run_index < run_count; ++run_index) {
    const DirectionRun& run = runs[run_index];
    const double run_length = run.s_end_m - run.s_begin_m;
    if (run_length <= 1e-6) {
      return fail(common::Status::kInvalidInput, "invalid_seed",
                  "direction run " + std::to_string(run_index) + " has zero length");
    }
    const int singul = run.singul;
    const double max_velocity = singul > 0 ? limits.max_forward_velocity_mps
                                           : limits.max_backward_velocity_mps;
    const double max_acceleration = singul > 0 ? limits.max_forward_acceleration_mps2
                                               : limits.max_backward_acceleration_mps2;

    // 初值：沿运行段的静止到静止梯形速度曲线，保持在动力学限制的 90 %，
    // 使 MINCO 初值从可行域内出发（过于激进的初值会让惩罚求解器
    // 在自由的横向方向上漂走）。
    const double v_cruise = 0.9 * max_velocity;
    const double a_cruise = 0.9 * max_acceleration;
    double t_acc = 0.0;
    double t_total = 0.0;
    double distance_at_acc = 0.0;
    if (run_length >= v_cruise * v_cruise / a_cruise) {
      t_acc = v_cruise / a_cruise;
      distance_at_acc = 0.5 * v_cruise * t_acc;
      t_total = 2.0 * t_acc + (run_length - 2.0 * distance_at_acc) / v_cruise;
    } else {
      t_acc = std::sqrt(run_length / a_cruise);
      distance_at_acc = 0.5 * run_length;
      t_total = 2.0 * t_acc;
    }
    t_total = std::max(t_total, 2.0 * options.min_piece_duration_s);
    int pieces = static_cast<int>(std::lround(t_total / tuning.piece_duration_s));
    pieces = std::max(pieces, 2);
    double dt = t_total / pieces;
    if (dt < options.min_piece_duration_s) {
      pieces = std::max(2, static_cast<int>(std::ceil(t_total / options.min_piece_duration_s)));
      dt = t_total / pieces;
    }
    const double duration_total = dt * pieces;

    // 梯形速度曲线下时刻 t 已行驶的弧长。
    auto arc_length_at_time = [&](double t) {
      t = std::min(std::max(t, 0.0), t_total);
      if (t <= t_acc) {
        return 0.5 * a_cruise * t * t;
      }
      if (t <= t_total - t_acc) {
        return distance_at_acc + v_cruise * (t - t_acc);
      }
      const double t_decel = t_total - t;
      return run_length - 0.5 * a_cruise * t_decel * t_decel;
    };

    Eigen::MatrixXd head(2, 3);
    Eigen::MatrixXd tail(2, 3);
    head.col(0) = Eigen::Vector2d(run.begin.x_m, run.begin.y_m);
    tail.col(0) = Eigen::Vector2d(run.end.x_m, run.end.y_m);
    head.col(1).setZero();
    head.col(2).setZero();
    tail.col(1).setZero();
    tail.col(2).setZero();

    Eigen::MatrixXd inner(2, pieces - 1);
    for (int i = 1; i < pieces; ++i) {
      const double s =
          run.s_begin_m + arc_length_at_time(static_cast<double>(i) * dt);
      const Point2d waypoint = point_at_arc_length(line, s);
      inner(0, i - 1) = waypoint.x_m;
      inner(1, i - 1) = waypoint.y_m;
    }

    MinJerkOpt min_jerk;
    min_jerk.reset(pieces);
    min_jerk.generate(inner, dt, head, tail);
    const Trajectory initial_trajectory = min_jerk.getTraj(singul);
    initial_trajectories.push_back(initial_trajectory);
    // 本运行段每个约束点处的走廊半平面，与移植代价函数使用的采样时刻一致。
    std::vector<Eigen::MatrixXd> run_constraints;
    run_constraints.reserve(static_cast<std::size_t>(
        (pieces - 2) * (traj_resolution + 1) +
        2 * (destraj_resolution + 1)));
    for (int piece = 0; piece < pieces; ++piece) {
      const int resolution =
          (piece == 0 || piece == pieces - 1) ? destraj_resolution : traj_resolution;
      for (int j = 0; j <= resolution; ++j) {
        const double t_local =
            (static_cast<double>(piece) + static_cast<double>(j) /
                                              static_cast<double>(resolution)) *
            dt;
        const Eigen::Vector2d position = initial_trajectory.getPos(t_local);
        const double s = project_arc_length(line, Point2d{position(0), position(1)});
        const int polygon_index =
            polygon_index_for_s(request.corridor, s, 1e-6);
        if (polygon_index < 0) {
          return fail(common::Status::kInvalidInput, "corridor_coverage",
                      "no corridor polygon covers seed arc length " +
                          format_double(s) + " (run " + std::to_string(run_index) + ")");
        }
        run_constraints.push_back(polygon_halfplanes(
            request.corridor.polygons[static_cast<std::size_t>(polygon_index)].vertices,
            options.corridor_tightening_m));
      }
    }
    constraint_point_count += run_constraints.size();

    head_states.push_back(head);
    tail_states.push_back(tail);
    inner_points.push_back(inner);
    total_durations.push_back(duration_total);
    corridor_constraints.push_back(std::move(run_constraints));
    singuls.push_back(singul);
    piece_counts.push_back(pieces);
  }

  Eigen::VectorXd init_ts(static_cast<Eigen::Index>(total_durations.size()));
  for (std::size_t i = 0; i < total_durations.size(); ++i) {
    init_ts(static_cast<Eigen::Index>(i)) = total_durations[i];
  }
  result.stats.total_pieces = 0;
  for (int pieces : piece_counts) {
    result.stats.total_pieces += pieces;
  }
  result.stats.corridor_constraint_points = constraint_point_count;

  // ---------- 4. 运行移植的优化器 -----------------------------------
  PolyTrajOptimizer optimizer;
  optimizer.setVehicleParam(vehicle_param);
  optimizer.setParam(config);
  // 协作式取消（移植扩展）：把适配层的谓词桥接进移植核心的代价/进度回调。
  // `request` 在整个调用期间存活，因此上下文指针有效。
  if (options.cancel_check) {
    optimizer.setCancelCheck(
        &ported_cancel_check,
        const_cast<std::function<bool()>*>(&options.cancel_check));
  }

  std::vector<Eigen::MatrixXd> inner_points_mutable = inner_points;
  const auto solve_start = std::chrono::steady_clock::now();
  const bool success =
      optimizer.OptimizeTrajectory(head_states, tail_states, inner_points_mutable,
                                   init_ts, corridor_constraints, singuls, 0.0, 1.0e-4);
  const auto solve_end = std::chrono::steady_clock::now();

  result.stats.solve_time_s = std::chrono::duration<double>(solve_end - solve_start).count();
  result.stats.lbfgs_iterations = optimizer.getIterations();
  result.stats.lbfgs_result = optimizer.getLbfgsResult();
  result.stats.final_cost = optimizer.getFinalCost();

  // 取消优先于任何求解结果：绝不消费部分迭代结果（移植核心已在迭代边界
  // 观察到取消时 wasCancelled() 为真；这里也覆盖“取消在求解刚返回后到达”
  // 的情况）。
  if (optimizer.wasCancelled() || cancel_requested(options)) {
    return fail(common::Status::kCancelled, "cancelled",
                "cooperative cancellation observed during the L-BFGS solve after " +
                    std::to_string(optimizer.getIterations()) + " cost evaluations");
  }

  const int lbfgs_result = optimizer.getLbfgsResult();
  const bool acceptable_solver_code =
      lbfgs_result == lbfgs::LBFGS_CONVERGENCE || lbfgs_result == lbfgs::LBFGS_CANCELED ||
      lbfgs_result == lbfgs::LBFGS_STOP || lbfgs_result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
      lbfgs_result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH;
  if (!success) {
    const std::string reason = acceptable_solver_code ? "cost_too_high" : "solver_failed";
    return fail(common::Status::kOptimizationFailed, reason,
                "lbfgs result=" + std::to_string(lbfgs_result) +
                    ", final cost=" + format_double(optimizer.getFinalCost()));
  }

  // ---------- 5. 采样优化后的分段 Bezier ----------------------
  const std::vector<MinJerkOpt>* min_jerk_optima = optimizer.getMinJerkOptPtr();
  std::vector<Trajectory> optimized_trajectories;
  std::vector<double> optimized_durations;
  double total_time = 0.0;
  for (std::size_t run_index = 0; run_index < run_count; ++run_index) {
    const MinJerkOpt& min_jerk = (*min_jerk_optima)[run_index];
    Trajectory trajectory = min_jerk.getTraj(singuls[run_index]);
    const int pieces = piece_counts[run_index];
    const double dt = min_jerk.getDt();
    optimized_trajectories.push_back(trajectory);
    optimized_durations.push_back(dt * pieces);
    total_time += dt * pieces;
  }
  result.stats.trajectory_duration_s = total_time;

  // 对走廊包含残差做确定性的投影修复
  // （Wave 7，dftpav-002）；最终判定仍属 planner::validation。
  if (options.corridor_repair) {
    const CorridorRepairResult repair = repair_corridor_containment(
        request, optimized_trajectories, initial_trajectories, singuls, -1e-4);
    optimized_trajectories = repair.trajectories;
    result.stats.corridor_repair_iterations = repair.iterations;
    result.stats.corridor_repair_worst_violation_m = repair.worst_violation;
  }
  // 取消（移植扩展）：修复过程中到达的取消同样丢弃全部中间结果。
  if (optimizer.wasCancelled() || cancel_requested(options)) {
    return fail(common::Status::kCancelled, "cancelled",
                "cooperative cancellation observed after the L-BFGS solve; "
                "the post-solve repair result is discarded");
  }

  const double sample_dt = options.sample_dt_s;
  std::vector<TrajectoryPoint> points;
  std::vector<int> point_run;
  double previous_yaw = 0.0;
  bool have_previous_yaw = false;

  // 静止边界航向来自预制路径：零速度时多项式不定义航向，优化器的端点
  // 状态就是初值的端点状态（validation 会把最终航向与目标比对）。
  const common::Pose2d seed_start_pose = request.seed.front().points.front().pose;
  const common::Pose2d seed_end_pose = request.seed.back().points.back().pose;

  std::vector<double> direction_switch_times;
  double run_start_time = 0.0;
  for (std::size_t run_index = 0; run_index < run_count; ++run_index) {
    const Trajectory& trajectory = optimized_trajectories[run_index];
    const double run_duration = optimized_durations[run_index];
    if (run_index > 0) {
      direction_switch_times.push_back(run_start_time);
    }
    std::size_t sample_index = 0;
    while (true) {
      double t_local = static_cast<double>(sample_index) * sample_dt;
      if (t_local >= run_duration) {
        t_local = run_duration;  // 精确的末端采样点
      }
      const double t_global = run_start_time + t_local;
      sample_index += 1;
      if (!points.empty() && t_global <= points.back().t_s) {
        if (t_local >= run_duration) {
          break;
        }
        continue;
      }
      const bool last_sample = (run_index + 1 == run_count) && t_local >= run_duration;
      TrajectoryPoint point;
      point.t_s = t_global;
      const Eigen::Vector2d position = trajectory.getPos(t_local);
      point.pose.x_m = position(0);
      point.pose.y_m = position(1);
      const double velocity = trajectory.getVel(t_local);
      point.velocity_mps = velocity;
      point.acceleration_mps2 = trajectory.getAcc(t_local);
      // 速度奇异尺度以下，MINCO 路径曲率是无界的静止假象（见
      // OptimizeOptions）；此处报告 0。
      point.curvature_inv_m =
          std::abs(velocity) >= options.curvature_report_min_speed_mps
              ? trajectory.getCurv(t_local)
              : 0.0;
      // 超过极小航向尺度时信任解析航向；低于它时（真正的静止，位置导数
      // 可能翻转）沿用携带的/初值航向。此处若使用更大的曲率奇异尺度，
      // 会在穿越点引入 yaw 跳变，planner::validation 会把它读成两个存储
      // 位姿之间的极小半径圆弧（破坏其弧长关联与中间包含性位姿）。
      if (std::abs(velocity) > options.heading_report_min_speed_mps) {
        point.pose.yaw_rad = common::normalize_angle_rad(trajectory.getAngle(t_local));
        previous_yaw = point.pose.yaw_rad;
        have_previous_yaw = true;
      } else if (last_sample) {
        point.pose.yaw_rad = seed_end_pose.yaw_rad;
        previous_yaw = point.pose.yaw_rad;
        have_previous_yaw = true;
      } else if (have_previous_yaw) {
        point.pose.yaw_rad = previous_yaw;
      } else {
        point.pose.yaw_rad = seed_start_pose.yaw_rad;
        previous_yaw = point.pose.yaw_rad;
        have_previous_yaw = true;
      }
      points.push_back(point);
      point_run.push_back(static_cast<int>(run_index));
      if (t_local >= run_duration) {
        break;
      }
    }
    run_start_time += run_duration;
  }

  result.trajectory = points;
  result.stats.trajectory_points = points.size();
  result.stats.direction_switch_times_s = direction_switch_times;

  // ---------- 6. 验证门槛（权威） ----------------------------
  // planner::validation 独立于本优化器，施加第 7.1 节固定的硬容差；
  // 它的判定决定轨迹是否会被作为有效结果返回。适配器从不放宽它。
  {
    const common::Pose2d validation_goal =
        request.goal.has_value()
            ? *request.goal
            : common::Pose2d{request.seed.back().points.back().pose.x_m,
                             request.seed.back().points.back().pose.y_m,
                             request.seed.back().points.back().pose.yaw_rad};
    validation::ValidationRequest validation_request;
    validation_request.trajectory = points;
    validation_request.vehicle = request.vehicle;
    // validation 每个轴只携带一个限制；下面的适配器临时性检查
    // 还会精确施加分方向的限制。
    validation_request.limits.max_speed_mps =
        std::max(limits.max_forward_velocity_mps, limits.max_backward_velocity_mps);
    validation_request.limits.max_acceleration_mps2 =
        std::max(limits.max_forward_acceleration_mps2, limits.max_backward_acceleration_mps2);
    validation_request.limits.max_deceleration_mps2 =
        std::max(limits.max_forward_acceleration_mps2, limits.max_backward_acceleration_mps2);
    validation_request.limits.max_curvature_inv_m =
        std::max(limits.max_forward_curvature_inv_m, limits.max_backward_curvature_inv_m);
    validation_request.goal = validation_goal;
    validation_request.scene = request.scene.value_or(collision::Scene{});
    validation_request.corridor = request.corridor;
    result.validation = validation::validate_trajectory(validation_request);
    if (!result.validation.valid) {
      result.status = common::Status::kValidationFailed;
      result.reason = "validation_" + result.validation.reason;
      result.detail = result.validation.detail;
      return result;
    }
  }

  // ---------- 7. 临时性可行性检查（诊断、方向感知） -
  auto limit_tolerance = [&options](double limit) {
    return options.constraint_tolerance * std::max(1.0, std::abs(limit));
  };

  auto check_fail = [&](const std::string& reason, const std::string& message) {
    result.status = common::Status::kValidationFailed;
    result.reason = reason;
    result.detail = message;
    return result;
  };

  // 无论下面临时性检查的结果如何，都会记录统计最大值。
  {
    double max_speed = 0.0;
    double max_acceleration = 0.0;
    double max_curvature = 0.0;
    double max_lateral_acceleration = 0.0;
    for (const TrajectoryPoint& point : points) {
      const double speed = std::abs(point.velocity_mps);
      const double acceleration = std::abs(point.acceleration_mps2);
      const double curvature = std::abs(point.curvature_inv_m);
      max_speed = std::max(max_speed, speed);
      max_acceleration = std::max(max_acceleration, acceleration);
      max_curvature = std::max(max_curvature, curvature);
      max_lateral_acceleration =
          std::max(max_lateral_acceleration, speed * speed * curvature);
    }
    result.stats.max_speed_mps = max_speed;
    result.stats.max_acceleration_mps2 = max_acceleration;
    result.stats.max_curvature_inv_m = max_curvature;
    result.stats.max_lateral_acceleration_mps2 = max_lateral_acceleration;
  }

  if (options.provisional_checks) {
    if (points.empty()) {
      return check_fail("provisional_non_finite", "sampler produced no trajectory point");
    }
    for (std::size_t i = 0; i < points.size(); ++i) {
      const TrajectoryPoint& point = points[i];
      if (!(finite(point.t_s) && finite(point.pose.x_m) && finite(point.pose.y_m) &&
            finite(point.pose.yaw_rad) && finite(point.velocity_mps) &&
            finite(point.acceleration_mps2) && finite(point.curvature_inv_m))) {
        return check_fail("provisional_non_finite",
                          "non-finite trajectory value at index " + std::to_string(i));
      }
      if (i > 0 && !(point.t_s > points[i - 1].t_s)) {
        return check_fail("provisional_time_order",
                          "time is not strictly increasing at index " + std::to_string(i));
      }
      const int singul = singuls[static_cast<std::size_t>(point_run[i])];
      const double max_velocity =
          singul > 0 ? limits.max_forward_velocity_mps : limits.max_backward_velocity_mps;
      const double max_acceleration_limit = singul > 0
                                                ? limits.max_forward_acceleration_mps2
                                                : limits.max_backward_acceleration_mps2;
      const double max_curvature_limit = singul > 0
                                             ? limits.max_forward_curvature_inv_m
                                             : limits.max_backward_curvature_inv_m;
      const double speed = std::abs(point.velocity_mps);
      const double acceleration = std::abs(point.acceleration_mps2);
      const double curvature = std::abs(point.curvature_inv_m);
      const double lateral_acceleration = point.velocity_mps * point.velocity_mps * curvature;
      if (speed > max_velocity + limit_tolerance(max_velocity)) {
        return check_fail("provisional_velocity_limit",
                          "|v|=" + format_double(speed) + " exceeds " +
                              format_double(max_velocity) + " at t=" +
                              format_double(point.t_s) + "s");
      }
      if (acceleration > max_acceleration_limit + limit_tolerance(max_acceleration_limit)) {
        return check_fail("provisional_acceleration_limit",
                          "|a|=" + format_double(acceleration) + " exceeds " +
                              format_double(max_acceleration_limit) + " at t=" +
                              format_double(point.t_s) + "s");
      }
      if (curvature > max_curvature_limit + limit_tolerance(max_curvature_limit)) {
        return check_fail("provisional_curvature_limit",
                          "|k|=" + format_double(curvature) + " exceeds " +
                              format_double(max_curvature_limit) + " at t=" +
                              format_double(point.t_s) + "s");
      }
      if (lateral_acceleration >
          limits.max_lateral_acceleration_mps2 +
              limit_tolerance(limits.max_lateral_acceleration_mps2)) {
        return check_fail("provisional_lateral_acceleration_limit",
                          "|v^2*k|=" + format_double(lateral_acceleration) + " exceeds " +
                              format_double(limits.max_lateral_acceleration_mps2) +
                              " at t=" + format_double(point.t_s) + "s");
      }
    }
    // 换向必须发生在（近乎）静止时：|v| <= 1e-3 m/s，
    // 除非调用方显式配置了更大的连接点速度。
    const double switch_speed_limit =
        std::max(1e-3, options.direction_switch_velocity_mps);
    for (double switch_time : direction_switch_times) {
      auto it = std::lower_bound(
          points.begin(), points.end(), switch_time,
          [](const TrajectoryPoint& point, double t) { return point.t_s < t; });
      if (it == points.end() || std::abs(it->t_s - switch_time) > 1e-9) {
        return check_fail("provisional_time_order",
                          "no sample at direction-change time " +
                              format_double(switch_time) + "s");
      }
      if (std::abs(it->velocity_mps) > switch_speed_limit) {
        return check_fail("provisional_direction_change_stop",
                          "|v|=" + format_double(std::abs(it->velocity_mps)) +
                              " at direction change t=" + format_double(switch_time) + "s");
      }
    }
  }

  // 走廊包含性由 planner::validation 检查（权威、保守足迹、固定 1e-6 m
  // 容差、按轨迹自身弧长关联）；这里曾有的第二个基于投影的检查与之矛盾
  // （关联方式不同），因此不作为否决项。临时性检查只保留 validation 的
  // 单限制接口无法表达的方向感知动力学检查。

  result.status = common::Status::kSuccess;
  result.reason.clear();
  result.detail.clear();
  return result;
}

}  // namespace

OptimizeResult optimize_trajectory(const OptimizeRequest& request) {
  OptimizeResult result;
  const OptimizeOptions& options = request.options;
  const OptimizeLimits& limits = request.limits;

  auto fail = [&result](common::Status status, const std::string& reason,
                        const std::string& detail) -> OptimizeResult {
    result.status = status;
    result.reason = reason;
    result.detail = detail;
    return result;
  };

  // ---------- 1. 输入校验 ------------------------------------------
  std::string detail;
  if (!validate_vehicle(request.vehicle, &detail)) {
    return fail(common::Status::kInvalidInput, "invalid_vehicle", detail);
  }
  if (!validate_limits(limits, &detail)) {
    return fail(common::Status::kInvalidInput, "invalid_limits", detail);
  }
  if (!validate_options(options, &detail)) {
    return fail(common::Status::kInvalidInput, "invalid_options", detail);
  }
  if (!request.map_id.empty() && !request.corridor.map_id.empty() &&
      request.map_id != request.corridor.map_id) {
    return fail(common::Status::kInvalidInput, "map_mismatch",
                "request.map_id != corridor.map_id");
  }
  if (!request.map_version.empty() && !request.corridor.map_version.empty() &&
      request.map_version != request.corridor.map_version) {
    return fail(common::Status::kInvalidInput, "map_mismatch",
                "request.map_version != corridor.map_version");
  }

  // 取消（移植扩展）：输入校验之后、任何求解工作之前轮询一次；取消只改变
  // 返回状态，绝不放宽 V/碰撞门禁。
  if (cancel_requested(options)) {
    return fail(common::Status::kCancelled, "cancelled",
                "cooperative cancellation requested before the solve");
  }

  SeedLine line;
  std::vector<DirectionRun> runs;
  if (!build_seed_line(request.seed, &line, &runs, &detail)) {
    return fail(common::Status::kInvalidInput, "invalid_seed", detail);
  }

  if (request.corridor.polygons.empty()) {
    return fail(common::Status::kInvalidInput, "empty_corridor",
                "corridor contains no polygon");
  }
  if (!request.corridor.polygons.empty()) {
    double previous_start = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < request.corridor.polygons.size(); ++i) {
      const common::CorridorPolygon& polygon = request.corridor.polygons[i];
      if (!(finite(polygon.s_start_m) && finite(polygon.s_end_m)) ||
          !(polygon.s_end_m > polygon.s_start_m)) {
        return fail(common::Status::kInvalidInput, "invalid_corridor",
                    "polygon " + std::to_string(i) + " has an empty or non-finite s interval");
      }
      if (polygon.s_start_m < previous_start - 1e-9) {
        return fail(common::Status::kInvalidInput, "invalid_corridor",
                    "polygon s intervals are not ordered");
      }
      previous_start = polygon.s_start_m;
      if (polygon.vertices.size() < 3) {
        return fail(common::Status::kInvalidInput, "invalid_corridor",
                    "polygon " + std::to_string(i) + " has fewer than 3 vertices");
      }
      for (const Point2d& vertex : polygon.vertices) {
        if (!finite(vertex.x_m) || !finite(vertex.y_m)) {
          return fail(common::Status::kInvalidInput, "invalid_corridor",
                      "polygon " + std::to_string(i) + " contains non-finite vertices");
        }
      }
      if (!is_strictly_convex_ccw(polygon.vertices)) {
        return fail(common::Status::kInvalidInput, "invalid_corridor",
                    "polygon " + std::to_string(i) + " is not strictly convex CCW");
      }
    }
    const double coverage_tolerance = 1e-6;
    if (request.corridor.polygons.front().s_start_m > coverage_tolerance ||
        request.corridor.polygons.back().s_end_m < line.total_length_m - coverage_tolerance) {
      return fail(common::Status::kInvalidInput, "corridor_coverage",
                  "corridor does not cover the seed arc length [0, " +
                      format_double(line.total_length_m) + "]");
    }
  }

  const int direction_changes =
      runs.empty() ? 0 : static_cast<int>(runs.size()) - 1;
  result.stats.direction_changes = direction_changes;

  // 面向平滑惩罚优化器的有界确定性重试阶梯。基础尝试保持初值分段时长，
  // 只逐级加大几何收紧（全程 5 % 的限制收紧）。最后一次尝试还会把分段
  // 时长减半：更密的初值能足够贴近紧绷的预制路径，使节点空间恢复收敛
  // （粗的 1 s 网格会留下分米级的切角，任何求解后修复都无法还原）。
  // 精细尝试放在最后，这样用粗网格已经通过的用例保持原有轨迹。
  // 只有限制/走廊残差类原因才触发重试，碰撞、端点、时间或有限性失败
  // 永不触发；validation 本身从不被放宽。
  constexpr int kMaxTighteningAttempts = 4;
  constexpr double kFinePieceDurationFactor = 0.5;
  auto is_retryable_reason = [](const std::string& reason) {
    return reason == "provisional_velocity_limit" ||
           reason == "provisional_acceleration_limit" ||
           reason == "provisional_curvature_limit" ||
           reason == "provisional_lateral_acceleration_limit" ||
           reason == "provisional_corridor_containment" ||
           reason == "validation_speed_limit" ||
           reason == "validation_acceleration_limit" ||
           reason == "validation_deceleration_limit" ||
           reason == "validation_curvature_limit" ||
           reason == "validation_corridor_containment";
  };

  // 所有尝试都失败时，用严重度保留最接近有效的尝试：包含性失败按测量到的
  // 越界距离排序（越小越好），其他失败保留最早的尝试。
  auto attempt_severity = [](const OptimizeResult& attempt) {
    if (attempt.ok()) {
      return -1.0;
    }
    if (attempt.reason == "validation_corridor_containment" ||
        attempt.reason == "provisional_corridor_containment") {
      const double outside = attempt.validation.valid
                                 ? 0.0
                                 : attempt.validation.first_failure.actual;
      return std::isfinite(outside) ? std::max(0.0, outside) : 1e12;
    }
    return 1e12;
  };

  const double base_piece_duration = request.options.traj_piece_duration_s;
  const double base_curvature_tightening = request.options.internal_curvature_tightening;
  AttemptTuning base_tuning;
  base_tuning.piece_duration_s = base_piece_duration;
  base_tuning.curvature_tightening = base_curvature_tightening;
  result = run_attempt(request, line, runs, base_tuning);
  const OptimizeResult first_result = result;
  OptimizeResult best_result = result;
  double best_severity = attempt_severity(result);
  auto describe_attempt = [&](const OptimizeResult& attempt) {
    std::string text = attempt.ok() ? "success" : attempt.reason;
    if (!attempt.ok() && !attempt.validation.valid &&
        attempt.validation.first_failure.constraint ==
            validation::Constraint::kCorridorContainment) {
      text += "(" + format_double(attempt.validation.first_failure.actual) + "m)";
    }
    double length = 0.0;
    for (std::size_t i = 1; i < attempt.trajectory.size(); ++i) {
      length += std::hypot(attempt.trajectory[i].pose.x_m - attempt.trajectory[i - 1].pose.x_m,
                           attempt.trajectory[i].pose.y_m - attempt.trajectory[i - 1].pose.y_m);
    }
    text += "[len=" + format_double(length) + "]";
    return text;
  };
  std::vector<std::string> attempt_reasons;
  attempt_reasons.push_back(describe_attempt(result));
  int attempts = 1;
  while (!result.ok() && attempts < kMaxTighteningAttempts &&
         is_retryable_reason(result.reason)) {
    // 取消（移植扩展）：不再启动新的收紧尝试（重试阶梯绝不覆盖取消）。
    if (cancel_requested(options)) {
      break;
    }
    // 只有几何收紧会逐级加大：它把优化器的活跃约束残差 r 转化为验证足迹
    // 余量（margin = geometry_tightening - r），而其他手段会让优化器停留在
    // 同一局部盆地。曾尝试加大走廊权重或把初值分段时长减半，结果残差
    // 反而更差（记录在组件报告中）。
    // 只有几何收紧会逐级加大：其他手段固定时优化器停留在同一局部盆地，
    // 验证足迹余量恰好按新增的收紧量增长（margin = delta - residual）。
    // 曾尝试加大走廊权重、分辨率或把初值分段时长减半，得到不同且通常
    // 更差的局部极小值（记录在组件报告中）。
    const bool fine_attempt = attempts == kMaxTighteningAttempts - 1;
    const double piece_duration =
        fine_attempt ? kFinePieceDurationFactor * base_piece_duration : base_piece_duration;
    const double geometry_tightening =
        fine_attempt ? 0.03 : 0.01 + 0.02 * attempts;
    AttemptTuning tuning;
    tuning.piece_duration_s = piece_duration;
    tuning.geometry_tightening_m = geometry_tightening;
    tuning.curvature_tightening = base_curvature_tightening;
    result = run_attempt(request, line, runs, tuning);
    attempt_reasons.push_back(describe_attempt(result));
    const double severity = attempt_severity(result);
    if (severity < best_severity) {
      best_severity = severity;
      best_result = result;
    }
    ++attempts;
  }
  result = best_result;

  // 碰撞驱动的定向重试（hybrid-006）。根因：种子合法但贴墙（保守足迹间隙
  // 0.0006–0.0828 m）时，优化轨迹相对初值的横向/航向偏离（1.2–5.2 cm/至
  // 6.3°）会让重建运动扫掠穿透墙体，planner::validation 以
  // `validation_collision` 正确拒绝。这里仅当首次尝试即碰撞失败时，按有界、
  // 确定性的阶梯收紧“跟随种子”的杠杆：
  //   1. 更细分段（初值网格加密到 0.5x/0.25x），锚定项能逐点贴紧贴墙段；
  //   2. 更强的初值锚定（2x/4x），压低相对初值的横向偏离；
  //   3. 必要时加大内部曲率收紧（1%→2%），把曲率软约束残差压回 V 的原始
  //      限制内（仍只是内部预算，V 的硬限制从不改变）。
  // 每次尝试都走完整 V 判定；全部失败时保留原失败结果（原因/详情/轨迹/
  // 统计），绝不返回部分结果，也绝不放宽任何门禁。
  if (!result.ok() && request.options.collision_retry &&
      first_result.reason == "validation_collision") {
    struct CollisionRetrySpec {
      double piece_factor;             // 初值分段时长倍率（相对基础值）
      double anchor_scale;             // 初值锚定权重倍率
      double curvature_tightening;     // 内部曲率收紧（绝对值，>= 基础值）
      double feasibility_weight_scale; // 可行性惩罚权重倍率
      int traj_resolution;             // 走廊采样分辨率下限（0 = 调用方默认）
      int destraj_resolution;          // 同上（首/末分段）
    };
    // 候选按“改动从小到大”排序，前一个失败才进入下一个；命中即停止。
    // 每个候选都由实测确定（证据见 README §6 hybrid-006 修复记录），
    // 对三类贴墙种子的分工：
    //   * zm0612 deadend_122：0.5x 网格 + 2x 锚定；
    //   * smac deadend_125：0.25x 网格 + 2% 内部曲率收紧；
    //   * zm0612 deadend_125：0.125x 网格 + 4x 可行性权重 + 2x 约束采样。
    const CollisionRetrySpec collision_retries[] = {
        {0.5, 2.0, base_curvature_tightening, 1.0, 0, 0},
        {0.25, 1.0, std::max(base_curvature_tightening, 0.02), 1.0, 0, 0},
        {0.125, 1.0, base_curvature_tightening, 4.0, 32, 64},
    };
    for (const CollisionRetrySpec& retry : collision_retries) {
      // 取消（移植扩展）：不再启动新的碰撞定向尝试。
      if (cancel_requested(options)) {
        break;
      }
      AttemptTuning tuning;
      tuning.piece_duration_s = base_piece_duration * retry.piece_factor;
      tuning.anchor_weight_scale = retry.anchor_scale;
      tuning.curvature_tightening = retry.curvature_tightening;
      tuning.feasibility_weight_scale = retry.feasibility_weight_scale;
      tuning.traj_resolution = retry.traj_resolution;
      tuning.destraj_resolution = retry.destraj_resolution;
      OptimizeResult attempt = run_attempt(request, line, runs, tuning);
      attempt_reasons.push_back(describe_attempt(attempt));
      ++attempts;
      if (attempt.ok()) {
        result = std::move(attempt);
        break;
      }
    }
  }

  result.stats.tightening_attempts = attempts;
  result.stats.attempt_reasons = attempt_reasons;
  // 取消（移植扩展）：取消若在最后一次尝试返回后才被观察到（例如边沿触发的
  // 谓词），丢弃该尝试的结果，返回稳定的非成功状态且不带轨迹。
  if (cancel_requested(options) && result.status != common::Status::kCancelled) {
    result.status = common::Status::kCancelled;
    result.reason = "cancelled";
    result.detail = "cooperative cancellation requested; the attempt result is discarded";
    result.trajectory.clear();
    result.validation = validation::ValidationResult{};
  }
  return result;
}

}  // namespace planner::dftpav
