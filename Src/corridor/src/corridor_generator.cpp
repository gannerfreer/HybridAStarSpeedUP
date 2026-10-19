// 安全走廊生成与独立几何验证。

#include "planner/corridor/corridor_generator.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "planner/corridor/geometry.hpp"

namespace planner::corridor {
namespace {

using common::Corridor;
using common::CorridorPolygon;
using common::Direction;
using common::PathSegment;
using common::Point2d;
using common::Pose2d;
using common::Status;
using common::VehicleModel;

// 路径点严格递增与锚点去重的弧长容差（米）。
constexpr double kArcLengthEps = 1e-9;
// 种子多边形与障碍物距离小于该值时视为受阻；该值只是浮点保护量，
// 不是间隙要求。
constexpr double kMinFreeGap = 1e-6;
// 覆盖/连续性检查的弧长容差（米）。
constexpr double kCoverageTol = 1e-6;
// 在换向尖点处，两个锚点位姿距离小于该值时视为同一点（米/弧度）。
constexpr double kPoseTol = 1e-6;
// 足迹包含容差的数值保护量。
constexpr double kMinContainmentTol = 1e-9;

bool pose_is_finite(const Pose2d& pose) {
  return std::isfinite(pose.x_m) && std::isfinite(pose.y_m) && std::isfinite(pose.yaw_rad);
}

void set_detail(std::string* detail, std::string text) {
  if (detail != nullptr) {
    *detail = std::move(text);
  }
}

std::string named_id(const std::string& id) {
  return id.empty() ? std::string("<unnamed>") : id;
}

std::string format_metres(double value) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << value << " m";
  return stream.str();
}

// ---------------------------------------------------------------------------
// 路径处理
// ---------------------------------------------------------------------------

struct Anchor {
  double s{0.0};
  Pose2d pose;
};

struct PathData {
  std::vector<Anchor> anchors;
  std::size_t seed_points{0};
  double s_first{0.0};
  double s_last{0.0};
};

// 展平并校验种子分段。失败时 `detail` 接收稳定的、人类可读的诊断信息。
bool build_path_data(const std::vector<PathSegment>& seed, PathData* out, std::string* detail) {
  out->anchors.clear();
  out->seed_points = 0;
  if (seed.empty()) {
    set_detail(detail, "seed must contain at least one path segment");
    return false;
  }
  for (const PathSegment& segment : seed) {
    if (segment.points.empty()) {
      set_detail(detail, "seed path segment must contain at least one point");
      return false;
    }
    const Direction direction = segment.points.front().direction;
    bool have_previous = false;
    double previous_s = 0.0;
    for (const common::PathPoint& point : segment.points) {
      ++out->seed_points;
      if (!pose_is_finite(point.pose) || !std::isfinite(point.s_m) ||
          !std::isfinite(point.curvature_inv_m)) {
        set_detail(detail, "seed path points must be finite");
        return false;
      }
      if (point.s_m < 0.0) {
        set_detail(detail, "seed arc length must be non-negative");
        return false;
      }
      if (point.direction != direction) {
        set_detail(detail, "every seed segment must have one direction");
        return false;
      }
      if (have_previous && point.s_m <= previous_s + kArcLengthEps) {
        set_detail(detail, "seed arc length must be strictly increasing inside a segment");
        return false;
      }
      have_previous = true;
      previous_s = point.s_m;
      if (!out->anchors.empty()) {
        const Anchor& back = out->anchors.back();
        if (point.s_m <= back.s + kArcLengthEps) {
          const double position_error =
              distance(Point2d{point.pose.x_m, point.pose.y_m},
                       Point2d{back.pose.x_m, back.pose.y_m});
          const double yaw_error = std::fabs(
              common::normalize_angle_rad(point.pose.yaw_rad - back.pose.yaw_rad));
          if (position_error <= kPoseTol && yaw_error <= kPoseTol) {
            continue;  // 重复的换向点；保留第一个
          }
          set_detail(detail, "seed arc length must be non-decreasing between segments");
          return false;
        }
      }
      out->anchors.push_back(Anchor{point.s_m, point.pose});
    }
  }
  if (out->anchors.size() < 2) {
    set_detail(detail, "seed path is degenerate: at least two distinct points are required");
    return false;
  }
  out->s_first = out->anchors.front().s;
  out->s_last = out->anchors.back().s;
  if (out->s_last - out->s_first <= kArcLengthEps) {
    set_detail(detail, "seed path length must be positive");
    return false;
  }
  return true;
}

// 种子折线上弧长 `s` 处的位姿：位置在锚点间线性插值，yaw 沿最短弧插值，
// 与验证器和路径消费方使用的插值一致。
Pose2d interpolate_pose(const std::vector<Anchor>& anchors, double s) {
  if (s <= anchors.front().s) {
    return anchors.front().pose;
  }
  if (s >= anchors.back().s) {
    return anchors.back().pose;
  }
  const auto it = std::upper_bound(
      anchors.begin(), anchors.end(), s,
      [](double value, const Anchor& anchor) { return value < anchor.s; });
  const Anchor& a = *(it - 1);
  const Anchor& b = *it;
  const double span = b.s - a.s;
  const double t = span > 0.0 ? (s - a.s) / span : 0.0;
  Pose2d pose;
  pose.x_m = a.pose.x_m + (b.pose.x_m - a.pose.x_m) * t;
  pose.y_m = a.pose.y_m + (b.pose.y_m - a.pose.y_m) * t;
  const double delta_yaw =
      common::normalize_angle_rad(b.pose.yaw_rad - a.pose.yaw_rad);
  pose.yaw_rad = common::normalize_angle_rad(a.pose.yaw_rad + delta_yaw * t);
  return pose;
}

// 弧长采样：每个长度不超过 `step` 的区间采一个点，两端点都精确包含在内，
// 并去除重复。
std::vector<double> resample_arc_length(const PathData& path, double step) {
  std::vector<double> samples;
  samples.push_back(path.s_first);
  for (std::size_t i = 0; i + 1 < path.anchors.size(); ++i) {
    const double s0 = path.anchors[i].s;
    const double s1 = path.anchors[i + 1].s;
    const double span = s1 - s0;
    if (!(span > 0.0)) {
      continue;
    }
    std::size_t substeps = static_cast<std::size_t>(std::ceil(span / step));
    if (substeps < 1) {
      substeps = 1;
    }
    for (std::size_t k = 1; k <= substeps; ++k) {
      const double s = (k == substeps)
                           ? s1
                           : s0 + span * (static_cast<double>(k) / static_cast<double>(substeps));
      if (s > samples.back() + kArcLengthEps) {
        samples.push_back(s);
      }
    }
  }
  if (path.s_last > samples.back() + kArcLengthEps) {
    samples.push_back(path.s_last);
  }
  return samples;
}

// ---------------------------------------------------------------------------
// 车辆足迹与输入校验
// ---------------------------------------------------------------------------

// 包围整车的保守矩形：每条边都外扩 `safety_margin_m`，因此走廊横向必须
// 包含车辆宽度加两倍裕度。
struct Footprint {
  double rear_m{0.0};
  double front_m{0.0};
  double half_width_m{0.0};
  double corner_radius_m{0.0};
  std::array<Point2d, 4> corners_body;
};

Footprint make_footprint(const VehicleModel& vehicle) {
  const double margin = vehicle.safety_margin_m;
  Footprint footprint;
  footprint.rear_m = vehicle.rear_overhang_m + margin;
  footprint.front_m = vehicle.wheelbase_m + vehicle.front_overhang_m + margin;
  footprint.half_width_m = 0.5 * vehicle.width_m + margin;
  footprint.corners_body = {
      Point2d{footprint.front_m, -footprint.half_width_m},
      Point2d{footprint.front_m, footprint.half_width_m},
      Point2d{-footprint.rear_m, footprint.half_width_m},
      Point2d{-footprint.rear_m, -footprint.half_width_m}};
  double radius_squared = 0.0;
  for (const Point2d& corner : footprint.corners_body) {
    radius_squared = std::max(radius_squared, dot(corner, corner));
  }
  footprint.corner_radius_m = std::sqrt(radius_squared);
  return footprint;
}

std::array<Point2d, 4> footprint_corners(const Pose2d& pose, const Footprint& footprint) {
  const double cosine = std::cos(pose.yaw_rad);
  const double sine = std::sin(pose.yaw_rad);
  std::array<Point2d, 4> corners;
  for (std::size_t i = 0; i < footprint.corners_body.size(); ++i) {
    const Point2d& body = footprint.corners_body[i];
    corners[i] = Point2d{pose.x_m + cosine * body.x_m - sine * body.y_m,
                         pose.y_m + sine * body.x_m + cosine * body.y_m};
  }
  return corners;
}

std::vector<Point2d> corners_to_vector(const std::array<Point2d, 4>& corners) {
  return std::vector<Point2d>(corners.begin(), corners.end());
}

bool valid_vehicle(const VehicleModel& vehicle, std::string* detail) {
  if (!std::isfinite(vehicle.wheelbase_m) || !(vehicle.wheelbase_m > 0.0)) {
    set_detail(detail, "invalid vehicle: wheelbase_m must be finite and positive");
    return false;
  }
  if (!std::isfinite(vehicle.width_m) || !(vehicle.width_m > 0.0)) {
    set_detail(detail, "invalid vehicle: width_m must be finite and positive");
    return false;
  }
  if (!std::isfinite(vehicle.front_overhang_m) || vehicle.front_overhang_m < 0.0) {
    set_detail(detail, "invalid vehicle: front_overhang_m must be finite and non-negative");
    return false;
  }
  if (!std::isfinite(vehicle.rear_overhang_m) || vehicle.rear_overhang_m < 0.0) {
    set_detail(detail, "invalid vehicle: rear_overhang_m must be finite and non-negative");
    return false;
  }
  if (!std::isfinite(vehicle.max_steer_rad) || !(vehicle.max_steer_rad > 0.0) ||
      !(vehicle.max_steer_rad < common::kPi / 2.0)) {
    set_detail(detail, "invalid vehicle: max_steer_rad must be finite and inside (0, pi/2)");
    return false;
  }
  if (!std::isfinite(vehicle.safety_margin_m) || vehicle.safety_margin_m < 0.0) {
    set_detail(detail, "invalid vehicle: safety_margin_m must be finite and non-negative");
    return false;
  }
  return true;
}

bool valid_options(const CorridorOptions& options, std::string* detail) {
  if (!std::isfinite(options.sample_step_m) || !(options.sample_step_m > 0.0)) {
    set_detail(detail, "invalid options: sample_step_m must be finite and positive");
    return false;
  }
  if (!std::isfinite(options.construction_step_m) || !(options.construction_step_m > 0.0)) {
    set_detail(detail, "invalid options: construction_step_m must be finite and positive");
    return false;
  }
  if (!std::isfinite(options.verification_step_m) || !(options.verification_step_m > 0.0)) {
    set_detail(detail, "invalid options: verification_step_m must be finite and positive");
    return false;
  }
  if (!std::isfinite(options.clearance_m) || options.clearance_m < 0.0) {
    set_detail(detail, "invalid options: clearance_m must be finite and non-negative");
    return false;
  }
  if (!std::isfinite(options.max_expansion_m) || options.max_expansion_m < 0.0) {
    set_detail(detail, "invalid options: max_expansion_m must be finite and non-negative");
    return false;
  }
  if (!std::isfinite(options.perturbation_yaw_rad) || options.perturbation_yaw_rad < 0.0) {
    set_detail(detail, "invalid options: perturbation_yaw_rad must be finite and non-negative");
    return false;
  }
  if (!std::isfinite(options.perturbation_position_m) ||
      options.perturbation_position_m < 0.0) {
    set_detail(detail,
               "invalid options: perturbation_position_m must be finite and non-negative");
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// 障碍物
// ---------------------------------------------------------------------------

struct ObstacleData {
  std::string id;
  std::vector<Point2d> vertices;  // 凸、逆时针
};

bool normalize_obstacle(const Obstacle& obstacle, ObstacleData* out, std::string* detail) {
  std::vector<Point2d> vertices;
  vertices.reserve(obstacle.vertices.size());
  for (const Point2d& vertex : obstacle.vertices) {
    if (!is_finite(vertex)) {
      set_detail(detail, "obstacle '" + named_id(obstacle.id) + "': vertices must be finite");
      return false;
    }
    if (!vertices.empty() && distance(vertices.back(), vertex) <= 1e-12) {
      continue;
    }
    vertices.push_back(vertex);
  }
  if (vertices.size() >= 2 && distance(vertices.front(), vertices.back()) <= 1e-12) {
    vertices.pop_back();
  }
  if (vertices.size() < 3) {
    set_detail(detail, "obstacle '" + named_id(obstacle.id) +
                           "': at least three distinct vertices are required");
    return false;
  }
  const double area = polygon_signed_area(vertices);
  if (!(std::fabs(area) > kGeometryEps)) {
    set_detail(detail, "obstacle '" + named_id(obstacle.id) + "': polygon area must be positive");
    return false;
  }
  if (area < 0.0) {
    std::reverse(vertices.begin(), vertices.end());
  }
  const std::size_t count = vertices.size();
  for (std::size_t i = 0; i < count; ++i) {
    const Point2d& a = vertices[(i + count - 1) % count];
    const Point2d& b = vertices[i];
    const Point2d& c = vertices[(i + 1) % count];
    const Point2d e1 = subtract(b, a);
    const Point2d e2 = subtract(c, b);
    const double scale = norm(e1) * norm(e2);
    if (!(scale > 1e-18)) {
      set_detail(detail, "obstacle '" + named_id(obstacle.id) + "': duplicate vertices");
      return false;
    }
    if (cross(e1, e2) < -kGeometryEps * std::max(1.0, scale)) {
      set_detail(detail, "obstacle '" + named_id(obstacle.id) +
                             "': polygon must be convex (vertices counter-clockwise)");
      return false;
    }
  }
  out->id = obstacle.id;
  out->vertices = std::move(vertices);
  return true;
}

struct BlockedClassification {
  CorridorFailure failure{CorridorFailure::kBlocked};
  std::string detail;
};

// 在 `blocked`（单侧或路径上受阻）与 `narrow`（两侧障碍物之间的自由
// 间距小于车辆宽度加两倍安全裕度）之间判定。
BlockedClassification classify_blocked(const std::vector<Point2d>& seed_polygon,
                                       const Pose2d& pose, const Footprint& footprint,
                                       const std::vector<ObstacleData>& obstacles,
                                       double s_m) {
  BlockedClassification result;
  const Point2d position{pose.x_m, pose.y_m};
  for (const ObstacleData& obstacle : obstacles) {
    if (point_in_convex_polygon_ccw(position, obstacle.vertices, 1e-6)) {
      result.failure = CorridorFailure::kBlocked;
      result.detail = "the seed path enters obstacle '" + named_id(obstacle.id) + "' near " +
                      format_metres(s_m);
      return result;
    }
  }
  const Point2d tangent{std::cos(pose.yaw_rad), std::sin(pose.yaw_rad)};
  const Point2d left{-tangent.y_m, tangent.x_m};
  bool left_side = false;
  bool right_side = false;
  double left_clearance = std::numeric_limits<double>::infinity();
  double right_clearance = std::numeric_limits<double>::infinity();
  for (const ObstacleData& obstacle : obstacles) {
    if (convex_polygons_gap(seed_polygon, obstacle.vertices) > kMinFreeGap) {
      continue;
    }
    double obstacle_left = std::numeric_limits<double>::infinity();
    double obstacle_right = std::numeric_limits<double>::infinity();
    for (const Point2d& vertex : obstacle.vertices) {
      const double lateral = dot(left, subtract(vertex, position));
      if (lateral > kGeometryEps) {
        obstacle_left = std::min(obstacle_left, lateral);
      } else if (lateral < -kGeometryEps) {
        obstacle_right = std::min(obstacle_right, -lateral);
      }
    }
    const bool reaches_left = std::isfinite(obstacle_left);
    const bool reaches_right = std::isfinite(obstacle_right);
    if (reaches_left && !reaches_right) {
      left_side = true;
      left_clearance = std::min(left_clearance, obstacle_left);
    } else if (reaches_right && !reaches_left) {
      right_side = true;
      right_clearance = std::min(right_clearance, obstacle_right);
    }
  }
  const double required_width = 2.0 * footprint.half_width_m;
  if (left_side && right_side) {
    const double free_width = left_clearance + right_clearance;
    if (free_width < required_width - kGeometryEps) {
      result.failure = CorridorFailure::kNarrow;
      result.detail = "lateral free width " + format_metres(free_width) + " near " +
                      format_metres(s_m) + " is below the required " +
                      format_metres(required_width) +
                      " (vehicle width + 2 x safety margin)";
      return result;
    }
  }
  result.failure = CorridorFailure::kBlocked;
  result.detail = "the convex footprint sweep near " + format_metres(s_m) +
                  " cannot avoid the surrounding obstacles";
  return result;
}

// ---------------------------------------------------------------------------
// 生成与验证共用的输入
// ---------------------------------------------------------------------------

CorridorResult make_failure(Status status, CorridorFailure failure, std::string reason,
                            std::string detail) {
  CorridorResult result;
  result.status = status;
  result.failure = failure;
  result.reason = std::move(reason);
  result.detail = std::move(detail);
  return result;
}

CorridorVerification make_verification_failure(std::string reason, std::string detail,
                                               std::size_t polygon = 0, double s_m = 0.0) {
  CorridorVerification verification;
  verification.valid = false;
  verification.reason = std::move(reason);
  verification.detail = std::move(detail);
  verification.failed_polygon = polygon;
  verification.s_m = s_m;
  return verification;
}

}  // namespace

// ---------------------------------------------------------------------------
// 公开工厂函数与失败名称
// ---------------------------------------------------------------------------

Obstacle make_rectangle_obstacle(std::string id, double x_min_m, double y_min_m,
                                 double x_max_m, double y_max_m) {
  Obstacle obstacle;
  obstacle.id = std::move(id);
  obstacle.kind = ObstacleKind::kRectangle;
  obstacle.vertices = {Point2d{x_min_m, y_min_m}, Point2d{x_max_m, y_min_m},
                       Point2d{x_max_m, y_max_m}, Point2d{x_min_m, y_max_m}};
  return obstacle;
}

Obstacle make_convex_obstacle(std::string id, std::vector<Point2d> vertices) {
  Obstacle obstacle;
  obstacle.id = std::move(id);
  obstacle.kind = ObstacleKind::kConvexPolygon;
  obstacle.vertices = std::move(vertices);
  return obstacle;
}

const char* to_string(CorridorFailure failure) noexcept {
  switch (failure) {
    case CorridorFailure::kNone:
      return "none";
    case CorridorFailure::kInvalidInput:
      return "invalid_input";
    case CorridorFailure::kBlocked:
      return "blocked";
    case CorridorFailure::kNarrow:
      return "narrow";
    case CorridorFailure::kNoCorridor:
      return "no_corridor";
  }
  return "unknown";
}

// ---------------------------------------------------------------------------
// 生成
// ---------------------------------------------------------------------------

CorridorResult generate_corridor(const CorridorRequest& request) {
  if (request.map_id.empty()) {
    return make_failure(Status::kInvalidInput, CorridorFailure::kInvalidInput, "invalid_input",
                        "map_id must not be empty");
  }
  if (request.map_version.empty()) {
    return make_failure(Status::kInvalidInput, CorridorFailure::kInvalidInput, "invalid_input",
                        "map_version must not be empty");
  }
  if (request.frame_id.empty()) {
    return make_failure(Status::kInvalidInput, CorridorFailure::kInvalidInput, "invalid_input",
                        "frame_id must not be empty");
  }
  std::string detail;
  if (!valid_vehicle(request.vehicle, &detail)) {
    return make_failure(Status::kInvalidInput, CorridorFailure::kInvalidInput, "invalid_input",
                        detail);
  }
  if (!valid_options(request.options, &detail)) {
    return make_failure(Status::kInvalidInput, CorridorFailure::kInvalidInput, "invalid_input",
                        detail);
  }
  std::vector<ObstacleData> obstacles;
  obstacles.reserve(request.obstacles.size());
  for (const Obstacle& obstacle : request.obstacles) {
    ObstacleData data;
    if (!normalize_obstacle(obstacle, &data, &detail)) {
      return make_failure(Status::kInvalidInput, CorridorFailure::kInvalidInput, "invalid_input",
                          detail);
    }
    obstacles.push_back(std::move(data));
  }
  PathData path;
  if (!build_path_data(request.seed, &path, &detail)) {
    return make_failure(Status::kInvalidInput, CorridorFailure::kInvalidInput, "invalid_input",
                        detail);
  }

  const Footprint footprint = make_footprint(request.vehicle);
  // 保守鲁棒性包络（corridor-002）：允许位姿扰动下足迹角点的最坏位移为
  // `corner_radius * 2 sin(delta_yaw / 2) + delta_position`；一个小因子
  // 用于防止构造受浮点边界效应影响。
  const double robustness_margin_m =
      (footprint.corner_radius_m * 2.0 *
           std::sin(0.5 * request.options.perturbation_yaw_rad) +
       request.options.perturbation_position_m) *
          1.01 +
      1e-12;
  const std::vector<double> sample_s =
      resample_arc_length(path, request.options.sample_step_m);
  if (sample_s.size() < 2) {
    return make_failure(Status::kNoPath, CorridorFailure::kNoCorridor, "no_corridor",
                        "seed resampling produced no positive interval");
  }

  std::vector<CorridorPolygon> polygons;
  polygons.reserve(sample_s.size());
  CorridorStats stats;
  stats.seed_points = path.seed_points;
  stats.path_length_m = path.s_last - path.s_first;
  stats.robustness_margin_m = robustness_margin_m;

  for (std::size_t i = 0; i + 1 < sample_s.size(); ++i) {
    const double s0 = sample_s[i];
    const double s1 = sample_s[i + 1];
    const double span = s1 - s0;
    if (!(span > 1e-12)) {
      continue;
    }

    // 该区间的保守足迹扫掠。
    std::size_t substeps =
        static_cast<std::size_t>(std::ceil(span / request.options.construction_step_m));
    if (substeps < 1) {
      substeps = 1;
    }
    std::vector<Pose2d> poses;
    poses.reserve(substeps + 1);
    for (std::size_t k = 0; k <= substeps; ++k) {
      const double s = (k == substeps)
                           ? s1
                           : s0 + span * (static_cast<double>(k) / static_cast<double>(substeps));
      poses.push_back(interpolate_pose(path.anchors, s));
    }
    stats.construction_poses += poses.size();

    std::vector<Point2d> hull_points;
    hull_points.reserve(poses.size() * 4);
    for (const Pose2d& pose : poses) {
      for (const Point2d& corner : footprint_corners(pose, footprint)) {
        hull_points.push_back(corner);
      }
    }
    std::vector<Point2d> hull = convex_hull(std::move(hull_points));
    if (hull.size() < 3) {
      return make_failure(Status::kNoPath, CorridorFailure::kNoCorridor, "no_corridor",
                          "degenerate footprint sweep hull near " + format_metres(0.5 * (s0 + s1)));
    }

    // 相邻构造位姿之间足迹偏差的严格上界：yaw 变化 `delta` 时，中间足迹
    // 包含在弦凸包按 r * (1 - cos(delta / 2)) 膨胀后的区域内。鲁棒性包络
    // 还覆盖任意种子位姿的配置扰动（corridor-002）。
    double slack_m = 0.0;
    for (std::size_t k = 0; k + 1 < poses.size(); ++k) {
      const double delta_yaw = std::fabs(common::normalize_angle_rad(
          poses[k + 1].yaw_rad - poses[k].yaw_rad));
      slack_m = std::max(slack_m, footprint.corner_radius_m * (1.0 - std::cos(0.5 * delta_yaw)));
    }
    slack_m = slack_m * 1.05 + robustness_margin_m;
    std::vector<Point2d> seed_polygon;
    if (!offset_convex_polygon(hull, slack_m, &seed_polygon)) {
      return make_failure(Status::kNoPath, CorridorFailure::kNoCorridor, "no_corridor",
                          "cannot build a convex seed polygon near " +
                              format_metres(0.5 * (s0 + s1)));
    }

    // 种子多边形必须严格避开所有障碍物；否则没有凸走廊能覆盖该区间。
    double seed_gap = std::numeric_limits<double>::infinity();
    for (const ObstacleData& obstacle : obstacles) {
      seed_gap = std::min(seed_gap, convex_polygons_gap(seed_polygon, obstacle.vertices));
    }
    if (seed_gap <= kMinFreeGap) {
      const double s_mid = 0.5 * (s0 + s1);
      const BlockedClassification classification = classify_blocked(
          seed_polygon, interpolate_pose(path.anchors, s_mid), footprint, obstacles, s_mid);
      const char* reason = to_string(classification.failure);
      return make_failure(Status::kNoPath, classification.failure, reason,
                          classification.detail);
    }

    const double clearance_m = std::isfinite(seed_gap)
                                   ? std::min(request.options.clearance_m, 0.5 * seed_gap)
                                   : request.options.clearance_m;

    // 逐边扩张种子多边形：每条边按精确的多边形-障碍物分离量推进到极限，
    // 同时其余各边保持当前偏移（逐坐标最大化，重复若干轮）。这取代了
    // 全局半平面支撑上限——后者会让路径远处的障碍物沿法向收紧整条边，
    // 把绕行走廊压缩到足迹宽度（corridor-001）。每个被接受的状态都恰好
    // 满足所请求间隙且无障碍，因此不放松任何安全裕度。
    const std::vector<HalfPlane> planes = polygon_half_planes(seed_polygon);
    if (planes.size() < 3) {
      return make_failure(Status::kNoPath, CorridorFailure::kNoCorridor, "no_corridor",
                          "degenerate seed polygon near " + format_metres(0.5 * (s0 + s1)));
    }

    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const Point2d& vertex : seed_polygon) {
      min_x = std::min(min_x, vertex.x_m);
      min_y = std::min(min_y, vertex.y_m);
      max_x = std::max(max_x, vertex.x_m);
      max_y = std::max(max_y, vertex.y_m);
    }
    // 裁剪窗口只是半平面求交的实现上界，绝不是几何约束：它足够大，使两条
    // 扩张边形成的顶点不会被截断。
    const double inflation = 4.0 * request.options.max_expansion_m + 1.0;
    const std::vector<Point2d> bounds{
        Point2d{min_x - inflation, min_y - inflation},
        Point2d{max_x + inflation, min_y - inflation},
        Point2d{max_x + inflation, max_y + inflation},
        Point2d{min_x - inflation, max_y + inflation}};

    auto build_candidate = [&](const std::vector<double>& edge_expansions,
                               std::vector<Point2d>* out) {
      std::vector<HalfPlane> scaled = planes;
      for (std::size_t k = 0; k < scaled.size(); ++k) {
        scaled[k].offset += edge_expansions[k];
      }
      return intersect_half_planes(scaled, bounds, out);
    };
    auto acceptable = [&](const std::vector<Point2d>& candidate) {
      if (candidate.size() < 3) {
        return false;
      }
      for (const ObstacleData& obstacle : obstacles) {
        if (convex_polygons_gap(candidate, obstacle.vertices) <
            clearance_m - kGeometryEps) {
          return false;
        }
      }
      return true;
    };

    std::vector<double> expansions(planes.size(), 0.0);
    std::vector<Point2d> candidate = seed_polygon;
    constexpr int kExpansionSweeps = 4;
    for (int sweep = 0; sweep < kExpansionSweeps; ++sweep) {
      bool changed = false;
      for (std::size_t k = 0; k < planes.size(); ++k) {
        if (expansions[k] >= request.options.max_expansion_m - kGeometryEps) {
          continue;
        }
        std::vector<double> trial_expansions = expansions;
        std::vector<Point2d> trial;
        trial_expansions[k] = request.options.max_expansion_m;
        if (build_candidate(trial_expansions, &trial) && acceptable(trial)) {
          expansions = std::move(trial_expansions);
          candidate = std::move(trial);
          changed = true;
          continue;
        }
        double good = expansions[k];
        double bad = request.options.max_expansion_m;
        for (int iteration = 0; iteration < 40; ++iteration) {
          const double middle = 0.5 * (good + bad);
          trial_expansions = expansions;
          trial_expansions[k] = middle;
          if (build_candidate(trial_expansions, &trial) && acceptable(trial)) {
            good = middle;
          } else {
            bad = middle;
          }
        }
        if (good > expansions[k] + kGeometryEps) {
          expansions[k] = good;
          if (!build_candidate(expansions, &candidate)) {
            return make_failure(Status::kNoPath, CorridorFailure::kNoCorridor, "no_corridor",
                                "corridor expansion failed near " +
                                    format_metres(0.5 * (s0 + s1)));
          }
          changed = true;
        }
      }
      if (!changed) {
        break;
      }
    }
    if (!acceptable(candidate)) {
      const double s_mid = 0.5 * (s0 + s1);
      return make_failure(Status::kNoPath, CorridorFailure::kBlocked, "blocked",
                          "corridor expansion is blocked near " + format_metres(s_mid));
    }
    for (std::size_t k = 0; k < expansions.size(); ++k) {
      if (expansions[k] < request.options.max_expansion_m - kGeometryEps) {
        ++stats.obstacle_limited_edges;
      }
    }

    CorridorPolygon polygon;
    polygon.s_start_m = s0;
    polygon.s_end_m = s1;
    polygon.vertices = std::move(candidate);
    polygons.push_back(std::move(polygon));
  }

  if (polygons.empty()) {
    return make_failure(Status::kNoPath, CorridorFailure::kNoCorridor, "no_corridor",
                        "seed produced no corridor polygons");
  }

  Corridor corridor;
  corridor.id = request.request_id.empty() ? "corridor" : request.request_id + ":corridor";
  corridor.map_id = request.map_id;
  corridor.map_version = request.map_version;
  corridor.frame_id = request.frame_id;
  corridor.polygons = std::move(polygons);

  stats.polygon_count = corridor.polygons.size();
  stats.min_polygon_width_m = std::numeric_limits<double>::infinity();
  for (const CorridorPolygon& polygon : corridor.polygons) {
    stats.min_polygon_width_m =
        std::min(stats.min_polygon_width_m, convex_polygon_min_width(polygon.vertices));
  }
  stats.min_obstacle_clearance_m = std::numeric_limits<double>::infinity();
  for (const CorridorPolygon& polygon : corridor.polygons) {
    for (const ObstacleData& obstacle : obstacles) {
      stats.min_obstacle_clearance_m =
          std::min(stats.min_obstacle_clearance_m,
                   convex_polygons_gap(polygon.vertices, obstacle.vertices));
    }
  }

  CorridorVerifyOptions verify_options;
  verify_options.sample_step_m = request.options.verification_step_m;
  // 对公开契约的常规自检；鲁棒性包络是构造保证（凸包按最坏情况角点位移
  // 膨胀，corridor-002），由使用独立几何的扰动回归测试覆盖。
  const CorridorVerification verification =
      verify_corridor(corridor, request.seed, request.obstacles, request.vehicle, verify_options);
  if (!verification.valid) {
    return make_failure(Status::kNoPath, CorridorFailure::kNoCorridor, "no_corridor",
                        "generated corridor failed verification: " + verification.detail);
  }

  CorridorResult result;
  result.status = Status::kSuccess;
  result.failure = CorridorFailure::kNone;
  result.corridor = std::move(corridor);
  result.stats = stats;
  return result;
}

// ---------------------------------------------------------------------------
// 验证
// ---------------------------------------------------------------------------

CorridorVerification verify_corridor(const Corridor& corridor,
                                     const std::vector<PathSegment>& seed,
                                     const std::vector<Obstacle>& obstacles,
                                     const VehicleModel& vehicle,
                                     const CorridorVerifyOptions& options) {
  if (!std::isfinite(options.sample_step_m) || !(options.sample_step_m > 0.0)) {
    return make_verification_failure("invalid_options",
                                     "verification sample_step_m must be finite and positive");
  }
  if (!std::isfinite(options.containment_tolerance_m) ||
      options.containment_tolerance_m < kMinContainmentTol) {
    return make_verification_failure(
        "invalid_options",
        "verification containment_tolerance_m must be finite and at least 1e-9");
  }
  std::string detail;
  if (!valid_vehicle(vehicle, &detail)) {
    return make_verification_failure("invalid_input", detail);
  }
  PathData path;
  if (!build_path_data(seed, &path, &detail)) {
    return make_verification_failure("invalid_input", detail);
  }
  std::vector<ObstacleData> normalized_obstacles;
  normalized_obstacles.reserve(obstacles.size());
  for (const Obstacle& obstacle : obstacles) {
    ObstacleData data;
    if (!normalize_obstacle(obstacle, &data, &detail)) {
      return make_verification_failure("invalid_input", detail);
    }
    normalized_obstacles.push_back(std::move(data));
  }
  if (corridor.id.empty() || corridor.map_id.empty() || corridor.map_version.empty() ||
      corridor.frame_id.empty()) {
    return make_verification_failure(
        "structural", "corridor id, map_id, map_version and frame_id must be non-empty");
  }
  if (corridor.polygons.empty()) {
    return make_verification_failure("structural", "corridor has no polygons");
  }

  const Footprint footprint = make_footprint(vehicle);
  const double required_width = 2.0 * footprint.half_width_m;

  // 结构、排序与弧长区间。
  double previous_start = -std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < corridor.polygons.size(); ++i) {
    const CorridorPolygon& polygon = corridor.polygons[i];
    if (!is_convex_ccw_polygon(polygon.vertices)) {
      return make_verification_failure("polygon_not_convex_ccw",
                                       "polygon is not strictly convex counter-clockwise",
                                       i, polygon.s_start_m);
    }
    if (!(polygon.s_end_m > polygon.s_start_m)) {
      return make_verification_failure("empty_s_interval",
                                       "polygon s interval must be positive", i,
                                       polygon.s_start_m);
    }
    if (polygon.s_start_m < -kGeometryEps) {
      return make_verification_failure("negative_s_start",
                                       "polygon s_start_m must be non-negative", i,
                                       polygon.s_start_m);
    }
    if (polygon.s_start_m < previous_start - kCoverageTol) {
      return make_verification_failure("unordered_polygons",
                                       "polygon s_start_m is not non-decreasing", i,
                                       polygon.s_start_m);
    }
    previous_start = polygon.s_start_m;
  }

  // 无缺口地覆盖种子弧长。
  if (corridor.polygons.front().s_start_m > path.s_first + kCoverageTol) {
    return make_verification_failure("coverage",
                                     "corridor does not start at the first seed arc length", 0,
                                     path.s_first);
  }
  if (corridor.polygons.back().s_end_m < path.s_last - kCoverageTol) {
    return make_verification_failure("coverage",
                                     "corridor does not reach the last seed arc length",
                                     corridor.polygons.size() - 1, path.s_last);
  }
  for (std::size_t i = 1; i < corridor.polygons.size(); ++i) {
    if (corridor.polygons[i].s_start_m >
        corridor.polygons[i - 1].s_end_m + kCoverageTol) {
      return make_verification_failure("coverage_gap", "gap between consecutive polygon intervals",
                                       i, corridor.polygons[i].s_start_m);
    }
  }

  // 相邻多边形必须以正面积重叠。
  for (std::size_t i = 0; i + 1 < corridor.polygons.size(); ++i) {
    const double area = convex_polygon_intersection_area(corridor.polygons[i].vertices,
                                                         corridor.polygons[i + 1].vertices);
    if (!(area > 1e-9)) {
      return make_verification_failure("no_overlap",
                                       "adjacent polygons do not overlap with positive area", i,
                                       corridor.polygons[i].s_end_m);
    }
  }

  // 障碍物必须位于每个多边形之外，包括其顶点。
  for (std::size_t i = 0; i < corridor.polygons.size(); ++i) {
    const CorridorPolygon& polygon = corridor.polygons[i];
    for (const ObstacleData& obstacle : normalized_obstacles) {
      if (convex_polygons_gap(polygon.vertices, obstacle.vertices) <= kGeometryEps) {
        return make_verification_failure(
            "obstacle_intersection",
            "obstacle '" + named_id(obstacle.id) + "' touches or intersects a polygon", i,
            polygon.s_start_m);
      }
      for (const Point2d& vertex : obstacle.vertices) {
        bool strictly_inside = polygon.vertices.size() >= 3;
        for (std::size_t k = 0; k < polygon.vertices.size() && strictly_inside; ++k) {
          const Point2d& a = polygon.vertices[k];
          const Point2d& b = polygon.vertices[(k + 1) % polygon.vertices.size()];
          if (cross(subtract(b, a), subtract(vertex, a)) <= kGeometryEps) {
            strictly_inside = false;
          }
        }
        if (strictly_inside) {
          return make_verification_failure(
              "obstacle_vertex_inside",
              "vertex of obstacle '" + named_id(obstacle.id) + "' is inside a polygon", i,
              polygon.s_start_m);
        }
      }
    }
  }

  // 可通行性：走廊宽度对车辆宽度加安全裕度。
  for (std::size_t i = 0; i < corridor.polygons.size(); ++i) {
    const double width = convex_polygon_min_width(corridor.polygons[i].vertices);
    if (width < required_width - options.containment_tolerance_m) {
      std::ostringstream stream;
      stream << "minimal polygon width " << std::fixed << std::setprecision(2) << width
             << " m is below the required " << required_width
             << " m (vehicle width + 2 x safety margin)";
      return make_verification_failure("narrow", stream.str(), i,
                                       corridor.polygons[i].s_start_m);
    }
  }

  // 对稠密路径点集做整足迹包含检查。
  for (std::size_t anchor_index = 0; anchor_index + 1 < path.anchors.size(); ++anchor_index) {
    const double s_a = path.anchors[anchor_index].s;
    const double s_b = path.anchors[anchor_index + 1].s;
    const double span = s_b - s_a;
    std::size_t substeps = static_cast<std::size_t>(std::ceil(span / options.sample_step_m));
    if (substeps < 1) {
      substeps = 1;
    }
    for (std::size_t k = 0; k <= substeps; ++k) {
      const double s = (k == substeps)
                           ? s_b
                           : s_a + span * (static_cast<double>(k) / static_cast<double>(substeps));
      const Pose2d pose = interpolate_pose(path.anchors, s);
      const std::vector<Point2d> corners =
          corners_to_vector(footprint_corners(pose, footprint));
      bool contained = false;
      for (const CorridorPolygon& polygon : corridor.polygons) {
        if (s < polygon.s_start_m - kCoverageTol || s > polygon.s_end_m + kCoverageTol) {
          continue;
        }
        if (convex_polygon_contains(polygon.vertices, corners,
                                    options.containment_tolerance_m)) {
          contained = true;
          break;
        }
      }
      if (!contained) {
        std::ostringstream stream;
        stream << "whole-vehicle footprint at s = " << std::fixed << std::setprecision(2) << s
               << " m is not contained in a covering corridor polygon";
        return make_verification_failure("containment", stream.str(), 0, s);
      }
    }
  }

  CorridorVerification verification;
  verification.valid = true;
  return verification;
}

}  // namespace planner::corridor
