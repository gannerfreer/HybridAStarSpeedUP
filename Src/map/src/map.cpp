// planner_map 实现：基于 Lanelet2 的导入、坐标、查询与局部窗口裁剪。
//
// 本翻译单元是 Lanelet2 的边界：上游库只在这里使用（以及内部 sha256
// 辅助实现），没有 Lanelet2 类型跨越公开头文件。
// The upstream source is compiled by the private
// `planner_lanelet2` target; see CMakeLists.txt for the build adaptation.

#include "planner/map/map.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

// Lanelet2（仅限本翻译单元内部使用）。
#include <lanelet2_core/Attribute.h>
#include <lanelet2_core/Exceptions.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Area.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/primitives/Area.h>
#include <lanelet2_core/primitives/GPSPoint.h>
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_core/primitives/LineString.h>
#include <lanelet2_core/primitives/Point.h>
#include <lanelet2_core/primitives/RegulatoryElement.h>
#include <lanelet2_io/Exceptions.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_io/Projection.h>
#include <lanelet2_projection/Mercator.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_validation/Validation.h>

#include "sha256.hpp"

// 通用栅格助手（X-A1-17）：窗口尺寸/单元预算数学。
#include "planner/grid/grid.hpp"

namespace planner::map {
namespace {

constexpr std::size_t kMaxDiagnostics = 200;
constexpr double kPointEps = 1e-9;

bool finite(double value) { return std::isfinite(value); }

bool finite(const GeoPoint& p) {
  return finite(p.lat_deg) && finite(p.lon_deg) && finite(p.ele_m);
}

std::string lower_copy(std::string text) {
  std::transform(text.begin(), text.end(), text.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return text;
}

std::optional<lanelet::Id> parse_object_id(const ObjectId& id) {
  if (id.empty()) {
    return std::nullopt;
  }
  errno = 0;
  char* end = nullptr;
  const long long value = std::strtoll(id.c_str(), &end, 10);
  if (errno != 0 || end == id.c_str() || *end != '\0') {
    return std::nullopt;
  }
  return static_cast<lanelet::Id>(value);
}

std::vector<MapPoint> to_points(const lanelet::ConstLineString3d& line_string) {
  std::vector<MapPoint> out;
  out.reserve(line_string.size());
  for (const auto& point : line_string) {
    out.push_back(MapPoint{std::to_string(point.id()), point.x(), point.y(), point.z()});
  }
  return out;
}

nlohmann::json attributes_to_json(const lanelet::AttributeMap& attributes) {
  nlohmann::json out = nlohmann::json::object();
  for (const auto& entry : attributes) {
    out[entry.first] = entry.second.value();
  }
  return out;
}

// 拼接一个环的线串，去除重复的接合点。
std::vector<MapPoint> join_line_strings(const lanelet::ConstLineStrings3d& parts) {
  std::vector<MapPoint> out;
  for (const auto& part : parts) {
    for (auto& point : to_points(part)) {
      if (out.empty() || out.back().id != point.id || out.back().x_m != point.x_m ||
          out.back().y_m != point.y_m) {
        out.push_back(std::move(point));
      }
    }
  }
  return out;
}

bool same_point(const MapPoint& a, const MapPoint& b) {
  if (!a.id.empty() && !b.id.empty()) {
    return a.id == b.id;
  }
  return std::abs(a.x_m - b.x_m) <= kPointEps && std::abs(a.y_m - b.y_m) <= kPointEps;
}

MapPoint lerp_point(const MapPoint& a, const MapPoint& b, double t) {
  if (t <= 0.0) {
    return a;  // 保留原始 id
  }
  if (t >= 1.0) {
    return b;  // 保留原始 id
  }
  MapPoint out;
  out.x_m = a.x_m + (b.x_m - a.x_m) * t;
  out.y_m = a.y_m + (b.y_m - a.y_m) * t;
  out.z_m = a.z_m + (b.z_m - a.z_m) * t;
  return out;  // 合成的裁剪点：空 id
}

struct SegmentClip {
  MapPoint a;
  MapPoint b;
};

// 二维 Liang-Barsky 线段/矩形裁剪。线段与窗口不相交时返回 std::nullopt。
std::optional<SegmentClip> clip_segment(const MapPoint& p0, const MapPoint& p1,
                                        const Bounds2d& window) {
  double t0 = 0.0;
  double t1 = 1.0;
  const double dx = p1.x_m - p0.x_m;
  const double dy = p1.y_m - p0.y_m;
  const double p_values[4] = {-dx, dx, -dy, dy};
  const double q_values[4] = {p0.x_m - window.min_x_m, window.max_x_m - p0.x_m,
                              p0.y_m - window.min_y_m, window.max_y_m - p0.y_m};
  for (int i = 0; i < 4; ++i) {
    if (std::abs(p_values[i]) < 1e-15) {
      if (q_values[i] < 0.0) {
        return std::nullopt;
      }
      continue;
    }
    const double r = q_values[i] / p_values[i];
    if (p_values[i] < 0.0) {
      if (r > t1) {
        return std::nullopt;
      }
      if (r > t0) {
        t0 = r;
      }
    } else {
      if (r < t0) {
        return std::nullopt;
      }
      if (r < t1) {
        t1 = r;
      }
    }
  }
  if (t0 > t1) {
    return std::nullopt;
  }
  return SegmentClip{lerp_point(p0, p1, t0), lerp_point(p0, p1, t1)};
}

struct ClippedPiece {
  std::vector<MapPoint> points;
  bool start_clip_edge{false};
  bool end_clip_edge{false};
};

// 将一条折线裁剪为窗口内若干连续片段。当片段起点/终点与原始折线的
// 首点/末点不重合时，会将其标记为裁剪边。
std::vector<ClippedPiece> clip_polyline(const std::vector<MapPoint>& points,
                                        const Bounds2d& window) {
  std::vector<ClippedPiece> out;
  if (points.empty()) {
    return out;
  }
  if (points.size() == 1) {
    if (window.contains(points.front().x_m, points.front().y_m)) {
      out.push_back(ClippedPiece{points, false, false});
    }
    return out;
  }

  ClippedPiece current;
  bool active = false;
  const auto finish = [&]() {
    if (!active) {
      return;
    }
    if (!current.points.empty()) {
      current.start_clip_edge = !same_point(current.points.front(), points.front());
      current.end_clip_edge = !same_point(current.points.back(), points.back());
      out.push_back(std::move(current));
    }
    current = ClippedPiece{};
    active = false;
  };

  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    const auto clipped = clip_segment(points[i], points[i + 1], window);
    if (!clipped) {
      finish();
      continue;
    }
    if (!active || !same_point(current.points.back(), clipped->a)) {
      finish();
      current.points.push_back(clipped->a);
      active = true;
    }
    if (!same_point(current.points.back(), clipped->b)) {
      current.points.push_back(clipped->b);
    }
  }
  finish();
  return out;
}

struct BBox {
  double min_x{0.0};
  double min_y{0.0};
  double max_x{0.0};
  double max_y{0.0};
  bool valid{false};
};

BBox merge_bbox(const BBox& a, const BBox& b) {
  if (!a.valid) {
    return b;
  }
  if (!b.valid) {
    return a;
  }
  return BBox{std::min(a.min_x, b.min_x), std::min(a.min_y, b.min_y),
              std::max(a.max_x, b.max_x), std::max(a.max_y, b.max_y), true};
}

BBox bounds_of_points(const std::vector<MapPoint>& points) {
  BBox box;
  for (const auto& point : points) {
    if (!box.valid) {
      box = BBox{point.x_m, point.y_m, point.x_m, point.y_m, true};
      continue;
    }
    box.min_x = std::min(box.min_x, point.x_m);
    box.min_y = std::min(box.min_y, point.y_m);
    box.max_x = std::max(box.max_x, point.x_m);
    box.max_y = std::max(box.max_y, point.y_m);
  }
  return box;
}

ObjectSummary make_summary(const ObjectId& id, const nlohmann::json& attributes,
                           const BBox& box) {
  ObjectSummary out;
  out.id = id;
  out.attributes = attributes;
  if (box.valid) {
    out.bounds_m = Bounds2d{box.min_x, box.min_y, box.max_x, box.max_y};
    out.has_bounds = true;
  }
  return out;
}

ObjectSummary lanelet_summary(const lanelet::ConstLanelet& lanelet) {
  BBox box = bounds_of_points(to_points(lanelet.leftBound()));
  box = merge_bbox(box, bounds_of_points(to_points(lanelet.rightBound())));
  return make_summary(std::to_string(lanelet.id()), attributes_to_json(lanelet.attributes()),
                      box);
}

ObjectSummary area_summary(const lanelet::ConstArea& area) {
  BBox box = bounds_of_points(join_line_strings(area.outerBound()));
  for (const auto& ring : area.innerBounds()) {
    box = merge_bbox(box, bounds_of_points(join_line_strings(ring)));
  }
  return make_summary(std::to_string(area.id()), attributes_to_json(area.attributes()), box);
}

void sort_summaries(std::vector<ObjectSummary>& summaries) {
  std::sort(summaries.begin(), summaries.end(),
            [](const ObjectSummary& lhs, const ObjectSummary& rhs) {
              const auto a = parse_object_id(lhs.id);
              const auto b = parse_object_id(rhs.id);
              if (a && b && *a != *b) {
                return *a < *b;
              }
              return lhs.id < rhs.id;
            });
}

void sort_object_ids(std::vector<ObjectId>& ids) {
  std::sort(ids.begin(), ids.end(), [](const ObjectId& lhs, const ObjectId& rhs) {
    const auto a = parse_object_id(lhs);
    const auto b = parse_object_id(rhs);
    if (a && b && *a != *b) {
      return *a < *b;
    }
    return lhs < rhs;
  });
}

void sort_points(std::vector<MapPoint>& points) {
  std::sort(points.begin(), points.end(), [](const MapPoint& lhs, const MapPoint& rhs) {
    const auto a = parse_object_id(lhs.id);
    const auto b = parse_object_id(rhs.id);
    if (a && b && *a != *b) {
      return *a < *b;
    }
    return lhs.id < rhs.id;
  });
}

std::string join_reason(const std::string& prefix, const std::exception& error) {
  std::string text = prefix;
  if (!text.empty()) {
    text += ": ";
  }
  text += error.what();
  return text;
}

// lanelet2 各图层的 R-tree 搜索窗口（边界框重叠语义，
// 与公开的 "in bounds" 查询一致）。
lanelet::BoundingBox2d search_box(const Bounds2d& window) {
  return lanelet::BoundingBox2d{lanelet::BasicPoint2d{window.min_x_m, window.min_y_m},
                                lanelet::BasicPoint2d{window.max_x_m, window.max_y_m}};
}

// ---------------------------------------------------------------------------
// 可行驶域栅格导出的几何辅助（闭集语义：接触算相交）。
// ---------------------------------------------------------------------------

// 简单多边形的闭集点包含测试（边界算作包含）。先显式检查点是否落在任一边
// 上（含容差），再用偶数次穿越法，保证边界判定确定。
bool point_in_polygon_closed(const std::vector<common::Point2d>& polygon, double x,
                             double y) {
  const std::size_t count = polygon.size();
  if (count < 3) {
    return false;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const common::Point2d& a = polygon[i];
    const common::Point2d& b = polygon[(i + 1) % count];
    const double cross = (b.x_m - a.x_m) * (y - a.y_m) - (b.y_m - a.y_m) * (x - a.x_m);
    const double scale = 1.0 + std::abs(b.x_m - a.x_m) + std::abs(b.y_m - a.y_m);
    if (std::abs(cross) <= kPointEps * scale &&
        x >= std::min(a.x_m, b.x_m) - kPointEps &&
        x <= std::max(a.x_m, b.x_m) + kPointEps &&
        y >= std::min(a.y_m, b.y_m) - kPointEps &&
        y <= std::max(a.y_m, b.y_m) + kPointEps) {
      return true;
    }
  }
  bool inside = false;
  for (std::size_t i = 0, j = count - 1; i < count; j = i++) {
    const common::Point2d& a = polygon[i];
    const common::Point2d& b = polygon[j];
    if (((a.y_m > y) != (b.y_m > y)) &&
        (x < (b.x_m - a.x_m) * (y - a.y_m) / (b.y_m - a.y_m) + a.x_m)) {
      inside = !inside;
    }
  }
  return inside;
}

// 线段与轴对齐矩形是否相交（闭集，接触算相交）。二维 Liang-Barsky 裁剪，
// 与上面的 clip_segment 相同的鲁棒性约定。
bool segment_intersects_rect(double ax, double ay, double bx, double by, double min_x,
                             double min_y, double max_x, double max_y) {
  double t0 = 0.0;
  double t1 = 1.0;
  const double dx = bx - ax;
  const double dy = by - ay;
  const double p_values[4] = {-dx, dx, -dy, dy};
  const double q_values[4] = {ax - min_x, max_x - ax, ay - min_y, max_y - ay};
  for (int i = 0; i < 4; ++i) {
    if (std::abs(p_values[i]) < 1e-15) {
      if (q_values[i] < 0.0) {
        return false;
      }
      continue;
    }
    const double r = q_values[i] / p_values[i];
    if (p_values[i] < 0.0) {
      if (r > t1) {
        return false;
      }
      if (r > t0) {
        t0 = r;
      }
    } else {
      if (r < t0) {
        return false;
      }
      if (r < t1) {
        t1 = r;
      }
    }
  }
  return t0 <= t1;
}

// 简单多边形与单元矩形是否相交（闭集）。判定：包围盒快速排除 →
// 任一线段顶点在矩形内 → 任一矩形角在多边形内 → 任一边与矩形相交。
bool polygon_intersects_rect(const std::vector<common::Point2d>& polygon, double min_x,
                             double min_y, double max_x, double max_y) {
  const std::size_t count = polygon.size();
  if (count < 3) {
    return false;
  }
  double p_min_x = polygon.front().x_m;
  double p_max_x = polygon.front().x_m;
  double p_min_y = polygon.front().y_m;
  double p_max_y = polygon.front().y_m;
  for (const common::Point2d& point : polygon) {
    p_min_x = std::min(p_min_x, point.x_m);
    p_max_x = std::max(p_max_x, point.x_m);
    p_min_y = std::min(p_min_y, point.y_m);
    p_max_y = std::max(p_max_y, point.y_m);
  }
  if (p_max_x < min_x || p_min_x > max_x || p_max_y < min_y || p_min_y > max_y) {
    return false;
  }
  for (const common::Point2d& point : polygon) {
    if (point.x_m >= min_x && point.x_m <= max_x && point.y_m >= min_y &&
        point.y_m <= max_y) {
      return true;
    }
  }
  if (point_in_polygon_closed(polygon, min_x, min_y) ||
      point_in_polygon_closed(polygon, max_x, min_y) ||
      point_in_polygon_closed(polygon, min_x, max_y) ||
      point_in_polygon_closed(polygon, max_x, max_y)) {
    return true;
  }
  for (std::size_t i = 0; i < count; ++i) {
    const common::Point2d& a = polygon[i];
    const common::Point2d& b = polygon[(i + 1) % count];
    if (segment_intersects_rect(a.x_m, a.y_m, b.x_m, b.y_m, min_x, min_y, max_x,
                                max_y)) {
      return true;
    }
  }
  return false;
}

// 为显式给出的投影类型和原点创建投影器。当投影模型拒绝该原点时抛出异常。
std::unique_ptr<lanelet::Projector> create_projector(ProjectionKind kind,
                                                     const GeoPoint& origin) {
  const lanelet::Origin lc_origin{
      lanelet::GPSPoint{origin.lat_deg, origin.lon_deg, origin.ele_m}};
  switch (kind) {
    case ProjectionKind::kUtm:
      return std::make_unique<lanelet::projection::UtmProjector>(lc_origin);
    case ProjectionKind::kMercator:
      return std::make_unique<lanelet::projection::Mercator>(lc_origin);
    case ProjectionKind::kSphericalMercator:
      return std::make_unique<lanelet::projection::SphericalMercatorProjector>(lc_origin);
  }
  return nullptr;
}

double checked_coordinate(double value, const ObjectId& id, const char* field) {
  if (!std::isfinite(value)) {
    throw std::runtime_error("map snapshot: non-finite " + std::string(field) + " in object " +
                             id);
  }
  return value;
}

nlohmann::json points_to_json(const std::vector<MapPoint>& points) {
  nlohmann::json out = nlohmann::json::array();
  for (const auto& point : points) {
    out.push_back(nlohmann::json{{"id", point.id},
                                 {"x_m", checked_coordinate(point.x_m, point.id, "x_m")},
                                 {"y_m", checked_coordinate(point.y_m, point.id, "y_m")},
                                 {"z_m", checked_coordinate(point.z_m, point.id, "z_m")}});
  }
  return out;
}

// 收集一个 regulatory element 角色所引用的对象 id。weak 变体在对象
// 仍存活时解析为当前 lanelet/区域。
class IdCollectorVisitor : public lanelet::RuleParameterVisitor {
 public:
  void operator()(const lanelet::ConstPoint3d& point) override {
    ids.push_back(std::to_string(point.id()));
  }
  void operator()(const lanelet::ConstLineString3d& line_string) override {
    ids.push_back(std::to_string(line_string.id()));
  }
  void operator()(const lanelet::ConstPolygon3d& polygon) override {
    ids.push_back(std::to_string(polygon.id()));
  }
  void operator()(const lanelet::ConstWeakLanelet& lanelet) override {
    if (!lanelet.expired()) {
      ids.push_back(std::to_string(lanelet.lock().id()));
    }
  }
  void operator()(const lanelet::ConstWeakArea& area) override {
    if (!area.expired()) {
      ids.push_back(std::to_string(area.lock().id()));
    }
  }

  std::vector<ObjectId> ids;
};

}  // namespace

// ---------------------------------------------------------------------------
// 小型值类型。
// ---------------------------------------------------------------------------

std::string to_string(ProjectionKind kind) {
  switch (kind) {
    case ProjectionKind::kUtm:
      return "utm";
    case ProjectionKind::kMercator:
      return "mercator";
    case ProjectionKind::kSphericalMercator:
      return "spherical_mercator";
  }
  return "unknown";
}

std::optional<ProjectionKind> projection_kind_from_string(const std::string& name) {
  const std::string lower = lower_copy(name);
  if (lower == "utm") {
    return ProjectionKind::kUtm;
  }
  if (lower == "mercator") {
    return ProjectionKind::kMercator;
  }
  if (lower == "spherical_mercator") {
    return ProjectionKind::kSphericalMercator;
  }
  return std::nullopt;
}

bool Bounds2d::contains(double x_m, double y_m) const noexcept {
  return x_m >= min_x_m && x_m <= max_x_m && y_m >= min_y_m && y_m <= max_y_m;
}

bool Bounds2d::intersects(const Bounds2d& other) const noexcept {
  if (empty() || other.empty()) {
    return false;
  }
  return min_x_m <= other.max_x_m && other.min_x_m <= max_x_m && min_y_m <= other.max_y_m &&
         other.min_y_m <= max_y_m;
}

bool Bounds2d::empty() const noexcept {
  return !(min_x_m <= max_x_m && min_y_m <= max_y_m);
}

std::size_t MapObjectCounts::total() const noexcept {
  return points + line_strings + polygons + lanelets + areas + regulatory_elements;
}

common::Point2d CropResult::local_to_map(const common::Point2d& p) const noexcept {
  const double c = std::cos(window_frame.yaw_rad);
  const double s = std::sin(window_frame.yaw_rad);
  return common::Point2d{window_frame.x_m + c * p.x_m - s * p.y_m,
                         window_frame.y_m + s * p.x_m + c * p.y_m};
}

common::Point2d CropResult::map_to_local(const common::Point2d& p) const noexcept {
  const double dx = p.x_m - window_frame.x_m;
  const double dy = p.y_m - window_frame.y_m;
  const double c = std::cos(window_frame.yaw_rad);
  const double s = std::sin(window_frame.yaw_rad);
  return common::Point2d{c * dx + s * dy, -s * dx + c * dy};
}

bool CropResult::empty() const noexcept {
  return paths.empty() && points.empty() && lanelet_ids.empty();
}

// ---------------------------------------------------------------------------
// Map::Impl.
// ---------------------------------------------------------------------------

struct Map::Impl {
  MapInfo info;
  std::unique_ptr<lanelet::LaneletMap> map;
  std::unique_ptr<lanelet::Projector> projector;
  lanelet::traffic_rules::TrafficRulesUPtr rules;

  // 惰性路由图：在首次拓扑查询时构建。
  mutable std::unique_ptr<lanelet::routing::RoutingGraph> routing;
  mutable bool routing_attempted{false};
  mutable std::string routing_error;

  bool ensure_routing() const {
    if (routing) {
      return true;
    }
    if (routing_attempted) {
      return false;
    }
    routing_attempted = true;
    try {
      routing = lanelet::routing::RoutingGraph::build(*map, *rules);
    } catch (const std::exception& error) {
      routing_error = join_reason("routing graph unavailable", error);
      return false;
    }
    if (!routing) {
      routing_error = "routing graph unavailable: builder returned no graph";
      return false;
    }
    return true;
  }

  IdListResult relation_query(
      const ObjectId& id,
      const std::function<lanelet::ConstLanelets(const lanelet::routing::RoutingGraph&,
                                                 const lanelet::ConstLanelet&)>& query) const {
    IdListResult result;
    const auto lanelet_id = parse_object_id(id);
    if (!lanelet_id) {
      result.reason = "invalid lanelet id: " + id;
      return result;
    }
    if (!ensure_routing()) {
      result.reason = routing_error;
      return result;
    }
    const auto it = map->laneletLayer.find(*lanelet_id);
    if (it == map->laneletLayer.end()) {
      result.reason = "unknown lanelet id: " + id;
      return result;
    }
    const lanelet::ConstLanelet lanelet = *it;
    lanelet::ConstLanelets found;
    try {
      found = query(*routing, lanelet);
    } catch (const std::exception& error) {
      result.reason = join_reason("topology query failed", error);
      return result;
    }
    result.ids.reserve(found.size());
    for (const auto& neighbor : found) {
      result.ids.push_back(std::to_string(neighbor.id()));
    }
    sort_object_ids(result.ids);
    result.status = common::Status::kSuccess;
    return result;
  }
};

Map::Map(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
Map::Map(Map&&) noexcept = default;
Map& Map::operator=(Map&&) noexcept = default;
Map::~Map() = default;

const MapInfo& Map::info() const noexcept { return impl_->info; }

// ---------------------------------------------------------------------------
// 导入。
// ---------------------------------------------------------------------------

ImportResult Map::import_osm_file(const std::string& path, const MapLoadOptions& options) {
  ImportResult result;
  result.status = common::Status::kInvalidInput;

  // 1. 路径 / 扩展名。
  std::error_code ec;
  const std::filesystem::path fs_path(path);
  if (!std::filesystem::exists(fs_path, ec) || ec) {
    result.reason = "map file not found: " + path;
    return result;
  }
  if (!std::filesystem::is_regular_file(fs_path, ec) || ec) {
    result.reason = "map path is not a regular file: " + path;
    return result;
  }
  const std::string extension = lower_copy(fs_path.extension().string());
  if (extension != ".osm") {
    result.reason =
        "unsupported map extension '" + extension + "': only .osm Lanelet2 maps are supported";
    return result;
  }

  // 2. 显式投影原点。
  if (!options.origin.has_value()) {
    result.reason =
        "projection origin is required: set MapLoadOptions::origin (lat/lon) explicitly";
    return result;
  }
  const GeoPoint& origin = *options.origin;
  if (!finite(origin)) {
    result.reason = "invalid projection origin: coordinates must be finite";
    return result;
  }
  if (origin.lat_deg < -90.0 || origin.lat_deg > 90.0 || origin.lon_deg < -180.0 ||
      origin.lon_deg > 180.0) {
    result.reason = "invalid projection origin: lat must be in [-90, 90] and lon in [-180, 180]";
    return result;
  }

  // 3. 显式交通规则配置（供路由/邻接使用）。
  lanelet::traffic_rules::TrafficRulesUPtr rules;
  try {
    rules = lanelet::traffic_rules::TrafficRulesFactory::create(
        options.traffic_rules_location, options.traffic_rules_participant);
  } catch (const std::exception& error) {
    result.reason = "invalid traffic rules configuration (" + options.traffic_rules_location +
                    "/" + options.traffic_rules_participant + "): " + error.what();
    return result;
  }

  // 4. 解析前先计算源哈希（地图版本）。
  std::size_t source_bytes = 0;
  std::string map_version;
  try {
    map_version = detail::sha256_file_hex(path, &source_bytes);
  } catch (const std::exception& error) {
    result.reason = join_reason("cannot read map file", error);
    return result;
  }

  // 5. 为配置的模型和原点创建投影器。
  std::unique_ptr<lanelet::Projector> projector;
  try {
    projector = create_projector(options.projection, origin);
  } catch (const std::exception& error) {
    result.reason = join_reason("projection origin rejected by " + to_string(options.projection),
                                error);
    return result;
  }
  if (!projector) {
    result.reason = "projection setup failed: unknown projection kind";
    return result;
  }

  // 6. 加载 OSM 地图。可恢复的解析错误会被收集而不是抛出；
  //    不可恢复的错误（文件不可读、XML 损坏）在此显现。
  lanelet::ErrorMessages load_errors;
  std::unique_ptr<lanelet::LaneletMap> lmap;
  try {
    lmap = lanelet::load(path, *projector, &load_errors);
  } catch (const std::exception& error) {
    result.reason = join_reason("OSM load failed", error);
    return result;
  }
  if (!lmap) {
    result.reason = "OSM load failed: parser returned no map";
    return result;
  }

  auto& diagnostics = result.diagnostics;
  const auto add_diagnostic = [&diagnostics](std::string text) {
    if (diagnostics.size() < kMaxDiagnostics) {
      diagnostics.push_back(std::move(text));
    } else if (diagnostics.size() == kMaxDiagnostics) {
      diagnostics.push_back("... diagnostics truncated");
    }
  };
  for (const auto& message : load_errors) {
    add_diagnostic("osm parse: " + message);
  }

  // 7. 元数据：计数、边界、标识。
  MapInfo info;
  info.map_id = fs_path.stem().string();
  info.map_version = std::move(map_version);
  info.source_path = path;
  info.source_bytes = source_bytes;
  info.projection = options.projection;
  info.origin = origin;
  info.traffic_rules_location = options.traffic_rules_location;
  info.traffic_rules_participant = options.traffic_rules_participant;
  info.counts.points = lmap->pointLayer.size();
  info.counts.line_strings = lmap->lineStringLayer.size();
  info.counts.polygons = lmap->polygonLayer.size();
  info.counts.lanelets = lmap->laneletLayer.size();
  info.counts.areas = lmap->areaLayer.size();
  info.counts.regulatory_elements = lmap->regulatoryElementLayer.size();

  BBox bounds;
  for (const auto& point : lmap->pointLayer) {
    if (!finite(point.x()) || !finite(point.y())) {
      continue;
    }
    if (!bounds.valid) {
      bounds = BBox{point.x(), point.y(), point.x(), point.y(), true};
      continue;
    }
    bounds.min_x = std::min(bounds.min_x, point.x());
    bounds.min_y = std::min(bounds.min_y, point.y());
    bounds.max_x = std::max(bounds.max_x, point.x());
    bounds.max_y = std::max(bounds.max_y, point.y());
  }
  if (bounds.valid) {
    info.bounds_m = Bounds2d{bounds.min_x, bounds.min_y, bounds.max_x, bounds.max_y};
  }

  if (info.counts.total() == 0) {
    if (!load_errors.empty()) {
      result.reason = "OSM parse failed: " + load_errors.front();
    } else {
      result.reason = "no map objects found: the file contains no usable Lanelet2 objects";
    }
    return result;
  }
  if (info.counts.lanelets == 0 && info.counts.areas == 0) {
    add_diagnostic("map contains no lanelets or areas (no road network)");
  }

  // 8. 可选的验证统计（上游 lanelet2_validation）。
  if (options.validate_on_import) {
    try {
      lanelet::validation::ValidationConfig config;
      config.checksFilter = "";
      config.location = options.traffic_rules_location;
      config.participants = {options.traffic_rules_participant};
      config.origin = lanelet::GPSPoint{origin.lat_deg, origin.lon_deg, origin.ele_m};
      const auto detected = lanelet::validation::validateMap(*lmap, config);
      const auto report = lanelet::validation::buildReport(detected);
      info.validation.checks_run = lanelet::validation::availabeChecks("").size();
      info.validation.errors = report.errors.size();
      info.validation.warnings = report.warnings.size();
      for (const auto& warning : report.warnings) {
        add_diagnostic("validation warning: " + warning);
      }
      for (const auto& error : report.errors) {
        add_diagnostic("validation error: " + error);
      }
    } catch (const std::exception& error) {
      add_diagnostic(join_reason("validation skipped", error));
    }
  }

  // 9. 组装地图快照。
  auto impl = std::make_unique<Impl>();
  impl->info = std::move(info);
  impl->map = std::move(lmap);
  impl->projector = std::move(projector);
  impl->rules = std::move(rules);

  result.map = std::unique_ptr<Map>(new Map(std::move(impl)));
  result.status = common::Status::kSuccess;
  result.reason.clear();
  return result;
}

// ---------------------------------------------------------------------------
// 坐标。
// ---------------------------------------------------------------------------

std::optional<GeoPoint> Map::to_gps(double x_m, double y_m, double z_m) const {
  if (!finite(x_m) || !finite(y_m) || !finite(z_m)) {
    return std::nullopt;
  }
  try {
    const auto gps = impl_->projector->reverse(lanelet::BasicPoint3d{x_m, y_m, z_m});
    GeoPoint out{gps.lat, gps.lon, gps.ele};
    if (!finite(out)) {
      return std::nullopt;
    }
    return out;
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

std::optional<common::Point2d> Map::to_local(const GeoPoint& gps) const {
  if (!finite(gps) || gps.lat_deg < -90.0 || gps.lat_deg > 90.0 || gps.lon_deg < -180.0 ||
      gps.lon_deg > 180.0) {
    return std::nullopt;
  }
  try {
    const auto point =
        impl_->projector->forward(lanelet::GPSPoint{gps.lat_deg, gps.lon_deg, gps.ele_m});
    if (!finite(point.x()) || !finite(point.y())) {
      return std::nullopt;
    }
    return common::Point2d{point.x(), point.y()};
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

std::optional<common::Point2d> Map::project_to_local(const GeoPoint& gps,
                                                     ProjectionKind projection,
                                                     const GeoPoint& origin) {
  const bool origin_ok =
      finite(origin) && origin.lat_deg >= -90.0 && origin.lat_deg <= 90.0 &&
      origin.lon_deg >= -180.0 && origin.lon_deg <= 180.0;
  if (!finite(gps) || gps.lat_deg < -90.0 || gps.lat_deg > 90.0 || gps.lon_deg < -180.0 ||
      gps.lon_deg > 180.0 || !origin_ok) {
    return std::nullopt;
  }
  try {
    const auto projector = create_projector(projection, origin);
    const auto point =
        projector->forward(lanelet::GPSPoint{gps.lat_deg, gps.lon_deg, gps.ele_m});
    if (!finite(point.x()) || !finite(point.y())) {
      return std::nullopt;
    }
    return common::Point2d{point.x(), point.y()};
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

std::optional<GeoPoint> Map::project_to_gps(const common::Point2d& local,
                                            ProjectionKind projection,
                                            const GeoPoint& origin) {
  const bool origin_ok =
      finite(origin) && origin.lat_deg >= -90.0 && origin.lat_deg <= 90.0 &&
      origin.lon_deg >= -180.0 && origin.lon_deg <= 180.0;
  if (!finite(local.x_m) || !finite(local.y_m) || !origin_ok) {
    return std::nullopt;
  }
  try {
    const auto projector = create_projector(projection, origin);
    const auto gps = projector->reverse(lanelet::BasicPoint3d{local.x_m, local.y_m, 0.0});
    GeoPoint out{gps.lat, gps.lon, gps.ele};
    if (!finite(out)) {
      return std::nullopt;
    }
    return out;
  } catch (const std::exception&) {
    return std::nullopt;
  }
}

// ---------------------------------------------------------------------------
// 对象查询。
// ---------------------------------------------------------------------------

std::vector<ObjectId> Map::lanelet_ids() const {
  std::vector<ObjectId> ids;
  ids.reserve(impl_->map->laneletLayer.size());
  for (const auto& lanelet : impl_->map->laneletLayer) {
    ids.push_back(std::to_string(lanelet.id()));
  }
  sort_object_ids(ids);
  return ids;
}

std::optional<LaneletGeometry> Map::lanelet_geometry(const ObjectId& id) const {
  const auto lanelet_id = parse_object_id(id);
  if (!lanelet_id) {
    return std::nullopt;
  }
  const auto it = impl_->map->laneletLayer.find(*lanelet_id);
  if (it == impl_->map->laneletLayer.end()) {
    return std::nullopt;
  }
  const lanelet::ConstLanelet lanelet = *it;
  LaneletGeometry out;
  out.id = std::to_string(lanelet.id());
  out.left = to_points(lanelet.leftBound());
  out.right = to_points(lanelet.rightBound());
  try {
    out.centerline = to_points(lanelet.centerline());
  } catch (const std::exception&) {
    out.centerline.clear();  // 该 lanelet 几何错误；边界仍可用
  }
  out.attributes = attributes_to_json(lanelet.attributes());
  return out;
}

std::vector<ObjectId> Map::area_ids() const {
  std::vector<ObjectId> ids;
  ids.reserve(impl_->map->areaLayer.size());
  for (const auto& area : impl_->map->areaLayer) {
    ids.push_back(std::to_string(area.id()));
  }
  sort_object_ids(ids);
  return ids;
}

std::optional<AreaGeometry> Map::area_geometry(const ObjectId& id) const {
  const auto area_id = parse_object_id(id);
  if (!area_id) {
    return std::nullopt;
  }
  const auto it = impl_->map->areaLayer.find(*area_id);
  if (it == impl_->map->areaLayer.end()) {
    return std::nullopt;
  }
  const lanelet::ConstArea area = *it;
  AreaGeometry out;
  out.id = std::to_string(area.id());
  out.outer = join_line_strings(area.outerBound());
  for (const auto& inner : area.innerBounds()) {
    out.inner.push_back(join_line_strings(inner));
  }
  out.attributes = attributes_to_json(area.attributes());
  return out;
}

std::vector<MapPoint> Map::points_in_bounds(const Bounds2d& window) const {
  std::vector<MapPoint> out;
  if (window.empty()) {
    return out;
  }
  const lanelet::LaneletMap& map = *impl_->map;
  const auto candidates = map.pointLayer.search(search_box(window));
  out.reserve(candidates.size());
  for (const auto& point : candidates) {
    if (window.contains(point.x(), point.y())) {
      out.push_back(MapPoint{std::to_string(point.id()), point.x(), point.y(), point.z()});
    }
  }
  sort_points(out);
  return out;
}

std::vector<ObjectId> Map::lanelets_in_bounds(const Bounds2d& window) const {
  std::vector<ObjectId> out;
  if (window.empty()) {
    return out;
  }
  const lanelet::LaneletMap& map = *impl_->map;
  const auto candidates = map.laneletLayer.search(search_box(window));
  out.reserve(candidates.size());
  for (const auto& lanelet : candidates) {
    out.push_back(std::to_string(lanelet.id()));
  }
  sort_object_ids(out);
  return out;
}

PointQueryResult Map::query_point(double x_m, double y_m) const {
  PointQueryResult out;
  if (!finite(x_m) || !finite(y_m)) {
    return out;
  }
  const lanelet::BasicPoint2d point{x_m, y_m};
  const lanelet::BoundingBox2d box{point, point};
  const lanelet::LaneletMap& map = *impl_->map;
  for (const auto& lanelet : map.laneletLayer.search(box)) {
    if (lanelet::geometry::inside(lanelet, point)) {
      out.lanelets.push_back(lanelet_summary(lanelet));
    }
  }
  for (const auto& area : map.areaLayer.search(box)) {
    if (lanelet::geometry::inside(area, point)) {
      out.areas.push_back(area_summary(area));
    }
  }
  sort_summaries(out.lanelets);
  sort_summaries(out.areas);
  return out;
}

BoundsQueryResult Map::query_bounds(const Bounds2d& window) const {
  BoundsQueryResult out;
  if (window.empty() || !finite(window.min_x_m) || !finite(window.min_y_m) ||
      !finite(window.max_x_m) || !finite(window.max_y_m)) {
    return out;
  }
  const auto box = search_box(window);
  const lanelet::LaneletMap& map = *impl_->map;
  for (const auto& lanelet : map.laneletLayer.search(box)) {
    out.lanelets.push_back(lanelet_summary(lanelet));
  }
  for (const auto& area : map.areaLayer.search(box)) {
    out.areas.push_back(area_summary(area));
  }
  sort_summaries(out.lanelets);
  sort_summaries(out.areas);
  return out;
}

// ---------------------------------------------------------------------------
// 可行驶域栅格导出（见 map.hpp 的语义说明）。
// ---------------------------------------------------------------------------

DriveableGridResult Map::export_driveable_grid(const Bounds2d& window,
                                               double resolution_m) const {
  DriveableGridResult result;
  if (!finite(window.min_x_m) || !finite(window.min_y_m) || !finite(window.max_x_m) ||
      !finite(window.max_y_m)) {
    result.reason = "invalid_window";
    return result;
  }
  if (!(window.min_x_m < window.max_x_m) || !(window.min_y_m < window.max_y_m)) {
    result.reason = "empty_window";
    return result;
  }
  if (!finite(resolution_m) || !(resolution_m > 0.0)) {
    result.reason = "invalid_resolution";
    return result;
  }

  // 窗口尺寸与单元预算复用 planner::grid 的数学助手（X-A1-17）；语义与
  // 迁移前一致：ceil 维度、超过 kMaxDriveableGridCells 拒绝。
  const std::optional<grid::WindowGridSpec> spec =
      grid::window_grid_spec(window.max_x_m - window.min_x_m,
                             window.max_y_m - window.min_y_m, resolution_m);
  if (!spec || spec->cells > kMaxDriveableGridCells) {
    result.reason = "grid_too_large";
    return result;
  }

  common::GridView grid;
  grid.map_id = impl_->info.map_id;
  grid.map_version = impl_->info.map_version;
  grid.width = spec->width;
  grid.height = spec->height;
  grid.resolution_m = resolution_m;
  grid.origin = common::Pose2d{window.min_x_m, window.min_y_m, 0.0};
  grid.cells.assign(static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height),
                    common::kCellOccupied);

  const lanelet::LaneletMap& map = *impl_->map;
  std::vector<common::Point2d> polygon;
  std::vector<common::Point2d> right_bound;
  for (const auto& lanelet : map.laneletLayer.search(search_box(window))) {
    polygon.clear();
    right_bound.clear();
    polygon.reserve(lanelet.leftBound().size() + lanelet.rightBound().size());
    right_bound.reserve(lanelet.rightBound().size());
    for (const auto& point : lanelet.leftBound()) {
      polygon.push_back(common::Point2d{point.x(), point.y()});
    }
    // lanelet 多边形 = leftBound + 反向 rightBound。
    for (const auto& point : lanelet.rightBound()) {
      right_bound.push_back(common::Point2d{point.x(), point.y()});
    }
    for (auto it = right_bound.rbegin(); it != right_bound.rend(); ++it) {
      polygon.push_back(*it);
    }
    if (polygon.size() < 3) {
      continue;  // 退化几何不构成可行驶区域
    }
    bool polygon_finite = true;
    for (const common::Point2d& point : polygon) {
      if (!finite(point.x_m) || !finite(point.y_m)) {
        polygon_finite = false;
        break;
      }
    }
    if (!polygon_finite) {
      continue;
    }

    double p_min_x = polygon.front().x_m;
    double p_max_x = polygon.front().x_m;
    double p_min_y = polygon.front().y_m;
    double p_max_y = polygon.front().y_m;
    for (const common::Point2d& point : polygon) {
      p_min_x = std::min(p_min_x, point.x_m);
      p_max_x = std::max(p_max_x, point.x_m);
      p_min_y = std::min(p_min_y, point.y_m);
      p_max_y = std::max(p_max_y, point.y_m);
    }
    // 车道包围盒与窗口的闭集交集（可能是空集）。
    const double lo_x = std::max(p_min_x, window.min_x_m);
    const double hi_x = std::min(p_max_x, window.max_x_m);
    const double lo_y = std::max(p_min_y, window.min_y_m);
    const double hi_y = std::min(p_max_y, window.max_y_m);
    if (lo_x > hi_x || lo_y > hi_y) {
      continue;
    }
    // 覆盖闭集的单元索引范围（单元边界接触也计入）。
    const int col_begin =
        std::max(0, static_cast<int>(std::floor((lo_x - window.min_x_m) / resolution_m)));
    const int col_end =
        std::min(grid.width - 1,
                 static_cast<int>(std::floor((hi_x - window.min_x_m) / resolution_m)));
    const int row_begin =
        std::max(0, static_cast<int>(std::floor((lo_y - window.min_y_m) / resolution_m)));
    const int row_end =
        std::min(grid.height - 1,
                 static_cast<int>(std::floor((hi_y - window.min_y_m) / resolution_m)));
    for (int row = row_begin; row <= row_end; ++row) {
      const double cell_min_y = window.min_y_m + row * resolution_m;
      const double cell_max_y = cell_min_y + resolution_m;
      for (int col = col_begin; col <= col_end; ++col) {
        const std::size_t index = static_cast<std::size_t>(row) *
                                      static_cast<std::size_t>(grid.width) +
                                  static_cast<std::size_t>(col);
        if (grid.cells[index] == common::kCellFree) {
          continue;
        }
        const double cell_min_x = window.min_x_m + col * resolution_m;
        const double cell_max_x = cell_min_x + resolution_m;
        if (polygon_intersects_rect(polygon, cell_min_x, cell_min_y, cell_max_x,
                                    cell_max_y)) {
          grid.cells[index] = common::kCellFree;
        }
      }
    }
  }

  result.status = common::Status::kSuccess;
  result.grid = std::move(grid);
  return result;
}

// ---------------------------------------------------------------------------
// 拓扑与路由。
// ---------------------------------------------------------------------------

IdListResult Map::following(const ObjectId& lanelet_id) const {
  return impl_->relation_query(
      lanelet_id,
      [](const lanelet::routing::RoutingGraph& graph, const lanelet::ConstLanelet& lanelet) {
        return graph.following(lanelet);
      });
}

IdListResult Map::preceding(const ObjectId& lanelet_id) const {
  return impl_->relation_query(
      lanelet_id,
      [](const lanelet::routing::RoutingGraph& graph, const lanelet::ConstLanelet& lanelet) {
        return graph.previous(lanelet);
      });
}

IdListResult Map::adjacent(const ObjectId& lanelet_id, NeighborSide side) const {
  return impl_->relation_query(
      lanelet_id,
      [side](const lanelet::routing::RoutingGraph& graph, const lanelet::ConstLanelet& lanelet) {
        return side == NeighborSide::kLeft ? graph.adjacentLefts(lanelet)
                                           : graph.adjacentRights(lanelet);
      });
}

RouteResult Map::route(const ObjectId& from_lanelet, const ObjectId& to_lanelet) const {
  RouteResult result;
  const auto from_id = parse_object_id(from_lanelet);
  if (!from_id) {
    result.reason = "invalid lanelet id: " + from_lanelet;
    return result;
  }
  const auto to_id = parse_object_id(to_lanelet);
  if (!to_id) {
    result.reason = "invalid lanelet id: " + to_lanelet;
    return result;
  }
  if (!impl_->ensure_routing()) {
    result.reason = impl_->routing_error;
    return result;
  }
  const auto from_it = impl_->map->laneletLayer.find(*from_id);
  const auto to_it = impl_->map->laneletLayer.find(*to_id);
  if (from_it == impl_->map->laneletLayer.end() || to_it == impl_->map->laneletLayer.end()) {
    result.reason = from_it == impl_->map->laneletLayer.end()
                        ? "unknown lanelet id: " + from_lanelet
                        : "unknown lanelet id: " + to_lanelet;
    return result;
  }
  try {
    const auto route = impl_->routing->getRoute(*from_it, *to_it, 0);
    if (!route) {
      result.status = common::Status::kNoPath;
      result.reason = "no route from lanelet " + from_lanelet + " to lanelet " + to_lanelet;
      return result;
    }
    const auto& path = route->shortestPath();
    result.lanelet_ids.reserve(path.size());
    for (const auto& lanelet : path) {
      result.lanelet_ids.push_back(std::to_string(lanelet.id()));
    }
    result.length_m = route->length2d();
    if (result.lanelet_ids.empty()) {
      result.status = common::Status::kNoPath;
      result.reason = "route is empty";
      return result;
    }
    result.status = common::Status::kSuccess;
    return result;
  } catch (const std::exception& error) {
    result.reason = join_reason("route query failed", error);
    return result;
  }
}

// ---------------------------------------------------------------------------
// 局部窗口裁剪（仅几何；拓扑仍在完整地图上）。
// ---------------------------------------------------------------------------

CropResult Map::crop_to_bounds(const Bounds2d& window) const {
  CropResult out;
  out.window_m = window;
  out.window_frame = common::Pose2d{window.min_x_m, window.min_y_m, 0.0};
  if (window.empty() || !finite(window.min_x_m) || !finite(window.min_y_m) ||
      !finite(window.max_x_m) || !finite(window.max_y_m)) {
    return out;
  }

  struct Piece {
    ObjectId lanelet_id;
    std::string role;
    std::size_t sequence;
    ClippedPiece piece;
    lanelet::Id numeric_id;
    int role_rank;
  };
  std::vector<Piece> pieces;

  const lanelet::LaneletMap& map = *impl_->map;
  for (const auto& lanelet : map.laneletLayer.search(search_box(window))) {
    const auto left = to_points(lanelet.leftBound());
    const auto right = to_points(lanelet.rightBound());
    const ObjectId lanelet_id = std::to_string(lanelet.id());
    out.lanelet_ids.push_back(lanelet_id);

    std::vector<MapPoint> centerline;
    try {
      centerline = to_points(lanelet.centerline());
    } catch (const std::exception& error) {
      out.diagnostics.push_back(join_reason(
          "centerline unavailable for lanelet " + lanelet_id, error));
    }

    struct Source {
      const std::vector<MapPoint>* points;
      const char* role;
      int rank;
    };
    const Source sources[3] = {
        {&left, "left", 0}, {&right, "right", 1}, {&centerline, "centerline", 2}};
    for (const auto& source : sources) {
      auto clipped = clip_polyline(*source.points, window);
      for (std::size_t i = 0; i < clipped.size(); ++i) {
        pieces.push_back(Piece{lanelet_id, source.role, i, std::move(clipped[i]), lanelet.id(),
                               source.rank});
      }
    }
  }

  sort_object_ids(out.lanelet_ids);
  std::stable_sort(pieces.begin(), pieces.end(), [](const Piece& lhs, const Piece& rhs) {
    if (lhs.numeric_id != rhs.numeric_id) {
      return lhs.numeric_id < rhs.numeric_id;
    }
    if (lhs.role_rank != rhs.role_rank) {
      return lhs.role_rank < rhs.role_rank;
    }
    return lhs.sequence < rhs.sequence;
  });
  out.paths.reserve(pieces.size());
  for (auto& piece : pieces) {
    out.paths.push_back(ClippedPath{piece.lanelet_id, piece.role, std::move(piece.piece.points),
                                    piece.piece.start_clip_edge, piece.piece.end_clip_edge});
  }
  out.points = points_in_bounds(window);
  return out;
}

// ---------------------------------------------------------------------------
// 公开 JSON 地图快照（M4）。
// ---------------------------------------------------------------------------

nlohmann::json Map::to_snapshot_json(const SnapshotOptions& options) const {
  const MapInfo& info = impl_->info;
  nlohmann::json doc = nlohmann::json::object();
  doc["schema_version"] = common::kSchemaVersion;
  doc["type"] = "map_snapshot";
  doc["map_id"] = info.map_id;
  doc["map_version"] = info.map_version;
  doc["source"] = nlohmann::json{{"path", info.source_path}, {"bytes", info.source_bytes}};
  doc["coordinate_system"] =
      nlohmann::json{{"frame_id", "map"},
                     {"unit", "m"},
                     {"x_axis", "east"},
                     {"y_axis", "north"},
                     {"projection", to_string(info.projection)},
                     {"origin", nlohmann::json{{"lat_deg", info.origin.lat_deg},
                                               {"lon_deg", info.origin.lon_deg},
                                               {"ele_m", info.origin.ele_m}}}};
  doc["bounds_m"] = nlohmann::json{{"min_x_m", info.bounds_m.min_x_m},
                                   {"min_y_m", info.bounds_m.min_y_m},
                                   {"max_x_m", info.bounds_m.max_x_m},
                                   {"max_y_m", info.bounds_m.max_y_m}};
  doc["counts"] = nlohmann::json{{"points", info.counts.points},
                                 {"line_strings", info.counts.line_strings},
                                 {"polygons", info.counts.polygons},
                                 {"lanelets", info.counts.lanelets},
                                 {"areas", info.counts.areas},
                                 {"regulatory_elements", info.counts.regulatory_elements}};
  doc["traffic_rules"] = nlohmann::json{{"location", info.traffic_rules_location},
                                        {"participant", info.traffic_rules_participant}};

  const lanelet::LaneletMap& map = *impl_->map;

  // lanelet 按数字 id 排序：图层迭代顺序不稳定，
  // 导出的文档必须稳定。
  std::vector<lanelet::ConstLanelet> lanelets;
  lanelets.reserve(map.laneletLayer.size());
  for (const auto& lanelet : map.laneletLayer) {
    lanelets.push_back(lanelet);
  }
  std::sort(lanelets.begin(), lanelets.end(),
            [](const lanelet::ConstLanelet& lhs, const lanelet::ConstLanelet& rhs) {
              return lhs.id() < rhs.id();
            });
  nlohmann::json lanelets_json = nlohmann::json::array();
  for (const auto& lanelet : lanelets) {
    nlohmann::json entry = nlohmann::json::object();
    entry["id"] = std::to_string(lanelet.id());
    entry["attributes"] = attributes_to_json(lanelet.attributes());
    entry["left"] = points_to_json(to_points(lanelet.leftBound()));
    entry["right"] = points_to_json(to_points(lanelet.rightBound()));
    if (options.include_centerline) {
      try {
        entry["centerline"] = points_to_json(to_points(lanelet.centerline()));
      } catch (const std::exception&) {
        entry["centerline"] = nlohmann::json::array();
      }
    }
    lanelets_json.push_back(std::move(entry));
  }
  doc["lanelets"] = std::move(lanelets_json);

  if (options.include_areas) {
    std::vector<lanelet::ConstArea> areas;
    areas.reserve(map.areaLayer.size());
    for (const auto& area : map.areaLayer) {
      areas.push_back(area);
    }
    std::sort(areas.begin(), areas.end(),
              [](const lanelet::ConstArea& lhs, const lanelet::ConstArea& rhs) {
                return lhs.id() < rhs.id();
              });
    nlohmann::json areas_json = nlohmann::json::array();
    for (const auto& area : areas) {
      nlohmann::json inner = nlohmann::json::array();
      for (const auto& ring : area.innerBounds()) {
        inner.push_back(points_to_json(join_line_strings(ring)));
      }
      areas_json.push_back(
          nlohmann::json{{"id", std::to_string(area.id())},
                         {"attributes", attributes_to_json(area.attributes())},
                         {"outer", points_to_json(join_line_strings(area.outerBound()))},
                         {"inner", std::move(inner)}});
    }
    doc["areas"] = std::move(areas_json);
  }

  if (options.include_regulatory_elements) {
    std::vector<lanelet::RegulatoryElementConstPtr> regulatory_elements;
    regulatory_elements.reserve(map.regulatoryElementLayer.size());
    for (const auto& element : map.regulatoryElementLayer) {
      regulatory_elements.push_back(element);
    }
    std::sort(regulatory_elements.begin(), regulatory_elements.end(),
              [](const lanelet::RegulatoryElementConstPtr& lhs,
                 const lanelet::RegulatoryElementConstPtr& rhs) {
                return lhs->id() < rhs->id();
              });
    nlohmann::json elements_json = nlohmann::json::array();
    for (const auto& element : regulatory_elements) {
      nlohmann::json roles = nlohmann::json::object();
      for (const auto& role : element->getParameters()) {
        IdCollectorVisitor collector;
        for (const auto& parameter : role.second) {
          boost::apply_visitor(collector, parameter);
        }
        roles[role.first] = collector.ids;
      }
      elements_json.push_back(
          nlohmann::json{{"id", std::to_string(element->id())},
                         {"attributes", attributes_to_json(element->attributes())},
                         {"roles", std::move(roles)}});
    }
    doc["regulatory_elements"] = std::move(elements_json);
  }

  return doc;
}

}  // namespace planner::map
