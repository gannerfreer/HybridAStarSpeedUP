// planner_map_service：本地 C++ HTTP 地图/查询服务（W2）。
//
// 以显式投影原点加载 Lanelet2 OSM 地图（与库相同的导入路径），
// 并为浏览器工作台提供一个小型 JSON API。每个响应都携带通用封装
// （{"schema_version": 1, "type": ...}）；错误在响应体中使用 Status 语义，
// 并匹配相应的 HTTP 状态码：
//   400 无效 JSON 请求体 / 字段缺失 / 值无效（invalid_input）
//   404 未知对象 id（invalid_input）或不可达路由（no_path）
//   500 内部失败（快照/路由图）
//
// 端点：
//   GET  /health
//   GET  /map/info
//   GET  /map/snapshot
//   POST /map/query/point        {"x_m": ..., "y_m": ...}
//   POST /map/query/bounds       {"min_x_m", "min_y_m", "max_x_m", "max_y_m"}
//   GET  /map/lanelet/{id}
//   GET  /map/lanelet/{id}/following | /preceding | /adjacent?side=left|right
//   POST /map/query/window       {"bounds": {...}} or
//                                {"center": {"x_m","y_m"}, "size": {"width_m","height_m"}}
//   POST /map/route              {"from_id": "...", "to_id": "..."}
//   POST /map/grid/driveable     {"window", "resolution_m", "request_seq"?}
//                                → grid_snapshot（source="driveable"；X-M1）
//   POST /map/scene/preview      {"window", "resolution_m", "obstacles"?, "request_seq"?}
//                                → grid_snapshot（source="planning_scene"；X-M1）
//
// 地图访问通过互斥锁串行化：首次拓扑查询会惰性构建路由图，因此按约定
// 并发访问地图不是线程安全的。服务在每次地图调用周围持有一把锁。
//
// 退出码：0 正常关闭；1 地图导入/运行时失败；2 用法错误；
// 3 无法绑定请求的端口。
//
// CLI（components.json service 条目）：
//   planner_map_service --map <osm> --origin-lat <deg> --origin-lon <deg>
//       --port <port> [--run-dir <dir>]

#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "planner/grid/grid.hpp"
#include "planner/map/map.hpp"

#if defined(PLANNER_MAP_SERVICE_WITH_SCENE)
// X-M1（协议 §3.2/§3.3）：栅格/场景预览复用 planner::scene 组合与统一
// scene_id；由组件 CMake 在 scene 目标可用时定义本宏。
#include "planner/scene/content_id.hpp"
#include "planner/scene/scene.hpp"
#include "planner/scene/snapshot.hpp"
#endif

namespace {

using planner::common::Point2d;
using planner::common::Pose2d;
using planner::map::AreaGeometry;
using planner::map::Bounds2d;
using planner::map::CropResult;
using planner::map::GeoPoint;
using planner::map::LaneletGeometry;
using planner::map::Map;
using planner::map::MapInfo;
using planner::map::MapLoadOptions;
using planner::map::MapPoint;
using planner::map::ObjectId;
using planner::map::ObjectSummary;
using planner::map::ProjectionKind;

constexpr int kDefaultPort = 8765;
constexpr const char* kDefaultHost = "127.0.0.1";
// 协议 §3.5/§7 的版本字段（与 planner::scene 常量同值）。
constexpr const char* kGridProtocolVersion = "1.0";
constexpr const char* kSceneRulesVersion = "scene_v1";

std::atomic<bool> g_stop_requested{false};
httplib::Server* g_server = nullptr;

extern "C" void handle_signal(int /*signal*/) {
  g_stop_requested = true;
  if (g_server != nullptr) {
    g_server->stop();
  }
}

struct ServiceState {
  std::unique_ptr<Map> map;
  MapInfo info;
  mutable std::mutex mutex;  // 串行化所有地图访问（惰性路由图）
  mutable std::optional<std::string> snapshot_cache;
  std::chrono::steady_clock::time_point started;
  std::string run_dir;
  bool quiet{false};
};

nlohmann::json envelope(const std::string& type) {
  return nlohmann::json{{"schema_version", 1}, {"type", type}};
}

void send_json(httplib::Response& res, const nlohmann::json& doc, int status = 200) {
  res.status = status;
  res.set_content(doc.dump(), "application/json; charset=utf-8");
}

void send_error(httplib::Response& res, int http_status, const std::string& status,
                const std::string& reason) {
  nlohmann::json doc = envelope("error");
  doc["status"] = status;
  doc["reason"] = reason;
  send_json(res, doc, http_status);
}

// ---------------------------------------------------------------------------
// JSON 构建函数。
// ---------------------------------------------------------------------------

nlohmann::json point_json(const MapPoint& point) {
  return nlohmann::json{
      {"id", point.id}, {"x_m", point.x_m}, {"y_m", point.y_m}, {"z_m", point.z_m}};
}

nlohmann::json points_json(const std::vector<MapPoint>& points) {
  nlohmann::json out = nlohmann::json::array();
  for (const auto& point : points) {
    out.push_back(point_json(point));
  }
  return out;
}

nlohmann::json bounds_json(const Bounds2d& bounds) {
  return nlohmann::json{{"min_x_m", bounds.min_x_m},
                        {"min_y_m", bounds.min_y_m},
                        {"max_x_m", bounds.max_x_m},
                        {"max_y_m", bounds.max_y_m}};
}

nlohmann::json pose_json(const Pose2d& pose) {
  return nlohmann::json{{"x_m", pose.x_m}, {"y_m", pose.y_m}, {"yaw_rad", pose.yaw_rad}};
}

nlohmann::json origin_json(const GeoPoint& origin) {
  return nlohmann::json{
      {"lat_deg", origin.lat_deg}, {"lon_deg", origin.lon_deg}, {"ele_m", origin.ele_m}};
}

nlohmann::json summary_json(const ObjectSummary& summary) {
  nlohmann::json doc{{"id", summary.id}, {"attributes", summary.attributes}};
  if (summary.has_bounds) {
    doc["bounds_m"] = bounds_json(summary.bounds_m);
  }
  return doc;
}

nlohmann::json summaries_json(const std::vector<ObjectSummary>& summaries) {
  nlohmann::json out = nlohmann::json::array();
  for (const auto& summary : summaries) {
    out.push_back(summary_json(summary));
  }
  return out;
}

nlohmann::json map_info_json(const MapInfo& info) {
  nlohmann::json doc = envelope("map_info");
  doc["map_id"] = info.map_id;
  doc["map_version"] = info.map_version;
  doc["source"] = nlohmann::json{{"path", info.source_path}, {"bytes", info.source_bytes}};
  doc["coordinate_system"] =
      nlohmann::json{{"frame_id", "map"},
                     {"unit", "m"},
                     {"x_axis", "east"},
                     {"y_axis", "north"},
                     {"projection", planner::map::to_string(info.projection)},
                     {"origin", origin_json(info.origin)}};
  doc["traffic_rules"] = nlohmann::json{{"location", info.traffic_rules_location},
                                        {"participant", info.traffic_rules_participant}};
  doc["counts"] = nlohmann::json{{"points", info.counts.points},
                                 {"line_strings", info.counts.line_strings},
                                 {"polygons", info.counts.polygons},
                                 {"lanelets", info.counts.lanelets},
                                 {"areas", info.counts.areas},
                                 {"regulatory_elements", info.counts.regulatory_elements}};
  doc["bounds_m"] = bounds_json(info.bounds_m);
  doc["validation"] = nlohmann::json{{"checks_run", info.validation.checks_run},
                                     {"errors", info.validation.errors},
                                     {"warnings", info.validation.warnings}};
  return doc;
}

nlohmann::json lanelet_geometry_json(const LaneletGeometry& geometry) {
  nlohmann::json doc = envelope("lanelet");
  doc["id"] = geometry.id;
  doc["attributes"] = geometry.attributes;
  doc["left"] = points_json(geometry.left);
  doc["right"] = points_json(geometry.right);
  doc["centerline"] = points_json(geometry.centerline);
  return doc;
}

nlohmann::json area_geometry_json(const AreaGeometry& geometry) {
  nlohmann::json inner = nlohmann::json::array();
  for (const auto& ring : geometry.inner) {
    inner.push_back(points_json(ring));
  }
  nlohmann::json doc = envelope("area");
  doc["id"] = geometry.id;
  doc["attributes"] = geometry.attributes;
  doc["outer"] = points_json(geometry.outer);
  doc["inner"] = std::move(inner);
  return doc;
}

nlohmann::json crop_json(const CropResult& crop) {
  nlohmann::json paths = nlohmann::json::array();
  for (const auto& path : crop.paths) {
    paths.push_back(nlohmann::json{{"lanelet_id", path.lanelet_id},
                                   {"role", path.role},
                                   {"points", points_json(path.points)},
                                   {"start_clip_edge", path.start_clip_edge},
                                   {"end_clip_edge", path.end_clip_edge}});
  }
  nlohmann::json doc = envelope("map_window");
  doc["window_m"] = bounds_json(crop.window_m);
  doc["window_frame"] = pose_json(crop.window_frame);
  doc["transform"] =
      nlohmann::json{{"kind", "translation"},
                     {"frame_id", "window"},
                     {"origin_in_map", nlohmann::json{{"x_m", crop.window_frame.x_m},
                                                      {"y_m", crop.window_frame.y_m}}},
                     {"yaw_rad", crop.window_frame.yaw_rad},
                     {"note", "map = window_frame origin + local offset (yaw applied)"}};
  doc["paths"] = std::move(paths);
  doc["points"] = points_json(crop.points);
  doc["lanelet_ids"] = crop.lanelet_ids;
  doc["diagnostics"] = crop.diagnostics;
  return doc;
}

// ---------------------------------------------------------------------------
// 请求解析辅助函数。
// ---------------------------------------------------------------------------

bool parse_finite(const nlohmann::json& body, const char* field, double* out,
                  std::string* error) {
  if (!body.is_object() || !body.contains(field)) {
    *error = std::string("missing field '") + field + "'";
    return false;
  }
  const auto& value = body.at(field);
  if (!value.is_number()) {
    *error = std::string("field '") + field + "' must be a number";
    return false;
  }
  const double parsed = value.get<double>();
  if (!std::isfinite(parsed)) {
    *error = std::string("field '") + field + "' must be finite";
    return false;
  }
  *out = parsed;
  return true;
}

bool parse_bounds_object(const nlohmann::json& object, Bounds2d* out, std::string* error) {
  if (!object.is_object()) {
    *error = "bounds must be a JSON object";
    return false;
  }
  const char* fields[4] = {"min_x_m", "min_y_m", "max_x_m", "max_y_m"};
  double values[4] = {0.0, 0.0, 0.0, 0.0};
  for (int i = 0; i < 4; ++i) {
    if (!parse_finite(object, fields[i], &values[i], error)) {
      return false;
    }
  }
  Bounds2d bounds{values[0], values[1], values[2], values[3]};
  if (bounds.empty()) {
    *error = "window is empty: min_x_m <= max_x_m and min_y_m <= max_y_m required";
    return false;
  }
  *out = bounds;
  return true;
}

bool looks_like_object_id(const std::string& id) {
  if (id.empty()) {
    return false;
  }
  return id.find_first_not_of("0123456789") == std::string::npos;
}

bool parse_body(const httplib::Request& req, nlohmann::json* body, std::string* error) {
  if (req.body.empty()) {
    *error = "request body is empty; expected a JSON object";
    return false;
  }
  try {
    *body = nlohmann::json::parse(req.body);
  } catch (const std::exception& parse_error) {
    *error = std::string("invalid JSON body: ") + parse_error.what();
    return false;
  }
  if (!body->is_object()) {
    *error = "request body must be a JSON object";
    return false;
  }
  return true;
}

std::string utc_now_iso8601() {
  const std::time_t now = std::time(nullptr);
  std::tm utc{};
  gmtime_r(&now, &utc);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &utc);
  return buffer;
}

// ---------------------------------------------------------------------------
// 处理器。
// ---------------------------------------------------------------------------

void handle_health(const ServiceState& state, httplib::Response& res) {
  nlohmann::json doc = envelope("health");
  doc["status"] = "ok";
  doc["map_id"] = state.info.map_id;
  doc["map_version"] = state.info.map_version;
  doc["uptime_s"] =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - state.started).count();
  // 协议 §3.5：版本一致性字段。
  doc["grid_protocol_version"] = kGridProtocolVersion;
  doc["scene_rules_version"] = kSceneRulesVersion;
  send_json(res, doc);
}

void handle_map_info(const ServiceState& state, httplib::Response& res) {
  send_json(res, map_info_json(state.info));
}

void handle_snapshot(const ServiceState& state, httplib::Response& res) {
  std::lock_guard<std::mutex> lock(state.mutex);
  try {
    if (!state.snapshot_cache.has_value()) {
      state.snapshot_cache = state.map->to_snapshot_json().dump();
    }
    res.status = 200;
    res.set_content(*state.snapshot_cache, "application/json; charset=utf-8");
  } catch (const std::exception& error) {
    send_error(res, 500, "invalid_input",
               std::string("snapshot generation failed: ") + error.what());
  }
}

void handle_query_point(const ServiceState& state, const httplib::Request& req,
                        httplib::Response& res) {
  nlohmann::json body;
  std::string error;
  if (!parse_body(req, &body, &error)) {
    send_error(res, 400, "invalid_input", error);
    return;
  }
  double x_m = 0.0;
  double y_m = 0.0;
  if (!parse_finite(body, "x_m", &x_m, &error) || !parse_finite(body, "y_m", &y_m, &error)) {
    send_error(res, 400, "invalid_input", error);
    return;
  }
  planner::map::PointQueryResult result;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    result = state.map->query_point(x_m, y_m);
  }
  nlohmann::json doc = envelope("point_query");
  doc["x_m"] = x_m;
  doc["y_m"] = y_m;
  doc["lanelets"] = summaries_json(result.lanelets);
  doc["areas"] = summaries_json(result.areas);
  send_json(res, doc);
}

void handle_query_bounds(const ServiceState& state, const httplib::Request& req,
                         httplib::Response& res) {
  nlohmann::json body;
  std::string error;
  if (!parse_body(req, &body, &error)) {
    send_error(res, 400, "invalid_input", error);
    return;
  }
  Bounds2d window;
  if (!parse_bounds_object(body, &window, &error)) {
    send_error(res, 400, "invalid_input", error);
    return;
  }
  planner::map::BoundsQueryResult result;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    result = state.map->query_bounds(window);
  }
  nlohmann::json doc = envelope("bounds_query");
  doc["window_m"] = bounds_json(window);
  doc["lanelets"] = summaries_json(result.lanelets);
  doc["areas"] = summaries_json(result.areas);
  send_json(res, doc);
}

void handle_lanelet_get(const ServiceState& state, const httplib::Request& req,
                        httplib::Response& res) {
  const std::string id = req.matches[1];
  if (!looks_like_object_id(id)) {
    send_error(res, 400, "invalid_input", "invalid lanelet id: " + id);
    return;
  }
  std::optional<LaneletGeometry> geometry;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    geometry = state.map->lanelet_geometry(id);
  }
  if (!geometry.has_value()) {
    send_error(res, 404, "invalid_input", "unknown lanelet id: " + id);
    return;
  }
  send_json(res, lanelet_geometry_json(*geometry));
}

void handle_lanelet_area_get(const ServiceState& state, const httplib::Request& req,
                             httplib::Response& res) {
  const std::string id = req.matches[1];
  if (!looks_like_object_id(id)) {
    send_error(res, 400, "invalid_input", "invalid area id: " + id);
    return;
  }
  std::optional<AreaGeometry> geometry;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    geometry = state.map->area_geometry(id);
  }
  if (!geometry.has_value()) {
    send_error(res, 404, "invalid_input", "unknown area id: " + id);
    return;
  }
  send_json(res, area_geometry_json(*geometry));
}

void handle_lanelet_relations(const ServiceState& state, const httplib::Request& req,
                              httplib::Response& res) {
  if (req.matches.size() <= 2) {
    send_error(res, 400, "invalid_input", "malformed lanelet relation request");
    return;
  }
  const std::string id = req.matches[1];
  const std::string relation = req.matches[2];
  if (!looks_like_object_id(id)) {
    send_error(res, 400, "invalid_input", "invalid lanelet id: " + id);
    return;
  }

  planner::map::IdListResult result;
  if (relation == "adjacent") {
    const std::string side = req.get_param_value("side");
    if (side != "left" && side != "right") {
      send_error(res, 400, "invalid_input",
                 "adjacent requires query parameter side=left|right");
      return;
    }
    const auto neighbor_side =
        side == "left" ? planner::map::NeighborSide::kLeft : planner::map::NeighborSide::kRight;
    std::lock_guard<std::mutex> lock(state.mutex);
    result = state.map->adjacent(id, neighbor_side);
  } else if (relation == "following") {
    std::lock_guard<std::mutex> lock(state.mutex);
    result = state.map->following(id);
  } else if (relation == "preceding") {
    std::lock_guard<std::mutex> lock(state.mutex);
    result = state.map->preceding(id);
  } else {
    send_error(res, 400, "invalid_input", "unknown relation: " + relation);
    return;
  }

  if (result.status != planner::common::Status::kSuccess) {
    const int http_status =
        result.reason.find("unknown lanelet id") != std::string::npos ? 404 : 500;
    send_error(res, http_status, "invalid_input", result.reason);
    return;
  }
  nlohmann::json doc = envelope("lanelet_relations");
  doc["lanelet_id"] = id;
  doc["relation"] = relation;
  if (relation == "adjacent") {
    doc["side"] = req.get_param_value("side");
  }
  doc["status"] = "success";
  doc["ids"] = result.ids;
  send_json(res, doc);
}

void handle_query_window(const ServiceState& state, const httplib::Request& req,
                         httplib::Response& res) {
  nlohmann::json body;
  std::string error;
  if (!parse_body(req, &body, &error)) {
    send_error(res, 400, "invalid_input", error);
    return;
  }
  Bounds2d window;
  if (body.contains("bounds")) {
    if (!parse_bounds_object(body.at("bounds"), &window, &error)) {
      send_error(res, 400, "invalid_input", error);
      return;
    }
  } else if (body.contains("center") && body.contains("size")) {
    const auto& center = body.at("center");
    const auto& size = body.at("size");
    double cx = 0.0;
    double cy = 0.0;
    double width = 0.0;
    double height = 0.0;
    if (!parse_finite(center, "x_m", &cx, &error) ||
        !parse_finite(center, "y_m", &cy, &error) ||
        !parse_finite(size, "width_m", &width, &error) ||
        !parse_finite(size, "height_m", &height, &error)) {
      send_error(res, 400, "invalid_input", error);
      return;
    }
    if (width <= 0.0 || height <= 0.0) {
      send_error(res, 400, "invalid_input", "window size must be positive");
      return;
    }
    window = Bounds2d{cx - width / 2.0, cy - height / 2.0, cx + width / 2.0,
                      cy + height / 2.0};
  } else {
    send_error(res, 400, "invalid_input",
               "expected {\"bounds\": {...}} or {\"center\": {...}, \"size\": {...}}");
    return;
  }

  CropResult crop;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    crop = state.map->crop_to_bounds(window);
  }
  send_json(res, crop_json(crop));
}

void handle_route(const ServiceState& state, const httplib::Request& req,
                  httplib::Response& res) {
  nlohmann::json body;
  std::string error;
  if (!parse_body(req, &body, &error)) {
    send_error(res, 400, "invalid_input", error);
    return;
  }
  const char* fields[2] = {"from_id", "to_id"};
  std::string ids[2];
  for (int i = 0; i < 2; ++i) {
    if (!body.contains(fields[i]) || !body.at(fields[i]).is_string()) {
      send_error(res, 400, "invalid_input",
                 std::string("field '") + fields[i] + "' must be a string object id");
      return;
    }
    ids[i] = body.at(fields[i]).get<std::string>();
    if (!looks_like_object_id(ids[i])) {
      send_error(res, 400, "invalid_input", std::string("invalid lanelet id: ") + ids[i]);
      return;
    }
  }

  planner::map::RouteResult result;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    result = state.map->route(ids[0], ids[1]);
  }
  if (result.status == planner::common::Status::kSuccess) {
    nlohmann::json doc = envelope("route_result");
    doc["from_id"] = ids[0];
    doc["to_id"] = ids[1];
    doc["status"] = "success";
    doc["lanelet_ids"] = result.lanelet_ids;
    doc["length_m"] = result.length_m;
    send_json(res, doc);
    return;
  }
  if (result.status == planner::common::Status::kNoPath) {
    send_error(res, 404, "no_path", result.reason);
    return;
  }
  if (result.reason.find("unknown lanelet id") != std::string::npos) {
    send_error(res, 404, "invalid_input", result.reason);
    return;
  }
  send_error(res, 500, "invalid_input", result.reason);
}

// ---------------------------------------------------------------------------
// 栅格/场景预览端点（栅格快照协议 v1 §3.2/§3.3；X-M1）。
//
// 请求为 JSON 对象（未知顶层字段容忍）；响应为 grid_snapshot 文档；错误
// 严格按协议 §8：400 invalid_input + 稳定 reason（parse_error、missing_field、
// type_error、non_finite_value、invalid_window、empty_window、invalid_resolution、
// invalid_scene、window_outside_map、grid_too_large、window_too_large），
// 500 internal_error。未使用 413。
// ---------------------------------------------------------------------------

#if defined(PLANNER_MAP_SERVICE_WITH_SCENE)

struct ProtocolError {
  bool ok{true};
  std::string reason;
  std::string detail;
};

ProtocolError protocol_error(const std::string& reason, const std::string& detail) {
  ProtocolError error;
  error.ok = false;
  error.reason = reason;
  error.detail = detail;
  return error;
}

bool parse_protocol_body(const httplib::Request& req, nlohmann::json* body,
                         ProtocolError* error) {
  if (req.body.empty()) {
    *error = protocol_error("parse_error", "request body is empty; expected a JSON object");
    return false;
  }
  try {
    *body = nlohmann::json::parse(req.body);
  } catch (const nlohmann::json::exception& parse_error) {
    // nlohmann 以 id 406（out_of_range，number overflow）拒绝超出 double 范围
    // 的数字字面量（如 1e999）；协议 §8 将其归入 non_finite_value，其余解析
    // 失败为 parse_error（裸 NaN 等非法字面量）。
    if (parse_error.id == 406) {
      *error = protocol_error("non_finite_value",
                              std::string("number literal out of range: ") +
                                  parse_error.what());
    } else {
      *error = protocol_error("parse_error",
                              std::string("invalid JSON body: ") + parse_error.what());
    }
    return false;
  } catch (const std::exception& parse_error) {
    *error = protocol_error("parse_error",
                            std::string("invalid JSON body: ") + parse_error.what());
    return false;
  }
  if (!body->is_object()) {
    *error = protocol_error("invalid_document", "request body must be a JSON object");
    return false;
  }
  return true;
}

bool parse_protocol_window(const nlohmann::json& body, Bounds2d* out,
                           ProtocolError* error) {
  if (!body.contains("window")) {
    *error = protocol_error("missing_field", "missing field 'window'");
    return false;
  }
  const auto& window = body.at("window");
  if (!window.is_object()) {
    *error = protocol_error("type_error", "field 'window' must be an object");
    return false;
  }
  const char* fields[4] = {"min_x_m", "min_y_m", "max_x_m", "max_y_m"};
  double values[4] = {0.0, 0.0, 0.0, 0.0};
  for (int i = 0; i < 4; ++i) {
    if (!window.contains(fields[i]) || !window.at(fields[i]).is_number()) {
      *error = protocol_error("invalid_window", std::string("window field '") +
                                                    fields[i] +
                                                    "' must be a finite number");
      return false;
    }
    values[i] = window.at(fields[i]).get<double>();
    if (!std::isfinite(values[i])) {
      *error = protocol_error("invalid_window",
                              std::string("window field '") + fields[i] + "' must be finite");
      return false;
    }
  }
  const Bounds2d bounds{values[0], values[1], values[2], values[3]};
  if (!(bounds.min_x_m < bounds.max_x_m) || !(bounds.min_y_m < bounds.max_y_m)) {
    *error = protocol_error(
        "empty_window",
        "window is empty: min_x_m < max_x_m and min_y_m < max_y_m required");
    return false;
  }
  *out = bounds;
  return true;
}

bool parse_protocol_resolution(const nlohmann::json& body, double* out,
                               ProtocolError* error) {
  if (!body.contains("resolution_m")) {
    *error = protocol_error("missing_field", "missing field 'resolution_m'");
    return false;
  }
  const auto& value = body.at("resolution_m");
  if (!value.is_number()) {
    *error = protocol_error("type_error", "field 'resolution_m' must be a number");
    return false;
  }
  const double parsed = value.get<double>();
  if (!std::isfinite(parsed)) {
    *error = protocol_error("non_finite_value", "field 'resolution_m' must be finite");
    return false;
  }
  if (!(parsed > 0.0)) {
    *error = protocol_error("invalid_resolution", "field 'resolution_m' must be > 0");
    return false;
  }
  *out = parsed;
  return true;
}

bool parse_protocol_request_seq(const nlohmann::json& body,
                                std::optional<long long>* out, ProtocolError* error) {
  if (!body.contains("request_seq")) {
    return true;
  }
  const auto& value = body.at("request_seq");
  if (!value.is_number_integer()) {
    *error = protocol_error("type_error", "field 'request_seq' must be an integer");
    return false;
  }
  const long long parsed = value.get<long long>();
  if (parsed < 0) {
    *error = protocol_error("invalid_value", "field 'request_seq' must be >= 0");
    return false;
  }
  *out = parsed;
  return true;
}

// 障碍 schema 与 /plan/submit 一致（协议 §3.3）：倒置矩形规范化、退化矩形
// 与非法多边形拒绝（invalid_scene）。
bool parse_protocol_obstacles(const nlohmann::json& body,
                              std::vector<planner::collision::Obstacle>* out,
                              ProtocolError* error) {
  if (!body.contains("obstacles")) {
    return true;
  }
  const auto& obstacles = body.at("obstacles");
  if (!obstacles.is_array()) {
    *error = protocol_error("type_error", "field 'obstacles' must be an array");
    return false;
  }
  for (const auto& value : obstacles) {
    if (!value.is_object()) {
      *error = protocol_error("type_error", "each obstacle must be a JSON object");
      return false;
    }
    if (!value.contains("id")) {
      *error = protocol_error("missing_field", "obstacle field 'id' is required");
      return false;
    }
    if (!value.at("id").is_string()) {
      *error = protocol_error("type_error", "obstacle field 'id' must be a string");
      return false;
    }
    const std::string id = value.at("id").get<std::string>();
    if (id.empty()) {
      *error = protocol_error("invalid_scene", "obstacle id must be non-empty");
      return false;
    }
    if (!value.contains("type")) {
      *error = protocol_error("missing_field", "obstacle field 'type' is required");
      return false;
    }
    if (!value.at("type").is_string()) {
      *error = protocol_error("type_error", "obstacle field 'type' must be a string");
      return false;
    }
    const std::string type = value.at("type").get<std::string>();
    if (type == "rectangle") {
      const char* fields[4] = {"x_min_m", "y_min_m", "x_max_m", "y_max_m"};
      double coords[4] = {0.0, 0.0, 0.0, 0.0};
      for (int i = 0; i < 4; ++i) {
        if (!value.contains(fields[i])) {
          *error = protocol_error("missing_field", "obstacle '" + id +
                                                        "' missing field '" + fields[i] + "'");
          return false;
        }
        if (!value.at(fields[i]).is_number()) {
          *error = protocol_error("type_error", "obstacle '" + id + "' field '" +
                                                    fields[i] + "' must be a number");
          return false;
        }
        coords[i] = value.at(fields[i]).get<double>();
        if (!std::isfinite(coords[i])) {
          *error = protocol_error("invalid_scene",
                                  "obstacle '" + id + "' has a non-finite coordinate");
          return false;
        }
      }
      // 倒置输入规范化（协议 §3.3）。
      const double x_min = std::min(coords[0], coords[2]);
      const double x_max = std::max(coords[0], coords[2]);
      const double y_min = std::min(coords[1], coords[3]);
      const double y_max = std::max(coords[1], coords[3]);
      if (!(x_min < x_max) || !(y_min < y_max)) {
        *error = protocol_error("invalid_scene",
                                "obstacle '" + id + "' is a degenerate rectangle");
        return false;
      }
      out->push_back(
          planner::collision::Obstacle::rectangle(id, x_min, y_min, x_max, y_max));
    } else if (type == "convex_polygon") {
      if (!value.contains("vertices")) {
        *error = protocol_error("missing_field",
                                "obstacle '" + id + "' missing field 'vertices'");
        return false;
      }
      const auto& vertices = value.at("vertices");
      if (!vertices.is_array()) {
        *error = protocol_error("type_error",
                                "obstacle '" + id + "' field 'vertices' must be an array");
        return false;
      }
      if (vertices.size() < 3U) {
        *error = protocol_error("invalid_scene",
                                "obstacle '" + id + "' needs at least 3 vertices");
        return false;
      }
      std::vector<Point2d> points;
      points.reserve(vertices.size());
      for (const auto& vertex : vertices) {
        if (!vertex.is_object() || !vertex.contains("x_m") || !vertex.contains("y_m")) {
          *error = protocol_error("invalid_scene", "obstacle '" + id +
                                                       "' vertices must be {x_m, y_m} objects");
          return false;
        }
        if (!vertex.at("x_m").is_number() || !vertex.at("y_m").is_number()) {
          *error = protocol_error("type_error", "obstacle '" + id +
                                                    "' vertex coordinates must be numbers");
          return false;
        }
        const double x_m = vertex.at("x_m").get<double>();
        const double y_m = vertex.at("y_m").get<double>();
        if (!std::isfinite(x_m) || !std::isfinite(y_m)) {
          *error = protocol_error("invalid_scene",
                                  "obstacle '" + id + "' has a non-finite vertex");
          return false;
        }
        points.push_back(Point2d{x_m, y_m});
      }
      const planner::collision::Obstacle validated =
          planner::collision::Obstacle::convex_polygon(id, points);
      std::string reason;
      if (!validated.valid(&reason)) {
        *error = protocol_error("invalid_scene", "obstacle '" + id + "': " + reason);
        return false;
      }
      out->push_back(validated);
    } else {
      *error = protocol_error("invalid_scene",
                              "obstacle '" + id + "' has unknown type '" + type + "'");
      return false;
    }
  }
  return true;
}

Bounds2d quantize_protocol_window(const Bounds2d& window) {
  Bounds2d out;
  out.min_x_m = planner::scene::quantize_scene_value(window.min_x_m);
  out.min_y_m = planner::scene::quantize_scene_value(window.min_y_m);
  out.max_x_m = planner::scene::quantize_scene_value(window.max_x_m);
  out.max_y_m = planner::scene::quantize_scene_value(window.max_y_m);
  return out;
}

// 栅格查询的稳定失败码映射（协议 §8；全部为 400 invalid_input）。
bool is_grid_failure_reason(const std::string& reason) {
  return reason == "invalid_window" || reason == "empty_window" ||
         reason == "invalid_resolution" || reason == "grid_too_large";
}

// 场景预览的稳定失败码映射（协议 §8）。
bool is_scene_failure_reason(const std::string& reason) {
  return reason == "window_outside_map" || reason == "window_too_large" ||
         reason == "invalid_scene" || reason == "invalid_window" ||
         reason == "empty_window";
}

void handle_grid_driveable(const ServiceState& state, const httplib::Request& req,
                           httplib::Response& res) {
  nlohmann::json body;
  ProtocolError error;
  if (!parse_protocol_body(req, &body, &error)) {
    send_error(res, 400, "invalid_input", error.reason);
    return;
  }
  Bounds2d window;
  double resolution = 0.0;
  std::optional<long long> request_seq;
  if (!parse_protocol_window(body, &window, &error) ||
      !parse_protocol_resolution(body, &resolution, &error) ||
      !parse_protocol_request_seq(body, &request_seq, &error)) {
    send_error(res, 400, "invalid_input", error.reason);
    return;
  }

  // 协议 §5.1/§4：量化窗口与请求分辨率，按单元预算降级（2 的幂）。
  window = quantize_protocol_window(window);
  if (!(window.min_x_m < window.max_x_m) || !(window.min_y_m < window.max_y_m)) {
    // 量化可能把低于 1e-9 的非空窗口归并为空（协议 §5.1 规则 4 的边界）。
    send_error(res, 400, "invalid_input", "empty_window");
    return;
  }
  resolution = planner::scene::quantize_scene_value(resolution);
  if (!(resolution > 0.0)) {
    // 低于 1e-9 的分辨率量化后为 0（协议 §5.1 规则 4 的边界）。
    send_error(res, 400, "invalid_input", "invalid_resolution");
    return;
  }
  const planner::grid::CellBudgetFit fit =
      planner::grid::fit_resolution_to_cell_budget(
          window.max_x_m - window.min_x_m, window.max_y_m - window.min_y_m, resolution,
          planner::map::kMaxDriveableGridCells);
  if (!fit.within_budget) {
    send_error(res, 400, "invalid_input", "grid_too_large");
    return;
  }

  planner::map::DriveableGridResult grid;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    grid = state.map->export_driveable_grid(window, fit.resolution_m);
  }
  if (!grid.ok()) {
    if (is_grid_failure_reason(grid.reason)) {
      send_error(res, 400, "invalid_input", grid.reason);
    } else {
      send_error(res, 500, "internal_error", "internal_error");
    }
    return;
  }

  planner::scene::GridSnapshotOptions options;
  options.source = "driveable";
  options.window = window;
  options.requested_resolution_m = resolution;
  options.budget_cells = planner::map::kMaxDriveableGridCells;
  options.request_seq = request_seq;
  send_json(res, planner::scene::make_grid_snapshot_json(grid.grid, options));
}

void handle_scene_preview(const ServiceState& state, const httplib::Request& req,
                          httplib::Response& res) {
  nlohmann::json body;
  ProtocolError error;
  if (!parse_protocol_body(req, &body, &error)) {
    send_error(res, 400, "invalid_input", error.reason);
    return;
  }
  Bounds2d window;
  double resolution = 0.0;
  std::optional<long long> request_seq;
  std::vector<planner::collision::Obstacle> obstacles;
  if (!parse_protocol_window(body, &window, &error) ||
      !parse_protocol_resolution(body, &resolution, &error) ||
      !parse_protocol_obstacles(body, &obstacles, &error) ||
      !parse_protocol_request_seq(body, &request_seq, &error)) {
    send_error(res, 400, "invalid_input", error.reason);
    return;
  }

  // 与规划服务同一 planner::scene 组合（协议 §3.3；窗口裁剪到地图边界、
  // 预算降级、可行驶域基础、障碍叠加、统一 scene_id）。
  const double quantized_resolution = planner::scene::quantize_scene_value(resolution);
  if (!(quantized_resolution > 0.0)) {
    send_error(res, 400, "invalid_input", "invalid_resolution");
    return;
  }
  planner::scene::ScenePreviewOptions options;
  options.window = window;
  options.resolution_m = quantized_resolution;
  options.max_cells = planner::map::kMaxDriveableGridCells;
  planner::scene::SceneBuildResult scene;
  {
    std::lock_guard<std::mutex> lock(state.mutex);
    scene = planner::scene::build_scene_preview(*state.map, obstacles, options);
  }
  if (!scene.ok) {
    if (is_scene_failure_reason(scene.reason)) {
      send_error(res, 400, "invalid_input", scene.reason);
    } else {
      send_error(res, 500, "internal_error", "internal_error");
    }
    return;
  }

  planner::scene::GridSnapshotOptions snapshot_options;
  snapshot_options.source = "planning_scene";
  snapshot_options.scene_id = scene.scene_id;
  snapshot_options.scene_version = scene.scene_version;
  snapshot_options.window = scene.window;
  snapshot_options.requested_resolution_m = scene.requested_resolution_m;
  snapshot_options.budget_cells = options.max_cells;
  snapshot_options.request_seq = request_seq;
  send_json(res, planner::scene::make_grid_snapshot_json(scene.grid, snapshot_options));
}

#else  // !PLANNER_MAP_SERVICE_WITH_SCENE

// 缩减构建（PLANNER_BUILD_SCENE=OFF）：栅格/场景端点需要 planner::scene，
// 在组件 CMake 中显式提示并在此返回稳定 500。
void handle_grid_driveable(const ServiceState&, const httplib::Request&,
                           httplib::Response& res) {
  send_error(res, 500, "internal_error", "internal_error");
}

void handle_scene_preview(const ServiceState&, const httplib::Request&,
                          httplib::Response& res) {
  send_error(res, 500, "internal_error", "internal_error");
}

#endif  // PLANNER_MAP_SERVICE_WITH_SCENE

void configure_server(httplib::Server& server, ServiceState& state) {
  server.Get("/health", [&state](const httplib::Request&, httplib::Response& res) {
    handle_health(state, res);
  });
  server.Get("/map/info", [&state](const httplib::Request&, httplib::Response& res) {
    handle_map_info(state, res);
  });
  server.Get("/map/snapshot", [&state](const httplib::Request&, httplib::Response& res) {
    handle_snapshot(state, res);
  });
  server.Post("/map/query/point",
              [&state](const httplib::Request& req, httplib::Response& res) {
                handle_query_point(state, req, res);
              });
  server.Post("/map/query/bounds",
              [&state](const httplib::Request& req, httplib::Response& res) {
                handle_query_bounds(state, req, res);
              });
  server.Get(R"(/map/lanelet/([^/]+)/(following|preceding|adjacent))",
             [&state](const httplib::Request& req, httplib::Response& res) {
               handle_lanelet_relations(state, req, res);
             });
  server.Get(R"(/map/lanelet/([^/]+))",
             [&state](const httplib::Request& req, httplib::Response& res) {
               handle_lanelet_get(state, req, res);
             });
  server.Get(R"(/map/area/([^/]+))",
             [&state](const httplib::Request& req, httplib::Response& res) {
               handle_lanelet_area_get(state, req, res);
             });
  server.Post("/map/query/window",
              [&state](const httplib::Request& req, httplib::Response& res) {
                handle_query_window(state, req, res);
              });
  server.Post("/map/route", [&state](const httplib::Request& req, httplib::Response& res) {
    handle_route(state, req, res);
  });
  // X-M1（协议 §3.2/§3.3）：栅格查询与场景预览。
  server.Post("/map/grid/driveable",
              [&state](const httplib::Request& req, httplib::Response& res) {
                handle_grid_driveable(state, req, res);
              });
  server.Post("/map/scene/preview",
              [&state](const httplib::Request& req, httplib::Response& res) {
                handle_scene_preview(state, req, res);
              });

  server.set_error_handler([](const httplib::Request& req, httplib::Response& res) {
    if (!res.body.empty()) {
      return;  // 处理器已生成结构化错误
    }
    nlohmann::json doc = envelope("error");
    doc["status"] = "invalid_input";
    doc["reason"] = "no handler for " + req.method + " " + req.path;
    res.set_content(doc.dump(), "application/json; charset=utf-8");
  });
  server.set_exception_handler(
      [](const httplib::Request&, httplib::Response& res, std::exception_ptr error) {
        std::string reason = "internal error";
        try {
          if (error) {
            std::rethrow_exception(error);
          }
        } catch (const std::exception& caught) {
          reason = std::string("internal error: ") + caught.what();
        } catch (...) {
        }
        send_error(res, 500, "invalid_input", reason);
      });
  if (!state.quiet) {
    server.set_logger([](const httplib::Request& req, const httplib::Response& res) {
      std::cout << req.method << " " << req.path << " -> " << res.status << std::endl;
    });
  }
}

void write_service_status(const ServiceState& state, const std::string& host, int port) {
  if (state.run_dir.empty()) {
    return;
  }
  std::error_code ec;
  std::filesystem::create_directories(state.run_dir, ec);
  nlohmann::json doc = envelope("map_service_status");
  doc["status"] = "listening";
  doc["host"] = host;
  doc["port"] = port;
  doc["map_id"] = state.info.map_id;
  doc["map_version"] = state.info.map_version;
  doc["started_at_utc"] = utc_now_iso8601();
  std::ofstream out(std::filesystem::path(state.run_dir) / "service_status.json");
  out << doc.dump(2) << '\n';
}

void print_usage(std::ostream& os) {
  os << "Usage: planner_map_service --map <map.osm> --origin-lat <deg> --origin-lon <deg>\n"
        "       [--port <port>] [--host <addr>] [--projection utm|mercator|spherical_mercator]\n"
        "       [--origin-ele <m>] [--traffic-rules-location de] [--traffic-rules-participant vehicle]\n"
        "       [--no-validation] [--run-dir <dir>] [--quiet]\n"
        "\n"
        "Serves the map JSON API on http://<host>:<port>/ (port 0 picks a free port\n"
        "and prints the actual value). Default port: 8765.\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string map_path;
  std::string host = kDefaultHost;
  std::string run_dir;
  int port = kDefaultPort;
  MapLoadOptions options;
  options.projection = ProjectionKind::kUtm;
  bool origin_set = false;
  GeoPoint origin;
  bool quiet = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto next = [&](const char* flag) -> std::optional<std::string> {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << flag << "\n";
        return std::nullopt;
      }
      return std::string(argv[++i]);
    };
    if (arg == "--map") {
      const auto value = next("--map");
      if (!value) return 2;
      map_path = *value;
    } else if (arg == "--origin-lat" || arg == "--origin-lon" || arg == "--origin-ele") {
      const auto value = next(arg.c_str());
      if (!value) return 2;
      try {
        const double parsed = std::stod(*value);
        if (arg == "--origin-lat") {
          origin.lat_deg = parsed;
        } else if (arg == "--origin-lon") {
          origin.lon_deg = parsed;
        } else {
          origin.ele_m = parsed;
        }
        origin_set = true;
      } catch (const std::exception&) {
        std::cerr << "invalid value for " << arg << "\n";
        return 2;
      }
    } else if (arg == "--port") {
      const auto value = next("--port");
      if (!value) return 2;
      try {
        port = std::stoi(*value);
      } catch (const std::exception&) {
        std::cerr << "invalid port: " << *value << "\n";
        return 2;
      }
      if (port < 0 || port > 65535) {
        std::cerr << "port out of range: " << port << "\n";
        return 2;
      }
    } else if (arg == "--host") {
      const auto value = next("--host");
      if (!value) return 2;
      host = *value;
    } else if (arg == "--projection") {
      const auto value = next("--projection");
      if (!value) return 2;
      const auto kind = planner::map::projection_kind_from_string(*value);
      if (!kind) {
        std::cerr << "unknown projection: " << *value << "\n";
        return 2;
      }
      options.projection = *kind;
    } else if (arg == "--traffic-rules-location") {
      const auto value = next("--traffic-rules-location");
      if (!value) return 2;
      options.traffic_rules_location = *value;
    } else if (arg == "--traffic-rules-participant") {
      const auto value = next("--traffic-rules-participant");
      if (!value) return 2;
      options.traffic_rules_participant = *value;
    } else if (arg == "--no-validation") {
      options.validate_on_import = false;
    } else if (arg == "--run-dir") {
      const auto value = next("--run-dir");
      if (!value) return 2;
      run_dir = *value;
    } else if (arg == "--quiet") {
      quiet = true;
    } else if (arg == "--help" || arg == "-h") {
      print_usage(std::cout);
      return 0;
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      print_usage(std::cerr);
      return 2;
    }
  }

  if (map_path.empty() || !origin_set) {
    std::cerr << "--map, --origin-lat and --origin-lon are required\n";
    print_usage(std::cerr);
    return 2;
  }
  options.origin = origin;

  auto import = Map::import_osm_file(map_path, options);
  if (!import.ok()) {
    std::cerr << "map import failed: " << import.reason << "\n";
    return 1;
  }

  auto state = std::make_shared<ServiceState>();
  state->map = std::move(import.map);
  state->info = state->map->info();
  state->started = std::chrono::steady_clock::now();
  state->run_dir = run_dir;
  state->quiet = quiet;

  httplib::Server server;
  g_server = &server;
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);
  configure_server(server, *state);

  int bound_port = port;
  if (port == 0) {
    bound_port = server.bind_to_any_port(host);
    if (bound_port <= 0) {
      std::cerr << "cannot bind an ephemeral port on " << host << "\n";
      return 3;
    }
  } else if (!server.bind_to_port(host, port)) {
    std::cerr << "cannot bind " << host << ":" << port << " (already in use?)\n";
    return 3;
  }

  std::cout << "planner_map_service listening on " << host << ":" << bound_port << std::endl;
  write_service_status(*state, host, bound_port);

  const bool listen_ok = server.listen_after_bind();
  std::cout << "planner_map_service stopped (listen_result=" << (listen_ok ? "ok" : "failed")
            << ", requested=" << (g_stop_requested ? "signal" : "none") << ")" << std::endl;
  return (listen_ok || g_stop_requested) ? 0 : 1;
}
