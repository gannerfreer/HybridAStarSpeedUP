// planner_collision_check_scene: 命令行碰撞检测示例。
//
// 加载 JSON 场景文档并运行位姿 / 运动 / 路径检测，每次检测打印一条 JSON 结果。
// 场景 schema（所有坐标单位为米）：
//
// {
//   "schema_version": 1,
//   "type": "collision_scene",
//   "vehicle": { ...VehicleModel 载荷... },
//   "grid": { ...GridView 文档... },              // 可选
//   "obstacles": [                                 // 可选
//     {"id":"a","type":"rectangle","x_min_m":0,"y_min_m":0,"x_max_m":1,"y_max_m":1},
//     {"id":"b","type":"convex_polygon","vertices":[{"x_m":0,"y_m":0}, ...]}
//   ],
//   "checks": [
//     {"kind":"pose","pose":{"x_m":0,"y_m":0,"yaw_rad":0}},
//     {"kind":"motion","start":{...},"length_m":1.0,"direction":"forward",
//      "curvature_inv_m":0.0},
//     {"kind":"path","segments":[{"points":[{"pose":{...},"s_m":0.0,
//      "curvature_inv_m":0.0,"direction":"forward"}, ...]}]}
//   ]
// }
//
// 退出码：所有检测均无障碍时为 0（若仅报告了无效输入呢？不：为 1）；至少一项
// 检测碰撞时为 1；用法或输入错误时为 2。传入 --demo 运行内置场景。

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "planner/collision/collision_checker.hpp"
#include "planner/common/serialization.hpp"
#include "planner/common/types.hpp"

namespace {

using planner::collision::CheckOptions;
using planner::collision::CheckResult;
using planner::collision::CheckStatus;
using planner::collision::CollisionChecker;
using planner::collision::CollisionReason;
using planner::collision::Scene;
using planner::common::Direction;
using planner::common::PathPoint;
using planner::common::PathSegment;
using planner::common::Point2d;
using planner::common::Pose2d;

Pose2d parse_pose(const nlohmann::json& json) {
  Pose2d pose;
  pose.x_m = json.at("x_m").get<double>();
  pose.y_m = json.at("y_m").get<double>();
  pose.yaw_rad = json.value("yaw_rad", 0.0);
  return pose;
}

Direction parse_direction(const std::string& text) {
  if (text == "forward") return Direction::kForward;
  if (text == "reverse") return Direction::kReverse;
  throw std::runtime_error("unknown direction: " + text);
}

nlohmann::json result_json(const std::string& kind, const CheckResult& result) {
  nlohmann::json out;
  out["kind"] = kind;
  out["status"] = planner::collision::to_string(result.status);
  out["reason"] = planner::collision::to_string(result.reason_code);
  out["detail"] = result.detail;
  out["first_pose"] = {{"x_m", result.first_pose.x_m},
                       {"y_m", result.first_pose.y_m},
                       {"yaw_rad", result.first_pose.yaw_rad}};
  out["first_s_m"] = result.first_s_m;
  if (result.path_segment_index >= 0) {
    out["path_segment_index"] = result.path_segment_index;
  }
  return out;
}

int run_scene(const nlohmann::json& document, bool acceleration) {
  Scene scene;
  if (document.contains("grid") && !document.at("grid").is_null()) {
    scene.grid =
        planner::common::decode_document<planner::common::GridView>(
            document.at("grid").dump());
  }
  if (document.contains("obstacles")) {
    for (const auto& entry : document.at("obstacles")) {
      const std::string id = entry.value("id", std::string("obstacle"));
      const std::string type = entry.value("type", std::string("rectangle"));
      if (type == "rectangle") {
        scene.obstacles.push_back(planner::collision::Obstacle::rectangle(
            id, entry.at("x_min_m").get<double>(),
            entry.at("y_min_m").get<double>(),
            entry.at("x_max_m").get<double>(),
            entry.at("y_max_m").get<double>()));
      } else if (type == "convex_polygon") {
        std::vector<Point2d> vertices;
        for (const auto& vertex : entry.at("vertices")) {
          vertices.push_back(Point2d{vertex.at("x_m").get<double>(),
                                     vertex.at("y_m").get<double>()});
        }
        scene.obstacles.push_back(
            planner::collision::Obstacle::convex_polygon(id,
                                                         std::move(vertices)));
      } else {
        std::cerr << "unknown obstacle type: " << type << "\n";
        return 2;
      }
    }
  }
  const auto vehicle = planner::common::decode_payload<
      planner::common::VehicleModel>(document.at("vehicle"));
  CheckOptions options;
  options.enable_acceleration = acceleration;
  CollisionChecker checker(vehicle, scene, options);
  if (!checker.valid()) {
    std::cerr << "invalid scene: " << checker.invalid_reason() << "\n";
    return 2;
  }

  bool any_collision = false;
  bool any_invalid = false;
  nlohmann::json results = nlohmann::json::array();
  for (const auto& check : document.at("checks")) {
    const std::string kind = check.at("kind").get<std::string>();
    if (kind == "pose") {
      const CheckResult result = checker.check_pose(parse_pose(check.at("pose")));
      any_collision = any_collision || result.collided();
      any_invalid = any_invalid || (result.status == CheckStatus::kInvalidInput);
      results.push_back(result_json(kind, result));
    } else if (kind == "motion") {
      planner::collision::MotionSegment motion;
      motion.start = parse_pose(check.at("start"));
      motion.length_m = check.at("length_m").get<double>();
      motion.direction =
          parse_direction(check.value("direction", std::string("forward")));
      motion.curvature_inv_m = check.value("curvature_inv_m", 0.0);
      const CheckResult result = checker.check_motion(motion);
      any_collision = any_collision || result.collided();
      any_invalid = any_invalid || (result.status == CheckStatus::kInvalidInput);
      results.push_back(result_json(kind, result));
    } else if (kind == "path") {
      std::vector<PathSegment> path;
      for (const auto& segment_json : check.at("segments")) {
        PathSegment segment;
        for (const auto& point_json : segment_json.at("points")) {
          PathPoint point;
          point.pose = parse_pose(point_json.at("pose"));
          point.s_m = point_json.value("s_m", 0.0);
          point.curvature_inv_m = point_json.value("curvature_inv_m", 0.0);
          point.direction = parse_direction(
              point_json.value("direction", std::string("forward")));
          segment.points.push_back(point);
        }
        path.push_back(std::move(segment));
      }
      const CheckResult result = checker.check_path(path);
      any_collision = any_collision || result.collided();
      any_invalid = any_invalid || (result.status == CheckStatus::kInvalidInput);
      results.push_back(result_json(kind, result));
    } else {
      std::cerr << "unknown check kind: " << kind << "\n";
      return 2;
    }
  }
  nlohmann::json output;
  output["type"] = "collision_check_results";
  output["acceleration"] = acceleration;
  output["results"] = results;
  output["any_collision"] = any_collision;
  output["any_invalid_input"] = any_invalid;
  std::cout << output.dump(2) << "\n";
  if (any_invalid) return 2;
  return any_collision ? 1 : 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string scene_path;
  bool acceleration = false;
  bool demo = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--scene" && i + 1 < argc) {
      scene_path = argv[++i];
    } else if (arg == "--accel") {
      acceleration = true;
    } else if (arg == "--demo") {
      demo = true;
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "usage: planner_collision_check_scene --scene FILE "
                   "[--accel] | --demo\n";
      return 0;
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return 2;
    }
  }
  if (!demo && scene_path.empty()) {
    std::cerr << "need --scene FILE or --demo\n";
    return 2;
  }
  if (demo) {
    nlohmann::json document = {
        {"schema_version", 1},
        {"type", "collision_scene"},
        {"vehicle",
         {{"id", "default_test_vehicle"},
          {"wheelbase_m", 2.8},
          {"width_m", 1.8},
          {"front_overhang_m", 0.9},
          {"rear_overhang_m", 1.0},
          {"max_steer_rad", 0.5},
          {"safety_margin_m", 0.1}}},
        {"grid",
         {{"schema_version", 1},
          {"type", "grid_view"},
          {"map_id", "demo_grid"},
          {"map_version", "v1"},
          {"width", 200},
          {"height", 100},
          {"resolution_m", 0.1},
          {"origin", {{"x_m", 0.0}, {"y_m", 0.0}, {"yaw_rad", 0.0}}},
          {"cells", std::vector<int>(200 * 100, 0)}}},
        {"obstacles",
         nlohmann::json::array(
             {{{"id", "block"}, {"type", "rectangle"}, {"x_min_m", 8.0},
               {"y_min_m", 2.0}, {"x_max_m", 9.0}, {"y_max_m", 4.0}}})},
        {"checks",
         nlohmann::json::array(
             {{{"kind", "pose"},
               {"pose", {{"x_m", 2.0}, {"y_m", 2.0}, {"yaw_rad", 0.0}}}},
              {{"kind", "motion"},
               {"start", {{"x_m", 2.0}, {"y_m", 2.0}, {"yaw_rad", 0.0}}},
               {"length_m", 5.0},
               {"direction", "forward"},
               {"curvature_inv_m", 0.0}}})}};
    return run_scene(document, acceleration);
  }
  std::ifstream stream(scene_path);
  if (!stream) {
    std::cerr << "cannot open scene file: " << scene_path << "\n";
    return 2;
  }
  try {
    nlohmann::json document;
    stream >> document;
    if (document.value("schema_version", 0) != 1 ||
        document.value("type", std::string()) != "collision_scene") {
      std::cerr << "not a v1 collision_scene document\n";
      return 2;
    }
    return run_scene(document, acceleration);
  } catch (const std::exception& error) {
    std::cerr << "input error: " << error.what() << "\n";
    return 2;
  }
}
