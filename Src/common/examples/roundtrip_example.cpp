// planner_common 命令行往返示例（E0）。
//
// 模式：
//   common_roundtrip_example --self-check
//       构造每种第一版公开类型的内存样本，编码为独立文档，
//       再解码并比较。
//   common_roundtrip_example --list-types
//       打印可接受的文档类型名称。
//   common_roundtrip_example <type> <input.json> [output.json]
//       解码 <input.json>，重新编码并验证往返无损。
//       未提供 [output.json] 时，规范 JSON 打印到 stdout；
//       提供时写入该文件（UTF-8，末尾换行）。
//
// 退出码：0 成功，1 往返不一致，2 序列化错误，
// 3 文件 I/O 错误，4 用法错误。
//
// E0 阶段交付本示例但不运行；稍后由协调者通过任务入口运行。

#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>

#include "planner/common/errors.hpp"
#include "planner/common/serialization.hpp"
#include "planner/common/types.hpp"

namespace {

using planner::common::Corridor;
using planner::common::CorridorPolygon;
using planner::common::Direction;
using planner::common::GridView;
using planner::common::PathPoint;
using planner::common::PathSegment;
using planner::common::PlanRequest;
using planner::common::PlanResult;
using planner::common::Point2d;
using planner::common::Pose2d;
using planner::common::SerializationError;
using planner::common::Status;
using planner::common::TrajectoryPoint;
using planner::common::VehicleModel;
using planner::common::decode_document;
using planner::common::encode_document;
using planner::common::to_string;

constexpr const char* kExampleUsage =
    "usage:\n"
    "  common_roundtrip_example --self-check\n"
    "  common_roundtrip_example --list-types\n"
    "  common_roundtrip_example <type> <input.json> [output.json]\n";

Pose2d make_pose(double x_m, double y_m, double yaw_rad) {
  Pose2d pose;
  pose.x_m = x_m;
  pose.y_m = y_m;
  pose.yaw_rad = yaw_rad;
  return pose;
}

VehicleModel make_vehicle() {
  VehicleModel vehicle;
  vehicle.id = "example_vehicle";
  vehicle.wheelbase_m = 2.8;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.9;
  vehicle.rear_overhang_m = 1.0;
  vehicle.max_steer_rad = 0.5;
  vehicle.safety_margin_m = 0.1;
  return vehicle;
}

PathPoint make_path_point(double x_m, double y_m, double yaw_rad, double s_m, double curvature_inv_m,
                          Direction direction) {
  PathPoint point;
  point.pose = make_pose(x_m, y_m, yaw_rad);
  point.s_m = s_m;
  point.curvature_inv_m = curvature_inv_m;
  point.direction = direction;
  return point;
}

PathSegment make_forward_segment() {
  PathSegment segment;
  segment.points.push_back(make_path_point(0.0, 0.0, 0.0, 0.0, 0.0, Direction::kForward));
  segment.points.push_back(make_path_point(1.0, 0.0, 0.0, 1.0, 0.0, Direction::kForward));
  segment.points.push_back(make_path_point(2.0, 0.5, 0.5, 2.1, 0.195108, Direction::kForward));
  return segment;
}

PathSegment make_reverse_segment() {
  PathSegment segment;
  segment.points.push_back(make_path_point(2.0, 0.5, 0.5, 2.1, 0.0, Direction::kReverse));
  segment.points.push_back(make_path_point(1.5, 0.5, 0.5, 2.6, 0.0, Direction::kReverse));
  return segment;
}

PlanRequest make_plan_request() {
  PlanRequest request;
  request.request_id = "example_request";
  request.map_id = "example_map";
  request.map_version = "v1";
  request.vehicle = make_vehicle();
  request.start = make_pose(0.0, 0.0, 0.0);
  request.goal = make_pose(10.0, 5.0, 1.5707963267948966);
  request.config = nlohmann::json{{"planner", {{"heuristic", "euclidean"}, {"weight", 1.5}}}};
  request.time_budget_s = 5.0;
  request.node_budget = 1000000;
  request.cancelled = false;
  return request;
}

PlanResult make_plan_result() {
  PlanResult result;
  result.request_id = "example_request";
  result.status = Status::kSuccess;
  result.reason = "goal_reached";
  result.path.push_back(make_forward_segment());
  result.path.push_back(make_reverse_segment());

  TrajectoryPoint first;
  first.t_s = 0.0;
  first.pose = make_pose(0.0, 0.0, 0.0);
  first.velocity_mps = 0.0;
  first.acceleration_mps2 = 0.5;
  first.curvature_inv_m = 0.0;
  result.trajectory.push_back(first);

  TrajectoryPoint second;
  second.t_s = 0.5;
  second.pose = make_pose(0.25, 0.0, 0.0);
  second.velocity_mps = 0.5;
  second.acceleration_mps2 = 0.0;
  second.curvature_inv_m = 0.0;
  result.trajectory.push_back(second);

  result.statistics = nlohmann::json{{"expanded_nodes", 42}, {"planning_time_s", 0.0123}};
  return result;
}

Corridor make_corridor() {
  Corridor corridor;
  corridor.id = "example_corridor";
  corridor.map_id = "example_map";
  corridor.map_version = "v1";
  corridor.frame_id = "map";

  CorridorPolygon first;
  first.s_start_m = 0.0;
  first.s_end_m = 2.0;
  first.vertices = {Point2d{-1.0, -1.0}, Point2d{3.0, -1.0}, Point2d{3.0, 1.0}, Point2d{-1.0, 1.0}};
  corridor.polygons.push_back(first);

  CorridorPolygon second;
  second.s_start_m = 2.0;
  second.s_end_m = 4.0;
  second.vertices = {Point2d{1.0, -1.0}, Point2d{5.0, -1.0}, Point2d{5.0, 1.0}, Point2d{1.0, 1.0}};
  corridor.polygons.push_back(second);
  return corridor;
}

bool read_file(const std::string& path, std::string* out) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    std::cerr << "cannot open input file: " << path << "\n";
    return false;
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  if (stream.bad()) {
    std::cerr << "error while reading input file: " << path << "\n";
    return false;
  }
  *out = buffer.str();
  return true;
}

template <typename T>
bool check_roundtrip(const char* name, const T& value) {
  try {
    const std::string encoded = encode_document(value);
    const T decoded = decode_document<T>(encoded);
    if (!(decoded == value)) {
      std::cerr << name << ": round-trip mismatch\n";
      return false;
    }
    std::cout << name << ": ok\n";
    return true;
  } catch (const SerializationError& error) {
    std::cerr << name << ": error [" << to_string(error.code()) << "]: " << error.what() << "\n";
    return false;
  }
}

int run_self_check() {
  bool ok = true;
  ok = check_roundtrip("pose2d", make_pose(1.0, -2.0, 0.5)) && ok;
  ok = check_roundtrip("vehicle_model", make_vehicle()) && ok;
  ok = check_roundtrip("status", Status::kSuccess) && ok;
  ok = check_roundtrip("path_point", make_path_point(1.0, 2.0, 0.25, 1.0, 0.1, Direction::kReverse)) && ok;
  ok = check_roundtrip("path_segment", make_forward_segment()) && ok;
  ok = check_roundtrip("trajectory_point", make_plan_result().trajectory.front()) && ok;
  ok = check_roundtrip("plan_request", make_plan_request()) && ok;
  ok = check_roundtrip("plan_result", make_plan_result()) && ok;
  ok = check_roundtrip("corridor", make_corridor()) && ok;
  return ok ? 0 : 1;
}

void print_type_list() {
  std::cout << "pose2d\n"
               "vehicle_model\n"
               "grid_view\n"
               "path_point\n"
               "path_segment\n"
               "trajectory_point\n"
               "plan_request\n"
               "plan_result\n"
               "status\n"
               "corridor\n";
}

template <typename T>
int roundtrip_file(const std::string& input_path, const std::optional<std::string>& output_path) {
  std::string text;
  if (!read_file(input_path, &text)) {
    return 3;
  }
  try {
    const T value = decode_document<T>(text);
    const std::string encoded = encode_document(value);
    const T decoded = decode_document<T>(encoded);
    if (!(decoded == value)) {
      std::cerr << "round-trip mismatch for " << input_path << "\n";
      return 1;
    }
    if (output_path.has_value()) {
      std::ofstream stream(*output_path, std::ios::binary | std::ios::trunc);
      if (!stream) {
        std::cerr << "cannot write output file: " << *output_path << "\n";
        return 3;
      }
      stream << encoded << '\n';
      if (!stream) {
        std::cerr << "error while writing output file: " << *output_path << "\n";
        return 3;
      }
      std::cout << "ok: " << input_path << " -> " << *output_path << "\n";
    } else {
      std::cout << encoded << '\n';
    }
    return 0;
  } catch (const SerializationError& error) {
    std::cerr << input_path << ": serialization error [" << to_string(error.code())
              << "]: " << error.what() << "\n";
    return 2;
  }
}

int run_named_type(const std::string& type, const std::string& input_path,
                   const std::optional<std::string>& output_path) {
  if (type == "pose2d") {
    return roundtrip_file<planner::common::Pose2d>(input_path, output_path);
  }
  if (type == "vehicle_model") {
    return roundtrip_file<planner::common::VehicleModel>(input_path, output_path);
  }
  if (type == "grid_view") {
    return roundtrip_file<planner::common::GridView>(input_path, output_path);
  }
  if (type == "path_point") {
    return roundtrip_file<planner::common::PathPoint>(input_path, output_path);
  }
  if (type == "path_segment") {
    return roundtrip_file<planner::common::PathSegment>(input_path, output_path);
  }
  if (type == "trajectory_point") {
    return roundtrip_file<planner::common::TrajectoryPoint>(input_path, output_path);
  }
  if (type == "plan_request") {
    return roundtrip_file<planner::common::PlanRequest>(input_path, output_path);
  }
  if (type == "plan_result") {
    return roundtrip_file<planner::common::PlanResult>(input_path, output_path);
  }
  if (type == "status") {
    return roundtrip_file<planner::common::Status>(input_path, output_path);
  }
  if (type == "corridor") {
    return roundtrip_file<planner::common::Corridor>(input_path, output_path);
  }
  std::cerr << "unknown type '" << type << "'; use --list-types\n";
  return 4;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << kExampleUsage;
    return 4;
  }
  const std::string first = argv[1];
  if (first == "--help" || first == "-h") {
    std::cout << kExampleUsage;
    return 0;
  }
  if (first == "--list-types") {
    print_type_list();
    return 0;
  }
  if (first == "--self-check") {
    return run_self_check();
  }
  if (argc != 3 && argc != 4) {
    std::cerr << kExampleUsage;
    return 4;
  }
  std::optional<std::string> output_path;
  if (argc == 4) {
    output_path = argv[3];
  }
  return run_named_type(first, argv[2], output_path);
}
