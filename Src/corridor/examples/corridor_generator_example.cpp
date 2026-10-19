// planner_corridor 命令行示例。
//
// 模式：
//   planner_corridor_example --list-scenarios
//       打印内置场景名称与描述。
//   planner_corridor_example --scenario <name> [--output <file>]
//       在内存中构建场景路径种子与障碍物，运行走廊生成器并把第一版
//       `corridor` JSON 文档（planner::common::encode_document）打印到
//       stdout；给出 --output 时写入 <file>。生成显式失败时，把
//       status/reason/detail 写到 stderr，退出码为 1。
//   planner_corridor_example --self-check
//       运行每个场景，检查预期的成功/失败结果；对成功的场景重新验证走廊
//       并对 JSON 文档做往返编解码。
//
// 退出码：0 成功 / 所有自检通过，1 生成失败，
// 2 用法错误，3 文件 I/O 错误。

#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "planner/common/serialization.hpp"
#include "planner/common/types.hpp"
#include "planner/corridor/corridor_generator.hpp"
#include "planner/corridor/geometry.hpp"

namespace {

using planner::common::Corridor;
using planner::common::Direction;
using planner::common::PathPoint;
using planner::common::PathSegment;
using planner::common::Pose2d;
using planner::common::VehicleModel;
using planner::corridor::CorridorRequest;
using planner::corridor::CorridorResult;
using planner::corridor::CorridorVerification;
using planner::corridor::Obstacle;
using planner::corridor::make_rectangle_obstacle;

const char* kUsage =
    "usage:\n"
    "  planner_corridor_example --list-scenarios\n"
    "  planner_corridor_example --scenario <name> [--output <file>]\n"
    "  planner_corridor_example --self-check\n";

struct ScenarioInput {
  std::vector<PathSegment> seed;
  std::vector<Obstacle> obstacles;
};

struct Scenario {
  const char* name;
  const char* description;
  bool expect_success;
  ScenarioInput (*build)();
};

PathPoint make_point(double x_m, double y_m, double yaw_rad, double s_m,
                     Direction direction = Direction::kForward) {
  PathPoint point;
  point.pose = Pose2d{x_m, y_m, yaw_rad};
  point.s_m = s_m;
  point.curvature_inv_m = 0.0;
  point.direction = direction;
  return point;
}

// 沿 +x 方向的直线种子，航向恒定。
PathSegment make_straight_segment(double x_start, double x_end, double y_m, double yaw_rad,
                                  double step_m = 0.5) {
  PathSegment segment;
  const double length = std::fabs(x_end - x_start);
  std::size_t steps = static_cast<std::size_t>(std::ceil(length / step_m));
  if (steps < 1) {
    steps = 1;
  }
  for (std::size_t k = 0; k <= steps; ++k) {
    const double t = static_cast<double>(k) / static_cast<double>(steps);
    const double x = x_start + (x_end - x_start) * t;
    segment.points.push_back(make_point(x, y_m, yaw_rad, length * t));
  }
  return segment;
}

// 以原点为中心的圆弧；正扫角为逆时针，车辆航向沿切线方向。
PathSegment make_arc_segment(double radius_m, double start_angle_rad, double sweep_rad,
                             double step_m = 0.25) {
  PathSegment segment;
  const double arc_length = radius_m * std::fabs(sweep_rad);
  std::size_t steps = static_cast<std::size_t>(std::ceil(arc_length / step_m));
  if (steps < 1) {
    steps = 1;
  }
  for (std::size_t k = 0; k <= steps; ++k) {
    const double t = static_cast<double>(k) / static_cast<double>(steps);
    const double phi = start_angle_rad + sweep_rad * t;
    const double tangent = sweep_rad >= 0.0
                               ? phi + 0.5 * planner::common::kPi
                               : phi - 0.5 * planner::common::kPi;
    segment.points.push_back(make_point(
        radius_m * std::cos(phi), radius_m * std::sin(phi),
        planner::common::normalize_angle_rad(tangent), arc_length * t));
  }
  return segment;
}

VehicleModel default_vehicle() {
  VehicleModel vehicle;
  vehicle.id = "default_test_vehicle";
  vehicle.wheelbase_m = 2.8;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.9;
  vehicle.rear_overhang_m = 1.0;
  vehicle.max_steer_rad = 0.5;
  vehicle.safety_margin_m = 0.1;
  return vehicle;
}

ScenarioInput build_straight() {
  ScenarioInput input;
  input.seed.push_back(make_straight_segment(0.0, 20.0, 5.0, 0.0));
  return input;
}

ScenarioInput build_arc() {
  ScenarioInput input;
  input.seed.push_back(
      make_arc_segment(8.0, -0.5 * planner::common::kPi, 0.5 * planner::common::kPi));
  return input;
}

ScenarioInput build_channel() {
  ScenarioInput input;
  input.seed.push_back(make_straight_segment(0.0, 22.0, 5.0, 0.0));
  input.obstacles.push_back(make_rectangle_obstacle("wall_lower", 6.0, 0.0, 16.0, 3.5));
  input.obstacles.push_back(make_rectangle_obstacle("wall_upper", 6.0, 6.5, 16.0, 10.0));
  return input;
}

ScenarioInput build_blocked() {
  ScenarioInput input;
  input.seed.push_back(make_straight_segment(0.0, 20.0, 5.0, 0.0));
  input.obstacles.push_back(make_rectangle_obstacle("blocker", 8.0, 4.0, 12.0, 6.0));
  return input;
}

ScenarioInput build_narrow_slit() {
  ScenarioInput input;
  input.seed.push_back(make_straight_segment(0.0, 20.0, 5.0, 0.0));
  input.obstacles.push_back(make_rectangle_obstacle("slit_lower", 8.0, 0.0, 10.0, 4.25));
  input.obstacles.push_back(make_rectangle_obstacle("slit_upper", 8.0, 5.75, 10.0, 11.0));
  return input;
}

const Scenario kScenarios[] = {
    {"straight", "20 m straight seed, no obstacles", true, &build_straight},
    {"arc", "90 degree arc with 8 m radius, no obstacles", true, &build_arc},
    {"channel", "3.0 m channel formed by two walls", true, &build_channel},
    {"blocked", "obstacle crossing the seed path", false, &build_blocked},
    {"narrow_slit", "1.5 m slit, below vehicle width plus margin", false, &build_narrow_slit},
};

CorridorResult run_scenario(const Scenario& scenario) {
  CorridorRequest request;
  request.request_id = scenario.name;
  request.map_id = "corridor_example_map";
  request.map_version = "v1";
  request.frame_id = "map";
  request.vehicle = default_vehicle();
  const ScenarioInput input = scenario.build();
  request.seed = input.seed;
  request.obstacles = input.obstacles;
  return planner::corridor::generate_corridor(request);
}

void print_scenario_list() {
  for (const Scenario& scenario : kScenarios) {
    std::cout << scenario.name << " - " << scenario.description << "\n";
  }
}

const Scenario* find_scenario(const std::string& name) {
  for (const Scenario& scenario : kScenarios) {
    if (name == scenario.name) {
      return &scenario;
    }
  }
  return nullptr;
}

bool write_text(const std::string& path, const std::string& text) {
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    std::cerr << "cannot open output file: " << path << "\n";
    return false;
  }
  stream << text << '\n';
  if (!stream) {
    std::cerr << "error while writing output file: " << path << "\n";
    return false;
  }
  return true;
}

int run_self_check() {
  bool all_ok = true;
  for (const Scenario& scenario : kScenarios) {
    const CorridorResult result = run_scenario(scenario);
    if (result.ok() != scenario.expect_success) {
      std::cerr << scenario.name << ": expected "
                << (scenario.expect_success ? "success" : "failure") << " but got status="
                << planner::common::to_string(result.status) << " reason=" << result.reason
                << " detail=" << result.detail << "\n";
      all_ok = false;
      continue;
    }
    if (!result.ok()) {
      std::cout << scenario.name << ": ok (explicit " << result.reason << ")\n";
      continue;
    }
    const ScenarioInput input = scenario.build();
    const CorridorVerification verification = planner::corridor::verify_corridor(
        result.corridor, input.seed, input.obstacles, default_vehicle());
    if (!verification.valid) {
      std::cerr << scenario.name << ": verification failed: " << verification.reason << ": "
                << verification.detail << "\n";
      all_ok = false;
      continue;
    }
    try {
      const std::string encoded = planner::common::encode_document(result.corridor);
      const Corridor decoded = planner::common::decode_document<Corridor>(encoded);
      if (!(decoded == result.corridor)) {
        std::cerr << scenario.name << ": JSON round-trip mismatch\n";
        all_ok = false;
        continue;
      }
    } catch (const std::exception& error) {
      std::cerr << scenario.name << ": serialization error: " << error.what() << "\n";
      all_ok = false;
      continue;
    }
    std::cout << scenario.name << ": ok (" << result.corridor.polygons.size() << " polygons, "
              << result.stats.path_length_m << " m)\n";
  }
  return all_ok ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  std::string scenario_name;
  std::string output_path;
  bool list_scenarios = false;
  bool self_check = false;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--help" || argument == "-h") {
      std::cout << kUsage;
      return 0;
    }
    if (argument == "--list-scenarios") {
      list_scenarios = true;
    } else if (argument == "--self-check") {
      self_check = true;
    } else if (argument == "--scenario" && i + 1 < argc) {
      scenario_name = argv[++i];
    } else if (argument == "--output" && i + 1 < argc) {
      output_path = argv[++i];
    } else {
      std::cerr << "unknown or incomplete argument: " << argument << "\n" << kUsage;
      return 2;
    }
  }
  if (list_scenarios && (self_check || !scenario_name.empty())) {
    std::cerr << "use --list-scenarios alone\n";
    return 2;
  }
  if (list_scenarios) {
    print_scenario_list();
    return 0;
  }
  if (self_check) {
    return run_self_check();
  }
  if (scenario_name.empty()) {
    std::cerr << kUsage;
    return 2;
  }
  const Scenario* scenario = find_scenario(scenario_name);
  if (scenario == nullptr) {
    std::cerr << "unknown scenario '" << scenario_name << "'; use --list-scenarios\n";
    return 2;
  }
  const CorridorResult result = run_scenario(*scenario);
  if (!result.ok()) {
    std::cerr << "scenario '" << scenario->name << "': status="
              << planner::common::to_string(result.status) << " reason=" << result.reason
              << " detail=" << result.detail << "\n";
    return 1;
  }
  const std::string encoded = planner::common::encode_document(result.corridor);
  if (!output_path.empty()) {
    if (!write_text(output_path, encoded)) {
      return 3;
    }
    std::cerr << "scenario '" << scenario->name << "': wrote " << output_path << " ("
              << result.corridor.polygons.size() << " polygons)\n";
  } else {
    std::cout << encoded << '\n';
  }
  return 0;
}
