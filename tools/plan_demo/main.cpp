/**
 * @file main.cpp
 * @brief 从 TOML 配置加载车辆、规划器与场景并执行一次 Hybrid A* 规划演示。
 */

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>

#include "collision_checker.hpp"
#include "planner/config/config_loader.hpp"
#include "scene.hpp"
#include "user_astar.hpp"

namespace {

struct Arguments {
  std::string vehicle_path;
  std::string planner_path;
  std::string scenario_path;
};

void print_usage(const char* program) {
  std::cout << "Usage: " << program
            << " --vehicle <vehicle.toml> --planner <planner.toml>"
               " --scenario <scenario.toml>\n";
}

bool parse_arguments(int argc, char** argv, Arguments* arguments) {
  for (int i = 1; i < argc; ++i) {
    const std::string_view argument(argv[i]);
    if (argument == "--help" || argument == "-h") {
      print_usage(argv[0]);
      std::exit(0);
    }
    if (i + 1 >= argc) {
      std::cerr << "missing value for argument: " << argument << "\n";
      return false;
    }
    const std::string value(argv[++i]);
    if (argument == "--vehicle") {
      arguments->vehicle_path = value;
    } else if (argument == "--planner") {
      arguments->planner_path = value;
    } else if (argument == "--scenario") {
      arguments->scenario_path = value;
    } else {
      std::cerr << "unknown argument: " << argument << "\n";
      return false;
    }
  }
  if (arguments->vehicle_path.empty() || arguments->planner_path.empty() ||
      arguments->scenario_path.empty()) {
    std::cerr << "all of --vehicle, --planner and --scenario are required\n";
    return false;
  }
  return true;
}

template <typename T>
bool print_config_error(const planner::config::ConfigResult<T>& result, const char* label) {
  if (result) {
    return false;
  }
  std::cerr << label << " config error: " << result.error.message << "\n";
  return true;
}

double path_length_m(const std::vector<planner::common::PathSegment>& path) {
  double length_m = 0.0;
  const planner::common::PathPoint* previous = nullptr;
  for (const planner::common::PathSegment& segment : path) {
    for (const planner::common::PathPoint& point : segment.points) {
      if (previous != nullptr) {
        length_m +=
            std::hypot(point.pose.x_m - previous->pose.x_m, point.pose.y_m - previous->pose.y_m);
      }
      previous = &point;
    }
  }
  return length_m;
}

}  // namespace

int main(int argc, char** argv) {
  using namespace planner;

  Arguments arguments;
  if (!parse_arguments(argc, argv, &arguments)) {
    print_usage(argv[0]);
    return 1;
  }

  const auto vehicle_result = config::load_vehicle_config(arguments.vehicle_path);
  if (print_config_error(vehicle_result, "vehicle")) {
    return 1;
  }
  const auto planner_result = config::load_planner_config(arguments.planner_path);
  if (print_config_error(planner_result, "planner")) {
    return 1;
  }
  const auto scenario_result = config::load_scenario_config(arguments.scenario_path);
  if (print_config_error(scenario_result, "scenario")) {
    return 1;
  }

  const common::VehicleModel& vehicle = *vehicle_result.value;
  const common::ScenarioConfig& scenario = *scenario_result.value;

  collision::Scene scene = collision::make_scene(scenario);
  std::string reason;
  if (!scene.valid(&reason)) {
    std::cerr << "scenario error: " << arguments.scenario_path << ": " << reason << "\n";
    return 1;
  }

  collision::CollisionChecker checker(vehicle, scene);
  if (!checker.valid()) {
    std::cerr << "collision checker error: " << checker.invalid_reason() << "\n";
    return 1;
  }

  hybrid_astar::UserAstar planner(*planner_result.value);
  common::PlanRequest request;
  request.request_id = "plan_demo";
  request.map_id = scene.grid.map_id;
  request.map_version = scene.grid.map_version;
  request.vehicle = vehicle;
  request.start = scenario.start;
  request.goal = scenario.goal;

  const common::PlanResult result = planner.plan(request, scene.grid, checker);
  std::cout << "status: " << common::to_string(result.status) << "\n";
  std::cout << "reason: " << (result.reason.empty() ? "(none)" : result.reason) << "\n";
  std::cout << "path_length_m: " << path_length_m(result.path) << "\n";

  return result.status == common::Status::kSuccess ? 0 : 2;
}
