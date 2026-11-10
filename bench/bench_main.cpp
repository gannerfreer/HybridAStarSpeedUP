#include "angle.hpp"
#include "collision_checker.hpp"
#include "geometry.hpp"
#include "motion.hpp"
#include "scene.hpp"
#include "user_astar.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
using namespace planner;

volatile double g_sink = 0.0;

template <typename Fn>
double measure_seconds(int iterations, Fn&& fn) {
  const auto begin = Clock::now();
  for (int i = 0; i < iterations; ++i) {
    fn(i);
  }
  const auto end = Clock::now();
  return std::chrono::duration<double>(end - begin).count();
}

void report_ns_per_op(const char* name, int iterations, double seconds) {
  std::printf("%-42s %12.2f ns/op  (%d iterations)\n", name,
              seconds * 1e9 / static_cast<double>(iterations), iterations);
}

void report_ms_per_op(const char* name, int iterations, double seconds) {
  std::printf("%-42s %12.4f ms/op  (%d iterations)\n", name,
              seconds * 1e3 / static_cast<double>(iterations), iterations);
}

common::VehicleModel bench_vehicle() {
  common::VehicleModel vehicle;
  vehicle.wheelbase_m = 2.5;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.8;
  vehicle.rear_overhang_m = 0.8;
  vehicle.max_steer_rad = 0.6;
  vehicle.safety_margin_m = 0.1;
  return vehicle;
}

collision::Scene bench_scene() {
  collision::Scene scene;
  auto& grid = scene.grid;
  grid.map_id = "benchmark_detour";
  grid.map_version = "1";
  grid.width = 80;
  grid.height = 80;
  grid.resolution_m = 0.5;
  grid.cells.assign(grid.width * grid.height, common::kCellFree);
  for (int row = 16; row < 24; ++row) {
    for (int col = 38; col < 42; ++col) {
      grid.cells[row * grid.width + col] = common::kCellOccupied;
    }
  }
  return scene;
}

}  // namespace

int main() {
  std::printf("planner benchmarks (std::chrono::steady_clock)\n");

  double seconds = measure_seconds(5000000, [](int i) {
    g_sink = g_sink + math::normalize_angle_rad(-1000.0 + i * 0.001);
  });
  report_ns_per_op("math::normalize_angle_rad", 5000000, seconds);

  const common::Pose2d arc_start{1.0, 2.0, 0.3};
  seconds = measure_seconds(2000000, [&](int i) {
    const double s = 0.1 + (i % 1000) * 0.001;
    const double curvature = -0.2 + (i % 200) * 0.001;
    const auto out = math::integrate_arc(arc_start, s, curvature);
    g_sink = g_sink + out.x_m + out.y_m;
  });
  report_ns_per_op("math::integrate_arc", 2000000, seconds);

  const auto polygon = math::rectangle_polygon(0.0, 0.0, 2.0, 2.0);
  seconds = measure_seconds(2000000, [&](int i) {
    const double offset = (i % 100) * 0.01;
    const math::Aabb2d box{0.5 + offset, 0.5, 1.5 + offset, 1.5};
    g_sink = g_sink + (math::convex_polygon_intersects_aabb(polygon, box, 0.0)
                           ? 1.0
                           : 0.0);
  });
  report_ns_per_op("math::convex_polygon_intersects_aabb", 2000000, seconds);

  const auto vehicle = bench_vehicle();
  const auto scene = bench_scene();
  const collision::CollisionChecker checker(vehicle, scene);
  if (!checker.valid()) {
    std::fprintf(stderr, "benchmark scene invalid: %s\n",
                 checker.invalid_reason().c_str());
    return EXIT_FAILURE;
  }

  seconds = measure_seconds(200000, [&](int i) {
    const common::Pose2d pose{10.0 + (i % 10) * 0.01, 10.0, 0.0};
    g_sink = g_sink + (checker.check_pose(pose).is_clear() ? 1.0 : 0.0);
  });
  report_ns_per_op("CollisionChecker::check_pose", 200000, seconds);

  seconds = measure_seconds(20000, [&](int i) {
    collision::MotionSegment motion;
    motion.start = {10.0, 10.0, 0.0};
    motion.length_m = 4.0 + (i % 10) * 0.1;
    g_sink = g_sink + (checker.check_motion(motion).is_clear() ? 1.0 : 0.0);
  });
  report_ns_per_op("CollisionChecker::check_motion", 20000, seconds);

  common::PlanRequest request;
  request.request_id = "benchmark_detour";
  request.map_id = scene.grid.map_id;
  request.map_version = scene.grid.map_version;
  request.vehicle = vehicle;
  request.start = {10.0, 10.0, 0.0};
  request.goal = {30.0, 10.0, 0.0};
  request.time_budget_s = 10.0;

  hybrid_astar::UserAstar planner;
  const int plan_iterations = 5;
  const auto warmup = planner.plan(request, scene.grid, checker);
  if (warmup.status != common::Status::kSuccess) {
    std::fprintf(stderr, "benchmark warmup plan failed: %s\n",
                 common::to_string(warmup.status).c_str());
    return EXIT_FAILURE;
  }

  int successes = 0;
  double total_seconds = 0.0;
  double best_seconds = 1e100;
  double path_length_m = 0.0;
  for (int i = 0; i < plan_iterations; ++i) {
    const auto begin = Clock::now();
    const auto result = planner.plan(request, scene.grid, checker);
    const auto end = Clock::now();
    const double elapsed =
        std::chrono::duration<double>(end - begin).count();
    total_seconds += elapsed;
    best_seconds = std::min(best_seconds, elapsed);
    if (result.status == common::Status::kSuccess && !result.path.empty()) {
      ++successes;
      path_length_m = result.path.back().points.back().s_m;
      g_sink = g_sink + path_length_m;
    }
  }
  if (successes != plan_iterations) {
    std::fprintf(stderr, "benchmark plan succeeded only %d/%d times\n",
                 successes, plan_iterations);
    return EXIT_FAILURE;
  }
  report_ms_per_op("UserAstar::plan (obstacle detour)", plan_iterations,
                   total_seconds);
  std::printf("%-42s %12.4f ms\n", "UserAstar::plan best", best_seconds * 1e3);
  std::printf("%-42s %12.3f m\n", "UserAstar::plan path length",
              path_length_m);
  std::printf("%-42s %12d/%d\n", "UserAstar::plan success", successes,
              plan_iterations);
  return EXIT_SUCCESS;
}
