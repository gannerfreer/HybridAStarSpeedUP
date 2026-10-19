// planner_polynomial 独立示例（Wave 3）。
//
// 模式：
//   planner_polynomial_example --self-check
//       构造演示、验证边界条件、衔接点连续性和时间缩放，
//       然后打印 "ok"（退出码 0）或诊断信息（退出码 1）。
//   planner_polynomial_example [--samples N]
//       将两段五次换道演示的 CSV 轨迹采样（每个分段 N 个点，
//       默认 100）打印到 stdout。
//
// 演示形状沿用合成显示样例
// tests/fixtures/v1/playback/polynomial_curve.json，但在此重新构建，
// 使示例在运行时不依赖夹具。

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>

#include "planner/common/types.hpp"
#include "planner/polynomial/piecewise_trajectory.hpp"
#include "planner/polynomial/polynomial.hpp"
#include "planner/polynomial/result.hpp"
#include "planner/polynomial/trajectory_segment.hpp"

namespace {

using planner::common::Point2d;
using planner::polynomial::CubicBoundary2d;
using planner::polynomial::PiecewiseTrajectory2d;
using planner::polynomial::QuinticBoundary2d;
using planner::polynomial::Result;
using planner::polynomial::TrajectorySegment;

constexpr const char* kUsage =
    "usage:\n"
    "  planner_polynomial_example --self-check\n"
    "  planner_polynomial_example [--samples N]\n";

Result<PiecewiseTrajectory2d> make_demo_trajectory() {
  const QuinticBoundary2d first{
      Point2d{10.0, 5.0}, Point2d{3.75, 0.0}, Point2d{0.0, 0.0},
      Point2d{25.0, 8.0}, Point2d{3.75, 0.0}, Point2d{0.0, 0.0}};
  const QuinticBoundary2d second{
      Point2d{25.0, 8.0}, Point2d{3.75, 0.0}, Point2d{0.0, 0.0},
      Point2d{32.5, 8.0}, Point2d{3.75, 0.0}, Point2d{0.0, 0.0}};
  const Result<TrajectorySegment> a = TrajectorySegment::quintic(first, 4.0);
  if (!a.ok()) {
    return Result<PiecewiseTrajectory2d>::failure(a.code);
  }
  const Result<TrajectorySegment> b = TrajectorySegment::quintic(second, 2.0);
  if (!b.ok()) {
    return Result<PiecewiseTrajectory2d>::failure(b.code);
  }
  std::vector<TrajectorySegment> segments;
  segments.push_back(a.value);
  segments.push_back(b.value);
  return PiecewiseTrajectory2d::create(std::move(segments));
}

bool nearly_equal(double actual, double expected, double tolerance) {
  return std::fabs(actual - expected) <= tolerance * (1.0 + std::fabs(expected));
}

int run_self_check() {
  const Result<PiecewiseTrajectory2d> trajectory = make_demo_trajectory();
  if (!trajectory.ok()) {
    std::cerr << "demo construction failed: " << planner::polynomial::to_string(trajectory.code)
              << "\n";
    return 1;
  }
  const PiecewiseTrajectory2d& demo = trajectory.value;
  bool ok = true;
  const auto check = [&ok](bool condition, const char* message) {
    if (!condition) {
      std::cerr << message << "\n";
      ok = false;
    }
  };

  check(demo.segment_count() == 2, "expected two segments");
  check(demo.duration_s() == 6.0, "expected total duration 6 s");

  // 演示的边界值（位置、速度、加速度）。
  const Result<Point2d> start = demo.evaluate(0.0, 0);
  check(start.ok() && nearly_equal(start.value.x_m, 10.0, 1e-9) &&
            nearly_equal(start.value.y_m, 5.0, 1e-9),
        "start position mismatch");
  const Result<Point2d> start_velocity = demo.evaluate(0.0, 1);
  check(start_velocity.ok() && nearly_equal(start_velocity.value.x_m, 3.75, 1e-9) &&
            nearly_equal(start_velocity.value.y_m, 0.0, 1e-9),
        "start velocity mismatch");
  const Result<Point2d> end = demo.evaluate(6.0, 0);
  check(end.ok() && nearly_equal(end.value.x_m, 32.5, 1e-9) &&
            nearly_equal(end.value.y_m, 8.0, 1e-9),
        "end position mismatch");

  // 衔接点连续性（经分段容器保持 C2）。
  for (int order = 0; order <= 2; ++order) {
    const Result<Point2d> right = demo.evaluate(4.0, order);
    const Result<Point2d> left = demo.segment(0).evaluate(4.0, order);
    check(right.ok() && left.ok() &&
              nearly_equal(right.value.x_m, left.value.x_m, 1e-9) &&
              nearly_equal(right.value.y_m, left.value.y_m, 1e-9),
          "junction continuity mismatch");
  }

  // 定义域外访问必须失败，不得外推。
  check(!demo.evaluate(-0.01, 0).ok(), "negative time must be rejected");
  check(!demo.evaluate(6.01, 0).ok(), "time past the end must be rejected");

  // 时间缩放使同一几何形状上的速度减半。
  const Result<PiecewiseTrajectory2d> stretched = demo.scaled_time(2.0);
  check(stretched.ok(), "time scaling failed");
  if (stretched.ok()) {
    const Result<Point2d> middle = stretched.value.evaluate(8.0, 0);
    const Result<Point2d> original_middle = demo.evaluate(4.0, 0);
    check(middle.ok() && original_middle.ok() &&
              nearly_equal(middle.value.x_m, original_middle.value.x_m, 1e-9) &&
              nearly_equal(middle.value.y_m, original_middle.value.y_m, 1e-9),
          "scaled-time geometry mismatch");
  }

  if (ok) {
    std::cout << "ok\n";
    return 0;
  }
  return 1;
}

int print_samples(int samples_per_segment) {
  const Result<PiecewiseTrajectory2d> trajectory = make_demo_trajectory();
  if (!trajectory.ok()) {
    std::cerr << "demo construction failed\n";
    return 1;
  }
  const PiecewiseTrajectory2d& demo = trajectory.value;
  std::cout << "segment,t_s,x_m,y_m,vx_mps,vy_mps,ax_mps2,ay_mps2\n";
  std::cout << std::fixed << std::setprecision(9);
  for (std::size_t index = 0; index < demo.segment_count(); ++index) {
    const TrajectorySegment& segment = demo.segment(index);
    const double start_s = demo.segment_start_time_s(index);
    const int count = samples_per_segment > 1 ? samples_per_segment - 1 : 1;
    for (int j = 0; j <= count; ++j) {
      const double local_s = segment.duration_s() * static_cast<double>(j) / static_cast<double>(count);
      const Result<Point2d> position = segment.evaluate(local_s, 0);
      const Result<Point2d> velocity = segment.evaluate(local_s, 1);
      const Result<Point2d> acceleration = segment.evaluate(local_s, 2);
      if (!position.ok() || !velocity.ok() || !acceleration.ok()) {
        std::cerr << "sampling failed\n";
        return 1;
      }
      std::cout << index << "," << start_s + local_s << "," << position.value.x_m << ","
                << position.value.y_m << "," << velocity.value.x_m << "," << velocity.value.y_m
                << "," << acceleration.value.x_m << "," << acceleration.value.y_m << "\n";
    }
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  int samples_per_segment = 100;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--self-check") {
      return run_self_check();
    }
    if (argument == "--samples" && i + 1 < argc) {
      samples_per_segment = std::atoi(argv[++i]);
      if (samples_per_segment < 2) {
        std::cerr << "--samples must be >= 2\n" << kUsage;
        return 2;
      }
      continue;
    }
    if (argument == "--help" || argument == "-h") {
      std::cout << kUsage;
      return 0;
    }
    std::cerr << "unknown argument: " << argument << "\n" << kUsage;
    return 2;
  }
  return print_samples(samples_per_segment);
}
