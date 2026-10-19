// planner_bspline 的命令行示例。
//
// 构造一条带 8 个控制点的夹持三次曲线，打印等参数采样点及其速度，
// 以及近似等弧长重采样结果。确定性；不读写任何文件。

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "planner/bspline/bspline.hpp"

namespace {

void print_usage(const char* program) {
  std::printf("usage: %s [--points N] [--help]\n", program);
  std::printf("  --points N  number of arc-length samples (default 11)\n");
}

}  // namespace

int main(int argc, char** argv) {
  int points = 11;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--help" || argument == "-h") {
      print_usage(argv[0]);
      return 0;
    }
    if (argument == "--points" && i + 1 < argc) {
      points = std::atoi(argv[++i]);
      if (points < 1) {
        std::fprintf(stderr, "error: --points must be at least 1\n");
        return 2;
      }
      continue;
    }
    std::fprintf(stderr, "error: unexpected argument '%s'\n", argument.c_str());
    print_usage(argv[0]);
    return 2;
  }

  using planner::bspline::Bspline;
  using planner::bspline::Status;
  using planner::common::Point2d;

  const std::vector<Point2d> control_points = {
      {0.0, -1.0}, {0.5, -0.75}, {1.0, -0.5}, {1.5, -0.25},
      {2.0, 0.0},  {2.5, 0.25},  {3.0, 0.5},  {3.5, 0.75}};
  const std::vector<double> knots =
      planner::bspline::clamped_uniform_knots(control_points.size(), 3);
  std::string error;
  std::optional<Bspline> curve =
      Bspline::create(control_points, knots, &error);
  if (!curve.has_value()) {
    std::fprintf(stderr, "error: curve construction failed: %s\n",
                 error.c_str());
    return 1;
  }

  const planner::bspline::ParameterDomain domain = curve->parameter_domain();
  std::printf("planner_bspline example\n");
  std::printf("  degree            : %d\n", curve->degree());
  std::printf("  control points    : %zu\n", curve->control_point_count());
  std::printf("  parameter domain  : [%.6f, %.6f] (%s)\n", domain.min_parameter,
              domain.max_parameter,
              domain.kind == planner::bspline::ParameterKind::kTime ? "time"
                                                                    : "geometric");
  std::printf("  total length      : %.9f m\n", curve->total_length_m());

  std::printf("\nequal-parameter samples (position / first derivative):\n");
  for (int i = 0; i <= 10; ++i) {
    const double u = static_cast<double>(i) / 10.0;
    const auto position = curve->evaluate(u, 0);
    const auto velocity = curve->evaluate(u, 1);
    if (position.status != Status::kSuccess ||
        velocity.status != Status::kSuccess) {
      std::fprintf(stderr, "error: evaluation failed at u=%.3f\n", u);
      return 1;
    }
    std::printf("  u=%.3f  p=(%9.6f, %9.6f)  v=(%9.6f, %9.6f)\n", u,
                position.value.x_m, position.value.y_m, velocity.value.x_m,
                velocity.value.y_m);
  }

  const auto resampled = curve->sample_equal_arc_length(
      static_cast<std::size_t>(points));
  if (resampled.status != Status::kSuccess) {
    std::fprintf(stderr, "error: arc-length resampling failed: %s\n",
                 resampled.message.c_str());
    return 1;
  }
  std::printf(
      "\nequal arc-length samples (target %.3e m; measured max error %.3e m):\n",
      1e-3, resampled.max_interval_error_m);
  for (const auto& sample : resampled.samples) {
    std::printf("  s=%9.6f m  u=%.6f  p=(%9.6f, %9.6f)\n", sample.arc_length_m,
                sample.parameter, sample.position.x_m, sample.position.y_m);
  }
  return 0;
}
