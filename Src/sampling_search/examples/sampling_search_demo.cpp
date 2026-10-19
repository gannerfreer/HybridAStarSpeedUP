// planner_sampling_search 演示与自检（X-S4）。
//
// 用法：
//   planner_sampling_search_demo              # 打印已编译适配器的规划结果表
//   planner_sampling_search_demo --self-check # 每个适配器在开阔场景自检

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "planner/sampling_search/planner.hpp"
#include "planner/sampling_search/types.hpp"
#include "planner/sampling_search/validity.hpp"

namespace {

using planner::sampling_search::Bounds2d;
using planner::sampling_search::CancelToken;
using planner::sampling_search::Planner;
using planner::sampling_search::Point2d;
using planner::sampling_search::Problem;
using planner::sampling_search::Result;
using planner::sampling_search::Status;
using planner::sampling_search::make_available_planners;

struct Rect {
  double x0, y0, x1, y1;
};

const std::vector<Rect> kObstacles = {
    {15.0, 0.0, 18.0, 30.0},
    {30.0, 20.0, 38.0, 28.0},
};

bool point_valid(const Point2d& p) {
  if (p.x_m < 0.0 || p.x_m > 50.0 || p.y_m < 0.0 || p.y_m > 50.0) {
    return false;
  }
  for (const Rect& r : kObstacles) {
    if (p.x_m >= r.x0 && p.x_m <= r.x1 && p.y_m >= r.y0 && p.y_m <= r.y1) {
      return false;
    }
  }
  return true;
}

Problem make_problem(bool with_obstacles, double budget_s, std::uint64_t seed) {
  Problem problem;
  problem.bounds = Bounds2d{0.0, 0.0, 50.0, 50.0};
  problem.start = Point2d{5.0, 5.0};
  problem.goal = Point2d{45.0, 45.0};
  problem.edge_check_step = 1.0;
  problem.budget_s = budget_s;
  problem.seed = seed;
  if (!with_obstacles) {
    problem.point_valid = [](const Point2d& p) {
      return p.x_m >= 0.0 && p.x_m <= 50.0 && p.y_m >= 0.0 && p.y_m <= 50.0;
    };
    problem.edge_valid = [](const Point2d& a, const Point2d& b) {
      const double dx = b.x_m - a.x_m;
      const double dy = b.y_m - a.y_m;
      const int n = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy))));
      for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        const Point2d p{a.x_m + t * dx, a.y_m + t * dy};
        if (p.x_m < 0.0 || p.x_m > 50.0 || p.y_m < 0.0 || p.y_m > 50.0) {
          return false;
        }
      }
      return true;
    };
  } else {
    problem.point_valid = [](const Point2d& p) { return point_valid(p); };
    problem.edge_valid = [](const Point2d& a, const Point2d& b) {
      const double dx = b.x_m - a.x_m;
      const double dy = b.y_m - a.y_m;
      const int n = std::max(1, static_cast<int>(std::ceil(std::hypot(dx, dy))));
      for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        if (!point_valid(Point2d{a.x_m + t * dx, a.y_m + t * dy})) {
          return false;
        }
      }
      return true;
    };
  }
  return problem;
}

const char* status_name(Status status) {
  switch (status) {
    case Status::kSuccess:
      return "success";
    case Status::kNoPath:
      return "no_path";
    case Status::kTimeout:
      return "timeout";
    case Status::kCancelled:
      return "cancelled";
    case Status::kInvalidInput:
      return "invalid_input";
    default:
      return "other";
  }
}

}  // namespace

int main(int argc, char** argv) {
  const bool self_check = argc > 1 && std::strcmp(argv[1], "--self-check") == 0;
  const auto planners = make_available_planners();
  if (planners.empty()) {
    std::fprintf(stderr, "no adapters compiled in this build\n");
    return self_check ? 1 : 0;
  }

  if (self_check) {
    int failures = 0;
    for (const auto& planner : planners) {
      CancelToken cancel;
      const Problem problem = make_problem(false, 1.0, 0);
      const Result result = planner->plan(problem, &cancel);
      const bool ok = result.status == Status::kSuccess && !result.path.empty();
      std::printf("self-check %-12s %-10s cost=%.3f wall=%.3fs\n",
                  planner->name(), status_name(result.status), result.cost,
                  result.stats.wall_s);
      if (!ok) {
        ++failures;
      }
    }
    return failures == 0 ? 0 : 1;
  }

  std::printf("%-12s %-10s %10s %10s %10s %10s\n", "backend", "status",
              "cost", "first_s", "wall_s", "iterations");
  for (const auto& planner : planners) {
    CancelToken cancel;
    const Problem problem = make_problem(true, 1.0, 0);
    const Result result = planner->plan(problem, &cancel);
    std::printf("%-12s %-10s %10.3f %10.4f %10.3f %10llu\n", planner->name(),
                status_name(result.status), result.cost,
                result.stats.first_solution_s, result.stats.wall_s,
                static_cast<unsigned long long>(result.stats.iterations));
  }
  return 0;
}
