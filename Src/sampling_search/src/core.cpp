// planner_sampling_search 共享实现：统一输入校验、路径助手与适配器注册表。
//
// 未编译的适配器（CMake 开关 OFF 或未启用）在这里提供显式回退：可用性返回
// false、工厂返回 nullptr，绝不假装可用。

#include "planner/sampling_search/planner.hpp"

#include <cmath>
#include <string>
#include <vector>

namespace planner::sampling_search {
namespace {

bool is_finite_point(const Point2d& p) {
  return std::isfinite(p.x_m) && std::isfinite(p.y_m);
}

}  // namespace

std::string validate_problem(const Problem& problem) {
  const Bounds2d& bounds = problem.bounds;
  if (!std::isfinite(bounds.min_x) || !std::isfinite(bounds.min_y) ||
      !std::isfinite(bounds.max_x) || !std::isfinite(bounds.max_y)) {
    return "bounds must be finite";
  }
  if (!(bounds.max_x > bounds.min_x) || !(bounds.max_y > bounds.min_y)) {
    return "bounds must have positive extent";
  }
  if (!is_finite_point(problem.start) || !is_finite_point(problem.goal)) {
    return "start/goal must be finite";
  }
  if (!bounds.contains(problem.start, 1e-9)) {
    return "start outside bounds";
  }
  if (!bounds.contains(problem.goal, 1e-9)) {
    return "goal outside bounds";
  }
  if (!std::isfinite(problem.edge_check_step) ||
      problem.edge_check_step <= 0.0) {
    return "edge_check_step must be positive and finite";
  }
  if (!std::isfinite(problem.length_target) || problem.length_target < 0.0) {
    return "length_target must be non-negative and finite";
  }
  if (!std::isfinite(problem.budget_s) || problem.budget_s <= 0.0) {
    return "budget_s must be positive and finite";
  }
  if (!problem.point_valid) {
    return "point_valid callback is required";
  }
  if (!problem.edge_valid) {
    return "edge_valid callback is required";
  }
  if (!problem.point_valid(problem.start)) {
    return "start is not valid under point_valid";
  }
  if (!problem.point_valid(problem.goal)) {
    return "goal is not valid under point_valid";
  }
  return {};
}

double polyline_length(const std::vector<Point2d>& path) {
  double total = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    total += std::hypot(path[i].x_m - path[i - 1].x_m, path[i].y_m - path[i - 1].y_m);
  }
  return total;
}

bool path_edges_valid(const Problem& problem,
                      const std::vector<Point2d>& path) {
  for (std::size_t i = 1; i < path.size(); ++i) {
    if (!problem.edge_valid(path[i - 1], path[i])) {
      return false;
    }
  }
  return true;
}

Result invalid_result(const std::string& message) {
  Result result;
  result.status = Status::kInvalidInput;
  result.message = message;
  return result;
}

bool is_trivial_problem(const Problem& problem) {
  return std::hypot(problem.start.x_m - problem.goal.x_m,
                    problem.start.y_m - problem.goal.y_m) <= 1e-9;
}

Result trivial_result(const Problem& problem, const char* backend,
                      const char* variant) {
  Result result;
  result.status = Status::kSuccess;
  result.path = {problem.start};
  result.cost = 0.0;
  result.stats.backend = backend;
  result.stats.variant = variant;
  result.stats.budget_s = problem.budget_s;
  result.stats.wall_s = 0.0;
  result.stats.first_solution_s = 0.0;
  result.stats.native_params =
      "trivial: start==goal (distance <= 1e-9); backend not invoked";
  result.message = "start equals goal; trivial single-point path";
  return result;
}

// ---------------------------------------------------------------------------
// 未编译适配器的回退定义
// ---------------------------------------------------------------------------

#if !defined(PLANNER_SAMPLING_SEARCH_HAS_OMPL)
bool ompl_rrtstar_available() { return false; }
bool ompl_informed_rrtstar_available() { return false; }
std::unique_ptr<Planner> make_ompl_rrtstar() { return nullptr; }
std::unique_ptr<Planner> make_ompl_informed_rrtstar() { return nullptr; }
#endif

#if !defined(PLANNER_SAMPLING_SEARCH_HAS_CH0P1K3)
bool ch0p1k3_rrtstar_available() { return false; }
std::unique_ptr<Planner> make_ch0p1k3_rrtstar() { return nullptr; }
#endif

#if !defined(PLANNER_SAMPLING_SEARCH_HAS_KLAMPT)
bool klampt_rrtstar_available() { return false; }
std::unique_ptr<Planner> make_klampt_rrtstar() { return nullptr; }
#endif

#if !defined(PLANNER_SAMPLING_SEARCH_HAS_VSS2SN)
bool vss2sn_rrtstar_available() { return false; }
std::unique_ptr<Planner> make_vss2sn_rrtstar() { return nullptr; }
#endif

#if !defined(PLANNER_SAMPLING_SEARCH_HAS_AIWINTER)
bool aiwinter_rrtstar_available() { return false; }
std::unique_ptr<Planner> make_aiwinter_rrtstar() { return nullptr; }
#endif

// ---------------------------------------------------------------------------
// 注册表
// ---------------------------------------------------------------------------

std::vector<std::string> available_backends() {
  std::vector<std::string> backends;
  if (ompl_rrtstar_available()) {
    backends.emplace_back("ompl_rrtstar");
  }
  if (ompl_informed_rrtstar_available()) {
    backends.emplace_back("ompl_informed_rrtstar");
  }
  if (ch0p1k3_rrtstar_available()) {
    backends.emplace_back("ch0p1k3_rrtstar");
  }
  if (klampt_rrtstar_available()) {
    backends.emplace_back("klampt_rrtstar");
  }
  if (vss2sn_rrtstar_available()) {
    backends.emplace_back("vss2sn_rrtstar");
  }
  if (aiwinter_rrtstar_available()) {
    backends.emplace_back("aiwinter_rrtstar");
  }
  return backends;
}

std::vector<std::unique_ptr<Planner>> make_available_planners() {
  std::vector<std::unique_ptr<Planner>> planners;
  if (auto planner = make_ompl_rrtstar()) {
    planners.push_back(std::move(planner));
  }
  if (auto planner = make_ompl_informed_rrtstar()) {
    planners.push_back(std::move(planner));
  }
  if (auto planner = make_ch0p1k3_rrtstar()) {
    planners.push_back(std::move(planner));
  }
  if (auto planner = make_klampt_rrtstar()) {
    planners.push_back(std::move(planner));
  }
  if (auto planner = make_vss2sn_rrtstar()) {
    planners.push_back(std::move(planner));
  }
  if (auto planner = make_aiwinter_rrtstar()) {
    planners.push_back(std::move(planner));
  }
  return planners;
}

}  // namespace planner::sampling_search
