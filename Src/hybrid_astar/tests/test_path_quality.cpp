// 路径质量测试：尖点连续性（hybrid-001）与 zm0612_normalized 的路径长度
// 目标（hybrid-002 / §7.2）。
//
// * 尖点连续性：冻结契约（序列化 README + 回放样例
//   `path_hybrid_reverse.json`）要求方向变化处的位姿既是前一分段的最后一
//   个点，也是后一分段的第一个点，即每个尖点处几何间隙为零。
// * 路径质量：`zm0612_normalized` 的 Wave 6 目标是 deadend_124 <= 1.10
//   （原为 1.181）。冻结配置（9 个转向采样，2.0/1.5 m 步长）在该场景达到
//   1.000；obstacle_109 仍约 1.105，被 selection.json 记为待优化项，因此
//   回归断言检查显式目标与 1.15 的合理性上界，并打印完整的逐场景比值表
//   备查。

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "test_support.hpp"

namespace planner::hybrid_astar {
namespace {

using test_support::PathCase;
using test_support::PreparedCase;

const std::vector<PathCase>& all_cases() {
  static const std::vector<PathCase> cases = test_support::load_path_cases();
  return cases;
}

const PathCase* find_case(const std::string& id) {
  for (const PathCase& path_case : all_cases()) {
    if (path_case.id == id) {
      return &path_case;
    }
  }
  return nullptr;
}

std::optional<double> plan_length(const std::string& backend_id,
                                  const PreparedCase& prepared,
                                  const common::VehicleModel& vehicle) {
  const std::unique_ptr<HybridAStarBackend> backend = create_backend(backend_id);
  if (backend == nullptr ||
      !backend->configure(nlohmann::json::object()).accepted) {
    return std::nullopt;
  }
  PlannerRsProvider provider(vehicle);
  common::PlanRequest request = prepared.request;
  request.time_budget_s = 60.0;
  const common::PlanResult result =
      backend->plan(request, prepared.grid, prepared.checker, provider);
  if (result.status != common::Status::kSuccess) {
    return std::nullopt;
  }
  return result.statistics.value("path_length_m", 0.0);
}

TEST(PathQuality, cusp_poses_are_duplicated_with_zero_gap) {
  const common::VehicleModel vehicle = test_support::load_default_vehicle();
  const std::vector<std::string> case_ids = {
      "park_116", "park_117", "park_118", "park_119", "park_120",
      "deadend_122", "deadend_125"};
  for (const std::string& case_id : case_ids) {
    SCOPED_TRACE(case_id);
    const PathCase* path_case = find_case(case_id);
    ASSERT_NE(path_case, nullptr);
    const PreparedCase prepared = test_support::prepare_case(*path_case, vehicle);
    const std::unique_ptr<HybridAStarBackend> backend =
        create_backend("ros_free");
    ASSERT_TRUE(backend->configure(nlohmann::json::object()).accepted);
    PlannerRsProvider provider(vehicle);
    common::PlanRequest request = prepared.request;
    request.time_budget_s = 60.0;
    const common::PlanResult result =
        backend->plan(request, prepared.grid, prepared.checker, provider);
    ASSERT_EQ(result.status, common::Status::kSuccess) << result.reason;
    ASSERT_GE(result.path.size(), 2u)
        << "expected at least one direction change";

    double max_gap = 0.0;
    for (std::size_t segment = 1; segment < result.path.size(); ++segment) {
      const common::PathPoint& previous =
          result.path[segment - 1].points.back();
      const common::PathPoint& next = result.path[segment].points.front();
      const double gap =
          std::hypot(previous.pose.x_m - next.pose.x_m,
                     previous.pose.y_m - next.pose.y_m);
      max_gap = std::max(max_gap, gap);
      EXPECT_LE(gap, 1e-9) << "cusp gap at segment " << segment;
      EXPECT_NE(previous.direction, next.direction);
      EXPECT_NEAR(previous.s_m, next.s_m, 1e-9);
    }
    std::fprintf(stderr, "CUSP %s: segments=%zu max_gap=%.3e m\n",
                 case_id.c_str(), result.path.size(), max_gap);
    const std::string error = test_support::validation_error(
        result, path_case->start, path_case->goal, vehicle);
    EXPECT_TRUE(error.empty()) << error;
  }
}

TEST(PathQuality, zm0612_normalized_default_meets_wave6_targets) {
  const common::VehicleModel vehicle = test_support::load_default_vehicle();
  const std::vector<std::string> scenario_ids = {
      "open_101",    "open_104",    "obstacle_106", "obstacle_109",
      "corridor_111", "corridor_112", "corridor_114", "park_116",
      "park_118",    "deadend_121", "deadend_124"};
  const std::vector<std::string> reference_ids = {"ros_free",
                                                  "smac_normalized",
                                                  "kth_normalized"};

  std::map<std::string, double> reference_best;
  for (const std::string& scenario_id : scenario_ids) {
    const PathCase* path_case = find_case(scenario_id);
    ASSERT_NE(path_case, nullptr) << scenario_id;
    const PreparedCase prepared =
        test_support::prepare_case(*path_case, vehicle);
    for (const std::string& reference_id : reference_ids) {
      const auto length = plan_length(reference_id, prepared, vehicle);
      if (length.has_value()) {
        const auto found = reference_best.find(scenario_id);
        if (found == reference_best.end() || *length < found->second) {
          reference_best[scenario_id] = *length;
        }
      }
    }
  }

  double worst_ratio = 0.0;
  std::string worst_scenario;
  for (const std::string& scenario_id : scenario_ids) {
    const PathCase* path_case = find_case(scenario_id);
    const PreparedCase prepared =
        test_support::prepare_case(*path_case, vehicle);
    const auto length = plan_length("zm0612_normalized", prepared, vehicle);
    ASSERT_TRUE(length.has_value()) << "no solution on " << scenario_id;
    const double best = std::min(reference_best[scenario_id], *length);
    const double ratio = best > 0.0 ? *length / best : 1.0;
    if (ratio > worst_ratio) {
      worst_ratio = ratio;
      worst_scenario = scenario_id;
    }
    std::fprintf(stderr, "ZM0612_DEFAULT %s ratio=%.3f len=%.3f best=%.3f\n",
                 scenario_id.c_str(), ratio, *length, best);
  }
  std::fprintf(stderr, "ZM0612_DEFAULT worst_ratio=%.3f (%s)\n", worst_ratio,
               worst_scenario.c_str());

  // 该候选的 Wave 6 显式目标。
  const PathCase* deadend = find_case("deadend_124");
  ASSERT_NE(deadend, nullptr);
  const PreparedCase prepared =
      test_support::prepare_case(*deadend, vehicle);
  const auto length = plan_length("zm0612_normalized", prepared, vehicle);
  ASSERT_TRUE(length.has_value());
  const double deadend_ratio =
      *length / std::min(reference_best["deadend_124"], *length);
  EXPECT_LE(deadend_ratio, 1.10)
      << "deadend_124 path-length target regressed";
  // 其余场景的合理性上界（obstacle_109 是已知待优化用例，约 1.105，由
  // selection.json 报告）。
  EXPECT_LE(worst_ratio, 1.15);
}

}  // namespace
}  // namespace planner::hybrid_astar
