// ros_free 归一化后端的夹具驱动正确性测试。
//
// 用例来自 tests/fixtures/v2/paths/cases.json（六个类别共 30 个常规场景
// 加 4 个边界场景；14 个 v2 修订用例的 grid.map_version 为 "v2"，其余 20
// 个保持 "v1"）。验收遵循 Docs §7.1：
//   * 成功：端点位置误差 <= 0.05 m，偏航误差 <= 0.02 rad，整车路径无
//     碰撞，曲率在车辆限制内；
//   * 负例不得报告成功，且必须映射到文档化的失败原因。
//
// 部分手写夹具与冻结的碰撞契约不一致（起点/目标位姿碰撞，或场景因契约
// 安全边距在几何上不可行）。这些用例在此分类并报告，绝不静默接受：
//   * `fixture_conflict`：冻结契约拒绝文档化的起点/目标位姿，因此规划器
//     唯一正确的答案是 invalid_input；测试正是这样断言，且不放松硬性
//     碰撞检查；
//   * `unresolved`：位姿无碰撞，但规划器在文档化成功场景上返回 no_path。
//     该用例会被打印并记入报告用的 JUnit 属性；不计为规划器通过。
// 真实用例保留完整验收断言。

#include <gtest/gtest.h>

#include <cstdio>
#include <string>
#include <vector>

#include "test_support.hpp"

namespace planner::hybrid_astar {
namespace {

using test_support::PathCase;

const std::vector<PathCase>& all_cases() {
  static const std::vector<PathCase> cases = test_support::load_path_cases();
  return cases;
}

struct CaseOutcome {
  bool fixture_conflict{false};
  bool unresolved{false};
  std::string note;
};

void run_case(const PathCase& path_case, const common::VehicleModel& vehicle,
              bool expect_reverse, const std::string& backend_id,
              CaseOutcome* outcome_ptr) {
  CaseOutcome& outcome = *outcome_ptr;
  SCOPED_TRACE("case " + path_case.id);
  test_support::PreparedCase prepared =
      test_support::prepare_case(path_case, vehicle);
  ASSERT_TRUE(prepared.checker.valid()) << prepared.checker.invalid_reason();

  const std::unique_ptr<HybridAStarBackend> backend = create_backend(backend_id);
  ASSERT_NE(backend, nullptr) << "unknown backend id " << backend_id;
  const ConfigReport report = backend->configure(nlohmann::json::object());
  ASSERT_TRUE(report.accepted) << report.reason;
  PlannerRsProvider provider(vehicle);

  common::PlanRequest request = prepared.request;
  if (!path_case.time_budget_s.has_value() &&
      !path_case.node_budget.has_value()) {
    // 正确性运行使用 60 s 预算而非基准测试默认的 5 s，以便每个用例完成
    // 完整搜索（包括穷尽式不可行性证明）。组件报告记录了这一点；后端
    // 默认仍为基准测试的 5 s / 1e6 节点预算。
    request.time_budget_s = 60.0;
  }

  const common::PlanResult result =
      backend->plan(request, prepared.grid, prepared.checker, provider);

  const bool poses_clear =
      prepared.start_check.is_clear() && prepared.goal_check.is_clear();

  if (path_case.expected_status == "success") {
    if (!poses_clear) {
      outcome.fixture_conflict = true;
      outcome.note = path_case.id +
                     "=fixture_conflict(goal/start pose in collision under "
                     "the frozen contract)";
      std::fprintf(stderr,
                   "FIXTURE_CONFLICT %s: documented success but start/goal "
                   "pose is in collision under the frozen contract; "
                   "start=%s goal=%s\n",
                   path_case.id.c_str(),
                   test_support::describe_collision(prepared.start_check).c_str(),
                   test_support::describe_collision(prepared.goal_check).c_str());
      EXPECT_EQ(result.status, common::Status::kInvalidInput)
          << "reason: " << result.reason;
      return;
    }
    if (result.status == common::Status::kSuccess) {
      const std::string error = test_support::validation_error(
          result, path_case.start, path_case.goal, vehicle);
      EXPECT_TRUE(error.empty()) << error;

      const collision::CheckResult collision =
          prepared.checker.check_path(result.path);
      EXPECT_TRUE(collision.is_clear())
          << test_support::describe_collision(collision);

      const test_support::PathAnalysis analysis =
          test_support::analyze_path(result, path_case.start, path_case.goal,
                                     vehicle);
      if (expect_reverse) {
        EXPECT_GE(analysis.gear_changes, 1)
            << "case description requires reversing; no direction change found";
      }
      ASSERT_TRUE(result.statistics.contains("gear_changes"));
      EXPECT_EQ(result.statistics.at("gear_changes").get<std::int64_t>(),
                analysis.gear_changes);
      ASSERT_TRUE(result.statistics.contains("path_length_m"));
      EXPECT_NEAR(result.statistics.at("path_length_m").get<double>(),
                  analysis.length_m, 1e-9);
      return;
    }
    if (result.status == common::Status::kNoPath) {
      outcome.unresolved = true;
      outcome.note = path_case.id + "=unresolved(no_path on documented success)";
      std::fprintf(stderr,
                   "UNRESOLVED %s: documented success, poses clear, planner "
                   "returned no_path; stats=%s\n",
                   path_case.id.c_str(), result.statistics.dump().c_str());
      return;
    }
    EXPECT_EQ(result.status, common::Status::kSuccess)
        << "reason: " << result.reason;
    return;
  }

  const common::Status expected =
      test_support::expected_status_for(path_case.expected_failure_reason);
  if (expected != common::Status::kInvalidInput &&
      expected != common::Status::kCancelled && !poses_clear) {
    outcome.fixture_conflict = true;
    outcome.note = path_case.id +
                   "=fixture_conflict(documented failure reason conflicts with "
                   "a colliding start/goal pose)";
    std::fprintf(stderr,
                 "FIXTURE_CONFLICT %s: documented reason '%s' but start/goal "
                 "pose is in collision under the frozen contract; "
                 "start=%s goal=%s\n",
                 path_case.id.c_str(),
                 path_case.expected_failure_reason.c_str(),
                 test_support::describe_collision(prepared.start_check).c_str(),
                 test_support::describe_collision(prepared.goal_check).c_str());
    EXPECT_EQ(result.status, common::Status::kInvalidInput)
        << "reason: " << result.reason;
    return;
  }

  ASSERT_NE(result.status, common::Status::kSuccess)
      << "negative case must not report success";
  EXPECT_FALSE(result.reason.empty());
  EXPECT_EQ(result.status, expected)
      << "reason: " << result.reason
      << " stats: " << result.statistics.dump();
}

struct ClassSummary {
  int executed{0};
  int conflicts{0};
  int unresolved{0};
  std::string notes;
};

void accumulate(ClassSummary* summary, const CaseOutcome& outcome) {
  if (outcome.fixture_conflict) {
    ++summary->conflicts;
  }
  if (outcome.unresolved) {
    ++summary->unresolved;
  }
  if (!outcome.note.empty()) {
    if (!summary->notes.empty()) {
      summary->notes += "; ";
    }
    summary->notes += outcome.note;
  }
}

// 运行一个夹具类别；`expected_conflicts` 是文档化期望与冻结碰撞契约冲突
// 的用例数（见文件头）。unresolved 计数只记录、不断言，因为未来的规划器
// 改进可能解决它们。
ClassSummary run_class(const std::string& class_name, bool expect_reverse,
                       int expected_conflicts, const std::string& backend_id) {
  const common::VehicleModel vehicle = test_support::load_default_vehicle();
  ClassSummary summary;
  for (const PathCase& path_case : all_cases()) {
    if (path_case.case_class != class_name) {
      continue;
    }
    ++summary.executed;
    CaseOutcome outcome;
    run_case(path_case, vehicle, expect_reverse, backend_id, &outcome);
    accumulate(&summary, outcome);
  }
  EXPECT_EQ(summary.executed, class_name == "boundary" ? 4 : 5)
      << "unexpected fixture count in class " << class_name;
  EXPECT_EQ(summary.conflicts, expected_conflicts)
      << "fixture-conflict count changed for class " << class_name
      << ": " << summary.notes;
  return summary;
}

}  // namespace

TEST(HybridAstarFixtureCases, open_direct_succeeds) {
  const ClassSummary summary = run_class("open_direct", false, 0, "ros_free");
  EXPECT_EQ(summary.unresolved, 0);
  RecordProperty("fixture_conflicts", summary.conflicts);
  RecordProperty("unresolved_cases", summary.unresolved);
}

TEST(HybridAstarFixtureCases, single_obstacle_detour_succeeds) {
  const ClassSummary summary = run_class("single_obstacle_detour", false, 0, "ros_free");
  EXPECT_EQ(summary.unresolved, 0);
  RecordProperty("fixture_conflicts", summary.conflicts);
  RecordProperty("unresolved_cases", summary.unresolved);
}

TEST(HybridAstarFixtureCases, narrow_corridor_succeeds) {
  const ClassSummary summary = run_class("narrow_corridor", false, 0, "ros_free");
  EXPECT_EQ(summary.unresolved, 0);
  RecordProperty("fixture_conflicts", summary.conflicts);
  RecordProperty("unresolved_cases", summary.unresolved);
  RecordProperty("case_notes", summary.notes);
}

TEST(HybridAstarFixtureCases, reverse_parking_succeeds_with_reverse) {
  const ClassSummary summary = run_class("reverse_parking", true, 0, "ros_free");
  // v2 重新设计了每个停车位，使其对 4.9 m x 2.0 m 的包络可行；现在每个
  // 用例都必须可解。
  EXPECT_EQ(summary.unresolved, 0) << summary.notes;
  RecordProperty("fixture_conflicts", summary.conflicts);
  RecordProperty("unresolved_cases", summary.unresolved);
  RecordProperty("case_notes", summary.notes);
}

TEST(HybridAstarFixtureCases, dead_end_escape_succeeds_with_reverse) {
  const ClassSummary summary = run_class("dead_end_escape", true, 0, "ros_free");
  // v2 加深了凹槽出口，使倒车四分之一圆弧可以容纳；现在每个用例都必须
  // 可解。
  EXPECT_EQ(summary.unresolved, 0) << summary.notes;
  RecordProperty("fixture_conflicts", summary.conflicts);
  RecordProperty("unresolved_cases", summary.unresolved);
  RecordProperty("case_notes", summary.notes);
}

TEST(HybridAstarFixtureCases, unreachable_reports_no_path) {
  const ClassSummary summary = run_class("unreachable", false, 0, "ros_free");
  EXPECT_EQ(summary.unresolved, 0);
  RecordProperty("fixture_conflicts", summary.conflicts);
  RecordProperty("unresolved_cases", summary.unresolved);
  RecordProperty("case_notes", summary.notes);
}

TEST(HybridAstarFixtureCases, boundary_cases_report_documented_status) {
  const ClassSummary summary = run_class("boundary", false, 0, "ros_free");
  EXPECT_EQ(summary.unresolved, 0);
  RecordProperty("fixture_conflicts", summary.conflicts);
  RecordProperty("unresolved_cases", summary.unresolved);
  RecordProperty("case_notes", summary.notes);
}

// 为一个归一化候选后端运行完整的 v2 夹具集，要求每个类别通过且没有夹具
// 冲突或未解决用例。
void run_all_classes(const std::string& backend_id) {
  const std::vector<std::pair<std::string, bool>> classes = {
      {"open_direct", false},          {"single_obstacle_detour", false},
      {"narrow_corridor", false},      {"reverse_parking", true},
      {"dead_end_escape", true},       {"unreachable", false},
      {"boundary", false}};
  ClassSummary total;
  for (const auto& entry : classes) {
    SCOPED_TRACE("class " + entry.first);
    const ClassSummary summary =
        run_class(entry.first, entry.second, 0, backend_id);
    EXPECT_EQ(summary.unresolved, 0) << summary.notes;
    total.executed += summary.executed;
    total.conflicts += summary.conflicts;
    total.unresolved += summary.unresolved;
  }
  EXPECT_EQ(total.executed, 34);
  EXPECT_EQ(total.conflicts, 0);
  EXPECT_EQ(total.unresolved, 0) << total.notes;
}

TEST(HybridAstarFixtureCases, smac_normalized_v2_cases) {
  run_all_classes("smac_normalized");
}

TEST(HybridAstarFixtureCases, kth_normalized_v2_cases) {
  run_all_classes("kth_normalized");
}

TEST(HybridAstarFixtureCases, zm0612_normalized_v2_cases) {
  run_all_classes("zm0612_normalized");
}

}  // namespace planner::hybrid_astar
