// 进程内差分冒烟：生产实现与独立参考在生成用例的确定性子集上对比。零漏检是硬性
// 门槛；误报被计数（允许保守判定）。

#include <gtest/gtest.h>

#include <string>

#include "planner/collision/differential.hpp"

namespace planner::collision {
namespace {

TEST(DifferentialSmoke, generated_subset_has_no_misses) {
  const differential::RegimeSpec regime = differential::dev_regime_spec();
  const std::vector<differential::CaseAllocation> allocation =
      differential::build_regime_allocation(regime);
  int decidable = 0;
  int misses = 0;
  int false_positives = 0;
  int inconclusive = 0;
  std::string first_miss;
  for (std::size_t i = 0; i < allocation.size() && decidable < 150; i += 31) {
    const differential::DifferentialCase test_case =
        differential::generate_case(regime, allocation[i],
                                    static_cast<int>(i), 0);
    const ReferenceResult reference = differential::reference_check(test_case);
    ASSERT_NE(reference.verdict, ReferenceVerdict::kInvalidInput)
        << test_case.case_id;
    if (reference.verdict == ReferenceVerdict::kInconclusive) {
      ++inconclusive;
      continue;
    }
    const CheckResult production =
        differential::production_check(test_case, CheckOptions{});
    ASSERT_NE(production.status, CheckStatus::kInvalidInput)
        << test_case.case_id;
    const bool reference_collision =
        reference.verdict == ReferenceVerdict::kCollision;
    if (production.collided() && !reference_collision) {
      ++false_positives;
    }
    if (!production.collided() && reference_collision) {
      ++misses;
      if (first_miss.empty()) {
        first_miss = test_case.case_id;
      }
    }
    ++decidable;
  }
  EXPECT_GE(decidable, 150) << "inconclusive=" << inconclusive;
  EXPECT_EQ(misses, 0) << "first miss: " << first_miss;
  // 允许保守误报，但预期应很少。
  EXPECT_LT(false_positives, 10) << "false positives: " << false_positives;
}

}  // namespace
}  // namespace planner::collision
