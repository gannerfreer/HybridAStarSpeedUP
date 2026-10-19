// 加速等价性：可选的包围体/空间索引路径必须与朴素几何路径完全一致（状态、原因、
// 首次接触位姿与弧长），因为每个被跳过的对象都可证明比查询包络更远。

#include <gtest/gtest.h>

#include "planner/collision/differential.hpp"

namespace planner::collision {
namespace {

TEST(AccelerationEquivalence, generated_cases_match_plain_results) {
  const differential::RegimeSpec regime = differential::dev_regime_spec();
  const std::vector<differential::CaseAllocation> allocation =
      differential::build_regime_allocation(regime);
  CheckOptions plain;
  plain.enable_acceleration = false;
  CheckOptions accelerated;
  accelerated.enable_acceleration = true;
  int checked = 0;
  for (std::size_t i = 0; i < allocation.size() && checked < 300; i += 17) {
    const differential::DifferentialCase test_case =
        differential::generate_case(regime, allocation[i],
                                    static_cast<int>(i), 0);
    const CheckResult expected = differential::production_check(test_case, plain);
    const CheckResult actual =
        differential::production_check(test_case, accelerated);
    EXPECT_EQ(expected.status, actual.status) << test_case.case_id;
    EXPECT_EQ(expected.reason_code, actual.reason_code) << test_case.case_id;
    EXPECT_DOUBLE_EQ(expected.first_s_m, actual.first_s_m) << test_case.case_id;
    EXPECT_DOUBLE_EQ(expected.first_pose.x_m, actual.first_pose.x_m)
        << test_case.case_id;
    EXPECT_DOUBLE_EQ(expected.first_pose.y_m, actual.first_pose.y_m)
        << test_case.case_id;
    EXPECT_EQ(expected.is_valid(), actual.is_valid()) << test_case.case_id;
    ++checked;
  }
  EXPECT_GE(checked, 300);
}

}  // namespace
}  // namespace planner::collision
