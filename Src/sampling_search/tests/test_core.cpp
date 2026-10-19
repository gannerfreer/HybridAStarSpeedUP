// X-S4 核心契约测试：统一输入校验、沿边检查助手、路径助手、适配器注册表。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "fixture.hpp"
#include "planner/sampling_search/planner.hpp"
#include "planner/sampling_search/types.hpp"
#include "planner/sampling_search/validity.hpp"

namespace planner::sampling_search {
namespace {

using test::open_fixture;
using test::RectFixture;
using test::three_wall_fixture;

TEST(SamplingSearchCore, ValidateProblemAcceptsValidInput) {
  const RectFixture fixture = open_fixture();
  const Problem problem =
      fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0}, 0.5, 0);
  EXPECT_EQ(validate_problem(problem), "");
}

TEST(SamplingSearchCore, ValidateProblemRejectsMissingCallbacks) {
  const RectFixture fixture = open_fixture();
  Problem problem =
      fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0}, 0.5, 0);
  problem.point_valid = nullptr;
  EXPECT_NE(validate_problem(problem), "");
  problem = fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0},
                                 0.5, 0);
  problem.edge_valid = nullptr;
  EXPECT_NE(validate_problem(problem), "");
}

TEST(SamplingSearchCore, ValidateProblemRejectsBadNumbers) {
  const RectFixture fixture = open_fixture();
  Problem problem =
      fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0}, 0.5, 0);
  problem.budget_s = 0.0;
  EXPECT_NE(validate_problem(problem), "");
  problem = fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0},
                                 0.5, 0);
  problem.edge_check_step = -1.0;
  EXPECT_NE(validate_problem(problem), "");
  problem = fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0},
                                 0.5, 0);
  problem.bounds.max_x = std::nan("");
  EXPECT_NE(validate_problem(problem), "");
  problem = fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0},
                                 0.5, 0);
  problem.length_target = -1.0;
  EXPECT_NE(validate_problem(problem), "");
}

TEST(SamplingSearchCore, ValidateProblemRejectsInvalidEndpoints) {
  const RectFixture fixture = three_wall_fixture();
  Problem problem = fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0},
                                         0.5, 0);
  problem.start = Point2d{-1.0, 10.0};
  EXPECT_NE(validate_problem(problem), "");
  problem = fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0}, 0.5,
                                 0);
  problem.start = Point2d{35.0, 30.0};  // 位于障碍内
  EXPECT_NE(validate_problem(problem), "");
  problem = fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0}, 0.5,
                                 0);
  problem.goal = Point2d{35.0, 30.0};
  EXPECT_NE(validate_problem(problem), "");
}

TEST(SamplingSearchCore, SampledEdgeChecksAlongEdgeNotOnlyEndpoints) {
  const RectFixture fixture = three_wall_fixture();
  auto validity = std::make_shared<SampledEdgeValidity>(
      [fixture](const Point2d& p) { return fixture.point_valid(p); }, 1.0);
  // 两端点均有效，但线段穿过 (30,0)-(45,55) 障碍。
  const Point2d a{20.0, 30.0};
  const Point2d b{55.0, 30.0};
  EXPECT_TRUE(validity->is_point_valid(a));
  EXPECT_TRUE(validity->is_point_valid(b));
  EXPECT_FALSE(validity->is_edge_valid(a, b));
  // 同一线段上方（y=60）不穿障碍：有效。
  EXPECT_TRUE(validity->is_edge_valid(Point2d{20.0, 60.0}, Point2d{55.0, 60.0}));
}

TEST(SamplingSearchCore, SampledEdgeHandlesDegenerateEdge) {
  const RectFixture fixture = open_fixture();
  auto validity = std::make_shared<SampledEdgeValidity>(
      [fixture](const Point2d& p) { return fixture.point_valid(p); }, 1.0);
  const Point2d p{10.0, 10.0};
  EXPECT_TRUE(validity->is_edge_valid(p, p));
  const Point2d outside{-5.0, 10.0};
  EXPECT_FALSE(validity->is_edge_valid(outside, outside));
}

TEST(SamplingSearchCore, PolylineLengthMatchesManualSum) {
  const std::vector<Point2d> path = {
      Point2d{0.0, 0.0}, Point2d{3.0, 4.0}, Point2d{3.0, 4.0},
      Point2d{6.0, 8.0}};
  EXPECT_DOUBLE_EQ(polyline_length(path), 10.0);
  EXPECT_DOUBLE_EQ(polyline_length({}), 0.0);
  EXPECT_DOUBLE_EQ(polyline_length({Point2d{1.0, 1.0}}), 0.0);
}

TEST(SamplingSearchCore, PathEdgesValidDetectsInvalidEdge) {
  const RectFixture fixture = three_wall_fixture();
  const Problem problem =
      fixture.make_problem(Point2d{10.0, 10.0}, Point2d{90.0, 90.0}, 0.5, 0);
  EXPECT_TRUE(path_edges_valid(
      problem, {Point2d{10.0, 10.0}, Point2d{20.0, 20.0}, Point2d{25.0, 25.0}}));
  EXPECT_FALSE(path_edges_valid(
      problem, {Point2d{20.0, 30.0}, Point2d{55.0, 30.0}, Point2d{60.0, 35.0}}));
}

TEST(SamplingSearchCore, AvailableBackendsMatchAvailabilityFlags) {
  const std::vector<std::string> backends = available_backends();
  EXPECT_EQ(backends.size(), make_available_planners().size());
  EXPECT_EQ(ompl_rrtstar_available(),
            std::find(backends.begin(), backends.end(), "ompl_rrtstar") !=
                backends.end());
  EXPECT_EQ(ompl_informed_rrtstar_available(),
            std::find(backends.begin(), backends.end(),
                      "ompl_informed_rrtstar") != backends.end());
  EXPECT_EQ(ch0p1k3_rrtstar_available(),
            std::find(backends.begin(), backends.end(), "ch0p1k3_rrtstar") !=
                backends.end());
  EXPECT_EQ(klampt_rrtstar_available(),
            std::find(backends.begin(), backends.end(), "klampt_rrtstar") !=
                backends.end());
  EXPECT_EQ(vss2sn_rrtstar_available(),
            std::find(backends.begin(), backends.end(), "vss2sn_rrtstar") !=
                backends.end());
}

TEST(SamplingSearchCore, DisabledAdaptersReturnNullWhenNotCompiled) {
  if (!ompl_rrtstar_available()) {
    EXPECT_EQ(make_ompl_rrtstar(), nullptr);
    EXPECT_EQ(make_ompl_informed_rrtstar(), nullptr);
  }
  if (!ch0p1k3_rrtstar_available()) {
    EXPECT_EQ(make_ch0p1k3_rrtstar(), nullptr);
  }
  if (!klampt_rrtstar_available()) {
    EXPECT_EQ(make_klampt_rrtstar(), nullptr);
  }
  if (!vss2sn_rrtstar_available()) {
    EXPECT_EQ(make_vss2sn_rrtstar(), nullptr);
  }
  if (!aiwinter_rrtstar_available()) {
    EXPECT_EQ(make_aiwinter_rrtstar(), nullptr);
  }
}

}  // namespace
}  // namespace planner::sampling_search
