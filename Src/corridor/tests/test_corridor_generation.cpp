// 第一版安全走廊生成器的场景测试。
//
// 确定性场景镜像 tests/fixtures/v1/paths/cases.json 的几何（狭窄走廊墙、
// 障碍物行、unreachable_128 的 1.5 m 窄缝），但不加载该夹具，因为它只含
// 起点/终点元数据而没有路径点。每个成功的走廊还会用
// corridor_test_support.hpp 中的测试本地独立几何做额外检查。

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "corridor_test_support.hpp"
#include "planner/common/serialization.hpp"
#include "planner/common/types.hpp"
#include "planner/corridor/corridor_generator.hpp"
#include "planner/corridor/geometry.hpp"

namespace planner::corridor {
namespace {

using common::Corridor;
using common::Status;
using corridor_test::arc_seed;
using corridor_test::check_corridor;
using corridor_test::make_request;
using corridor_test::straight_seed;
using corridor_test::test_vehicle;

TEST(CorridorGenerationTest, empty_seed_is_rejected_as_invalid_input) {
  CorridorRequest request = make_request({}, {});
  const CorridorResult result = generate_corridor(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status, Status::kInvalidInput);
  EXPECT_EQ(result.failure, CorridorFailure::kInvalidInput);
  EXPECT_EQ(result.reason, "invalid_input");
  EXPECT_FALSE(result.detail.empty());
  EXPECT_TRUE(result.corridor.polygons.empty());
}

TEST(CorridorGenerationTest, single_point_seed_is_rejected_as_invalid_input) {
  common::PathSegment segment;
  segment.points.push_back(corridor_test::make_point(0.0, 0.0, 0.0, 0.0));
  CorridorRequest request = make_request({segment}, {});
  const CorridorResult result = generate_corridor(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status, Status::kInvalidInput);
  EXPECT_EQ(result.reason, "invalid_input");
}

TEST(CorridorGenerationTest, non_monotonic_seed_arc_length_is_rejected) {
  common::PathSegment segment;
  segment.points.push_back(corridor_test::make_point(0.0, 0.0, 0.0, 1.0));
  segment.points.push_back(corridor_test::make_point(1.0, 0.0, 0.0, 0.5));
  CorridorRequest request = make_request({segment}, {});
  const CorridorResult result = generate_corridor(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status, Status::kInvalidInput);
  EXPECT_EQ(result.reason, "invalid_input");
}

TEST(CorridorGenerationTest, invalid_vehicle_dimensions_are_rejected) {
  CorridorRequest request =
      make_request({straight_seed(0.0, 10.0, 0.0, 0.0)}, {});
  request.vehicle.width_m = 0.0;
  const CorridorResult result = generate_corridor(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status, Status::kInvalidInput);
  EXPECT_EQ(result.reason, "invalid_input");
}

TEST(CorridorGenerationTest, invalid_options_are_rejected) {
  CorridorRequest request =
      make_request({straight_seed(0.0, 10.0, 0.0, 0.0)}, {});
  request.options.sample_step_m = 0.0;
  const CorridorResult result = generate_corridor(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status, Status::kInvalidInput);
  EXPECT_EQ(result.reason, "invalid_input");
}

TEST(CorridorGenerationTest, non_convex_obstacle_is_rejected) {
  // L 形（非凸）障碍物。
  const Obstacle obstacle = make_convex_obstacle(
      "l_shape", {common::Point2d{0.0, 0.0}, common::Point2d{2.0, 0.0},
                  common::Point2d{2.0, 1.0}, common::Point2d{1.0, 1.0},
                  common::Point2d{1.0, 2.0}, common::Point2d{0.0, 2.0}});
  CorridorRequest request = make_request({straight_seed(0.0, 10.0, 5.0, 0.0)}, {obstacle});
  const CorridorResult result = generate_corridor(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status, Status::kInvalidInput);
  EXPECT_EQ(result.reason, "invalid_input");
}

TEST(CorridorGenerationTest, degenerate_obstacle_is_rejected) {
  const Obstacle obstacle = make_rectangle_obstacle("degenerate", 4.0, 4.0, 4.0, 6.0);
  CorridorRequest request = make_request({straight_seed(0.0, 10.0, 5.0, 0.0)}, {obstacle});
  const CorridorResult result = generate_corridor(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status, Status::kInvalidInput);
  EXPECT_EQ(result.reason, "invalid_input");
}

TEST(CorridorGenerationTest, straight_path_generates_ordered_convex_corridor) {
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 20.0, 5.0, 0.0)};
  CorridorRequest request = make_request(seed, {});
  const CorridorResult result = generate_corridor(request);
  ASSERT_TRUE(result.ok()) << result.reason << ": " << result.detail;

  const Corridor& corridor = result.corridor;
  EXPECT_EQ(corridor.id, "test:corridor");
  EXPECT_EQ(corridor.map_id, "test_map");
  EXPECT_EQ(corridor.map_version, "v1");
  EXPECT_EQ(corridor.frame_id, "map");
  ASSERT_GE(corridor.polygons.size(), 2u);
  EXPECT_EQ(result.stats.polygon_count, corridor.polygons.size());

  for (std::size_t i = 0; i < corridor.polygons.size(); ++i) {
    const common::CorridorPolygon& polygon = corridor.polygons[i];
    EXPECT_GE(polygon.vertices.size(), 3u);
    EXPECT_GT(corridor_test::local_area(polygon.vertices), 1e-9);
    EXPECT_GT(polygon.s_end_m, polygon.s_start_m);
    if (i > 0) {
      EXPECT_GE(polygon.s_start_m, corridor.polygons[i - 1].s_start_m);
      EXPECT_LE(polygon.s_start_m, corridor.polygons[i - 1].s_end_m + 1e-6);
    }
  }
  EXPECT_NEAR(corridor.polygons.front().s_start_m, 0.0, 1e-9);
  EXPECT_NEAR(corridor.polygons.back().s_end_m, 20.0, 1e-9);

  const CorridorVerification verification =
      verify_corridor(corridor, seed, request.obstacles, request.vehicle);
  EXPECT_TRUE(verification.valid) << verification.reason << ": " << verification.detail;

  const corridor_test::LocalCheckResult local =
      check_corridor(corridor, seed, request.obstacles, request.vehicle);
  EXPECT_TRUE(local.ok) << local.message;
}

TEST(CorridorGenerationTest, arc_path_generates_continuous_corridor) {
  const std::vector<common::PathSegment> seed = {arc_seed(8.0, -0.5 * common::kPi,
                                                          0.5 * common::kPi)};
  CorridorRequest request = make_request(seed, {});
  const CorridorResult result = generate_corridor(request);
  ASSERT_TRUE(result.ok()) << result.reason << ": " << result.detail;

  const CorridorVerification verification =
      verify_corridor(result.corridor, seed, request.obstacles, request.vehicle);
  EXPECT_TRUE(verification.valid) << verification.reason << ": " << verification.detail;

  const corridor_test::LocalCheckResult local =
      check_corridor(result.corridor, seed, request.obstacles, request.vehicle);
  EXPECT_TRUE(local.ok) << local.message;
}

TEST(CorridorGenerationTest, reverse_segment_path_is_supported) {
  common::PathSegment forward = corridor_test::straight_seed(0.0, 6.0, 5.0, 0.0);
  common::PathSegment reverse;
  reverse.points.push_back(
      corridor_test::make_point(6.0, 5.0, 0.0, 6.0, common::Direction::kReverse));
  reverse.points.push_back(
      corridor_test::make_point(2.5, 5.0, 0.0, 9.5, common::Direction::kReverse));
  const std::vector<common::PathSegment> seed = {forward, reverse};
  CorridorRequest request = make_request(seed, {});
  const CorridorResult result = generate_corridor(request);
  ASSERT_TRUE(result.ok()) << result.reason << ": " << result.detail;

  const CorridorVerification verification =
      verify_corridor(result.corridor, seed, request.obstacles, request.vehicle);
  EXPECT_TRUE(verification.valid) << verification.reason << ": " << verification.detail;
}

TEST(CorridorGenerationTest, channel_with_two_walls_keeps_clearance) {
  // 镜像 tests/fixtures/v1/paths/cases.json 中 3.0 m 的狭窄走廊
  // （corridor_111）：墙位于 x = 6 到 16，路径居中于 y = 5。
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 22.0, 5.0, 0.0)};
  const std::vector<Obstacle> obstacles = {
      make_rectangle_obstacle("wall_lower", 6.0, 0.0, 16.0, 3.5),
      make_rectangle_obstacle("wall_upper", 6.0, 6.5, 16.0, 10.0)};
  CorridorRequest request = make_request(seed, obstacles);
  const CorridorResult result = generate_corridor(request);
  ASSERT_TRUE(result.ok()) << result.reason << ": " << result.detail;

  const CorridorVerification verification =
      verify_corridor(result.corridor, seed, obstacles, request.vehicle);
  EXPECT_TRUE(verification.valid) << verification.reason << ": " << verification.detail;

  const corridor_test::LocalCheckResult local =
      check_corridor(result.corridor, seed, obstacles, request.vehicle);
  EXPECT_TRUE(local.ok) << local.message;

  // 走廊穿过通道中部（探测线 x = 11）时严格保持在两墙之间；通道外的
  // 多边形可以扩张（局部逐边扩张，corridor-001/002）。
  for (const common::CorridorPolygon& polygon : result.corridor.polygons) {
    const corridor_test::LocalYRange range =
        corridor_test::local_y_range_at_x(polygon.vertices, 11.0);
    if (!range.exists) {
      continue;
    }
    EXPECT_GT(range.min_y, 3.4);
    EXPECT_LT(range.max_y, 6.6);
  }
}

TEST(CorridorGenerationTest, single_obstacle_row_keeps_clearance) {
  // 路径下侧的一排障碍物：走廊必须在障碍物行处止住，并让每个障碍物
  // 保持在外部。
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 24.0, 5.0, 0.0)};
  const std::vector<Obstacle> obstacles = {
      make_rectangle_obstacle("row_a", 4.0, 2.0, 8.0, 3.0),
      make_rectangle_obstacle("row_b", 9.0, 2.0, 14.0, 3.0),
      make_rectangle_obstacle("row_c", 15.0, 2.0, 20.0, 3.0)};
  CorridorRequest request = make_request(seed, obstacles);
  const CorridorResult result = generate_corridor(request);
  ASSERT_TRUE(result.ok()) << result.reason << ": " << result.detail;

  const CorridorVerification verification =
      verify_corridor(result.corridor, seed, obstacles, request.vehicle);
  EXPECT_TRUE(verification.valid) << verification.reason << ": " << verification.detail;

  const corridor_test::LocalCheckResult local =
      check_corridor(result.corridor, seed, obstacles, request.vehicle);
  EXPECT_TRUE(local.ok) << local.message;

  // 走廊穿过障碍物行（探测线 x = 12，位于 row_b 内）时保持在行上方；
  // 行外的多边形可以向下扩张（局部逐边扩张，corridor-001/002）。
  for (const common::CorridorPolygon& polygon : result.corridor.polygons) {
    const corridor_test::LocalYRange range =
        corridor_test::local_y_range_at_x(polygon.vertices, 12.0);
    if (!range.exists) {
      continue;
    }
    EXPECT_GE(range.min_y, 3.0);
  }
}

TEST(CorridorGenerationTest, multiple_obstacle_rows_keep_clearance) {
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 24.0, 5.0, 0.0)};
  const std::vector<Obstacle> obstacles = {
      make_rectangle_obstacle("lower_a", 4.0, 2.0, 9.0, 3.2),
      make_rectangle_obstacle("lower_b", 11.0, 2.0, 17.0, 3.2),
      make_rectangle_obstacle("upper_a", 6.0, 6.8, 12.0, 8.0),
      make_rectangle_obstacle("upper_b", 14.0, 6.8, 19.0, 8.0)};
  CorridorRequest request = make_request(seed, obstacles);
  const CorridorResult result = generate_corridor(request);
  ASSERT_TRUE(result.ok()) << result.reason << ": " << result.detail;

  const corridor_test::LocalCheckResult local =
      check_corridor(result.corridor, seed, obstacles, request.vehicle);
  EXPECT_TRUE(local.ok) << local.message;
}

TEST(CorridorGenerationTest, obstacle_crossing_path_reports_blocked) {
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 20.0, 5.0, 0.0)};
  const std::vector<Obstacle> obstacles = {
      make_rectangle_obstacle("blocker", 8.0, 4.0, 12.0, 6.0)};
  CorridorRequest request = make_request(seed, obstacles);
  const CorridorResult result = generate_corridor(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status, Status::kNoPath);
  EXPECT_EQ(result.failure, CorridorFailure::kBlocked);
  EXPECT_EQ(result.reason, to_string(result.failure));
  EXPECT_FALSE(result.detail.empty());
  EXPECT_TRUE(result.corridor.polygons.empty());
}

TEST(CorridorGenerationTest, obstacle_touching_path_centerline_reports_blocked) {
  // 障碍物上边缘位于路径中心线上（接触即算）。
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 20.0, 5.0, 0.0)};
  const std::vector<Obstacle> obstacles = {
      make_rectangle_obstacle("touch", 8.0, 4.0, 12.0, 5.0)};
  CorridorRequest request = make_request(seed, obstacles);
  const CorridorResult result = generate_corridor(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status, Status::kNoPath);
  EXPECT_EQ(result.failure, CorridorFailure::kBlocked);
  EXPECT_TRUE(result.corridor.polygons.empty());
}

TEST(CorridorGenerationTest, narrow_slit_smaller_than_vehicle_reports_narrow) {
  // 镜像 unreachable_128：1.5 m 的窄缝，小于 1.8 m 的车宽。
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 20.0, 5.0, 0.0)};
  const std::vector<Obstacle> obstacles = {
      make_rectangle_obstacle("slit_lower", 8.0, 0.0, 10.0, 4.25),
      make_rectangle_obstacle("slit_upper", 8.0, 5.75, 10.0, 11.0)};
  CorridorRequest request = make_request(seed, obstacles);
  const CorridorResult result = generate_corridor(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status, Status::kNoPath);
  EXPECT_EQ(result.failure, CorridorFailure::kNarrow);
  EXPECT_EQ(result.reason, "narrow");
  EXPECT_TRUE(result.corridor.polygons.empty());
}

TEST(CorridorGenerationTest, gap_wider_than_vehicle_plus_margin_succeeds) {
  // 2.2 m 的间距宽于 1.8 m + 2 x 0.1 m 裕度。
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 20.0, 5.0, 0.0)};
  const std::vector<Obstacle> obstacles = {
      make_rectangle_obstacle("slit_lower", 8.0, 0.0, 10.0, 3.9),
      make_rectangle_obstacle("slit_upper", 8.0, 6.1, 10.0, 11.0)};
  CorridorRequest request = make_request(seed, obstacles);
  const CorridorResult result = generate_corridor(request);
  ASSERT_TRUE(result.ok()) << result.reason << ": " << result.detail;

  const CorridorVerification verification =
      verify_corridor(result.corridor, seed, obstacles, request.vehicle);
  EXPECT_TRUE(verification.valid) << verification.reason << ": " << verification.detail;

  const corridor_test::LocalCheckResult local =
      check_corridor(result.corridor, seed, obstacles, request.vehicle);
  EXPECT_TRUE(local.ok) << local.message;
}

TEST(CorridorGenerationTest, detour_around_centreline_obstacle_keeps_usable_width) {
  // 镜像 Dftpav 的 “avoid” 夹具（Src/dftpav/tests/test_corridor_avoidance.cpp）：
  // 绕中心线障碍物的平滑 12 m 圆角绕行。在 corridor-001 修复前，只要障碍物
  // 的 y 面落在足迹支撑线上，全局半平面上限就会把该走廊压缩到恰好 2.0 m
  // 的足迹宽度；局部逐边扩张必须保持其可用。
  const common::VehicleModel vehicle = test_vehicle();
  const std::vector<common::PathSegment> seed = {
      corridor_test::fillet_seed(
          {common::Point2d{0.0, 0.0}, common::Point2d{4.0, 0.0},
           common::Point2d{16.0, 6.0}, common::Point2d{28.0, 0.0},
           common::Point2d{34.0, 0.0}},
          12.0, 0.1)};
  const std::vector<Obstacle> obstacles = {
      make_rectangle_obstacle("centre_block", 12.0, -1.0, 14.0, 1.0)};
  CorridorRequest request = make_request(seed, obstacles);
  const CorridorResult result = generate_corridor(request);
  ASSERT_TRUE(result.ok()) << result.reason << ": " << result.detail;

  const CorridorVerification verification =
      verify_corridor(result.corridor, seed, obstacles, request.vehicle);
  EXPECT_TRUE(verification.valid) << verification.reason << ": " << verification.detail;

  const corridor_test::LocalCheckResult local =
      check_corridor(result.corridor, seed, obstacles, request.vehicle);
  EXPECT_TRUE(local.ok) << local.message;

  const double footprint_width = vehicle.width_m + 2.0 * vehicle.safety_margin_m;
  for (const common::CorridorPolygon& polygon : result.corridor.polygons) {
    const double width = convex_polygon_min_width(polygon.vertices);
    // 任何位置都不应出现刀锋状多边形；旧行为在该场景中恰好为足迹宽度。
    EXPECT_GE(width, footprint_width + 0.5);
    // 绕行前的直线接近段与绕行区域本身都保持充裕的可用宽度。
    if (polygon.s_start_m <= 5.0) {
      EXPECT_GE(width, footprint_width + 4.0);
    }
    if (polygon.s_start_m >= 8.0 && polygon.s_start_m <= 16.0) {
      EXPECT_GE(width, footprint_width + 0.75);
    }
  }
  EXPECT_GT(result.stats.obstacle_limited_edges, 0);
  EXPECT_GE(result.stats.min_polygon_width_m, footprint_width + 0.5);
}

TEST(CorridorGenerationTest, tight_pocket_corridor_contains_perturbed_footprints) {
  // corridor-002：v2 的 deadend_121 凹槽，种子紧贴右墙（Hybrid A* 规划
  // 可能偏离中心）。走廊必须在 +/-1 度航向和 +/-2 cm 位置扰动下仍包含
  // 保守整车足迹；检查只使用独立的测试本地几何。
  const common::VehicleModel vehicle = test_vehicle();
  const std::vector<Obstacle> obstacles = {
      make_rectangle_obstacle("pocket_left_121", 5.2, 10.4, 6.0, 17.0),
      make_rectangle_obstacle("pocket_right_121", 9.0, 10.4, 9.8, 17.0),
      make_rectangle_obstacle("pocket_back_121", 5.2, 16.2, 9.8, 17.0)};
  common::PathSegment reverse;
  const double x = 7.9;         // 距凹槽中心 0.4 m，偏向墙壁一侧
  const double y_start = 12.2;  // 夹具的起始位姿
  const double y_end = 6.4;     // 从开口直线倒车驶出
  const int count = static_cast<int>(std::ceil((y_start - y_end) / 0.1));
  for (int k = 0; k <= count; ++k) {
    const double t = static_cast<double>(k) / static_cast<double>(count);
    const double y = y_start + (y_end - y_start) * t;
    reverse.points.push_back(corridor_test::make_point(
        x, y, 0.5 * common::kPi, y_start - y, common::Direction::kReverse));
  }
  const std::vector<common::PathSegment> seed = {reverse};
  CorridorRequest request = make_request(seed, obstacles);
  const CorridorResult result = generate_corridor(request);
  ASSERT_TRUE(result.ok()) << result.reason << ": " << result.detail;

  const CorridorVerification verification =
      verify_corridor(result.corridor, seed, obstacles, request.vehicle);
  EXPECT_TRUE(verification.valid) << verification.reason << ": " << verification.detail;
  const corridor_test::LocalCheckResult local =
      check_corridor(result.corridor, seed, obstacles, request.vehicle);
  EXPECT_TRUE(local.ok) << local.message;

  // 对文档所述扰动界限的鲁棒性。
  const corridor_test::LocalCheckResult perturbed = corridor_test::check_perturbed_footprints(
      result.corridor, seed, vehicle, 1.0 * common::kPi / 180.0, 0.02);
  EXPECT_TRUE(perturbed.ok) << perturbed.message;
}

TEST(CorridorGenerationTest, corridor_json_roundtrip_preserves_generated_corridor) {
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 12.0, 5.0, 0.0)};
  CorridorRequest request = make_request(seed, {});
  const CorridorResult result = generate_corridor(request);
  ASSERT_TRUE(result.ok()) << result.reason << ": " << result.detail;

  const std::string encoded = common::encode_document(result.corridor);
  const Corridor decoded = common::decode_document<Corridor>(encoded);
  EXPECT_TRUE(decoded == result.corridor);
}

TEST(CorridorGenerationTest, generation_is_deterministic) {
  const std::vector<common::PathSegment> seed = {
      arc_seed(6.0, 0.0, 0.4 * common::kPi)};
  const std::vector<Obstacle> obstacles = {
      make_rectangle_obstacle("side", 2.0, 8.0, 8.0, 9.0)};
  CorridorRequest request = make_request(seed, obstacles);
  const CorridorResult first = generate_corridor(request);
  const CorridorResult second = generate_corridor(request);
  ASSERT_TRUE(first.ok()) << first.reason << ": " << first.detail;
  ASSERT_TRUE(second.ok()) << second.reason << ": " << second.detail;
  EXPECT_TRUE(first.corridor == second.corridor);
}

TEST(CorridorGenerationTest, generated_corridors_pass_independent_geometric_checks) {
  struct Scenario {
    std::vector<common::PathSegment> seed;
    std::vector<Obstacle> obstacles;
  };
  std::vector<Scenario> scenarios;
  scenarios.push_back(Scenario{{straight_seed(0.0, 18.0, 5.0, 0.0)},
                               {make_rectangle_obstacle("left", 6.0, 1.0, 14.0, 3.0)}});
  scenarios.push_back(Scenario{{arc_seed(10.0, 0.0, 0.6 * common::kPi)},
                               {make_rectangle_obstacle("inner", 0.5, 3.0, 3.0, 5.0)}});
  scenarios.push_back(Scenario{
      {straight_seed(0.0, 20.0, 5.0, 0.0)},
      {make_rectangle_obstacle("wall_lower", 5.0, 0.0, 15.0, 3.6),
       make_rectangle_obstacle("wall_upper", 5.0, 6.4, 15.0, 10.0)}});

  for (std::size_t i = 0; i < scenarios.size(); ++i) {
    CorridorRequest request = make_request(scenarios[i].seed, scenarios[i].obstacles);
    const CorridorResult result = generate_corridor(request);
    ASSERT_TRUE(result.ok()) << "scenario " << i << ": " << result.reason << ": "
                             << result.detail;
    const corridor_test::LocalCheckResult local =
        check_corridor(result.corridor, scenarios[i].seed, scenarios[i].obstacles,
                       request.vehicle);
    EXPECT_TRUE(local.ok) << "scenario " << i << ": " << local.message;
  }
}

TEST(CorridorGenerationTest, failure_reason_names_are_stable) {
  EXPECT_STREQ(to_string(CorridorFailure::kNone), "none");
  EXPECT_STREQ(to_string(CorridorFailure::kInvalidInput), "invalid_input");
  EXPECT_STREQ(to_string(CorridorFailure::kBlocked), "blocked");
  EXPECT_STREQ(to_string(CorridorFailure::kNarrow), "narrow");
  EXPECT_STREQ(to_string(CorridorFailure::kNoCorridor), "no_corridor");
}

}  // namespace
}  // namespace planner::corridor
