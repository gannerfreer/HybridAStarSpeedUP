// 验证测试：`verify_corridor` 必须接受有效走廊（生成的或手工构造的），
// 并对每种结构/几何违规给出稳定原因码并拒绝。手工构造的走廊使用
// y = 5 m 处 0..10 m 的直线种子，便于跟踪预期的包含性检查。

#include <gtest/gtest.h>

#include <algorithm>
#include <utility>
#include <vector>

#include "corridor_test_support.hpp"
#include "planner/common/types.hpp"
#include "planner/corridor/corridor_generator.hpp"

namespace planner::corridor {
namespace {

using common::Corridor;
using common::CorridorPolygon;
using common::Point2d;
using corridor_test::straight_seed;
using corridor_test::test_vehicle;

CorridorPolygon rectangle_polygon(double s_start_m, double s_end_m, double x_min_m,
                                  double x_max_m, double y_min_m, double y_max_m) {
  CorridorPolygon polygon;
  polygon.s_start_m = s_start_m;
  polygon.s_end_m = s_end_m;
  polygon.vertices = {Point2d{x_min_m, y_min_m}, Point2d{x_max_m, y_min_m},
                      Point2d{x_max_m, y_max_m}, Point2d{x_min_m, y_max_m}};
  return polygon;
}

Corridor make_corridor(std::vector<CorridorPolygon> polygons) {
  Corridor corridor;
  corridor.id = "test_corridor";
  corridor.map_id = "test_map";
  corridor.map_version = "v1";
  corridor.frame_id = "map";
  corridor.polygons = std::move(polygons);
  return corridor;
}

// 覆盖 y = 5 m 处 0..10 m 直线种子保守足迹的手工走廊。两个多边形宽
// 2.4 m（车宽 1.8 m 加 2 x 0.1 m 裕度），在 x = 5 到 9 之间重叠。
std::vector<CorridorPolygon> covering_polygons() {
  return {rectangle_polygon(0.0, 5.0, -2.1, 9.0, 3.8, 6.2),
          rectangle_polygon(5.0, 10.0, 3.9, 14.0, 3.8, 6.2)};
}

TEST(CorridorVerificationTest, accepts_valid_handcrafted_corridor) {
  const Corridor corridor = make_corridor(covering_polygons());
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 10.0, 5.0, 0.0)};
  const CorridorVerification verification =
      verify_corridor(corridor, seed, {}, test_vehicle());
  EXPECT_TRUE(verification.valid) << verification.reason << ": " << verification.detail;
  EXPECT_TRUE(verification.reason.empty());
}

TEST(CorridorVerificationTest, accepts_generated_corridor) {
  CorridorRequest request =
      corridor_test::make_request({straight_seed(0.0, 16.0, 5.0, 0.0)}, {});
  const CorridorResult result = generate_corridor(request);
  ASSERT_TRUE(result.ok()) << result.reason << ": " << result.detail;
  const CorridorVerification verification =
      verify_corridor(result.corridor, request.seed, request.obstacles, request.vehicle);
  EXPECT_TRUE(verification.valid) << verification.reason << ": " << verification.detail;
}

TEST(CorridorVerificationTest, rejects_empty_corridor) {
  const Corridor corridor = make_corridor({});
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 10.0, 5.0, 0.0)};
  const CorridorVerification verification =
      verify_corridor(corridor, seed, {}, test_vehicle());
  EXPECT_FALSE(verification.valid);
  EXPECT_EQ(verification.reason, "structural");
}

TEST(CorridorVerificationTest, rejects_missing_map_id) {
  Corridor corridor = make_corridor(covering_polygons());
  corridor.map_id.clear();
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 10.0, 5.0, 0.0)};
  const CorridorVerification verification =
      verify_corridor(corridor, seed, {}, test_vehicle());
  EXPECT_FALSE(verification.valid);
  EXPECT_EQ(verification.reason, "structural");
}

TEST(CorridorVerificationTest, rejects_polygon_with_two_vertices) {
  std::vector<CorridorPolygon> polygons = covering_polygons();
  polygons[0].vertices = {Point2d{0.0, 3.8}, Point2d{5.0, 6.2}};
  const Corridor corridor = make_corridor(std::move(polygons));
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 10.0, 5.0, 0.0)};
  const CorridorVerification verification =
      verify_corridor(corridor, seed, {}, test_vehicle());
  EXPECT_FALSE(verification.valid);
  EXPECT_EQ(verification.reason, "polygon_not_convex_ccw");
}

TEST(CorridorVerificationTest, rejects_clockwise_polygon) {
  std::vector<CorridorPolygon> polygons = covering_polygons();
  std::reverse(polygons[1].vertices.begin(), polygons[1].vertices.end());
  const Corridor corridor = make_corridor(std::move(polygons));
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 10.0, 5.0, 0.0)};
  const CorridorVerification verification =
      verify_corridor(corridor, seed, {}, test_vehicle());
  EXPECT_FALSE(verification.valid);
  EXPECT_EQ(verification.reason, "polygon_not_convex_ccw");
}

TEST(CorridorVerificationTest, rejects_non_convex_polygon) {
  std::vector<CorridorPolygon> polygons = covering_polygons();
  polygons[0].vertices = {Point2d{0.0, 0.0}, Point2d{4.0, 0.0}, Point2d{4.0, 1.0},
                          Point2d{1.0, 1.0}, Point2d{1.0, 4.0}, Point2d{0.0, 4.0}};
  const Corridor corridor = make_corridor(std::move(polygons));
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 10.0, 5.0, 0.0)};
  const CorridorVerification verification =
      verify_corridor(corridor, seed, {}, test_vehicle());
  EXPECT_FALSE(verification.valid);
  EXPECT_EQ(verification.reason, "polygon_not_convex_ccw");
}

TEST(CorridorVerificationTest, rejects_negative_s_start) {
  std::vector<CorridorPolygon> polygons = covering_polygons();
  polygons[0].s_start_m = -1.0;
  const Corridor corridor = make_corridor(std::move(polygons));
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 10.0, 5.0, 0.0)};
  const CorridorVerification verification =
      verify_corridor(corridor, seed, {}, test_vehicle());
  EXPECT_FALSE(verification.valid);
  EXPECT_EQ(verification.reason, "negative_s_start");
}

TEST(CorridorVerificationTest, rejects_unordered_polygons) {
  std::vector<CorridorPolygon> polygons = covering_polygons();
  std::swap(polygons[0], polygons[1]);
  const Corridor corridor = make_corridor(std::move(polygons));
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 10.0, 5.0, 0.0)};
  const CorridorVerification verification =
      verify_corridor(corridor, seed, {}, test_vehicle());
  EXPECT_FALSE(verification.valid);
  EXPECT_EQ(verification.reason, "unordered_polygons");
}

TEST(CorridorVerificationTest, rejects_coverage_gap_between_polygons) {
  const Corridor corridor = make_corridor(
      {rectangle_polygon(0.0, 5.0, -2.1, 9.0, 3.8, 6.2),
       rectangle_polygon(7.0, 10.0, 5.9, 14.0, 3.8, 6.2)});
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 10.0, 5.0, 0.0)};
  const CorridorVerification verification =
      verify_corridor(corridor, seed, {}, test_vehicle());
  EXPECT_FALSE(verification.valid);
  EXPECT_EQ(verification.reason, "coverage_gap");
}

TEST(CorridorVerificationTest, rejects_non_overlapping_neighbours) {
  const Corridor corridor = make_corridor(
      {rectangle_polygon(0.0, 5.0, -2.1, 9.0, 3.8, 6.2),
       rectangle_polygon(5.0, 10.0, 100.0, 110.0, 3.8, 6.2)});
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 10.0, 5.0, 0.0)};
  const CorridorVerification verification =
      verify_corridor(corridor, seed, {}, test_vehicle());
  EXPECT_FALSE(verification.valid);
  EXPECT_EQ(verification.reason, "no_overlap");
}

TEST(CorridorVerificationTest, rejects_corridor_narrower_than_vehicle) {
  const Corridor corridor =
      make_corridor({rectangle_polygon(0.0, 10.0, -2.1, 14.0, 4.75, 5.25)});
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 10.0, 5.0, 0.0)};
  const CorridorVerification verification =
      verify_corridor(corridor, seed, {}, test_vehicle());
  EXPECT_FALSE(verification.valid);
  EXPECT_EQ(verification.reason, "narrow");
}

TEST(CorridorVerificationTest, rejects_footprint_not_contained) {
  const Corridor corridor =
      make_corridor({rectangle_polygon(0.0, 10.0, -2.1, 14.0, 3.8, 6.2)});
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 10.0, 50.0, 0.0)};
  const CorridorVerification verification =
      verify_corridor(corridor, seed, {}, test_vehicle());
  EXPECT_FALSE(verification.valid);
  EXPECT_EQ(verification.reason, "containment");
}

TEST(CorridorVerificationTest, rejects_obstacle_inside_corridor) {
  const Corridor corridor = make_corridor(covering_polygons());
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 10.0, 5.0, 0.0)};
  const std::vector<Obstacle> obstacles = {
      make_rectangle_obstacle("inside", 6.0, 4.5, 8.0, 5.5)};
  const CorridorVerification verification =
      verify_corridor(corridor, seed, obstacles, test_vehicle());
  EXPECT_FALSE(verification.valid);
  EXPECT_EQ(verification.reason, "obstacle_intersection");
}

TEST(CorridorVerificationTest, rejects_degenerate_seed) {
  const Corridor corridor = make_corridor(covering_polygons());
  const CorridorVerification verification = verify_corridor(corridor, {}, {}, test_vehicle());
  EXPECT_FALSE(verification.valid);
  EXPECT_EQ(verification.reason, "invalid_input");
}

TEST(CorridorVerificationTest, rejects_invalid_verification_options) {
  const Corridor corridor = make_corridor(covering_polygons());
  const std::vector<common::PathSegment> seed = {straight_seed(0.0, 10.0, 5.0, 0.0)};
  CorridorVerifyOptions options;
  options.sample_step_m = 0.0;
  const CorridorVerification verification =
      verify_corridor(corridor, seed, {}, test_vehicle(), options);
  EXPECT_FALSE(verification.valid);
  EXPECT_EQ(verification.reason, "invalid_options");
}

}  // namespace
}  // namespace planner::corridor
