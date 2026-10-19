// 走廊生成器与验证器所用凸几何辅助函数的单元测试。所有期望值都是
// 轴对齐或直角三角形输入的解析值。

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "planner/common/types.hpp"
#include "planner/corridor/geometry.hpp"

namespace planner::corridor {
namespace {

using common::Point2d;

std::vector<Point2d> square(double x_min, double y_min, double x_max, double y_max) {
  return {Point2d{x_min, y_min}, Point2d{x_max, y_min}, Point2d{x_max, y_max},
          Point2d{x_min, y_max}};
}

TEST(CorridorGeometryTest, convex_hull_returns_counter_clockwise_quadrilateral) {
  const std::vector<Point2d> points = {Point2d{0.0, 0.0}, Point2d{2.0, 0.0},
                                       Point2d{2.0, 2.0}, Point2d{0.0, 2.0},
                                       Point2d{1.0, 1.0}};
  const std::vector<Point2d> hull = convex_hull(points);
  ASSERT_EQ(hull.size(), 4u);
  EXPECT_NEAR(polygon_signed_area(hull), 4.0, 1e-12);
  EXPECT_TRUE(is_convex_ccw_polygon(hull));
  for (const Point2d& point : points) {
    EXPECT_TRUE(point_in_convex_polygon_ccw(point, hull)) << point.x_m << ", " << point.y_m;
  }
}

TEST(CorridorGeometryTest, convex_hull_drops_interior_and_collinear_points) {
  const std::vector<Point2d> points = {
      Point2d{0.0, 0.0}, Point2d{1.0, 0.0}, Point2d{2.0, 0.0}, Point2d{2.0, 1.0},
      Point2d{2.0, 2.0}, Point2d{1.0, 2.0}, Point2d{0.0, 2.0}, Point2d{0.0, 1.0},
      Point2d{1.0, 1.0}};
  const std::vector<Point2d> hull = convex_hull(points);
  EXPECT_EQ(hull.size(), 4u);
  EXPECT_NEAR(polygon_signed_area(hull), 4.0, 1e-12);
}

TEST(CorridorGeometryTest, convex_hull_of_collinear_points_is_degenerate) {
  const std::vector<Point2d> hull =
      convex_hull({Point2d{0.0, 0.0}, Point2d{1.0, 0.0}, Point2d{2.0, 0.0}});
  EXPECT_LT(hull.size(), 3u);
}

TEST(CorridorGeometryTest, is_convex_ccw_rejects_clockwise_and_concave_polygons) {
  const std::vector<Point2d> ccw_square = square(0.0, 0.0, 1.0, 1.0);
  EXPECT_TRUE(is_convex_ccw_polygon(ccw_square));

  std::vector<Point2d> clockwise = ccw_square;
  std::reverse(clockwise.begin(), clockwise.end());
  EXPECT_FALSE(is_convex_ccw_polygon(clockwise));

  const std::vector<Point2d> concave = {Point2d{0.0, 0.0}, Point2d{4.0, 0.0},
                                        Point2d{4.0, 1.0}, Point2d{1.0, 1.0},
                                        Point2d{1.0, 4.0}, Point2d{0.0, 4.0}};
  EXPECT_FALSE(is_convex_ccw_polygon(concave));
  EXPECT_FALSE(is_convex_ccw_polygon({Point2d{0.0, 0.0}, Point2d{1.0, 1.0}}));
}

TEST(CorridorGeometryTest, point_in_convex_polygon_handles_inside_boundary_and_outside) {
  const std::vector<Point2d> unit_square = square(0.0, 0.0, 1.0, 1.0);
  EXPECT_TRUE(point_in_convex_polygon_ccw(Point2d{0.5, 0.5}, unit_square));
  EXPECT_TRUE(point_in_convex_polygon_ccw(Point2d{0.0, 0.5}, unit_square));
  EXPECT_TRUE(point_in_convex_polygon_ccw(Point2d{1.0, 1.0}, unit_square));
  EXPECT_FALSE(point_in_convex_polygon_ccw(Point2d{1.5, 0.5}, unit_square));
  EXPECT_FALSE(point_in_convex_polygon_ccw(Point2d{-0.1, 0.5}, unit_square));
}

TEST(CorridorGeometryTest, convex_polygons_gap_is_positive_for_separated_squares) {
  const std::vector<Point2d> a = square(0.0, 0.0, 1.0, 1.0);
  const std::vector<Point2d> b = square(2.0, 0.0, 3.0, 1.0);
  EXPECT_NEAR(convex_polygons_gap(a, b), 1.0, 1e-9);
  EXPECT_NEAR(convex_polygons_gap(b, a), 1.0, 1e-9);
}

TEST(CorridorGeometryTest, convex_polygons_gap_is_non_positive_for_overlapping_squares) {
  const std::vector<Point2d> a = square(0.0, 0.0, 2.0, 2.0);
  const std::vector<Point2d> b = square(1.0, 1.0, 3.0, 3.0);
  EXPECT_LE(convex_polygons_gap(a, b), 0.0);

  const std::vector<Point2d> touching = square(2.0, 0.0, 3.0, 2.0);
  EXPECT_NEAR(convex_polygons_gap(a, touching), 0.0, 1e-12);
}

TEST(CorridorGeometryTest, convex_polygon_contains_detects_subset_and_escape) {
  const std::vector<Point2d> outer = square(-1.0, -1.0, 3.0, 3.0);
  const std::vector<Point2d> inner = square(0.0, 0.0, 1.0, 1.0);
  EXPECT_TRUE(convex_polygon_contains(outer, inner));
  EXPECT_FALSE(convex_polygon_contains(inner, outer));

  const std::vector<Point2d> escaping = {Point2d{0.0, 0.0}, Point2d{4.0, 0.0},
                                         Point2d{4.0, 0.5}, Point2d{0.0, 0.5}};
  EXPECT_FALSE(convex_polygon_contains(outer, escaping));
}

TEST(CorridorGeometryTest, offset_convex_polygon_grows_and_contains_original) {
  const std::vector<Point2d> original = square(0.0, 0.0, 2.0, 2.0);
  std::vector<Point2d> offset;
  ASSERT_TRUE(offset_convex_polygon(original, 0.5, &offset));
  EXPECT_TRUE(is_convex_ccw_polygon(offset));
  EXPECT_TRUE(convex_polygon_contains(offset, original));
  EXPECT_NEAR(polygon_signed_area(offset), 9.0, 1e-9);
  // 半平面偏移包含圆盘偏移：距边 0.4 m 的点在内部，距边 0.6 m 的点不在。
  EXPECT_TRUE(point_in_convex_polygon_ccw(Point2d{1.0, -0.4}, offset));
  EXPECT_FALSE(point_in_convex_polygon_ccw(Point2d{1.0, -0.6}, offset));
}

TEST(CorridorGeometryTest, intersect_half_planes_builds_unit_square) {
  const std::vector<HalfPlane> planes = {
      HalfPlane{Point2d{-1.0, 0.0}, 0.0}, HalfPlane{Point2d{1.0, 0.0}, 1.0},
      HalfPlane{Point2d{0.0, -1.0}, 0.0}, HalfPlane{Point2d{0.0, 1.0}, 1.0}};
  const std::vector<Point2d> bounds = square(-5.0, -5.0, 5.0, 5.0);
  std::vector<Point2d> result;
  ASSERT_TRUE(intersect_half_planes(planes, bounds, &result));
  EXPECT_NEAR(polygon_signed_area(result), 1.0, 1e-9);
  EXPECT_TRUE(point_in_convex_polygon_ccw(Point2d{0.5, 0.5}, result));
  EXPECT_FALSE(point_in_convex_polygon_ccw(Point2d{1.5, 0.5}, result));
}

TEST(CorridorGeometryTest, intersect_half_planes_detects_empty_intersection) {
  const std::vector<HalfPlane> planes = {HalfPlane{Point2d{1.0, 0.0}, 1.0},
                                         HalfPlane{Point2d{-1.0, 0.0}, -2.0}};
  const std::vector<Point2d> bounds = square(-5.0, -5.0, 5.0, 5.0);
  std::vector<Point2d> result;
  EXPECT_FALSE(intersect_half_planes(planes, bounds, &result));
}

TEST(CorridorGeometryTest, intersection_area_of_overlapping_squares) {
  const std::vector<Point2d> a = square(0.0, 0.0, 2.0, 2.0);
  const std::vector<Point2d> b = square(1.0, 1.0, 3.0, 3.0);
  EXPECT_NEAR(convex_polygon_intersection_area(a, b), 1.0, 1e-9);
  const std::vector<Point2d> disjoint = square(4.0, 4.0, 5.0, 5.0);
  EXPECT_DOUBLE_EQ(convex_polygon_intersection_area(a, disjoint), 0.0);
}

TEST(CorridorGeometryTest, min_width_of_axis_aligned_rectangle) {
  const std::vector<Point2d> rectangle = square(0.0, 0.0, 4.0, 2.0);
  EXPECT_NEAR(convex_polygon_min_width(rectangle), 2.0, 1e-9);
}

TEST(CorridorGeometryTest, min_width_of_right_triangle_matches_smallest_altitude) {
  const std::vector<Point2d> triangle = {Point2d{0.0, 0.0}, Point2d{4.0, 0.0},
                                         Point2d{0.0, 1.0}};
  // 面积 2；最小高为 2 * 面积 / |斜边| = 4 / sqrt(17)。
  EXPECT_NEAR(convex_polygon_min_width(triangle), 4.0 / std::sqrt(17.0), 1e-9);
}

TEST(CorridorGeometryTest, matrix_helpers_behave_as_expected) {
  const Point2d a{3.0, 4.0};
  const Point2d b{1.0, -2.0};
  EXPECT_DOUBLE_EQ(dot(a, b), -5.0);
  EXPECT_DOUBLE_EQ(cross(a, b), -10.0);
  EXPECT_DOUBLE_EQ(norm(a), 5.0);
  EXPECT_DOUBLE_EQ(distance(a, b), std::sqrt(40.0));
  EXPECT_TRUE(is_finite(a));
  EXPECT_FALSE(is_finite(Point2d{std::nan(""), 0.0}));
}

}  // namespace
}  // namespace planner::corridor
