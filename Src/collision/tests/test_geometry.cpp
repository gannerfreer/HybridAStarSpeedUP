// 几何图元测试：分离轴、接触处理、距离。

#include <gtest/gtest.h>

#include "planner/collision/geometry.hpp"
#include "test_helpers.hpp"

namespace planner::collision {
namespace {

TEST(Geometry, RectangleIsCounterClockwise) {
  const std::vector<Point2d> rectangle =
      rectangle_polygon(0.0, 0.0, 2.0, 1.0);
  ASSERT_EQ(rectangle.size(), 4U);
  EXPECT_GT(signed_area_twice(rectangle), 0.0);
  EXPECT_TRUE(is_counter_clockwise(rectangle));
}

TEST(Geometry, ClockwisePolygonIsNormalized) {
  std::vector<Point2d> clockwise{Point2d{0.0, 0.0}, Point2d{0.0, 1.0},
                                 Point2d{1.0, 1.0}, Point2d{1.0, 0.0}};
  EXPECT_FALSE(is_counter_clockwise(clockwise));
  normalize_counter_clockwise(clockwise);
  EXPECT_TRUE(is_counter_clockwise(clockwise));
}

TEST(Geometry, RejectsDegenerateAndNonConvexPolygons) {
  std::string reason;
  EXPECT_FALSE(convex_polygon_valid({Point2d{0.0, 0.0}, Point2d{1.0, 0.0}},
                                    &reason));
  EXPECT_FALSE(convex_polygon_valid({Point2d{0.0, 0.0}, Point2d{1.0, 1.0},
                                     Point2d{2.0, 2.0}},
                                    &reason));
  // 飞镖形：非凸四边形。
  EXPECT_FALSE(convex_polygon_valid({Point2d{0.0, 0.0}, Point2d{2.0, 0.0},
                                     Point2d{1.0, 1.0}, Point2d{2.0, 2.0}},
                                    &reason));
}

TEST(Geometry, TangentPolygonsCountAsIntersecting) {
  const std::vector<Point2d> a = rectangle_polygon(0.0, 0.0, 1.0, 1.0);
  const std::vector<Point2d> b = rectangle_polygon(1.0, 0.0, 2.0, 1.0);
  EXPECT_TRUE(convex_polygons_intersect(a, b, 0.0));
  const std::vector<Point2d> c = rectangle_polygon(1.0 + 1e-9, 0.0, 2.0, 1.0);
  EXPECT_FALSE(convex_polygons_intersect(a, c, 0.0));
}

TEST(Geometry, CornerContactCountsAsIntersecting) {
  const std::vector<Point2d> a = rectangle_polygon(0.0, 0.0, 1.0, 1.0);
  const std::vector<Point2d> b = rectangle_polygon(1.0, 1.0, 2.0, 2.0);
  EXPECT_TRUE(convex_polygons_intersect(a, b, 0.0));
  EXPECT_DOUBLE_EQ(convex_polygon_distance(a, b), 0.0);
}

TEST(Geometry, RotatedPolygonsIntersect) {
  const std::vector<Point2d> a = rectangle_polygon(-1.0, -0.5, 1.0, 0.5);
  std::vector<Point2d> diamond{Point2d{0.0, -1.2}, Point2d{1.0, 0.0},
                               Point2d{0.0, 1.2}, Point2d{-1.0, 0.0}};
  EXPECT_TRUE(convex_polygons_intersect(a, diamond, 0.0));
  std::vector<Point2d> far_diamond{Point2d{3.0, -1.2}, Point2d{4.0, 0.0},
                                   Point2d{3.0, 1.2}, Point2d{2.0, 0.0}};
  EXPECT_FALSE(convex_polygons_intersect(a, far_diamond, 0.0));
}

TEST(Geometry, ConvexPolygonDistanceMatchesHandValues) {
  const std::vector<Point2d> a = rectangle_polygon(0.0, 0.0, 1.0, 1.0);
  const std::vector<Point2d> b = rectangle_polygon(3.0, 0.0, 4.0, 1.0);
  EXPECT_DOUBLE_EQ(convex_polygon_distance(a, b), 2.0);
  const std::vector<Point2d> corner =
      rectangle_polygon(4.0, 4.0, 5.0, 5.0);
  EXPECT_NEAR(convex_polygon_distance(a, corner), std::hypot(3.0, 3.0), 1e-12);
}

TEST(Geometry, PointPolygonDistance) {
  const std::vector<Point2d> a = rectangle_polygon(0.0, 0.0, 1.0, 1.0);
  EXPECT_DOUBLE_EQ(point_polygon_distance(Point2d{0.5, 0.5}, a), 0.0);
  EXPECT_DOUBLE_EQ(point_polygon_distance(Point2d{2.0, 0.5}, a), 1.0);
  EXPECT_NEAR(point_polygon_distance(Point2d{2.0, 2.0}, a), std::sqrt(2.0),
              1e-12);
}

TEST(Geometry, ConvexHullIsCounterClockwise) {
  const std::vector<Point2d> hull =
      convex_hull({Point2d{0.0, 0.0}, Point2d{2.0, 0.0}, Point2d{2.0, 2.0},
                   Point2d{0.0, 2.0}, Point2d{1.0, 1.0}});
  ASSERT_EQ(hull.size(), 4U);
  EXPECT_GT(signed_area_twice(hull), 0.0);
}

TEST(Geometry, PolygonIntersectsAabb) {
  const std::vector<Point2d> polygon =
      rectangle_polygon(1.0, 1.0, 2.0, 2.0);
  EXPECT_TRUE(convex_polygon_intersects_aabb(polygon, Aabb2d{1.5, 1.5, 3.0, 3.0},
                                             0.0));
  EXPECT_TRUE(convex_polygon_intersects_aabb(polygon, Aabb2d{2.0, 1.0, 3.0, 2.0},
                                             0.0));
  EXPECT_FALSE(
      convex_polygon_intersects_aabb(polygon, Aabb2d{2.5, 2.5, 3.0, 3.0}, 0.0));
}

}  // namespace
}  // namespace planner::collision
