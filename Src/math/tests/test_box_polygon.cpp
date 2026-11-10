#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "planner/math/angle.hpp"
#include "planner/math/box2d.hpp"
#include "planner/math/polygon2d.hpp"

namespace {
using namespace planner;

void expect_near(double a, double b, double tolerance = 1e-12) {
  EXPECT_TRUE(std::isfinite(a));
  EXPECT_TRUE(std::isfinite(b));
  EXPECT_LE(std::abs(a - b), tolerance);
}

std::vector<math::Point2d> corner_vector(const math::Box2d& box) {
  const std::array<math::Point2d, 4> corners = box.corners();
  return std::vector<math::Point2d>(corners.begin(), corners.end());
}

TEST(MathBox2d, CornersAndAabbFollowFootprintOrder) {
  const math::Box2d box{{1.0, 2.0}, 4.0, 2.0, 0.0};
  const std::array<math::Point2d, 4> corners = box.corners();
  EXPECT_EQ(corners[0], (math::Point2d{-1.0, 1.0}));
  EXPECT_EQ(corners[1], (math::Point2d{3.0, 1.0}));
  EXPECT_EQ(corners[2], (math::Point2d{3.0, 3.0}));
  EXPECT_EQ(corners[3], (math::Point2d{-1.0, 3.0}));
  const math::Aabb2d aabb = box.aabb();
  EXPECT_DOUBLE_EQ(aabb.min_x, -1.0);
  EXPECT_DOUBLE_EQ(aabb.min_y, 1.0);
  EXPECT_DOUBLE_EQ(aabb.max_x, 3.0);
  EXPECT_DOUBLE_EQ(aabb.max_y, 3.0);

  const math::Box2d rotated{{0.0, 0.0}, 2.0, 2.0, math::kPi * 0.5};
  const std::array<math::Point2d, 4> rc = rotated.corners();
  expect_near(rc[0].x_m, 1.0);
  expect_near(rc[0].y_m, -1.0);
  expect_near(rc[1].x_m, 1.0);
  expect_near(rc[1].y_m, 1.0);
  expect_near(rc[2].x_m, -1.0);
  expect_near(rc[2].y_m, 1.0);
  expect_near(rc[3].x_m, -1.0);
  expect_near(rc[3].y_m, -1.0);
}

TEST(MathBox2d, ContainsTouchAndTolerance) {
  const math::Box2d box{{0.0, 0.0}, 2.0, 2.0, 0.0};
  EXPECT_TRUE(box.contains({0.0, 0.0}));
  EXPECT_TRUE(box.contains({1.0, 1.0}));
  EXPECT_TRUE(box.contains({1.0, 0.0}));
  EXPECT_FALSE(box.contains({1.001, 0.0}));
  EXPECT_TRUE(box.contains({1.001, 0.0}, 0.01));
  EXPECT_FALSE(box.contains({1.001, 1.001}));

  const math::Box2d rotated{{0.0, 0.0}, 2.0, 1.0, math::kPi * 0.25};
  EXPECT_TRUE(rotated.contains({0.0, 0.0}));
  EXPECT_TRUE(rotated.contains(rotated.corners()[1]));
  EXPECT_FALSE(rotated.contains({1.5, 0.0}));
}

TEST(MathBox2d, IntersectsAabbTouchOverlapAndSeparation) {
  const math::Box2d box{{0.0, 0.0}, 2.0, 2.0, 0.0};
  EXPECT_TRUE(box.intersects({0.5, -0.5, 1.5, 0.5}));
  EXPECT_TRUE(box.intersects({1.0, -1.0, 2.0, 1.0})) << "edge touch";
  EXPECT_TRUE(box.intersects({1.0, 1.0, 2.0, 2.0})) << "corner touch";
  EXPECT_FALSE(box.intersects({1.0001, -1.0, 2.0, 1.0}));
  EXPECT_TRUE(box.intersects({1.0001, -1.0, 2.0, 1.0}, 0.001)) << "tolerance closes the gap";

  const math::Box2d rotated{{0.0, 0.0}, 2.0, 2.0, math::kPi * 0.25};
  EXPECT_TRUE(rotated.intersects({-0.1, -0.1, 0.1, 0.1}));
  EXPECT_FALSE(rotated.intersects({3.0, 3.0, 4.0, 4.0}));
}

TEST(MathBox2d, IntersectsBoxTouchOverlapAndSeparation) {
  const math::Box2d a{{0.0, 0.0}, 2.0, 2.0, 0.0};
  const math::Box2d touching{{2.0, 0.0}, 2.0, 2.0, 0.0};
  const math::Box2d separated{{2.001, 0.0}, 2.0, 2.0, 0.0};
  const math::Box2d overlapping{{1.0, 0.0}, 2.0, 2.0, 0.3};
  EXPECT_TRUE(a.intersects(touching));
  EXPECT_FALSE(a.intersects(separated));
  EXPECT_TRUE(a.intersects(separated, 0.01));
  EXPECT_TRUE(a.intersects(overlapping));
  EXPECT_DOUBLE_EQ(a.distance_sq_to({3.0, 0.0}), 4.0);
  EXPECT_DOUBLE_EQ(a.distance_sq_to({1.0, 1.0}), 0.0);
}

TEST(MathBox2d, RandomDifferentialAgainstPolygonFunctions) {
  std::mt19937 engine(20261111);
  std::uniform_real_distribution<double> center(-6.0, 6.0);
  std::uniform_real_distribution<double> size(0.05, 3.0);
  std::uniform_real_distribution<double> angle(-math::kPi, math::kPi);
  for (int i = 0; i < 400; ++i) {
    SCOPED_TRACE(i);
    const math::Box2d a{
        {center(engine), center(engine)}, size(engine), size(engine), angle(engine)};
    const math::Box2d b{
        {center(engine), center(engine)}, size(engine), size(engine), angle(engine)};
    const double x = center(engine);
    const double y = center(engine);
    const math::Aabb2d box{x, y, x + size(engine), y + size(engine)};
    const std::vector<math::Point2d> va = corner_vector(a);
    const std::vector<math::Point2d> vb = corner_vector(b);
    for (double tol : {0.0, 1e-12, 1e-4, 0.05}) {
      SCOPED_TRACE(tol);
      EXPECT_EQ(a.intersects(b, tol), math::convex_polygons_intersect(va, vb, tol));
      EXPECT_EQ(a.intersects(box, tol), math::convex_polygon_intersects_aabb(va, box, tol));
    }
    const math::Point2d p{x, y};
    EXPECT_EQ(a.distance_sq_to(p), math::point_polygon_distance_sq(p, va));
  }
}

TEST(MathPolygon2d, FactoryValidatesNormalizesAndCachesAabb) {
  const std::vector<math::Point2d> clockwise{{0.0, 0.0}, {0.0, 2.0}, {2.0, 2.0}, {2.0, 0.0}};
  std::string reason;
  const auto polygon = math::Polygon2d::convex(clockwise, &reason);
  ASSERT_TRUE(polygon.has_value()) << reason;
  EXPECT_TRUE(reason.empty());
  ASSERT_EQ(polygon->size(), 4u);
  EXPECT_FALSE(polygon->empty());
  EXPECT_TRUE(math::is_counter_clockwise(polygon->vertices()));
  EXPECT_EQ(polygon->data(), polygon->vertices().data());
  EXPECT_TRUE(polygon->valid());
  EXPECT_DOUBLE_EQ(polygon->aabb().min_x, 0.0);
  EXPECT_DOUBLE_EQ(polygon->aabb().min_y, 0.0);
  EXPECT_DOUBLE_EQ(polygon->aabb().max_x, 2.0);
  EXPECT_DOUBLE_EQ(polygon->aabb().max_y, 2.0);
  EXPECT_EQ(std::accumulate(polygon->vertices().begin(), polygon->vertices().end(), math::Point2d{},
                            [](const math::Point2d& acc, const math::Point2d& p) {
                              return math::Point2d{acc.x_m + p.x_m, acc.y_m + p.y_m};
                            }),
            (math::Point2d{4.0, 4.0}));
}

TEST(MathPolygon2d, FactoryRejectsInvalidInputs) {
  std::string reason;
  EXPECT_FALSE(math::Polygon2d::convex({{0.0, 0.0}, {1.0, 0.0}, {2.0, 0.0}}, &reason).has_value());
  EXPECT_FALSE(reason.empty());

  reason.clear();
  EXPECT_FALSE(
      math::Polygon2d::convex({{0.0, 0.0}, {2.0, 0.0}, {1.0, 0.2}, {2.0, 2.0}, {0.0, 2.0}}, &reason)
          .has_value());
  EXPECT_FALSE(reason.empty());

  reason.clear();
  EXPECT_FALSE(
      math::Polygon2d::convex(
          {{0.0, 0.0}, {std::numeric_limits<double>::infinity(), 0.0}, {1.0, 1.0}}, &reason)
          .has_value());
  EXPECT_FALSE(reason.empty());

  const math::Polygon2d empty;
  EXPECT_TRUE(empty.empty());
  EXPECT_EQ(empty.size(), 0u);
  EXPECT_FALSE(empty.valid());
}

TEST(MathPolygon2d, RawOverloadsMatchVectorWrappersAndHandleClockwise) {
  const std::vector<math::Point2d> ccw{{0.0, 0.0}, {2.0, 0.0}, {2.0, 1.0}, {1.0, 2.0}, {0.0, 1.0}};
  std::vector<math::Point2d> cw = ccw;
  std::reverse(cw.begin(), cw.end());
  const std::vector<math::Point2d> other{{3.0, 0.5}, {4.0, 0.5}, {4.0, 1.5}, {3.0, 1.5}};
  const math::Aabb2d box{2.5, 0.0, 3.5, 1.0};
  const math::Point2d p{3.0, 2.0};

  EXPECT_EQ(math::convex_polygons_intersect(cw.data(), cw.size(), other.data(), other.size(), 0.0),
            math::convex_polygons_intersect(cw, other, 0.0));
  EXPECT_EQ(math::convex_polygon_intersects_aabb(cw.data(), cw.size(), box, 0.0),
            math::convex_polygon_intersects_aabb(cw, box, 0.0));
  EXPECT_DOUBLE_EQ(math::point_polygon_distance(p, cw.data(), cw.size()),
                   math::point_polygon_distance(p, ccw));
  EXPECT_DOUBLE_EQ(math::point_polygon_distance_sq(p, cw.data(), cw.size()),
                   math::point_polygon_distance_sq(p, ccw));
  EXPECT_DOUBLE_EQ(math::convex_polygon_distance(cw.data(), cw.size(), other.data(), other.size()),
                   math::convex_polygon_distance(ccw, other));
  EXPECT_DOUBLE_EQ(
      math::convex_polygon_distance_sq(cw.data(), cw.size(), other.data(), other.size()),
      math::convex_polygon_distance_sq(ccw, other));
  EXPECT_DOUBLE_EQ(math::polygon_aabb_distance(cw.data(), cw.size(), box),
                   math::polygon_aabb_distance(ccw, box));
  EXPECT_DOUBLE_EQ(math::polygon_aabb_distance_sq(cw.data(), cw.size(), box),
                   math::polygon_aabb_distance_sq(ccw, box));
}

TEST(MathPolygon2d, RawOverloadsHandleEmptyAndDegenerateInputs) {
  const std::vector<math::Point2d> empty;
  const std::vector<math::Point2d> point{{1.0, 1.0}};
  const std::vector<math::Point2d> far_point{{2.0, 3.0}};
  const std::vector<math::Point2d> horizontal{{0.0, 2.0}, {2.0, 2.0}};
  const std::vector<math::Point2d> vertical{{1.0, 1.5}, {1.0, 2.5}};
  const math::Aabb2d box{0.0, 0.0, 1.0, 1.0};

  EXPECT_FALSE(
      math::convex_polygons_intersect(empty.data(), empty.size(), point.data(), point.size(), 0.0));
  EXPECT_EQ(
      math::convex_polygons_intersect(empty.data(), empty.size(), point.data(), point.size(), 0.0),
      math::convex_polygons_intersect(empty, point, 0.0));
  EXPECT_TRUE(
      math::convex_polygons_intersect(point.data(), point.size(), point.data(), point.size(), 0.0))
      << "touching degenerate points count as intersection";
  EXPECT_TRUE(math::convex_polygons_intersect(horizontal.data(), horizontal.size(), vertical.data(),
                                              vertical.size(), 0.0));
  EXPECT_FALSE(math::convex_polygons_intersect(horizontal.data(), horizontal.size(),
                                               far_point.data(), far_point.size(), 0.0));

  EXPECT_TRUE(std::isinf(
      math::convex_polygon_distance_sq(empty.data(), empty.size(), point.data(), point.size())));
  EXPECT_DOUBLE_EQ(
      math::convex_polygon_distance_sq(point.data(), point.size(), point.data(), point.size()),
      0.0);
  EXPECT_DOUBLE_EQ(math::convex_polygon_distance_sq(point.data(), point.size(), far_point.data(),
                                                    far_point.size()),
                   5.0);
  EXPECT_DOUBLE_EQ(math::convex_polygon_distance_sq(horizontal.data(), horizontal.size(),
                                                    far_point.data(), far_point.size()),
                   1.0);

  EXPECT_TRUE(std::isinf(math::polygon_aabb_distance_sq(empty.data(), empty.size(), box)));
  EXPECT_DOUBLE_EQ(math::polygon_aabb_distance_sq(point.data(), point.size(), box), 0.0);
  EXPECT_DOUBLE_EQ(math::polygon_aabb_distance_sq(horizontal.data(), horizontal.size(), box), 1.0);
}

TEST(MathPolygon2d, RandomDifferentialAgainstVectorWrappers) {
  std::mt19937 engine(77);
  std::uniform_real_distribution<double> coordinate(-4.0, 4.0);
  std::uniform_real_distribution<double> angle(-math::kPi, math::kPi);
  for (int i = 0; i < 300; ++i) {
    SCOPED_TRACE(i);
    const math::Box2d a{{coordinate(engine), coordinate(engine)},
                        std::abs(coordinate(engine)) + 0.1,
                        std::abs(coordinate(engine)) + 0.1,
                        angle(engine)};
    const math::Box2d b{{coordinate(engine), coordinate(engine)},
                        std::abs(coordinate(engine)) + 0.1,
                        std::abs(coordinate(engine)) + 0.1,
                        angle(engine)};
    const std::vector<math::Point2d> va = corner_vector(a);
    const std::vector<math::Point2d> vb = corner_vector(b);
    const math::Aabb2d box{coordinate(engine), coordinate(engine), coordinate(engine),
                           coordinate(engine)};
    const math::Aabb2d ordered{std::min(box.min_x, box.max_x), std::min(box.min_y, box.max_y),
                               std::max(box.min_x, box.max_x), std::max(box.min_y, box.max_y)};
    EXPECT_EQ(math::convex_polygons_intersect(va.data(), va.size(), vb.data(), vb.size(), 0.0),
              math::convex_polygons_intersect(va, vb, 0.0));
    EXPECT_EQ(math::convex_polygon_intersects_aabb(va.data(), va.size(), ordered, 0.0),
              math::convex_polygon_intersects_aabb(va, ordered, 0.0));
    EXPECT_DOUBLE_EQ(math::convex_polygon_distance_sq(va.data(), va.size(), vb.data(), vb.size()),
                     math::convex_polygon_distance_sq(va, vb));
    EXPECT_DOUBLE_EQ(math::polygon_aabb_distance_sq(va.data(), va.size(), ordered),
                     math::polygon_aabb_distance_sq(va, ordered));
  }
}

}  // namespace
