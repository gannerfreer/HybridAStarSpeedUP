#include "planner/math/aabb2d.hpp"
#include "planner/math/angle.hpp"
#include "planner/math/detail/sat_cache.hpp"
#include "planner/math/kinematics/arc.hpp"
#include "planner/math/kinematics/bicycle_model.hpp"
#include "planner/math/pose2d.hpp"
#include "planner/math/polygon2d.hpp"
#include "planner/math/segment2d.hpp"
#include "planner/math/vec2d.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace {
using namespace planner;

void expect_near(double a, double b, double tolerance = 1e-11) {
  EXPECT_TRUE(std::isfinite(a));
  EXPECT_TRUE(std::isfinite(b));
  EXPECT_LE(std::abs(a - b), tolerance);
}

TEST(MathAngle, NormalizesIntoHalfOpenRange) {
  EXPECT_EQ(math::normalize_angle_rad(math::kPi), -math::kPi);
  EXPECT_EQ(math::normalize_angle_rad(-math::kPi), -math::kPi);
  for (double a : {0., -0., 1., -1., 100., -100., 1e12, -1e12}) {
    SCOPED_TRACE(a);
    const double out = math::normalize_angle_rad(a);
    EXPECT_GE(out, -math::kPi);
    EXPECT_LT(out, math::kPi);
    EXPECT_EQ(math::normalize_angle_rad(out), out);
  }
  EXPECT_TRUE(std::isnan(
      math::normalize_angle_rad(std::numeric_limits<double>::infinity())));
}

TEST(MathArc, MatchesClosedFormAndRoundTrips) {
  std::mt19937 engine(42);
  std::uniform_real_distribution<double> distribution(-3., 3.);
  for (int i = 0; i < 600; ++i) {
    SCOPED_TRACE(i);
    math::Pose2d start{distribution(engine), distribution(engine),
                       distribution(engine)};
    const double s = distribution(engine);
    const double k = i % 3 == 0 ? 1e-12
                                : (i % 3 == 1 ? 0. : distribution(engine));
    const auto out = math::integrate_arc(start, s, k);
    const auto cached = math::integrate_arc_trig(
        start, s, k, std::cos(start.yaw_rad), std::sin(start.yaw_rad));
    expect_near(out.x_m, cached.x_m, 0);
    expect_near(out.y_m, cached.y_m, 0);
    expect_near(out.yaw_rad, cached.yaw_rad, 0);
    // 独立的长双精度中点公式，避免差分正余弦在近直线时相消。
    const long double h = static_cast<long double>(k) * s / 2;
    const long double scale = h == 0 ? 1 : std::sin(h) / h;
    expect_near(out.x_m,
                static_cast<double>(start.x_m +
                                    s * scale * std::cos(start.yaw_rad + h)));
    expect_near(out.y_m,
                static_cast<double>(start.y_m +
                                    s * scale * std::sin(start.yaw_rad + h)));
    const auto back = math::integrate_arc(out, -s, k);
    expect_near(back.x_m, start.x_m);
    expect_near(back.y_m, start.y_m);
    expect_near(math::normalize_angle_rad(back.yaw_rad - start.yaw_rad), 0.);
  }
  const auto straight = math::integrate_arc({0, 0, 0}, -5, 0);
  expect_near(straight.x_m, -5);
  expect_near(straight.y_m, 0);
}

TEST(MathGeometry, PrimitivesAndSatFallback) {
  const auto polygon = math::rectangle_polygon(0, 0, 2, 2);
  EXPECT_TRUE(math::convex_polygon_valid(polygon)) << "valid rectangle";
  EXPECT_FALSE(math::convex_polygon_valid({{0, 0}, {1, 0}, {2, 0}}))
      << "degenerate polygon";
  EXPECT_TRUE(math::convex_polygon_intersects_aabb(polygon, {2, 0, 3, 1}))
      << "touch counts as intersection";
  expect_near(math::point_polygon_distance_sq({3, 3}, polygon), 2.);
  expect_near(
      math::Segment2d{{0, 0}, {0, 0}}.distance_sq_to({2, 2}), 8.);
  expect_near(math::aabb_distance({0, 0, 1, 1}, {2, 2, 3, 3}),
              std::sqrt(2.));
  const math::Pose2d pose{3, -2, 0.7};
  const math::Point2d p{1, 4};
  const auto restored =
      math::to_body_frame(pose, math::transform_point(pose, p));
  expect_near(restored.x_m, p.x_m);
  expect_near(restored.y_m, p.y_m);

  std::mt19937 engine(17);
  std::uniform_real_distribution<double> distribution(-5, 5);
  for (int i = 0; i < 500; ++i) {
    SCOPED_TRACE(i);
    auto rotated = polygon;
    const math::Pose2d transform{distribution(engine), distribution(engine),
                                 distribution(engine)};
    for (auto& point : rotated) point = math::transform_point(transform, point);
    const double x = distribution(engine);
    const double y = distribution(engine);
    const math::Aabb2d box{x, y, x + 1, y + 1};
    math::internal::PolygonSatCache8 cache;
    cache.build(rotated.data(), rotated.size());
    const bool reference = math::internal::convex_polygon_intersects_aabb_normalized(
        rotated.data(), rotated.size(), box, 1e-10);
    EXPECT_EQ(math::convex_polygon_intersects_aabb(rotated, box, 1e-10),
              reference)
        << "fast/reference SAT mismatch";
    EXPECT_EQ(cache.maybe_intersects(box, 1e-10), reference)
        << "cached/reference SAT mismatch";
    const double distance = math::point_polygon_distance({x, y}, rotated);
    expect_near(distance * distance,
                math::point_polygon_distance_sq({x, y}, rotated), 1e-10);
  }

  // 超出平方域快速路径的坐标仍需安全回退，保持相交/分离判定。
  const auto huge = math::rectangle_polygon(0, 0, 1e80, 1e80);
  math::internal::PolygonSatCache8 cache;
  cache.build(huge.data(), huge.size());
  EXPECT_FALSE(cache.fast_domain) << "must select numerical fallback";
  EXPECT_TRUE(cache.maybe_intersects({5e79, 5e79, 2e80, 2e80}, 0))
      << "fallback intersection";
  EXPECT_FALSE(cache.maybe_intersects({2e80, 2e80, 3e80, 3e80}, 0))
      << "fallback separation";

  std::array<math::Point2d, 8> input{
      {{0, 0}, {1, 0}, {1, 1}, {0, 1}, {0, 0}, {0.5, 0.5}, {1, 0}, {0, 1}}},
      out;
  EXPECT_EQ(math::convex_hull_small(input, 8, out), 4)
      << "small hull duplicates";
  std::vector<math::Point2d> many(input.begin(), input.end());
  many.push_back({0.2, 0.2});
  many.push_back({0.7, 0.7});
  EXPECT_EQ(math::convex_hull(many).size(), 4) << "generic hull duplicates";
  EXPECT_TRUE(math::convex_hull({}).empty()) << "empty hull";
}

TEST(MathAngle, DifferenceAndUnitConversions) {
  expect_near(math::deg_to_rad(180.0), math::kPi);
  expect_near(math::rad_to_deg(math::kPi), 180.0);
  expect_near(math::deg_to_rad(90.0), 0.5 * math::kPi);
  expect_near(math::angle_diff_rad(math::kPi - 0.1, -math::kPi + 0.1), -0.2);
  expect_near(math::angle_diff_rad(-math::kPi + 0.1, math::kPi - 0.1), 0.2);
  expect_near(math::angle_diff_rad(0.5, 0.5), 0.0);
  EXPECT_TRUE(std::isnan(math::angle_diff_rad(
      std::numeric_limits<double>::infinity(), 0.0)));
}

TEST(MathVec2d, PrimitiveAlgebra) {
  const math::Point2d a{3.0, 4.0};
  expect_near(a.norm(), 5.0);
  expect_near(a.norm_sq(), 25.0);
  EXPECT_EQ(a.dot({1.0, 0.0}), 3.0);
  EXPECT_EQ(a.cross({1.0, 0.0}), -4.0);
  expect_near(math::dot(a, {1.0, 0.0}), a.dot({1.0, 0.0}));
  expect_near(math::cross(a, {1.0, 0.0}), a.cross({1.0, 0.0}));
  expect_near(math::norm(a), a.norm());
  expect_near(math::distance({1.0, 2.0}, {4.0, 6.0}), 5.0);

  const math::Point2d unit = a.normalized_or_zero();
  expect_near(unit.norm(), 1.0);
  EXPECT_EQ(math::Point2d{}.normalized_or_zero(), (math::Point2d{0.0, 0.0}));

  const math::Point2d rotated = math::Point2d{1.0, 0.0}.rotated(0.5 * math::kPi);
  expect_near(rotated.x_m, 0.0);
  expect_near(rotated.y_m, 1.0);

  const math::Point2d projected = a.projected_on({1.0, 0.0});
  expect_near(projected.x_m, 3.0);
  expect_near(projected.y_m, 0.0);
  EXPECT_EQ(a.projected_on({0.0, 0.0}), (math::Point2d{0.0, 0.0}));
}

TEST(MathSegment, LengthDirectionClosestPointAndDistances) {
  const math::Segment2d segment{{0.0, 0.0}, {3.0, 4.0}};
  EXPECT_DOUBLE_EQ(segment.length(), 5.0);
  EXPECT_DOUBLE_EQ(segment.length_sq(), 25.0);
  expect_near(segment.unit_direction().x_m, 0.6);
  expect_near(segment.unit_direction().y_m, 0.8);
  EXPECT_EQ((math::Segment2d{{1.0, 1.0}, {1.0, 1.0}}.unit_direction()),
            (math::Point2d{0.0, 0.0}));

  const math::Point2d p{0.0, 4.0};
  const math::Point2d closest = segment.closest_point(p);
  expect_near(closest.x_m, 1.92);
  expect_near(closest.y_m, 2.56);
  expect_near(segment.distance_to(p), math::distance(p, closest));
  expect_near(segment.distance_sq_to(p),
              math::distance(p, closest) * math::distance(p, closest));

  const math::Segment2d horizontal{{0.0, 0.0}, {2.0, 0.0}};
  expect_near(horizontal.distance_to({4.0, 3.0}), std::sqrt(13.0));
  expect_near(horizontal.distance_sq_to({4.0, 3.0}), 13.0);
  expect_near((math::Segment2d{{0.0, 0.0}, {0.0, 0.0}}.distance_sq_to(
                  {2.0, 2.0})),
              8.0);
}

TEST(MathSegment, IntersectionsAndSegmentDistances) {
  const math::Segment2d a{{0.0, 0.0}, {2.0, 0.0}};
  const math::Segment2d crossing{{1.0, -1.0}, {1.0, 1.0}};
  EXPECT_TRUE(a.intersects(crossing));
  EXPECT_DOUBLE_EQ(a.distance_to(crossing), 0.0);
  EXPECT_DOUBLE_EQ(a.distance_sq_to(crossing), 0.0);

  const math::Segment2d touching{{2.0, 0.0}, {3.0, 0.0}};
  EXPECT_TRUE(a.intersects(touching));
  EXPECT_DOUBLE_EQ(a.distance_to(touching), 0.0);

  const math::Segment2d collinear{{1.0, 0.0}, {3.0, 0.0}};
  EXPECT_TRUE(a.intersects(collinear));

  const math::Segment2d parallel{{0.0, 2.0}, {2.0, 2.0}};
  EXPECT_FALSE(a.intersects(parallel));
  EXPECT_DOUBLE_EQ(a.distance_to(parallel), 2.0);
  EXPECT_DOUBLE_EQ(a.distance_sq_to(parallel), 4.0);

  const math::Segment2d diagonal{{3.0, 0.0}, {3.0, 4.0}};
  EXPECT_DOUBLE_EQ(a.distance_to(diagonal), 1.0);
  EXPECT_DOUBLE_EQ(a.distance_sq_to(diagonal), 1.0);
}

TEST(MathAabb, MethodsAndDistanceQueries) {
  math::Aabb2d box{0.0, 0.0, 2.0, 2.0};
  EXPECT_TRUE(box.contains({0.0, 0.0}));
  EXPECT_TRUE(box.contains({2.0, 2.0}));
  EXPECT_FALSE(box.contains({2.1, 1.0}));
  EXPECT_TRUE(box.intersects({2.0, 2.0, 3.0, 3.0}));
  EXPECT_FALSE(box.intersects({2.1, 0.0, 3.0, 1.0}));

  math::Aabb2d expanded = box;
  expanded.expand(0.5);
  EXPECT_DOUBLE_EQ(expanded.min_x, -0.5);
  EXPECT_DOUBLE_EQ(expanded.min_y, -0.5);
  EXPECT_DOUBLE_EQ(expanded.max_x, 2.5);
  EXPECT_DOUBLE_EQ(expanded.max_y, 2.5);

  math::Aabb2d merged{1.0, 1.0, 1.0, 1.0};
  merged.merge_point({3.0, -1.0});
  EXPECT_DOUBLE_EQ(merged.min_x, 1.0);
  EXPECT_DOUBLE_EQ(merged.min_y, -1.0);
  EXPECT_DOUBLE_EQ(merged.max_x, 3.0);
  EXPECT_DOUBLE_EQ(merged.max_y, 1.0);
  EXPECT_DOUBLE_EQ(merged.max_abs_coordinate(), 3.0);

  EXPECT_DOUBLE_EQ(box.distance_sq_to({2.0, 2.0}), 0.0);
  EXPECT_DOUBLE_EQ(box.distance_sq_to({3.0, 3.0}), 2.0);
  expect_near(box.distance_to({3.0, 3.0}), std::sqrt(2.0));
  EXPECT_DOUBLE_EQ(box.distance_to({1.0, 1.0}), 0.0);

  const math::Aabb2d other{3.0, 3.0, 4.0, 4.0};
  expect_near(box.gap_x(other), 1.0);
  expect_near(box.gap_y(other), 1.0);
  EXPECT_DOUBLE_EQ(math::aabb_distance(box, other), std::sqrt(2.0));
  EXPECT_TRUE(math::aabb_within(box, other, std::sqrt(2.0)));
  EXPECT_FALSE(math::aabb_within(box, other, 1.0));
  EXPECT_DOUBLE_EQ(math::point_aabb_distance({3.0, 3.0}, box),
                   box.distance_to({3.0, 3.0}));
  EXPECT_DOUBLE_EQ(math::point_aabb_distance_sq({3.0, 3.0}, box),
                   box.distance_sq_to({3.0, 3.0}));

  const std::vector<math::Point2d> points{
      {1.0, -2.0}, {-3.0, 4.0}, {0.5, 0.0}};
  const math::Aabb2d hull = math::aabb_of(points);
  EXPECT_DOUBLE_EQ(hull.min_x, -3.0);
  EXPECT_DOUBLE_EQ(hull.min_y, -2.0);
  EXPECT_DOUBLE_EQ(hull.max_x, 1.0);
  EXPECT_DOUBLE_EQ(hull.max_y, 4.0);
  const math::Aabb2d raw = math::aabb_of_raw(points.data(), points.size());
  EXPECT_EQ(raw.min_x, hull.min_x);
  EXPECT_EQ(raw.min_y, hull.min_y);
  EXPECT_EQ(raw.max_x, hull.max_x);
  EXPECT_EQ(raw.max_y, hull.max_y);
  const math::Aabb2d empty = math::aabb_of_raw(points.data(), 0);
  EXPECT_DOUBLE_EQ(empty.min_x, 0.0);
  EXPECT_DOUBLE_EQ(empty.max_y, 0.0);
}

TEST(MathBicycleModel, CurvatureSteerAndTurnRadius) {
  const double wheelbase_m = 2.5;
  const double steer_rad = 0.6;
  const double curvature =
      math::kinematics::curvature_from_steer(steer_rad, wheelbase_m);
  expect_near(curvature, std::tan(steer_rad) / wheelbase_m);
  expect_near(
      math::kinematics::steer_from_curvature(curvature, wheelbase_m),
      steer_rad);
  expect_near(math::kinematics::min_turn_radius_m(wheelbase_m, steer_rad),
              wheelbase_m / std::tan(steer_rad));
  expect_near(math::kinematics::min_turn_radius_m(wheelbase_m, steer_rad),
              1.0 / curvature);
}

}  // namespace
