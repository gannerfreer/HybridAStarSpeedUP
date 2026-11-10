#include "planner/math/detail/sat_cache.hpp"
#include "planner/math/geometry.hpp"
#include "planner/math/kinematics/arc.hpp"

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
  expect_near(math::point_segment_distance_sq({2, 2}, {0, 0}, {0, 0}), 8.);
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

}  // namespace
