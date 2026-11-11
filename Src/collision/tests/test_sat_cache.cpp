#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <limits>
#include <random>
#include <vector>

#include "planner/math/aabb2d.hpp"
#include "planner/math/angle.hpp"
#include "planner/math/polygon2d.hpp"
#include "planner/math/vec2d.hpp"
#include "sat_cache.hpp"

namespace {
using namespace planner;

TEST(SatCache, MatchesPublicIntersectionOnRandomInputs) {
  std::mt19937 engine(20251111);
  std::uniform_real_distribution<double> coordinate(-5.0, 5.0);
  std::uniform_real_distribution<double> tolerance(0.0, 0.25);
  std::uniform_int_distribution<int> point_count(3, 6);
  for (int i = 0; i < 2000; ++i) {
    SCOPED_TRACE(i);
    std::vector<math::Point2d> points(static_cast<std::size_t>(point_count(engine)));
    for (math::Point2d& p : points) {
      p = math::Point2d{coordinate(engine), coordinate(engine)};
    }
    const std::vector<math::Point2d> polygon = math::convex_hull(points);
    if (polygon.size() < 3 || polygon.size() > 8) {
      continue;
    }
    const double x = coordinate(engine);
    const double y = coordinate(engine);
    const math::Aabb2d box{x, y, x + tolerance(engine) + 1e-3, y + tolerance(engine) + 1e-3};
    const double tol = (i % 7 == 0) ? -1e-3 : tolerance(engine);
    collision::internal::PolygonSatCache8 cache;
    cache.build(polygon.data(), polygon.size());
    ASSERT_TRUE(cache.usable);
    EXPECT_EQ(cache.maybe_intersects(box, tol),
              math::convex_polygon_intersects_aabb(polygon, box, tol));
  }
}

TEST(SatCache, MatchesPublicIntersectionForInvalidTolerances) {
  const auto polygon = math::rectangle_polygon(-1.0, -1.0, 1.0, 1.0);
  collision::internal::PolygonSatCache8 cache;
  cache.build(polygon.data(), polygon.size());
  ASSERT_TRUE(cache.usable);
  const math::Aabb2d box{0.5, 0.5, 1.5, 1.5};
  for (double tol : {0.0, 1e-9, 0.25, -1e-3, std::numeric_limits<double>::quiet_NaN()}) {
    SCOPED_TRACE(tol);
    EXPECT_EQ(cache.maybe_intersects(box, tol),
              math::convex_polygon_intersects_aabb(polygon, box, tol));
  }
}

TEST(SatCache, FallsBackOutsideFastDomain) {
  // 1e80 坐标超出未归一化平方域的安全范围，缓存必须标记并回退至公共归一化 SAT 路径
  const auto polygon = math::rectangle_polygon(0.0, 0.0, 1.0e80, 1.0e80);
  ASSERT_EQ(polygon.size(), 4u);
  collision::internal::PolygonSatCache8 cache;
  cache.build(polygon.data(), polygon.size());
  ASSERT_TRUE(cache.usable);
  ASSERT_FALSE(cache.fast_domain) << "1e80 coordinates must select the normalized fallback";

  const math::Aabb2d inside{5.0e79, 5.0e79, 2.0e80, 2.0e80};
  const math::Aabb2d separated{2.0e80, 2.0e80, 3.0e80, 3.0e80};
  EXPECT_TRUE(cache.maybe_intersects(inside, 0.0));
  EXPECT_EQ(cache.maybe_intersects(inside, 0.0),
            math::convex_polygon_intersects_aabb(polygon, inside, 0.0));
  EXPECT_FALSE(cache.maybe_intersects(separated, 0.0));
  EXPECT_EQ(cache.maybe_intersects(separated, 0.0),
            math::convex_polygon_intersects_aabb(polygon, separated, 0.0));

  // 多边形位于快速域内、但查询 AABB 超出快速域时同样必须回退
  const auto unit = math::rectangle_polygon(0.0, 0.0, 1.0, 1.0);
  collision::internal::PolygonSatCache8 unit_cache;
  unit_cache.build(unit.data(), unit.size());
  ASSERT_TRUE(unit_cache.fast_domain);
  const math::Aabb2d far_box{1.0e80, 1.0e80, 2.0e80, 2.0e80};
  EXPECT_EQ(unit_cache.maybe_intersects(far_box, 0.0),
            math::convex_polygon_intersects_aabb(unit, far_box, 0.0));
}

TEST(SatCache, HandlesEmptyAndOverCapacityPolygons) {
  collision::internal::PolygonSatCache8 empty;
  empty.build(nullptr, 0);
  EXPECT_TRUE(empty.usable);
  EXPECT_FALSE(empty.maybe_intersects({0.0, 0.0, 1.0, 1.0}, 0.0))
      << "empty polygons never intersect";

  // 超过固定栈容量时缓存不可用；查询保守返回 true，调用方必须回退至通用相交测试
  std::vector<math::Point2d> nonagon;
  nonagon.reserve(9);
  for (int i = 0; i < 9; ++i) {
    const double angle = 2.0 * math::kPi * static_cast<double>(i) / 9.0;
    nonagon.push_back(math::Point2d{std::cos(angle), std::sin(angle)});
  }
  collision::internal::PolygonSatCache8 over_capacity;
  over_capacity.build(nonagon.data(), nonagon.size());
  EXPECT_FALSE(over_capacity.usable);
  EXPECT_TRUE(over_capacity.maybe_intersects({10.0, 10.0, 11.0, 11.0}, 0.0))
      << "over-capacity caches conservatively report potential intersection";
}

}  // namespace
