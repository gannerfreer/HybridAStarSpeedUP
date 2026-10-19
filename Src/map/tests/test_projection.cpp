// 投影配置与坐标往返测试（M2）。
// 验收容差：本地往返误差 <= 1e-3 m（方案第 7.1 节）。

#include <gtest/gtest.h>

#include <cmath>
#include <memory>

#include "test_support.hpp"

namespace planner::map {
namespace {

constexpr double kRoundTripToleranceM = 1e-3;

std::unique_ptr<Map> load_with(ProjectionKind kind) {
  auto result = test::load_mapping_example(kind);
  if (!result.ok()) {
    ADD_FAILURE() << "import failed for " << to_string(kind) << ": " << result.reason;
    return nullptr;
  }
  return std::move(result.map);
}

void check_round_trip(const Map& map, ProjectionKind kind) {
  const auto origin_local = map.to_local(test::example_origin());
  ASSERT_TRUE(origin_local.has_value());
  std::size_t samples = 0;
  for (double dx = -500.0; dx <= 500.0; dx += 125.0) {
    for (double dy = -500.0; dy <= 500.0; dy += 125.0) {
      const double x = origin_local->x_m + dx;
      const double y = origin_local->y_m + dy;
      const auto gps = map.to_gps(x, y, 0.0);
      ASSERT_TRUE(gps.has_value()) << to_string(kind) << " forward failed at " << x << ", " << y;
      const auto back = map.to_local(*gps);
      ASSERT_TRUE(back.has_value()) << to_string(kind) << " reverse failed at " << x << ", " << y;
      const double error = std::hypot(back->x_m - x, back->y_m - y);
      EXPECT_LE(error, kRoundTripToleranceM)
          << to_string(kind) << " round-trip error " << error << " m at offset (" << dx << ", "
          << dy << ")";
      ++samples;
    }
  }
  EXPECT_GE(samples, 81u);
}

TEST(MapProjection, utm_round_trip_within_tolerance) {
  const auto map = load_with(ProjectionKind::kUtm);
  ASSERT_TRUE(map != nullptr);
  check_round_trip(*map, ProjectionKind::kUtm);
}

TEST(MapProjection, mercator_round_trip_within_tolerance) {
  const auto map = load_with(ProjectionKind::kMercator);
  ASSERT_TRUE(map != nullptr);
  check_round_trip(*map, ProjectionKind::kMercator);
}

TEST(MapProjection, spherical_mercator_round_trip_within_tolerance) {
  const auto map = load_with(ProjectionKind::kSphericalMercator);
  ASSERT_TRUE(map != nullptr);
  check_round_trip(*map, ProjectionKind::kSphericalMercator);
}

TEST(MapProjection, gps_to_local_to_gps_round_trip) {
  const auto map = load_with(ProjectionKind::kUtm);
  ASSERT_TRUE(map != nullptr);
  for (double dlat = -0.002; dlat <= 0.002; dlat += 0.0005) {
    for (double dlon = -0.002; dlon <= 0.002; dlon += 0.0005) {
      const GeoPoint gps{49.0034 + dlat, 8.4242 + dlon, 0.0};
      const auto local = map->to_local(gps);
      ASSERT_TRUE(local.has_value());
      const auto back = map->to_gps(local->x_m, local->y_m, 0.0);
      ASSERT_TRUE(back.has_value());
      // 借助投影本身以米为单位比较。
      const auto local_back = map->to_local(*back);
      ASSERT_TRUE(local_back.has_value());
      EXPECT_LE(std::hypot(local_back->x_m - local->x_m, local_back->y_m - local->y_m),
                kRoundTripToleranceM);
    }
  }
}

TEST(MapProjection, rejects_invalid_coordinates) {
  const auto map = load_with(ProjectionKind::kUtm);
  ASSERT_TRUE(map != nullptr);
  EXPECT_FALSE(map->to_gps(std::nan(""), 0.0).has_value());
  EXPECT_FALSE(map->to_gps(0.0, std::nan(""), 0.0).has_value());
  EXPECT_FALSE(map->to_local(GeoPoint{95.0, 8.4, 0.0}).has_value());
  EXPECT_FALSE(map->to_local(GeoPoint{49.0, 181.0, 0.0}).has_value());
  EXPECT_FALSE(map->to_local(GeoPoint{std::nan(""), 8.4, 0.0}).has_value());
}

TEST(MapProjection, local_frame_is_metric_and_origin_anchored) {
  const auto map = load_with(ProjectionKind::kUtm);
  ASSERT_TRUE(map != nullptr);
  const auto origin_local = map->to_local(test::example_origin());
  ASSERT_TRUE(origin_local.has_value());
  // 投影原点不强制为 (0, 0)（UTM 保留带区坐标），但纬度上 1e-4 度的
  // 步长在本地坐标系中应约为 11.1 m。
  const auto north = map->to_local(GeoPoint{49.00345654351 + 1e-4, 8.42427590707, 0.0});
  ASSERT_TRUE(north.has_value());
  const double dy = north->y_m - origin_local->y_m;
  EXPECT_NEAR(dy, 11.1, 0.2);
}

}  // namespace
}  // namespace planner::map
