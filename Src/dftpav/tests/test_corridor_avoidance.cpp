#include <gtest/gtest.h>

#include "dftpav_test_support.hpp"

namespace planner::dftpav::test {
namespace {

// 运动学平滑迂回：圆角半径 12 m（最大曲率 0.083 1/m），绕过位于起点与终点
// 直线连线上的障碍物。局部车道走廊由该初值的保守足迹构建
// （公共 planner::common::Corridor 类型），额外留出 0.4 m 间隙，
// 使优化器在绕开障碍物的同时可以平滑转向。
struct DetourFixture {
  common::VehicleModel vehicle = default_vehicle();
  Rect obstacle{12.0, -1.0, 14.0, 1.0};
  std::vector<common::PathSegment> seed;
  common::Corridor corridor;
  std::vector<std::vector<common::Point2d>> obstacle_polygons;
};

DetourFixture make_detour_fixture() {
  DetourFixture fixture;
  const std::vector<common::Point2d> waypoints = {
      {0.0, 0.0}, {4.0, 0.0}, {16.0, 6.0}, {28.0, 0.0}, {34.0, 0.0}};
  fixture.seed = {make_fillet_segment(waypoints, 12.0, 0.1)};
  fixture.corridor =
      make_footprint_lane_corridor(fixture.seed, fixture.vehicle, 0.5, 0.6);
  fixture.obstacle_polygons = {rect_vertices_ccw(fixture.obstacle)};
  return fixture;
}

}  // namespace

TEST(DftpavCorridorAvoidance, DetoursAroundObstacleInsidePublicCorridor) {
  const DetourFixture fixture = make_detour_fixture();

  OptimizeRequest request;
  request.request_id = "test-detour";
  request.map_id = "test_map";
  request.map_version = "v1";
  request.vehicle = fixture.vehicle;
  request.seed = fixture.seed;
  request.corridor = fixture.corridor;
  request.limits = default_limits();
  // 让平滑后的轨迹保持在初值的站点结构上，使 validation 使用的
  // 走廊关联保持一致。
  request.options.anchor_weight = 10000.0;

  const OptimizeResult result = optimize_trajectory(request);
  ASSERT_TRUE(result.ok()) << "reason=" << result.reason << " detail=" << result.detail;
  EXPECT_EQ(result.stats.direction_changes, 0);

  const CheckResult semantics =
      check_trajectory_semantics(result.trajectory, request.limits, 1e-6, 1);
  EXPECT_TRUE(semantics.ok) << semantics.reason << " at t=" << semantics.value;

  // 独立安全检查：每个采样的真实车辆足迹都在走廊并集内，且不接触
  // 障碍物（接触 = 碰撞）。
  const CheckResult safety = check_corridor_and_obstacles(
      result.trajectory, fixture.vehicle, fixture.seed, fixture.corridor,
      fixture.obstacle_polygons, 0.0);
  EXPECT_TRUE(safety.ok) << safety.reason << " at t=" << safety.value;

  const common::TrajectoryPoint& last = result.trajectory.back();
  EXPECT_NEAR(last.pose.x_m, 34.0, 0.05);
  EXPECT_NEAR(last.pose.y_m, 0.0, 0.05);
  EXPECT_NEAR(last.velocity_mps, 0.0, 1e-6);
}

TEST(DftpavCorridorAvoidance, ConsumesGeneratedCorridorWithSideObstacles) {
  // 上方有一排障碍物的直线初值：走廊由公共 planner::corridor 生成器
  // 产出并被直接消费。
  const common::VehicleModel vehicle = default_vehicle();
  const std::vector<common::PathSegment> seed = {straight_segment(0.0, 0.0, 20.0, 0.0, 0.5)};
  const Rect obstacle{5.0, 2.0, 15.0, 3.0};
  const std::vector<corridor::Obstacle> obstacles = {corridor::make_rectangle_obstacle(
      "row", obstacle.x_min, obstacle.y_min, obstacle.x_max, obstacle.y_max)};

  corridor::CorridorRequest corridor_request;
  corridor_request.request_id = "test-channel";
  corridor_request.map_id = "test_map";
  corridor_request.map_version = "v1";
  corridor_request.vehicle = vehicle;
  corridor_request.seed = seed;
  corridor_request.obstacles = obstacles;
  const corridor::CorridorResult corridor_result =
      corridor::generate_corridor(corridor_request);
  ASSERT_TRUE(corridor_result.ok()) << corridor_result.reason << " "
                                    << corridor_result.detail;
  const corridor::CorridorVerification verification =
      corridor::verify_corridor(corridor_result.corridor, seed, obstacles, vehicle);
  EXPECT_TRUE(verification.valid) << verification.reason << " " << verification.detail;

  OptimizeRequest request;
  request.request_id = "test-channel-opt";
  request.map_id = "test_map";
  request.map_version = "v1";
  request.vehicle = vehicle;
  request.seed = seed;
  request.corridor = corridor_result.corridor;
  request.limits = default_limits();

  const OptimizeResult result = optimize_trajectory(request);
  ASSERT_TRUE(result.ok()) << "reason=" << result.reason << " detail=" << result.detail;

  const CheckResult safety =
      check_corridor_and_obstacles(result.trajectory, vehicle, seed,
                                   corridor_result.corridor, {rect_vertices_ccw(obstacle)}, 0.0);
  EXPECT_TRUE(safety.ok) << safety.reason << " at t=" << safety.value;

  const common::TrajectoryPoint& last = result.trajectory.back();
  EXPECT_NEAR(last.pose.x_m, 20.0, 0.05);
  EXPECT_NEAR(last.pose.y_m, 0.0, 0.05);
}

TEST(DftpavCorridorAvoidance, EmptyCorridorFromFailedGenerationIsRejected) {
  // 被阻塞的初值会让走廊生成器以空走廊失败；
  // 优化器必须将其作为显式输入错误拒绝。
  common::VehicleModel vehicle = default_vehicle();
  std::vector<common::PathSegment> seed = {straight_segment(0.0, 0.0, 10.0, 0.0, 0.1)};
  corridor::CorridorRequest corridor_request;
  corridor_request.request_id = "test-blocked";
  corridor_request.map_id = "test_map";
  corridor_request.map_version = "v1";
  corridor_request.vehicle = vehicle;
  corridor_request.seed = seed;
  // 障碍物填满初值周围的狭窄间隙。
  corridor_request.obstacles = {
      corridor::make_rectangle_obstacle("blocker", 4.0, -3.0, 6.0, 3.0)};
  const corridor::CorridorResult corridor_result =
      corridor::generate_corridor(corridor_request);
  EXPECT_FALSE(corridor_result.ok());

  OptimizeRequest request;
  request.request_id = "test-blocked-opt";
  request.map_id = "test_map";
  request.map_version = "v1";
  request.vehicle = vehicle;
  request.seed = seed;
  request.corridor = corridor_result.corridor;  // 失败时为空
  request.limits = default_limits();

  const OptimizeResult result = optimize_trajectory(request);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.reason, "empty_corridor");
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
  EXPECT_TRUE(result.trajectory.empty());
}

}  // namespace planner::dftpav::test
