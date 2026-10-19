// 适配器契约测试（§6.3）：configure/plan/reset_cache、native 与 normalized
// 标签、不支持参数的报告、预算与取消检查、缓存失效和统计语义。

#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "test_support.hpp"

namespace planner::hybrid_astar {
namespace {

struct TestWorld {
  common::VehicleModel vehicle;
  common::GridView grid;
  collision::Scene scene;
  collision::CollisionChecker checker;
  common::PlanRequest request;

  // CollisionChecker 没有默认构造函数；先以空场景开始，场景填好后重建
  // checker。
  TestWorld() : checker(vehicle, scene) {}
};

TestWorld make_open_world() {
  TestWorld world;
  world.vehicle = test_support::load_default_vehicle();
  world.grid.map_id = "contract_open_map";
  world.grid.map_version = "v1";
  world.grid.resolution_m = 0.1;
  world.grid.width = 200;
  world.grid.height = 100;
  world.grid.origin = common::Pose2d{0.0, 0.0, 0.0};
  world.grid.cells.assign(
      static_cast<std::size_t>(world.grid.width) * world.grid.height,
      common::kCellFree);
  world.scene.grid = world.grid;
  world.checker = collision::CollisionChecker(world.vehicle, world.scene);
  world.request.request_id = "contract_request";
  world.request.map_id = world.grid.map_id;
  world.request.map_version = world.grid.map_version;
  world.request.vehicle = world.vehicle;
  world.request.start = common::Pose2d{2.0, 5.0, 0.0};
  world.request.goal = common::Pose2d{16.0, 5.0, 0.0};
  return world;
}

TestWorld make_obstacle_world() {
  TestWorld world = make_open_world();
  test_support::ObstacleSpec wall;
  wall.id = "contract_wall";
  wall.x_min_m = 8.0;
  wall.y_min_m = 0.0;
  wall.x_max_m = 10.0;
  wall.y_max_m = 6.0;
  test_support::rasterize_obstacles(&world.grid, {wall});
  world.scene.grid = world.grid;
  world.scene.obstacles.push_back(collision::Obstacle::rectangle(
      wall.id, wall.x_min_m, wall.y_min_m, wall.x_max_m, wall.y_max_m));
  world.checker = collision::CollisionChecker(world.vehicle, world.scene);
  return world;
}

common::PlanResult plan_with(RosFreeBackend& backend, TestWorld& world) {
  PlannerRsProvider provider(world.vehicle);
  return backend.plan(world.request, world.grid, world.checker, provider);
}

common::PlanResult plan_default(TestWorld& world) {
  RosFreeBackend backend;
  backend.configure(nlohmann::json::object());
  return plan_with(backend, world);
}

TEST(HybridAstarAdapter, registry_lists_ros_free_as_normalized) {
  const std::vector<BackendDescriptor> backends = list_backends();
  ASSERT_FALSE(backends.empty());
  bool found = false;
  for (const BackendDescriptor& descriptor : backends) {
    if (descriptor.id == "ros_free") {
      found = true;
      EXPECT_EQ(descriptor.kind, BackendKind::kNormalized);
    }
  }
  EXPECT_TRUE(found);

  EXPECT_EQ(create_backend("does_not_exist"), nullptr);
  const std::unique_ptr<HybridAStarBackend> backend = create_backend("ros_free");
  ASSERT_NE(backend, nullptr);
  EXPECT_EQ(backend->id(), "ros_free");
  EXPECT_EQ(backend->kind(), BackendKind::kNormalized);
  EXPECT_STREQ(to_string(backend->kind()), "normalized");
  EXPECT_STREQ(to_string(BackendKind::kNative), "native");
}

TEST(HybridAstarAdapter, configure_reports_defaults_and_support_tables) {
  RosFreeBackend backend;
  const ConfigReport report = backend.configure(nlohmann::json::object());
  EXPECT_TRUE(report.accepted) << report.reason;
  EXPECT_TRUE(report.unsupported.empty());
  ASSERT_TRUE(report.effective_config.is_object());
  EXPECT_EQ(report.effective_config.at("heading_bins").get<int>(), 72);
  EXPECT_DOUBLE_EQ(report.effective_config.at("motion_step_m").get<double>(),
                   2.0);

  const BackendInfo info = backend.info();
  EXPECT_EQ(info.id, "ros_free");
  EXPECT_EQ(info.kind, BackendKind::kNormalized);
  EXPECT_FALSE(info.parameters.empty());
  EXPECT_FALSE(info.statistics.empty());

  bool has_supported_statistic = false;
  bool has_unsupported_statistic = false;
  for (const ParameterSupport& support : info.statistics) {
    has_supported_statistic = has_supported_statistic || support.supported;
    has_unsupported_statistic = has_unsupported_statistic || !support.supported;
    if (!support.supported) {
      EXPECT_FALSE(support.note.empty());
    }
  }
  EXPECT_TRUE(has_supported_statistic);
  EXPECT_TRUE(has_unsupported_statistic);
}

TEST(HybridAstarAdapter, unknown_parameter_is_reported_and_plan_refuses) {
  RosFreeBackend backend;
  const ConfigReport report =
      backend.configure(nlohmann::json::object({{"wobble", 3}}));
  EXPECT_TRUE(report.accepted);
  ASSERT_EQ(report.unsupported.size(), 1u);
  EXPECT_NE(report.unsupported.front().find("wobble"), std::string::npos);

  TestWorld world = make_open_world();
  const common::PlanResult result = plan_with(backend, world);
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
  EXPECT_NE(result.reason.find("wobble"), std::string::npos);
}

TEST(HybridAstarAdapter, unsupported_smooth_true_is_not_ignored) {
  RosFreeBackend backend;
  const ConfigReport report =
      backend.configure(nlohmann::json::object({{"smooth", true}}));
  EXPECT_TRUE(report.accepted);
  ASSERT_EQ(report.unsupported.size(), 1u);
  EXPECT_NE(report.unsupported.front().find("smooth"), std::string::npos);

  TestWorld world = make_open_world();
  const common::PlanResult result = plan_with(backend, world);
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
  EXPECT_NE(result.reason.find("smooth"), std::string::npos);

  // `smooth: false` 与实现一致，会被接受。
  const ConfigReport accepted =
      backend.configure(nlohmann::json::object({{"smooth", false}}));
  EXPECT_TRUE(accepted.accepted);
  EXPECT_TRUE(accepted.unsupported.empty());
  const common::PlanResult result_ok = plan_with(backend, world);
  EXPECT_EQ(result_ok.status, common::Status::kSuccess) << result_ok.reason;
}

TEST(HybridAstarAdapter, wrong_type_rejects_whole_configuration) {
  RosFreeBackend backend;
  const ConfigReport report = backend.configure(
      nlohmann::json::object({{"heuristic_weight", "fast"}}));
  EXPECT_FALSE(report.accepted);
  EXPECT_NE(report.reason.find("heuristic_weight"), std::string::npos);

  TestWorld world = make_open_world();
  const common::PlanResult result = plan_with(backend, world);
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
  EXPECT_NE(result.reason.find("configuration was rejected"),
            std::string::npos);
}

TEST(HybridAstarAdapter, cancelled_request_returns_cancelled) {
  TestWorld world = make_open_world();
  world.request.cancelled = true;
  const common::PlanResult result = plan_default(world);
  EXPECT_EQ(result.status, common::Status::kCancelled);
  EXPECT_TRUE(result.path.empty());
  EXPECT_EQ(result.request_id, "contract_request");
}

TEST(HybridAstarAdapter, cancel_callback_stops_the_expansion_loop) {
  TestWorld world = make_obstacle_world();
  RosFreeBackend backend;
  ASSERT_TRUE(backend.configure(nlohmann::json::object()).accepted);
  backend.set_cancel_callback([]() { return true; });
  PlannerRsProvider provider(world.vehicle);
  const common::PlanResult result =
      backend.plan(world.request, world.grid, world.checker, provider);
  EXPECT_EQ(result.status, common::Status::kCancelled);
  EXPECT_FALSE(result.reason.empty());
}

TEST(HybridAstarAdapter, node_budget_exhaustion_is_timeout) {
  TestWorld world = make_open_world();
  world.request.node_budget = 1;
  RosFreeBackend backend;
  // 较小的 RS 长度上限使第一次解析尝试不会成功，从而真正进入扩展循环
  //（进而消耗节点预算）。
  ASSERT_TRUE(
      backend
          .configure(nlohmann::json::object(
              {{"analytic_expansion_max_length_m", 0.001}}))
          .accepted);
  PlannerRsProvider provider(world.vehicle);
  const common::PlanResult result =
      backend.plan(world.request, world.grid, world.checker, provider);
  EXPECT_EQ(result.status, common::Status::kTimeout);
  EXPECT_NE(result.reason.find("node budget"), std::string::npos);
}

TEST(HybridAstarAdapter, time_budget_exhaustion_is_timeout) {
  TestWorld world = make_open_world();
  world.request.time_budget_s = 1e-9;
  const common::PlanResult result = plan_default(world);
  EXPECT_EQ(result.status, common::Status::kTimeout);
  EXPECT_FALSE(result.reason.empty());
}

TEST(HybridAstarAdapter, reset_cache_invalidates_hot_start) {
  TestWorld world = make_open_world();
  RosFreeBackend backend;
  ASSERT_TRUE(backend.configure(nlohmann::json::object()).accepted);
  PlannerRsProvider provider(world.vehicle);

  const common::PlanResult first =
      backend.plan(world.request, world.grid, world.checker, provider);
  ASSERT_EQ(first.status, common::Status::kSuccess) << first.reason;
  EXPECT_EQ(first.statistics.at("cache_mode").get<std::string>(), "cold");

  const common::PlanResult second =
      backend.plan(world.request, world.grid, world.checker, provider);
  ASSERT_EQ(second.status, common::Status::kSuccess) << second.reason;
  EXPECT_EQ(second.statistics.at("cache_mode").get<std::string>(), "warm");
  EXPECT_EQ(second.statistics.at("heuristic_cache_reused").get<bool>(), true);

  backend.reset_cache();
  const common::PlanResult third =
      backend.plan(world.request, world.grid, world.checker, provider);
  ASSERT_EQ(third.status, common::Status::kSuccess) << third.reason;
  EXPECT_EQ(third.statistics.at("cache_mode").get<std::string>(), "cold");
  EXPECT_EQ(third.statistics.at("heuristic_cache_reused").get<bool>(), false);
}

TEST(HybridAstarAdapter, vehicle_mismatch_is_invalid_input) {
  TestWorld world = make_open_world();
  common::VehicleModel other_vehicle = world.vehicle;
  other_vehicle.wheelbase_m = 3.4;
  const collision::CollisionChecker other_checker(other_vehicle, world.scene);
  PlannerRsProvider other_provider(other_vehicle);
  RosFreeBackend backend;
  ASSERT_TRUE(backend.configure(nlohmann::json::object()).accepted);
  const common::PlanResult result = backend.plan(
      world.request, world.grid, other_checker, other_provider);
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
  EXPECT_NE(result.reason.find("vehicle"), std::string::npos);
}

TEST(HybridAstarAdapter, scene_grid_mismatch_is_invalid_input) {
  TestWorld world = make_open_world();
  common::GridView other_grid = world.grid;
  other_grid.map_id = "another_map";
  world.request.map_id = other_grid.map_id;
  world.request.map_version = other_grid.map_version;
  RosFreeBackend backend;
  ASSERT_TRUE(backend.configure(nlohmann::json::object()).accepted);
  PlannerRsProvider provider(world.vehicle);
  const common::PlanResult result =
      backend.plan(world.request, other_grid, world.checker, provider);
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
  EXPECT_NE(result.reason.find("scene grid"), std::string::npos);
}

TEST(HybridAstarAdapter, statistics_leave_unsupported_metrics_absent) {
  TestWorld world = make_open_world();
  const common::PlanResult result = plan_default(world);
  ASSERT_EQ(result.status, common::Status::kSuccess) << result.reason;

  EXPECT_TRUE(result.statistics.contains("expanded_nodes"));
  EXPECT_TRUE(result.statistics.contains("collision_checks"));
  EXPECT_TRUE(result.statistics.contains("path_length_m"));
  EXPECT_TRUE(result.statistics.contains("total_time_ms"));
  EXPECT_FALSE(result.statistics.contains("cpu_ms"));
  EXPECT_FALSE(result.statistics.contains("peak_rss_bytes"));
  EXPECT_FALSE(result.statistics.contains("smooth_time_ms"));

  ASSERT_TRUE(result.statistics.contains("unsupported_statistics"));
  const nlohmann::json& unsupported =
      result.statistics.at("unsupported_statistics");
  ASSERT_TRUE(unsupported.is_array());
  bool has_cpu_entry = false;
  for (const nlohmann::json& entry : unsupported) {
    ASSERT_TRUE(entry.contains("name"));
    ASSERT_TRUE(entry.contains("reason"));
    EXPECT_FALSE(entry.at("reason").get<std::string>().empty());
    if (entry.at("name") == "cpu_ms") {
      has_cpu_entry = true;
    }
  }
  EXPECT_TRUE(has_cpu_entry);
}

}  // namespace
}  // namespace planner::hybrid_astar
