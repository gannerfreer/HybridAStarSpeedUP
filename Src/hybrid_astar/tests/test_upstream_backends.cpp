// Wave-5 上游后端的契约、别名映射与冒烟测试。
//
// 六个上游系列后端（`smac_*`、`kth_*`、`zm0612_*`）是对锁定上游搜索结构
// 的 ROS-free 复现（见 docs/upstream_notes.md）；`native` 类别仅供参考，
// `normalized` 类别遵循统一契约并参与选择。这些测试覆盖注册表、参数别名、
// 不支持键的报告、预算、取消、缓存失效和一个小的 v2 夹具冒烟集。

#include <gtest/gtest.h>

#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "test_support.hpp"

namespace planner::hybrid_astar {
namespace {

struct World {
  common::VehicleModel vehicle;
  common::GridView grid;
  collision::Scene scene;
  collision::CollisionChecker checker;
  common::PlanRequest request;

  World() : checker(vehicle, scene) {}
};

World make_open_world() {
  World world;
  world.vehicle = test_support::load_default_vehicle();
  world.grid.map_id = "upstream_open_map";
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
  world.request.request_id = "upstream_contract_request";
  world.request.map_id = world.grid.map_id;
  world.request.map_version = world.grid.map_version;
  world.request.vehicle = world.vehicle;
  world.request.start = common::Pose2d{2.0, 5.0, 0.0};
  world.request.goal = common::Pose2d{16.0, 5.0, 0.0};
  return world;
}

TEST(UpstreamBackends, registry_lists_nine_backends) {
  const std::map<std::string, BackendKind> expected = {
      {"ros_free", BackendKind::kNormalized},
      {"smac_native", BackendKind::kNative},
      {"smac_normalized", BackendKind::kNormalized},
      {"kth_native", BackendKind::kNative},
      {"kth_normalized", BackendKind::kNormalized},
      {"zm0612_native", BackendKind::kNative},
      {"zm0612_normalized", BackendKind::kNormalized},
      {"user_astar_raw", BackendKind::kNative},
      {"user_astar_normalized", BackendKind::kNormalized}};
  const std::vector<BackendDescriptor> backends = list_backends();
  ASSERT_EQ(backends.size(), expected.size());
  for (const BackendDescriptor& descriptor : backends) {
    const auto found = expected.find(descriptor.id);
    ASSERT_NE(found, expected.end()) << "unexpected backend id "
                                     << descriptor.id;
    EXPECT_EQ(descriptor.kind, found->second) << descriptor.id;
    EXPECT_FALSE(descriptor.description.empty());
  }
  EXPECT_EQ(create_backend("unknown_backend"), nullptr);
}

TEST(UpstreamBackends, every_backend_accepts_default_configuration) {
  for (const BackendDescriptor& descriptor : list_backends()) {
    SCOPED_TRACE(descriptor.id);
    const std::unique_ptr<HybridAStarBackend> backend =
        create_backend(descriptor.id);
    ASSERT_NE(backend, nullptr);
    const ConfigReport report = backend->configure(nlohmann::json::object());
    EXPECT_TRUE(report.accepted) << report.reason;
    EXPECT_TRUE(report.unsupported.empty());
    EXPECT_EQ(backend->id(), descriptor.id);
    EXPECT_EQ(backend->kind(), descriptor.kind);
    EXPECT_STREQ(to_string(backend->kind()), to_string(descriptor.kind));
    const BackendInfo info = backend->info();
    EXPECT_EQ(info.id, descriptor.id);
    EXPECT_FALSE(info.parameters.empty());
    EXPECT_FALSE(info.statistics.empty());
    EXPECT_TRUE(report.effective_config.contains("xy_resolution_m"));
  }
}

TEST(UpstreamBackends, upstream_alias_keys_map_to_core_parameters) {
  {
    const std::unique_ptr<HybridAStarBackend> backend =
        create_backend("zm0612_native");
    const ConfigReport report =
        backend->configure(nlohmann::json::object({{"steering_penalty", 1.7},
                                                   {"reversing_penalty", 2.4},
                                                   {"shot_distance", 4.5}}));
    ASSERT_TRUE(report.accepted) << report.reason;
    EXPECT_DOUBLE_EQ(backend->effective_config().at("turn_penalty").get<double>(),
                     1.7);
    EXPECT_DOUBLE_EQ(backend->effective_config().at("reverse_cost").get<double>(),
                     2.4);
    EXPECT_DOUBLE_EQ(
        backend->effective_config().at("analytic_shot_distance_m").get<double>(),
        4.5);
  }
  {
    const std::unique_ptr<HybridAStarBackend> backend =
        create_backend("kth_native");
    const ConfigReport report = backend->configure(
        nlohmann::json::object({{"cellSize", 0.75}, {"headings", 144}}));
    ASSERT_TRUE(report.accepted) << report.reason;
    EXPECT_DOUBLE_EQ(
        backend->effective_config().at("xy_resolution_m").get<double>(), 0.75);
    EXPECT_EQ(backend->effective_config().at("heading_bins").get<int>(), 144);
  }
  {
    const std::unique_ptr<HybridAStarBackend> backend =
        create_backend("smac_native");
    const ConfigReport report = backend->configure(nlohmann::json::object(
        {{"minimum_turning_radius", 6.0}, {"analytic_expansion_ratio", 3.5}}));
    ASSERT_TRUE(report.accepted) << report.reason;
    EXPECT_DOUBLE_EQ(
        backend->effective_config().at("min_turn_radius_m").get<double>(), 6.0);
    // 3.5 会四舍五入为最近的整数扩展比。
    EXPECT_EQ(
        backend->effective_config().at("analytic_expansion_ratio").get<int>(),
        4);
  }
}

TEST(UpstreamBackends, documented_unsupported_keys_are_reported_and_refuse) {
  const std::unique_ptr<HybridAStarBackend> backend =
      create_backend("zm0612_native");
  const ConfigReport report =
      backend->configure(nlohmann::json::object({{"wheel_base", 1.0}}));
  EXPECT_TRUE(report.accepted);
  ASSERT_EQ(report.unsupported.size(), 1u);
  EXPECT_NE(report.unsupported.front().find("PlanRequest"), std::string::npos);

  World world = make_open_world();
  PlannerRsProvider provider(world.vehicle);
  const common::PlanResult result =
      backend->plan(world.request, world.grid, world.checker, provider);
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
  EXPECT_NE(result.reason.find("wheel_base"), std::string::npos);
}

TEST(UpstreamBackends, native_budget_cancel_and_cache_contract) {
  for (const char* id : {"smac_native", "kth_native", "zm0612_native"}) {
    SCOPED_TRACE(id);
    World world = make_open_world();
    const std::unique_ptr<HybridAStarBackend> backend = create_backend(id);
    ASSERT_NE(backend, nullptr);
    PlannerRsProvider provider(world.vehicle);

    // 节点预算：限制解析连接，使扩展循环真正运行。
    world.request.node_budget = 1;
    ASSERT_TRUE(backend
                    ->configure(nlohmann::json::object(
                        {{"analytic_expansion_max_length_m", 0.001},
                         {"analytic_shot_distance_m", 0.001}}))
                    .accepted);
    common::PlanResult result =
        backend->plan(world.request, world.grid, world.checker, provider);
    EXPECT_EQ(result.status, common::Status::kTimeout) << result.reason;

    // 取消。
    world.request.node_budget.reset();
    ASSERT_TRUE(backend->configure(nlohmann::json::object()).accepted);
    world.request.cancelled = true;
    result = backend->plan(world.request, world.grid, world.checker, provider);
    EXPECT_EQ(result.status, common::Status::kCancelled);

    // 缓存周期：cold -> warm -> reset -> cold。
    world.request.cancelled = false;
    result = backend->plan(world.request, world.grid, world.checker, provider);
    ASSERT_EQ(result.status, common::Status::kSuccess) << result.reason;
    EXPECT_EQ(result.statistics.at("cache_mode").get<std::string>(), "cold");
    result = backend->plan(world.request, world.grid, world.checker, provider);
    ASSERT_EQ(result.status, common::Status::kSuccess) << result.reason;
    EXPECT_EQ(result.statistics.at("cache_mode").get<std::string>(), "warm");
    backend->reset_cache();
    result = backend->plan(world.request, world.grid, world.checker, provider);
    ASSERT_EQ(result.status, common::Status::kSuccess) << result.reason;
    EXPECT_EQ(result.statistics.at("cache_mode").get<std::string>(), "cold");
  }
}

TEST(UpstreamBackends, native_smoke_on_v2_fixtures) {
  const common::VehicleModel vehicle = test_support::load_default_vehicle();
  const std::vector<test_support::PathCase> cases =
      test_support::load_path_cases();
  const std::vector<std::string> backend_ids = {"smac_native", "kth_native",
                                                "zm0612_native"};
  const std::vector<std::string> case_ids = {
      "open_101", "obstacle_106", "corridor_112",
      "park_116", "deadend_121", "unreachable_127"};
  for (const std::string& backend_id : backend_ids) {
    for (const std::string& case_id : case_ids) {
      SCOPED_TRACE(backend_id + " / " + case_id);
      const test_support::PathCase* path_case = nullptr;
      for (const test_support::PathCase& candidate : cases) {
        if (candidate.id == case_id) {
          path_case = &candidate;
          break;
        }
      }
      ASSERT_NE(path_case, nullptr);
      const test_support::PreparedCase prepared =
          test_support::prepare_case(*path_case, vehicle);
      ASSERT_TRUE(prepared.checker.valid()) << prepared.checker.invalid_reason();

      const std::unique_ptr<HybridAStarBackend> backend =
          create_backend(backend_id);
      ASSERT_NE(backend, nullptr);
      ASSERT_TRUE(backend->configure(nlohmann::json::object()).accepted);
      PlannerRsProvider provider(vehicle);
      common::PlanRequest request = prepared.request;
      request.time_budget_s = 60.0;

      const common::PlanResult result =
          backend->plan(request, prepared.grid, prepared.checker, provider);
      if (path_case->expected_status == "success") {
        EXPECT_EQ(result.status, common::Status::kSuccess)
            << result.reason << " stats: " << result.statistics.dump();
        if (result.status == common::Status::kSuccess) {
          const std::string error = test_support::validation_error(
              result, path_case->start, path_case->goal, vehicle);
          EXPECT_TRUE(error.empty()) << error;
          const collision::CheckResult collision =
              prepared.checker.check_path(result.path);
          EXPECT_TRUE(collision.is_clear())
              << test_support::describe_collision(collision);
        }
      } else {
        EXPECT_NE(result.status, common::Status::kSuccess);
        EXPECT_EQ(result.status,
                  test_support::expected_status_for(
                      path_case->expected_failure_reason))
            << result.reason;
      }
    }
  }
}

TEST(UpstreamBackends, user_astar_alias_keys_map_to_core_parameters) {
  const std::unique_ptr<HybridAStarBackend> backend =
      create_backend("user_astar_raw");
  const ConfigReport report = backend->configure(nlohmann::json::object(
      {{"grid_dist", 0.3},
       {"delta_dist", 0.4},
       {"angle_discrete_num", 15.0},
       {"turnning_penalty", 2.5},
       {"max_iterations_astar", 500000.0}}));
  ASSERT_TRUE(report.accepted) << report.reason;
  const nlohmann::json& effective = backend->effective_config();
  EXPECT_DOUBLE_EQ(effective.at("xy_resolution_m").get<double>(), 0.3);
  EXPECT_DOUBLE_EQ(effective.at("motion_step_m").get<double>(), 0.4);
  EXPECT_EQ(effective.at("steering_samples").get<int>(), 15);
  EXPECT_DOUBLE_EQ(effective.at("turn_penalty").get<double>(), 2.5);
  EXPECT_EQ(effective.at("default_node_budget").get<std::int64_t>(), 500000);
}

TEST(UpstreamBackends, user_astar_tuning_parameters_round_trip) {
  const std::unique_ptr<HybridAStarBackend> backend =
      create_backend("user_astar_normalized");
  const ConfigReport report = backend->configure(nlohmann::json::object(
      {{"min_connector_segment_m", 0.3},
       {"analytic_expansion_max_length_m", 15.0}}));
  ASSERT_TRUE(report.accepted) << report.reason;
  EXPECT_DOUBLE_EQ(
      backend->effective_config().at("min_connector_segment_m").get<double>(),
      0.3);
  EXPECT_DOUBLE_EQ(
      backend->effective_config()
          .at("analytic_expansion_max_length_m")
          .get<double>(),
      15.0);
  // 0 = 关闭退化连接拒绝 / 长度上限；负值被拒绝。
  const ConfigReport disabled = backend->configure(nlohmann::json::object(
      {{"min_connector_segment_m", 0.0},
       {"analytic_expansion_max_length_m", 0.0}}));
  EXPECT_TRUE(disabled.accepted) << disabled.reason;
  const ConfigReport rejected = backend->configure(
      nlohmann::json::object({{"min_connector_segment_m", -1.0}}));
  EXPECT_FALSE(rejected.accepted);
}

TEST(UpstreamBackends, user_astar_unsupported_keys_are_reported_and_refuse) {
  const std::unique_ptr<HybridAStarBackend> backend =
      create_backend("user_astar_normalized");
  const ConfigReport report =
      backend->configure(nlohmann::json::object({{"min_steering", -20.0}}));
  EXPECT_TRUE(report.accepted);
  ASSERT_EQ(report.unsupported.size(), 1u);
  EXPECT_EQ(report.unsupported.front(), "min_steering");

  World world = make_open_world();
  PlannerRsProvider provider(world.vehicle);
  const common::PlanResult result =
      backend->plan(world.request, world.grid, world.checker, provider);
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
  EXPECT_NE(result.reason.find("min_steering"), std::string::npos);
}

TEST(UpstreamBackends, user_astar_normalized_budget_and_cancel) {
  World world = make_open_world();
  const std::unique_ptr<HybridAStarBackend> backend =
      create_backend("user_astar_normalized");
  ASSERT_TRUE(backend->configure(nlohmann::json::object()).accepted);
  PlannerRsProvider provider(world.vehicle);

  world.request.node_budget = 1;
  common::PlanResult result =
      backend->plan(world.request, world.grid, world.checker, provider);
  EXPECT_EQ(result.status, common::Status::kTimeout) << result.reason;
  // 预检在此调用中计算并缓存目标势能（即使搜索随后超时）。
  ASSERT_TRUE(result.statistics.contains("cache_mode"));
  EXPECT_EQ(result.statistics.at("cache_mode").get<std::string>(), "cold");

  world.request.node_budget.reset();
  world.request.cancelled = true;
  result = backend->plan(world.request, world.grid, world.checker, provider);
  EXPECT_EQ(result.status, common::Status::kCancelled);

  world.request.cancelled = false;
  result = backend->plan(world.request, world.grid, world.checker, provider);
  ASSERT_EQ(result.status, common::Status::kSuccess) << result.reason;
  EXPECT_EQ(result.statistics.at("cache_mode").get<std::string>(), "warm");

  backend->reset_cache();
  result = backend->plan(world.request, world.grid, world.checker, provider);
  ASSERT_EQ(result.status, common::Status::kSuccess) << result.reason;
  EXPECT_EQ(result.statistics.at("cache_mode").get<std::string>(), "cold");
}

TEST(UpstreamBackends, user_astar_normalized_reachability_precheck) {
  // unreachable_127 是贯穿网格的整墙：自由栅格松弛不连通时预检直接
  // no_path（0 扩展、0 原语、仅起/终点两次位姿检查），第二次调用复用
  // 目标势能（warm）。这替代了此前扩展到前沿耗尽（35418 次扩展）的行为。
  const common::VehicleModel vehicle = test_support::load_default_vehicle();
  const std::vector<test_support::PathCase> cases =
      test_support::load_path_cases();
  const test_support::PathCase* path_case = nullptr;
  for (const test_support::PathCase& candidate : cases) {
    if (candidate.id == "unreachable_127") {
      path_case = &candidate;
      break;
    }
  }
  ASSERT_NE(path_case, nullptr);
  const test_support::PreparedCase prepared =
      test_support::prepare_case(*path_case, vehicle);
  ASSERT_TRUE(prepared.checker.valid()) << prepared.checker.invalid_reason();

  const std::unique_ptr<HybridAStarBackend> backend =
      create_backend("user_astar_normalized");
  ASSERT_TRUE(backend->configure(nlohmann::json::object()).accepted);
  PlannerRsProvider provider(vehicle);
  common::PlanRequest request = prepared.request;
  request.time_budget_s = 60.0;

  const common::PlanResult first =
      backend->plan(request, prepared.grid, prepared.checker, provider);
  EXPECT_EQ(first.status, common::Status::kNoPath) << first.reason;
  EXPECT_EQ(first.statistics.at("expanded_nodes").get<std::int64_t>(), 0);
  EXPECT_EQ(first.statistics.at("motion_primitives").get<std::int64_t>(), 0);
  EXPECT_EQ(first.statistics.at("cache_mode").get<std::string>(), "cold");
  EXPECT_TRUE(first.statistics.contains("pruned_unreachable_states"));

  const common::PlanResult second =
      backend->plan(request, prepared.grid, prepared.checker, provider);
  EXPECT_EQ(second.status, common::Status::kNoPath) << second.reason;
  EXPECT_EQ(second.statistics.at("expanded_nodes").get<std::int64_t>(), 0);
  EXPECT_EQ(second.statistics.at("cache_mode").get<std::string>(), "warm");
}

TEST(UpstreamBackends, user_astar_normalized_rejects_mismatched_checker_grid) {
  World world = make_open_world();
  const std::unique_ptr<HybridAStarBackend> backend =
      create_backend("user_astar_normalized");
  ASSERT_TRUE(backend->configure(nlohmann::json::object()).accepted);
  PlannerRsProvider provider(world.vehicle);

  // 同一标识但单元值不同：目标势场必须与碰撞检查器使用同一栅格快照。
  common::GridView other = world.grid;
  other.cells[0] = common::kCellOccupied;
  const common::PlanResult result =
      backend->plan(world.request, other, world.checker, provider);
  EXPECT_EQ(result.status, common::Status::kInvalidInput);
  EXPECT_NE(result.reason.find("grid"), std::string::npos) << result.reason;
}

TEST(UpstreamBackends, user_astar_smoke_on_v2_fixtures) {
  const common::VehicleModel vehicle = test_support::load_default_vehicle();
  const std::vector<test_support::PathCase> cases =
      test_support::load_path_cases();
  const std::vector<std::string> backend_ids = {"user_astar_raw",
                                                "user_astar_normalized"};
  const std::vector<std::string> case_ids = {
      "open_101", "obstacle_106", "corridor_112",
      "park_116", "deadend_121", "unreachable_127"};
  for (const std::string& backend_id : backend_ids) {
    for (const std::string& case_id : case_ids) {
      SCOPED_TRACE(backend_id + " / " + case_id);
      const test_support::PathCase* path_case = nullptr;
      for (const test_support::PathCase& candidate : cases) {
        if (candidate.id == case_id) {
          path_case = &candidate;
          break;
        }
      }
      ASSERT_NE(path_case, nullptr);
      const test_support::PreparedCase prepared =
          test_support::prepare_case(*path_case, vehicle);
      ASSERT_TRUE(prepared.checker.valid()) << prepared.checker.invalid_reason();

      const std::unique_ptr<HybridAStarBackend> backend =
          create_backend(backend_id);
      ASSERT_NE(backend, nullptr);
      ASSERT_TRUE(backend->configure(nlohmann::json::object()).accepted);
      PlannerRsProvider provider(vehicle);
      common::PlanRequest request = prepared.request;
      request.time_budget_s = 60.0;

      const common::PlanResult result =
          backend->plan(request, prepared.grid, prepared.checker, provider);
      if (backend_id == "user_astar_raw") {
        // raw 为参考项：其内置碰撞/目标容差与契约预期可能不同（记录实际
        // 状态即可，不在本测试断言成功/失败一致性）。
        RecordProperty("raw_" + case_id + "_status",
                       common::to_string(result.status));
        continue;
      }
      if (path_case->expected_status == "success") {
        EXPECT_EQ(result.status, common::Status::kSuccess)
            << result.reason << " stats: " << result.statistics.dump();
      } else {
        EXPECT_NE(result.status, common::Status::kSuccess) << result.reason;
      }
    }
  }
}

}  // namespace
}  // namespace planner::hybrid_astar
