// 流水线协作式取消的确定性单元测试（不启动服务进程）。
//
// 覆盖服务侧接入：`run_pipeline` 在 Dftpav 阶段把作业取消状态传入优化器
// （`OptimizeOptions::cancel_check`），使取消在**阶段内部**生效——优化器
// 返回 kCancelled/"cancelled" 且不带任何轨迹，而不是先跑完整求解再在阶段
// 边界丢弃结果。取消的对外语义与既有契约一致（status=cancelled、
// reason="cancelled"、无几何、job 层由调用方映射为 cancelled）。

#include <gtest/gtest.h>

#include <atomic>
#include <memory>
#include <string>

#include "map_fixture.hpp"
#include "planner/service/pipeline.hpp"
#include "planner/service/scene.hpp"
#include "planner/service/types.hpp"

namespace planner::service {
namespace {

// 场景域为真实 v3 地图的 lanelet 可行驶域；请求使用路面上的固定点。
std::unique_ptr<map::Map> load_test_map() {
  auto result = test::load_example_map();
  return result.ok() ? std::move(result.map) : nullptr;
}

ServicePlanRequest open_request(const map::MapInfo& info) {
  ServicePlanRequest request;
  request.request_id = "req-cancel";
  request.map_id = info.map_id;
  request.map_version = info.map_version;
  request.vehicle = default_vehicle();
  request.start = common::Pose2d{2.0, 5.0, 0.0};
  request.goal = common::Pose2d{18.0, 5.0, 0.0};
  return request;
}

// 对照组：同一请求不取消时成功，证明取消才是结果差异的原因。
TEST(PlanPipelineCancel, OpenRequestSucceedsWithoutCancellation) {
  const std::unique_ptr<map::Map> map = load_test_map();
  ASSERT_TRUE(map != nullptr);
  const ServicePlanRequest request = open_request(map->info());
  const SceneBuildResult scene = build_scene(*map, request.vehicle, request.start,
                                             request.goal, {});
  ASSERT_TRUE(scene.ok) << scene.reason << " " << scene.detail;

  const PipelineResult result =
      run_pipeline(request, scene.grid, scene.scene, {}, {});
  ASSERT_EQ(result.status, common::Status::kSuccess)
      << result.reason << " " << result.detail;
  EXPECT_EQ(result.stage, stage::kDone);
  EXPECT_FALSE(result.optimize.trajectory.empty());
}

// 取消在 Dftpav 阶段内部生效：阶段开始后第 3 次轮询即请求取消（阶段边界
// 检查与优化器入口检查之后，Dftpav 求解内部的轮询点），因此
// result.optimize 必须已经是 kCancelled 而不是完整求解的成功结果。
TEST(PlanPipelineCancel, CancelsInsideOptimizeStageWithoutGeometry) {
  const std::unique_ptr<map::Map> map = load_test_map();
  ASSERT_TRUE(map != nullptr);
  const ServicePlanRequest request = open_request(map->info());
  const SceneBuildResult scene = build_scene(*map, request.vehicle, request.start,
                                             request.goal, {});
  ASSERT_TRUE(scene.ok) << scene.reason << " " << scene.detail;

  std::atomic<bool> armed{false};
  int polls_after_arm = 0;
  const CancelCallback cancelled = [&] {
    if (!armed.load()) {
      return false;
    }
    return ++polls_after_arm >= 3;
  };
  const StageCallback on_stage = [&](const std::string& stage_name) {
    if (stage_name == stage::kOptimize) {
      armed.store(true);
    }
  };

  const PipelineResult result =
      run_pipeline(request, scene.grid, scene.scene, on_stage, cancelled);

  // 既有取消语义：稳定状态/原因，无几何。
  EXPECT_EQ(result.status, common::Status::kCancelled);
  EXPECT_EQ(result.reason, "cancelled");
  EXPECT_EQ(result.stage, stage::kOptimize);  // 从未进入 validate
  EXPECT_FALSE(result.detail.empty());

  // 前序阶段正常完成；取消命中 Dftpav 阶段。
  EXPECT_TRUE(result.hybrid_ran);
  EXPECT_EQ(result.hybrid.status, common::Status::kSuccess);
  EXPECT_TRUE(result.corridor_ran);
  EXPECT_EQ(result.corridor.status, common::Status::kSuccess);
  EXPECT_TRUE(result.optimize_ran);

  // 关键证据：优化器自身的取消钩子观察到取消（否则 result.optimize 会是
  // 完整求解的 kSuccess，只有阶段返回后才会被取消）。
  EXPECT_EQ(result.optimize.status, common::Status::kCancelled);
  EXPECT_EQ(result.optimize.reason, "cancelled");
  EXPECT_FALSE(result.optimize.ok());
  EXPECT_TRUE(result.optimize.trajectory.empty());
  EXPECT_FALSE(result.optimize.validation.valid);
  EXPECT_LT(result.optimize.stats.lbfgs_iterations, 64)
      << "the solve must stop long before it completes";

  // 取消响应不含任何几何（统计信息里也不出现轨迹长度/点数）。
  EXPECT_FALSE(result.validation_ran);
  EXPECT_FALSE(result.statistics.contains("trajectory_length_m"));
  EXPECT_FALSE(result.statistics.contains("trajectory_points"));
  EXPECT_FALSE(result.statistics.contains("validation"));
}

}  // namespace
}  // namespace planner::service
