// planner_plan_service 的 HTTP 集成测试：submit -> status -> result 成功链路、
// stale_map 409、无效输入 400、未知作业 404、取消、失败原因和单调的 job_seq。
// 每个测试进程都会启动真实的 service 二进制（service_test_support.hpp）。

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "service_test_support.hpp"

namespace planner::service::test {
namespace {

nlohmann::json corridor_channel_request(const ServiceFixture& fixture,
                                        const std::string& request_id) {
  nlohmann::json request = open_request(fixture, request_id);
  request["goal"] = pose(20.0, 5.0, 0.0);
  request["obstacles"] = nlohmann::json::array(
      {rectangle_obstacle("wall_lower", 6.0, 0.0, 16.0, 3.5),
       rectangle_obstacle("wall_upper", 6.0, 6.5, 16.0, 10.0)});
  request["algorithm_params"] = nlohmann::json{{"hybrid_backend", "ros_free"}};
  return request;
}

// 慢作业（供取消测试）：目标位于可行驶域内的 lanelet 上（(25,5)），被
// 闭环障碍在精确几何上完全封闭，不存在可行路径。
//
// 环墙刻意取 0.02 m 厚（薄于 0.1 m 栅格单元）：精确碰撞仍不可穿越，但
// 「单元中心在多边形内即占据」的栅格化看不到这些墙，因此 Hybrid A* 的
// 栅格势场不会提前判定目标不可达（薄墙版本实测持续搜索到 5 s 预算），
// 作业足够长时间保持运行；厚墙版本会在 ~0.1 s 内以 no_path 结束，无法
// 稳定测试「运行中取消」。两个远处的"窗口填充"障碍把局部窗口扩大到覆盖
// 成片路网，进一步保证搜索时长。
nlohmann::json blocked_goal_request(const ServiceFixture& fixture,
                                    const std::string& request_id) {
  nlohmann::json request = open_request(fixture, request_id);
  request["goal"] = pose(25.0, 5.0, 0.0);
  request["obstacles"] = nlohmann::json::array(
      {rectangle_obstacle("ring_left", 20.00, 0.0, 20.02, 10.0),
       rectangle_obstacle("ring_right", 29.98, 0.0, 30.00, 10.0),
       rectangle_obstacle("ring_bottom", 20.0, 0.00, 30.0, 0.02),
       rectangle_obstacle("ring_top", 20.0, 9.98, 30.0, 10.00),
       rectangle_obstacle("pad_south", 4.5, -100.0, 5.5, -99.5),
       rectangle_obstacle("pad_north", 4.5, 300.0, 5.5, 300.5)});
  request["algorithm_params"] = nlohmann::json{{"time_budget_s", 5.0}};
  return request;
}

TEST(PlanServiceHttp, HealthReportsMapIdentityAndRunDir) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;
  const HttpResult health = http_get(*fixture.client, "/health");
  ASSERT_TRUE(health.transport_ok) << health.error;
  ASSERT_EQ(health.status, 200);
  EXPECT_EQ(health.body.value("schema_version", 0), 1);
  EXPECT_EQ(health.body.value("type", ""), "health");
  EXPECT_EQ(health.body.value("status", ""), "ok");
  EXPECT_EQ(health.body.value("map_id", ""), "mapping_example");
  EXPECT_EQ(health.body.value("map_version", "").size(), 64u);
  EXPECT_GE(health.body.value("uptime_s", -1.0), 0.0);

  const std::filesystem::path status_path =
      std::filesystem::path(fixture.run_dir) / "service_status.json";
  ASSERT_TRUE(std::filesystem::exists(status_path));
  std::ifstream stream(status_path);
  nlohmann::json status;
  stream >> status;
  EXPECT_EQ(status.value("type", ""), "plan_service_status");
  EXPECT_EQ(status.value("status", ""), "listening");
  EXPECT_EQ(status.value("map_version", ""), fixture.map_version);
}

TEST(PlanServiceHttp, OpenPlanSucceedsEndToEnd) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;
  const HttpResult submitted =
      http_post(*fixture.client, "/plan/submit", open_request(fixture, "req-open"));
  ASSERT_TRUE(submitted.transport_ok) << submitted.error;
  ASSERT_EQ(submitted.status, 200);
  EXPECT_EQ(submitted.body.value("schema_version", 0), 1);
  EXPECT_EQ(submitted.body.value("type", ""), "plan_submit");
  EXPECT_EQ(submitted.body.value("status", ""), "queued");
  EXPECT_EQ(submitted.body.value("request_id", ""), "req-open");
  EXPECT_EQ(submitted.body.value("map_id", ""), fixture.map_id);
  EXPECT_EQ(submitted.body.value("map_version", ""), fixture.map_version);
  const std::string job_id = submitted.body.value("job_id", "");
  ASSERT_FALSE(job_id.empty());
  EXPECT_GT(submitted.body.value("job_seq", 0), 0);

  std::string error;
  const nlohmann::json status =
      wait_for_terminal(*fixture.client, job_id, 120.0, &error);
  ASSERT_FALSE(status.empty()) << error;
  ASSERT_EQ(status.value("status", ""), "succeeded") << status.dump();
  EXPECT_EQ(status.value("stage", ""), "done");
  EXPECT_EQ(status.value("request_id", ""), "req-open");
  EXPECT_EQ(status.value("job_id", ""), job_id);

  const HttpResult result = http_get(*fixture.client, "/plan/result/" + job_id);
  ASSERT_TRUE(result.transport_ok) << result.error;
  ASSERT_EQ(result.status, 200);
  EXPECT_EQ(result.body.value("type", ""), "plan_result");
  EXPECT_EQ(result.body.value("status", ""), "succeeded");
  EXPECT_EQ(result.body.value("request_id", ""), "req-open");
  EXPECT_EQ(result.body.value("map_version", ""), fixture.map_version);
  EXPECT_FALSE(result.body.contains("reason"));

  ASSERT_TRUE(result.body.contains("hybrid"));
  EXPECT_FALSE(result.body["hybrid"]["segments"].empty());
  EXPECT_TRUE(result.body["hybrid"]["statistics"].is_object());
  ASSERT_TRUE(result.body.contains("corridor"));
  EXPECT_FALSE(result.body["corridor"]["polygons"].empty());
  EXPECT_EQ(result.body["corridor"].value("map_version", ""), fixture.map_version);
  ASSERT_TRUE(result.body.contains("trajectory"));
  ASSERT_GT(result.body["trajectory"].size(), 1u);
  ASSERT_TRUE(result.body.contains("validation"));
  EXPECT_TRUE(result.body["validation"].value("valid", false));
  EXPECT_TRUE(result.body.contains("statistics"));
  EXPECT_TRUE(result.body["statistics"].contains("timings_s"));

  const nlohmann::json& last = result.body["trajectory"].back();
  EXPECT_NEAR(last["pose"]["x_m"].get<double>(), 18.0, 0.05);
  EXPECT_NEAR(last["pose"]["y_m"].get<double>(), 5.0, 0.05);
  EXPECT_NEAR(last["pose"]["yaw_rad"].get<double>(), 0.0, 0.02);
  // 路径分段使用已冻结的 common 编码。
  const nlohmann::json& first_point = result.body["hybrid"]["segments"].front()["points"].front();
  EXPECT_TRUE(first_point.contains("pose"));
  EXPECT_TRUE(first_point.contains("s_m"));
  EXPECT_TRUE(first_point.contains("curvature_inv_m"));
  EXPECT_TRUE(first_point.contains("direction"));
}

TEST(PlanServiceHttp, ObstacleChannelPlanSucceedsWithSelectedBackend) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;
  const HttpResult submitted = http_post(
      *fixture.client, "/plan/submit", corridor_channel_request(fixture, "req-channel"));
  ASSERT_TRUE(submitted.transport_ok) << submitted.error;
  ASSERT_EQ(submitted.status, 200);
  const std::string job_id = submitted.body.value("job_id", "");
  ASSERT_FALSE(job_id.empty());

  std::string error;
  const nlohmann::json status =
      wait_for_terminal(*fixture.client, job_id, 120.0, &error);
  ASSERT_FALSE(status.empty()) << error;
  ASSERT_EQ(status.value("status", ""), "succeeded") << status.dump();

  const HttpResult result = http_get(*fixture.client, "/plan/result/" + job_id);
  ASSERT_TRUE(result.transport_ok) << result.error;
  ASSERT_EQ(result.status, 200);
  EXPECT_EQ(result.body["hybrid"].value("backend", ""), "ros_free");
  EXPECT_FALSE(result.body["corridor"]["polygons"].empty());
  EXPECT_TRUE(result.body["validation"].value("valid", false));
  // 通道位于 y=3.5 与 y=6.5 之间；轨迹必须保持在通道内。
  for (const nlohmann::json& point : result.body["trajectory"]) {
    const double y_m = point["pose"]["y_m"].get<double>();
    EXPECT_GT(y_m, 3.0) << point.dump();
    EXPECT_LT(y_m, 7.0) << point.dump();
  }
}

TEST(PlanServiceHttp, StaleMapIsRejectedWith409) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;

  nlohmann::json request = open_request(fixture, "req-stale-version");
  request["map_version"] = std::string(64, '0');
  const HttpResult stale_version = http_post(*fixture.client, "/plan/submit", request);
  ASSERT_TRUE(stale_version.transport_ok) << stale_version.error;
  EXPECT_EQ(stale_version.status, 409);
  EXPECT_EQ(stale_version.body.value("type", ""), "error");
  EXPECT_EQ(stale_version.body.value("status", ""), "stale_map");

  request = open_request(fixture, "req-stale-id");
  request["map_id"] = "some_other_map";
  const HttpResult stale_id = http_post(*fixture.client, "/plan/submit", request);
  ASSERT_TRUE(stale_id.transport_ok) << stale_id.error;
  EXPECT_EQ(stale_id.status, 409);
  EXPECT_EQ(stale_id.body.value("status", ""), "stale_map");
}

TEST(PlanServiceHttp, MalformedSubmitIsRejectedWith400) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;

  nlohmann::json request = open_request(fixture, "req-bad");
  request.erase("request_id");
  HttpResult response = http_post(*fixture.client, "/plan/submit", request);
  ASSERT_TRUE(response.transport_ok) << response.error;
  EXPECT_EQ(response.status, 400);
  EXPECT_EQ(response.body.value("type", ""), "error");
  EXPECT_EQ(response.body.value("status", ""), "invalid_input");

  request = open_request(fixture, "req-bad");
  request["algorithm_params"] = nlohmann::json{{"corridor", nlohmann::json{{"bogus", 1}}}};
  response = http_post(*fixture.client, "/plan/submit", request);
  ASSERT_TRUE(response.transport_ok) << response.error;
  EXPECT_EQ(response.status, 400);
  EXPECT_EQ(response.body.value("status", ""), "invalid_input");

  // 无效的 JSON 请求体。
  httplib::Result raw =
      fixture.client->Post("/plan/submit", "{not json", "application/json");
  ASSERT_TRUE(raw);
  EXPECT_EQ(raw->status, 400);
}

TEST(PlanServiceHttp, UnknownJobAndEndpointReturn404) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;

  HttpResult response = http_get(*fixture.client, "/plan/status/job-999999");
  ASSERT_TRUE(response.transport_ok) << response.error;
  EXPECT_EQ(response.status, 404);
  EXPECT_EQ(response.body.value("status", ""), "not_found");

  response = http_get(*fixture.client, "/plan/result/job-999999");
  ASSERT_TRUE(response.transport_ok) << response.error;
  EXPECT_EQ(response.status, 404);
  EXPECT_EQ(response.body.value("status", ""), "not_found");

  response = http_get(*fixture.client, "/no/such/endpoint");
  ASSERT_TRUE(response.transport_ok) << response.error;
  EXPECT_EQ(response.status, 404);
  EXPECT_EQ(response.body.value("type", ""), "error");
}

TEST(PlanServiceHttp, CancellationStatesAndQueuedCancel) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;

  // 先提交一个慢（目标被闭环障碍包围）作业；工作线程忙于它时，第二个
  // 作业保持排队。
  const HttpResult slow_submitted = http_post(
      *fixture.client, "/plan/submit", blocked_goal_request(fixture, "req-slow"));
  ASSERT_TRUE(slow_submitted.transport_ok) << slow_submitted.error;
  ASSERT_EQ(slow_submitted.status, 200);
  const std::string slow_job = slow_submitted.body.value("job_id", "");
  ASSERT_FALSE(slow_job.empty());

  // 排队中的作业在运行前即可获取结果，但不含几何数据。
  const HttpResult queued_result = http_get(*fixture.client, "/plan/result/" + slow_job);
  ASSERT_TRUE(queued_result.transport_ok) << queued_result.error;
  ASSERT_EQ(queued_result.status, 200);
  const std::string early_status = queued_result.body.value("status", "");
  EXPECT_TRUE(early_status == "queued" || early_status == "running")
      << early_status;
  EXPECT_FALSE(queued_result.body.contains("trajectory"));
  EXPECT_FALSE(queued_result.body.contains("hybrid"));

  const HttpResult queued_submitted =
      http_post(*fixture.client, "/plan/submit", open_request(fixture, "req-queued"));
  ASSERT_TRUE(queued_submitted.transport_ok) << queued_submitted.error;
  ASSERT_EQ(queued_submitted.status, 200);
  const std::string queued_job = queued_submitted.body.value("job_id", "");
  ASSERT_FALSE(queued_job.empty());

  const HttpResult cancelled = http_post(
      *fixture.client, "/plan/cancel", nlohmann::json{{"job_id", queued_job}});
  ASSERT_TRUE(cancelled.transport_ok) << cancelled.error;
  EXPECT_EQ(cancelled.status, 200);
  EXPECT_EQ(cancelled.body.value("type", ""), "plan_cancel");
  EXPECT_EQ(cancelled.body.value("status", ""), "cancelled");

  std::string error;
  const nlohmann::json queued_terminal =
      wait_for_terminal(*fixture.client, queued_job, 30.0, &error);
  ASSERT_FALSE(queued_terminal.empty()) << error;
  EXPECT_EQ(queued_terminal.value("status", ""), "cancelled");
  EXPECT_EQ(queued_terminal.value("stage", ""), "done");

  // 取消慢作业（运行中或仍在排队）；它必须以 cancelled 结束。
  const HttpResult slow_cancelled = http_post(
      *fixture.client, "/plan/cancel", nlohmann::json{{"job_id", slow_job}});
  ASSERT_TRUE(slow_cancelled.transport_ok) << slow_cancelled.error;
  EXPECT_EQ(slow_cancelled.body.value("status", ""), "cancelled");
  const nlohmann::json slow_terminal =
      wait_for_terminal(*fixture.client, slow_job, 60.0, &error);
  ASSERT_FALSE(slow_terminal.empty()) << error;
  EXPECT_EQ(slow_terminal.value("status", ""), "cancelled") << slow_terminal.dump();
  EXPECT_EQ(slow_terminal.value("reason", ""), "cancelled");

  // 对终态作业再次取消会报告 already_finished。
  const HttpResult again = http_post(
      *fixture.client, "/plan/cancel", nlohmann::json{{"job_id", slow_job}});
  ASSERT_TRUE(again.transport_ok) << again.error;
  EXPECT_EQ(again.status, 200);
  EXPECT_EQ(again.body.value("status", ""), "already_finished");

  // 未知的作业 id 和格式错误的请求体。
  const HttpResult missing = http_post(
      *fixture.client, "/plan/cancel", nlohmann::json{{"job_id", "job-does-not-exist"}});
  ASSERT_TRUE(missing.transport_ok) << missing.error;
  EXPECT_EQ(missing.status, 200);
  EXPECT_EQ(missing.body.value("status", ""), "not_found");

  const HttpResult malformed =
      http_post(*fixture.client, "/plan/cancel", nlohmann::json{{"job_id", 7}});
  ASSERT_TRUE(malformed.transport_ok) << malformed.error;
  EXPECT_EQ(malformed.status, 400);
  EXPECT_EQ(malformed.body.value("status", ""), "invalid_input");
}

TEST(PlanServiceHttp, FailedJobReportsReasonWithoutGeometry) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;

  nlohmann::json request = open_request(fixture, "req-colliding-start");
  request["obstacles"] = nlohmann::json::array(
      {rectangle_obstacle("block", 1.0, 4.0, 3.0, 6.0)});  // 包含起点 (2, 5)
  const HttpResult submitted = http_post(*fixture.client, "/plan/submit", request);
  ASSERT_TRUE(submitted.transport_ok) << submitted.error;
  ASSERT_EQ(submitted.status, 200);
  const std::string job_id = submitted.body.value("job_id", "");
  ASSERT_FALSE(job_id.empty());

  std::string error;
  const nlohmann::json status =
      wait_for_terminal(*fixture.client, job_id, 60.0, &error);
  ASSERT_FALSE(status.empty()) << error;
  ASSERT_EQ(status.value("status", ""), "failed") << status.dump();
  EXPECT_EQ(status.value("reason", ""), "invalid_input");
  EXPECT_EQ(status.value("stage", ""), "done");

  const HttpResult result = http_get(*fixture.client, "/plan/result/" + job_id);
  ASSERT_TRUE(result.transport_ok) << result.error;
  ASSERT_EQ(result.status, 200);
  EXPECT_EQ(result.body.value("status", ""), "failed");
  EXPECT_EQ(result.body.value("reason", ""), "invalid_input");
  EXPECT_TRUE(result.body.contains("detail"));
  EXPECT_FALSE(result.body.contains("trajectory"));
  EXPECT_FALSE(result.body.contains("hybrid"));
  EXPECT_FALSE(result.body.contains("corridor"));
  EXPECT_FALSE(result.body.contains("validation"));
}

// 可行驶域约束：起点/终点必须落在 lanelet 可行驶域内。两点都在地图边界内
// 但距最近 lanelet 数米（见 Src/service/tests/test_scene.cpp 的同一夹具
// 校验），场景构建在 map 阶段以稳定原因码失败，且结果不含几何。
TEST(PlanServiceHttp, OffRoadStartAndGoalAreRejectedByDriveableDomain) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;
  std::string error;

  nlohmann::json request = open_request(fixture, "req-offroad-start");
  request["start"] = pose(5.0, -5.0, 0.0);  // 道路外约 5.2 m
  HttpResult submitted = http_post(*fixture.client, "/plan/submit", request);
  ASSERT_TRUE(submitted.transport_ok) << submitted.error;
  ASSERT_EQ(submitted.status, 200);
  const std::string start_job = submitted.body.value("job_id", "");
  ASSERT_FALSE(start_job.empty());
  const nlohmann::json start_status =
      wait_for_terminal(*fixture.client, start_job, 60.0, &error);
  ASSERT_FALSE(start_status.empty()) << error;
  EXPECT_EQ(start_status.value("status", ""), "failed") << start_status.dump();
  EXPECT_EQ(start_status.value("reason", ""), "start_outside_driveable");
  EXPECT_EQ(start_status.value("stage", ""), "done");

  request = open_request(fixture, "req-offroad-goal");
  request["goal"] = pose(8.0, -4.0, 0.0);  // 道路外约 3.7 m
  submitted = http_post(*fixture.client, "/plan/submit", request);
  ASSERT_TRUE(submitted.transport_ok) << submitted.error;
  ASSERT_EQ(submitted.status, 200);
  const std::string goal_job = submitted.body.value("job_id", "");
  ASSERT_FALSE(goal_job.empty());
  const nlohmann::json goal_status =
      wait_for_terminal(*fixture.client, goal_job, 60.0, &error);
  ASSERT_FALSE(goal_status.empty()) << error;
  EXPECT_EQ(goal_status.value("status", ""), "failed") << goal_status.dump();
  EXPECT_EQ(goal_status.value("reason", ""), "goal_outside_driveable");

  const HttpResult result = http_get(*fixture.client, "/plan/result/" + goal_job);
  ASSERT_TRUE(result.transport_ok) << result.error;
  EXPECT_EQ(result.body.value("status", ""), "failed");
  EXPECT_FALSE(result.body.contains("hybrid"));
  EXPECT_FALSE(result.body.contains("trajectory"));
}

TEST(PlanServiceHttp, JobSeqIsMonotonicAndJobIdsAreUnique) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;

  std::vector<std::uint64_t> sequences;
  std::vector<std::string> job_ids;
  for (int i = 0; i < 3; ++i) {
    nlohmann::json request = open_request(fixture, "req-seq-" + std::to_string(i));
    request["obstacles"] = nlohmann::json::array(
        {rectangle_obstacle("block", 1.0, 4.0, 3.0, 6.0)});  // 快速失败
    const HttpResult submitted = http_post(*fixture.client, "/plan/submit", request);
    ASSERT_TRUE(submitted.transport_ok) << submitted.error;
    ASSERT_EQ(submitted.status, 200);
    sequences.push_back(submitted.body.value("job_seq", 0));
    job_ids.push_back(submitted.body.value("job_id", ""));
  }
  ASSERT_EQ(sequences.size(), 3u);
  EXPECT_LT(sequences[0], sequences[1]);
  EXPECT_LT(sequences[1], sequences[2]);
  EXPECT_NE(job_ids[0], job_ids[1]);
  EXPECT_NE(job_ids[1], job_ids[2]);

  std::string error;
  for (const std::string& job_id : job_ids) {
    const nlohmann::json status =
        wait_for_terminal(*fixture.client, job_id, 60.0, &error);
    ASSERT_FALSE(status.empty()) << error;
    EXPECT_EQ(status.value("status", ""), "failed");
  }
}

}  // namespace
}  // namespace planner::service::test
