// planner_plan_service（/plan/events）的 WebSocket 推送测试：订阅、提交作业、
// 接收 job_update / job_result 事件并验证广播语义。每个测试进程都会启动
// 真实的 service 二进制。

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "service_test_support.hpp"

namespace planner::service::test {
namespace {

std::unique_ptr<httplib::ws::WebSocketClient> connect_events(
    const ServiceFixture& fixture, std::string* error) {
  auto client = std::make_unique<httplib::ws::WebSocketClient>(
      "ws://127.0.0.1:" + fixture.process->port + "/plan/events");
  if (!client->is_valid()) {
    *error = "invalid websocket URL";
    return nullptr;
  }
  const auto connected = client->connect();
  if (!connected) {
    *error = std::string("websocket connect failed: ") +
             httplib::to_string(connected.error()) + " status=" +
             std::to_string(connected.status());
    return nullptr;
  }
  client->set_read_timeout(120, 0);
  return client;
}

// 读取事件，直到收到 `job_id` 的一个 job_result。返回该作业收到的所有事件
// （更新加上结果）。
std::vector<nlohmann::json> read_events_for_job(
    httplib::ws::WebSocketClient& client, const std::string& job_id,
    std::string* error) {
  std::vector<nlohmann::json> events;
  while (true) {
    std::string message;
    const httplib::ws::ReadResult result = client.read(message);
    if (result == httplib::ws::Fail) {
      *error = "websocket closed before job_result; events=" +
               std::to_string(events.size());
      return events;
    }
    if (result == httplib::ws::Timeout) {
      *error = "timeout waiting for job_result; events=" +
               std::to_string(events.size());
      return events;
    }
    nlohmann::json event;
    try {
      event = nlohmann::json::parse(message);
    } catch (const std::exception& parse_error) {
      *error = std::string("invalid event JSON: ") + parse_error.what();
      return events;
    }
    if (event.value("job_id", "") != job_id) {
      continue;
    }
    events.push_back(event);
    if (event.value("type", "") == "job_result") {
      return events;
    }
  }
}

TEST(PlanServiceEvents, JobUpdatesAndResultArePushedInOrder) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;

  std::string error;
  std::unique_ptr<httplib::ws::WebSocketClient> events =
      connect_events(fixture, &error);
  ASSERT_TRUE(events != nullptr) << error;

  const HttpResult submitted =
      http_post(*fixture.client, "/plan/submit", open_request(fixture, "req-events"));
  ASSERT_TRUE(submitted.transport_ok) << submitted.error;
  ASSERT_EQ(submitted.status, 200);
  const std::string job_id = submitted.body.value("job_id", "");
  const std::uint64_t job_seq = submitted.body.value("job_seq", 0);
  ASSERT_FALSE(job_id.empty());

  const std::vector<nlohmann::json> received =
      read_events_for_job(*events, job_id, &error);
  ASSERT_FALSE(received.empty()) << error;
  events->close();

  // 第一个事件：queued 更新；最后一个事件：结果通知。
  EXPECT_EQ(received.front().value("type", ""), "job_update");
  EXPECT_EQ(received.front().value("status", ""), "queued");
  EXPECT_EQ(received.back().value("type", ""), "job_result");
  EXPECT_EQ(received.back().value("job_id", ""), job_id);
  EXPECT_EQ(received.back().value("request_id", ""), "req-events");
  EXPECT_EQ(received.back().value("job_seq", std::uint64_t{0}), job_seq);
  EXPECT_EQ(received.back().value("map_version", ""), fixture.map_version);

  // 每个更新都携带契约字段；各阶段覆盖整条流水线，最后一个更新报告终态。
  std::set<std::string> stages;
  std::set<std::string> statuses;
  for (const nlohmann::json& event : received) {
    if (event.value("type", "") != "job_update") {
      continue;
    }
    EXPECT_EQ(event.value("job_id", ""), job_id);
    EXPECT_EQ(event.value("request_id", ""), "req-events");
    EXPECT_EQ(event.value("job_seq", std::uint64_t{0}), job_seq);
    EXPECT_EQ(event.value("map_version", ""), fixture.map_version);
    EXPECT_EQ(event.value("schema_version", 0), 1);
    stages.insert(event.value("stage", ""));
    statuses.insert(event.value("status", ""));
  }
  EXPECT_TRUE(statuses.count("queued") == 1);
  EXPECT_TRUE(statuses.count("running") == 1);
  EXPECT_TRUE(statuses.count("succeeded") == 1);
  EXPECT_TRUE(stages.count("hybrid") == 1);
  EXPECT_TRUE(stages.count("corridor") == 1);
  EXPECT_TRUE(stages.count("optimize") == 1);
  EXPECT_TRUE(stages.count("validate") == 1);
  EXPECT_TRUE(stages.count("done") == 1);

  // 结果通过 HTTP 拉取，并与推送的通知一致。
  const HttpResult result = http_get(*fixture.client, "/plan/result/" + job_id);
  ASSERT_TRUE(result.transport_ok) << result.error;
  EXPECT_EQ(result.body.value("status", ""), "succeeded");
}

TEST(PlanServiceEvents, EventsAreBroadcastAndReportFailures) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;

  std::string error;
  std::unique_ptr<httplib::ws::WebSocketClient> first = connect_events(fixture, &error);
  ASSERT_TRUE(first != nullptr) << error;
  std::unique_ptr<httplib::ws::WebSocketClient> second = connect_events(fixture, &error);
  ASSERT_TRUE(second != nullptr) << error;

  nlohmann::json request = open_request(fixture, "req-events-fail");
  request["obstacles"] = nlohmann::json::array(
      {rectangle_obstacle("block", 1.0, 4.0, 3.0, 6.0)});  // 包含起点
  const HttpResult submitted = http_post(*fixture.client, "/plan/submit", request);
  ASSERT_TRUE(submitted.transport_ok) << submitted.error;
  ASSERT_EQ(submitted.status, 200);
  const std::string job_id = submitted.body.value("job_id", "");
  ASSERT_FALSE(job_id.empty());

  const std::vector<nlohmann::json> first_events =
      read_events_for_job(*first, job_id, &error);
  ASSERT_FALSE(first_events.empty()) << error;
  const std::vector<nlohmann::json> second_events =
      read_events_for_job(*second, job_id, &error);
  ASSERT_FALSE(second_events.empty()) << error;
  first->close();
  second->close();

  // 两个订阅者都收到终态结果通知。
  EXPECT_EQ(first_events.back().value("type", ""), "job_result");
  EXPECT_EQ(second_events.back().value("type", ""), "job_result");
  EXPECT_EQ(second_events.back().value("job_id", ""), job_id);

  // 结果之前的最后一个更新报告 failed 终态。
  bool saw_failed = false;
  for (const nlohmann::json& event : first_events) {
    if (event.value("type", "") == "job_update" &&
        event.value("status", "") == "failed") {
      saw_failed = true;
      EXPECT_EQ(event.value("stage", ""), "done");
    }
  }
  EXPECT_TRUE(saw_failed) << "no failed job_update observed";
}

}  // namespace
}  // namespace planner::service::test
