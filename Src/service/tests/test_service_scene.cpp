// X-M1 规划服务集成测试：场景标识字段（status/result/WS）、
// GET /plan/scene/{scene_id} 快照、LRU 淘汰与 404，以及地图服务预览与
// 规划输入快照的逐格一致性（协议 §3.3/§3.4/§5；v6 P11/P13 服务侧语义）。
//
// 每个测试进程启动真实的 planner_plan_service；跨服务用例另外启动
// planner_map_service（同一 v3 地图与投影原点）。

#include <gtest/gtest.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "planner/collision/scene.hpp"
#include "planner/common/types.hpp"
#include "planner/scene/content_id.hpp"
#include "planner/service/scene.hpp"
#include "service_test_support.hpp"

#ifndef PLANNER_MAP_SERVICE_PATH
#error "PLANNER_MAP_SERVICE_PATH must be defined by the test CMakeLists"
#endif

namespace planner::service::test {
namespace {

// ---------------------------------------------------------------------------
// 地图服务夹具（跨服务一致性用例）。
// ---------------------------------------------------------------------------

struct MapServiceFixture {
  std::optional<ServiceProcess> process;
  std::unique_ptr<httplib::Client> client;
  std::string error;

  static MapServiceFixture& instance() {
    static MapServiceFixture fixture;
    return fixture;
  }

  MapServiceFixture() {
    std::string start_error;
    process = start_service_binary(
        PLANNER_MAP_SERVICE_PATH,
        {"--map", PLANNER_SERVICE_TEST_MAP_PATH, "--origin-lat",
         PLANNER_SERVICE_TEST_ORIGIN_LAT, "--origin-lon", PLANNER_SERVICE_TEST_ORIGIN_LON,
         "--port", "0", "--quiet"},
        &start_error);
    if (!process.has_value()) {
      error = start_error;
      return;
    }
    client = std::make_unique<httplib::Client>("127.0.0.1", std::stoi(process->port));
    client->set_connection_timeout(5, 0);
    client->set_read_timeout(120, 0);
    const HttpResult health = http_get(*client, "/health");
    if (!health.transport_ok || health.status != 200) {
      error = "map service health request failed";
    }
  }

  ~MapServiceFixture() {
    if (process.has_value()) {
      process->stop();
    }
  }
};

MapServiceFixture& map_service() { return MapServiceFixture::instance(); }

// 标准 base64 解码（协议 §2.3 长度自检用）。
std::optional<std::vector<unsigned char>> base64_decode(const std::string& text) {
  static const std::string kAlphabet =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::vector<int> values(256, -1);
  for (std::size_t i = 0; i < kAlphabet.size(); ++i) {
    values[static_cast<unsigned char>(kAlphabet[i])] = static_cast<int>(i);
  }
  std::vector<unsigned char> out;
  int buffer = 0;
  int bits = 0;
  for (const char raw : text) {
    const unsigned char ch = static_cast<unsigned char>(raw);
    if (ch == '=') {
      break;
    }
    const int value = values[ch];
    if (value < 0) {
      return std::nullopt;
    }
    buffer = (buffer << 6) | value;
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      out.push_back(static_cast<unsigned char>((buffer >> bits) & 0xFF));
    }
  }
  return out;
}

// 快速失败请求：障碍覆盖起点 → 场景构建成功、Hybrid 阶段快速 invalid_input。
nlohmann::json fast_fail_request(const ServiceFixture& fixture,
                                 const std::string& request_id, double offset) {
  nlohmann::json request = open_request(fixture, request_id);
  request["obstacles"] = nlohmann::json::array(
      {rectangle_obstacle("block", 1.0 + offset, 4.0, 3.0 + offset, 6.0)});
  return request;
}

const char* kSceneIdField = "scene_id";
const char* kSceneVersionField = "scene_version";

// 提交并等待终态；返回 job_id 与终态状态体。
struct SubmittedJob {
  std::string job_id;
  nlohmann::json terminal;
  std::string error;
};

SubmittedJob submit_and_wait(ServiceFixture& fixture, const nlohmann::json& request) {
  SubmittedJob out;
  const HttpResult submitted = http_post(*fixture.client, "/plan/submit", request);
  if (!submitted.transport_ok || submitted.status != 200) {
    out.error = "submit failed: " + submitted.error +
                " status=" + std::to_string(submitted.status);
    return out;
  }
  out.job_id = submitted.body.value("job_id", "");
  if (out.job_id.empty()) {
    out.error = "submit response has no job_id";
    return out;
  }
  out.terminal = wait_for_terminal(*fixture.client, out.job_id, 120.0, &out.error);
  return out;
}

// ---------------------------------------------------------------------------
// 测试。
// ---------------------------------------------------------------------------

TEST(PlanServiceScene, HealthReportsGridProtocolAndSceneVersions) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;
  const HttpResult health = http_get(*fixture.client, "/health");
  ASSERT_TRUE(health.transport_ok) << health.error;
  ASSERT_EQ(health.status, 200);
  EXPECT_EQ(health.body.value("grid_protocol_version", ""), "1.0");
  EXPECT_EQ(health.body.value("scene_rules_version", ""), "scene_v1");
}

TEST(PlanServiceScene, SuccessfulPlanReportsSceneIdAndServesSnapshot) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;

  const SubmittedJob job =
      submit_and_wait(fixture, open_request(fixture, "req-scene-open"));
  ASSERT_TRUE(job.error.empty()) << job.error;
  ASSERT_EQ(job.terminal.value("status", ""), "succeeded") << job.terminal.dump();
  const std::string scene_id = job.terminal.value(kSceneIdField, "");
  ASSERT_EQ(scene_id.size(), 64u);
  EXPECT_EQ(job.terminal.value(kSceneVersionField, ""), "scene_v1");

  const HttpResult result = http_get(*fixture.client, "/plan/result/" + job.job_id);
  ASSERT_TRUE(result.transport_ok) << result.error;
  EXPECT_EQ(result.body.value(kSceneIdField, ""), scene_id);
  EXPECT_EQ(result.body.value(kSceneVersionField, ""), "scene_v1");
  EXPECT_TRUE(result.body.contains("statistics"));  // 成功结果保留既有 statistics。

  const HttpResult snapshot = http_get(*fixture.client, "/plan/scene/" + scene_id);
  ASSERT_TRUE(snapshot.transport_ok) << snapshot.error;
  ASSERT_EQ(snapshot.status, 200) << snapshot.body.dump();
  EXPECT_EQ(snapshot.body.value("type", ""), "grid_snapshot");
  EXPECT_EQ(snapshot.body.value("schema_version", 0), 1);
  EXPECT_EQ(snapshot.body.value("source", ""), "planning_scene");
  EXPECT_EQ(snapshot.body.value(kSceneIdField, ""), scene_id);
  EXPECT_EQ(snapshot.body.value(kSceneVersionField, ""), "scene_v1");
  EXPECT_EQ(snapshot.body.value("map_id", ""), fixture.map_id);
  EXPECT_EQ(snapshot.body.value("map_version", ""), fixture.map_version);
  EXPECT_EQ(snapshot.body.value("encoding", ""), "base64_u8");
  EXPECT_EQ(snapshot.body.value("budget_cells", 0), 16777216);
  EXPECT_FALSE(snapshot.body.contains("request_seq"));  // 规划快照不携带请求序号。
  const auto decoded = base64_decode(snapshot.body.value("cells", ""));
  ASSERT_TRUE(decoded.has_value());
  EXPECT_EQ(decoded->size(), static_cast<std::size_t>(snapshot.body.value("width", 0)) *
                                 static_cast<std::size_t>(snapshot.body.value("height", 0)));
  // 从快照字段独立复算 scene_id（无障碍；§5.2 规范输入）。
  EXPECT_EQ(snapshot.body.value(kSceneIdField, ""),
            scene::scene_content_id(
                fixture.map_id, fixture.map_version,
                planner::map::Bounds2d{
                    snapshot.body["window"]["min_x_m"].get<double>(),
                    snapshot.body["window"]["min_y_m"].get<double>(),
                    snapshot.body["window"]["max_x_m"].get<double>(),
                    snapshot.body["window"]["max_y_m"].get<double>()},
                snapshot.body["effective_resolution_m"].get<double>(),
                snapshot.body.value("width", 0), snapshot.body.value("height", 0),
                planner::common::Pose2d{snapshot.body["origin"]["x_m"].get<double>(),
                                        snapshot.body["origin"]["y_m"].get<double>(),
                                        snapshot.body["origin"]["yaw_rad"].get<double>()},
                {}));

  // 重复读取返回相同字节（快照原文存储，逐字节一致）。
  const httplib::Result raw_first =
      fixture.client->Get(("/plan/scene/" + scene_id).c_str());
  const httplib::Result raw_second =
      fixture.client->Get(("/plan/scene/" + scene_id).c_str());
  ASSERT_TRUE(raw_first);
  ASSERT_TRUE(raw_second);
  ASSERT_EQ(raw_first->status, 200);
  EXPECT_EQ(raw_first->body, raw_second->body);
}

TEST(PlanServiceScene, FailedJobStillReportsSceneIdAndSnapshot) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;

  const SubmittedJob job = submit_and_wait(fixture, fast_fail_request(fixture, "req-scene-fail", 0.0));
  ASSERT_TRUE(job.error.empty()) << job.error;
  ASSERT_EQ(job.terminal.value("status", ""), "failed") << job.terminal.dump();
  const std::string scene_id = job.terminal.value(kSceneIdField, "");
  ASSERT_EQ(scene_id.size(), 64u);
  EXPECT_EQ(job.terminal.value(kSceneVersionField, ""), "scene_v1");

  const HttpResult result = http_get(*fixture.client, "/plan/result/" + job.job_id);
  ASSERT_TRUE(result.transport_ok) << result.error;
  EXPECT_EQ(result.body.value("status", ""), "failed");
  EXPECT_EQ(result.body.value(kSceneIdField, ""), scene_id);

  const HttpResult snapshot = http_get(*fixture.client, "/plan/scene/" + scene_id);
  ASSERT_TRUE(snapshot.transport_ok) << snapshot.error;
  EXPECT_EQ(snapshot.status, 200);
  EXPECT_EQ(snapshot.body.value(kSceneIdField, ""), scene_id);
  EXPECT_EQ(snapshot.body.value("source", ""), "planning_scene");
}

TEST(PlanServiceScene, UnknownSceneReturns404) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;
  HttpResult response =
      http_get(*fixture.client, "/plan/scene/" + std::string(64, '0'));
  ASSERT_TRUE(response.transport_ok) << response.error;
  EXPECT_EQ(response.status, 404);
  EXPECT_EQ(response.body.value("type", ""), "error");
  EXPECT_EQ(response.body.value("status", ""), "not_found");
  EXPECT_EQ(response.body.value("reason", ""), "unknown_scene");

  response = http_get(*fixture.client, "/plan/scene/not-a-scene-id");
  ASSERT_TRUE(response.transport_ok) << response.error;
  EXPECT_EQ(response.status, 404);
  EXPECT_EQ(response.body.value("reason", ""), "unknown_scene");
}

TEST(PlanServiceScene, IdenticalScenesDeduplicateInSnapshotStore) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;

  const nlohmann::json request = fast_fail_request(fixture, "req-dedup", 0.0);
  const SubmittedJob first = submit_and_wait(fixture, request);
  ASSERT_TRUE(first.error.empty()) << first.error;
  const SubmittedJob second = submit_and_wait(fixture, request);
  ASSERT_TRUE(second.error.empty()) << second.error;
  const std::string first_scene = first.terminal.value(kSceneIdField, "");
  ASSERT_FALSE(first_scene.empty());
  EXPECT_EQ(second.terminal.value(kSceneIdField, ""), first_scene);
  // 去重后两个任务都可读取同一份快照。
  const HttpResult snapshot = http_get(*fixture.client, "/plan/scene/" + first_scene);
  ASSERT_TRUE(snapshot.transport_ok) << snapshot.error;
  EXPECT_EQ(snapshot.status, 200);
}

TEST(PlanServiceScene, SnapshotStoreEvictsOldestAfterSixteenScenes) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;

  // 17 个不同场景（障碍几何不同 → scene_id 不同）；容量 16 → 最早一个被淘汰。
  std::vector<std::string> scene_ids;
  for (int i = 0; i < 17; ++i) {
    const SubmittedJob job = submit_and_wait(
        fixture, fast_fail_request(fixture, "req-lru-" + std::to_string(i),
                                   0.01 * static_cast<double>(i)));
    ASSERT_TRUE(job.error.empty()) << job.error;
    const std::string scene_id = job.terminal.value(kSceneIdField, "");
    ASSERT_EQ(scene_id.size(), 64u) << job.terminal.dump();
    scene_ids.push_back(scene_id);
  }
  std::set<std::string> unique(scene_ids.begin(), scene_ids.end());
  EXPECT_EQ(unique.size(), 17u);

  const HttpResult evicted = http_get(*fixture.client, "/plan/scene/" + scene_ids.front());
  ASSERT_TRUE(evicted.transport_ok) << evicted.error;
  EXPECT_EQ(evicted.status, 404);
  EXPECT_EQ(evicted.body.value("reason", ""), "unknown_scene");
  for (std::size_t i = 1; i < scene_ids.size(); ++i) {
    const HttpResult snapshot = http_get(*fixture.client, "/plan/scene/" + scene_ids[i]);
    ASSERT_TRUE(snapshot.transport_ok) << snapshot.error;
    EXPECT_EQ(snapshot.status, 200) << "scene index " << i;
  }
}

TEST(PlanServiceScene, PreviewMatchesPlanSnapshotCellByCell) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;
  MapServiceFixture& maps = map_service();
  ASSERT_TRUE(maps.error.empty()) << maps.error;

  const nlohmann::json request = fast_fail_request(fixture, "req-p11", 0.0);
  const SubmittedJob job = submit_and_wait(fixture, request);
  ASSERT_TRUE(job.error.empty()) << job.error;
  const std::string scene_id = job.terminal.value(kSceneIdField, "");
  ASSERT_EQ(scene_id.size(), 64u);

  const HttpResult snapshot = http_get(*fixture.client, "/plan/scene/" + scene_id);
  ASSERT_TRUE(snapshot.transport_ok) << snapshot.error;
  ASSERT_EQ(snapshot.status, 200);

  // 用规划快照的有效窗口/分辨率/障碍向地图服务请求预览。
  const HttpResult preview =
      http_post(*maps.client, "/map/scene/preview",
                nlohmann::json{{"window", snapshot.body["window"]},
                               {"resolution_m",
                                snapshot.body["effective_resolution_m"].get<double>()},
                               {"obstacles", request["obstacles"]},
                               {"request_seq", 42}});
  ASSERT_TRUE(preview.transport_ok) << preview.error;
  ASSERT_EQ(preview.status, 200) << preview.body.dump();

  // P11：cells 逐格相等、scene_id/scene_version/width/height/resolution/origin 相等。
  EXPECT_EQ(preview.body.value(kSceneIdField, ""), scene_id);
  EXPECT_EQ(preview.body.value(kSceneVersionField, ""),
            snapshot.body.value(kSceneVersionField, ""));
  EXPECT_EQ(preview.body.value("width", 0), snapshot.body.value("width", 0));
  EXPECT_EQ(preview.body.value("height", 0), snapshot.body.value("height", 0));
  EXPECT_DOUBLE_EQ(preview.body["resolution_m"].get<double>(),
                   snapshot.body["resolution_m"].get<double>());
  EXPECT_DOUBLE_EQ(preview.body["origin"]["x_m"].get<double>(),
                   snapshot.body["origin"]["x_m"].get<double>());
  EXPECT_DOUBLE_EQ(preview.body["origin"]["y_m"].get<double>(),
                   snapshot.body["origin"]["y_m"].get<double>());
  EXPECT_DOUBLE_EQ(preview.body["origin"]["yaw_rad"].get<double>(),
                   snapshot.body["origin"]["yaw_rad"].get<double>());
  EXPECT_EQ(preview.body.value("request_seq", -1), 42);  // 预览回显请求序号。

  const auto plan_cells = base64_decode(snapshot.body.value("cells", ""));
  const auto preview_cells = base64_decode(preview.body.value("cells", ""));
  ASSERT_TRUE(plan_cells.has_value());
  ASSERT_TRUE(preview_cells.has_value());
  EXPECT_EQ(*plan_cells, *preview_cells);
}

TEST(PlanServiceScene, WebSocketEventsCarrySceneIdAfterMapStage) {
  ServiceFixture& fixture = require_service();
  ASSERT_TRUE(fixture.ok()) << fixture.error;

  auto ws = std::make_unique<httplib::ws::WebSocketClient>(
      "ws://127.0.0.1:" + fixture.process->port + "/plan/events");
  ASSERT_TRUE(ws->is_valid());
  const auto connected = ws->connect();
  ASSERT_TRUE(connected) << httplib::to_string(connected.error());
  ws->set_read_timeout(120, 0);

  const SubmittedJob submitted =
      submit_and_wait(fixture, open_request(fixture, "req-ws-scene"));
  ASSERT_TRUE(submitted.error.empty()) << submitted.error;
  const std::string scene_id = submitted.terminal.value(kSceneIdField, "");
  ASSERT_EQ(scene_id.size(), 64u);

  bool update_with_scene = false;
  bool result_with_scene = false;
  while (true) {
    std::string message;
    const httplib::ws::ReadResult read = ws->read(message);
    if (read == httplib::ws::Fail || read == httplib::ws::Timeout) {
      break;
    }
    nlohmann::json event;
    try {
      event = nlohmann::json::parse(message);
    } catch (const std::exception&) {
      continue;
    }
    if (event.value("job_id", "") != submitted.job_id) {
      continue;
    }
    if (event.value("type", "") == "job_update" && event.contains(kSceneIdField)) {
      if (event.value(kSceneIdField, "") == scene_id) {
        update_with_scene = true;
      }
    }
    if (event.value("type", "") == "job_result") {
      result_with_scene = event.value(kSceneIdField, "") == scene_id;
      break;
    }
  }
  ws->close();
  EXPECT_TRUE(update_with_scene);
  EXPECT_TRUE(result_with_scene);
}

}  // namespace
}  // namespace planner::service::test
