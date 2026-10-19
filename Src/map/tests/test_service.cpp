// planner_map_service 测试：在临时端口（--port 0）上启动真实的服务二进制，
// 解析打印出的端口，然后用 httplib 客户端查询每个端点，
// 并与直接调用库的结果比较。

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstring>
#include <memory>
#include <optional>
#include <poll.h>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "test_support.hpp"

namespace planner::map {
namespace {

// ---------------------------------------------------------------------------
// 子进程管理。
// ---------------------------------------------------------------------------

struct ServiceProcess {
  pid_t pid{-1};
  int stdout_fd{-1};
  std::string port;

  ServiceProcess() = default;
  ServiceProcess(const ServiceProcess&) = delete;
  ServiceProcess& operator=(const ServiceProcess&) = delete;
  ServiceProcess(ServiceProcess&& other) noexcept { move_from(std::move(other)); }
  ServiceProcess& operator=(ServiceProcess&& other) noexcept {
    if (this != &other) {
      stop();
      move_from(std::move(other));
    }
    return *this;
  }

  ~ServiceProcess() { stop(); }

  void stop() {
    if (pid > 0) {
      ::kill(pid, SIGTERM);
      int status = 0;
      for (int i = 0; i < 100; ++i) {
        if (::waitpid(pid, &status, WNOHANG) == pid) {
          pid = -1;
          break;
        }
        ::usleep(50000);
      }
      if (pid > 0) {
        ::kill(pid, SIGKILL);
        ::waitpid(pid, &status, 0);
        pid = -1;
      }
    }
    if (stdout_fd >= 0) {
      ::close(stdout_fd);
      stdout_fd = -1;
    }
  }

 private:
  void move_from(ServiceProcess&& other) noexcept {
    pid = other.pid;
    stdout_fd = other.stdout_fd;
    port = std::move(other.port);
    other.pid = -1;
    other.stdout_fd = -1;
    other.port.clear();
  }
};

std::optional<ServiceProcess> start_service(const std::vector<std::string>& extra_args,
                                            std::string* error) {
  int pipe_fds[2];
  if (::pipe(pipe_fds) != 0) {
    *error = "pipe() failed";
    return std::nullopt;
  }
  std::vector<std::string> args{PLANNER_MAP_SERVICE_PATH};
  args.insert(args.end(), extra_args.begin(), extra_args.end());
  std::vector<char*> argv;
  argv.reserve(args.size() + 1);
  for (auto& arg : args) {
    argv.push_back(arg.data());
  }
  argv.push_back(nullptr);

  const pid_t pid = ::fork();
  if (pid < 0) {
    *error = "fork() failed";
    ::close(pipe_fds[0]);
    ::close(pipe_fds[1]);
    return std::nullopt;
  }
  if (pid == 0) {
    ::dup2(pipe_fds[1], STDOUT_FILENO);
    ::close(pipe_fds[0]);
    ::close(pipe_fds[1]);
    ::execv(PLANNER_MAP_SERVICE_PATH, argv.data());
    ::_exit(127);
  }
  ::close(pipe_fds[1]);

  ServiceProcess process;
  process.pid = pid;
  process.stdout_fd = pipe_fds[0];
  std::string buffer;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
  while (std::chrono::steady_clock::now() < deadline) {
    struct pollfd pfd {
      process.stdout_fd, POLLIN, 0
    };
    const int ready = ::poll(&pfd, 1, 200);
    if (ready < 0) {
      *error = "poll() failed";
      process.stop();
      return std::nullopt;
    }
    if (ready == 0) {
      continue;
    }
    char chunk[512];
    const ssize_t got = ::read(process.stdout_fd, chunk, sizeof(chunk));
    if (got <= 0) {
      *error = "service exited before listening; output: " + buffer;
      process.stop();
      return std::nullopt;
    }
    buffer.append(chunk, static_cast<std::size_t>(got));
    const auto marker = buffer.find("listening on ");
    if (marker == std::string::npos) {
      continue;
    }
    const auto line_end = buffer.find('\n', marker);
    std::string line = buffer.substr(marker, line_end == std::string::npos
                                                 ? std::string::npos
                                                 : line_end - marker);
    const auto colon = line.rfind(':');
    if (colon == std::string::npos) {
      continue;
    }
    std::string port = line.substr(colon + 1);
    while (!port.empty() && (port.back() == '\r' || port.back() == ' ')) {
      port.pop_back();
    }
    if (!port.empty()) {
      process.port = port;
      return process;
    }
  }
  *error = "timeout waiting for service startup; output: " + buffer;
  process.stop();
  return std::nullopt;
}

// ---------------------------------------------------------------------------
// 共享测试夹具：整个套件共用一个服务和一个库端地图。
// ---------------------------------------------------------------------------

struct HttpResult {
  bool ok{false};
  int status{0};
  nlohmann::json body;
  std::string error;
};

struct ServiceFixture {
  std::optional<ServiceProcess> process;
  std::unique_ptr<httplib::Client> client;
  std::unique_ptr<Map> map;
  std::string error;

  static ServiceFixture& instance() {
    static ServiceFixture fixture;
    return fixture;
  }

  ServiceFixture() {
    auto imported = test::load_mapping_example();
    if (!imported.ok()) {
      error = "library import failed: " + imported.reason;
      return;
    }
    map = std::move(imported.map);

    std::string start_error;
    process = start_service({"--map", test::data_path("mapping_example.osm"),
                             "--origin-lat", "49.00345654351",
                             "--origin-lon", "8.42427590707",
                             "--port", "0", "--quiet"},
                            &start_error);
    if (!process.has_value()) {
      error = start_error;
      return;
    }
    client = std::make_unique<httplib::Client>("127.0.0.1", std::stoi(process->port));
    client->set_read_timeout(30, 0);
  }

  ~ServiceFixture() {
    if (process.has_value()) {
      process->stop();
    }
  }
};

ServiceFixture& service() { return ServiceFixture::instance(); }

HttpResult http_get(const std::string& path) {
  HttpResult result;
  auto response = service().client->Get(path.c_str());
  if (!response) {
    result.error = "GET " + path + " failed: " + httplib::to_string(response.error());
    return result;
  }
  result.status = response->status;
  try {
    result.body = nlohmann::json::parse(response->body);
    result.ok = true;
  } catch (const std::exception& parse_error) {
    result.error = std::string("response is not JSON: ") + parse_error.what();
  }
  return result;
}

HttpResult http_post(const std::string& path, const nlohmann::json& body) {
  HttpResult result;
  auto response = service().client->Post(path.c_str(), body.dump(), "application/json");
  if (!response) {
    result.error = "POST " + path + " failed: " + httplib::to_string(response.error());
    return result;
  }
  result.status = response->status;
  try {
    result.body = nlohmann::json::parse(response->body);
    result.ok = true;
  } catch (const std::exception& parse_error) {
    result.error = std::string("response is not JSON: ") + parse_error.what();
  }
  return result;
}

HttpResult http_post_raw(const std::string& path, const std::string& body) {
  HttpResult result;
  auto response = service().client->Post(path.c_str(), body, "application/json");
  if (!response) {
    result.error = "POST " + path + " failed: " + httplib::to_string(response.error());
    return result;
  }
  result.status = response->status;
  try {
    result.body = nlohmann::json::parse(response->body);
    result.ok = true;
  } catch (const std::exception& parse_error) {
    result.error = std::string("response is not JSON: ") + parse_error.what();
  }
  return result;
}

void expect_error_envelope(const HttpResult& result, int expected_status,
                           const std::string& expected_reason_part = "") {
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.status, expected_status);
  EXPECT_EQ(result.body["schema_version"], 1);
  EXPECT_EQ(result.body["type"], "error");
  EXPECT_TRUE(result.body.contains("status"));
  EXPECT_TRUE(result.body.contains("reason"));
  if (!expected_reason_part.empty()) {
    EXPECT_NE(result.body["reason"].get<std::string>().find(expected_reason_part),
              std::string::npos)
        << result.body["reason"];
  }
}

// ---------------------------------------------------------------------------
// 测试。
// ---------------------------------------------------------------------------

TEST(MapService, health_reports_ok_and_map_identity) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const auto result = http_get("/health");
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.status, 200);
  EXPECT_EQ(result.body["schema_version"], 1);
  EXPECT_EQ(result.body["type"], "health");
  EXPECT_EQ(result.body["status"], "ok");
  EXPECT_EQ(result.body["map_id"], fixture.map->info().map_id);
  EXPECT_EQ(result.body["map_version"], fixture.map->info().map_version);
  EXPECT_GE(result.body["uptime_s"].get<double>(), 0.0);
}

TEST(MapService, map_info_matches_library) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const auto result = http_get("/map/info");
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.status, 200);
  EXPECT_EQ(result.body["type"], "map_info");
  const auto& info = fixture.map->info();
  EXPECT_EQ(result.body["map_id"], info.map_id);
  EXPECT_EQ(result.body["map_version"], info.map_version);
  EXPECT_EQ(result.body["coordinate_system"]["projection"], "utm");
  EXPECT_DOUBLE_EQ(result.body["coordinate_system"]["origin"]["lat_deg"].get<double>(),
                   info.origin.lat_deg);
  EXPECT_EQ(result.body["counts"]["lanelets"], info.counts.lanelets);
  EXPECT_EQ(result.body["counts"]["areas"], info.counts.areas);
  EXPECT_DOUBLE_EQ(result.body["bounds_m"]["min_x_m"].get<double>(), info.bounds_m.min_x_m);
  EXPECT_DOUBLE_EQ(result.body["bounds_m"]["max_y_m"].get<double>(), info.bounds_m.max_y_m);
}

TEST(MapService, snapshot_matches_library_snapshot) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const auto result = http_get("/map/snapshot");
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.status, 200);
  EXPECT_EQ(result.body["schema_version"], 1);
  EXPECT_EQ(result.body["type"], "map_snapshot");
  EXPECT_EQ(result.body["lanelets"].size(), fixture.map->info().counts.lanelets);
  EXPECT_EQ(result.body["areas"].size(), fixture.map->info().counts.areas);
  // 与库生成的文档相同。
  EXPECT_EQ(result.body, fixture.map->to_snapshot_json());
}

TEST(MapService, point_query_matches_library_hits) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const auto ids = fixture.map->lanelet_ids();
  ASSERT_FALSE(ids.empty());
  const auto geometry = fixture.map->lanelet_geometry(ids.front());
  ASSERT_TRUE(geometry.has_value());
  ASSERT_FALSE(geometry->centerline.empty());
  const auto& mid = geometry->centerline[geometry->centerline.size() / 2];

  const auto library = fixture.map->query_point(mid.x_m, mid.y_m);
  const auto result =
      http_post("/map/query/point", nlohmann::json{{"x_m", mid.x_m}, {"y_m", mid.y_m}});
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.status, 200);
  EXPECT_EQ(result.body["type"], "point_query");
  EXPECT_EQ(result.body["lanelets"].size(), library.lanelets.size());
  std::vector<std::string> http_ids;
  std::vector<std::string> library_ids;
  for (const auto& entry : result.body["lanelets"]) {
    http_ids.push_back(entry["id"].get<std::string>());
    EXPECT_TRUE(entry["attributes"].is_object());
    EXPECT_TRUE(entry.contains("bounds_m"));
  }
  for (const auto& summary : library.lanelets) {
    library_ids.push_back(summary.id);
  }
  EXPECT_EQ(http_ids, library_ids);
  EXPECT_NE(std::find(http_ids.begin(), http_ids.end(), ids.front()), http_ids.end());
}

TEST(MapService, point_query_outside_returns_empty) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const auto& bounds = fixture.map->info().bounds_m;
  const auto result = http_post("/map/query/point",
                                nlohmann::json{{"x_m", bounds.max_x_m + 5000.0},
                                               {"y_m", bounds.max_y_m + 5000.0}});
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.status, 200);
  EXPECT_TRUE(result.body["lanelets"].empty());
  EXPECT_TRUE(result.body["areas"].empty());
}

TEST(MapService, bounds_query_matches_library) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const auto ids = fixture.map->lanelet_ids();
  ASSERT_FALSE(ids.empty());
  const auto geometry = fixture.map->lanelet_geometry(ids.front());
  ASSERT_TRUE(geometry.has_value());
  ASSERT_FALSE(geometry->left.empty());
  const auto& anchor = geometry->left[geometry->left.size() / 2];
  const Bounds2d window{anchor.x_m - 15.0, anchor.y_m - 15.0, anchor.x_m + 15.0,
                        anchor.y_m + 15.0};

  const auto library = fixture.map->query_bounds(window);
  const auto result = http_post("/map/query/bounds",
                                nlohmann::json{{"min_x_m", window.min_x_m},
                                               {"min_y_m", window.min_y_m},
                                               {"max_x_m", window.max_x_m},
                                               {"max_y_m", window.max_y_m}});
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.status, 200);
  EXPECT_EQ(result.body["type"], "bounds_query");
  std::vector<std::string> http_ids;
  for (const auto& entry : result.body["lanelets"]) {
    http_ids.push_back(entry["id"].get<std::string>());
  }
  std::vector<std::string> library_ids;
  for (const auto& summary : library.lanelets) {
    library_ids.push_back(summary.id);
  }
  EXPECT_EQ(http_ids, library_ids);
  EXPECT_EQ(http_ids, fixture.map->lanelets_in_bounds(window));
}

TEST(MapService, lanelet_get_matches_library_geometry) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const auto ids = fixture.map->lanelet_ids();
  ASSERT_FALSE(ids.empty());
  const auto library = fixture.map->lanelet_geometry(ids.front());
  ASSERT_TRUE(library.has_value());
  const auto result = http_get("/map/lanelet/" + ids.front());
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.status, 200);
  EXPECT_EQ(result.body["type"], "lanelet");
  EXPECT_EQ(result.body["id"], ids.front());
  EXPECT_EQ(result.body["attributes"], library->attributes);
  ASSERT_EQ(result.body["left"].size(), library->left.size());
  ASSERT_EQ(result.body["right"].size(), library->right.size());
  ASSERT_EQ(result.body["centerline"].size(), library->centerline.size());
  for (std::size_t i = 0; i < library->left.size(); ++i) {
    EXPECT_EQ(result.body["left"][i]["id"], library->left[i].id);
    EXPECT_DOUBLE_EQ(result.body["left"][i]["x_m"].get<double>(), library->left[i].x_m);
  }
}

TEST(MapService, lanelet_get_unknown_and_invalid_ids) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const auto unknown = http_get("/map/lanelet/999999999");
  expect_error_envelope(unknown, 404, "unknown lanelet id");
  const auto invalid = http_get("/map/lanelet/not-a-number");
  expect_error_envelope(invalid, 400, "invalid lanelet id");
}

TEST(MapService, following_matches_library) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  for (const auto& id : fixture.map->lanelet_ids()) {
    const auto library = fixture.map->following(id);
    if (library.status != planner::common::Status::kSuccess || library.ids.empty()) {
      continue;
    }
    const auto result = http_get("/map/lanelet/" + id + "/following");
    ASSERT_TRUE(result.ok) << result.error;
    EXPECT_EQ(result.status, 200);
    EXPECT_EQ(result.body["type"], "lanelet_relations");
    EXPECT_EQ(result.body["lanelet_id"], id);
    EXPECT_EQ(result.body["relation"], "following");
    std::vector<std::string> http_ids = result.body["ids"].get<std::vector<std::string>>();
    EXPECT_EQ(http_ids, library.ids);
    return;
  }
  FAIL() << "no lanelet with a follower found in the example map";
}

TEST(MapService, adjacent_requires_side_and_matches_library) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const auto ids = fixture.map->lanelet_ids();
  ASSERT_FALSE(ids.empty());
  const std::string& id = ids.front();

  const auto missing_side = http_get("/map/lanelet/" + id + "/adjacent");
  expect_error_envelope(missing_side, 400, "side=left|right");

  const auto library = fixture.map->adjacent(id, NeighborSide::kLeft);
  ASSERT_EQ(library.status, planner::common::Status::kSuccess) << library.reason;
  const auto result = http_get("/map/lanelet/" + id + "/adjacent?side=left");
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.status, 200);
  EXPECT_EQ(result.body["side"], "left");
  EXPECT_EQ(result.body["ids"].get<std::vector<std::string>>(), library.ids);

  const auto unknown = http_get("/map/lanelet/999999999/following");
  expect_error_envelope(unknown, 404, "unknown lanelet id");
}

TEST(MapService, window_query_matches_crop_result) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const auto ids = fixture.map->lanelet_ids();
  ASSERT_FALSE(ids.empty());
  const auto geometry = fixture.map->lanelet_geometry(ids.front());
  ASSERT_TRUE(geometry.has_value());
  ASSERT_FALSE(geometry->left.empty());
  const auto& anchor = geometry->left[geometry->left.size() / 2];
  const Bounds2d window{anchor.x_m - 25.0, anchor.y_m - 25.0, anchor.x_m + 25.0,
                        anchor.y_m + 25.0};
  const auto library = fixture.map->crop_to_bounds(window);

  const auto result = http_post("/map/query/window",
                                nlohmann::json{{"bounds",
                                                nlohmann::json{{"min_x_m", window.min_x_m},
                                                               {"min_y_m", window.min_y_m},
                                                               {"max_x_m", window.max_x_m},
                                                               {"max_y_m", window.max_y_m}}}});
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.status, 200);
  EXPECT_EQ(result.body["type"], "map_window");
  EXPECT_EQ(result.body["lanelet_ids"].get<std::vector<std::string>>(), library.lanelet_ids);
  EXPECT_EQ(result.body["paths"].size(), library.paths.size());
  EXPECT_DOUBLE_EQ(result.body["window_frame"]["x_m"].get<double>(), window.min_x_m);
  EXPECT_DOUBLE_EQ(result.body["window_frame"]["y_m"].get<double>(), window.min_y_m);
  EXPECT_DOUBLE_EQ(result.body["window_frame"]["yaw_rad"].get<double>(), 0.0);
  EXPECT_DOUBLE_EQ(result.body["transform"]["origin_in_map"]["x_m"].get<double>(),
                   window.min_x_m);
  ASSERT_FALSE(result.body["paths"].empty());
  for (const auto& point : result.body["paths"][0]["points"]) {
    EXPECT_GE(point["x_m"].get<double>(), window.min_x_m - 1e-9);
    EXPECT_LE(point["x_m"].get<double>(), window.max_x_m + 1e-9);
    EXPECT_GE(point["y_m"].get<double>(), window.min_y_m - 1e-9);
    EXPECT_LE(point["y_m"].get<double>(), window.max_y_m + 1e-9);
  }
}

TEST(MapService, window_center_size_variant_matches_bounds_variant) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const nlohmann::json center_size{{"center", nlohmann::json{{"x_m", 0.0}, {"y_m", 0.0}}},
                                   {"size", nlohmann::json{{"width_m", 100.0},
                                                           {"height_m", 80.0}}}};
  const auto result = http_post("/map/query/window", center_size);
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.status, 200);
  EXPECT_DOUBLE_EQ(result.body["window_m"]["min_x_m"].get<double>(), -50.0);
  EXPECT_DOUBLE_EQ(result.body["window_m"]["min_y_m"].get<double>(), -40.0);
  EXPECT_DOUBLE_EQ(result.body["window_m"]["max_x_m"].get<double>(), 50.0);
  EXPECT_DOUBLE_EQ(result.body["window_m"]["max_y_m"].get<double>(), 40.0);

  const auto invalid = http_post("/map/query/window",
                                 nlohmann::json{{"bounds",
                                                 nlohmann::json{{"min_x_m", 10.0},
                                                                {"min_y_m", 10.0},
                                                                {"max_x_m", -10.0},
                                                                {"max_y_m", -10.0}}}});
  expect_error_envelope(invalid, 400, "empty");
}

TEST(MapService, route_self_matches_library) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const auto ids = fixture.map->lanelet_ids();
  ASSERT_FALSE(ids.empty());
  const auto library = fixture.map->route(ids.front(), ids.front());
  ASSERT_EQ(library.status, planner::common::Status::kSuccess) << library.reason;
  const auto result = http_post("/map/route", nlohmann::json{{"from_id", ids.front()},
                                                             {"to_id", ids.front()}});
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.status, 200);
  EXPECT_EQ(result.body["type"], "route_result");
  EXPECT_EQ(result.body["status"], "success");
  EXPECT_EQ(result.body["lanelet_ids"].get<std::vector<std::string>>(), library.lanelet_ids);
  EXPECT_DOUBLE_EQ(result.body["length_m"].get<double>(), library.length_m);
}

TEST(MapService, route_no_path_and_unknown_ids) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  // 在 M3 测试中已固定：45312 -> 45318 是反向进入的连接，
  // 没有按存储方向的路由。
  const auto no_path =
      http_post("/map/route", nlohmann::json{{"from_id", "45312"}, {"to_id", "45318"}});
  expect_error_envelope(no_path, 404, "no route");

  const auto ids = fixture.map->lanelet_ids();
  ASSERT_FALSE(ids.empty());
  const auto unknown = http_post("/map/route", nlohmann::json{{"from_id", ids.front()},
                                                              {"to_id", "999999999"}});
  expect_error_envelope(unknown, 404, "unknown lanelet id");
}

TEST(MapService, invalid_requests_are_rejected_with_400) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;

  const auto bad_json = http_post_raw("/map/query/point", "not json");
  expect_error_envelope(bad_json, 400, "invalid JSON");

  const auto missing_field =
      http_post("/map/query/point", nlohmann::json{{"x_m", 1.0}});
  expect_error_envelope(missing_field, 400, "missing field");

  const auto wrong_type =
      http_post("/map/query/point", nlohmann::json{{"x_m", "one"}, {"y_m", 0.0}});
  expect_error_envelope(wrong_type, 400, "must be a number");

  const auto bad_id_type = http_post("/map/route", nlohmann::json{{"from_id", 1},
                                                                  {"to_id", "2"}});
  expect_error_envelope(bad_id_type, 400, "must be a string");

  const auto empty_window = http_post("/map/query/window", nlohmann::json::object());
  expect_error_envelope(empty_window, 400, "expected");
}

TEST(MapService, unknown_path_returns_json_error_envelope) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const auto result = http_get("/no/such/endpoint");
  expect_error_envelope(result, 404, "no handler");
}

TEST(MapService, area_endpoint_matches_library) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const auto ids = fixture.map->area_ids();
  if (ids.empty()) {
    GTEST_SKIP() << "example map has no areas";
  }
  const auto library = fixture.map->area_geometry(ids.front());
  ASSERT_TRUE(library.has_value());
  const auto result = http_get("/map/area/" + ids.front());
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.status, 200);
  EXPECT_EQ(result.body["type"], "area");
  EXPECT_EQ(result.body["id"], ids.front());
  EXPECT_EQ(result.body["outer"].size(), library->outer.size());
  EXPECT_EQ(result.body["attributes"], library->attributes);
}

}  // namespace
}  // namespace planner::map
