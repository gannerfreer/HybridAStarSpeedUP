// X-M1 planner_map_service 栅格/场景端点测试（协议 v1 §3.2/§3.3/§8）。
//
// 在临时端口（--port 0）启动真实服务二进制，用 httplib 客户端验证：
//   * /health 的 grid_protocol_version / scene_rules_version；
//   * /map/grid/driveable 的成功字段、base64 自检、与库导出逐格一致、
//     P1–P6 错误码、P10 guard 耗尽、P8/P9 降级 k=2/k=3；
//   * /map/scene/preview 的 v6 P11/P13 冻结 scene_id、独立逐格复算、
//     窗口裁剪、障碍错误码与"预览基础域 = 可行驶域导出"。

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
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
// 子进程管理（与 test_service.cpp 同一模式）。
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
// 共享测试夹具：整个测试进程共用一个服务与库端地图。
// ---------------------------------------------------------------------------

struct HttpResult {
  bool ok{false};
  int status{0};
  nlohmann::json body;
  std::string error;
};

struct GridServiceFixture {
  std::optional<ServiceProcess> process;
  std::unique_ptr<httplib::Client> client;
  std::unique_ptr<Map> map;
  std::string error;

  static GridServiceFixture& instance() {
    static GridServiceFixture fixture;
    return fixture;
  }

  GridServiceFixture() {
    auto imported = test::load_mapping_example();
    if (!imported.ok()) {
      error = "library import failed: " + imported.reason;
      return;
    }
    map = std::move(imported.map);

    std::string start_error;
    process = start_service({"--map", test::data_path("mapping_example.osm"),
                             "--origin-lat", "49.00345654351",
                             "--origin-lon", "8.42427590707", "--port", "0", "--quiet"},
                            &start_error);
    if (!process.has_value()) {
      error = start_error;
      return;
    }
    client = std::make_unique<httplib::Client>("127.0.0.1", std::stoi(process->port));
    client->set_connection_timeout(5, 0);
    client->set_read_timeout(120, 0);
  }

  ~GridServiceFixture() {
    if (process.has_value()) {
      process->stop();
    }
  }
};

GridServiceFixture& service() { return GridServiceFixture::instance(); }

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

void expect_protocol_error(const HttpResult& result, int http_status,
                           const std::string& reason) {
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.status, http_status);
  EXPECT_EQ(result.body.value("schema_version", 0), 1);
  EXPECT_EQ(result.body.value("type", ""), "error");
  EXPECT_EQ(result.body.value("status", ""), "invalid_input");
  EXPECT_EQ(result.body.value("reason", ""), reason);
}

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

// 协议 §2.3 长度与三态自检。
void expect_valid_snapshot_cells(const nlohmann::json& doc) {
  const std::string encoding = doc.value("encoding", "");
  EXPECT_EQ(encoding, "base64_u8");
  const auto decoded = base64_decode(doc.value("cells", ""));
  ASSERT_TRUE(decoded.has_value()) << "cells is not valid base64";
  const std::size_t expected = static_cast<std::size_t>(doc.value("width", 0)) *
                               static_cast<std::size_t>(doc.value("height", 0));
  EXPECT_EQ(decoded->size(), expected);
  for (const unsigned char value : *decoded) {
    ASSERT_LE(value, 2u);
  }
}

nlohmann::json window_json(double min_x, double min_y, double max_x, double max_y) {
  return nlohmann::json{{"min_x_m", min_x},
                        {"min_y_m", min_y},
                        {"max_x_m", max_x},
                        {"max_y_m", max_y}};
}

// 与场景库相同的"单元中心位于闭合凸多边形内即占据"规则（独立复算用）。
bool center_inside(const std::vector<planner::common::Point2d>& polygon,
                   const planner::common::Point2d& point) {
  const std::size_t count = polygon.size();
  if (count < 3U) {
    return false;
  }
  double signed_area = 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    signed_area += polygon[i].x_m * polygon[(i + 1) % count].y_m -
                   polygon[(i + 1) % count].x_m * polygon[i].y_m;
  }
  const bool ccw = signed_area >= 0.0;
  for (std::size_t i = 0; i < count; ++i) {
    const planner::common::Point2d& a = polygon[i];
    const planner::common::Point2d& b = polygon[(i + 1) % count];
    const double cross =
        (b.x_m - a.x_m) * (point.y_m - a.y_m) - (b.y_m - a.y_m) * (point.x_m - a.x_m);
    if (ccw ? cross < 0.0 : cross > 0.0) {
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// 测试。
// ---------------------------------------------------------------------------

TEST(MapGridService, health_reports_grid_protocol_and_scene_versions) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const auto result = http_get("/health");
  ASSERT_TRUE(result.ok) << result.error;
  EXPECT_EQ(result.status, 200);
  EXPECT_EQ(result.body.value("type", ""), "health");
  EXPECT_EQ(result.body.value("grid_protocol_version", ""), "1.0");
  EXPECT_EQ(result.body.value("scene_rules_version", ""), "scene_v1");
}

TEST(MapGridService, driveable_grid_matches_library_export) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const Bounds2d window{0.0, 0.0, 3.0, 2.0};
  const double resolution = 0.1;
  const auto library = fixture.map->export_driveable_grid(window, resolution);
  ASSERT_TRUE(library.ok()) << library.reason;

  const auto result = http_post("/map/grid/driveable",
                                nlohmann::json{{"window", window_json(0.0, 0.0, 3.0, 2.0)},
                                               {"resolution_m", resolution},
                                               {"request_seq", 11},
                                               {"future_field", "tolerated"}});
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.status, 200);
  EXPECT_EQ(result.body.value("schema_version", 0), 1);
  EXPECT_EQ(result.body.value("type", ""), "grid_snapshot");
  EXPECT_EQ(result.body.value("source", ""), "driveable");
  EXPECT_FALSE(result.body.contains("scene_id"));
  EXPECT_FALSE(result.body.contains("scene_version"));
  EXPECT_EQ(result.body.value("map_id", ""), fixture.map->info().map_id);
  EXPECT_EQ(result.body.value("map_version", ""), fixture.map->info().map_version);
  EXPECT_EQ(result.body.value("request_seq", -1), 11);
  EXPECT_EQ(result.body.value("budget_cells", 0), 16777216);
  EXPECT_DOUBLE_EQ(result.body["window"]["min_x_m"].get<double>(), 0.0);
  EXPECT_DOUBLE_EQ(result.body["window"]["max_y_m"].get<double>(), 2.0);
  EXPECT_DOUBLE_EQ(result.body["requested_resolution_m"].get<double>(), 0.1);
  EXPECT_DOUBLE_EQ(result.body["effective_resolution_m"].get<double>(), 0.1);
  EXPECT_DOUBLE_EQ(result.body["resolution_m"].get<double>(), 0.1);
  EXPECT_FALSE(result.body["degraded"].get<bool>());
  EXPECT_EQ(result.body["width"], library.grid.width);
  EXPECT_EQ(result.body["height"], library.grid.height);
  EXPECT_DOUBLE_EQ(result.body["origin"]["x_m"].get<double>(), 0.0);
  EXPECT_DOUBLE_EQ(result.body["origin"]["yaw_rad"].get<double>(), 0.0);
  expect_valid_snapshot_cells(result.body);

  const auto decoded = base64_decode(result.body.value("cells", ""));
  ASSERT_TRUE(decoded.has_value());
  ASSERT_EQ(decoded->size(), library.grid.cells.size());
  std::size_t mismatches = 0;
  for (std::size_t i = 0; i < decoded->size(); ++i) {
    if (static_cast<int>((*decoded)[i]) != library.grid.cells[i]) {
      ++mismatches;
    }
  }
  EXPECT_EQ(mismatches, 0u);
}

TEST(MapGridService, driveable_grid_protocol_error_codes) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;

  // P1/P2/P3：空窗口 / 倒置窗口 → empty_window。
  expect_protocol_error(
      http_post("/map/grid/driveable",
                nlohmann::json{{"window", window_json(0.0, 0.0, 0.0, 1.0)},
                               {"resolution_m", 0.1}}),
      400, "empty_window");
  expect_protocol_error(
      http_post("/map/grid/driveable",
                nlohmann::json{{"window", window_json(0.0, 0.0, 1.0, 0.0)},
                               {"resolution_m", 0.1}}),
      400, "empty_window");
  expect_protocol_error(
      http_post("/map/grid/driveable",
                nlohmann::json{{"window", window_json(1.0, 1.0, 0.0, 0.0)},
                               {"resolution_m", 0.1}}),
      400, "empty_window");

  // P4/P5：分辨率 0 / 负 → invalid_resolution。
  expect_protocol_error(
      http_post("/map/grid/driveable",
                nlohmann::json{{"window", window_json(0.0, 0.0, 1.0, 1.0)},
                               {"resolution_m", 0.0}}),
      400, "invalid_resolution");
  expect_protocol_error(
      http_post("/map/grid/driveable",
                nlohmann::json{{"window", window_json(0.0, 0.0, 1.0, 1.0)},
                               {"resolution_m", -0.1}}),
      400, "invalid_resolution");

  // P6：1e999 → non_finite_value；裸 NaN → parse_error。
  expect_protocol_error(
      http_post_raw("/map/grid/driveable",
                    "{\"window\":{\"min_x_m\":0.0,\"min_y_m\":0.0,\"max_x_m\":1.0,"
                    "\"max_y_m\":1.0},\"resolution_m\":1e999}"),
      400, "non_finite_value");
  expect_protocol_error(
      http_post_raw("/map/grid/driveable",
                    "{\"window\":{\"min_x_m\":0.0,\"min_y_m\":0.0,\"max_x_m\":1.0,"
                    "\"max_y_m\":1.0},\"resolution_m\":NaN}"),
      400, "parse_error");

  // 缺字段 / 类型错误 / 请求序号非法 / 空请求体。
  expect_protocol_error(
      http_post("/map/grid/driveable", nlohmann::json{{"resolution_m", 0.1}}), 400,
      "missing_field");
  expect_protocol_error(
      http_post("/map/grid/driveable",
                nlohmann::json{{"window", window_json(0.0, 0.0, 1.0, 1.0)}}),
      400, "missing_field");
  expect_protocol_error(
      http_post("/map/grid/driveable",
                nlohmann::json{{"window", "not-an-object"}, {"resolution_m", 0.1}}),
      400, "type_error");
  expect_protocol_error(
      http_post("/map/grid/driveable",
                nlohmann::json{{"window", window_json(0.0, 0.0, 1.0, 1.0)},
                               {"resolution_m", 0.1},
                               {"request_seq", -1}}),
      400, "invalid_value");
  expect_protocol_error(
      http_post_raw("/map/grid/driveable", "{not json"), 400, "parse_error");
}

TEST(MapGridService, driveable_grid_rejects_guard_exhausted) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  // P10：32 次倍增后仍超预算 → grid_too_large（未使用 413）。
  expect_protocol_error(
      http_post("/map/grid/driveable",
                nlohmann::json{{"window", window_json(0.0, 0.0, 2e12, 2e12)},
                               {"resolution_m", 0.1},
                               {"request_seq", 8}}),
      400, "grid_too_large");
}

TEST(MapGridService, driveable_grid_degrades_to_k2) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  // P8：窗口 (0,0)-(1600,1600)、0.1 → 0.4、4000×4000 = 16,000,000 格。
  const auto result =
      http_post("/map/grid/driveable",
                nlohmann::json{{"window", window_json(0.0, 0.0, 1600.0, 1600.0)},
                               {"resolution_m", 0.1},
                               {"request_seq", 7}});
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.status, 200);
  EXPECT_TRUE(result.body["degraded"].get<bool>());
  EXPECT_DOUBLE_EQ(result.body["requested_resolution_m"].get<double>(), 0.1);
  EXPECT_DOUBLE_EQ(result.body["effective_resolution_m"].get<double>(), 0.4);
  EXPECT_DOUBLE_EQ(result.body["resolution_m"].get<double>(), 0.4);
  EXPECT_EQ(result.body["width"], 4000);
  EXPECT_EQ(result.body["height"], 4000);
  EXPECT_EQ(result.body.value("budget_cells", 0), 16777216);
  const std::string cells = result.body.value("cells", "");
  // base64 长度 = ceil(width*height/3)*4（协议 §2.3 长度自检）。
  EXPECT_EQ(cells.size(), ((16000000u + 2u) / 3u) * 4u);
  EXPECT_EQ(result.body.value("request_seq", -1), 7);
}

TEST(MapGridService, driveable_grid_degrades_to_k3) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  // P9：窗口 (0,0)-(2000,2000)、0.1 → 0.8、2500×2500 = 6,250,000 格。
  const auto result =
      http_post("/map/grid/driveable",
                nlohmann::json{{"window", window_json(0.0, 0.0, 2000.0, 2000.0)},
                               {"resolution_m", 0.1},
                               {"request_seq", 7}});
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.status, 200);
  EXPECT_TRUE(result.body["degraded"].get<bool>());
  EXPECT_DOUBLE_EQ(result.body["effective_resolution_m"].get<double>(), 0.8);
  EXPECT_EQ(result.body["width"], 2500);
  EXPECT_EQ(result.body["height"], 2500);
  const std::string cells = result.body.value("cells", "");
  EXPECT_EQ(cells.size(), ((6250000u + 2u) / 3u) * 4u);
}

TEST(MapGridService, scene_preview_matches_v6_p11_frozen_scene_id_and_cells) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const nlohmann::json obstacles = nlohmann::json::array(
      {nlohmann::json{{"id", "obs-a"},
                      {"type", "rectangle"},
                      {"x_min_m", 2.0},
                      {"y_min_m", 2.0},
                      {"x_max_m", 4.0},
                      {"y_max_m", 4.0}},
       nlohmann::json{{"id", "obs-b"},
                      {"type", "convex_polygon"},
                      {"vertices", nlohmann::json::array(
                                       {nlohmann::json{{"x_m", 10.0}, {"y_m", 10.0}},
                                        nlohmann::json{{"x_m", 12.0}, {"y_m", 10.0}},
                                        nlohmann::json{{"x_m", 11.0}, {"y_m", 13.0}}})}}});
  const auto result =
      http_post("/map/scene/preview",
                nlohmann::json{{"window", window_json(0.0, 0.0, 20.0, 20.0)},
                               {"resolution_m", 0.1},
                               {"obstacles", obstacles},
                               {"request_seq", 9}});
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.status, 200) << result.body.dump();
  EXPECT_EQ(result.body.value("source", ""), "planning_scene");
  EXPECT_EQ(result.body.value("scene_version", ""), "scene_v1");
  EXPECT_EQ(result.body.value("scene_id", ""),
            "044007e558d42fbe4ff7cafc2a239834cf75ceefc6fbc464425233e799ee22d8");
  EXPECT_EQ(result.body["width"], 200);
  EXPECT_EQ(result.body["height"], 200);
  EXPECT_DOUBLE_EQ(result.body["origin"]["x_m"].get<double>(), 0.0);
  EXPECT_DOUBLE_EQ(result.body["origin"]["y_m"].get<double>(), 0.0);
  EXPECT_FALSE(result.body["degraded"].get<bool>());
  EXPECT_EQ(result.body.value("request_seq", -1), 9);
  expect_valid_snapshot_cells(result.body);

  // 独立复算：可行驶域直接导出 + 手写中心点包含规则叠加，逐格比较。
  const Bounds2d window{0.0, 0.0, 20.0, 20.0};
  const auto base = fixture.map->export_driveable_grid(window, 0.1);
  ASSERT_TRUE(base.ok()) << base.reason;
  const auto decoded = base64_decode(result.body.value("cells", ""));
  ASSERT_TRUE(decoded.has_value());
  ASSERT_EQ(decoded->size(), base.grid.cells.size());
  const std::vector<planner::common::Point2d> obs_a{
      {2.0, 2.0}, {4.0, 2.0}, {4.0, 4.0}, {2.0, 4.0}};
  const std::vector<planner::common::Point2d> obs_b{
      {10.0, 10.0}, {12.0, 10.0}, {11.0, 13.0}};
  std::size_t mismatches = 0;
  for (int row = 0; row < base.grid.height; ++row) {
    for (int col = 0; col < base.grid.width; ++col) {
      int expected = base.grid.cell(row, col);
      const planner::common::Point2d center{base.grid.origin.x_m + (col + 0.5) * 0.1,
                                            base.grid.origin.y_m + (row + 0.5) * 0.1};
      if (center_inside(obs_a, center) || center_inside(obs_b, center)) {
        expected = common::kCellOccupied;
      }
      const std::size_t index = static_cast<std::size_t>(row) * base.grid.width + col;
      if (static_cast<int>((*decoded)[index]) != expected) {
        ++mismatches;
      }
    }
  }
  EXPECT_EQ(mismatches, 0u);
}

TEST(MapGridService, scene_preview_matches_v6_p13_preview_scene_id) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const auto result = http_post(
      "/map/scene/preview",
      nlohmann::json{
          {"window", window_json(0.0, 0.0, 6.0, 6.0)},
          {"resolution_m", 0.1},
          {"obstacles", nlohmann::json::array(
                            {nlohmann::json{{"id", "obs-preview"},
                                            {"type", "rectangle"},
                                            {"x_min_m", 1.0},
                                            {"y_min_m", 1.0},
                                            {"x_max_m", 2.0},
                                            {"y_max_m", 2.0}}})}});
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.status, 200);
  EXPECT_EQ(result.body.value("scene_id", ""),
            "f903719dba895138267c04fb1d7aa2588ab0420e25e5c1e60fee4c536c01d766");
  EXPECT_EQ(result.body["width"], 60);
  EXPECT_EQ(result.body["height"], 60);
}

TEST(MapGridService, scene_preview_clips_window_to_map_bounds) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const Bounds2d& bounds = fixture.map->info().bounds_m;
  const auto result = http_post(
      "/map/scene/preview",
      nlohmann::json{{"window", window_json(bounds.min_x_m - 500.0, -100.0, 5.0, 5.0)},
                     {"resolution_m", 0.1}});
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.status, 200);
  // 服务回显量化后的窗口（协议 §2.4/§5.1，步长 1e-9）。
  EXPECT_NEAR(result.body["window"]["min_x_m"].get<double>(), bounds.min_x_m, 1e-9);
  EXPECT_NEAR(result.body["window"]["min_y_m"].get<double>(), -100.0, 1e-9);
  EXPECT_NEAR(result.body["window"]["max_x_m"].get<double>(), 5.0, 1e-9);
  EXPECT_NEAR(result.body["window"]["max_y_m"].get<double>(), 5.0, 1e-9);
  EXPECT_DOUBLE_EQ(result.body["origin"]["x_m"].get<double>(),
                   result.body["window"]["min_x_m"].get<double>());
  EXPECT_DOUBLE_EQ(result.body["origin"]["y_m"].get<double>(),
                   result.body["window"]["min_y_m"].get<double>());
  expect_valid_snapshot_cells(result.body);
}

TEST(MapGridService, scene_preview_without_obstacles_matches_driveable_export) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  // v6 P11 规则：同一窗口、无障碍时，预览基础域 = 可行驶域直接输出。
  const Bounds2d window{0.0, 0.0, 3.0, 2.0};
  const auto base = fixture.map->export_driveable_grid(window, 0.1);
  ASSERT_TRUE(base.ok()) << base.reason;
  const auto result =
      http_post("/map/scene/preview",
                nlohmann::json{{"window", window_json(0.0, 0.0, 3.0, 2.0)},
                               {"resolution_m", 0.1}});
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.status, 200);
  EXPECT_EQ(result.body["width"], base.grid.width);
  EXPECT_EQ(result.body["height"], base.grid.height);
  const auto decoded = base64_decode(result.body.value("cells", ""));
  ASSERT_TRUE(decoded.has_value());
  ASSERT_EQ(decoded->size(), base.grid.cells.size());
  std::size_t mismatches = 0;
  for (std::size_t i = 0; i < decoded->size(); ++i) {
    if (static_cast<int>((*decoded)[i]) != base.grid.cells[i]) {
      ++mismatches;
    }
  }
  EXPECT_EQ(mismatches, 0u);
}

TEST(MapGridService, scene_preview_protocol_error_codes) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;

  // P7：窗口在地图外。
  expect_protocol_error(
      http_post("/map/scene/preview",
                nlohmann::json{{"window", window_json(1e6, 1e6, 1e6 + 10.0, 1e6 + 10.0)},
                               {"resolution_m", 0.1},
                               {"request_seq", 6}}),
      400, "window_outside_map");

  // 空窗口 / 分辨率非法（与栅格端点同一错误码）。
  expect_protocol_error(
      http_post("/map/scene/preview",
                nlohmann::json{{"window", window_json(0.0, 0.0, 0.0, 1.0)},
                               {"resolution_m", 0.1}}),
      400, "empty_window");
  expect_protocol_error(
      http_post("/map/scene/preview",
                nlohmann::json{{"window", window_json(0.0, 0.0, 1.0, 1.0)},
                               {"resolution_m", 0.0}}),
      400, "invalid_resolution");

  // 退化矩形（含倒置规范化后退化）→ invalid_scene。
  expect_protocol_error(
      http_post("/map/scene/preview",
                nlohmann::json{
                    {"window", window_json(0.0, 0.0, 6.0, 6.0)},
                    {"resolution_m", 0.1},
                    {"obstacles", nlohmann::json::array(
                                      {nlohmann::json{{"id", "bad"},
                                                      {"type", "rectangle"},
                                                      {"x_min_m", 1.0},
                                                      {"y_min_m", 1.0},
                                                      {"x_max_m", 1.0},
                                                      {"y_max_m", 2.0}}})}}),
      400, "invalid_scene");

  // 顶点 < 3 → invalid_scene。
  expect_protocol_error(
      http_post("/map/scene/preview",
                nlohmann::json{
                    {"window", window_json(0.0, 0.0, 6.0, 6.0)},
                    {"resolution_m", 0.1},
                    {"obstacles", nlohmann::json::array(
                                      {nlohmann::json{{"id", "bad"},
                                                      {"type", "convex_polygon"},
                                                      {"vertices", nlohmann::json::array(
                                                                       {nlohmann::json{{"x_m", 0.0}, {"y_m", 0.0}},
                                                                        nlohmann::json{{"x_m", 1.0}, {"y_m", 0.0}}})}}})}}),
      400, "invalid_scene");

  // 非凸多边形 → invalid_scene。
  expect_protocol_error(
      http_post("/map/scene/preview",
                nlohmann::json{
                    {"window", window_json(0.0, 0.0, 6.0, 6.0)},
                    {"resolution_m", 0.1},
                    {"obstacles", nlohmann::json::array(
                                      {nlohmann::json{{"id", "bad"},
                                                      {"type", "convex_polygon"},
                                                      {"vertices", nlohmann::json::array(
                                                                       {nlohmann::json{{"x_m", 0.0}, {"y_m", 0.0}},
                                                                        nlohmann::json{{"x_m", 2.0}, {"y_m", 0.0}},
                                                                        nlohmann::json{{"x_m", 1.0}, {"y_m", 0.5}},
                                                                        nlohmann::json{{"x_m", 2.0}, {"y_m", 2.0}}})}}})}}),
      400, "invalid_scene");

  // obstacles 类型错误 → type_error。
  expect_protocol_error(
      http_post("/map/scene/preview",
                nlohmann::json{{"window", window_json(0.0, 0.0, 6.0, 6.0)},
                               {"resolution_m", 0.1},
                               {"obstacles", "not-an-array"}}),
      400, "type_error");
}

TEST(MapGridService, scene_preview_normalizes_inverted_rectangle) {
  auto& fixture = service();
  ASSERT_TRUE(fixture.error.empty()) << fixture.error;
  const auto result = http_post(
      "/map/scene/preview",
      nlohmann::json{
          {"window", window_json(0.0, 0.0, 6.0, 6.0)},
          {"resolution_m", 0.1},
          {"obstacles", nlohmann::json::array(
                            {nlohmann::json{{"id", "inv"},
                                            {"type", "rectangle"},
                                            {"x_min_m", 2.0},
                                            {"y_min_m", 2.0},
                                            {"x_max_m", 1.0},
                                            {"y_max_m", 1.0}}})}});
  ASSERT_TRUE(result.ok) << result.error;
  ASSERT_EQ(result.status, 200) << result.body.dump();
  const auto decoded = base64_decode(result.body.value("cells", ""));
  ASSERT_TRUE(decoded.has_value());
  const int width = result.body.value("width", 0);
  // 规范化矩形 (1,1)-(2,2) 覆盖单元中心 (1.5,1.5) → 占据。
  const int row = static_cast<int>(std::floor(1.5 / 0.1));
  const int col = static_cast<int>(std::floor(1.5 / 0.1));
  const std::size_t index = static_cast<std::size_t>(row) * static_cast<std::size_t>(width) +
                            static_cast<std::size_t>(col);
  ASSERT_LT(index, decoded->size());
  EXPECT_EQ(static_cast<int>((*decoded)[index]), common::kCellOccupied);
}

}  // namespace
}  // namespace planner::map
