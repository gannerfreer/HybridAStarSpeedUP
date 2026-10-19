#pragma once

// W5 规划服务集成测试的共享测试支持。
//
// 测试夹具在临时端口（--port 0）上启动真实的 planner_plan_service 二进制，
// 等待打印出的端口，并提供小型 HTTP 辅助函数。每个 gtest 测试进程拥有一个
// 服务实例；测试夹具析构函数用 SIGTERM 停止服务。

#include <gtest/gtest.h>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <optional>
#include <poll.h>
#include <string>
#include <sys/types.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

#include <httplib.h>
#include <nlohmann/json.hpp>

#ifndef PLANNER_PLAN_SERVICE_PATH
#error "PLANNER_PLAN_SERVICE_PATH must be defined by the test CMakeLists"
#endif
#ifndef PLANNER_SERVICE_TEST_MAP_PATH
#error "PLANNER_SERVICE_TEST_MAP_PATH must be defined by the test CMakeLists"
#endif
#ifndef PLANNER_SERVICE_TEST_ORIGIN_LAT
#error "PLANNER_SERVICE_TEST_ORIGIN_LAT must be defined by the test CMakeLists"
#endif
#ifndef PLANNER_SERVICE_TEST_ORIGIN_LON
#error "PLANNER_SERVICE_TEST_ORIGIN_LON must be defined by the test CMakeLists"
#endif

namespace planner::service::test {

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

// 启动任意服务二进制（--port 0 自动分配），等待打印出的端口。
// X-M1：集成测试同时启动 planner_plan_service 与 planner_map_service。
inline std::optional<ServiceProcess> start_service_binary(
    const std::string& binary, const std::vector<std::string>& extra_args,
    std::string* error) {
  int pipe_fds[2];
  if (::pipe(pipe_fds) != 0) {
    *error = "pipe() failed";
    return std::nullopt;
  }
  std::vector<std::string> args{binary};
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
    ::execv(binary.c_str(), argv.data());
    ::_exit(127);
  }
  ::close(pipe_fds[1]);

  ServiceProcess process;
  process.pid = pid;
  process.stdout_fd = pipe_fds[0];
  std::string buffer;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(60);
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

inline std::optional<ServiceProcess> start_service(
    const std::vector<std::string>& extra_args, std::string* error) {
  return start_service_binary(PLANNER_PLAN_SERVICE_PATH, extra_args, error);
}

// ---------------------------------------------------------------------------
// HTTP 辅助函数。
// ---------------------------------------------------------------------------

struct HttpResult {
  bool transport_ok{false};
  int status{0};
  nlohmann::json body;
  std::string error;
};

inline HttpResult http_get(httplib::Client& client, const std::string& path) {
  HttpResult result;
  httplib::Result response = client.Get(path.c_str());
  if (!response) {
    result.error = httplib::to_string(response.error());
    return result;
  }
  result.transport_ok = true;
  result.status = response->status;
  try {
    result.body = nlohmann::json::parse(response->body);
  } catch (const std::exception& parse_error) {
    result.error = std::string("invalid JSON response: ") + parse_error.what();
    result.transport_ok = false;
  }
  return result;
}

inline HttpResult http_post(httplib::Client& client, const std::string& path,
                            const nlohmann::json& body) {
  HttpResult result;
  httplib::Result response = client.Post(path.c_str(), body.dump(), "application/json");
  if (!response) {
    result.error = httplib::to_string(response.error());
    return result;
  }
  result.transport_ok = true;
  result.status = response->status;
  try {
    result.body = nlohmann::json::parse(response->body);
  } catch (const std::exception& parse_error) {
    result.error = std::string("invalid JSON response: ") + parse_error.what();
    result.transport_ok = false;
  }
  return result;
}

// ---------------------------------------------------------------------------
// 共享测试夹具：每个测试进程一个服务进程。
// ---------------------------------------------------------------------------

struct ServiceFixture {
  std::optional<ServiceProcess> process;
  std::unique_ptr<httplib::Client> client;
  std::string map_id;
  std::string map_version;
  std::string run_dir;
  std::string error;

  static ServiceFixture& instance() {
    static ServiceFixture fixture;
    return fixture;
  }

  bool ok() const { return error.empty() && process.has_value() && client != nullptr; }

  ServiceFixture() {
    std::string template_dir =
        (std::filesystem::temp_directory_path() / "planner_plan_service_test_XXXXXX")
            .string();
    std::vector<char> buffer(template_dir.begin(), template_dir.end());
    buffer.push_back('\0');
    const char* made = ::mkdtemp(buffer.data());
    if (made != nullptr) {
      run_dir = made;
    }

    std::vector<std::string> args{
        "--map", PLANNER_SERVICE_TEST_MAP_PATH,
        "--origin-lat", PLANNER_SERVICE_TEST_ORIGIN_LAT,
        "--origin-lon", PLANNER_SERVICE_TEST_ORIGIN_LON,
        "--port", "0",
        "--quiet"};
    if (!run_dir.empty()) {
      args.push_back("--run-dir");
      args.push_back(run_dir);
    }
    std::string start_error;
    process = start_service(args, &start_error);
    if (!process.has_value()) {
      error = start_error;
      return;
    }
    client = std::make_unique<httplib::Client>("127.0.0.1", std::stoi(process->port));
    client->set_connection_timeout(5, 0);
    client->set_read_timeout(300, 0);

    const HttpResult health = http_get(*client, "/health");
    if (!health.transport_ok || health.status != 200) {
      error = "health request failed: " + health.error + " status=" +
              std::to_string(health.status);
      return;
    }
    map_id = health.body.value("map_id", "");
    map_version = health.body.value("map_version", "");
    if (map_id.empty() || map_version.empty()) {
      error = "health response is missing map_id/map_version";
    }
  }

  ~ServiceFixture() {
    if (process.has_value()) {
      process->stop();
    }
    if (!run_dir.empty()) {
      std::error_code ec;
      std::filesystem::remove_all(run_dir, ec);
    }
  }
};

inline ServiceFixture& require_service() {
  return ServiceFixture::instance();
}

// ---------------------------------------------------------------------------
// 请求构造器。
// ---------------------------------------------------------------------------

inline nlohmann::json pose(double x_m, double y_m, double yaw_rad) {
  return nlohmann::json{{"x_m", x_m}, {"y_m", y_m}, {"yaw_rad", yaw_rad}};
}

inline nlohmann::json rectangle_obstacle(const std::string& id, double x_min_m,
                                         double y_min_m, double x_max_m,
                                         double y_max_m) {
  return nlohmann::json{{"id", id},
                        {"type", "rectangle"},
                        {"x_min_m", x_min_m},
                        {"y_min_m", y_min_m},
                        {"x_max_m", x_max_m},
                        {"y_max_m", y_max_m}};
}

inline nlohmann::json open_request(const ServiceFixture& fixture,
                                   const std::string& request_id) {
  return nlohmann::json{{"request_id", request_id},
                        {"map_id", fixture.map_id},
                        {"map_version", fixture.map_version},
                        {"start", pose(2.0, 5.0, 0.0)},
                        {"goal", pose(18.0, 5.0, 0.0)},
                        {"obstacles", nlohmann::json::array()}};
}

// 轮询 GET /plan/status，直到到达终态或超时。返回状态体；
// 传输失败或超时时设置 `error`。
inline nlohmann::json wait_for_terminal(httplib::Client& client,
                                        const std::string& job_id,
                                        double timeout_s, std::string* error) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(
                                             static_cast<int>(timeout_s * 1000.0));
  while (std::chrono::steady_clock::now() < deadline) {
    const HttpResult status = http_get(client, "/plan/status/" + job_id);
    if (!status.transport_ok) {
      *error = "status request failed: " + status.error;
      return nlohmann::json::object();
    }
    if (status.status == 200) {
      const std::string state = status.body.value("status", "");
      if (state == "succeeded" || state == "failed" || state == "cancelled") {
        return status.body;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  *error = "timeout waiting for job " + job_id;
  return nlohmann::json::object();
}

}  // namespace planner::service::test
