// planner_plan_service：W5 规划服务（HTTP/JSON + WebSocket 事件）。
//
// 启动时加载一次 Lanelet2 OSM 地图（标识、版本、坐标系和外层规划域），
// 并提供 Src/service/README.md 中记录的 W5 契约：
//
//   GET  /health
//   POST /plan/submit
//   POST /plan/cancel
//   GET  /plan/status/{job_id}
//   GET  /plan/result/{job_id}
//   GET  /plan/scene/{scene_id}      （X-M1：规划输入快照）
//   WS   /plan/events
//
// 每个 HTTP 响应都携带公共信封（{"schema_version": 1, "type": ...}）；
// 错误在请求体中使用 Status 语义，并匹配相应 HTTP 状态码：400 输入无效、
// 404 未知作业、409 stale_map、500 内部故障。/plan/events 在每次状态/阶段
// 切换时推送 {type:"job_update", ...}，作业到达终态时推送
// {type:"job_result", ...}；结果通过 HTTP 拉取。
//
// WebSocket 选择：锁定版本的 cpp-httplib v0.60.1 提供原生 WebSocket 服务器
// （RFC 6455，每连接一线程，阻塞 I/O），因此直接实现
// ws://.../plan/events；无需 SSE 回退。理由与规模边界见 README“推送方式”。
//
// 命令行（tools/components.json 的 service 条目）：
//   planner_plan_service --map <osm> --origin-lat <deg> --origin-lon <deg>
//       --port <port> [--run-dir <dir>]
//
// 退出码：0 正常关闭；1 地图导入/运行时失败；2 用法错误；3 无法绑定请求的端口。
//
// License/provenance: this binary is the only integration consumer of the
// isolated experimental planner::dftpav backend (GPL-3.0 port; see
// Src/dftpav/PROVENANCE.md). The service itself only consumes the public
// planner component interfaces.

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <httplib.h>
#include <nlohmann/json.hpp>

#include "planner/map/map.hpp"
#include "planner/service/event_hub.hpp"
#include "planner/service/service.hpp"
#include "planner/service/types.hpp"

namespace {

using planner::map::GeoPoint;
using planner::map::Map;
using planner::map::MapLoadOptions;
using planner::map::ProjectionKind;
using planner::service::EventHub;
using planner::service::HttpResponse;
using planner::service::PlanningService;
using planner::service::ServiceConfig;

constexpr int kDefaultPort = 8766;
constexpr const char* kDefaultHost = "127.0.0.1";

std::atomic<bool> g_stop_requested{false};
httplib::Server* g_server = nullptr;

extern "C" void handle_signal(int /*signal*/) {
  g_stop_requested = true;
  if (g_server != nullptr) {
    g_server->stop();
  }
}

HttpResponse error_response(int http_status, const std::string& status,
                            const std::string& reason) {
  HttpResponse response;
  response.status = http_status;
  response.body = planner::service::envelope("error");
  response.body["status"] = status;
  response.body["reason"] = reason;
  return response;
}

void send_json(httplib::Response& res, const HttpResponse& response) {
  res.status = response.status;
  if (!response.raw_body.empty()) {
    // GET /plan/scene/{scene_id}：逐字节返回保存的 grid_snapshot 原文。
    res.set_content(response.raw_body, "application/json; charset=utf-8");
    return;
  }
  res.set_content(response.body.dump(), "application/json; charset=utf-8");
}

bool parse_body(const httplib::Request& req, nlohmann::json* body, std::string* error) {
  if (req.body.empty()) {
    *error = "request body is empty; expected a JSON object";
    return false;
  }
  try {
    *body = nlohmann::json::parse(req.body);
  } catch (const std::exception& parse_error) {
    *error = std::string("invalid JSON body: ") + parse_error.what();
    return false;
  }
  if (!body->is_object()) {
    *error = "request body must be a JSON object";
    return false;
  }
  return true;
}

void handle_submit(PlanningService& service, const httplib::Request& req,
                   httplib::Response& res) {
  nlohmann::json body;
  std::string error;
  if (!parse_body(req, &body, &error)) {
    send_json(res, error_response(400, "invalid_input", error));
    return;
  }
  send_json(res, service.submit(body));
}

void handle_cancel(PlanningService& service, const httplib::Request& req,
                   httplib::Response& res) {
  nlohmann::json body;
  std::string error;
  if (!parse_body(req, &body, &error)) {
    send_json(res, error_response(400, "invalid_input", error));
    return;
  }
  send_json(res, service.cancel(body));
}

// 一个 WebSocket 连接：排空订阅者队列并转发事件。读超时让循环能及时响应
// 排队事件和服务关闭；客户端消息被忽略（该流仅为服务器推送）。
void handle_events(EventHub& hub, httplib::ws::WebSocket& ws) {
  const std::shared_ptr<EventHub::Subscriber> subscriber = hub.subscribe();
  ws.set_read_timeout(std::chrono::milliseconds(20));
  std::string outgoing;
  std::string incoming;
  while (ws.is_open() && !g_stop_requested.load()) {
    if (subscriber->pop(&outgoing, std::chrono::milliseconds(20))) {
      if (!ws.send(outgoing)) {
        break;
      }
      continue;
    }
    if (subscriber->closed()) {
      if (subscriber->overflowed()) {
        ws.close(httplib::ws::CloseStatus::PolicyViolation,
                 "event queue overflow; resynchronize over HTTP");
      } else {
        ws.close(httplib::ws::CloseStatus::GoingAway, "service stopping");
      }
      break;
    }
    const httplib::ws::ReadResult result = ws.read(incoming);
    if (result == httplib::ws::Fail) {
      break;
    }
  }
  hub.unsubscribe(subscriber);
}

void configure_server(httplib::Server& server, PlanningService& service,
                      EventHub& hub, bool quiet) {
  server.Get("/health", [&service](const httplib::Request&, httplib::Response& res) {
    send_json(res, service.health());
  });
  server.Post("/plan/submit",
              [&service](const httplib::Request& req, httplib::Response& res) {
                handle_submit(service, req, res);
              });
  server.Post("/plan/cancel",
              [&service](const httplib::Request& req, httplib::Response& res) {
                handle_cancel(service, req, res);
              });
  server.Get(R"(/plan/status/([^/]+))",
             [&service](const httplib::Request& req, httplib::Response& res) {
               send_json(res, service.status(req.matches[1]));
             });
  server.Get(R"(/plan/result/([^/]+))",
             [&service](const httplib::Request& req, httplib::Response& res) {
               send_json(res, service.result(req.matches[1]));
             });
  // X-M1（协议 §3.4）：规划输入快照；未知/已淘汰返回 404 unknown_scene。
  server.Get(R"(/plan/scene/([^/]+))",
             [&service](const httplib::Request& req, httplib::Response& res) {
               send_json(res, service.scene_snapshot(req.matches[1]));
             });
  server.WebSocket("/plan/events",
                   [&hub](const httplib::Request&, httplib::ws::WebSocket& ws) {
                     handle_events(hub, ws);
                   });

  server.set_error_handler([](const httplib::Request& req, httplib::Response& res) {
    if (!res.body.empty()) {
      return;  // 处理函数已生成结构化错误
    }
    send_json(res, error_response(404, "not_found",
                                  "no handler for " + req.method + " " + req.path));
  });
  server.set_exception_handler(
      [](const httplib::Request&, httplib::Response& res, std::exception_ptr error) {
        std::string reason = "internal error";
        try {
          if (error) {
            std::rethrow_exception(error);
          }
        } catch (const std::exception& caught) {
          reason = std::string("internal error: ") + caught.what();
        } catch (...) {
        }
        send_json(res, error_response(500, "internal_error", reason));
      });
  if (!quiet) {
    server.set_logger([](const httplib::Request& req, const httplib::Response& res) {
      std::cout << req.method << " " << req.path << " -> " << res.status << std::endl;
    });
  }
}

std::string utc_now_iso8601() {
  const std::time_t now = std::time(nullptr);
  std::tm utc{};
  gmtime_r(&now, &utc);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &utc);
  return buffer;
}

void write_service_status(const ServiceConfig& config,
                          const planner::map::MapInfo& info,
                          const std::string& host, int port) {
  if (config.run_dir.empty()) {
    return;
  }
  std::error_code ec;
  std::filesystem::create_directories(config.run_dir, ec);
  nlohmann::json doc = planner::service::envelope("plan_service_status");
  doc["status"] = "listening";
  doc["host"] = host;
  doc["port"] = port;
  doc["map_id"] = info.map_id;
  doc["map_version"] = info.map_version;
  doc["started_at_utc"] = utc_now_iso8601();
  doc["default_hybrid_backend"] = "kth_normalized";
  doc["push_transport"] = "websocket";
  std::ofstream out(std::filesystem::path(config.run_dir) / "service_status.json");
  out << doc.dump(2) << '\n';
}

void print_usage(std::ostream& os) {
  os << "Usage: planner_plan_service --map <map.osm> --origin-lat <deg>\n"
        "       --origin-lon <deg> [--port <port>] [--host <addr>]\n"
        "       [--projection utm|mercator|spherical_mercator] [--origin-ele <m>]\n"
        "       [--no-validation] [--run-dir <dir>] [--quiet]\n"
        "\n"
        "Serves the W5 planning API on http://<host>:<port>/ (port 0 picks a\n"
        "free port and prints the actual value) and pushes job events on\n"
        "ws://<host>:<port>/plan/events. Default port: 8766.\n";
}

}  // namespace

int main(int argc, char** argv) {
  std::string map_path;
  std::string host = kDefaultHost;
  std::string run_dir;
  int port = kDefaultPort;
  MapLoadOptions options;
  options.projection = ProjectionKind::kUtm;
  bool origin_set = false;
  GeoPoint origin;
  bool quiet = false;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto next = [&](const char* flag) -> std::optional<std::string> {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << flag << "\n";
        return std::nullopt;
      }
      return std::string(argv[++i]);
    };
    if (arg == "--map") {
      const auto value = next("--map");
      if (!value) return 2;
      map_path = *value;
    } else if (arg == "--origin-lat" || arg == "--origin-lon" || arg == "--origin-ele") {
      const auto value = next(arg.c_str());
      if (!value) return 2;
      try {
        const double parsed = std::stod(*value);
        if (arg == "--origin-lat") {
          origin.lat_deg = parsed;
        } else if (arg == "--origin-lon") {
          origin.lon_deg = parsed;
        } else {
          origin.ele_m = parsed;
        }
        origin_set = true;
      } catch (const std::exception&) {
        std::cerr << "invalid value for " << arg << "\n";
        return 2;
      }
    } else if (arg == "--port") {
      const auto value = next("--port");
      if (!value) return 2;
      try {
        port = std::stoi(*value);
      } catch (const std::exception&) {
        std::cerr << "invalid port: " << *value << "\n";
        return 2;
      }
      if (port < 0 || port > 65535) {
        std::cerr << "port out of range: " << port << "\n";
        return 2;
      }
    } else if (arg == "--host") {
      const auto value = next("--host");
      if (!value) return 2;
      host = *value;
    } else if (arg == "--projection") {
      const auto value = next("--projection");
      if (!value) return 2;
      const auto kind = planner::map::projection_kind_from_string(*value);
      if (!kind) {
        std::cerr << "unknown projection: " << *value << "\n";
        return 2;
      }
      options.projection = *kind;
    } else if (arg == "--no-validation") {
      options.validate_on_import = false;
    } else if (arg == "--run-dir") {
      const auto value = next("--run-dir");
      if (!value) return 2;
      run_dir = *value;
    } else if (arg == "--quiet") {
      quiet = true;
    } else if (arg == "--help" || arg == "-h") {
      print_usage(std::cout);
      return 0;
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      print_usage(std::cerr);
      return 2;
    }
  }

  if (map_path.empty() || !origin_set) {
    std::cerr << "--map, --origin-lat and --origin-lon are required\n";
    print_usage(std::cerr);
    return 2;
  }
  options.origin = origin;

  auto import = Map::import_osm_file(map_path, options);
  if (!import.ok()) {
    std::cerr << "map import failed: " << import.reason << "\n";
    return 1;
  }

  ServiceConfig config;
  config.host = host;
  config.port = port;
  config.run_dir = run_dir;

  EventHub hub;
  // 服务只保存地图指针；import.map 在服务停止后仍然存活（见作用域顺序）。
  PlanningService service(*import.map, config, &hub);

  httplib::Server server;
  g_server = &server;
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);
  configure_server(server, service, hub, quiet);

  int bound_port = port;
  if (port == 0) {
    bound_port = server.bind_to_any_port(host);
    if (bound_port <= 0) {
      std::cerr << "cannot bind an ephemeral port on " << host << "\n";
      return 3;
    }
  } else if (!server.bind_to_port(host, port)) {
    std::cerr << "cannot bind " << host << ":" << port << " (already in use?)\n";
    return 3;
  }

  std::cout << "planner_plan_service listening on " << host << ":" << bound_port
            << std::endl;
  write_service_status(config, import.map->info(), host, bound_port);

  const bool listen_ok = server.listen_after_bind();
  service.stop();
  std::cout << "planner_plan_service stopped (listen_result="
            << (listen_ok ? "ok" : "failed")
            << ", requested=" << (g_stop_requested ? "signal" : "none") << ")"
            << std::endl;
  return (listen_ok || g_stop_requested) ? 0 : 1;
}
