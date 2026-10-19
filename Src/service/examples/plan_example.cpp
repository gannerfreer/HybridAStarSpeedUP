// planner_plan_service_example：独立流水线示例（无 HTTP 服务器）。
//
// 与服务完全一致地导入 OSM 地图，为固定的开阔场景构建本地规划场景
// （--self-check 模式下还包含窄通道场景），运行完整流水线
// Hybrid A* -> 走廊 -> Dftpav -> 验证，并打印简要摘要。
//
// 用法：
//   planner_plan_service_example --map <map.osm> --origin-lat <deg>
//       --origin-lon <deg> [--self-check] [--backend kth_normalized]
//
// 退出码：0 成功；1 流水线失败（或 --self-check 用例失败）；2 用法错误。

#include <chrono>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include "planner/map/map.hpp"
#include "planner/service/parse.hpp"
#include "planner/service/pipeline.hpp"
#include "planner/service/scene.hpp"
#include "planner/service/types.hpp"

namespace {

using planner::common::Pose2d;
using planner::map::GeoPoint;
using planner::map::Map;
using planner::map::MapLoadOptions;
using planner::map::MapInfo;
using planner::map::ProjectionKind;
using planner::service::PipelineResult;
using planner::service::ServiceObstacle;
using planner::service::ServicePlanRequest;
using planner::service::build_scene;
using planner::service::default_vehicle;
using planner::service::run_pipeline;

struct Options {
  std::string map_path;
  GeoPoint origin;
  bool origin_set{false};
  std::string backend{"kth_normalized"};
  bool self_check{false};
};

ServiceObstacle rectangle(const std::string& id, double x_min_m, double y_min_m,
                          double x_max_m, double y_max_m) {
  ServiceObstacle obstacle;
  obstacle.kind = ServiceObstacle::Kind::kRectangle;
  obstacle.id = id;
  obstacle.x_min_m = x_min_m;
  obstacle.y_min_m = y_min_m;
  obstacle.x_max_m = x_max_m;
  obstacle.y_max_m = y_max_m;
  return obstacle;
}

ServicePlanRequest make_request(const MapInfo& info, const std::string& backend,
                                const std::string& request_id, const Pose2d& goal,
                                std::vector<ServiceObstacle> obstacles) {
  ServicePlanRequest request;
  request.request_id = request_id;
  request.map_id = info.map_id;
  request.map_version = info.map_version;
  request.vehicle = default_vehicle();
  request.start = Pose2d{2.0, 5.0, 0.0};
  request.goal = goal;
  request.obstacles = std::move(obstacles);
  request.params.hybrid_backend = backend;
  return request;
}

int run_case(const Map& map, const ServicePlanRequest& request,
             const std::string& label) {
  const auto scene = build_scene(map, request.vehicle, request.start, request.goal,
                                 request.obstacles);
  if (!scene.ok) {
    std::cout << label << ": scene failed: " << scene.reason << " " << scene.detail
              << "\n";
    return 1;
  }
  std::cout << label << ": grid " << scene.grid.width << "x" << scene.grid.height
            << " @ " << scene.grid.resolution_m << " m\n";
  const auto started = std::chrono::steady_clock::now();
  const PipelineResult result =
      run_pipeline(request, scene.grid, scene.scene,
                   [&](const std::string& stage) {
                     std::cout << label << ": stage " << stage << "\n";
                   });
  const double total_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
  if (!result.ok()) {
    std::cout << label << ": FAILED status=" << planner::common::to_string(result.status)
              << " reason=" << result.reason << " detail=" << result.detail << "\n";
    return 1;
  }
  std::size_t path_points = 0;
  for (const auto& segment : result.hybrid.path) {
    path_points += segment.points.size();
  }
  std::cout << label << ": OK backend=" << result.hybrid_backend
            << " segments=" << result.hybrid.path.size()
            << " path_points=" << path_points
            << " corridor_polygons=" << result.corridor.corridor.polygons.size()
            << " trajectory_points=" << result.optimize.trajectory.size()
            << " validation=" << (result.validation.valid ? "valid" : "invalid")
            << " total_s=" << total_s << "\n";
  return 0;
}

void print_usage(std::ostream& os) {
  os << "Usage: planner_plan_service_example --map <map.osm> --origin-lat <deg>\n"
        "       --origin-lon <deg> [--self-check] [--backend <id>]\n";
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  MapLoadOptions load;
  load.projection = ProjectionKind::kUtm;

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
      options.map_path = *value;
    } else if (arg == "--origin-lat" || arg == "--origin-lon") {
      const auto value = next(arg.c_str());
      if (!value) return 2;
      try {
        if (arg == "--origin-lat") {
          options.origin.lat_deg = std::stod(*value);
        } else {
          options.origin.lon_deg = std::stod(*value);
        }
        options.origin_set = true;
      } catch (const std::exception&) {
        std::cerr << "invalid value for " << arg << "\n";
        return 2;
      }
    } else if (arg == "--backend") {
      const auto value = next("--backend");
      if (!value) return 2;
      options.backend = *value;
    } else if (arg == "--self-check") {
      options.self_check = true;
    } else if (arg == "--help" || arg == "-h") {
      print_usage(std::cout);
      return 0;
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      print_usage(std::cerr);
      return 2;
    }
  }

  if (options.map_path.empty() || !options.origin_set) {
    std::cerr << "--map, --origin-lat and --origin-lon are required\n";
    print_usage(std::cerr);
    return 2;
  }
  load.origin = options.origin;

  auto import = Map::import_osm_file(options.map_path, load);
  if (!import.ok()) {
    std::cerr << "map import failed: " << import.reason << "\n";
    return 1;
  }
  const MapInfo info = import.map->info();
  std::cout << "map " << info.map_id << " version " << info.map_version << "\n";

  int failures = 0;
  if (!options.self_check) {
    failures += run_case(*import.map,
                         make_request(info, options.backend, "example-open",
                                      Pose2d{18.0, 5.0, 0.0}, {}),
                         "open");
    return failures == 0 ? 0 : 1;
  }

  // 自检：开阔场景加上两堵墙之间 3 m 宽的通道（corridor_111 测试夹具的
  // 几何形状），使用所选后端。场景域为地图 lanelet 可行驶域 + 请求障碍。
  failures += run_case(*import.map,
                       make_request(info, options.backend, "selfcheck-open",
                                    Pose2d{18.0, 5.0, 0.0}, {}),
                       "selfcheck-open");
  failures += run_case(
      *import.map,
      make_request(info, options.backend, "selfcheck-channel", Pose2d{20.0, 5.0, 0.0},
                   {rectangle("wall_lower", 6.0, 0.0, 16.0, 3.5),
                    rectangle("wall_upper", 6.0, 6.5, 16.0, 10.0)}),
      "selfcheck-channel");
  return failures == 0 ? 0 : 1;
}
