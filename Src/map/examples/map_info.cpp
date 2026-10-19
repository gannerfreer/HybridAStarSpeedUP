// planner_map_info：命令行地图导入 / 统计示例。
//
// 以显式指定的投影和原点加载 Lanelet2 OSM 地图，并打印导入元数据
// （对象计数、边界、验证统计、诊断）。可选执行局部窗口裁剪和路由查询。
//
// 退出码：0 成功，1 导入/查询失败，2 用法错误。

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

#include <planner/map/map.hpp>

namespace {

void print_usage(std::ostream& os) {
  os << "Usage: planner_map_info <map.osm> --origin-lat <deg> --origin-lon <deg>\n"
        "       [--projection utm|mercator|spherical_mercator] [--origin-ele <m>]\n"
        "       [--no-validation] [--window <min_x> <min_y> <max_x> <max_y>]\n"
        "       [--route <from_lanelet_id> <to_lanelet_id>] [--export-json <file>]\n"
        "\n"
        "Loads a Lanelet2 OSM map with an explicit projection origin and prints\n"
        "map statistics; optionally crops a local window, queries a route and/or\n"
        "exports the public JSON map snapshot (type \"map_snapshot\", schema v1).\n"
        "\n"
        "Example:\n"
        "  planner_map_info mapping_example.osm --origin-lat 49.003456 --origin-lon 8.424276\n";
}

bool parse_double(const std::string& text, double& out) {
  try {
    std::size_t pos = 0;
    const double value = std::stod(text, &pos);
    if (pos != text.size() || !std::isfinite(value)) {
      return false;
    }
    out = value;
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

}  // namespace

int main(int argc, char** argv) {
  const std::vector<std::string> args(argv + 1, argv + argc);
  if (args.empty() || args.front() == "--help" || args.front() == "-h") {
    print_usage(args.empty() ? std::cerr : std::cout);
    return args.empty() ? 2 : 0;
  }

  const std::string map_path = args.front();
  planner::map::MapLoadOptions options;
  std::optional<planner::map::Bounds2d> window;
  std::optional<std::pair<std::string, std::string>> route;
  std::optional<std::string> export_json;

  for (std::size_t i = 1; i < args.size(); ++i) {
    const std::string& arg = args[i];
    const auto next = [&](const char* flag) -> std::optional<std::string> {
      if (i + 1 >= args.size()) {
        std::cerr << "missing value for " << flag << "\n";
        return std::nullopt;
      }
      return args[++i];
    };
    if (arg == "--projection") {
      const auto value = next("--projection");
      if (!value) {
        return 2;
      }
      const auto kind = planner::map::projection_kind_from_string(*value);
      if (!kind) {
        std::cerr << "unknown projection '" << *value << "'\n";
        return 2;
      }
      options.projection = *kind;
    } else if (arg == "--origin-lat" || arg == "--origin-lon" || arg == "--origin-ele") {
      const auto value = next(arg.c_str());
      double parsed = 0.0;
      if (!value || !parse_double(*value, parsed)) {
        std::cerr << "invalid value for " << arg << "\n";
        return 2;
      }
      if (!options.origin) {
        options.origin = planner::map::GeoPoint{};
      }
      if (arg == "--origin-lat") {
        options.origin->lat_deg = parsed;
      } else if (arg == "--origin-lon") {
        options.origin->lon_deg = parsed;
      } else {
        options.origin->ele_m = parsed;
      }
    } else if (arg == "--no-validation") {
      options.validate_on_import = false;
    } else if (arg == "--window") {
      planner::map::Bounds2d bounds;
      double values[4] = {0.0, 0.0, 0.0, 0.0};
      for (double& value : values) {
        const auto text = next("--window");
        if (!text || !parse_double(*text, value)) {
          std::cerr << "--window expects four numbers: min_x min_y max_x max_y\n";
          return 2;
        }
      }
      bounds = planner::map::Bounds2d{values[0], values[1], values[2], values[3]};
      window = bounds;
    } else if (arg == "--route") {
      const auto from = next("--route");
      const auto to = next("--route");
      if (!from || !to) {
        std::cerr << "--route expects two lanelet ids\n";
        return 2;
      }
      route = std::make_pair(*from, *to);
    } else if (arg == "--export-json") {
      const auto value = next("--export-json");
      if (!value) {
        return 2;
      }
      export_json = *value;
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      print_usage(std::cerr);
      return 2;
    }
  }

  const auto import = planner::map::Map::import_osm_file(map_path, options);
  if (!import.ok()) {
    std::cerr << "import failed: " << import.reason << "\n";
    for (const auto& note : import.diagnostics) {
      std::cerr << "  " << note << "\n";
    }
    return 1;
  }

  const planner::map::Map& map = *import.map;
  const auto& info = map.info();
  std::cout << "map_id          : " << info.map_id << "\n";
  std::cout << "source          : " << info.source_path << " (" << info.source_bytes
            << " bytes)\n";
  std::cout << "map_version     : " << info.map_version << "\n";
  std::cout << "projection      : " << planner::map::to_string(info.projection)
            << " origin=(" << info.origin.lat_deg << ", " << info.origin.lon_deg << ", "
            << info.origin.ele_m << ")\n";
  std::cout << "traffic_rules   : " << info.traffic_rules_location << "/"
            << info.traffic_rules_participant << "\n";
  std::cout << "counts          : points=" << info.counts.points
            << " line_strings=" << info.counts.line_strings
            << " polygons=" << info.counts.polygons << " lanelets=" << info.counts.lanelets
            << " areas=" << info.counts.areas
            << " regulatory_elements=" << info.counts.regulatory_elements
            << " total=" << info.counts.total() << "\n";
  std::cout << "bounds_m        : [" << info.bounds_m.min_x_m << ", " << info.bounds_m.min_y_m
            << "] .. [" << info.bounds_m.max_x_m << ", " << info.bounds_m.max_y_m << "]\n";
  std::cout << "validation      : checks=" << info.validation.checks_run
            << " errors=" << info.validation.errors
            << " warnings=" << info.validation.warnings << "\n";
  if (!import.diagnostics.empty()) {
    std::cout << "diagnostics     : " << import.diagnostics.size() << " entr"
              << (import.diagnostics.size() == 1 ? "y" : "ies") << "\n";
    for (const auto& note : import.diagnostics) {
      std::cout << "  " << note << "\n";
    }
  }

  if (export_json) {
    try {
      const auto doc = map.to_snapshot_json();
      const std::string text = doc.dump();
      std::ofstream out(*export_json, std::ios::binary);
      if (!out) {
        std::cerr << "cannot open snapshot output file: " << *export_json << "\n";
        return 1;
      }
      out << text;
      out.close();
      if (!out) {
        std::cerr << "failed to write snapshot file: " << *export_json << "\n";
        return 1;
      }
      std::cout << "snapshot        : " << *export_json << " (" << text.size() << " bytes)\n";
    } catch (const std::exception& error) {
      std::cerr << "snapshot export failed: " << error.what() << "\n";
      return 1;
    }
  }

  if (window) {
    const auto crop = map.crop_to_bounds(*window);
    std::cout << "window          : [" << crop.window_m.min_x_m << ", " << crop.window_m.min_y_m
              << "] .. [" << crop.window_m.max_x_m << ", " << crop.window_m.max_y_m << "]\n";
    std::cout << "window_frame    : origin=(" << crop.window_frame.x_m << ", "
              << crop.window_frame.y_m << ") yaw=" << crop.window_frame.yaw_rad << "\n";
    std::cout << "crop            : lanelets=" << crop.lanelet_ids.size()
              << " paths=" << crop.paths.size() << " points=" << crop.points.size() << "\n";
    std::size_t clip_edges = 0;
    for (const auto& path : crop.paths) {
      clip_edges += (path.start_clip_edge ? 1U : 0U) + (path.end_clip_edge ? 1U : 0U);
    }
    std::cout << "crop_clip_edges : " << clip_edges << "\n";
    for (const auto& note : crop.diagnostics) {
      std::cout << "  " << note << "\n";
    }
    const planner::common::Point2d local{0.0, 0.0};
    const auto in_map = crop.local_to_map(local);
    const auto back = crop.map_to_local(in_map);
    std::cout << "window_to_map   : local(0,0) -> map(" << in_map.x_m << ", " << in_map.y_m
              << ") -> local(" << back.x_m << ", " << back.y_m << ")\n";
  }

  if (route) {
    const auto result = map.route(route->first, route->second);
    std::cout << "route           : status=" << planner::common::to_string(result.status)
              << " lanelets=" << result.lanelet_ids.size() << " length_m=" << result.length_m
              << "\n";
    if (!result.reason.empty()) {
      std::cout << "route_reason    : " << result.reason << "\n";
    }
    for (const auto& id : result.lanelet_ids) {
      std::cout << "  " << id << "\n";
    }
    if (result.status != planner::common::Status::kSuccess) {
      return 1;
    }
  }
  return 0;
}
