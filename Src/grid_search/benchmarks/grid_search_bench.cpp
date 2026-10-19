// planner_grid_search_bench --suite v1 --output-dir <dir> [options]
//
// X-B1 栅格基准运行器：对 grid_scenario_gen v1 生成的实例运行点到点与完整
// 距离场基准，验证独立 Python 参考（容差 1e-8 + 1e-8·|ref|），输出
// benchmark_metrics.csv（tools/README.md §7.2 固定列 + benchmark_tiers.md §5
// 栅格/距离场专有列 + 组件附加列）与 bench_summary.json。
//
// 口径（benchmark_tiers.md §2、§5、§6；组件开发计划 §7.3）：
//   * 点到点预算 5 s、距离场预算 30 s；预热 5 + 测量 30；
//   * 八邻域主组；四邻域对照组同协议、报告单列（不混排）；
//   * 参考代价/可达性与逐单元距离场参考由独立 Python 参考给出；
//   * 超时单列、不从成功率剔除；失败与不匹配保留并阻止该后端参与排名；
//   * 每个后端在独立子进程中运行（进程级隔离 RSS 与崩溃域），父进程合并。
//
// 容量档（--capacity）：2048²/4096² 单次冷启动；峰值 RSS 构成（输入/搜索/
// 输出，采样估计）与 conversion_ms 单列；扩重复判据见 benchmark_tiers.md
// §3.3（--expand-repeats 时自动评估）。
//
// 退出码：0 完成且全部测量结果与参考一致；1 存在不匹配或后端工作进程失败；
// 2 用法/IO 错误。部分产出在失败时同样保留。

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

#include <malloc.h>
#include <nlohmann/json.hpp>

#include "planner/grid_search/backend.hpp"
#include "planner/grid_search/distance_field.hpp"
#include "planner/grid_search/types.hpp"

#include "bench_util.hpp"
#include "scenario_gen.hpp"
#include "sha256.hpp"

namespace {

namespace fs = std::filesystem;
using nlohmann::json;
using planner::common::GridView;
using planner::common::Status;
using planner::grid_search::DistanceField;
using planner::grid_search::DistanceFieldRequest;
using planner::grid_search::DistanceFieldResult;
using planner::grid_search::GridCell;
using planner::grid_search::GridSearchBackend;
using planner::grid_search::GridSearchRequest;
using planner::grid_search::GridSearchResult;
using planner::grid_search::Neighborhood;
using planner::grid_search::bench::CsvWriter;
using planner::grid_search::bench::RssSampler;

// ---------------------------------------------------------------------------
// 命令行
// ---------------------------------------------------------------------------

struct Options {
  std::string suite{"v1"};
  std::string output_dir;
  std::string instances_path;
  std::string reference_path;
  std::string repo_root;
  std::vector<std::string> backends;
  std::vector<std::string> neighborhoods{"eight", "four"};
  int warmup{5};
  int trials{30};
  double p2p_budget_s{5.0};
  double df_budget_s{30.0};
  std::string cache_mode{"warm"};
  bool capacity{false};
  bool expand_repeats{false};
  std::vector<std::string> filter_classes;
  std::vector<std::string> filter_scales;
  std::vector<std::string> filter_seeds;
  bool worker{false};
  std::string worker_backend;
  std::string run_id;
};

[[noreturn]] void usage_error(const std::string& message) {
  std::cerr << "planner_grid_search_bench: " << message << "\n";
  std::cerr
      << "usage: planner_grid_search_bench --suite v1 --output-dir <dir>"
         " --instances <instances.json> --reference <reference.json>"
         " [--backends a,b] [--neighborhoods eight,four] [--warmup 5]"
         " [--trials 30] [--p2p-budget-s 5] [--df-budget-s 30]"
         " [--cache-mode warm|cold] [--capacity] [--expand-repeats]"
         " [--filter-class a,b] [--filter-scale 128] [--filter-seed 0]\n";
  std::exit(2);
}

Options parse_args(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string arg = argv[index];
    const auto value = [&](const char* name) -> std::string {
      if (index + 1 >= argc) {
        usage_error(std::string("missing value for ") + name);
      }
      return argv[++index];
    };
    if (arg == "--suite") {
      options.suite = value("--suite");
    } else if (arg == "--output-dir") {
      options.output_dir = value("--output-dir");
    } else if (arg == "--instances") {
      options.instances_path = value("--instances");
    } else if (arg == "--reference") {
      options.reference_path = value("--reference");
    } else if (arg == "--repo-root") {
      options.repo_root = value("--repo-root");
    } else if (arg == "--backends") {
      options.backends = planner::grid_search::bench::split_csv(value("--backends"));
    } else if (arg == "--neighborhoods") {
      options.neighborhoods =
          planner::grid_search::bench::split_csv(value("--neighborhoods"));
    } else if (arg == "--warmup") {
      options.warmup = std::stoi(value("--warmup"));
    } else if (arg == "--trials") {
      options.trials = std::stoi(value("--trials"));
    } else if (arg == "--p2p-budget-s") {
      options.p2p_budget_s = std::stod(value("--p2p-budget-s"));
    } else if (arg == "--df-budget-s") {
      options.df_budget_s = std::stod(value("--df-budget-s"));
    } else if (arg == "--cache-mode") {
      options.cache_mode = value("--cache-mode");
    } else if (arg == "--capacity") {
      options.capacity = true;
    } else if (arg == "--expand-repeats") {
      options.expand_repeats = true;
    } else if (arg == "--filter-class") {
      options.filter_classes =
          planner::grid_search::bench::split_csv(value("--filter-class"));
    } else if (arg == "--filter-scale") {
      options.filter_scales =
          planner::grid_search::bench::split_csv(value("--filter-scale"));
    } else if (arg == "--filter-seed") {
      options.filter_seeds =
          planner::grid_search::bench::split_csv(value("--filter-seed"));
    } else if (arg == "--worker") {
      options.worker = true;
    } else if (arg == "--backend") {
      options.worker_backend = value("--backend");
    } else {
      usage_error("unknown argument: " + arg);
    }
  }
  if (options.suite != "v1") {
    usage_error("unsupported suite: " + options.suite + " (only v1)");
  }
  if (options.output_dir.empty()) {
    usage_error("--output-dir is required");
  }
  if (options.instances_path.empty()) {
    usage_error("--instances is required");
  }
  if (options.reference_path.empty()) {
    usage_error("--reference is required");
  }
  if (options.cache_mode != "warm" && options.cache_mode != "cold") {
    usage_error("--cache-mode must be warm or cold");
  }
  if (options.capacity) {
    // 容量档固定单次协议：warmup 0 + trials 1；默认冷启动（§3.1）。
    options.warmup = 0;
    options.trials = 1;
    if (options.cache_mode == "warm") {
      options.cache_mode = "cold";
    }
  }
  for (const std::string& neighborhood : options.neighborhoods) {
    if (neighborhood != "eight" && neighborhood != "four") {
      usage_error("unknown neighborhood: " + neighborhood);
    }
  }
  if (options.warmup < 0 || options.trials < 1) {
    usage_error("--warmup must be >= 0 and --trials >= 1");
  }
  if (options.repo_root.empty()) {
    options.repo_root = fs::current_path().string();
  }
  options.run_id =
      planner::grid_search::bench::env_string("PLANNER_RUN_ID", "unmonitored");
  return options;
}

// ---------------------------------------------------------------------------
// 实例与参考加载
// ---------------------------------------------------------------------------

struct InstanceRecord {
  std::string instance_id;
  std::string scenario_class;
  std::string expected;
  int scale{0};
  int seed{0};
  int width{0};
  int height{0};
  double resolution_m{0.1};
  GridCell start{};
  GridCell goal{};
  GridCell distance_source{};
  std::string cells_sha256;
  std::int64_t size_bytes{0};
  bool archived{false};
  std::string cells_path;  // 解析后的绝对路径
};

struct NeighborhoodReference {
  bool present{false};
  bool reachable{false};
  std::optional<double> cost_m;
  std::int64_t unreachable_cells{0};
  double max_distance_m{0.0};
  std::string field_path;
  std::string field_sha256;
  std::optional<double> corner_cut_cost_m;
};

struct InstanceReference {
  std::map<std::string, NeighborhoodReference> neighborhoods;
};

bool matches_filter(const std::vector<std::string>& filter,
                    const std::string& value) {
  if (filter.empty()) {
    return true;
  }
  return std::find(filter.begin(), filter.end(), value) != filter.end();
}

std::vector<InstanceRecord> load_instances(const Options& options) {
  const auto text =
      planner::grid_search::bench::read_text_file(options.instances_path);
  if (!text.has_value()) {
    throw std::runtime_error("cannot read instances file: " +
                             options.instances_path);
  }
  const json document = json::parse(*text);
  const fs::path instances_dir =
      fs::path(options.instances_path).parent_path();
  std::vector<InstanceRecord> records;
  for (const auto& entry : document.at("instances")) {
    InstanceRecord record;
    record.instance_id = entry.at("instance_id").get<std::string>();
    record.scenario_class = entry.at("class").get<std::string>();
    record.expected = entry.at("expected").get<std::string>();
    record.scale = entry.at("scale").get<int>();
    record.seed = entry.at("seed").get<int>();
    record.width = entry.at("width").get<int>();
    record.height = entry.at("height").get<int>();
    record.resolution_m = entry.at("resolution_m").get<double>();
    record.start = GridCell{entry.at("start")[0].get<int>(),
                            entry.at("start")[1].get<int>()};
    record.goal = GridCell{entry.at("goal")[0].get<int>(),
                           entry.at("goal")[1].get<int>()};
    record.distance_source = GridCell{
        entry.at("distance_source")[0].get<int>(),
        entry.at("distance_source")[1].get<int>()};
    const auto& cells = entry.at("cells");
    record.cells_sha256 = cells.at("sha256").get<std::string>();
    record.size_bytes = cells.at("size_bytes").get<std::int64_t>();
    record.archived = cells.value("archived", false);
    if (record.archived) {
      record.cells_path =
          (fs::path(options.repo_root) /
           cells.at("archive_relative_path").get<std::string>())
              .string();
    } else {
      record.cells_path =
          (instances_dir / cells.at("local_path").get<std::string>()).string();
    }
    if (!matches_filter(options.filter_classes, record.scenario_class) ||
        !matches_filter(options.filter_scales, std::to_string(record.scale)) ||
        !matches_filter(options.filter_seeds, std::to_string(record.seed))) {
      continue;
    }
    records.push_back(record);
  }
  return records;
}

std::map<std::string, InstanceReference> load_reference(const Options& options) {
  const auto text =
      planner::grid_search::bench::read_text_file(options.reference_path);
  if (!text.has_value()) {
    throw std::runtime_error("cannot read reference file: " +
                             options.reference_path);
  }
  const json document = json::parse(*text);
  const fs::path reference_dir =
      fs::path(options.reference_path).parent_path();
  std::map<std::string, InstanceReference> references;
  for (const auto& [instance_id, entry] :
       document.at("instances").items()) {
    InstanceReference reference;
    for (const auto& [neighborhood, data] : entry.items()) {
      if (neighborhood == "eight_corner_cut_allowed") {
        NeighborhoodReference corner;
        corner.present = true;
        corner.reachable = data.value("reachable", false);
        if (data.contains("cost_m") && !data.at("cost_m").is_null()) {
          corner.cost_m = data.at("cost_m").get<double>();
        }
        reference.neighborhoods["eight_corner_cut_allowed"] = corner;
        continue;
      }
      if (neighborhood != "eight" && neighborhood != "four") {
        continue;  // 跳过实例元数据（width/height/resolution_m/...）
      }
      NeighborhoodReference record;
      record.present = true;
      record.reachable = data.value("reachable", false);
      if (data.contains("cost_m") && !data.at("cost_m").is_null()) {
        record.cost_m = data.at("cost_m").get<double>();
      }
      record.unreachable_cells =
          data.value("unreachable_cells", static_cast<std::int64_t>(0));
      record.max_distance_m = data.value("max_distance_m", 0.0);
      record.field_path =
          (reference_dir / data.value("field_path", std::string())).string();
      record.field_sha256 = data.value("field_sha256", std::string());
      reference.neighborhoods[neighborhood] = record;
    }
    references[instance_id] = reference;
  }
  return references;
}

GridView make_grid(const InstanceRecord& record,
                   const std::vector<std::uint8_t>& packed) {
  GridView grid;
  grid.map_id = record.instance_id;
  grid.map_version = record.cells_sha256;
  grid.width = record.width;
  grid.height = record.height;
  grid.resolution_m = record.resolution_m;
  grid.origin = planner::common::Pose2d{};
  grid.cells.resize(packed.size());
  for (std::size_t index = 0; index < packed.size(); ++index) {
    grid.cells[index] = packed[index] == 0 ? planner::common::kCellFree
                                           : planner::common::kCellOccupied;
  }
  return grid;
}

std::optional<std::vector<double>> load_reference_field(
    const std::string& path, std::size_t cells, const std::string& sha256) {
  const auto bytes = planner::grid_search::bench::read_binary_file(path);
  if (!bytes.has_value() || bytes->size() != cells * sizeof(double)) {
    return std::nullopt;
  }
  if (!sha256.empty()) {
    bool ok = false;
    const std::string actual =
        planner::grid_search::bench::sha256_file_hex(path, &ok);
    if (!ok || actual != sha256) {
      return std::nullopt;
    }
  }
  std::vector<double> field(cells);
  std::memcpy(field.data(), bytes->data(), cells * sizeof(double));
  return field;
}

// ---------------------------------------------------------------------------
// 指标列（tools/README.md §7.2 固定列 + benchmark_tiers.md §5 专有列 + 附加）
// ---------------------------------------------------------------------------

std::vector<std::string> column_names() {
  return {
      // §9 固定列
      "run_id", "case_id", "repeat_id", "cache_mode", "seed", "status",
      "failure_reason", "preprocess_ms", "wall_ms", "cpu_ms", "peak_rss_bytes",
      "expanded_nodes", "collision_checks", "path_length_m", "gear_changes",
      "endpoint_position_error_m", "endpoint_yaw_error_rad", "valid",
      // §5 栅格搜索专有列
      "instance_id", "scale", "class", "neighborhood", "pair_id", "budget_s",
      "timeout",
      // §5 距离场专有列
      "source_id", "unreachable_cells", "max_distance_m", "reference_cost",
      // 组件附加列
      "backend", "query", "single_list", "reference_reachable", "cost_m",
      "cost_error_m", "path_cells", "field_cells", "field_mismatch_cells",
      "field_max_abs_error_m", "reference_max_distance_m",
      "reference_unreachable_cells", "conversion_ms", "postprocess_ms",
      "rss_input_bytes", "rss_search_bytes", "rss_output_bytes",
      "rss_hwm_bytes"};
}

struct TrialRecord {
  std::string instance_id;
  std::string scenario_class;
  std::string neighborhood;
  std::string backend;
  std::string query;  // point_to_point / distance_field
  int scale{0};
  int seed{0};
  int repeat_id{0};
  std::string cache_mode;
  std::string status;
  std::string failure_reason;
  std::optional<double> preprocess_ms;
  std::optional<double> wall_ms;
  std::optional<double> cpu_ms;
  std::optional<std::int64_t> peak_rss_bytes;
  std::optional<std::int64_t> expanded_nodes;
  std::optional<double> path_length_m;
  std::optional<double> cost_m;
  std::optional<double> cost_error_m;
  std::optional<std::int64_t> path_cells;
  bool reference_reachable{false};
  std::optional<std::int64_t> unreachable_cells;
  std::optional<double> max_distance_m;
  std::optional<double> reference_cost;
  std::optional<double> reference_max_distance_m;
  std::optional<std::int64_t> reference_unreachable_cells;
  std::optional<std::int64_t> field_cells;
  std::optional<std::int64_t> field_mismatch_cells;
  std::optional<double> field_max_abs_error_m;
  std::optional<double> conversion_ms;
  std::optional<double> postprocess_ms;
  std::optional<std::int64_t> rss_input_bytes;
  std::optional<std::int64_t> rss_search_bytes;
  std::optional<std::int64_t> rss_output_bytes;
  std::optional<std::int64_t> rss_hwm_bytes;
  double budget_s{5.0};
  bool timeout{false};
  bool valid{false};
  bool single_list{false};
};

std::vector<std::string> row_fields(const TrialRecord& record,
                                    const std::string& run_id) {
  using planner::grid_search::bench::fmt_double;
  using planner::grid_search::bench::fmt_int;
  using planner::grid_search::bench::opt_double;
  using planner::grid_search::bench::opt_int;
  const std::string case_id = record.instance_id + "|" + record.neighborhood +
                              "|" + record.backend + "|" + record.query;
  const bool is_df = record.query == "distance_field";
  return {
      run_id,
      case_id,
      fmt_int(record.repeat_id),
      record.cache_mode,
      fmt_int(record.seed),
      record.status,
      record.failure_reason,
      opt_double(record.preprocess_ms, 3),
      opt_double(record.wall_ms, 3),
      opt_double(record.cpu_ms, 3),
      opt_int(record.peak_rss_bytes),
      opt_int(record.expanded_nodes),
      "",  // collision_checks：所有后端未暴露（留空，不用 0 代表未测量）
      opt_double(record.path_length_m, 6),
      "",  // gear_changes：不适用
      "",  // endpoint_position_error_m：不适用
      "",  // endpoint_yaw_error_rad：不适用
      record.valid ? "1" : "0",
      record.instance_id,
      fmt_int(record.scale),
      record.scenario_class,
      record.neighborhood,
      "0",  // pair_id（每实例 1 对固定起终点）
      fmt_double(record.budget_s, 1),
      record.timeout ? "1" : "0",
      is_df ? "0" : "",  // source_id（每实例 1 个固定距离场源点）
      opt_int(record.unreachable_cells),
      opt_double(record.max_distance_m, 6),
      opt_double(record.reference_cost, 6),
      record.backend,
      record.query,
      record.single_list ? "1" : "0",
      record.reference_reachable ? "true" : "false",
      opt_double(record.cost_m, 6),
      opt_double(record.cost_error_m, 9),
      opt_int(record.path_cells),
      opt_int(record.field_cells),
      opt_int(record.field_mismatch_cells),
      opt_double(record.field_max_abs_error_m, 9),
      opt_double(record.reference_max_distance_m, 6),
      opt_int(record.reference_unreachable_cells),
      opt_double(record.conversion_ms, 3),
      opt_double(record.postprocess_ms, 3),
      opt_int(record.rss_input_bytes),
      opt_int(record.rss_search_bytes),
      opt_int(record.rss_output_bytes),
      opt_int(record.rss_hwm_bytes)};
}

// ---------------------------------------------------------------------------
// 结果校验（独立 Python 参考；容差 1e-8 + 1e-8·|ref|）
// ---------------------------------------------------------------------------

struct CheckOutcome {
  bool valid{false};
  std::string failure_reason;
  std::optional<double> cost_error_m;
  std::optional<std::int64_t> field_cells;
  std::optional<std::int64_t> field_mismatch_cells;
  std::optional<double> field_max_abs_error_m;
};

CheckOutcome check_point_to_point(const GridView& grid,
                                  const GridSearchRequest& request,
                                  const GridSearchResult& result,
                                  bool corner_variant,
                                  const NeighborhoodReference& reference) {
  CheckOutcome outcome;
  if (result.status == Status::kTimeout) {
    outcome.failure_reason = "timeout";
    return outcome;
  }
  if (result.status == Status::kCancelled) {
    outcome.failure_reason = "cancelled";
    return outcome;
  }
  if (!reference.present) {
    outcome.failure_reason = "missing_reference";
    return outcome;
  }
  if (reference.reachable) {
    if (result.status != Status::kSuccess) {
      outcome.failure_reason =
          "expected_success_got_" + planner::common::to_string(result.status);
      return outcome;
    }
    const std::string path_problem =
        planner::grid_search::validate_path(grid, request, result.path,
                                            !corner_variant);
    if (!path_problem.empty()) {
      outcome.failure_reason = "path_invalid: " + path_problem;
      return outcome;
    }
    const double recomputed =
        planner::grid_search::path_cost_m(grid, request.neighborhood,
                                          result.path);
    if (std::fabs(recomputed - result.cost_m) >
        1e-9 + 1e-9 * std::fabs(recomputed)) {
      outcome.failure_reason = "cost_inconsistent_with_path";
      return outcome;
    }
    if (!reference.cost_m.has_value()) {
      outcome.failure_reason = "missing_reference_cost";
      return outcome;
    }
    outcome.cost_error_m = std::fabs(result.cost_m - *reference.cost_m);
    if (!planner::grid_search::costs_match(result.cost_m, *reference.cost_m)) {
      outcome.failure_reason = "cost_mismatch";
      return outcome;
    }
  } else {
    if (result.status != Status::kNoPath) {
      outcome.failure_reason =
          "expected_no_path_got_" + planner::common::to_string(result.status);
      return outcome;
    }
  }
  outcome.valid = true;
  return outcome;
}

CheckOutcome check_distance_field(const DistanceFieldResult& result,
                                  const NeighborhoodReference& reference,
                                  const std::vector<double>& reference_field) {
  CheckOutcome outcome;
  if (result.status == Status::kTimeout) {
    outcome.failure_reason = "timeout";
    return outcome;
  }
  if (result.status == Status::kCancelled) {
    outcome.failure_reason = "cancelled";
    return outcome;
  }
  if (!reference.present) {
    outcome.failure_reason = "missing_reference";
    return outcome;
  }
  if (result.status != Status::kSuccess) {
    outcome.failure_reason =
        "distance_field_failed_" + planner::common::to_string(result.status);
    return outcome;
  }
  const std::size_t expected_cells =
      static_cast<std::size_t>(result.field.width) *
      static_cast<std::size_t>(result.field.height);
  if (reference_field.empty()) {
    outcome.failure_reason = "missing_reference_field";
    return outcome;
  }
  if (result.field.width <= 0 || result.field.height <= 0 ||
      result.field.distances.size() != expected_cells ||
      reference_field.size() != expected_cells) {
    outcome.failure_reason = "distance_field_size_mismatch";
    return outcome;
  }
  std::int64_t mismatches = 0;
  double max_abs_error = 0.0;
  for (std::size_t index = 0; index < expected_cells; ++index) {
    const double actual = result.field.distances[index];
    const double expected = reference_field[index];
    const bool actual_inf = !std::isfinite(actual);
    const bool expected_inf = !std::isfinite(expected);
    if (actual_inf || expected_inf) {
      if (actual_inf != expected_inf) {
        ++mismatches;
      }
      continue;
    }
    const double error = std::fabs(actual - expected);
    if (error > max_abs_error) {
      max_abs_error = error;
    }
    if (!planner::grid_search::costs_match(actual, expected)) {
      ++mismatches;
    }
  }
  outcome.field_cells = static_cast<std::int64_t>(expected_cells);
  outcome.field_mismatch_cells = mismatches;
  outcome.field_max_abs_error_m = max_abs_error;
  if (mismatches != 0) {
    outcome.failure_reason = "distance_field_mismatch";
    return outcome;
  }
  outcome.valid = true;
  return outcome;
}

// ---------------------------------------------------------------------------
// 后端元数据（单列/穿角变体/展开计数暴露；依据 provenance.md 登记）
// ---------------------------------------------------------------------------

bool is_single_list(const std::string& backend) {
  return backend == "jps3d_jps" || backend == "justinhj_astar_nopool";
}

bool is_corner_cut_variant(const std::string& backend) {
  return backend == "jps3d_jps";  // 原生允许对角穿角（单列）
}

bool expanded_nodes_exposed(const std::string& backend) {
  return backend != "vss2sn_astar" && backend != "vss2sn_dijkstra";
}

// ---------------------------------------------------------------------------
// 聚合
// ---------------------------------------------------------------------------

double percentile_nearest_rank(std::vector<double> values, double fraction) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  std::size_t index = static_cast<std::size_t>(
      std::ceil(fraction * static_cast<double>(values.size()))) - 1;
  if (index >= values.size()) {
    index = values.size() - 1;
  }
  return values[index];
}

struct GroupStats {
  std::string neighborhood;
  std::string query;
  int cases{0};
  std::int64_t trials{0};
  std::int64_t success{0};
  std::int64_t no_path{0};
  std::int64_t timeout{0};
  std::int64_t other_status{0};
  std::int64_t mismatches{0};
  std::int64_t warmup_mismatches{0};
  std::int64_t expanded_repeats{0};
  std::int64_t expanded_cases{0};
  std::vector<std::string> expansion_notes;
  std::int64_t peak_rss_max{0};
  std::int64_t expanded_nodes_sum{0};
  std::int64_t expanded_nodes_cases{0};
  std::vector<double> wall_ms;
  std::vector<double> cpu_ms;
  std::vector<std::string> mismatch_samples;

  void add_status(const std::string& status) {
    if (status == "success") {
      ++success;
    } else if (status == "no_path") {
      ++no_path;
    } else if (status == "timeout") {
      ++timeout;
    } else {
      ++other_status;
    }
  }

  json to_json() const {
    json out;
    out["cases"] = cases;
    out["trials"] = trials;
    out["success"] = success;
    out["no_path"] = no_path;
    out["timeout"] = timeout;
    out["other_status"] = other_status;
    out["mismatches"] = mismatches;
    out["warmup_mismatches"] = warmup_mismatches;
    if (!wall_ms.empty()) {
      out["wall_ms_p50"] = percentile_nearest_rank(wall_ms, 0.50);
      out["wall_ms_p95"] = percentile_nearest_rank(wall_ms, 0.95);
    }
    if (!cpu_ms.empty()) {
      out["cpu_ms_p50"] = percentile_nearest_rank(cpu_ms, 0.50);
      out["cpu_ms_p95"] = percentile_nearest_rank(cpu_ms, 0.95);
    }
    out["peak_rss_max_bytes"] = peak_rss_max;
    if (expanded_nodes_cases > 0) {
      out["expanded_nodes_mean"] =
          static_cast<double>(expanded_nodes_sum) /
          static_cast<double>(expanded_nodes_cases);
    }
    if (expanded_cases > 0) {
      out["expanded_cases"] = expanded_cases;
      out["expanded_repeats"] = expanded_repeats;
    }
    if (!expansion_notes.empty()) {
      out["expansion_notes"] = expansion_notes;
    }
    if (!mismatch_samples.empty()) {
      out["mismatch_samples"] = mismatch_samples;
    }
    return out;
  }
};

// ---------------------------------------------------------------------------
// 工作进程（单后端）
// ---------------------------------------------------------------------------

class BackendWorker {
 public:
  BackendWorker(const Options& options, std::vector<InstanceRecord> instances,
                std::map<std::string, InstanceReference> references)
      : options_(options),
        instances_(std::move(instances)),
        references_(std::move(references)),
        backend_name_(options.worker_backend) {}

  int run() {
    const fs::path output_dir = options_.output_dir;
    fs::create_directories(output_dir);
    CsvWriter writer((output_dir / "metrics.csv").string());
    if (!writer.open_ok()) {
      throw std::runtime_error("cannot open metrics.csv in " + output_dir.string());
    }
    writer.write_header(column_names());

    auto backend = planner::grid_search::make_backend(backend_name_);
    if (backend == nullptr) {
      throw std::runtime_error("unknown or disabled backend: " + backend_name_);
    }
    summary_["backend"] = backend_name_;
    summary_["distance_field"] = backend->supports_distance_field();
    summary_["single_list"] = is_single_list(backend_name_);
    summary_["corner_cut_variant"] = is_corner_cut_variant(backend_name_);
    summary_["expanded_nodes_exposed"] = expanded_nodes_exposed(backend_name_);
    summary_["cache_mode"] = options_.cache_mode;
    summary_["capacity"] = options_.capacity;

    baseline_rss_ = planner::grid_search::bench::vm_rss_bytes();

    // 四邻域能力探测（jps3d_jps 原生仅八邻域 → invalid_input）。
    bool four_supported = true;
    if (!instances_.empty() &&
        std::find(options_.neighborhoods.begin(), options_.neighborhoods.end(),
                  "four") != options_.neighborhoods.end()) {
      const auto packed =
          planner::grid_search::bench::read_binary_file(instances_.front().cells_path);
      if (packed.has_value()) {
        const GridView grid = make_grid(instances_.front(), *packed);
        GridSearchRequest request;
        request.grid = &grid;
        request.start = instances_.front().start;
        request.goal = instances_.front().goal;
        request.neighborhood = Neighborhood::kFour;
        request.time_budget_s = 1.0;
        const GridSearchResult probe = backend->search(request);
        four_supported = probe.status != Status::kInvalidInput;
      }
    }
    summary_["four_neighborhood_supported"] = four_supported;
    if (!four_supported) {
      summary_["notes"] = json::array(
          {"four-neighborhood control skipped: backend reports invalid_input "
           "(native eight-neighborhood only; single-list)"});
    }

    int case_index = 0;
    for (const InstanceRecord& record : instances_) {
      load_grid(record);
      const bool capacity = options_.capacity;
      const std::int64_t rss_after_load = planner::grid_search::bench::vm_rss_bytes();
      instance_rss_input_ = std::max<std::int64_t>(
          0, rss_after_load - baseline_rss_);
      for (const std::string& neighborhood_name : options_.neighborhoods) {
        if (neighborhood_name == "four" && !four_supported) {
          continue;
        }
        const Neighborhood neighborhood =
            neighborhood_name == "eight" ? Neighborhood::kEight
                                          : Neighborhood::kFour;
        // 点到点。
        run_group(writer, record, neighborhood, neighborhood_name,
                  "point_to_point",
                  options_.p2p_budget_s, backend.get(), capacity,
                  &case_index);
        // 距离场（仅支持的后端）。
        if (backend->supports_distance_field()) {
          run_group(writer, record, neighborhood, neighborhood_name,
                    "distance_field",
                    options_.df_budget_s, backend.get(), capacity,
                    &case_index);
        }
      }
      malloc_trim(0);  // 容量档 RSS 构成按实例单列（采样估计）
    }

    json groups = json::object();
    for (const auto& [key, stats] : group_stats_) {
      groups[key] = stats.to_json();
    }
    summary_["instances"] = static_cast<int>(instances_.size());
    summary_["cases"] = case_index;
    summary_["groups"] = groups;
    summary_["baseline_rss_bytes"] = baseline_rss_;
    planner::grid_search::bench::write_text_file(
        (output_dir / "summary.json").string(), summary_.dump(2) + "\n");
    return 0;
  }

 private:
  void load_grid(const InstanceRecord& record) {
    const double t0 = planner::grid_search::bench::wall_now_s();
    const auto packed =
        planner::grid_search::bench::read_binary_file(record.cells_path);
    if (!packed.has_value()) {
      throw std::runtime_error("cannot read instance cells: " +
                               record.cells_path);
    }
    const std::string actual_sha =
        planner::grid_search::bench::sha256_hex(*packed);
    if (actual_sha != record.cells_sha256) {
      throw std::runtime_error("instance SHA256 mismatch for " +
                               record.instance_id + " (" + record.cells_path +
                               ")");
    }
    current_grid_ = make_grid(record, *packed);
    conversion_ms_ = (planner::grid_search::bench::wall_now_s() - t0) * 1000.0;
    current_record_ = &record;
  }

  void run_group(CsvWriter& writer, const InstanceRecord& record,
                 Neighborhood neighborhood, const std::string& neighborhood_name,
                 const std::string& query, double budget_s,
                 GridSearchBackend* backend, bool capacity, int* case_index) {
    const std::string key = neighborhood_name + "|" + query;
    GroupStats& stats = group_stats_[key];
    stats.neighborhood = neighborhood_name;
    stats.query = query;
    ++stats.cases;
    ++(*case_index);

    // 距离场参考（逐单元；加载一次，组内复用）。
    std::vector<double> reference_field;
    NeighborhoodReference reference;
    const InstanceReference& instance_reference =
        references_.at(record.instance_id);
    if (query == "point_to_point") {
      const auto it = instance_reference.neighborhoods.find(neighborhood_name);
      if (it != instance_reference.neighborhoods.end()) {
        reference = it->second;
        if (is_corner_cut_variant(backend_name_)) {
          const auto corner =
              instance_reference.neighborhoods.find(
                  "eight_corner_cut_allowed");
          if (corner != instance_reference.neighborhoods.end()) {
            reference.cost_m = corner->second.cost_m;
            reference.reachable = corner->second.reachable;
          } else {
            reference.cost_m.reset();
          }
        }
      }
    } else {
      const auto it = instance_reference.neighborhoods.find(neighborhood_name);
      if (it != instance_reference.neighborhoods.end()) {
        reference = it->second;
        if (!reference.field_path.empty()) {
          const auto field = load_reference_field(
              reference.field_path,
              static_cast<std::size_t>(record.width) *
                  static_cast<std::size_t>(record.height),
              reference.field_sha256);
          if (field.has_value()) {
            reference_field = *field;
          }
        }
      }
    }

    // 预热。
    for (int warm = 0; warm < options_.warmup; ++warm) {
      const TrialOutcome outcome =
          run_trial(record, neighborhood, query, budget_s, backend,
                    reference, reference_field, /*record=*/false, warm,
                    capacity);
      if (!outcome.valid && !outcome.timed_out) {
        ++stats.warmup_mismatches;
        record_mismatch(stats, record, neighborhood_name, query, -1,
                        outcome.failure_reason);
      }
    }

    // 测量。
    int repeats = options_.trials;
    for (int repeat = 0; repeat < repeats; ++repeat) {
      const TrialOutcome outcome =
          run_trial(record, neighborhood, query, budget_s, backend,
                    reference, reference_field, /*record=*/true, repeat,
                    capacity);
      ++stats.trials;
      stats.add_status(outcome.status_name);
      if (outcome.timed_out) {
        // timeout 已计入状态；不匹配计数单列。
      } else if (!outcome.valid) {
        ++stats.mismatches;
        record_mismatch(stats, record, neighborhood_name, query, repeat,
                        outcome.failure_reason);
      }
      if (outcome.peak_rss_bytes > stats.peak_rss_max) {
        stats.peak_rss_max = outcome.peak_rss_bytes;
      }
      if (outcome.wall_ms.has_value()) {
        stats.wall_ms.push_back(*outcome.wall_ms);
      }
      if (outcome.cpu_ms.has_value()) {
        stats.cpu_ms.push_back(*outcome.cpu_ms);
      }
      if (outcome.expanded_nodes.has_value()) {
        stats.expanded_nodes_sum += *outcome.expanded_nodes;
        ++stats.expanded_nodes_cases;
      }
      if (outcome.record != nullptr) {
        writer.write_row(row_fields(*outcome.record, options_.run_id));
        writer.flush();
      }

      // 容量档扩重复判据（benchmark_tiers.md §3.3）：单次完成后仅当
      // (a) success、(b) 墙钟 ≤ 50% 预算、(c) 峰值 RSS ≤ 50% 内存、
      // (d) 预估总时长 ≤ 2 h 时才扩为“预热 1 + 测量 5”。
      if (capacity && options_.expand_repeats && repeats == 1 &&
          outcome.valid) {
        const std::string reason =
            expansion_block_reason(record, outcome, budget_s, *case_index);
        if (reason.empty()) {
          ++stats.expanded_cases;
          stats.expanded_repeats += 5;
          stats.expansion_notes.push_back(
              record.instance_id + "|" + neighborhood_name + "|" + query +
              "|expanded");
          for (int warm = 0; warm < 1; ++warm) {
            run_trial(record, neighborhood, query, budget_s, backend,
                      reference, reference_field, /*record=*/false, -2,
                      capacity);
          }
          repeats = 6;
        } else {
          stats.expansion_notes.push_back(
              record.instance_id + "|" + neighborhood_name + "|" + query +
              "|single_run|" + reason);
        }
      }
    }
  }

  struct TrialOutcome {
    bool valid{false};
    bool timed_out{false};
    std::string failure_reason;
    std::string status_name;
    std::optional<double> wall_ms;
    std::optional<double> cpu_ms;
    std::int64_t peak_rss_bytes{0};
    std::optional<std::int64_t> expanded_nodes;
    TrialRecord* record{nullptr};
    std::unique_ptr<TrialRecord> owned_record;
  };

  std::string expansion_block_reason(const InstanceRecord& record,
                                     const TrialOutcome& outcome,
                                     double budget_s, int case_index) const {
    if (!outcome.wall_ms.has_value()) {
      return "no_wall_time";
    }
    if (*outcome.wall_ms > 500.0 * budget_s) {
      return "wall_over_50_percent_budget";
    }
    const std::int64_t total_memory = total_memory_bytes();
    if (outcome.peak_rss_bytes > total_memory / 2) {
      return "peak_rss_over_50_percent_memory";
    }
    const int remaining_cases =
        static_cast<int>(instances_.size()) *
            static_cast<int>(options_.neighborhoods.size()) *
            (options_.capacity ? 2 : 1) -
        case_index;
    const double estimated =
        6.0 * (*outcome.wall_ms / 1000.0) *
        static_cast<double>(std::max(1, remaining_cases));
    if (estimated > 7200.0) {
      return "estimated_group_over_2h";
    }
    (void)record;
    return std::string();
  }

  std::int64_t total_memory_bytes() const {
    const auto text = planner::grid_search::bench::read_text_file("/proc/meminfo");
    if (!text.has_value()) {
      return 0;
    }
    std::istringstream stream(*text);
    std::string key;
    std::int64_t value = 0;
    std::string unit;
    while (stream >> key >> value >> unit) {
      if (key == "MemTotal:") {
        return value * 1024;
      }
    }
    return 0;
  }

  TrialOutcome run_trial(const InstanceRecord& record, Neighborhood neighborhood,
                         const std::string& query, double budget_s,
                         GridSearchBackend* backend,
                         const NeighborhoodReference& reference,
                         const std::vector<double>& reference_field,
                         bool record_row, int repeat_id, bool capacity) {
    TrialOutcome outcome;
    std::unique_ptr<GridSearchBackend> owned_backend;
    const double preprocess_begin = planner::grid_search::bench::wall_now_s();
    if (options_.cache_mode == "cold") {
      owned_backend = planner::grid_search::make_backend(backend_name_);
      backend = owned_backend.get();
    }
    const double preprocess_ms =
        (planner::grid_search::bench::wall_now_s() - preprocess_begin) * 1000.0;

    const bool is_df = query == "distance_field";
    const std::int64_t rss_before = planner::grid_search::bench::vm_rss_bytes();
    RssSampler sampler(1);
    sampler.start();
    const double wall_begin = planner::grid_search::bench::wall_now_s();
    const double cpu_begin = planner::grid_search::bench::cpu_now_s();

    std::string status_name;
    std::string failure_reason;
    CheckOutcome check;
    std::optional<double> cost_m;
    std::optional<std::int64_t> path_cells;
    std::optional<std::int64_t> unreachable_cells;
    std::optional<double> max_distance_m;
    std::optional<std::int64_t> expanded_nodes;
    std::optional<double> reference_cost;
    std::optional<double> reference_max_distance_m;
    std::optional<std::int64_t> reference_unreachable_cells;
    bool reference_reachable = reference.reachable;
    double check_ms = 0.0;

    if (!is_df) {
      GridSearchRequest request;
      request.grid = &current_grid_;
      request.start = record.start;
      request.goal = record.goal;
      request.neighborhood = neighborhood;
      request.time_budget_s = budget_s;
      const GridSearchResult result = backend->search(request);
      const double wall_end = planner::grid_search::bench::wall_now_s();
      const double cpu_end = planner::grid_search::bench::cpu_now_s();
      const std::int64_t peak = sampler.stop();
      const double check_begin = planner::grid_search::bench::wall_now_s();
      outcome.wall_ms = (wall_end - wall_begin) * 1000.0;
      outcome.cpu_ms = (cpu_end - cpu_begin) * 1000.0;
      outcome.peak_rss_bytes = peak;
      status_name = planner::common::to_string(result.status);
      check = check_point_to_point(current_grid_, request, result,
                                   is_corner_cut_variant(backend_name_),
                                   reference);
      check_ms = (planner::grid_search::bench::wall_now_s() - check_begin) * 1000.0;
      failure_reason = check.failure_reason;
      if (result.status == Status::kSuccess) {
        cost_m = result.cost_m;
        path_cells = static_cast<std::int64_t>(result.path.size());
      }
      if (expanded_nodes_exposed(backend_name_)) {
        expanded_nodes = result.stats.expanded_nodes;
      }
      reference_cost = reference.cost_m;  // jps3d_jps 已切换为允许穿角参考
    } else {
      DistanceFieldRequest request;
      request.grid = &current_grid_;
      request.source = record.distance_source;
      request.neighborhood = neighborhood;
      request.time_budget_s = budget_s;
      const DistanceFieldResult result = backend->distance_field(request);
      const double wall_end = planner::grid_search::bench::wall_now_s();
      const double cpu_end = planner::grid_search::bench::cpu_now_s();
      const std::int64_t peak = sampler.stop();
      const double check_begin = planner::grid_search::bench::wall_now_s();
      outcome.wall_ms = (wall_end - wall_begin) * 1000.0;
      outcome.cpu_ms = (cpu_end - cpu_begin) * 1000.0;
      outcome.peak_rss_bytes = peak;
      status_name = planner::common::to_string(result.status);
      if (result.status == Status::kSuccess) {
        check = check_distance_field(result, reference, reference_field);
        unreachable_cells = result.field.unreachable_count();
        max_distance_m = result.field.max_reachable_m();
      } else {
        check.failure_reason = "distance_field_failed_" + status_name;
      }
      check_ms = (planner::grid_search::bench::wall_now_s() - check_begin) * 1000.0;
      failure_reason = check.failure_reason;
      reference_cost = reference.max_distance_m;
      reference_max_distance_m = reference.max_distance_m;
      reference_unreachable_cells = reference.unreachable_cells;
      if (expanded_nodes_exposed(backend_name_)) {
        expanded_nodes = result.stats.expanded_nodes;
      }
    }
    outcome.valid = check.valid;
    outcome.timed_out = status_name == "timeout";
    outcome.failure_reason = failure_reason;
    outcome.status_name = status_name;
    outcome.expanded_nodes = expanded_nodes;

    if (record_row) {
      outcome.owned_record = std::make_unique<TrialRecord>();
      TrialRecord& row = *outcome.owned_record;
      row.instance_id = record.instance_id;
      row.scenario_class = record.scenario_class;
      row.neighborhood =
          neighborhood == Neighborhood::kEight ? "eight" : "four";
      row.backend = backend_name_;
      row.query = query;
      row.scale = record.scale;
      row.seed = record.seed;
      row.repeat_id = repeat_id;
      row.cache_mode = options_.cache_mode;
      row.status = status_name;
      row.failure_reason = failure_reason;
      if (options_.cache_mode == "cold") {
        row.preprocess_ms = preprocess_ms;
      }
      row.wall_ms = outcome.wall_ms;
      row.cpu_ms = outcome.cpu_ms;
      row.peak_rss_bytes = outcome.peak_rss_bytes;
      row.expanded_nodes = expanded_nodes;
      row.cost_m = cost_m;
      row.cost_error_m = check.cost_error_m;
      row.path_cells = path_cells;
      if (cost_m.has_value()) {
        row.path_length_m = cost_m;
      }
      row.reference_reachable = reference_reachable;
      row.unreachable_cells = unreachable_cells;
      row.max_distance_m = max_distance_m;
      row.reference_cost = reference_cost;
      row.reference_max_distance_m = reference_max_distance_m;
      row.reference_unreachable_cells = reference_unreachable_cells;
      row.field_cells = check.field_cells;
      row.field_mismatch_cells = check.field_mismatch_cells;
      row.field_max_abs_error_m = check.field_max_abs_error_m;
      row.conversion_ms = conversion_ms_;
      row.postprocess_ms = check_ms;
      row.budget_s = budget_s;
      row.timeout = outcome.timed_out;
      row.valid = outcome.valid;
      row.single_list = is_single_list(backend_name_);
      if (capacity) {
        const std::int64_t rss_after = planner::grid_search::bench::vm_rss_bytes();
        row.rss_input_bytes = instance_rss_input_;
        row.rss_search_bytes =
            std::max<std::int64_t>(0, outcome.peak_rss_bytes - rss_before);
        row.rss_output_bytes =
            std::max<std::int64_t>(0, rss_after - outcome.peak_rss_bytes);
        row.rss_hwm_bytes = planner::grid_search::bench::vm_hwm_bytes();
      }
      outcome.record = &row;
    }
    return outcome;
  }

  void record_mismatch(GroupStats& stats, const InstanceRecord& record,
                       const std::string& neighborhood,
                       const std::string& query, int repeat,
                       const std::string& reason) {
    if (stats.mismatch_samples.size() < 10) {
      stats.mismatch_samples.push_back(record.instance_id + "|" + neighborhood +
                                       "|" + query + "|repeat=" +
                                       std::to_string(repeat) + "|" + reason);
    }
  }

  const Options& options_;
  std::vector<InstanceRecord> instances_;
  std::map<std::string, InstanceReference> references_;
  std::string backend_name_;
  GridView current_grid_;
  const InstanceRecord* current_record_{nullptr};
  double conversion_ms_{0.0};
  std::int64_t baseline_rss_{0};
  std::int64_t instance_rss_input_{0};
  std::map<std::string, GroupStats> group_stats_;
  json summary_ = json::object();
};

int run_worker(const Options& options) {
  std::vector<InstanceRecord> instances = load_instances(options);
  std::map<std::string, InstanceReference> references =
      load_reference(options);
  BackendWorker worker(options, std::move(instances), std::move(references));
  return worker.run();
}

// ---------------------------------------------------------------------------
// 父进程：逐后端子进程、合并输出
// ---------------------------------------------------------------------------

std::string self_executable() {
  char buffer[4096];
  const ssize_t size = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
  if (size <= 0) {
    throw std::runtime_error("cannot resolve /proc/self/exe");
  }
  buffer[size] = '\0';
  return std::string(buffer);
}

struct WorkerRun {
  std::string backend;
  int exit_code{-1};
  bool timed_out{false};
  bool spawned{false};
};

WorkerRun run_worker_process(const Options& options, const std::string& backend,
                             const std::string& worker_dir) {
  WorkerRun run;
  run.backend = backend;
  std::vector<std::string> args = {
      self_executable(),
      "--suite",      options.suite,
      "--worker",     "--backend", backend,
      "--output-dir", worker_dir,
      "--instances",  options.instances_path,
      "--reference",  options.reference_path,
      "--repo-root",  options.repo_root,
      "--neighborhoods", planner::grid_search::bench::join_csv(options.neighborhoods),
      "--warmup",     std::to_string(options.warmup),
      "--trials",     std::to_string(options.trials),
      "--p2p-budget-s", std::to_string(options.p2p_budget_s),
      "--df-budget-s",  std::to_string(options.df_budget_s),
      "--cache-mode", options.cache_mode};
  if (options.capacity) {
    args.push_back("--capacity");
  }
  if (options.expand_repeats) {
    args.push_back("--expand-repeats");
  }
  if (!options.filter_classes.empty()) {
    args.push_back("--filter-class");
    args.push_back(planner::grid_search::bench::join_csv(options.filter_classes));
  }
  if (!options.filter_scales.empty()) {
    args.push_back("--filter-scale");
    args.push_back(planner::grid_search::bench::join_csv(options.filter_scales));
  }
  if (!options.filter_seeds.empty()) {
    args.push_back("--filter-seed");
    args.push_back(planner::grid_search::bench::join_csv(options.filter_seeds));
  }

  std::vector<char*> argv;
  argv.reserve(args.size() + 1);
  for (std::string& arg : args) {
    argv.push_back(const_cast<char*>(arg.c_str()));
  }
  argv.push_back(nullptr);

  const pid_t pid = fork();
  if (pid < 0) {
    throw std::runtime_error("fork failed");
  }
  if (pid == 0) {
    execv(argv[0], argv.data());
    _exit(127);
  }
  run.spawned = true;

  // 卡死检测安全网：工作进程 metrics.csv 在 (4×最大预算 + 300 s) 内没有新增
  // 写入即视为卡死（后端预算钩子失效时的最后防线）；SIGTERM → 5 s → SIGKILL。
  // 预热不写行，慢后端的 5 次预热 + 1 次测量仍在阈值内。
  const double stall_limit_s =
      300.0 + 4.0 * std::max(options.p2p_budget_s, options.df_budget_s);
  const fs::path metrics_path = fs::path(worker_dir) / "metrics.csv";
  std::error_code time_error;
  fs::file_time_type last_write{};
  if (fs::exists(metrics_path, time_error)) {
    last_write = fs::last_write_time(metrics_path, time_error);
  }
  double last_change = planner::grid_search::bench::wall_now_s();
  int status = 0;
  while (true) {
    const pid_t waited = waitpid(pid, &status, WNOHANG);
    if (waited == pid) {
      break;
    }
    if (waited < 0) {
      break;
    }
    const fs::file_time_type current =
        fs::exists(metrics_path, time_error)
            ? fs::last_write_time(metrics_path, time_error)
            : fs::file_time_type{};
    if (current != last_write) {
      last_write = current;
      last_change = planner::grid_search::bench::wall_now_s();
    }
    if (planner::grid_search::bench::wall_now_s() - last_change >
        stall_limit_s) {
      run.timed_out = true;
      kill(pid, SIGTERM);
      const double kill_deadline =
          planner::grid_search::bench::wall_now_s() + 5.0;
      while (planner::grid_search::bench::wall_now_s() < kill_deadline) {
        if (waitpid(pid, &status, WNOHANG) == pid) {
          break;
        }
        usleep(50000);
      }
      if (waitpid(pid, &status, WNOHANG) != pid) {
        kill(pid, SIGKILL);
        waitpid(pid, &status, 0);
      }
      break;
    }
    usleep(20000);
  }
  if (WIFEXITED(status)) {
    run.exit_code = WEXITSTATUS(status);
  } else if (WIFSIGNALED(status)) {
    run.exit_code = 128 + WTERMSIG(status);
  }
  return run;
}

int run_parent(const Options& options) {
  const fs::path output_dir = options.output_dir;
  fs::create_directories(output_dir / "workers");

  // 后端列表：默认全部已启用后端；显式列表校验存在性。
  std::vector<std::string> backends = options.backends;
  const std::vector<std::string> available =
      planner::grid_search::available_backends();
  if (backends.empty()) {
    backends = available;
  } else {
    for (const std::string& backend : backends) {
      if (std::find(available.begin(), available.end(), backend) ==
          available.end()) {
        usage_error("backend not available in this build: " + backend);
      }
    }
  }
  if (backends.empty()) {
    usage_error("no backends available");
  }

  const auto instances = load_instances(options);
  if (instances.empty()) {
    usage_error("no instances match the filters");
  }
  const auto references = load_reference(options);

  json summary;
  summary["type"] = "grid_bench_summary";
  summary["schema_version"] = 1;
  summary["run_id"] = options.run_id;
  summary["suite"] = options.suite;
  summary["mode"] = options.capacity ? "capacity" : "formal";
  summary["created_at"] = planner::grid_search::bench::utc_now_iso8601();
  {
    char host[256] = {0};
    gethostname(host, sizeof(host) - 1);
    summary["host"] = std::string(host);
    summary["cpu_count"] = static_cast<int>(sysconf(_SC_NPROCESSORS_ONLN));
  }
  summary["protocol"] = {
      {"warmup", options.warmup},
      {"trials", options.trials},
      {"p2p_budget_s", options.p2p_budget_s},
      {"df_budget_s", options.df_budget_s},
      {"cache_mode", options.cache_mode},
      {"neighborhoods", options.neighborhoods},
      {"tolerance", "1e-8 + 1e-8*abs(reference)"},
      {"single_run_capacity", options.capacity},
      {"expand_repeats", options.expand_repeats},
  };
  {
    std::map<std::string, int> by_class;
    std::map<std::string, int> by_scale;
    for (const auto& record : instances) {
      by_class[record.scenario_class] += 1;
      by_scale[std::to_string(record.scale)] += 1;
    }
    summary["instances"] = {
        {"total", static_cast<int>(instances.size())},
        {"by_class", by_class},
        {"by_scale", by_scale},
    };
  }
  summary["backends_requested"] = backends;
  summary["backends_available"] = available;
  {
    bool ok = false;
    const std::string reference_sha =
        planner::grid_search::bench::sha256_file_hex(options.reference_path, &ok);
    summary["reference"] = {{"path", options.reference_path},
                            {"sha256", ok ? reference_sha : ""},
                            {"instances", static_cast<int>(references.size())}};
  }

  // 合并 CSV：父进程写表头，逐后端追加（工作进程失败时保留已产出行）。
  CsvWriter merged((output_dir / "benchmark_metrics.csv").string());
  if (!merged.open_ok()) {
    throw std::runtime_error("cannot open benchmark_metrics.csv");
  }
  merged.write_header(column_names());

  json backend_summaries = json::object();
  std::vector<std::string> worker_failures;
  std::int64_t total_mismatches = 0;
  std::int64_t total_trials = 0;
  std::int64_t total_timeouts = 0;
  std::int64_t total_warmup_mismatches = 0;

  for (const std::string& backend : backends) {
    const fs::path worker_dir = output_dir / "workers" / backend;
    fs::create_directories(worker_dir);
    const WorkerRun run = run_worker_process(options, backend,
                                             worker_dir.string());
    json backend_summary;
    const auto summary_text =
        planner::grid_search::bench::read_text_file(
            (worker_dir / "summary.json").string());
    if (summary_text.has_value()) {
      backend_summary = json::parse(*summary_text);
    } else {
      backend_summary = json::object();
    }
    backend_summary["worker_exit_code"] = run.exit_code;
    backend_summary["worker_timed_out"] = run.timed_out;
    if (run.exit_code != 0 || run.timed_out) {
      backend_summary["worker_failed"] = true;
      worker_failures.push_back(
          backend + " (exit=" + std::to_string(run.exit_code) +
          (run.timed_out ? ", timed_out" : "") + ")");
    } else {
      backend_summary["worker_failed"] = false;
    }
    if (backend_summary.contains("groups")) {
      for (const auto& [key, group] : backend_summary.at("groups").items()) {
        total_mismatches += group.value("mismatches", static_cast<std::int64_t>(0));
        total_trials += group.value("trials", static_cast<std::int64_t>(0));
        total_timeouts += group.value("timeout", static_cast<std::int64_t>(0));
        total_warmup_mismatches +=
            group.value("warmup_mismatches", static_cast<std::int64_t>(0));
      }
    }

    // 追加该工作进程的 metrics.csv（跳过表头）。
    const auto metrics_text =
        planner::grid_search::bench::read_text_file(
            (worker_dir / "metrics.csv").string());
    if (metrics_text.has_value()) {
      std::istringstream stream(*metrics_text);
      std::string line;
      bool first = true;
      while (std::getline(stream, line)) {
        if (first) {
          first = false;
          continue;
        }
        if (!line.empty()) {
          merged.write_row_raw(line);
        }
      }
    }
    backend_summaries[backend] = backend_summary;
  }
  merged.flush();

  summary["backends"] = backend_summaries;
  summary["totals"] = {
      {"backends", static_cast<int>(backends.size())},
      {"trials", total_trials},
      {"timeouts", total_timeouts},
      {"mismatches", total_mismatches},
      {"warmup_mismatches", total_warmup_mismatches},
      {"worker_failures", worker_failures},
  };
  summary["finished_at"] = planner::grid_search::bench::utc_now_iso8601();
  summary["pending"] = json::array(
      {"formal 全矩阵与容量档搜索属独占窗口阶段；本轮为 harness + 烟测",
       "vss2sn 不暴露展开节点计数（expanded_nodes 留空）",
       "collision_checks 所有后端未暴露（留空）"});
  planner::grid_search::bench::write_text_file(
      (output_dir / "bench_summary.json").string(), summary.dump(2) + "\n");

  std::cout << "planner_grid_search_bench: " << backends.size()
            << " backends, " << total_trials << " measured trials, "
            << total_timeouts << " timeouts, " << total_mismatches
            << " mismatches, " << worker_failures.size() << " worker failures\n";
  return (total_mismatches == 0 && worker_failures.empty()) ? 0 : 1;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Options options = parse_args(argc, argv);
    if (options.worker) {
      if (options.worker_backend.empty()) {
        usage_error("--worker requires --backend");
      }
      return run_worker(options);
    }
    return run_parent(options);
  } catch (const std::exception& exception) {
    std::cerr << "planner_grid_search_bench: " << exception.what() << "\n";
    return 2;
  }
}
