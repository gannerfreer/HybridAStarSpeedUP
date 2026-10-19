// planner_hybrid_astar_bench：Hybrid A* 后端的性能对比测试工具
//（Docs/component_development_plan.md §7.2，tools/README.md §7.2）。
//
// 契约：
//   planner_hybrid_astar_bench --suite v1 --output-dir <dir>
//   planner_hybrid_astar_bench --suite v2 --fixtures-dir <v7 目录>
//                                --output-dir <dir>
//   planner_hybrid_astar_bench --suite v3 --fixtures-dir <v8 目录>
//                                --vehicle parking_bench.json
//                                --output-dir <dir>
//   planner_hybrid_astar_bench --suite v4 --fixtures-dir <v9 目录>
//                                --vehicle parkbench_vehicle.json
//                                --output-dir <dir>
//   planner_hybrid_astar_bench --help
//   * --time-budget-s <秒>：每次试验的时间预算（double，默认 5.0，即 v1
//     正式协议；v2 实验套件经 tools/components.json 传 2.0 以加速后续
//     运行；v3 停车套件按注册命令传 30.0）。节点预算固定 1e6 不变；实际
//     预算写入 CSV 的 time_budget_s 列与 selection.json，便于证据核对。
//   * --vehicle <名>：车辆文件（默认 default.json；v3 用 parking_bench.json）；
//   * --warmup-runs/--measured-runs <n>：每（后端×场景）的预热/实测次数
//     （默认 5/30；v3 注册命令 0/1）；--cold-only 跳过热模式；
//   * --jobs <n>：以（后端×场景）组合为并行单元（默认 1，输出顺序不变）；
//     组合内保持预热→实测→热模式复用语义；单 trial CPU 口径在线程级
//     （Linux getrusage(RUSAGE_THREAD)）；
//   * --export-paths-dir <dir>：实测 trial 的 plan_result JSON 导出
//     （<dir>/<case_id>/<backend_id>[.<warm>].json），供对比绘图消费；
//   * --precheck：仅准备全部场景并输出 <dir>/precheck.json（起终点碰撞
//     预检），不运行搜索；
//   * 写入 <dir>/benchmark_metrics.csv，包含固定指标列与组件特有列；
//     每次实测试验一行，失败保留，未测量的值留空；
//   * 写入 <dir>/plots/*.svg 及对应的 *.csv；
//   * 写入 <dir>/selection.json，包含初步的 §7.2 选择数据；suite v2/v3
//     额外标记 experimental=true 并回显统一配置与协议；
//   * 套件完成时退出码为 0（失败会被记录），设置错误时非零。
//
// v1：`tests/fixtures/v2/paths/cases.json` 的 12 个代表性场景，注册后端按
// 各自冻结配置运行。v2（2026-10-16 统一栅格档实验）：读取
// `--fixtures-dir/paths/cases_bench_v2.json` 的 34 个场景（fixtures v7 全量
// 36 例中排除 2 个穷举型负例 b2-unreachable-03/06——缺口 1.5/1.6 m、几何
// 不可通过但自由栅格松弛连通，需穷举证明；按 v1 惯例（unreachable_128 类）
// 不进入基准计时/门槛集合，5 s 预算下 normalized 后端会超时，超时证据保留
// 在 run 20261016T113415176181Z_hybrid_astar_a22d3479；两者仍保留在
// fixtures v7 供正确性覆盖），即 30 个正例 + 4 个快速可证负例；
// 4 个 normalized 后端（ros_free /
// kth_normalized / zm0612_normalized / smac_normalized）在 configure() 时
// 叠加统一配置 {xy_resolution_m: 0.5, heading_bins: 72, motion_step_m: 1.0,
// reverse_motion_step_m: 1.0}（代价项保持各自默认），3 个 native 后端保持
// 原配置作参考行；selection.json 的选择字段照旧计算但仅供实验参考，不替换
// v1 正式选型。user_astar 后端不接受 reverse_motion_step_m，不参与 v2。
//
// v3 停车套件（fixtures v8；2026-10-17）：读取
// `--fixtures-dir/paths/cases_bench_parking.json` 的 115 个论文停车场景
// （id case_1..case_115，统一画布、统一 expected=success，探索性对比），
// 后端固定为上述 4 个统一栅格 normalized 后端 + 同一 overlay；协议为冷启动
// 单次（--cold-only --warmup-runs 0 --measured-runs 1 --time-budget-s 30）、
// --jobs 并行与 --export-paths-dir 路径导出；选择字段仅供实验参考，不替换
// v1 正式选型。停车场景源数据、车辆与画布边界说明见
// tests/fixtures/v8/README.md。
//
// v4 ParkBench 套件（fixtures v9；2026-10-17）：读取
// `--fixtures-dir/paths/cases_bench_parkbench.json` 的 51 个受限倒车入库
// 场景（id pb-001..pb-051，细线段墙体→薄凸四边形，目标/起点框裁剪，统一
// expected=success），后端与 overlay、协议（冷启动单次 + 路径导出）、以及
// selection.json 的实验性标记与 v3 相同；车辆为 ParkBench 论文车
// （parkbench_vehicle.json，4.95×2.00 m、轴距 3.0 m、满舵 32°）。场景来源、
// 清洗审计（逐例计数）与许可（CC-BY-NC-4.0）见 tests/fixtures/v9/README.md。
//
// 每个后端和场景的流程：先 5 次预热，再用种子 0..29 做 30 次实测；冷模式
// 每次试验新建后端并调用 reset_cache()，热模式在预热后对同一 (map, goal)
// 复用一个后端。预算：每次运行默认 5 s（`--time-budget-s` 可调，v2 实验
// 为 2.0 s）/ 1e6 个节点。墙钟时间与 CPU 时间在 plan() 前后测量；峰值 RSS
// 不按单次试验测量，保持为空。
//
// `--smoke` 运行各套件的代表性子集（v1：2 场景；v2：3 场景；v3：3 场景；
// v4：3 场景 pb-001/026/051），停车套件预热 0 次、实测 1 次，用于 CTest
// 自检与入口烟测。

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#if defined(__linux__)
#include <sys/resource.h>
#endif

#include "fixture_scene.hpp"
#include "planner/common/serialization.hpp"
#include "planner/hybrid_astar/backend.hpp"

#ifndef PLANNER_HYBRID_ASTAR_FIXTURES_DIR
#error "PLANNER_HYBRID_ASTAR_FIXTURES_DIR must be defined by the bench CMakeLists"
#endif

namespace {

using planner::collision::CheckResult;
using planner::common::PlanRequest;
using planner::common::PlanResult;
using planner::common::Status;
using planner::hybrid_astar::HybridAStarBackend;
using planner::hybrid_astar::PlannerRsProvider;
using planner::hybrid_astar::test_support::PathCase;
using planner::hybrid_astar::test_support::PreparedCase;

constexpr int kWarmupRuns = 5;
constexpr int kMeasuredRuns = 30;
constexpr double kDefaultTimeBudgetS = 5.0;
constexpr std::int64_t kNodeBudget = 1000000;

// 单次 trial 的 CPU 时间：Linux 上用线程级 rusage（--jobs > 1 时进程级
// std::clock 会把其他线程的 CPU 计入本 trial，语义错误）；其他平台退回
// 进程级 std::clock（仅单线程套件使用）。
double trial_cpu_ms() {
#if defined(__linux__)
  struct rusage usage {};
  getrusage(RUSAGE_THREAD, &usage);
  return 1000.0 * (usage.ru_utime.tv_sec + usage.ru_stime.tv_sec) +
         (usage.ru_utime.tv_usec + usage.ru_stime.tv_usec) / 1000.0;
#else
  return 1000.0 * static_cast<double>(std::clock()) / CLOCKS_PER_SEC;
#endif
}

// v3 停车套件（论文 115 场景；fixtures v8）：与本文件其余套件共用加载与
// 校验路径；路径导出供对比绘图工具（tools/parking_compare_plots.py）消费。
constexpr const char* kParkingSuite = "v3";
// v4 ParkBench 套件（受限倒车入库 51 场景；fixtures v9）：同一套加载/校验/
// 导出路径；车辆为 ParkBench 论文车（parkbench_vehicle.json）。
constexpr const char* kParkbenchSuite = "v4";

bool is_parking_like_suite(const std::string& suite) {
  return suite == kParkingSuite || suite == kParkbenchSuite;
}

// v2 统一栅格档实验（2026-10-16）：对可接受这些键的 4 个 normalized 后端在
// configure() 时叠加的统一配置（cell / 航向 / 步长统一；代价项保持各自
// 默认）。smac_normalized 的 heading 64 -> 72 由该 overlay 覆盖。
const nlohmann::json kUnifiedGridOverlay = {
    {"xy_resolution_m", 0.5},
    {"heading_bins", 72},
    {"motion_step_m", 1.0},
    {"reverse_motion_step_m", 1.0},
};
const std::vector<std::string> kUnifiedGridBackends = {
    "ros_free", "kth_normalized", "zm0612_normalized", "smac_normalized"};

bool is_unified_grid_backend(const std::string& backend_id) {
  return std::find(kUnifiedGridBackends.begin(), kUnifiedGridBackends.end(),
                   backend_id) != kUnifiedGridBackends.end();
}

// 后端在本次套件中使用的 configure() 输入：v2/v3/v4 对统一栅格后端叠加
// kUnifiedGridOverlay，其余（含 native 参考行与 v1 全部后端）保持默认。
nlohmann::json backend_config_for(const std::string& suite,
                                  const std::string& backend_id) {
  if ((suite == "v2" || is_parking_like_suite(suite)) &&
      is_unified_grid_backend(backend_id)) {
    return kUnifiedGridOverlay;
  }
  return nlohmann::json::object();
}

struct Scenario {
  std::string id;
  PathCase path_case;
  PreparedCase prepared;

  Scenario()
      : prepared(planner::common::GridView{}, planner::collision::Scene{},
                 planner::common::VehicleModel{}) {}
};

struct TrialRow {
  std::string backend_id;
  std::string backend_kind;
  std::string case_id;
  std::string scenario_class;
  std::string cache_mode;
  int repeat_id{0};
  int seed{0};
  std::string status;
  std::string failure_reason;
  std::string preprocess_ms;  // 未单独采集 -> 留空
  double wall_ms{0.0};
  double cpu_ms{0.0};
  std::string peak_rss_bytes;  // 不按单次试验测量 -> 留空
  std::int64_t expanded_nodes{0};
  std::int64_t collision_checks{0};
  std::string path_length_m;
  std::string gear_changes;
  std::string endpoint_position_error_m;
  std::string endpoint_yaw_error_rad;
  bool valid{false};
  std::string motion_primitives;
  std::string rs_connection_attempts;
  std::string motion_rejections;
  std::string search_time_ms;
  std::string postprocess_time_ms;
  std::string total_time_ms;
};

std::string csv_field(const std::string& value) {
  if (value.find_first_of(",\"\n") == std::string::npos) {
    return value;
  }
  std::string quoted = "\"";
  for (const char character : value) {
    if (character == '"') {
      quoted += "\"\"";
    } else {
      quoted += character;
    }
  }
  quoted += "\"";
  return quoted;
}

double percentile_nearest_rank(std::vector<double> values, double percentile) {
  if (values.empty()) {
    return std::nan("");
  }
  std::sort(values.begin(), values.end());
  const double rank = std::ceil(percentile / 100.0 * values.size());
  std::size_t index = rank <= 1.0 ? 0 : static_cast<std::size_t>(rank) - 1;
  index = std::min(index, values.size() - 1);
  return values[index];
}

bool prepare_scenario(const PathCase& path_case,
                      const planner::common::VehicleModel& vehicle,
                      Scenario* scenario) {
  scenario->id = path_case.id;
  scenario->path_case = path_case;
  scenario->prepared =
      planner::hybrid_astar::test_support::prepare_case(path_case, vehicle);
  return scenario->prepared.checker.valid();
}

// 把 PlanResult 序列化为 plan_result JSON 写入导出目录（v3 对比绘图消费）。
// 序列化失败（契约校验拒绝）不中断套件：写入带 export_error 的最小记录。
void export_plan_result(const std::string& export_dir,
                        const std::string& case_id,
                        const std::string& backend_id,
                        const std::string& cache_mode,
                        const PlanResult& result) {
  if (export_dir.empty()) {
    return;
  }
  namespace fs = std::filesystem;
  const fs::path directory = fs::path(export_dir) / case_id;
  std::error_code error;
  fs::create_directories(directory, error);
  const std::string suffix = cache_mode == "cold" ? "" : "." + cache_mode;
  const fs::path file = directory / (backend_id + suffix + ".json");
  std::ofstream stream(file);
  if (!stream) {
    return;
  }
  try {
    stream << planner::common::encode_document(result, 2);
  } catch (const std::exception& exception) {
    stream.clear();
    stream.seekp(0);
    stream << "{\"request_id\": \"" << result.request_id
           << "\", \"status\": \"export_error\", \"reason\": \""
           << exception.what() << "\"}\n";
  }
}

TrialRow run_trial(const std::string& backend_id,
                   const std::string& backend_kind, const Scenario& scenario,
                   const planner::common::VehicleModel& vehicle,
                   const std::string& cache_mode, int repeat_id, int seed,
                   double time_budget_s, const nlohmann::json& backend_config,
                   HybridAStarBackend* reusable_backend,
                   const std::string& export_paths_dir) {
  TrialRow row;
  row.backend_id = backend_id;
  row.backend_kind = backend_kind;
  row.case_id = scenario.id;
  row.scenario_class = scenario.path_case.case_class;
  row.cache_mode = cache_mode;
  row.repeat_id = repeat_id;
  row.seed = seed;

  std::unique_ptr<HybridAStarBackend> owned;
  HybridAStarBackend* backend = reusable_backend;
  if (backend == nullptr) {
    owned = planner::hybrid_astar::create_backend(backend_id);
    backend = owned.get();
  }
  if (backend == nullptr) {
    row.status = "invalid_input";
    row.failure_reason = "unknown backend id";
    return row;
  }
  backend->configure(backend_config);
  backend->reset_cache();

  PlanRequest request = scenario.prepared.request;
  request.time_budget_s = time_budget_s;
  request.node_budget = kNodeBudget;
  PlannerRsProvider provider(vehicle);

  const auto wall_start = std::chrono::steady_clock::now();
  const double cpu_start = trial_cpu_ms();
  const PlanResult result = backend->plan(request, scenario.prepared.grid,
                                          scenario.prepared.checker, provider);
  const auto wall_end = std::chrono::steady_clock::now();
  const double cpu_end = trial_cpu_ms();
  row.wall_ms = std::chrono::duration<double, std::milli>(wall_end - wall_start)
                    .count();
  row.cpu_ms = cpu_end - cpu_start;
  row.status = planner::common::to_string(result.status);
  row.failure_reason = result.reason;

  if (repeat_id >= 0) {
    export_plan_result(export_paths_dir, scenario.id, backend_id, cache_mode,
                       result);
  }

  const nlohmann::json& stats = result.statistics;
  const auto stats_value = [&](const char* key, std::string* out) {
    if (!stats.contains(key)) {
      return;
    }
    const nlohmann::json& value = stats.at(key);
    if (value.is_string()) {
      *out = value.get<std::string>();
    } else if (!value.is_null()) {
      *out = value.dump();
    }
  };
  const auto number_to_string = [](double value) {
    std::ostringstream stream;
    stream << std::setprecision(9) << value;
    return stream.str();
  };
  row.expanded_nodes =
      stats.value("expanded_nodes", static_cast<std::int64_t>(0));
  row.collision_checks =
      stats.value("collision_checks", static_cast<std::int64_t>(0));
  stats_value("motion_primitives", &row.motion_primitives);
  stats_value("rs_connection_attempts", &row.rs_connection_attempts);
  stats_value("motion_rejections", &row.motion_rejections);
  stats_value("search_time_ms", &row.search_time_ms);
  stats_value("postprocess_time_ms", &row.postprocess_time_ms);
  stats_value("total_time_ms", &row.total_time_ms);

  if (result.status == Status::kSuccess) {
    row.path_length_m = number_to_string(stats.value("path_length_m", 0.0));
    row.gear_changes = std::to_string(
        stats.value("gear_changes", static_cast<std::int64_t>(0)));
    row.endpoint_position_error_m =
        number_to_string(stats.value("endpoint_position_error_m", 0.0));
    row.endpoint_yaw_error_rad =
        number_to_string(stats.value("endpoint_yaw_error_rad", 0.0));
    const std::string error = planner::hybrid_astar::test_support::validation_error(
        result, scenario.path_case.start, scenario.path_case.goal, vehicle);
    const CheckResult collision =
        scenario.prepared.checker.check_path(result.path);
    row.valid = error.empty() && collision.is_clear();
    if (!error.empty()) {
      row.failure_reason = error;
    } else if (!collision.is_clear()) {
      row.failure_reason = planner::hybrid_astar::test_support::describe_collision(
          collision);
    }
  }
  return row;
}

void write_metrics_csv(const std::string& path,
                       const std::vector<TrialRow>& rows,
                       double time_budget_s) {
  std::ofstream stream(path);
  stream << "run_id,case_id,repeat_id,cache_mode,seed,status,failure_reason,"
            "preprocess_ms,wall_ms,cpu_ms,peak_rss_bytes,expanded_nodes,"
            "collision_checks,path_length_m,gear_changes,"
            "endpoint_position_error_m,endpoint_yaw_error_rad,valid,"
            "backend_id,backend_kind,scenario_class,node_budget,time_budget_s,"
            "motion_primitives,rs_connection_attempts,motion_rejections,"
            "search_time_ms,postprocess_time_ms,total_time_ms\n";
  const char* run_id = std::getenv("PLANNER_RUN_ID");
  const std::string run = run_id != nullptr ? run_id : "local";
  stream << std::setprecision(9);
  for (const TrialRow& row : rows) {
    stream << csv_field(run) << "," << csv_field(row.case_id) << ","
           << row.repeat_id << "," << row.cache_mode << "," << row.seed << ","
           << row.status << "," << csv_field(row.failure_reason) << ","
           << row.preprocess_ms << "," << row.wall_ms << "," << row.cpu_ms
           << "," << row.peak_rss_bytes << "," << row.expanded_nodes << ","
           << row.collision_checks << "," << row.path_length_m << ","
           << row.gear_changes << "," << row.endpoint_position_error_m << ","
           << row.endpoint_yaw_error_rad << "," << (row.valid ? "true" : "false")
           << "," << row.backend_id << "," << row.backend_kind << ","
           << row.scenario_class << "," << kNodeBudget << "," << time_budget_s
           << "," << row.motion_primitives << "," << row.rs_connection_attempts
           << "," << row.motion_rejections << "," << row.search_time_ms << ","
           << row.postprocess_time_ms << "," << row.total_time_ms << "\n";
  }
}

struct BackendAggregate {
  std::string backend_id;
  std::string kind;
  int trials{0};
  int successes{0};
  int failures{0};
  std::vector<double> success_wall_ms;
  std::vector<double> all_wall_ms;
  double p50_ms{0.0};
  double p95_ms{0.0};
  double total_cpu_ms{0.0};
  double mean_path_length_m{0.0};
  double max_gear_changes{0.0};
  std::map<std::string, double> scenario_p95_ms;  // 成功样本
  std::map<std::string, double> scenario_path_length;
  std::map<std::string, double> scenario_gear_changes;
  std::map<std::string, int> scenario_success_count;
  std::map<std::string, bool> negative_expected_seen;
  std::map<std::string, bool> negative_wrong_success;
  bool correctness_gate{true};
  bool path_gate{true};
  bool gear_gate{true};
  bool qualified{false};
  std::vector<std::string> gate_failures;
  std::map<std::string, double> length_ratios;
  std::vector<std::string> gear_violations;
};

std::string svg_bar_chart(const std::string& title,
                          const std::vector<std::pair<std::string, double>>& data,
                          const std::string& unit) {
  const double max_value = [&]() {
    double maximum = 0.0;
    for (const auto& entry : data) {
      maximum = std::max(maximum, entry.second);
    }
    return maximum > 0.0 ? maximum : 1.0;
  }();
  std::ostringstream svg;
  svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"760\" height=\""
      << 60 + 30 * data.size() << "\" font-family=\"sans-serif\" font-size=\"12\">\n";
  svg << "<text x=\"10\" y=\"20\" font-size=\"14\">" << title << "</text>\n";
  double y = 40;
  for (const auto& entry : data) {
    const double width = 500.0 * entry.second / max_value;
    svg << "<text x=\"10\" y=\"" << y + 12 << "\">" << entry.first
        << "</text>\n";
    svg << "<rect x=\"220\" y=\"" << y << "\" width=\"" << width
        << "\" height=\"16\" fill=\"#4477aa\"/>\n";
    svg << "<text x=\"" << 230 + width << "\" y=\"" << y + 12 << "\">"
        << std::fixed << std::setprecision(1) << entry.second << " " << unit
        << "</text>\n";
    y += 30;
  }
  svg << "</svg>\n";
  return svg.str();
}

}  // namespace

void print_usage() {
  std::printf(
      "usage: planner_hybrid_astar_bench --suite <v1|v2|v3|v4> --output-dir <dir>\n"
      "       [--fixtures-dir <dir>] [--time-budget-s <seconds>] [--smoke]\n"
      "       [--vehicle <name>] [--warmup-runs <n>] [--measured-runs <n>]\n"
      "       [--cold-only] [--jobs <n>] [--export-paths-dir <dir>]\n"
      "       [--precheck]\n"
      "options:\n"
      "  --time-budget-s <seconds>  每次试验的时间预算秒数（默认 5.0，即 v1\n"
      "                             正式协议；v2 实验套件使用 2.0；v3/v4 停车\n"
      "                             套件按注册命令使用 30.0）\n"
      "  --vehicle <name>           车辆文件名（默认 default.json；v3 用\n"
      "                             parking_bench.json、v4 用 parkbench_vehicle.json）\n"
      "  --warmup-runs <n>          每（后端×场景）测量的预热次数（默认 5）\n"
      "  --measured-runs <n>        每（后端×场景）实测次数（默认 30）\n"
      "  --cold-only                跳过热模式（仅冷启动试验；v3 协议）\n"
      "  --jobs <n>                 并行 worker 数（默认 1；按后端×场景组合\n"
      "                             并行，组合内保持原顺序与热启动复用语义）\n"
      "  --export-paths-dir <dir>   实测 trial 的 plan_result JSON 导出目录\n"
      "                             （<case_id>/<backend_id>[.warm].json）\n"
      "  --precheck                 仅准备全部场景并输出起终点碰撞预检\n"
      "                             （<output-dir>/precheck.json），不运行搜索\n");
}

int main(int argc, char** argv) {
  std::string suite = "v1";
  std::string output_dir;
  std::string fixtures_dir = PLANNER_HYBRID_ASTAR_FIXTURES_DIR;
  std::string vehicle_name = "default.json";
  std::string export_paths_dir;
  double time_budget_s = kDefaultTimeBudgetS;
  int warmup_runs = kWarmupRuns;
  int measured_runs = kMeasuredRuns;
  int jobs = 1;
  bool smoke = false;
  bool cold_only = false;
  bool precheck = false;
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--suite" && i + 1 < argc) {
      suite = argv[++i];
    } else if (argument == "--output-dir" && i + 1 < argc) {
      output_dir = argv[++i];
    } else if (argument == "--fixtures-dir" && i + 1 < argc) {
      fixtures_dir = argv[++i];
    } else if (argument == "--vehicle" && i + 1 < argc) {
      vehicle_name = argv[++i];
    } else if (argument == "--export-paths-dir" && i + 1 < argc) {
      export_paths_dir = argv[++i];
    } else if (argument == "--time-budget-s" && i + 1 < argc) {
      const std::string value = argv[++i];
      char* end = nullptr;
      const double parsed = std::strtod(value.c_str(), &end);
      if (end == value.c_str() || *end != '\0' || !std::isfinite(parsed) ||
          parsed <= 0.0) {
        std::fprintf(stderr,
                     "--time-budget-s requires a positive number, got: %s\n",
                     value.c_str());
        return 2;
      }
      time_budget_s = parsed;
    } else if (argument == "--warmup-runs" && i + 1 < argc) {
      warmup_runs = std::atoi(argv[++i]);
      if (warmup_runs < 0) {
        std::fprintf(stderr, "--warmup-runs must be >= 0\n");
        return 2;
      }
    } else if (argument == "--measured-runs" && i + 1 < argc) {
      measured_runs = std::atoi(argv[++i]);
      if (measured_runs < 1) {
        std::fprintf(stderr, "--measured-runs must be >= 1\n");
        return 2;
      }
    } else if (argument == "--jobs" && i + 1 < argc) {
      jobs = std::atoi(argv[++i]);
      if (jobs < 1 || jobs > 256) {
        std::fprintf(stderr, "--jobs must be in [1, 256]\n");
        return 2;
      }
    } else if (argument == "--help" || argument == "-h") {
      print_usage();
      return 0;
    } else if (argument == "--smoke") {
      smoke = true;
    } else if (argument == "--cold-only") {
      cold_only = true;
    } else if (argument == "--precheck") {
      precheck = true;
    } else {
      std::fprintf(stderr, "unknown argument: %s\n", argument.c_str());
      return 2;
    }
  }
  if (suite != "v1" && suite != "v2" && suite != kParkingSuite &&
      suite != kParkbenchSuite) {
    std::fprintf(stderr, "unsupported suite: %s (defined: v1, v2, v3, v4)\n",
                 suite.c_str());
    return 2;
  }
  if (output_dir.empty()) {
    std::fprintf(stderr, "--output-dir is required\n");
    return 2;
  }
  {
    std::error_code directory_error;
    std::filesystem::create_directories(output_dir, directory_error);
  }

  const auto vehicle = planner::hybrid_astar::test_support::load_default_vehicle_from(
      fixtures_dir + "/vehicles/" + vehicle_name);
  const std::string cases_file =
      suite == kParkingSuite
          ? fixtures_dir + "/paths/cases_bench_parking.json"
          : suite == kParkbenchSuite
                ? fixtures_dir + "/paths/cases_bench_parkbench.json"
                : suite == "v2" ? fixtures_dir + "/paths/cases_bench_v2.json"
                                : fixtures_dir + "/paths/cases.json";
  const auto cases = planner::hybrid_astar::test_support::load_path_cases_from(
      cases_file);
  std::printf("suite %s: %zu cases loaded from %s\n", suite.c_str(),
              cases.size(), cases_file.c_str());

  std::vector<std::string> scenario_ids;
  if (suite == "v1") {
    // 固定的 v1 套件场景：具有代表性的成功场景加一个不可达负例；耗时的
    // 穷尽性证明（unreachable_128）和边界用例不进入计时集合。
    scenario_ids = {"open_101",  "open_104",    "obstacle_106",
                    "obstacle_109", "corridor_111", "corridor_112",
                    "corridor_114", "park_116",    "park_118",
                    "deadend_121", "deadend_124",  "unreachable_127"};
    if (smoke) {
      scenario_ids = {"open_101", "obstacle_106"};
      warmup_runs = 1;
      measured_runs = 2;
    }
  } else if (suite == "v2") {
    // v2：fixtures v7 全量 36 例（6 类 × 6 变体，统一画布）按文件顺序，
    // 排除 2 个穷举型负例 b2-unreachable-03/06（缺口 1.5/1.6 m、几何不可
    // 通过但自由栅格松弛连通，需穷举证明；按 v1 惯例 unreachable_128 类
    // 负例不进入基准计时/门槛集合——5 s 预算下 4 个 normalized 后端会
    // 超时，超时证据保留在 run 20261016T113415176181Z_hybrid_astar_a22d3479）。
    // 两者仍保留在 fixtures v7 供正确性覆盖；场景加载与校验复用 v1 的
    // prepare_scenario 路径。基准集合为 34 例 = 30 正例 + 4 快速可证负例。
    const std::vector<std::string> exhaustive_negative_ids = {
        "b2-unreachable-03", "b2-unreachable-06"};
    for (const PathCase& path_case : cases) {
      if (std::find(exhaustive_negative_ids.begin(),
                    exhaustive_negative_ids.end(),
                    path_case.id) != exhaustive_negative_ids.end()) {
        continue;
      }
      scenario_ids.push_back(path_case.id);
    }
    if (smoke) {
      scenario_ids = {"b2-open-01", "b2-obstacle-01", "b2-unreachable-01"};
      warmup_runs = 1;
      measured_runs = 2;
    }
  } else {
    // v3 停车套件（fixtures v8）/ v4 ParkBench 套件（fixtures v9）：全量场景
    // 按文件顺序，无排除项；统一 expected=success（探索性对比，失败只记录）。
    // 协议由注册命令给定（--cold-only --warmup-runs 0 --measured-runs 1
    // --time-budget-s 30 --jobs 8 --export-paths-dir …）；本工具的默认值保持
    // v1 正式协议语义。
    for (const PathCase& path_case : cases) {
      scenario_ids.push_back(path_case.id);
    }
    if (smoke) {
      scenario_ids = suite == kParkbenchSuite
                         ? std::vector<std::string>{"pb-001", "pb-026", "pb-051"}
                         : std::vector<std::string>{"case_1", "case_57",
                                                    "case_115"};
      warmup_runs = 0;
      measured_runs = 1;
    }
  }

  std::printf("suite %s: %zu scenarios selected\n", suite.c_str(),
              scenario_ids.size());

  std::vector<Scenario> scenarios;
  for (const std::string& id : scenario_ids) {
    for (const PathCase& path_case : cases) {
      if (path_case.id != id) {
        continue;
      }
      Scenario scenario;
      if (!prepare_scenario(path_case, vehicle, &scenario)) {
        std::fprintf(stderr, "scenario %s is not valid\n", id.c_str());
        return 1;
      }
      scenarios.push_back(std::move(scenario));
      break;
    }
  }
  if (scenarios.size() != scenario_ids.size()) {
    std::fprintf(stderr, "some scenarios were not found in the fixtures\n");
    return 1;
  }

  // ------------------------------------------------------------------
  // --precheck：仅输出起终点碰撞预检（不运行搜索）。用于 v8 停车夹具的
  // 正式预检证据（写入 <output-dir>/precheck.json，退出码 0/1）。
  // ------------------------------------------------------------------
  if (precheck) {
    std::error_code directory_error;
    std::filesystem::create_directories(output_dir, directory_error);
    nlohmann::json report;
    report["performed_by"] =
        "planner_hybrid_astar_bench --suite " + suite + " --precheck";
    report["suite"] = suite;
    report["vehicle"] = vehicle_name;
    report["case_count"] = scenarios.size();
    nlohmann::json per_case = nlohmann::json::object();
    std::vector<std::string> invalid_cases;
    for (const Scenario& scenario : scenarios) {
      const CheckResult& start_check = scenario.prepared.start_check;
      const CheckResult& goal_check = scenario.prepared.goal_check;
      nlohmann::json entry;
      entry["start_clear"] = start_check.is_clear();
      entry["goal_clear"] = goal_check.is_clear();
      if (!start_check.is_clear()) {
        entry["start_detail"] = planner::hybrid_astar::test_support::describe_collision(start_check);
      }
      if (!goal_check.is_clear()) {
        entry["goal_detail"] = planner::hybrid_astar::test_support::describe_collision(goal_check);
      }
      per_case[scenario.id] = entry;
      if (!start_check.is_clear() || !goal_check.is_clear()) {
        invalid_cases.push_back(scenario.id);
      }
    }
    report["cases"] = per_case;
    report["valid_cases"] =
        static_cast<int>(scenarios.size()) - static_cast<int>(invalid_cases.size());
    report["invalid_cases"] = invalid_cases;
    std::ofstream stream(output_dir + "/precheck.json");
    stream << report.dump(2) << "\n";
    std::printf("precheck: %zu scenarios, %zu invalid -> %s\n",
                scenarios.size(), invalid_cases.size(),
                (output_dir + "/precheck.json").c_str());
    for (const std::string& id : invalid_cases) {
      std::printf("  invalid: %s\n", id.c_str());
    }
    return invalid_cases.empty() ? 0 : 1;
  }

  const auto registered_backends = planner::hybrid_astar::list_backends();
  std::vector<planner::hybrid_astar::BackendDescriptor> backends;
  for (const auto& descriptor : registered_backends) {
    // v2 统一栅格档实验固定为 v1 的候选/参考集合：4 个可叠加统一配置的
    // normalized 后端 + 3 个 native 参考行。user_astar 后端不接受
    // reverse_motion_step_m，无法进入统一栅格对比，不参与 v2。
    if (suite == "v2" && (descriptor.id == "user_astar_raw" ||
                          descriptor.id == "user_astar_normalized")) {
      continue;
    }
    // v3 停车套件 / v4 ParkBench 套件固定为 4 个统一栅格 normalized 后端：
    // ros_free / kth_normalized / zm0612_normalized / smac_normalized（native
    // 参考行与 user_astar 后端不参与，后者不接受 reverse_motion_step_m）。
    if (is_parking_like_suite(suite) && !is_unified_grid_backend(descriptor.id)) {
      continue;
    }
    backends.push_back(descriptor);
  }

  // 配置预检：拒绝任何无法接受本套件配置的后端，并记录实际生效配置
  // （v2 写入 selection.json，用于核对 overlay 生效）。
  nlohmann::json effective_configs = nlohmann::json::object();
  for (const auto& descriptor : backends) {
    const auto backend = planner::hybrid_astar::create_backend(descriptor.id);
    if (backend == nullptr) {
      std::fprintf(stderr, "unknown backend id: %s\n", descriptor.id.c_str());
      return 1;
    }
    const auto report =
        backend->configure(backend_config_for(suite, descriptor.id));
    if (!report.accepted) {
      std::fprintf(stderr, "backend %s cannot be configured: %s\n",
                   descriptor.id.c_str(), report.reason.c_str());
      return 1;
    }
    effective_configs[descriptor.id] = report.effective_config;
  }

  // ------------------------------------------------------------------
  // 试验循环：以（后端 × 场景）组合为并行单元（--jobs，默认 1 = 原顺序与
  // 输出顺序不变）；组合内保持“预热 → 实测 → 热模式复用后端”的原语义。
  // ------------------------------------------------------------------
  struct Combo {
    const planner::hybrid_astar::BackendDescriptor* descriptor;
    const Scenario* scenario;
    nlohmann::json backend_config;
  };
  std::vector<Combo> combos;
  combos.reserve(backends.size() * scenarios.size());
  for (const auto& descriptor : backends) {
    for (const Scenario& scenario : scenarios) {
      combos.push_back(Combo{&descriptor, &scenario,
                             backend_config_for(suite, descriptor.id)});
    }
  }

  std::vector<std::vector<TrialRow>> combo_rows(combos.size());
  std::atomic<std::size_t> next_combo{0};
  const auto run_combo = [&](std::size_t index) {
    const Combo& combo = combos[index];
    std::vector<TrialRow>& collected = combo_rows[index];
    const std::string backend_id = combo.descriptor->id;
    const std::string backend_kind = to_string(combo.descriptor->kind);
    // 冷模式：每次实测新建后端。
    for (int trial = 0; trial < warmup_runs + measured_runs; ++trial) {
      const bool measured = trial >= warmup_runs;
      const int repeat = trial - warmup_runs;
      if (!measured) {
        run_trial(backend_id, backend_kind, *combo.scenario, vehicle, "cold",
                  -1, -1, time_budget_s, combo.backend_config, nullptr, "");
        continue;
      }
      collected.push_back(run_trial(backend_id, backend_kind, *combo.scenario,
                                    vehicle, "cold", repeat, repeat,
                                    time_budget_s, combo.backend_config, nullptr,
                                    export_paths_dir));
    }
    // 热模式：跨试验复用一个后端实例（--cold-only 时跳过）。
    if (!cold_only) {
      const auto warm_backend =
          planner::hybrid_astar::create_backend(backend_id);
      for (int trial = 0; trial < warmup_runs + measured_runs; ++trial) {
        const bool measured = trial >= warmup_runs;
        const int repeat = trial - warmup_runs;
        if (!measured) {
          run_trial(backend_id, backend_kind, *combo.scenario, vehicle, "warm",
                    -1, -1, time_budget_s, combo.backend_config,
                    warm_backend.get(), "");
          continue;
        }
        collected.push_back(run_trial(backend_id, backend_kind, *combo.scenario,
                                      vehicle, "warm", repeat, repeat,
                                      time_budget_s, combo.backend_config,
                                      warm_backend.get(), export_paths_dir));
      }
    }
  };

  if (jobs <= 1) {
    for (std::size_t index = 0; index < combos.size(); ++index) {
      run_combo(index);
      std::printf("backend %s x %s: measured=%zu\n",
                  combos[index].descriptor->id.c_str(),
                  combos[index].scenario->id.c_str(),
                  combo_rows[index].size());
    }
  } else {
    std::printf("running %zu combos with %d workers\n", combos.size(), jobs);
    std::vector<std::thread> workers;
    workers.reserve(static_cast<std::size_t>(jobs));
    for (int worker = 0; worker < jobs; ++worker) {
      workers.emplace_back([&]() {
        for (;;) {
          const std::size_t index = next_combo.fetch_add(1);
          if (index >= combos.size()) {
            break;
          }
          run_combo(index);
        }
      });
    }
    for (auto& worker : workers) {
      worker.join();
    }
  }
  std::vector<TrialRow> rows;
  for (auto& collected : combo_rows) {
    rows.insert(rows.end(), collected.begin(), collected.end());
  }
  std::map<std::string, std::size_t> trials_by_backend;
  for (const TrialRow& row : rows) {
    ++trials_by_backend[row.backend_id];
  }
  for (const auto& entry : trials_by_backend) {
    std::printf("backend %s: measured trials=%zu\n", entry.first.c_str(),
                entry.second);
  }

  write_metrics_csv(output_dir + "/benchmark_metrics.csv", rows, time_budget_s);

  // ------------------------------------------------------------------
  // 聚合与 §7.2 选择数据。
  //
  // 资格判定（Docs §7.2）：归一化候选必须 (1) 在每个套件场景上表现符合
  // 预期（正确性代理：成功场景产出有效路径，负例场景返回文档化的状态），
  // (2) 平均路径长度不超过通过正确性检查的候选中各场景最优值的 110%，
  // 且 (3) 方向变化次数不超过最小值 +1。默认后端为合格候选中冷启动 P95
  // 等权平均值最低者；相差低于 5% 时比较总 CPU（不测单次试验内存），
  // 平局仍存在时保留 Smac 基线。
  // ------------------------------------------------------------------
  std::map<std::string, bool> scenario_expects_success;
  std::map<std::string, std::string> scenario_expected_failure;
  for (const Scenario& scenario : scenarios) {
    scenario_expects_success[scenario.id] =
        scenario.path_case.expected_status == "success";
    scenario_expected_failure[scenario.id] =
        scenario.path_case.expected_failure_reason;
  }

  std::map<std::string, BackendAggregate> aggregates;
  for (const TrialRow& row : rows) {
    if (row.cache_mode != "cold") {
      continue;
    }
    BackendAggregate& aggregate = aggregates[row.backend_id];
    aggregate.backend_id = row.backend_id;
    aggregate.kind = row.backend_kind;
    ++aggregate.trials;
    aggregate.total_cpu_ms += row.cpu_ms;
    aggregate.all_wall_ms.push_back(row.wall_ms);
    if (row.valid) {
      ++aggregate.successes;
      aggregate.success_wall_ms.push_back(row.wall_ms);
      aggregate.scenario_p95_ms[row.case_id] = 0.0;  // 标记为存在
      aggregate.scenario_path_length[row.case_id] +=
          std::stod(row.path_length_m.empty() ? "0" : row.path_length_m);
      aggregate.scenario_gear_changes[row.case_id] =
          std::max(aggregate.scenario_gear_changes[row.case_id],
                   std::stod(row.gear_changes.empty() ? "0" : row.gear_changes));
    } else {
      ++aggregate.failures;
    }
    if (scenario_expects_success[row.case_id]) {
      if (row.valid) {
        ++aggregate.scenario_success_count[row.case_id];
      }
    } else {
      if (row.valid) {
        aggregate.negative_wrong_success[row.case_id] = true;
      }
      if (row.status == scenario_expected_failure[row.case_id]) {
        aggregate.negative_expected_seen[row.case_id] = true;
      }
    }
  }
  // 各场景成功样本的 P95、平均路径长度和最大换挡次数。
  for (auto& entry : aggregates) {
    BackendAggregate& aggregate = entry.second;
    std::map<std::string, std::vector<double>> per_scenario;
    for (const TrialRow& row : rows) {
      if (row.cache_mode != "cold" || row.backend_id != aggregate.backend_id) {
        continue;
      }
      if (row.valid) {
        per_scenario[row.case_id].push_back(row.wall_ms);
      }
    }
    aggregate.scenario_p95_ms.clear();
    for (auto& scenario_entry : per_scenario) {
      aggregate.scenario_p95_ms[scenario_entry.first] =
          percentile_nearest_rank(scenario_entry.second, 95.0);
    }
    aggregate.p50_ms = percentile_nearest_rank(aggregate.success_wall_ms, 50.0);
    aggregate.p95_ms = percentile_nearest_rank(aggregate.success_wall_ms, 95.0);
    std::map<std::string, double> path_sum;
    std::map<std::string, int> path_count;
    std::map<std::string, double> gear_max;
    for (const TrialRow& row : rows) {
      if (row.cache_mode != "cold" || row.backend_id != aggregate.backend_id ||
          !row.valid) {
        continue;
      }
      path_sum[row.case_id] += std::stod(row.path_length_m);
      ++path_count[row.case_id];
      gear_max[row.case_id] =
          std::max(gear_max[row.case_id],
                   std::stod(row.gear_changes.empty() ? "0" : row.gear_changes));
    }
    for (auto& scenario_entry : path_sum) {
      aggregate.scenario_path_length[scenario_entry.first] =
          scenario_entry.second / path_count[scenario_entry.first];
    }
    aggregate.scenario_gear_changes = gear_max;
  }

  std::vector<std::string> normalized_ids;
  for (const auto& descriptor : backends) {
    if (descriptor.kind == planner::hybrid_astar::BackendKind::kNormalized) {
      normalized_ids.push_back(descriptor.id);
    }
  }

  // 正确性代理门限。
  for (const std::string& backend_id : normalized_ids) {
    BackendAggregate& aggregate = aggregates[backend_id];
    for (const Scenario& scenario : scenarios) {
      if (scenario_expects_success[scenario.id]) {
        if (aggregate.scenario_success_count[scenario.id] < 1) {
          aggregate.correctness_gate = false;
          aggregate.gate_failures.push_back(
              "correctness: no valid success on " + scenario.id);
        }
      } else {
        if (aggregate.negative_wrong_success[scenario.id]) {
          aggregate.correctness_gate = false;
          aggregate.gate_failures.push_back(
              "correctness: unexpected success on " + scenario.id);
        }
        if (!aggregate.negative_expected_seen[scenario.id]) {
          aggregate.correctness_gate = false;
          aggregate.gate_failures.push_back(
              "correctness: documented failure status not observed on " +
              scenario.id);
        }
      }
    }
  }

  // 参考集合：通过正确性检查的归一化候选。
  std::vector<std::string> reference_ids;
  for (const std::string& backend_id : normalized_ids) {
    if (aggregates[backend_id].correctness_gate) {
      reference_ids.push_back(backend_id);
    }
  }

  // 公共成功场景：每个参考候选都有有效成功记录。
  std::vector<std::string> common_scenarios;
  for (const std::string& scenario_id : scenario_ids) {
    if (!scenario_expects_success[scenario_id]) {
      continue;
    }
    bool common = !reference_ids.empty();
    for (const std::string& backend_id : reference_ids) {
      if (aggregates[backend_id].scenario_p95_ms.find(scenario_id) ==
          aggregates[backend_id].scenario_p95_ms.end()) {
        common = false;
        break;
      }
    }
    if (common) {
      common_scenarios.push_back(scenario_id);
    }
  }

  // 路径长度（<=110%）与换挡次数（<=min+1）门限。
  nlohmann::json backend_report = nlohmann::json::object();
  for (const std::string& backend_id : normalized_ids) {
    BackendAggregate& aggregate = aggregates[backend_id];
    for (const std::string& scenario_id : scenario_ids) {
      if (!scenario_expects_success[scenario_id]) {
        continue;
      }
      double best_length = std::numeric_limits<double>::infinity();
      double min_gears = std::numeric_limits<double>::infinity();
      for (const std::string& other_id : reference_ids) {
        const BackendAggregate& other = aggregates[other_id];
        const auto length = other.scenario_path_length.find(scenario_id);
        if (length != other.scenario_path_length.end()) {
          best_length = std::min(best_length, length->second);
        }
        const auto gears = other.scenario_gear_changes.find(scenario_id);
        if (gears != other.scenario_gear_changes.end()) {
          min_gears = std::min(min_gears, gears->second);
        }
      }
      const auto length = aggregate.scenario_path_length.find(scenario_id);
      const auto gears = aggregate.scenario_gear_changes.find(scenario_id);
      if (length != aggregate.scenario_path_length.end() &&
          std::isfinite(best_length) && best_length > 0.0) {
        const double ratio = length->second / best_length;
        aggregate.length_ratios[scenario_id] = ratio;
        if (ratio > 1.10) {
          aggregate.path_gate = false;
          std::ostringstream reason;
          reason << "path_length: " << scenario_id << " ratio "
                 << std::fixed << std::setprecision(3) << ratio << " > 1.10";
          aggregate.gate_failures.push_back(reason.str());
        }
      }
      if (gears != aggregate.scenario_gear_changes.end() &&
          std::isfinite(min_gears) && gears->second > min_gears + 1.0) {
        aggregate.gear_gate = false;
        aggregate.gear_violations.push_back(scenario_id);
        aggregate.gate_failures.push_back("gear_change: " + scenario_id +
                                          " exceeds min+1");
      }
    }
    aggregate.qualified = aggregate.correctness_gate && aggregate.path_gate &&
                          aggregate.gear_gate;
  }

  nlohmann::json selection = nlohmann::json::object();
  selection["suite"] = suite;
  selection["preliminary"] = true;
  selection["selection_status"] = "preliminary_pending_formal_window";
  if (suite == "v2" || is_parking_like_suite(suite)) {
    // 统一栅格档实验（v2，2026-10-16）与停车套件（v3/v4）：选择字段照旧计算，
    // 仅供实验参考，不替换 v1 正式选型；同时回显 overlay 与实际生效配置。
    std::vector<std::string> native_reference_ids;
    for (const auto& descriptor : backends) {
      if (descriptor.kind == planner::hybrid_astar::BackendKind::kNative) {
        native_reference_ids.push_back(descriptor.id);
      }
    }
    selection["experimental"] = true;
    if (suite == kParkingSuite) {
      selection["experimental_note"] =
          "停车基准探索性套件：IEEE TITS 2022 论文 115 个停车场景（fixtures "
          "v8）× 4 个 normalized 后端（ros_free / kth_normalized / "
          "zm0612_normalized / smac_normalized），统一配置 overlay 与 v2 相同"
          "（代价项各自默认）；协议为冷启动单次（--cold-only --warmup-runs 0 "
          "--measured-runs 1）；expected 统一为 success（论文基线），失败只"
          "记录；选择字段仅供实验参考，不替换 v1 正式选型。";
      selection["parking_suite"] = true;
    } else if (suite == kParkbenchSuite) {
      selection["experimental_note"] =
          "ParkBench 探索性套件：Constrained_Parking_Scenarios（arXiv:2601.22545，"
          "CC-BY-NC-4.0）51 个受限倒车入库场景（fixtures v9，细线段墙体→薄凸"
          "四边形、目标/起点框裁剪）× 4 个 normalized 后端，统一配置 overlay 与 "
          "v2 相同（代价项各自默认）；车辆为 ParkBench 论文车 4.95×2.00 m"
          "（轴距 3.0 m、满舵 32°）；协议为冷启动单次；expected 统一为 success"
          "（原论文可达基准），失败只记录；选择字段仅供实验参考，不替换 v1 "
          "正式选型。";
      selection["parkbench_suite"] = true;
    } else {
      selection["experimental_note"] =
          "统一栅格档实验：4 个 normalized 后端叠加统一配置 "
          "xy_resolution_m=0.5 / heading_bins=72 / motion_step_m=1.0 / "
          "reverse_motion_step_m=1.0（代价项保持各自默认）；3 个 native "
          "后端保持原配置作参考；选择字段仅供实验参考，不替换 v1 正式选型";
    }
    selection["unified_grid_overlay"] = kUnifiedGridOverlay;
    selection["unified_grid_backends"] = kUnifiedGridBackends;
    selection["reference_backends"] = native_reference_ids;
    selection["effective_configs"] = effective_configs;
    selection["protocol"] = {
        {"warmup_runs", warmup_runs},
        {"measured_runs", measured_runs},
        {"cold_only", cold_only},
        {"jobs", jobs},
        {"vehicle", vehicle_name},
        {"time_budget_s", time_budget_s},
        {"node_budget", kNodeBudget},
        {"export_paths_dir", export_paths_dir}};
  }
  selection["warmup_runs"] = warmup_runs;
  selection["measured_runs"] = measured_runs;
  selection["budgets"] = {{"time_budget_s", time_budget_s},
                          {"node_budget", kNodeBudget}};
  selection["common_success_scenarios"] = common_scenarios;
  selection["reference_candidates"] = reference_ids;

  double best_p95_mean = std::nan("");
  std::string fastest_backend;
  for (const std::string& backend_id : normalized_ids) {
    BackendAggregate& aggregate = aggregates[backend_id];
    double p95_sum = 0.0;
    bool complete = !common_scenarios.empty();
    for (const std::string& scenario_id : common_scenarios) {
      const auto found = aggregate.scenario_p95_ms.find(scenario_id);
      if (found == aggregate.scenario_p95_ms.end()) {
        complete = false;
        break;
      }
      p95_sum += found->second;
    }
    nlohmann::json report;
    report["kind"] = aggregate.kind;
    report["trials"] = aggregate.trials;
    report["successes"] = aggregate.successes;
    report["failures"] = aggregate.failures;
    report["p50_ms"] = aggregate.p50_ms;
    report["p95_ms"] = aggregate.p95_ms;
    report["total_cpu_ms"] = aggregate.total_cpu_ms;
    report["correctness_gate"] = aggregate.correctness_gate;
    report["path_length_gate"] = aggregate.path_gate;
    report["gear_change_gate"] = aggregate.gear_gate;
    report["qualified"] = aggregate.qualified;
    report["gate_failures"] = aggregate.gate_failures;
    report["path_length_ratios_vs_best"] = aggregate.length_ratios;
    report["gear_change_violations"] = aggregate.gear_violations;
    if (complete) {
      const double mean = p95_sum / common_scenarios.size();
      report["p95_equal_weight_mean_ms"] = mean;
      if (std::isnan(best_p95_mean) || mean < best_p95_mean) {
        best_p95_mean = mean;
        fastest_backend = backend_id;
      }
    } else {
      report["p95_equal_weight_mean_ms"] = nullptr;
    }
    backend_report[backend_id] = report;
  }
  for (const auto& descriptor : backends) {
    if (descriptor.kind == planner::hybrid_astar::BackendKind::kNative) {
      const BackendAggregate& aggregate = aggregates[descriptor.id];
      nlohmann::json report;
      report["kind"] = aggregate.kind;
      report["reference_only"] = true;
      report["trials"] = aggregate.trials;
      report["successes"] = aggregate.successes;
      report["failures"] = aggregate.failures;
      report["p50_ms"] = aggregate.p50_ms;
      report["p95_ms"] = aggregate.p95_ms;
      report["total_cpu_ms"] = aggregate.total_cpu_ms;
      backend_report[aggregate.backend_id] = report;
    }
  }
  selection["backends"] = backend_report;

  // 默认后端：合格候选中冷启动 P95 等权平均值最低者，按 §7.2 使用 <5% 的
  // CPU 决胜规则（不测内存）。
  std::vector<std::pair<double, std::string>> qualified_means;
  for (const std::string& backend_id : normalized_ids) {
    const BackendAggregate& aggregate = aggregates[backend_id];
    if (!aggregate.qualified) {
      continue;
    }
    if (aggregate.scenario_p95_ms.empty()) {
      continue;
    }
    double p95_sum = 0.0;
    bool complete = !common_scenarios.empty();
    for (const std::string& scenario_id : common_scenarios) {
      const auto found = aggregate.scenario_p95_ms.find(scenario_id);
      if (found == aggregate.scenario_p95_ms.end()) {
        complete = false;
        break;
      }
      p95_sum += found->second;
    }
    if (complete) {
      qualified_means.emplace_back(p95_sum / common_scenarios.size(),
                                   backend_id);
    }
  }
  std::sort(qualified_means.begin(), qualified_means.end());
  std::string default_backend;
  std::string default_note;
  if (!qualified_means.empty()) {
    default_backend = qualified_means.front().second;
    default_note =
        "lowest cold-start P95 equal-weight mean among qualified candidates";
    if (qualified_means.size() >= 2) {
      const double best = qualified_means.front().first;
      std::vector<std::string> tie;
      for (const auto& entry : qualified_means) {
        if (entry.first <= best * 1.05) {
          tie.push_back(entry.second);
        }
      }
      if (tie.size() >= 2) {
        std::string cpu_winner;
        double cpu_best = std::numeric_limits<double>::infinity();
        for (const std::string& backend_id : tie) {
          const double cpu = aggregates[backend_id].total_cpu_ms;
          if (cpu < cpu_best) {
            cpu_best = cpu;
            cpu_winner = backend_id;
          }
        }
        default_backend = cpu_winner;
        default_note =
            "P95 means within 5%; total CPU time decides (memory not "
            "measured per trial)";
        bool smac_in_tie = false;
        for (const std::string& backend_id : tie) {
          if (backend_id == "smac_normalized") {
            smac_in_tie = true;
          }
        }
        if (smac_in_tie) {
          default_backend = "smac_normalized";
          default_note =
              "P95 and CPU within 5%; Smac baseline kept (memory not "
              "measured per trial)";
        }
      }
    }
  }
  selection["qualified_candidates"] = nlohmann::json::array();
  for (const auto& entry : qualified_means) {
    selection["qualified_candidates"].push_back(entry.second);
  }
  if (default_backend.empty()) {
    selection["default_backend"] = nullptr;
    selection["default_selection_note"] =
        "no qualified candidate (see gate_failures); selection pending";
  } else {
    selection["default_backend"] = default_backend;
    selection["default_selection_note"] = default_note;
  }
  selection["fastest_backend"] =
      fastest_backend.empty() ? nlohmann::json(nullptr)
                              : nlohmann::json(fastest_backend);
  selection["fastest_backend_note"] =
      "reference only: lowest P95 mean among correctness-passing candidates "
      "before the path/gear gates; not a selection result";
  nlohmann::json pending = nlohmann::json::array();
  for (const std::string& backend_id : normalized_ids) {
    const BackendAggregate& aggregate = aggregates[backend_id];
    if (aggregate.qualified) {
      continue;
    }
    nlohmann::json entry;
    entry["backend"] = backend_id;
    entry["reasons"] = aggregate.gate_failures;
    const auto mean = backend_report[backend_id].find("p95_equal_weight_mean_ms");
    entry["p95_equal_weight_mean_ms"] =
        (mean != backend_report[backend_id].end()) ? *mean
                                                   : nlohmann::json(nullptr);
    pending.push_back(entry);
  }
  selection["pending_optimization"] = pending;
  selection["selection_rule_note"] =
      "Docs §7.2: qualified = correctness + mean path length within 110% of "
      "the per-scenario best + direction changes within min+1; default = "
      "qualified candidate with the lowest cold-start P95 equal-weight mean "
      "over common success scenarios; ties below 5% compare total CPU then "
      "memory (memory not measured here) and keep the Smac baseline";
  std::ofstream selection_stream(output_dir + "/selection.json");
  selection_stream << selection.dump(2) << "\n";

  // 绘图：各后端的冷启动 P50/P95 墙钟时间。
  std::string plots_dir = output_dir + "/plots";
  const int mkdir_result = std::system(("mkdir -p " + plots_dir).c_str());
  (void)mkdir_result;
  std::vector<std::pair<std::string, double>> p50_data;
  std::vector<std::pair<std::string, double>> p95_data;
  std::ofstream plot_csv(plots_dir + "/latency_by_backend.csv");
  plot_csv << "backend_id,kind,samples,p50_ms,p95_ms\n";
  for (const auto& entry : aggregates) {
    const BackendAggregate& aggregate = entry.second;
    p50_data.emplace_back(aggregate.backend_id, aggregate.p50_ms);
    p95_data.emplace_back(aggregate.backend_id, aggregate.p95_ms);
    plot_csv << aggregate.backend_id << "," << aggregate.kind << ","
             << aggregate.successes << "," << aggregate.p50_ms << ","
             << aggregate.p95_ms << "\n";
  }
  std::ofstream(plots_dir + "/latency_p50_by_backend.svg")
      << svg_bar_chart("Hybrid A* cold-start success latency P50 (ms)", p50_data,
                       "ms");
  std::ofstream(plots_dir + "/latency_p95_by_backend.svg")
      << svg_bar_chart("Hybrid A* cold-start success latency P95 (ms)", p95_data,
                       "ms");

  std::printf("benchmark complete: %zu trials, %zu scenarios, %zu backends\n",
              rows.size(), scenarios.size(), backends.size());
  return 0;
}
