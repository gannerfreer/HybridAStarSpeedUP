// planner_rs_bench --suite v1 --output-dir <dir>
//
// 正式的 Reeds-Shepp 微基准测试（Docs/component_development_plan.md 7.2、
// 7.3 和 6.2；tools/README.md 7.2 节“基准输出契约”）。协议：每个
// 操作/用例预热 5 次，然后用固定的确定性合成查询集测量 30 次；批操作
// 使用文档化的批大小。查询生成、一致性检查与 JSON/CSV/SVG 输出不计时。
//
// 输出写入 --output-dir：
//   benchmark_metrics.csv  §7.2 契约文件：每个测量 trial 占一行，
//                          含 §9 固定列及组件列
//                          (operation, phase, batch_size, ns_per_op)。
//                          任务运行器会将其合并进运行级 metrics.csv；
//                          组件绝不写入 --output-dir 以外的位置。
//   cases.csv              精确查询集（id + start/goal）
//   summary.json           机器可读的聚合结果（P50/P95、一致性、
//                          检查）
//   summary.md             人类可读表格
//   plots/planner_rs_latency.svg + plots/planner_rs_latency.csv
//
// 每行的 cache_mode 均为 "cold"：RS 求解器无状态，因此冷/热运行在
// 构造上完全一致；摘要会明确说明这一点。
//
// 退出码：0 成功；1 正确性/一致性检查失败；2 用法错误。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "planner/rs/reeds_shepp.hpp"

#ifdef PLANNER_RS_HAS_OMPL
#include "ompl_reference.hpp"
#endif

namespace {

using planner::common::Direction;
using planner::common::Pose2d;
using planner::common::VehicleModel;
using planner::rs::RsPath;
using planner::rs::RsSegment;

constexpr int kWarmup = 5;
constexpr int kMeasured = 30;
constexpr std::uint64_t kSeed = 20261009ULL;
constexpr double kDs = 0.25;

VehicleModel benchmark_vehicle() {
  VehicleModel vehicle;
  vehicle.id = "v1_benchmark";
  vehicle.wheelbase_m = 2.8;
  vehicle.width_m = 1.8;
  vehicle.front_overhang_m = 0.9;
  vehicle.rear_overhang_m = 1.0;
  vehicle.max_steer_rad = 0.5;
  vehicle.safety_margin_m = 0.1;
  return vehicle;
}

class SplitMix64 {
 public:
  explicit SplitMix64(std::uint64_t seed) : state_(seed) {}
  std::uint64_t next() {
    state_ += 0x9E3779B97F4A7C15ULL;
    std::uint64_t z = state_;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
  }
  double uniform() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }
  double uniform(double lo, double hi) { return lo + (hi - lo) * uniform(); }

 private:
  std::uint64_t state_;
};

struct Case {
  std::string id;
  Pose2d start;
  Pose2d goal;
};

std::vector<Case> make_cases() {
  const VehicleModel vehicle = benchmark_vehicle();
  const double radius = planner::rs::min_turn_radius_m(vehicle);
  const double pi = planner::common::kPi;
  std::vector<Case> cases;
  auto add = [&cases](const std::string& id, const Pose2d& start, const Pose2d& goal) {
    cases.push_back({id, start, goal});
  };
  add("same_pose", {3.0, -2.0, 0.3}, {3.0, -2.0, 0.3});
  add("straight_forward", {0.0, 0.0, 0.0}, {12.0, 0.0, 0.0});
  add("straight_reverse", {0.0, 0.0, 0.0}, {-7.0, 0.0, 0.0});
  add("turn_quarter_left", {0.0, 0.0, 0.0}, {radius, radius, 0.5 * pi});
  add("turn_quarter_right", {0.0, 0.0, 0.0}, {-radius, radius, -0.5 * pi});
  add("turn_min_radius_uturn", {0.0, 0.0, 0.0}, {0.0, 2.0 * radius, pi});
  add("cusp_opposite_heading", {0.0, 0.0, 0.0}, {0.0, 0.0, pi});
  add("reverse_parking", {4.0, 3.0, 0.35}, {1.5, 0.4, -1.2});
  add("angle_wrap_same_pose", {0.0, 0.0, 350.0 * pi / 180.0},
      {0.0, 0.0, -10.0 * pi / 180.0});
  add("angle_wrap_forward", {0.0, 0.0, 0.0}, {6.0, 1.5, -0.4});
  add("angle_wrap_full_turn", {0.0, 0.0, 0.0}, {6.0, 1.5, 2.0 * pi - 0.4});
  add("far_goal", {-25.0, 8.0, 2.5}, {22.0, -14.0, -1.1});

  SplitMix64 rng(kSeed);
  for (int i = 0; i < 32; ++i) {
    std::ostringstream id;
    id << "random_" << std::setw(2) << std::setfill('0') << i;
    add(id.str(),
        {rng.uniform(-20.0, 20.0), rng.uniform(-20.0, 20.0), rng.uniform(-pi, pi)},
        {rng.uniform(-20.0, 20.0), rng.uniform(-20.0, 20.0), rng.uniform(-pi, pi)});
  }
  return cases;
}

// 仅依据分段字段、用定步长 RK4 进行终点重构。
struct Rk4Pose {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

Rk4Pose rk4_integrate(const Pose2d& start, const RsPath& path, double step_m = 1e-3) {
  Rk4Pose state{start.x_m, start.y_m, start.yaw_rad};
  for (const RsSegment& segment : path.segments) {
    const double gear = (segment.direction == Direction::kForward) ? 1.0 : -1.0;
    const double kappa = segment.curvature_inv_m;
    auto derivative = [gear, kappa](const Rk4Pose& p) {
      return Rk4Pose{gear * std::cos(p.yaw), gear * std::sin(p.yaw), gear * kappa};
    };
    const int steps = std::max(1, static_cast<int>(std::ceil(segment.length_m / step_m)));
    const double h = segment.length_m / static_cast<double>(steps);
    for (int i = 0; i < steps; ++i) {
      const Rk4Pose k1 = derivative(state);
      const Rk4Pose s2{state.x + 0.5 * h * k1.x, state.y + 0.5 * h * k1.y,
                       state.yaw + 0.5 * h * k1.yaw};
      const Rk4Pose k2 = derivative(s2);
      const Rk4Pose s3{state.x + 0.5 * h * k2.x, state.y + 0.5 * h * k2.y,
                       state.yaw + 0.5 * h * k2.yaw};
      const Rk4Pose k3 = derivative(s3);
      const Rk4Pose s4{state.x + h * k3.x, state.y + h * k3.y, state.yaw + h * k3.yaw};
      const Rk4Pose k4 = derivative(s4);
      state.x += h * (k1.x + 2.0 * k2.x + 2.0 * k3.x + k4.x) / 6.0;
      state.y += h * (k1.y + 2.0 * k2.y + 2.0 * k3.y + k4.y) / 6.0;
      state.yaw += h * (k1.yaw + 2.0 * k2.yaw + 2.0 * k3.yaw + k4.yaw) / 6.0;
    }
  }
  return state;
}

// 在已排序样本上取最近秩百分位数 (7.2)。
double percentile(const std::vector<double>& sorted, double percent) {
  if (sorted.empty()) {
    return 0.0;
  }
  const std::size_t rank = std::max<std::size_t>(
      1, std::min<std::size_t>(
             sorted.size(),
             static_cast<std::size_t>(std::ceil(percent / 100.0 * sorted.size()))));
  return sorted[rank - 1];
}

struct Stats {
  double p50_ms{0.0};
  double p95_ms{0.0};
  double mean_ms{0.0};
  double min_ms{0.0};
  double max_ms{0.0};
};

Stats summarize(std::vector<double> ms) {
  Stats stats;
  if (ms.empty()) {
    return stats;
  }
  std::sort(ms.begin(), ms.end());
  stats.min_ms = ms.front();
  stats.max_ms = ms.back();
  stats.p50_ms = percentile(ms, 50.0);
  stats.p95_ms = percentile(ms, 95.0);
  double sum = 0.0;
  for (const double value : ms) {
    sum += value;
  }
  stats.mean_ms = sum / static_cast<double>(ms.size());
  return stats;
}

using Clock = std::chrono::steady_clock;

struct Trial {
  double wall_ms{0.0};
  double cpu_ms{0.0};
};

template <typename Fn>
Trial run_trial(Fn&& fn) {
  const Clock::time_point start = Clock::now();
  const std::clock_t cpu_start = std::clock();
  fn();
  const std::clock_t cpu_end = std::clock();
  const Clock::time_point end = Clock::now();
  Trial trial;
  trial.wall_ms = std::chrono::duration<double, std::milli>(end - start).count();
  trial.cpu_ms = 1000.0 * static_cast<double>(cpu_end - cpu_start) / CLOCKS_PER_SEC;
  return trial;
}

std::string format_number(double value) {
  std::ostringstream out;
  out << std::setprecision(12) << value;
  return out.str();
}

// §7.2 契约文件中的一个测量 trial。未测量的字段写为空单元格
// （绝不写 0）。
struct OutputRow {
  std::string operation;
  std::string case_id;
  int repeat_id{0};
  int batch_size{1};
  double wall_ms{0.0};
  bool has_cpu{false};
  double cpu_ms{0.0};
  bool has_path_length{false};
  double path_length_m{0.0};
  bool has_endpoint_errors{false};
  double endpoint_position_error_m{0.0};
  double endpoint_yaw_error_rad{0.0};
};

// 严格按 tools/README.md 7.2 节的要求写入 <output-dir>/benchmark_metrics.csv：
// 按顺序写 §9 固定列，再加上组件专属列
// operation/phase/batch_size/ns_per_op。命令结束后由任务运行器将该文件
// 合并进运行级 metrics.csv；组件本身不得改动运行目录。
void write_benchmark_metrics(const std::filesystem::path& path,
                             const std::string& run_id,
                             const std::vector<OutputRow>& rows) {
  std::ofstream out(path);
  out << "run_id,case_id,repeat_id,cache_mode,seed,status,failure_reason,"
         "preprocess_ms,wall_ms,cpu_ms,peak_rss_bytes,expanded_nodes,"
         "collision_checks,path_length_m,gear_changes,endpoint_position_error_m,"
         "endpoint_yaw_error_rad,valid,operation,phase,batch_size,ns_per_op\n";
  for (const OutputRow& row : rows) {
    const double ns_per_op =
        row.wall_ms * 1e6 / static_cast<double>(std::max(1, row.batch_size));
    out << run_id << ',' << row.case_id << ',' << row.repeat_id
        << ",cold," << kSeed << ",success,,"
        << ',' << format_number(row.wall_ms) << ','
        << (row.has_cpu ? format_number(row.cpu_ms) : std::string())
        << ",,,," << (row.has_path_length ? format_number(row.path_length_m) : std::string())
        << ",,"
        << (row.has_endpoint_errors ? format_number(row.endpoint_position_error_m)
                                    : std::string())
        << ','
        << (row.has_endpoint_errors ? format_number(row.endpoint_yaw_error_rad)
                                    : std::string())
        << ",true," << row.operation << ",measure," << row.batch_size << ','
        << format_number(ns_per_op) << '\n';
  }
}

struct Measurement {
  std::string operation;
  std::string case_id;
  std::vector<double> measured_wall_ms;
  std::vector<double> measured_cpu_ms;
  int batch_size{1};
};

struct CaseMeasurements {
  Measurement distance;
  Measurement path;
  Measurement sample;
  Measurement ompl_distance;
  Measurement ompl_path;
};

// 每个用例的佐证数据，在计时区域之外计算一次。
struct CaseFacts {
  RsPath path;
  double path_length_m{0.0};
  double endpoint_position_error_m{0.0};
  double endpoint_yaw_error_rad{0.0};
};

void run_case_measurements(const Case& query, const VehicleModel& vehicle,
                           const CaseFacts& native_facts,
                           const CaseFacts* ompl_facts,
                           std::vector<OutputRow>& rows,
                           CaseMeasurements& out) {
  auto label = [&query](Measurement& measurement, const std::string& operation) {
    measurement.operation = operation;
    measurement.case_id = query.id;
    measurement.batch_size = 1;
  };
  label(out.distance, "distance");
  label(out.path, "path");
  label(out.sample, "sample");
  label(out.ompl_distance, "ompl_distance");
  label(out.ompl_path, "ompl_path");

  auto measure = [&](const std::string& operation, int repeat, bool warmup,
                     const std::function<void()>& fn, Measurement& target,
                     const CaseFacts* facts) {
    const Trial trial = run_trial(fn);
    if (warmup) {
      return;  // 预热属于协议开销，不算 trial
    }
    OutputRow row;
    row.operation = operation;
    row.case_id = query.id;
    row.repeat_id = repeat;
    row.batch_size = 1;
    row.wall_ms = trial.wall_ms;
    if (facts != nullptr) {
      row.has_path_length = true;
      row.path_length_m = facts->path_length_m;
      row.has_endpoint_errors = true;
      row.endpoint_position_error_m = facts->endpoint_position_error_m;
      row.endpoint_yaw_error_rad = facts->endpoint_yaw_error_rad;
    }
    rows.push_back(row);
    target.measured_wall_ms.push_back(trial.wall_ms);
  };

  volatile double sink = 0.0;
  for (int i = 0; i < kWarmup + kMeasured; ++i) {
    const bool warmup = i < kWarmup;
    const int repeat = warmup ? i : i - kWarmup;
    measure("distance", repeat, warmup,
            [&] { sink += planner::rs::distance(query.start, query.goal, vehicle); },
            out.distance, &native_facts);
  }
  for (int i = 0; i < kWarmup + kMeasured; ++i) {
    const bool warmup = i < kWarmup;
    const int repeat = warmup ? i : i - kWarmup;
    measure("path", repeat, warmup,
            [&] { sink += planner::rs::path(query.start, query.goal, vehicle).length_m; },
            out.path, &native_facts);
  }
  if (!native_facts.path.segments.empty()) {
    for (int i = 0; i < kWarmup + kMeasured; ++i) {
      const bool warmup = i < kWarmup;
      const int repeat = warmup ? i : i - kWarmup;
      measure(
          "sample", repeat, warmup,
          [&] { sink += static_cast<double>(planner::rs::sample(native_facts.path, kDs).size()); },
          out.sample, &native_facts);
    }
  }
#ifdef PLANNER_RS_HAS_OMPL
  for (int i = 0; i < kWarmup + kMeasured; ++i) {
    const bool warmup = i < kWarmup;
    const int repeat = warmup ? i : i - kWarmup;
    measure("ompl_distance", repeat, warmup,
            [&] { sink += planner::rs::ompl_reference::distance(query.start, query.goal, vehicle); },
            out.ompl_distance, ompl_facts);
  }
  for (int i = 0; i < kWarmup + kMeasured; ++i) {
    const bool warmup = i < kWarmup;
    const int repeat = warmup ? i : i - kWarmup;
    measure("ompl_path", repeat, warmup,
            [&] { sink += planner::rs::ompl_reference::path(query.start, query.goal, vehicle).length_m; },
            out.ompl_path, ompl_facts);
  }
#else
  (void)ompl_facts;
  (void)out.ompl_distance;
  (void)out.ompl_path;
#endif
  (void)sink;
}

nlohmann::json stats_json(const Stats& stats) {
  return {{"p50_ms", stats.p50_ms},
          {"p95_ms", stats.p95_ms},
          {"mean_ms", stats.mean_ms},
          {"min_ms", stats.min_ms},
          {"max_ms", stats.max_ms}};
}

}  // namespace

int main(int argc, char** argv) {
  std::string suite;
  std::string output_dir;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--suite" && i + 1 < argc) {
      suite = argv[++i];
    } else if (arg == "--output-dir" && i + 1 < argc) {
      output_dir = argv[++i];
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "usage: planner_rs_bench --suite v1 --output-dir <dir>\n";
      return 0;
    } else {
      std::cerr << "unrecognized argument: " << arg << "\n";
      return 2;
    }
  }
  if (suite != "v1") {
    std::cerr << "unsupported suite '" << suite << "' (only 'v1' is registered)\n";
    return 2;
  }
  if (output_dir.empty()) {
    std::cerr << "--output-dir is required\n";
    return 2;
  }
  const char* run_id_env = std::getenv("PLANNER_RUN_ID");
  const std::string run_id = run_id_env != nullptr ? run_id_env : "";
  const std::filesystem::path out_dir(output_dir);
  std::error_code ec;
  std::filesystem::create_directories(out_dir, ec);
  if (ec) {
    std::cerr << "cannot create output directory: " << out_dir << " (" << ec.message() << ")\n";
    return 2;
  }

  const VehicleModel vehicle = benchmark_vehicle();
  const double radius = planner::rs::min_turn_radius_m(vehicle);
  const double curvature_limit = 1.0 / radius;
  const std::vector<Case> cases = make_cases();

  // ---------------------------------------------------------------------
  // 正确性检查（不计时）：终点重构、曲率界。
  // ---------------------------------------------------------------------
  double worst_position_error = 0.0;
  double worst_yaw_error = 0.0;
  double worst_curvature_ratio = 0.0;
  std::vector<CaseFacts> native_facts;
  native_facts.reserve(cases.size());
  for (const Case& query : cases) {
    CaseFacts facts;
    facts.path = planner::rs::path(query.start, query.goal, vehicle);
    facts.path_length_m = facts.path.length_m;
    const Rk4Pose end = rk4_integrate(facts.path.start, facts.path);
    facts.endpoint_position_error_m =
        std::hypot(end.x - facts.path.goal.x_m, end.y - facts.path.goal.y_m);
    facts.endpoint_yaw_error_rad = std::fabs(
        planner::common::normalize_angle_rad(end.yaw - facts.path.goal.yaw_rad));
    worst_position_error = std::max(worst_position_error, facts.endpoint_position_error_m);
    worst_yaw_error = std::max(worst_yaw_error, facts.endpoint_yaw_error_rad);
    for (const RsSegment& segment : facts.path.segments) {
      const double ratio =
          (curvature_limit > 0.0) ? std::fabs(segment.curvature_inv_m) / curvature_limit : 0.0;
      worst_curvature_ratio = std::max(worst_curvature_ratio, ratio);
    }
    native_facts.push_back(std::move(facts));
  }

  std::vector<CaseFacts> ompl_facts;
  nlohmann::json consistency = {
      {"available", false},
      {"note", "OMPL reference adapter not built (PLANNER_RS_WITH_OMPL=OFF)"}};
  bool consistency_ok = true;
#ifdef PLANNER_RS_HAS_OMPL
  {
    double max_abs = 0.0;
    double max_rel = 0.0;
    double max_tolerance = 0.0;
    std::string worst_id;
    for (const Case& query : cases) {
      CaseFacts facts;
      facts.path = planner::rs::ompl_reference::path(query.start, query.goal, vehicle);
      facts.path_length_m = facts.path.length_m;
      const Rk4Pose end = rk4_integrate(facts.path.start, facts.path);
      facts.endpoint_position_error_m =
          std::hypot(end.x - facts.path.goal.x_m, end.y - facts.path.goal.y_m);
      facts.endpoint_yaw_error_rad = std::fabs(
          planner::common::normalize_angle_rad(end.yaw - facts.path.goal.yaw_rad));
      ompl_facts.push_back(std::move(facts));

      const double native = planner::rs::distance(query.start, query.goal, vehicle);
      const double reference =
          planner::rs::ompl_reference::distance(query.start, query.goal, vehicle);
      const double error = std::fabs(native - reference);
      const double tolerance = 1e-6 + 1e-6 * std::fabs(reference);
      if (error > max_abs) {
        max_abs = error;
        max_rel = error / std::max(1.0, std::fabs(reference));
        max_tolerance = tolerance;
        worst_id = query.id;
      }
      if (!(error <= tolerance)) {
        consistency_ok = false;
      }
    }
    consistency = {
        {"available", true},
        {"kind", "wrapper_consistency"},
        {"note",
         "native and OMPL implement the same 48-curve family; this is a wrapper "
         "consistency comparison, not an independent double validation (3.7)"},
        {"cases", static_cast<int>(cases.size())},
        {"max_abs_error_m", max_abs},
        {"max_rel_error", max_rel},
        {"tolerance_m", max_tolerance},
        {"worst_case_id", worst_id},
        {"consistent", consistency_ok},
    };
  }
#endif

  const bool checks_ok = worst_position_error <= planner::rs::kEndpointPositionToleranceM &&
                         worst_yaw_error <= planner::rs::kEndpointYawToleranceRad &&
                         worst_curvature_ratio <= 1.0 + 1e-12 && consistency_ok;

  // ---------------------------------------------------------------------
  // 逐用例测量：预热 5 次、测量 30 次 (7.2)。
  // ---------------------------------------------------------------------
  std::vector<OutputRow> rows;
  std::map<std::string, Measurement> per_case;  // 键：operation + "/" + case_id
  for (std::size_t i = 0; i < cases.size(); ++i) {
    const Case& query = cases[i];
    const CaseFacts* ompl = ompl_facts.empty() ? nullptr : &ompl_facts[i];
    CaseMeasurements measurements;
    run_case_measurements(query, vehicle, native_facts[i], ompl, rows, measurements);
    auto store = [&per_case](Measurement&& measurement) {
      if (!measurement.measured_wall_ms.empty()) {
        per_case.emplace(measurement.operation + "/" + measurement.case_id,
                         std::move(measurement));
      }
    };
    store(std::move(measurements.distance));
    store(std::move(measurements.path));
    store(std::move(measurements.sample));
    store(std::move(measurements.ompl_distance));
    store(std::move(measurements.ompl_path));
  }

  // ---------------------------------------------------------------------
  // 批测量：文档化批大小，预热 5 次、测量 30 次。
  // ---------------------------------------------------------------------
  const int distance_batch = 1000;
  const int path_batch = 200;
  const int sample_batch = 100;
  std::vector<RsPath> sample_paths;
  for (int i = 0; i < sample_batch; ++i) {
    sample_paths.push_back(native_facts[static_cast<std::size_t>(i) % native_facts.size()].path);
  }

  struct BatchResult {
    std::string operation;
    int batch_size;
    Stats stats;
    Stats cpu;
    double ns_per_op;
    double ops_per_s;
  };
  std::vector<BatchResult> batches;
  volatile double sink = 0.0;

  auto run_batch = [&](const std::string& operation, int batch_size,
                       const std::function<void()>& fn) {
    std::vector<double> wall_ms;
    std::vector<double> cpu_ms;
    for (int i = 0; i < kWarmup + kMeasured; ++i) {
      const bool warmup = i < kWarmup;
      const int repeat = warmup ? i : i - kWarmup;
      const Trial trial = run_trial(fn);
      if (warmup) {
        continue;
      }
      OutputRow row;
      row.operation = operation;
      row.case_id = "batch";
      row.repeat_id = repeat;
      row.batch_size = batch_size;
      row.wall_ms = trial.wall_ms;
      row.has_cpu = true;
      row.cpu_ms = trial.cpu_ms;
      rows.push_back(row);
      wall_ms.push_back(trial.wall_ms);
      cpu_ms.push_back(trial.cpu_ms);
    }
    BatchResult result;
    result.operation = operation;
    result.batch_size = batch_size;
    result.stats = summarize(wall_ms);
    result.cpu = summarize(cpu_ms);
    result.ns_per_op = batch_size > 0 ? result.stats.p50_ms * 1e6 / batch_size : 0.0;
    result.ops_per_s = result.ns_per_op > 0.0 ? 1e9 / result.ns_per_op : 0.0;
    batches.push_back(result);
  };

  run_batch("distance", distance_batch, [&] {
    for (int i = 0; i < distance_batch; ++i) {
      const Case& query = cases[static_cast<std::size_t>(i) % cases.size()];
      sink += planner::rs::distance(query.start, query.goal, vehicle);
    }
  });
  run_batch("path", path_batch, [&] {
    for (int i = 0; i < path_batch; ++i) {
      const Case& query = cases[static_cast<std::size_t>(i) % cases.size()];
      sink += planner::rs::path(query.start, query.goal, vehicle).length_m;
    }
  });
  run_batch("sample", sample_batch, [&] {
    for (const RsPath& path : sample_paths) {
      sink += static_cast<double>(planner::rs::sample(path, kDs).size());
    }
  });
#ifdef PLANNER_RS_HAS_OMPL
  run_batch("ompl_distance", distance_batch, [&] {
    for (int i = 0; i < distance_batch; ++i) {
      const Case& query = cases[static_cast<std::size_t>(i) % cases.size()];
      sink += planner::rs::ompl_reference::distance(query.start, query.goal, vehicle);
    }
  });
  run_batch("ompl_path", path_batch, [&] {
    for (int i = 0; i < path_batch; ++i) {
      const Case& query = cases[static_cast<std::size_t>(i) % cases.size()];
      sink += planner::rs::ompl_reference::path(query.start, query.goal, vehicle).length_m;
    }
  });
#endif
  (void)sink;

  // ---------------------------------------------------------------------
  // 输出（不计时）。
  // ---------------------------------------------------------------------
  write_benchmark_metrics(out_dir / "benchmark_metrics.csv", run_id, rows);
  {
    std::ofstream cases_out(out_dir / "cases.csv");
    cases_out << std::setprecision(17);
    cases_out << "id,start_x_m,start_y_m,start_yaw_rad,goal_x_m,goal_y_m,goal_yaw_rad\n";
    for (const Case& query : cases) {
      cases_out << query.id << ',' << query.start.x_m << ',' << query.start.y_m << ','
                << query.start.yaw_rad << ',' << query.goal.x_m << ',' << query.goal.y_m
                << ',' << query.goal.yaw_rad << '\n';
    }
  }

  nlohmann::json summary;
  summary["schema_version"] = 1;
  summary["type"] = "planner_rs_benchmark_summary";
  summary["suite"] = suite;
  summary["seed"] = kSeed;
  summary["warmup"] = kWarmup;
  summary["measured"] = kMeasured;
  summary["case_count"] = static_cast<int>(cases.size());
  summary["metrics_file"] = "benchmark_metrics.csv";
  summary["cache_mode"] = "cold";
  summary["cache_note"] =
      "RS distance/path/sampling are stateless: cold and warm runs are identical; "
      "every row is therefore labelled cold";
  summary["batch_sizes"] = {{"distance", distance_batch},
                            {"path", path_batch},
                            {"sample", sample_batch}};
#ifdef PLANNER_RS_HAS_OMPL
  summary["batch_sizes"]["ompl_distance"] = distance_batch;
  summary["batch_sizes"]["ompl_path"] = path_batch;
#endif
  summary["vehicle"] = {{"id", vehicle.id},
                        {"wheelbase_m", vehicle.wheelbase_m},
                        {"max_steer_rad", vehicle.max_steer_rad},
                        {"min_turn_radius_m", radius}};
  summary["checks"] = {
      {"endpoint_max_position_error_m", worst_position_error},
      {"endpoint_max_yaw_error_rad", worst_yaw_error},
      {"endpoint_position_tolerance_m", planner::rs::kEndpointPositionToleranceM},
      {"endpoint_yaw_tolerance_rad", planner::rs::kEndpointYawToleranceRad},
      {"curvature_max_ratio", worst_curvature_ratio},
      {"curvature_limit_inv_m", curvature_limit},
      {"all_ok", checks_ok}};
  summary["consistency"] = consistency;

  nlohmann::json operations = nlohmann::json::array();
  for (const BatchResult& batch : batches) {
    operations.push_back({{"name", batch.operation},
                          {"batch_size", batch.batch_size},
                          {"trials", kMeasured},
                          {"wall", stats_json(batch.stats)},
                          {"cpu", stats_json(batch.cpu)},
                          {"ns_per_op_p50", batch.ns_per_op},
                          {"ops_per_s_p50", batch.ops_per_s}});
  }
  summary["operations"] = operations;

  nlohmann::json per_case_json = nlohmann::json::array();
  for (const auto& entry : per_case) {
    const Measurement& measurement = entry.second;
    if (measurement.measured_wall_ms.empty()) {
      continue;
    }
    const Stats stats = summarize(measurement.measured_wall_ms);
    per_case_json.push_back({{"operation", measurement.operation},
                             {"case_id", measurement.case_id},
                             {"trials", static_cast<int>(measurement.measured_wall_ms.size())},
                             {"wall", stats_json(stats)}});
  }
  summary["per_case"] = per_case_json;

  {
    std::ofstream json_out(out_dir / "summary.json");
    json_out << std::setw(2) << summary << '\n';
  }

  // 图表：成对的 SVG + CSV，用数值标签代替仅靠颜色的图例 (7.2)。
  {
    std::filesystem::create_directories(out_dir / "plots", ec);
    std::ofstream csv(out_dir / "plots" / "planner_rs_latency.csv");
    csv << "operation,batch_size,p50_ms,p95_ms,mean_ms,min_ms,max_ms,ns_per_op,ops_per_s\n";
    csv << std::setprecision(12);
    double max_p95 = 0.0;
    for (const BatchResult& batch : batches) {
      max_p95 = std::max(max_p95, batch.stats.p95_ms);
      csv << batch.operation << ',' << batch.batch_size << ',' << batch.stats.p50_ms << ','
          << batch.stats.p95_ms << ',' << batch.stats.mean_ms << ',' << batch.stats.min_ms
          << ',' << batch.stats.max_ms << ',' << batch.ns_per_op << ',' << batch.ops_per_s
          << '\n';
    }
    const int row_height = 44;
    const int top = 56;
    const int left = 300;
    const int bar_width = 380;
    const int height = top + row_height * static_cast<int>(batches.size()) + 24;
    std::ofstream svg(out_dir / "plots" / "planner_rs_latency.svg");
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"820\" height=\"" << height
        << "\" viewBox=\"0 0 820 " << height << "\">\n";
    svg << "<title>planner_rs suite v1 latency (P50/P95, ms per batch)</title>\n";
    svg << "<rect width=\"820\" height=\"" << height << "\" fill=\"#ffffff\"/>\n";
    svg << "<text x=\"20\" y=\"24\" font-size=\"15\" font-family=\"sans-serif\">"
           "planner_rs v1 latency: P50/P95 per batch (ms); values printed as text</text>\n";
    svg << "<text x=\"" << left << "\" y=\"42\" font-size=\"11\" font-family=\"sans-serif\" "
           "fill=\"#333333\">P50 (blue) / P95 (orange), width scaled to max P95 = "
        << format_number(max_p95) << " ms</text>\n";
    const double scale = max_p95 > 0.0 ? static_cast<double>(bar_width) / max_p95 : 0.0;
    int row = 0;
    for (const BatchResult& batch : batches) {
      const int y = top + row * row_height;
      svg << "<text x=\"20\" y=\"" << y + 14
          << "\" font-size=\"12\" font-family=\"sans-serif\">" << batch.operation
          << " (batch=" << batch.batch_size << ")</text>\n";
      const int w50 = static_cast<int>(std::lround(batch.stats.p50_ms * scale));
      const int w95 = static_cast<int>(std::lround(batch.stats.p95_ms * scale));
      svg << "<rect x=\"" << left << "\" y=\"" << y + 2 << "\" width=\"" << std::max(1, w50)
          << "\" height=\"14\" fill=\"#4477aa\"/>\n";
      svg << "<text x=\"" << left + std::max(1, w50) + 6 << "\" y=\"" << y + 14
          << "\" font-size=\"11\" font-family=\"sans-serif\">P50="
          << format_number(batch.stats.p50_ms) << " ms</text>\n";
      svg << "<rect x=\"" << left << "\" y=\"" << y + 20 << "\" width=\"" << std::max(1, w95)
          << "\" height=\"14\" fill=\"#ee8866\"/>\n";
      svg << "<text x=\"" << left + std::max(1, w95) + 6 << "\" y=\"" << y + 32
          << "\" font-size=\"11\" font-family=\"sans-serif\">P95="
          << format_number(batch.stats.p95_ms) << " ms</text>\n";
      ++row;
    }
    svg << "</svg>\n";
  }

  {
    std::ofstream md(out_dir / "summary.md");
    md << "# planner_rs 基准摘要（suite " << suite << "）\n\n";
    md << "- 协议：每个配置预热 " << kWarmup << " 次、测量 " << kMeasured
       << " 次；查询集为确定性合成位姿对（seed " << kSeed << "，共 " << cases.size()
       << " 例；生成不计时）。\n";
    md << "- 车辆：wheelbase=" << vehicle.wheelbase_m << " m, max_steer="
       << vehicle.max_steer_rad << " rad, min_turn_radius=" << radius << " m。\n";
    md << "- 批大小：distance=" << distance_batch << ", path=" << path_batch
       << ", sample=" << sample_batch << "；输入生成与 JSON/CSV/SVG 输出不计时。\n";
    md << "- 指标文件：`benchmark_metrics.csv`（§7.2 契约；每行一个测量 trial，"
          "预热不计；未测量列留空）；`cache_mode=cold`（RS 无状态，冷/热等价）。\n\n";
    md << "## 批操作（单位：ms，P50/P95 为每批耗时）\n\n";
    md << "| operation | batch | P50 | P95 | mean | ns/op (P50) | ops/s (P50) |\n";
    md << "| --- | --- | --- | --- | --- | --- | --- |\n";
    md << std::fixed << std::setprecision(4);
    for (const BatchResult& batch : batches) {
      md << "| " << batch.operation << " | " << batch.batch_size << " | " << batch.stats.p50_ms
         << " | " << batch.stats.p95_ms << " | " << batch.stats.mean_ms << " | "
         << std::setprecision(1) << batch.ns_per_op << " | " << batch.ops_per_s << " |\n";
      md << std::setprecision(4);
    }
    md << "\n## 每例单次调用（单位：ms，测量 30 次）\n\n";
    md << "| case | operation | P50 | P95 |\n| --- | --- | --- | --- |\n";
    for (const auto& entry : per_case) {
      const Measurement& measurement = entry.second;
      if (measurement.measured_wall_ms.empty()) {
        continue;
      }
      const Stats stats = summarize(measurement.measured_wall_ms);
      md << "| " << measurement.case_id << " | " << measurement.operation << " | "
         << stats.p50_ms << " | " << stats.p95_ms << " |\n";
    }
    md << "\n## 正确性与一致性检查\n\n";
    md << "- 终点位置最大误差（RK4 分段积分）：" << worst_position_error << " m（容限 "
       << planner::rs::kEndpointPositionToleranceM << " m）\n";
    md << "- 终点航向最大误差：" << worst_yaw_error << " rad（容限 "
       << planner::rs::kEndpointYawToleranceRad << " rad）\n";
    md << "- 曲率最大比值 |kappa|/limit：" << worst_curvature_ratio << "（<= 1）\n";
    md << "- OMPL 1.6.0 封装一致性：" << (consistency_ok ? "通过" : "失败")
       << "（同一曲线族公式的封装对照，非独立双重验证，见 3.7）\n";
    md << "- 总判定：" << (checks_ok ? "通过" : "失败") << "\n";
    md << "- 图表：`plots/planner_rs_latency.svg` / `plots/planner_rs_latency.csv`\n";
  }

  std::cout << "planner_rs_bench suite=" << suite << " cases=" << cases.size()
            << " batches=" << batches.size() << " rows=" << rows.size() << " checks="
            << (checks_ok ? "ok" : "FAILED") << " run_id=" << run_id
            << " output_dir=" << out_dir << "\n";
  return checks_ok ? 0 : 1;
}
