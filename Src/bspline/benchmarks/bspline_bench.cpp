// planner_bspline v1 微基准测试。
//
// 实现 B-spline 组件第 7.3 节的批大小，以及
// tools/README.md 第 7.2 节的输出契约：
//   * 来自 tests/fixtures/v1/benchmarks/bspline/inputs.json 的三次 2D 样条；
//   * 控制点分组 cp8 / cp32 / cp128；
//   * 求值：每批 10,000 个域内等距参数，求导阶数 0..3 分别测量
//     （输入准备与缓冲区预分配在计时区间之外）；
//   * 构造：每批 100 条样条；等弧长重采样：每批 100 条曲线 x 1,000 个点；
//     构造与采样分别计时；
//   * 后端维度：tinyspline（基线）、eigen_splines（对照候选）与
//     ego_uniform（EGO-Planner UniformBspline 对照，GPL-3.0 隔离库），
//     由 --backend all|both|tinyspline|eigen|ego 选择（默认 all；both 保持
//     旧行为 = tinyspline + eigen_splines）；用例 ID 带后端前缀，输出
//     backend 列，并写 outputs/selection.json 供内部选型；
//   * 预热 5 批、测量 30 批、单线程顺序执行；
//   * 报告批大小、批时间 P50/P95（最近秩）、每次操作的摊薄时间与吞吐量。
//     摊薄值绝不标记为单次调用 P95。
//
// 输出（位于 --output-dir 下）：benchmark_metrics.csv（固定指标列加组件列）、
// bspline_benchmark_results.json（完整证据）与
// plots/bspline_latency_p50_p95.{svg,csv}。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "planner/bspline/bspline.hpp"

namespace {

using planner::bspline::ArcLengthResampleResult;
using planner::bspline::ArcLengthSample;
using planner::bspline::BackendKind;
using planner::bspline::Bspline;
using planner::bspline::EvaluationResult;
using planner::bspline::ParameterDomain;
using planner::bspline::Status;
using planner::common::Point2d;
using Clock = std::chrono::steady_clock;

// 基准的后端维度（供 --backend 选择与输出标注）。
struct BackendSpec {
  BackendKind kind;
  const char* name;  // 稳定后端名，写入指标与 selection.json
};

const BackendSpec kTinysplineBackend{BackendKind::kTinyspline, "tinyspline"};
const BackendSpec kEigenBackend{BackendKind::kEigenSplines, "eigen_splines"};
const BackendSpec kEgoBackend{BackendKind::kEgoUniform, "ego_uniform"};

constexpr int kConstructionCurvesPerBatch = 100;
constexpr int kArcLengthPointsPerCurve = 1000;
constexpr int kEvalParameterCount = 10000;

struct GroupInput {
  std::string name;
  std::vector<double> knots;
  std::vector<Point2d> base_points;
  std::vector<std::vector<Point2d>> construction_inputs;
};

struct Fixture {
  std::string path;
  std::uint64_t fnv1a64 = 0;
  int warmup_batches = 5;
  int measured_batches = 30;
  std::vector<GroupInput> groups;
};

struct CaseResult {
  std::string case_id;
  std::string operation;
  std::string group;
  std::string backend;
  int derivative_order = -1;
  std::size_t batch_operations = 0;
  int curves_per_batch = 0;
  int points_per_curve = 0;
  std::vector<double> batch_ms;
  double p50_ms = 0.0;
  double p95_ms = 0.0;
  double amortized_us_per_operation = 0.0;
  double throughput_operations_per_s = 0.0;
  double error_metric_m = -1.0;
  std::uint64_t checksum = 0;
  bool failed = false;
  std::string failure_reason;
};

std::uint64_t fnv1a64(const std::string& bytes) {
  std::uint64_t hash = 1469598103934665603ull;
  for (const unsigned char byte : bytes) {
    hash ^= byte;
    hash *= 1099511628211ull;
  }
  return hash;
}

double elapsed_ms(Clock::time_point start, Clock::time_point end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

// 最近秩百分位数（第 7.2 节）：rank = ceil(p * n)，从 1 开始。
double percentile_nearest_rank(std::vector<double> sorted, double fraction) {
  if (sorted.empty()) {
    return 0.0;
  }
  std::sort(sorted.begin(), sorted.end());
  std::size_t rank = static_cast<std::size_t>(
      std::ceil(fraction * static_cast<double>(sorted.size())));
  if (rank < 1) {
    rank = 1;
  }
  if (rank > sorted.size()) {
    rank = sorted.size();
  }
  return sorted[rank - 1];
}

void finalize_case(CaseResult* result) {
  if (result->failed) {
    return;
  }
  if (result->batch_ms.empty()) {
    result->failed = true;
    result->failure_reason = "no_measured_batches";
    return;
  }
  double total_ms = 0.0;
  for (const double value : result->batch_ms) {
    total_ms += value;
  }
  const double mean_s =
      total_ms / static_cast<double>(result->batch_ms.size()) / 1000.0;
  result->p50_ms = percentile_nearest_rank(result->batch_ms, 0.50);
  result->p95_ms = percentile_nearest_rank(result->batch_ms, 0.95);
  if (result->batch_operations > 0 && mean_s > 0.0) {
    result->amortized_us_per_operation =
        mean_s / static_cast<double>(result->batch_operations) * 1e6;
    result->throughput_operations_per_s =
        static_cast<double>(result->batch_operations) / mean_s;
  }
}

bool read_fixture(const std::string& path, Fixture* fixture) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream.is_open()) {
    std::fprintf(stderr, "error: cannot open fixture '%s'\n", path.c_str());
    return false;
  }
  std::string bytes((std::istreambuf_iterator<char>(stream)),
                    std::istreambuf_iterator<char>());
  fixture->path = path;
  fixture->fnv1a64 = fnv1a64(bytes);
  nlohmann::json document;
  try {
    document = nlohmann::json::parse(bytes);
  } catch (const std::exception& error) {
    std::fprintf(stderr, "error: fixture is not valid JSON: %s\n", error.what());
    return false;
  }
  try {
    if (document.at("schema_version").get<int>() != 1 ||
        document.at("type").get<std::string>() != "bspline_benchmark_inputs") {
      std::fprintf(stderr, "error: unexpected fixture schema\n");
      return false;
    }
    if (document.at("dimension").get<int>() != 2 ||
        document.at("degree").get<int>() != 3) {
      std::fprintf(stderr, "error: fixture must describe cubic 2D splines\n");
      return false;
    }
    fixture->warmup_batches =
        document.at("timing_boundaries").at("warmup_batches").get<int>();
    fixture->measured_batches =
        document.at("timing_boundaries").at("measured_batches").get<int>();
    for (const nlohmann::json& group : document.at("control_groups")) {
      GroupInput parsed;
      parsed.name = group.at("name").get<std::string>();
      parsed.knots = group.at("knots").get<std::vector<double>>();
      for (const nlohmann::json& point : group.at("control_points")) {
        parsed.base_points.push_back(
            Point2d{point.at("x_m").get<double>(),
                    point.at("y_m").get<double>()});
      }
      if (parsed.base_points.size() !=
          group.at("control_point_count").get<std::size_t>()) {
        std::fprintf(stderr, "error: group '%s' control point count mismatch\n",
                     parsed.name.c_str());
        return false;
      }
      // 测试夹具中的构造规则：曲线 c 将基础点平移
      // (0.25 * c, -0.125 * c) 米；节点与参数域保持不变。
      parsed.construction_inputs.resize(kConstructionCurvesPerBatch);
      for (int c = 0; c < kConstructionCurvesPerBatch; ++c) {
        std::vector<Point2d> shifted = parsed.base_points;
        for (Point2d& point : shifted) {
          point.x_m += 0.25 * static_cast<double>(c);
          point.y_m -= 0.125 * static_cast<double>(c);
        }
        parsed.construction_inputs[static_cast<std::size_t>(c)] =
            std::move(shifted);
      }
      fixture->groups.push_back(std::move(parsed));
    }
  } catch (const std::exception& error) {
    std::fprintf(stderr, "error: fixture field error: %s\n", error.what());
    return false;
  }
  return !fixture->groups.empty();
}

CaseResult run_construction_case(const GroupInput& group,
                                 const BackendSpec& backend,
                                 int warmup_batches, int measured_batches,
                                 std::vector<Bspline>* out_curves) {
  CaseResult result;
  result.case_id = std::string(backend.name) + "/construct-" + group.name;
  result.operation = "construct";
  result.group = group.name;
  result.backend = backend.name;
  result.batch_operations = kConstructionCurvesPerBatch;
  result.curves_per_batch = kConstructionCurvesPerBatch;

  std::vector<Bspline> curves;
  curves.reserve(kConstructionCurvesPerBatch);
  std::uint64_t checksum = 0;
  const auto one_batch = [&]() -> bool {
    curves.clear();
    for (int c = 0; c < kConstructionCurvesPerBatch; ++c) {
      std::optional<Bspline> curve = Bspline::create(
          group.construction_inputs[static_cast<std::size_t>(c)],
          group.knots, backend.kind);
      if (!curve.has_value()) {
        return false;
      }
      checksum += static_cast<std::uint64_t>(curve->degree()) +
                  curve->control_point_count();
      curves.push_back(std::move(*curve));
    }
    return true;
  };

  for (int i = 0; i < warmup_batches; ++i) {
    if (!one_batch()) {
      result.failed = true;
      result.failure_reason = "construction_failed";
      return result;
    }
  }
  for (int i = 0; i < measured_batches; ++i) {
    const Clock::time_point start = Clock::now();
    const bool ok = one_batch();
    const Clock::time_point end = Clock::now();
    if (!ok) {
      result.failed = true;
      result.failure_reason = "construction_failed";
      return result;
    }
    result.batch_ms.push_back(elapsed_ms(start, end));
  }
  result.checksum = checksum;
  if (out_curves != nullptr) {
    *out_curves = std::move(curves);
  }
  finalize_case(&result);
  return result;
}

CaseResult run_evaluation_case(const GroupInput& group, const Bspline& curve,
                               const BackendSpec& backend, int derivative_order,
                               const std::vector<double>& parameters,
                               int warmup_batches, int measured_batches) {
  CaseResult result;
  result.case_id = std::string(backend.name) + "/eval-" + group.name + "-d" +
                   std::to_string(derivative_order);
  result.operation = "evaluate";
  result.group = group.name;
  result.backend = backend.name;
  result.derivative_order = derivative_order;
  result.batch_operations = parameters.size();
  result.points_per_curve = static_cast<int>(parameters.size());

  std::vector<Point2d> buffer(parameters.size());
  std::uint64_t checksum = 0;
  const auto one_batch = [&]() -> bool {
    if (curve.sample(parameters, buffer, derivative_order) != Status::kSuccess) {
      return false;
    }
    checksum += static_cast<std::uint64_t>(std::llround(buffer.front().x_m)) +
                static_cast<std::uint64_t>(std::llround(buffer.back().y_m));
    return true;
  };

  for (int i = 0; i < warmup_batches; ++i) {
    if (!one_batch()) {
      result.failed = true;
      result.failure_reason = "evaluation_failed";
      return result;
    }
  }
  for (int i = 0; i < measured_batches; ++i) {
    const Clock::time_point start = Clock::now();
    const bool ok = curve.sample(parameters, buffer, derivative_order) ==
                    Status::kSuccess;
    const Clock::time_point end = Clock::now();
    if (!ok) {
      result.failed = true;
      result.failure_reason = "evaluation_failed";
      return result;
    }
    checksum += static_cast<std::uint64_t>(std::llround(buffer.front().x_m)) +
                static_cast<std::uint64_t>(std::llround(buffer.back().y_m));
    result.batch_ms.push_back(elapsed_ms(start, end));
  }
  result.checksum = checksum;
  finalize_case(&result);
  return result;
}

CaseResult run_arc_length_case(const GroupInput& group,
                               const std::vector<Bspline>& curves,
                               const BackendSpec& backend, int warmup_batches,
                               int measured_batches) {
  CaseResult result;
  result.case_id = std::string(backend.name) + "/arclength-" + group.name;
  result.operation = "resample_equal_arc_length";
  result.group = group.name;
  result.backend = backend.name;
  result.batch_operations =
      curves.size() * static_cast<std::size_t>(kArcLengthPointsPerCurve);
  result.curves_per_batch = static_cast<int>(curves.size());
  result.points_per_curve = kArcLengthPointsPerCurve;

  // 输出缓冲区在计时区间之外预分配。
  std::vector<std::vector<ArcLengthSample>> buffers(
      curves.size(),
      std::vector<ArcLengthSample>(kArcLengthPointsPerCurve));
  std::uint64_t checksum = 0;
  double max_error = 0.0;
  const auto one_batch = [&]() -> bool {
    max_error = 0.0;
    for (std::size_t c = 0; c < curves.size(); ++c) {
      const ArcLengthResampleResult resampled = curves[c].sample_equal_arc_length(
          static_cast<std::size_t>(kArcLengthPointsPerCurve), buffers[c], 1e-3);
      if (resampled.status != Status::kSuccess) {
        result.failure_reason = std::string("arc_length_") +
                                planner::bspline::to_string(resampled.status);
        return false;
      }
      max_error = std::max(max_error, resampled.max_interval_error_m);
      checksum += static_cast<std::uint64_t>(std::llround(
          buffers[c].front().position.x_m * 1000.0));
      checksum += static_cast<std::uint64_t>(std::llround(
          buffers[c].back().position.y_m * 1000.0));
    }
    return true;
  };

  for (int i = 0; i < warmup_batches; ++i) {
    if (!one_batch()) {
      result.failed = true;
      return result;
    }
  }
  for (int i = 0; i < measured_batches; ++i) {
    const Clock::time_point start = Clock::now();
    const bool ok = one_batch();
    const Clock::time_point end = Clock::now();
    if (!ok) {
      result.failed = true;
      return result;
    }
    result.batch_ms.push_back(elapsed_ms(start, end));
  }
  result.checksum = checksum;
  result.error_metric_m = max_error;
  finalize_case(&result);
  return result;
}

// ---------------------------------------------------------------------------
// 跨后端一致性与内部选型（计划 §3.3“数值与性能对照”）
// ---------------------------------------------------------------------------

// 跨后端一致性：以 tinyspline 为基线，在共享夹具曲线上比较
// 值/导数（0..3 阶）的最大绝对差；阈值与曲线容差一致（1e-6）。
struct EquivalenceResult {
  bool available{false};
  bool passed{false};
  std::string baseline{"tinyspline"};
  std::vector<std::string> compared;
  double max_abs_diff{0.0};
};

EquivalenceResult compute_equivalence(
    const std::map<std::string, std::map<std::string, Bspline>>& curves,
    const std::vector<GroupInput>& groups) {
  EquivalenceResult result;
  const auto baseline = curves.find("tinyspline");
  if (baseline == curves.end() || curves.size() < 2) {
    return result;  // 单后端运行：不给出一致性结论。
  }
  result.available = true;
  for (const auto& entry : curves) {
    if (entry.first == "tinyspline") {
      continue;
    }
    result.compared.push_back(entry.first);
    for (const GroupInput& group : groups) {
      const auto expected = baseline->second.find(group.name);
      const auto actual = entry.second.find(group.name);
      if (expected == baseline->second.end() ||
          actual == entry.second.end()) {
        result.available = false;
        return result;
      }
      const ParameterDomain domain = actual->second.parameter_domain();
      std::vector<double> parameters;
      parameters.reserve(260);
      for (int i = 0; i <= 256; ++i) {
        parameters.push_back(domain.min_parameter +
                             (domain.max_parameter - domain.min_parameter) *
                                 static_cast<double>(i) / 256.0);
      }
      for (const double knot : actual->second.knots()) {
        if (knot >= domain.min_parameter && knot <= domain.max_parameter) {
          parameters.push_back(knot);
        }
      }
      for (const double parameter : parameters) {
        for (int order = 0; order <= 3; ++order) {
          const EvaluationResult a = expected->second.evaluate(parameter, order);
          const EvaluationResult b = actual->second.evaluate(parameter, order);
          if (a.status != Status::kSuccess || b.status != Status::kSuccess) {
            result.available = false;
            return result;
          }
          result.max_abs_diff =
              std::max(result.max_abs_diff,
                       std::max(std::fabs(a.value.x_m - b.value.x_m),
                                std::fabs(a.value.y_m - b.value.y_m)));
        }
      }
    }
  }
  result.passed = result.available && result.max_abs_diff <= 1e-6;
  return result;
}

// 每个后端的聚合指标（用于选型排名）。
struct BackendAggregate {
  std::string name;
  bool all_cases_success{true};
  int eval_cases{0};
  int total_cases{0};
  double eval_p95_mean_ms{0.0};
  double all_p95_mean_ms{0.0};
  double max_interval_error_m{0.0};
  bool quality_gate_passed{true};
};

std::vector<BackendAggregate> aggregate_backends(
    const std::vector<CaseResult>& cases) {
  std::map<std::string, BackendAggregate> aggregates;
  std::map<std::string, double> eval_p95_sum;
  std::map<std::string, double> all_p95_sum;
  for (const CaseResult& result : cases) {
    BackendAggregate& aggregate = aggregates[result.backend];
    aggregate.name = result.backend;
    if (result.failed) {
      aggregate.all_cases_success = false;
      continue;
    }
    aggregate.total_cases += 1;
    all_p95_sum[result.backend] += result.p95_ms;
    if (result.operation == "evaluate") {
      aggregate.eval_cases += 1;
      eval_p95_sum[result.backend] += result.p95_ms;
    }
    if (result.operation == "resample_equal_arc_length") {
      aggregate.max_interval_error_m = std::max(
          aggregate.max_interval_error_m, result.error_metric_m);
      if (result.error_metric_m > 1e-3) {
        aggregate.quality_gate_passed = false;
      }
    }
  }
  std::vector<BackendAggregate> ordered;
  for (auto& entry : aggregates) {
    BackendAggregate& aggregate = entry.second;
    if (aggregate.eval_cases > 0) {
      aggregate.eval_p95_mean_ms =
          eval_p95_sum[entry.first] / static_cast<double>(aggregate.eval_cases);
    }
    if (aggregate.total_cases > 0) {
      aggregate.all_p95_mean_ms =
          all_p95_sum[entry.first] / static_cast<double>(aggregate.total_cases);
    }
    ordered.push_back(aggregate);
  }
  return ordered;
}

void write_selection_json(const std::filesystem::path& output_dir,
                          const char* run_id,
                          const std::vector<CaseResult>& cases,
                          const EquivalenceResult& equivalence) {
  const std::vector<BackendAggregate> aggregates = aggregate_backends(cases);
  nlohmann::json document;
  document["schema_version"] = 1;
  document["type"] = "bspline_backend_selection";
  document["suite"] = "v1";
  document["component"] = "bspline";
  document["run_id"] = run_id != nullptr ? run_id : "";
  document["incumbent"] = "tinyspline";
  document["rule"] = {
      {"correctness_gate",
       "cross-backend |value difference| <= 1e-6 on shared fixture curves "
       "(orders 0..3); evaluated in-run"},
      {"quality_gate", "arc-length max interval error <= 1e-3 m"},
      {"ranking",
       "equal-weight mean of per-case batch P95 over evaluation cases "
       "(3 control-point groups x 4 derivative orders); lower wins"},
      {"tie_break",
       "if the leader is within 5% of the incumbent, compare the equal-weight "
       "mean of batch P95 over all cases; still within 5% keeps the incumbent"}};
  document["equivalence"] = {
      {"available", equivalence.available},
      {"passed", equivalence.passed},
      {"baseline", equivalence.baseline},
      {"compared", equivalence.compared},
      {"max_abs_diff", equivalence.max_abs_diff},
      {"tolerance", 1e-6}};

  nlohmann::json ranking = nlohmann::json::array();
  std::vector<const BackendAggregate*> qualified;
  for (const BackendAggregate& aggregate : aggregates) {
    const bool equivalence_passed =
        !equivalence.available ||
        aggregate.name == equivalence.baseline || equivalence.passed;
    const bool qualifies = aggregate.all_cases_success &&
                           aggregate.quality_gate_passed &&
                           equivalence_passed;
    if (qualifies) {
      qualified.push_back(&aggregate);
    }
    ranking.push_back({{"backend", aggregate.name},
                       {"qualified", qualifies},
                       {"all_cases_success", aggregate.all_cases_success},
                       {"correctness_gate_passed", equivalence_passed},
                       {"quality_gate_passed", aggregate.quality_gate_passed},
                       {"max_interval_error_m", aggregate.max_interval_error_m},
                       {"eval_cases", aggregate.eval_cases},
                       {"eval_p95_mean_ms", aggregate.eval_p95_mean_ms},
                       {"all_p95_mean_ms", aggregate.all_p95_mean_ms}});
  }

  std::string default_backend;
  std::vector<std::string> pending;
  if (!qualified.empty()) {
    const BackendAggregate* best = qualified.front();
    for (const BackendAggregate* candidate : qualified) {
      if (candidate->eval_p95_mean_ms < best->eval_p95_mean_ms) {
        best = candidate;
      }
    }
    const BackendAggregate* incumbent = nullptr;
    for (const BackendAggregate* candidate : qualified) {
      if (candidate->name == "tinyspline") {
        incumbent = candidate;
      }
    }
    if (incumbent != nullptr && best != incumbent &&
        best->eval_p95_mean_ms >= 0.95 * incumbent->eval_p95_mean_ms) {
      // 差距在 5% 内：比较全体用例的 P95 等权均值；仍接近则保留基线。
      default_backend =
          best->all_p95_mean_ms < 0.95 * incumbent->all_p95_mean_ms
              ? best->name
              : incumbent->name;
    } else {
      default_backend = best->name;
    }
    for (const BackendAggregate* candidate : qualified) {
      if (candidate->name != default_backend) {
        pending.push_back(candidate->name);
      }
    }
  }
  document["default_backend"] =
      default_backend.empty() ? nlohmann::json(nullptr)
                              : nlohmann::json(default_backend);
  document["pending_optimization"] = pending;
  document["ranking"] = std::move(ranking);
  document["preliminary"] = !equivalence.available;
  if (!equivalence.available) {
    document["notes"] = {
        "single-backend run: correctness comparison and default selection are "
        "not meaningful; run --backend both for the internal selection"};
  } else if (!equivalence.passed) {
    document["notes"] = {"candidate failed the correctness gate; no default "
                         "switch recommended"};
  }

  std::ofstream out(output_dir / "selection.json");
  out << document.dump(2) << '\n';
}

std::string format_double(double value, const char* format) {
  char buffer[64];
  std::snprintf(buffer, sizeof buffer, format, value);
  return std::string(buffer);
}

void write_metrics_csv(const std::filesystem::path& output_dir,
                       const char* run_id,
                       const std::vector<CaseResult>& cases) {
  // 第 9 节固定列加组件专属列（tools/README.md 第 7.2 节）。
  // 每个测量批次一行；未测量值保持为空（绝不为 0）。失败会被保留。
  std::ofstream out(output_dir / "benchmark_metrics.csv");
  out << "run_id,case_id,backend,repeat_id,cache_mode,seed,status,"
         "failure_reason,"
         "preprocess_ms,wall_ms,cpu_ms,peak_rss_bytes,expanded_nodes,"
         "collision_checks,path_length_m,gear_changes,"
         "endpoint_position_error_m,endpoint_yaw_error_rad,valid,"
         "batch_operations,amortized_us_per_operation,"
         "operations_per_second,error_metric_m\n";

  const auto write_row = [&](const CaseResult& result, std::size_t repeat,
                             double wall_ms, bool has_wall) {
    std::vector<std::string> fields;
    fields.reserve(23);
    fields.push_back(run_id != nullptr ? run_id : "");
    fields.push_back(result.case_id);
    fields.push_back(result.backend);
    fields.push_back(std::to_string(repeat));
    fields.push_back("warm");
    fields.push_back("");
    fields.push_back(result.failed ? "failed" : "success");
    fields.push_back(result.failure_reason);
    fields.push_back("");
    fields.push_back(has_wall ? format_double(wall_ms, "%.6f") : "");
    fields.push_back("");
    fields.push_back("");
    fields.push_back("");
    fields.push_back("");
    fields.push_back("");
    fields.push_back("");
    fields.push_back("");
    fields.push_back("");
    fields.push_back(result.failed ? "false" : "true");
    fields.push_back(std::to_string(result.batch_operations));
    fields.push_back(format_double(result.amortized_us_per_operation, "%.6f"));
    fields.push_back(format_double(result.throughput_operations_per_s, "%.3f"));
    fields.push_back(result.error_metric_m >= 0.0
                         ? format_double(result.error_metric_m, "%.9g")
                         : "");
    for (std::size_t i = 0; i < fields.size(); ++i) {
      if (i > 0) {
        out << ',';
      }
      out << fields[i];
    }
    out << '\n';
  };

  for (const CaseResult& result : cases) {
    if (result.batch_ms.empty()) {
      write_row(result, 0, 0.0, false);
      continue;
    }
    for (std::size_t i = 0; i < result.batch_ms.size(); ++i) {
      write_row(result, i, result.batch_ms[i], true);
    }
  }
}

void write_results_json(const std::filesystem::path& output_dir,
                        const Fixture& fixture, const char* run_id,
                        const std::vector<CaseResult>& cases) {
  nlohmann::json document;
  document["schema_version"] = 1;
  document["type"] = "bspline_microbenchmark_results";
  document["suite"] = "v1";
  document["component"] = "bspline";
  document["library"] =
      "planner_bspline (TinySpline 0.6.0 baseline + Eigen 3.4.0 Splines "
      "candidate + EGO-Planner UniformBspline comparison backend, GPL-3.0 "
      "isolated)";
  document["run_id"] = run_id != nullptr ? run_id : "";
  document["fixture"] = fixture.path;
  document["fixture_fnv1a64"] = fixture.fnv1a64;
  {
    std::vector<std::string> backends;
    for (const CaseResult& result : cases) {
      bool known = false;
      for (const std::string& name : backends) {
        if (name == result.backend) {
          known = true;
          break;
        }
      }
      if (!known && !result.backend.empty()) {
        backends.push_back(result.backend);
      }
    }
    document["backends"] = backends;
  }
  document["timing"] = {
      {"warmup_batches", fixture.warmup_batches},
      {"measured_batches", fixture.measured_batches},
      {"execution", "sequential_single_thread"},
      {"clock", "std::chrono::steady_clock"},
      {"timed_region_excludes",
       {"input_generation", "output_buffer_allocation", "json_encoding",
        "disk_output"}}};
  document["notes"] = {
      "Batch time P50/P95 use nearest rank over the measured batches.",
      "amortized_us_per_operation = mean batch time / operations per batch; "
      "it is not a single-call P95.",
      "Construction and arc-length sampling are timed separately.",
      "Arc-length interval error is measured by numerical integration; the "
      "v1 tolerance is 1e-3 m."};

  nlohmann::json case_array = nlohmann::json::array();
  for (const CaseResult& result : cases) {
    nlohmann::json entry;
    entry["case_id"] = result.case_id;
    entry["backend"] = result.backend;
    entry["operation"] = result.operation;
    entry["control_group"] = result.group;
    if (result.derivative_order >= 0) {
      entry["derivative_order"] = result.derivative_order;
    }
    entry["curves_per_batch"] = result.curves_per_batch;
    if (result.points_per_curve > 0) {
      entry["points_per_curve"] = result.points_per_curve;
    }
    entry["batch_operations"] = result.batch_operations;
    entry["status"] = result.failed ? "failed" : "success";
    if (result.failed) {
      entry["failure_reason"] = result.failure_reason;
    }
    entry["batch_times_ms"] = result.batch_ms;
    entry["p50_ms"] = result.p50_ms;
    entry["p95_ms"] = result.p95_ms;
    entry["amortized_us_per_operation"] = result.amortized_us_per_operation;
    entry["throughput_operations_per_s"] =
        result.throughput_operations_per_s;
    if (result.error_metric_m >= 0.0) {
      entry["max_interval_error_m"] = result.error_metric_m;
    }
    entry["checksum"] = result.checksum;
    case_array.push_back(std::move(entry));
  }
  document["cases"] = std::move(case_array);

  std::ofstream out(output_dir / "bspline_benchmark_results.json");
  out << document.dump(2) << '\n';
}

void write_latency_plot(const std::filesystem::path& output_dir,
                        const std::vector<CaseResult>& cases) {
  const std::filesystem::path plots_dir = output_dir / "plots";
  std::filesystem::create_directories(plots_dir);
  const std::filesystem::path csv_path = plots_dir / "bspline_latency_p50_p95.csv";
  std::ofstream csv(csv_path);
  csv << "case_id,operation,control_group,batch_operations,p50_ms,p95_ms,"
         "amortized_us_per_operation,throughput_operations_per_s\n";
  char line[256];
  for (const CaseResult& result : cases) {
    std::snprintf(line, sizeof line, "%s,%s,%s,%zu,%.6f,%.6f,%.6f,%.3f\n",
                  result.case_id.c_str(), result.operation.c_str(),
                  result.group.c_str(), result.batch_operations,
                  result.p50_ms, result.p95_ms,
                  result.amortized_us_per_operation,
                  result.throughput_operations_per_s);
    csv << line;
  }

  const int count = static_cast<int>(cases.size());
  const int width = std::max(720, 46 * count + 160);
  const int height = 420;
  const int plot_top = 40;
  const int plot_bottom = 320;
  double max_p95 = 0.0;
  for (const CaseResult& result : cases) {
    max_p95 = std::max(max_p95, result.p95_ms);
  }
  if (max_p95 <= 0.0) {
    max_p95 = 1.0;
  }
  const auto bar_height = [&](double value) {
    return static_cast<double>(plot_bottom - plot_top) * value / max_p95;
  };

  std::ofstream svg(plots_dir / "bspline_latency_p50_p95.svg");
  svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width
      << "\" height=\"" << height << "\">\n";
  svg << "<rect width=\"100%\" height=\"100%\" fill=\"white\"/>\n";
  svg << "<text x=\"16\" y=\"24\" font-family=\"sans-serif\" font-size=\"16\">"
         "planner_bspline v1 batch latency (P50/P95, ms) - labels carry the "
         "values; color is not the only encoding</text>\n";
  svg << "<line x1=\"50\" y1=\"" << plot_bottom << "\" x2=\"" << width - 10
      << "\" y2=\"" << plot_bottom << "\" stroke=\"black\"/>\n";
  svg << "<line x1=\"50\" y1=\"" << plot_top << "\" x2=\"50\" y2=\""
      << plot_bottom << "\" stroke=\"black\"/>\n";
  svg << "<text x=\"8\" y=\"" << plot_top + 10
      << "\" font-family=\"sans-serif\" font-size=\"11\">" << max_p95
      << " ms</text>\n";
  svg << "<text x=\"8\" y=\"" << plot_bottom
      << "\" font-family=\"sans-serif\" font-size=\"11\">0</text>\n";
  for (int i = 0; i < count; ++i) {
    const CaseResult& result = cases[static_cast<std::size_t>(i)];
    const double x = 60.0 + 46.0 * i;
    const double p50_h = bar_height(result.p50_ms);
    const double p95_h = bar_height(result.p95_ms);
    svg << "<rect x=\"" << x << "\" y=\"" << plot_bottom - p95_h
        << "\" width=\"14\" height=\"" << std::max(p95_h, 0.5)
        << "\" fill=\"#9ecae1\" stroke=\"#3182bd\"/>\n";
    svg << "<rect x=\"" << x + 14 << "\" y=\"" << plot_bottom - p50_h
        << "\" width=\"14\" height=\"" << std::max(p50_h, 0.5)
        << "\" fill=\"#fdae6b\" stroke=\"#e6550d\"/>\n";
    char value_label[64];
    std::snprintf(value_label, sizeof value_label, "%.3f/%.3f",
                  result.p50_ms, result.p95_ms);
    svg << "<text x=\"" << x << "\" y=\"" << plot_bottom - p95_h - 6
        << "\" font-family=\"sans-serif\" font-size=\"9\">" << value_label
        << "</text>\n";
    svg << "<text x=\"" << x << "\" y=\"" << plot_bottom + 12
        << "\" font-family=\"sans-serif\" font-size=\"9\" "
           "transform=\"rotate(60 "
        << x << ' ' << plot_bottom + 12 << ")\">" << result.case_id
        << "</text>\n";
  }
  svg << "<text x=\"" << width - 260 << "\" y=\"" << height - 60
      << "\" font-family=\"sans-serif\" font-size=\"11\">legend: light bar = "
         "P95, dark bar = P50 (values printed above each pair)</text>\n";
  svg << "</svg>\n";
}

void print_summary(const std::vector<CaseResult>& cases) {
  std::printf("%-24s %10s %12s %12s %14s %16s\n", "case", "ops/batch", "P50 ms",
              "P95 ms", "amortized us", "throughput/s");
  for (const CaseResult& result : cases) {
    std::printf("%-24s %10zu %12.6f %12.6f %14.6f %16.3f\n",
                result.case_id.c_str(), result.batch_operations,
                result.p50_ms, result.p95_ms,
                result.amortized_us_per_operation,
                result.throughput_operations_per_s);
    if (result.error_metric_m >= 0.0) {
      std::printf("  arc-length max interval error: %.3e m\n",
                  result.error_metric_m);
    }
    if (result.failed) {
      std::printf("  FAILED: %s\n", result.failure_reason.c_str());
    }
  }
}

void print_usage(const char* program) {
  std::printf(
      "usage: %s --suite v1 --output-dir <dir> [--fixture <inputs.json>] "
      "[--backend all|both|tinyspline|eigen|ego]\n",
      program);
}

}  // namespace

int main(int argc, char** argv) {
  std::string suite = "v1";
  std::string output_dir;
  std::string fixture_path = PLANNER_BSPLINE_BENCH_INPUTS;
  std::string backend_option = "all";
  for (int i = 1; i < argc; ++i) {
    const std::string argument = argv[i];
    if (argument == "--suite" && i + 1 < argc) {
      suite = argv[++i];
    } else if (argument == "--output-dir" && i + 1 < argc) {
      output_dir = argv[++i];
    } else if (argument == "--fixture" && i + 1 < argc) {
      fixture_path = argv[++i];
    } else if (argument == "--backend" && i + 1 < argc) {
      backend_option = argv[++i];
    } else if (argument == "--help" || argument == "-h") {
      print_usage(argv[0]);
      return 0;
    } else {
      std::fprintf(stderr, "error: unexpected argument '%s'\n", argument.c_str());
      print_usage(argv[0]);
      return 2;
    }
  }
  if (suite != "v1") {
    std::fprintf(stderr, "error: unsupported suite '%s' (only v1)\n",
                 suite.c_str());
    return 65;
  }
  if (output_dir.empty()) {
    std::fprintf(stderr, "error: --output-dir is required\n");
    print_usage(argv[0]);
    return 2;
  }

  std::vector<BackendSpec> backends;
  if (backend_option == "all") {
    backends = {kTinysplineBackend, kEigenBackend, kEgoBackend};
  } else if (backend_option == "both") {
    backends = {kTinysplineBackend, kEigenBackend};
  } else if (backend_option == "tinyspline") {
    backends = {kTinysplineBackend};
  } else if (backend_option == "eigen") {
    backends = {kEigenBackend};
  } else if (backend_option == "ego") {
    backends = {kEgoBackend};
  } else {
    std::fprintf(stderr, "error: unsupported --backend '%s'\n",
                 backend_option.c_str());
    print_usage(argv[0]);
    return 2;
  }
  // EGO 对照后端（GPL-3.0 隔离库）需在使用前注册。
  bool needs_ego = false;
  for (const BackendSpec& backend : backends) {
    needs_ego = needs_ego || backend.kind == BackendKind::kEgoUniform;
  }
  if (needs_ego) {
    planner::bspline::detail::install_ego_backend();
  }

  Fixture fixture;
  if (!read_fixture(fixture_path, &fixture)) {
    return 65;
  }
  std::error_code directory_error;
  std::filesystem::create_directories(output_dir, directory_error);
  if (directory_error) {
    std::fprintf(stderr, "error: cannot create output dir '%s': %s\n",
                 output_dir.c_str(), directory_error.message().c_str());
    return 70;
  }

  const char* run_id = std::getenv("PLANNER_RUN_ID");

  // 共享的 10,000 个域内等距参数 t_j = j / 9999。
  std::vector<double> parameters(kEvalParameterCount);
  for (int j = 0; j < kEvalParameterCount; ++j) {
    parameters[static_cast<std::size_t>(j)] =
        static_cast<double>(j) / (kEvalParameterCount - 1.0);
  }

  std::vector<CaseResult> cases;
  bool failed = false;
  // 每个后端、每个控制点组的基准曲线，用于跨后端一致性检查。
  std::map<std::string, std::map<std::string, Bspline>> backend_curves;
  for (const BackendSpec& backend : backends) {
    for (const GroupInput& group : fixture.groups) {
      std::vector<Bspline> curves;
      CaseResult construction =
          run_construction_case(group, backend, fixture.warmup_batches,
                                fixture.measured_batches, &curves);
      failed = failed || construction.failed;
      cases.push_back(std::move(construction));

      if (curves.empty() || cases.back().failed) {
        CaseResult skipped;
        skipped.case_id =
            std::string(backend.name) + "/group-" + group.name + "-skipped";
        skipped.operation = "none";
        skipped.group = group.name;
        skipped.backend = backend.name;
        skipped.failed = true;
        skipped.failure_reason = "construction_unavailable";
        cases.push_back(std::move(skipped));
        failed = true;
        continue;
      }

      const std::optional<Bspline> base =
          Bspline::create(group.base_points, group.knots, backend.kind);
      if (!base.has_value()) {
        failed = true;
        CaseResult broken;
        broken.case_id = std::string(backend.name) + "/eval-" + group.name;
        broken.group = group.name;
        broken.backend = backend.name;
        broken.failed = true;
        broken.failure_reason = "base_curve_construction_failed";
        cases.push_back(std::move(broken));
      } else {
        backend_curves[backend.name].insert_or_assign(group.name, *base);
        for (int order = 0; order <= 3; ++order) {
          CaseResult evaluation = run_evaluation_case(
              group, *base, backend, order, parameters,
              fixture.warmup_batches, fixture.measured_batches);
          failed = failed || evaluation.failed;
          cases.push_back(std::move(evaluation));
        }
      }

      CaseResult arc_length =
          run_arc_length_case(group, curves, backend, fixture.warmup_batches,
                              fixture.measured_batches);
      failed = failed || arc_length.failed;
      cases.push_back(std::move(arc_length));
    }
  }

  const EquivalenceResult equivalence =
      compute_equivalence(backend_curves, fixture.groups);

  write_metrics_csv(output_dir, run_id, cases);
  write_results_json(output_dir, fixture, run_id, cases);
  write_latency_plot(output_dir, cases);
  write_selection_json(output_dir, run_id, cases, equivalence);
  print_summary(cases);

  if (failed) {
    std::fprintf(stderr, "benchmark completed with failures; metrics preserved\n");
    return 1;
  }
  std::printf("benchmark completed: %zu cases\n", cases.size());
  return 0;
}
