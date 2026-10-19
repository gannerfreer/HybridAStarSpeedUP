// planner_polynomial_bench：三次/五次多项式组件的微基准测试
// （Docs/component_development_plan.md 第 7.3 节，夹具
// tests/fixtures/v1/benchmarks/polynomial/inputs.json）。
//
// 测量负载（每组 5 个预热批次、随后 30 个测量批次，
// 顺序单线程执行；输入生成、缓冲区分配、JSON
// 解析和磁盘输出均在计时区间之外）：
//   * construct_cubic / construct_quintic：每批求解 1,000 个分段；
//   * eval_cubic_d0..d3 / eval_quintic_d0..d3：每批 10,000 个预分配的
//     局部时间点，阶数 0..3，输出到预分配缓冲区。
//
// 内部对照（计划 §3.4）：--backend both|horner|mav 增加
// mav_trajectory_generation 求值候选（同一系数与时间点），并写
// outputs/selection.json；构造/求解对比为 N/A（上游没有等价的闭式
// 边界求解 API），由测试报告单独给证据。
//
// 输出（tools/README.md 第 7.2 节 "基准输出契约"）：
//   <output-dir>/benchmark_metrics.csv   每个测量批次试验一行
//   <output-dir>/polynomial_bench_summary.json  各用例的 P50/P95 和校验和
//   <output-dir>/selection.json          后端对照与选型建议（both 时）
//   <output-dir>/plots/batch_wall_ms_by_case.csv / .svg
//
// 退出码：0 成功，2 用法错误，65 夹具/数据错误，
// 70 基准用例求值失败。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "mav_segment_evaluator.hpp"
#include "planner/polynomial/piecewise_trajectory.hpp"
#include "planner/polynomial/polynomial.hpp"
#include "planner/polynomial/trajectory_segment.hpp"

#ifndef PLANNER_POLYNOMIAL_BENCH_FIXTURE
#error "PLANNER_POLYNOMIAL_BENCH_FIXTURE must be defined by CMake (absolute fixture path)"
#endif

namespace {

using planner::common::Point2d;
using planner::polynomial::BatchResult;
using planner::polynomial::Code;
using planner::polynomial::compare::MavSegmentEvaluator;
using planner::polynomial::CubicBoundary;
using planner::polynomial::CubicBoundary2d;
using planner::polynomial::Polynomial;
using planner::polynomial::QuinticBoundary;
using planner::polynomial::QuinticBoundary2d;
using planner::polynomial::Result;
using planner::polynomial::TrajectorySegment;

using Json = nlohmann::json;
using Clock = std::chrono::steady_clock;

// 内部对照的后端名（写入指标与 selection.json）。
constexpr const char* kHornerBackend = "planner_horner";
constexpr const char* kMavBackend = "mav_trajectory_generation";

constexpr const char* kUsage =
    "usage: planner_polynomial_bench --suite v1 --output-dir <dir> "
    "[--fixture <path>] [--backend both|horner|mav]\n";

double ms_since(Clock::time_point start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

double nearest_rank(const std::vector<double>& values, double percent) {
  std::vector<double> ordered = values;
  std::sort(ordered.begin(), ordered.end());
  if (ordered.empty()) {
    return 0.0;
  }
  std::size_t rank = static_cast<std::size_t>(
      std::ceil(percent / 100.0 * static_cast<double>(ordered.size())));
  rank = std::max<std::size_t>(1, std::min(rank, ordered.size()));
  return ordered[rank - 1];
}

// ---------------------------------------------------------------------------
// 夹具解析与确定性输入生成（在计时区间之外）。
// 规则遵循夹具中的 boundary_condition_sets。
// ---------------------------------------------------------------------------

CubicBoundary2d make_cubic_boundary(std::size_t k) {
  const double kd = static_cast<double>(k);
  const int m8 = static_cast<int>(k % 8);
  const int m5 = static_cast<int>(k % 5);
  const int m3 = static_cast<int>(k % 3);
  const int m2 = static_cast<int>(k % 2);
  CubicBoundary2d boundary;
  boundary.p0_m = Point2d{0.5 * kd, 0.25 * static_cast<double>(m8 - 4)};
  boundary.v0_mps = Point2d{1.0, 0.5};
  boundary.p1_m = Point2d{0.5 * kd + 1.0 + 0.25 * static_cast<double>(m5),
                          0.25 * static_cast<double>(m8 - 2)};
  boundary.v1_mps = Point2d{1.0 + 0.25 * static_cast<double>(m3),
                            0.5 + 0.25 * static_cast<double>(m2)};
  return boundary;
}

QuinticBoundary2d make_quintic_boundary(std::size_t k) {
  const double kd = static_cast<double>(k);
  const int m8 = static_cast<int>(k % 8);
  const int m4 = static_cast<int>(k % 4);
  QuinticBoundary2d boundary;
  boundary.p0_m = Point2d{0.5 * kd, 0.25 * static_cast<double>(m8 - 4)};
  boundary.v0_mps = Point2d{0.25 * static_cast<double>(m8 - 4), 0.5};
  boundary.a0_mps2 = Point2d{0.5, 0.0};
  boundary.p1_m = Point2d{0.5 * kd + 2.0, 0.25 * static_cast<double>(m8 - 2)};
  boundary.v1_mps = Point2d{0.5, 0.25 * static_cast<double>(m4 - 2)};
  boundary.a1_mps2 = Point2d{-0.5, 0.0};
  return boundary;
}

struct FixtureInputs {
  std::size_t cubic_segment_count = 0;
  std::size_t quintic_segment_count = 0;
  std::size_t eval_time_point_count = 0;
  int warmup_batches = 0;
  int measured_batches = 0;
  double segment_duration_s = 1.0;
  std::vector<CubicBoundary2d> cubic_boundaries;
  std::vector<QuinticBoundary2d> quintic_boundaries;
  std::vector<double> local_times;  // eval_time_point_count / segment_count 个点
  std::string fixture_path;
};

bool parse_fixture(const std::string& path, FixtureInputs* inputs, std::string* error) {
  std::ifstream stream(path);
  if (!stream) {
    *error = "cannot open fixture: " + path;
    return false;
  }
  Json document;
  try {
    stream >> document;
  } catch (const std::exception& exc) {
    *error = std::string("fixture is not valid JSON: ") + exc.what();
    return false;
  }
  try {
    if (document.at("schema_version").get<int>() != 1 ||
        document.at("type").get<std::string>() != "polynomial_benchmark_inputs") {
      *error = "fixture schema_version/type mismatch";
      return false;
    }
    const Json& sets = document.at("boundary_condition_sets");
    inputs->cubic_segment_count = sets.at("cubic").at("segment_count").get<std::size_t>();
    inputs->quintic_segment_count = sets.at("quintic").at("segment_count").get<std::size_t>();
    inputs->segment_duration_s = document.at("eval_batch").at("cubic").contains("duration_s")
                                      ? document.at("eval_batch").at("cubic").at("duration_s").get<double>()
                                      : 1.0;
    inputs->eval_time_point_count = document.at("eval_batch").at("cubic").at("time_point_count").get<std::size_t>();
    inputs->warmup_batches = document.at("timing_boundaries").at("warmup_batches").get<int>();
    inputs->measured_batches = document.at("timing_boundaries").at("measured_batches").get<int>();
  } catch (const std::exception& exc) {
    *error = std::string("fixture field error: ") + exc.what();
    return false;
  }
  if (inputs->cubic_segment_count == 0 || inputs->quintic_segment_count == 0 ||
      inputs->eval_time_point_count == 0 || inputs->eval_time_point_count % inputs->cubic_segment_count != 0 ||
      inputs->warmup_batches <= 0 || inputs->measured_batches <= 0) {
    *error = "fixture declares inconsistent batch sizes";
    return false;
  }

  // 分布规则：每个分段 k 取 u = (j + 0.5) / points_per_segment。
  const std::size_t points_per_segment = inputs->eval_time_point_count / inputs->cubic_segment_count;
  inputs->local_times.resize(points_per_segment);
  for (std::size_t j = 0; j < points_per_segment; ++j) {
    inputs->local_times[j] =
        (static_cast<double>(j) + 0.5) / static_cast<double>(points_per_segment);
  }

  inputs->cubic_boundaries.resize(inputs->cubic_segment_count);
  for (std::size_t k = 0; k < inputs->cubic_segment_count; ++k) {
    inputs->cubic_boundaries[k] = make_cubic_boundary(k);
  }
  inputs->quintic_boundaries.resize(inputs->quintic_segment_count);
  for (std::size_t k = 0; k < inputs->quintic_segment_count; ++k) {
    inputs->quintic_boundaries[k] = make_quintic_boundary(k);
  }
  inputs->fixture_path = path;
  return true;
}

// 针对 validation_examples 的自洽性预检（在计时之外）。
bool precheck_conversion_factor(const Json& document, std::string* error) {
  constexpr double kTolerance = 1e-12;
  try {
    for (const Json& example : document.at("validation_examples")) {
      const Json& boundary = example.at("boundary");
      const std::string degree = example.at("degree").get<int>() == 3 ? "cubic" : "quintic";
      std::vector<double> expected =
          example.at("expected_coefficients_ascending").get<std::vector<double>>();
      std::vector<double> actual;
      if (degree == "cubic") {
        const Result<Polynomial> solved = Polynomial::cubic(
            CubicBoundary{boundary.at("p0").get<double>(), boundary.at("v0").get<double>(),
                          boundary.at("p1").get<double>(), boundary.at("v1").get<double>()},
            1.0);
        if (!solved.ok()) {
          *error = "validation example '" + example.at("id").get<std::string>() +
                   "' failed to solve";
          return false;
        }
        actual = solved.value.coefficients();
      } else {
        const Result<Polynomial> solved = Polynomial::quintic(
            QuinticBoundary{boundary.at("p0").get<double>(), boundary.at("v0").get<double>(),
                            boundary.at("a0").get<double>(), boundary.at("p1").get<double>(),
                            boundary.at("v1").get<double>(), boundary.at("a1").get<double>()},
            1.0);
        if (!solved.ok()) {
          *error = "validation example '" + example.at("id").get<std::string>() +
                   "' failed to solve";
          return false;
        }
        actual = solved.value.coefficients();
      }
      if (actual.size() != expected.size()) {
        *error = "validation example coefficient count mismatch";
        return false;
      }
      for (std::size_t i = 0; i < actual.size(); ++i) {
        if (std::fabs(actual[i] - expected[i]) > kTolerance) {
          *error = "validation example coefficient mismatch at index " + std::to_string(i);
          return false;
        }
      }
    }
  } catch (const std::exception& exc) {
    *error = std::string("validation_examples field error: ") + exc.what();
    return false;
  }
  return true;
}

// ---------------------------------------------------------------------------
// 负载。
// ---------------------------------------------------------------------------

struct CaseSummary {
  std::string case_id;
  std::string backend;  // planner_horner 或 mav_trajectory_generation
  std::string kind;  // "construct" 或 "eval"
  std::string degree;  // "cubic" 或 "quintic"
  int derivative_order = -1;  // 构造类为 -1
  std::size_t batch_size = 0;
  std::vector<double> wall_ms;
  double checksum = 0.0;
  bool valid = true;
  std::string failure_reason;
};

struct BenchmarkOutputs {
  std::vector<TrajectorySegment> cubic_segments;
  std::vector<TrajectorySegment> quintic_segments;
  std::vector<Point2d> eval_buffer;
  // 内部对照候选（mav_trajectory_generation 求值核心）。
  std::vector<MavSegmentEvaluator> mav_cubic_segments;
  std::vector<MavSegmentEvaluator> mav_quintic_segments;
  bool mav_available = false;
  std::string mav_failure_reason;
  // 同一系数/时间点上候选与基线的最大绝对差（全阶 0..3）。
  bool equivalence_available = false;
  double equivalence_max_abs_diff = 0.0;
};

double run_construct_case(const std::string& degree,
                          const std::vector<CubicBoundary2d>& cubic_boundaries,
                          const std::vector<QuinticBoundary2d>& quintic_boundaries,
                          double duration_s,
                          std::vector<TrajectorySegment>* segments, bool* batch_ok,
                          std::string* failure_reason) {
  const auto start = Clock::now();
  const std::size_t count = degree == "cubic" ? cubic_boundaries.size() : quintic_boundaries.size();
  for (std::size_t k = 0; k < count; ++k) {
    if (degree == "cubic") {
      Result<TrajectorySegment> solved =
          TrajectorySegment::cubic(cubic_boundaries[k], duration_s);
      if (!solved.ok()) {
        *batch_ok = false;
        *failure_reason = std::string("cubic_solve_") + planner::polynomial::to_string(solved.code);
        continue;
      }
      (*segments)[k] = std::move(solved.value);
    } else {
      Result<TrajectorySegment> solved =
          TrajectorySegment::quintic(quintic_boundaries[k], duration_s);
      if (!solved.ok()) {
        *batch_ok = false;
        *failure_reason = std::string("quintic_solve_") +
                          planner::polynomial::to_string(solved.code);
        continue;
      }
      (*segments)[k] = std::move(solved.value);
    }
  }
  return ms_since(start);
}

double run_eval_case(const std::vector<TrajectorySegment>& segments,
                     const std::vector<double>& local_times, int derivative_order,
                     std::vector<Point2d>* output, bool* batch_ok,
                     std::string* failure_reason) {
  const std::size_t points_per_segment = local_times.size();
  const auto start = Clock::now();
  for (std::size_t k = 0; k < segments.size(); ++k) {
    const BatchResult result = segments[k].evaluate_batch(
        local_times.data(), points_per_segment, derivative_order,
        output->data() + k * points_per_segment);
    if (!result.ok()) {
      *batch_ok = false;
      *failure_reason = std::string("evaluate_batch_") +
                        planner::polynomial::to_string(result.code);
    }
  }
  return ms_since(start);
}

// 候选（mav_trajectory_generation）求值：与基线相同的分段/点数布局。
double run_mav_eval_case(
    const std::vector<MavSegmentEvaluator>& evaluators,
    const std::vector<double>& local_times, int derivative_order,
    std::vector<Point2d>* output, bool* batch_ok, std::string* failure_reason) {
  const std::size_t points_per_segment = local_times.size();
  const auto start = Clock::now();
  for (std::size_t k = 0; k < evaluators.size(); ++k) {
    if (!evaluators[k].evaluate_batch(local_times.data(), points_per_segment,
                                      derivative_order,
                                      output->data() + k * points_per_segment)) {
      *batch_ok = false;
      *failure_reason = "mav_evaluate_batch_failed";
    }
  }
  return ms_since(start);
}

// 候选与基线的数值一致性（计时区间之外）：同一系数、同一时间点、
// 阶数 0..3；返回最大绝对差。
bool compute_eval_equivalence(
    const std::vector<TrajectorySegment>& segments,
    const std::vector<MavSegmentEvaluator>& evaluators,
    const std::vector<double>& local_times, double* max_abs_diff) {
  *max_abs_diff = 0.0;
  if (segments.size() != evaluators.size()) {
    return false;
  }
  const std::size_t points_per_segment = local_times.size();
  for (std::size_t k = 0; k < segments.size(); ++k) {
    for (int order = 0; order <= 3; ++order) {
      for (std::size_t j = 0; j < points_per_segment; ++j) {
        const Result<Point2d> expected =
            segments[k].evaluate(local_times[j], order);
        if (!expected.ok()) {
          return false;
        }
        Point2d actual;
        if (!evaluators[k].evaluate(local_times[j], order, &actual)) {
          return false;
        }
        *max_abs_diff =
            std::max(*max_abs_diff,
                     std::max(std::fabs(actual.x_m - expected.value.x_m),
                              std::fabs(actual.y_m - expected.value.y_m)));
      }
    }
  }
  return true;
}

double checksum_segments(const std::vector<TrajectorySegment>& segments) {
  double checksum = 0.0;
  for (const TrajectorySegment& segment : segments) {
    checksum += segment.x_polynomial().evaluate(0.5);
    checksum += segment.y_polynomial().evaluate(0.5);
  }
  return checksum;
}

double checksum_points(const std::vector<Point2d>& points) {
  double checksum = 0.0;
  for (const Point2d& point : points) {
    checksum += point.x_m + point.y_m;
  }
  return checksum;
}

// 构建所有测量用例，运行预热批次和测量批次。
// 按固定报告顺序返回用例摘要。run_mav 为 true 时增加对照候选用例。
std::vector<CaseSummary> run_suite(const FixtureInputs& inputs, bool run_mav,
                                   BenchmarkOutputs* outputs) {
  std::vector<CaseSummary> cases;
  const double duration_s = inputs.segment_duration_s;

  const auto make_case = [&cases](const std::string& id,
                                  const std::string& backend,
                                  const std::string& kind,
                                  const std::string& degree, int order,
                                  std::size_t batch_size) {
    CaseSummary summary;
    summary.case_id = id;
    summary.backend = backend;
    summary.kind = kind;
    summary.degree = degree;
    summary.derivative_order = order;
    summary.batch_size = batch_size;
    cases.push_back(summary);
    return cases.size() - 1;
  };

  // 构造类用例（仅基线：mav_trajectory_generation 没有等价的闭式边界
  // 求解 API，构造对比为 N/A）。
  const std::size_t construct_cubic = make_case(
      "construct_cubic", kHornerBackend, "construct", "cubic", -1,
      inputs.cubic_boundaries.size());
  const std::size_t construct_quintic = make_case(
      "construct_quintic", kHornerBackend, "construct", "quintic", -1,
      inputs.quintic_boundaries.size());
  // 基线求值用例。
  std::size_t eval_cubic[4];
  std::size_t eval_quintic[4];
  for (int order = 0; order <= 3; ++order) {
    eval_cubic[order] =
        make_case("eval_cubic_d" + std::to_string(order), kHornerBackend,
                  "eval", "cubic", order, inputs.eval_time_point_count);
    eval_quintic[order] =
        make_case("eval_quintic_d" + std::to_string(order), kHornerBackend,
                  "eval", "quintic", order, inputs.eval_time_point_count);
  }
  // 对照候选求值用例（与基线相同的分段/点数/阶数）。
  std::size_t mav_eval_cubic[4];
  std::size_t mav_eval_quintic[4];
  if (run_mav) {
    for (int order = 0; order <= 3; ++order) {
      mav_eval_cubic[order] =
          make_case("mav_eval_cubic_d" + std::to_string(order), kMavBackend,
                    "eval", "cubic", order, inputs.eval_time_point_count);
      mav_eval_quintic[order] =
          make_case("mav_eval_quintic_d" + std::to_string(order), kMavBackend,
                    "eval", "quintic", order, inputs.eval_time_point_count);
    }
  }

  outputs->cubic_segments.assign(inputs.cubic_boundaries.size(), TrajectorySegment{});
  outputs->quintic_segments.assign(inputs.quintic_boundaries.size(), TrajectorySegment{});
  outputs->eval_buffer.assign(inputs.eval_time_point_count, Point2d{});

  const auto run_batches = [&](std::size_t case_index, auto&& run_once) {
    CaseSummary& summary = cases[case_index];
    for (int batch = 0; batch < inputs.warmup_batches && summary.valid; ++batch) {
      bool ok = true;
      std::string reason;
      run_once(&ok, &reason);
      if (!ok) {
        summary.valid = false;
        summary.failure_reason = reason;
      }
    }
    summary.wall_ms.reserve(static_cast<std::size_t>(inputs.measured_batches));
    for (int batch = 0; batch < inputs.measured_batches && summary.valid; ++batch) {
      bool ok = true;
      std::string reason;
      const double elapsed = run_once(&ok, &reason);
      if (!ok) {
        summary.valid = false;
        summary.failure_reason = reason;
        break;
      }
      summary.wall_ms.push_back(elapsed);
    }
    if (!summary.valid) {
      // 保留证据：失败的试验保留失败前已测到的挂钟时间，
      // 该用例报告为失败。
      return;
    }
    summary.checksum = 0.0;
  };

  run_batches(construct_cubic, [&](bool* ok, std::string* reason) {
    const double elapsed = run_construct_case("cubic", inputs.cubic_boundaries,
                                              inputs.quintic_boundaries, duration_s,
                                              &outputs->cubic_segments, ok, reason);
    return elapsed;
  });
  if (cases[construct_cubic].valid) {
    cases[construct_cubic].checksum = checksum_segments(outputs->cubic_segments);
  }

  run_batches(construct_quintic, [&](bool* ok, std::string* reason) {
    const double elapsed = run_construct_case("quintic", inputs.cubic_boundaries,
                                              inputs.quintic_boundaries, duration_s,
                                              &outputs->quintic_segments, ok, reason);
    return elapsed;
  });
  if (cases[construct_quintic].valid) {
    cases[construct_quintic].checksum = checksum_segments(outputs->quintic_segments);
  }

  for (int order = 0; order <= 3; ++order) {
    run_batches(eval_cubic[order], [&](bool* ok, std::string* reason) {
      return run_eval_case(outputs->cubic_segments, inputs.local_times, order,
                           &outputs->eval_buffer, ok, reason);
    });
    if (cases[eval_cubic[order]].valid) {
      cases[eval_cubic[order]].checksum = checksum_points(outputs->eval_buffer);
    }
    run_batches(eval_quintic[order], [&](bool* ok, std::string* reason) {
      return run_eval_case(outputs->quintic_segments, inputs.local_times, order,
                           &outputs->eval_buffer, ok, reason);
    });
    if (cases[eval_quintic[order]].valid) {
      cases[eval_quintic[order]].checksum = checksum_points(outputs->eval_buffer);
    }
  }

  if (!run_mav) {
    return cases;
  }

  // 从基线系数构建候选求值器（计时区间之外）。
  outputs->mav_cubic_segments.assign(outputs->cubic_segments.size(),
                                     MavSegmentEvaluator{});
  outputs->mav_quintic_segments.assign(outputs->quintic_segments.size(),
                                       MavSegmentEvaluator{});
  bool mav_available = cases[construct_cubic].valid && cases[construct_quintic].valid;
  std::string mav_failure_reason;
  if (mav_available) {
    for (std::size_t k = 0; k < outputs->cubic_segments.size(); ++k) {
      const TrajectorySegment& segment = outputs->cubic_segments[k];
      if (!MavSegmentEvaluator::create(
              segment.x_polynomial().coefficients(),
              segment.y_polynomial().coefficients(), segment.duration_s(),
              &outputs->mav_cubic_segments[k])) {
        mav_available = false;
        mav_failure_reason = "mav_cubic_create_failed";
        break;
      }
    }
  }
  if (mav_available) {
    for (std::size_t k = 0; k < outputs->quintic_segments.size(); ++k) {
      const TrajectorySegment& segment = outputs->quintic_segments[k];
      if (!MavSegmentEvaluator::create(
              segment.x_polynomial().coefficients(),
              segment.y_polynomial().coefficients(), segment.duration_s(),
              &outputs->mav_quintic_segments[k])) {
        mav_available = false;
        mav_failure_reason = "mav_quintic_create_failed";
        break;
      }
    }
  }
  outputs->mav_available = mav_available;
  outputs->mav_failure_reason = mav_failure_reason;

  if (!mav_available) {
    // 候选不可用时也保留失败证据（未测量值留空）。
    for (int order = 0; order <= 3; ++order) {
      for (const std::size_t index : {mav_eval_cubic[order], mav_eval_quintic[order]}) {
        cases[index].valid = false;
        cases[index].failure_reason =
            mav_failure_reason.empty() ? "mav_unavailable" : mav_failure_reason;
      }
    }
    return cases;
  }

  for (int order = 0; order <= 3; ++order) {
    run_batches(mav_eval_cubic[order], [&](bool* ok, std::string* reason) {
      return run_mav_eval_case(outputs->mav_cubic_segments, inputs.local_times,
                               order, &outputs->eval_buffer, ok, reason);
    });
    if (cases[mav_eval_cubic[order]].valid) {
      cases[mav_eval_cubic[order]].checksum =
          checksum_points(outputs->eval_buffer);
    }
    run_batches(mav_eval_quintic[order], [&](bool* ok, std::string* reason) {
      return run_mav_eval_case(outputs->mav_quintic_segments, inputs.local_times,
                               order, &outputs->eval_buffer, ok, reason);
    });
    if (cases[mav_eval_quintic[order]].valid) {
      cases[mav_eval_quintic[order]].checksum =
          checksum_points(outputs->eval_buffer);
    }
  }

  // 数值一致性（计时区间之外；全部分段 x 时间点 x 阶数 0..3）。
  double diff_cubic = 0.0;
  double diff_quintic = 0.0;
  const bool equivalence_cubic =
      compute_eval_equivalence(outputs->cubic_segments,
                               outputs->mav_cubic_segments,
                               inputs.local_times, &diff_cubic);
  const bool equivalence_quintic =
      compute_eval_equivalence(outputs->quintic_segments,
                               outputs->mav_quintic_segments,
                               inputs.local_times, &diff_quintic);
  outputs->equivalence_available = equivalence_cubic && equivalence_quintic;
  outputs->equivalence_max_abs_diff = std::max(diff_cubic, diff_quintic);
  return cases;
}

// ---------------------------------------------------------------------------
// 输出契约写出器。
// ---------------------------------------------------------------------------

std::string csv_field(const std::string& value) {
  if (value.find_first_of(",\"\n") == std::string::npos) {
    return value;
  }
  std::string escaped = "\"";
  for (char c : value) {
    escaped += c;
    if (c == '"') {
      escaped += '"';
    }
  }
  escaped += '"';
  return escaped;
}

std::string format_number(double value) {
  std::ostringstream stream;
  stream << std::setprecision(10) << value;
  return stream.str();
}

bool write_metrics_csv(const std::filesystem::path& path, const std::vector<CaseSummary>& cases,
                       const std::string& run_id) {
  // 固定的第 9 节列加上组件专属列（任务运行器把文件合并到
  // metrics.csv 时会忽略未知列）。
  const char* header =
      "run_id,case_id,backend,repeat_id,cache_mode,seed,status,failure_reason,preprocess_ms,"
      "wall_ms,cpu_ms,peak_rss_bytes,expanded_nodes,collision_checks,path_length_m,"
      "gear_changes,endpoint_position_error_m,endpoint_yaw_error_rad,valid,"
      "degree,derivative_order,batch_size,ops_per_s,amortized_ms_per_op";
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    return false;
  }
  stream << header << "\n";
  for (const CaseSummary& summary : cases) {
    // 在任何测量批次之前就失败的用例仍写一行失败证据
    // （未测量的数值保持为空，绝不写 0）。
    const bool has_trials = !summary.wall_ms.empty();
    const std::size_t trial_count = has_trials ? summary.wall_ms.size() : 1;
    for (std::size_t repeat = 0; repeat < trial_count; ++repeat) {
      const double wall_ms = has_trials ? summary.wall_ms[repeat] : 0.0;
      const std::string wall_text = has_trials ? format_number(wall_ms) : "";
      const std::string ops_text =
          has_trials && wall_ms > 0.0
              ? format_number(static_cast<double>(summary.batch_size) * 1000.0 / wall_ms)
              : "";
      const std::string amortized_text =
          has_trials && summary.batch_size > 0
              ? format_number(wall_ms / static_cast<double>(summary.batch_size))
              : "";
      stream << csv_field(run_id) << "," << csv_field(summary.case_id) << ","
             << csv_field(summary.backend) << "," << repeat << ","
             << "warm"
             << ","
             << ""  // seed：确定性负载，不使用种子
             << "," << (summary.valid ? "success" : "failed") << ","
             << csv_field(summary.failure_reason) << ","
             << ""  // preprocess_ms：输入生成不计入，在 summary.json 中报告
             << "," << wall_text << ",,,,,,,,,"  // cpu_ms..endpoint_yaw_error_rad 保持为空
             << (summary.valid ? "true" : "false") << "," << summary.degree << ","
             << (summary.derivative_order >= 0 ? std::to_string(summary.derivative_order) : "")
             << "," << summary.batch_size << "," << ops_text << "," << amortized_text << "\n";
    }
  }
  return static_cast<bool>(stream);
}

bool write_summary_json(const std::filesystem::path& path, const FixtureInputs& inputs,
                        const std::vector<CaseSummary>& cases, const std::string& suite,
                        const std::string& run_id,
                        const BenchmarkOutputs& outputs, int exit_code) {
  Json document;
  document["schema_version"] = 1;
  document["type"] = "polynomial_benchmark_summary";
  document["suite"] = suite;
  document["run_id"] = run_id;
  document["fixture"] = inputs.fixture_path;
  document["dimension"] = 2;
  document["coefficient_layout"] = "ascending powers in segment-local time (seconds)";
  document["segment_duration_s"] = inputs.segment_duration_s;
  {
    std::vector<std::string> backends;
    for (const CaseSummary& summary : cases) {
      bool known = false;
      for (const std::string& name : backends) {
        if (name == summary.backend) {
          known = true;
          break;
        }
      }
      if (!known && !summary.backend.empty()) {
        backends.push_back(summary.backend);
      }
    }
    document["backends"] = backends;
  }
  document["candidate_backend_available"] = outputs.mav_available;
  if (!outputs.mav_available && !outputs.mav_failure_reason.empty()) {
    document["candidate_failure_reason"] = outputs.mav_failure_reason;
  }
  document["equivalence"] = {
      {"available", outputs.equivalence_available},
      {"max_abs_diff", outputs.equivalence_max_abs_diff},
      {"tolerance", 1e-6}};
  document["construct_segment_count"] = inputs.cubic_segment_count;
  document["eval_time_point_count"] = inputs.eval_time_point_count;
  document["warmup_batches"] = inputs.warmup_batches;
  document["measured_batches"] = inputs.measured_batches;
  document["execution"] = "sequential single-threaded; input generation, allocation and "
                          "output outside the timed region";
  document["timing_unit"] = "ms";
  document["notes"] = Json::array(
      {"Per-op values are batch P50 divided by the batch size (amortized); they are not "
       "single-call P95.",
       "Warmup batches are not recorded; only the 30 measured batches are trials.",
       "cache_mode=warm for every measured batch: inputs are prebuilt before warmup."});
  document["cases"] = Json::array();
  for (const CaseSummary& summary : cases) {
    Json item;
    item["case_id"] = summary.case_id;
    item["backend"] = summary.backend;
    item["kind"] = summary.kind;
    item["degree"] = summary.degree;
    item["derivative_order"] = summary.derivative_order >= 0 ? Json(summary.derivative_order)
                                                             : Json(nullptr);
    item["batch_size"] = summary.batch_size;
    item["trial_count"] = summary.wall_ms.size();
    item["valid"] = summary.valid;
    item["failure_reason"] = summary.failure_reason;
    if (!summary.wall_ms.empty()) {
      const double p50 = nearest_rank(summary.wall_ms, 50.0);
      const double p95 = nearest_rank(summary.wall_ms, 95.0);
      item["batch_wall_ms_p50"] = p50;
      item["batch_wall_ms_p95"] = p95;
      item["amortized_ms_per_op"] = p50 / static_cast<double>(summary.batch_size);
      item["throughput_ops_per_s"] =
          p50 > 0.0 ? static_cast<double>(summary.batch_size) * 1000.0 / p50 : 0.0;
    }
    item["checksum"] = summary.checksum;
    document["cases"].push_back(std::move(item));
  }
  document["exit_code"] = exit_code;

  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    return false;
  }
  stream << document.dump(2) << "\n";
  return static_cast<bool>(stream);
}

// ---------------------------------------------------------------------------
// 内部选型输出（对照候选 vs 基线）。
// ---------------------------------------------------------------------------

struct BackendAggregate {
  std::string name;
  bool all_cases_success{true};
  int eval_cases{0};
  int total_cases{0};
  double eval_p95_sum{0.0};
  double all_p95_sum{0.0};
};

std::vector<BackendAggregate> aggregate_backends(
    const std::vector<CaseSummary>& cases) {
  std::map<std::string, BackendAggregate> aggregates;
  for (const CaseSummary& summary : cases) {
    BackendAggregate& aggregate = aggregates[summary.backend];
    aggregate.name = summary.backend;
    if (!summary.valid || summary.wall_ms.empty()) {
      aggregate.all_cases_success = false;
      continue;
    }
    const double p95 = nearest_rank(summary.wall_ms, 95.0);
    aggregate.total_cases += 1;
    aggregate.all_p95_sum += p95;
    if (summary.kind == "eval") {
      aggregate.eval_cases += 1;
      aggregate.eval_p95_sum += p95;
    }
  }
  std::vector<BackendAggregate> ordered;
  ordered.reserve(aggregates.size());
  for (auto& entry : aggregates) {
    ordered.push_back(entry.second);
  }
  return ordered;
}

bool write_selection_json(const std::filesystem::path& path,
                          const std::string& suite, const std::string& run_id,
                          const std::vector<CaseSummary>& cases,
                          const BenchmarkOutputs& outputs) {
  const std::vector<BackendAggregate> aggregates = aggregate_backends(cases);
  Json document;
  document["schema_version"] = 1;
  document["type"] = "polynomial_backend_selection";
  document["suite"] = suite;
  document["component"] = "polynomial";
  document["run_id"] = run_id;
  document["incumbent"] = kHornerBackend;
  document["rule"] = {
      {"correctness_gate",
       "cross-backend |value difference| <= 1e-6 on shared coefficients, "
       "segment-local times and derivative orders 0..3; evaluated in-run "
       "outside the timed region"},
      {"quality_gate", "all candidate evaluation cases complete (construction "
                       "comparison is N/A: upstream has no equivalent "
                       "closed-form boundary solver)"},
      {"ranking",
       "equal-weight mean of per-case batch P95 over evaluation cases "
       "(cubic + quintic x orders 0..3); lower wins"},
      {"tie_break",
       "if the leader is within 5% of the incumbent, compare the equal-weight "
       "mean of batch P95 over all cases; still within 5% keeps the incumbent"}};

  document["construction_comparison"] =
      "N/A: mav_trajectory_generation exposes no equivalent closed-form "
      "cubic/quintic boundary-condition solver; the production solver is "
      "verified against an independent Gaussian-elimination reference in the "
      "component tests";
  document["equivalence"] = {
      {"available", outputs.equivalence_available},
      {"max_abs_diff", outputs.equivalence_max_abs_diff},
      {"tolerance", 1e-6}};

  Json ranking = Json::array();
  std::vector<const BackendAggregate*> qualified;
  for (const BackendAggregate& aggregate : aggregates) {
    const bool equivalence_passed =
        !outputs.equivalence_available ||
        aggregate.name == kHornerBackend || outputs.equivalence_max_abs_diff <= 1e-6;
    const bool qualifies =
        aggregate.all_cases_success && equivalence_passed;
    if (qualifies) {
      qualified.push_back(&aggregate);
    }
    ranking.push_back(
        {{"backend", aggregate.name},
         {"qualified", qualifies},
         {"all_cases_success", aggregate.all_cases_success},
         {"correctness_gate_passed", equivalence_passed},
         {"eval_cases", aggregate.eval_cases},
         {"eval_p95_mean_ms",
          aggregate.eval_cases > 0
              ? aggregate.eval_p95_sum / static_cast<double>(aggregate.eval_cases)
              : 0.0},
         {"all_p95_mean_ms",
          aggregate.total_cases > 0
              ? aggregate.all_p95_sum / static_cast<double>(aggregate.total_cases)
              : 0.0}});
  }

  std::string default_backend;
  std::vector<std::string> pending;
  if (!qualified.empty()) {
    const auto mean_eval = [](const BackendAggregate* aggregate) {
      return aggregate->eval_cases > 0
                 ? aggregate->eval_p95_sum / static_cast<double>(aggregate->eval_cases)
                 : std::numeric_limits<double>::infinity();
    };
    const auto mean_all = [](const BackendAggregate* aggregate) {
      return aggregate->total_cases > 0
                 ? aggregate->all_p95_sum / static_cast<double>(aggregate->total_cases)
                 : std::numeric_limits<double>::infinity();
    };
    const BackendAggregate* best = qualified.front();
    for (const BackendAggregate* candidate : qualified) {
      if (mean_eval(candidate) < mean_eval(best)) {
        best = candidate;
      }
    }
    const BackendAggregate* incumbent = nullptr;
    for (const BackendAggregate* candidate : qualified) {
      if (candidate->name == kHornerBackend) {
        incumbent = candidate;
      }
    }
    if (incumbent != nullptr && best != incumbent &&
        mean_eval(best) >= 0.95 * mean_eval(incumbent)) {
      default_backend = mean_all(best) < 0.95 * mean_all(incumbent)
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
      default_backend.empty() ? Json(nullptr) : Json(default_backend);
  document["pending_optimization"] = pending;
  document["ranking"] = std::move(ranking);
  if (!outputs.mav_available) {
    document["notes"] = Json::array(
        {"candidate backend unavailable or single-backend run; selection is "
         "preliminary"});
  }

  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    return false;
  }
  stream << document.dump(2) << "\n";
  return static_cast<bool>(stream);
}

bool write_plot(const std::filesystem::path& dir, const std::vector<CaseSummary>& cases) {
  // 含绘图数值的数据 CSV（与 SVG 同名）。
  std::ofstream csv(dir / "batch_wall_ms_by_case.csv", std::ios::binary | std::ios::trunc);
  if (!csv) {
    return false;
  }
  csv << "case_id,degree,derivative_order,batch_size,p50_ms,p95_ms,amortized_ms_per_op,"
         "throughput_ops_per_s\n";
  csv << std::setprecision(10);
  double max_value = 0.0;
  for (const CaseSummary& summary : cases) {
    if (summary.wall_ms.empty()) {
      continue;
    }
    const double p50 = nearest_rank(summary.wall_ms, 50.0);
    const double p95 = nearest_rank(summary.wall_ms, 95.0);
    max_value = std::max(max_value, p95);
    csv << summary.case_id << "," << summary.degree << ","
        << (summary.derivative_order >= 0 ? std::to_string(summary.derivative_order) : "") << ","
        << summary.batch_size << "," << p50 << "," << p95 << ","
        << p50 / static_cast<double>(summary.batch_size) << ","
        << (p50 > 0.0 ? static_cast<double>(summary.batch_size) * 1000.0 / p50 : 0.0) << "\n";
  }
  csv.close();

  // 极简手写 SVG 柱状图；数值打印在柱子旁边，
  // 使颜色永远不是唯一的标识。
  const std::size_t count = cases.size();
  const double margin_left = 110.0;
  const double margin_bottom = 150.0;
  const double width = margin_left + 96.0 * static_cast<double>(count) + 40.0;
  const double height = 460.0;
  const double plot_top = 60.0;
  const double plot_height = height - margin_bottom - plot_top;
  const double scale = max_value > 0.0 ? plot_height / max_value : 0.0;

  std::ofstream svg(dir / "batch_wall_ms_by_case.svg", std::ios::binary | std::ios::trunc);
  if (!svg) {
    return false;
  }
  svg << std::setprecision(6);
  svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << width << "\" height=\""
      << height << "\" viewBox=\"0 0 " << width << " " << height << "\">\n";
  svg << "<rect width=\"100%\" height=\"100%\" fill=\"#ffffff\"/>\n";
  svg << "<text x=\"" << margin_left << "\" y=\"26\" font-size=\"15\" font-family=\"sans-serif\">"
         "planner_polynomial bench: batch wall time "
         "(ms), P50/P95 over 30 measured batches</text>\n";
  svg << "<rect x=\"" << margin_left << "\" y=\"" << (plot_top + 10)
      << "\" width=\"10\" height=\"10\" fill=\"#4c72b0\"/>"
         "<text x=\"" << (margin_left + 16) << "\" y=\"" << (plot_top + 19)
      << "\" font-size=\"11\" font-family=\"sans-serif\">P50 (nearest-rank)</text>\n";
  svg << "<rect x=\"" << (margin_left + 130) << "\" y=\"" << (plot_top + 10)
      << "\" width=\"10\" height=\"10\" fill=\"#dd8452\"/>"
         "<text x=\"" << (margin_left + 146) << "\" y=\"" << (plot_top + 19)
      << "\" font-size=\"11\" font-family=\"sans-serif\">P95 (nearest-rank)</text>\n";
  const double baseline = plot_top + plot_height;
  svg << "<line x1=\"" << margin_left << "\" y1=\"" << baseline << "\" x2=\""
      << (width - 40.0) << "\" y2=\"" << baseline
      << "\" stroke=\"#333333\" stroke-width=\"1\"/>\n";
  for (std::size_t i = 0; i < count; ++i) {
    const CaseSummary& summary = cases[i];
    const double x = margin_left + 14.0 + 96.0 * static_cast<double>(i);
    svg << "<text x=\"" << (x + 30.0) << "\" y=\"" << (baseline + 14)
        << "\" font-size=\"9\" font-family=\"sans-serif\" text-anchor=\"middle\">"
        << summary.case_id << "</text>\n";
    if (summary.wall_ms.empty()) {
      svg << "<text x=\"" << (x + 30.0) << "\" y=\"" << (baseline - 8)
          << "\" font-size=\"10\" font-family=\"sans-serif\" text-anchor=\"middle\">failed"
             "</text>\n";
      continue;
    }
    const double p50 = nearest_rank(summary.wall_ms, 50.0);
    const double p95 = nearest_rank(summary.wall_ms, 95.0);
    const double h50 = p50 * scale;
    const double h95 = p95 * scale;
    svg << "<rect x=\"" << x << "\" y=\"" << (baseline - h50) << "\" width=\"28\" height=\""
        << h50 << "\" fill=\"#4c72b0\"/>\n";
    svg << "<rect x=\"" << (x + 32.0) << "\" y=\"" << (baseline - h95) << "\" width=\"28\" "
        << "height=\"" << h95 << "\" fill=\"#dd8452\"/>\n";
    svg << "<text x=\"" << (x + 14.0) << "\" y=\"" << (baseline - h50 - 4)
        << "\" font-size=\"9\" font-family=\"sans-serif\" text-anchor=\"middle\">" << p50
        << "</text>\n";
    svg << "<text x=\"" << (x + 46.0) << "\" y=\"" << (baseline - h95 - 4)
        << "\" font-size=\"9\" font-family=\"sans-serif\" text-anchor=\"middle\">" << p95
        << "</text>\n";
  }
  svg << "</svg>\n";
  return static_cast<bool>(svg);
}

}  // namespace

int main(int argc, char** argv) {
  std::string suite = "v1";
  std::string output_dir;
  std::string fixture_path = PLANNER_POLYNOMIAL_BENCH_FIXTURE;
  std::string backend_option = "both";
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
      std::cout << kUsage;
      return 0;
    } else {
      std::cerr << "unknown argument: " << argument << "\n" << kUsage;
      return 2;
    }
  }
  if (output_dir.empty()) {
    std::cerr << "missing --output-dir\n" << kUsage;
    return 2;
  }
  if (suite != "v1") {
    std::cerr << "unsupported suite '" << suite << "' (only v1 is implemented)\n";
    return 65;
  }
  if (backend_option != "both" && backend_option != "horner" &&
      backend_option != "mav") {
    std::cerr << "unsupported --backend '" << backend_option << "'\n" << kUsage;
    return 2;
  }
  const bool run_mav = backend_option != "horner";

  std::string error;
  FixtureInputs inputs;
  if (!parse_fixture(fixture_path, &inputs, &error)) {
    std::cerr << "fixture error: " << error << "\n";
    return 65;
  }
  {
    std::ifstream precheck_stream(fixture_path);
    Json document;
    try {
      precheck_stream >> document;
    } catch (const std::exception& exc) {
      std::cerr << "fixture error: " << exc.what() << "\n";
      return 65;
    }
    if (!precheck_conversion_factor(document, &error)) {
      std::cerr << "benchmark precheck failed: " << error << "\n";
      return 70;
    }
  }

  const char* run_id_env = std::getenv("PLANNER_RUN_ID");
  const std::string run_id = run_id_env != nullptr ? run_id_env : "local";

  BenchmarkOutputs outputs;
  const std::vector<CaseSummary> cases = run_suite(inputs, run_mav, &outputs);

  bool all_valid = true;
  for (const CaseSummary& summary : cases) {
    all_valid = all_valid && summary.valid &&
                summary.wall_ms.size() == static_cast<std::size_t>(inputs.measured_batches);
  }

  const std::filesystem::path out(output_dir);
  std::error_code filesystem_error;
  std::filesystem::create_directories(out / "plots", filesystem_error);
  if (filesystem_error) {
    std::cerr << "cannot create output directory '" << out << "': " << filesystem_error.message()
              << "\n";
    return 70;
  }

  const int exit_code = all_valid ? 0 : 70;
  bool written = true;
  written = write_metrics_csv(out / "benchmark_metrics.csv", cases, run_id) && written;
  written = write_summary_json(out / "polynomial_bench_summary.json", inputs, cases, suite, run_id,
                               outputs, exit_code) &&
            written;
  written = write_selection_json(out / "selection.json", suite, run_id, cases, outputs) &&
            written;
  written = write_plot(out / "plots", cases) && written;
  if (!written) {
    std::cerr << "failed to write benchmark outputs under '" << out << "'\n";
    return 70;
  }

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "planner_polynomial_bench suite=" << suite << " run_id=" << run_id << "\n";
  std::cout << "fixture=" << inputs.fixture_path << "\n";
  std::cout << "warmup_batches=" << inputs.warmup_batches
            << " measured_batches=" << inputs.measured_batches << "\n";
  std::cout << "case,batch_size,p50_ms,p95_ms,amortized_ms_per_op,throughput_ops_per_s,valid\n";
  for (const CaseSummary& summary : cases) {
    if (summary.wall_ms.empty()) {
      std::cout << summary.case_id << "," << summary.batch_size << ",,,,,"
                << (summary.valid ? "true" : "false") << " (" << summary.failure_reason << ")\n";
      continue;
    }
    const double p50 = nearest_rank(summary.wall_ms, 50.0);
    const double p95 = nearest_rank(summary.wall_ms, 95.0);
    std::cout << summary.case_id << "," << summary.batch_size << "," << p50 << "," << p95 << ","
              << p50 / static_cast<double>(summary.batch_size) << ","
              << (p50 > 0.0 ? static_cast<double>(summary.batch_size) * 1000.0 / p50 : 0.0) << ","
              << (summary.valid ? "true" : "false") << "\n";
  }
  std::cout << "metrics=" << (out / "benchmark_metrics.csv").string() << "\n";
  std::cout << "summary=" << (out / "polynomial_bench_summary.json").string() << "\n";
  std::cout << "plots=" << (out / "plots").string() << "\n";
  return exit_code;
}
