// 随机差分运行器（docs §7.1、tests/fixtures/v1/collision/random_spec.json）。
//
// 生成开发（10,000）或验收（每个种子 1,000）用例集，在相同输入上运行生产检查器
// （plain，可选 accelerated）与独立参考，并比较判定：
//   * 零漏检是硬性门槛（生产 clear + 参考 collision）；
//   * 保守误报被计数并报告；
//   * 参考 inconclusive 的样本被排除，完整保存并补样，以达到可判定目标数。
//
// 证据写入唯一输出目录（默认
// artifacts/collision_differential/<utc>_<pid>）：summary.json、cases.csv、
// failures/<case_id>.json（漏检与误报）与 inconclusive_samples.jsonl（完整输入）。
// 样本输入绝不提交到 Git。
//
// 退出码：0 正常；1 验证失败（漏检/加速不一致，或未达到目标用例数）；2 用法错误；
// 3 spec 不匹配；4 内部错误。

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include <unistd.h>

#include <nlohmann/json.hpp>

#include "planner/collision/collision_checker.hpp"
#include "planner/collision/differential.hpp"
#include "planner/common/serialization.hpp"
#include "planner/common/types.hpp"

namespace {

using planner::collision::CheckOptions;
using planner::collision::CheckResult;
using planner::collision::CheckStatus;
using planner::collision::CollisionReason;
using planner::collision::ReferenceResult;
using planner::collision::ReferenceVerdict;
using planner::collision::differential::CaseAllocation;
using planner::collision::differential::DifferentialCase;
using planner::collision::differential::RegimeSpec;
using planner::collision::differential::Stratum;

struct Options {
  std::string regime{"dev"};
  int seed{0};
  int case_target{-1};
  std::string output_dir;
  std::string accel_mode{"both"};
  std::string spec_path;
  int max_ordinals{-1};
  bool quiet{false};
};

void usage() {
  std::cout
      << "usage: planner_collision_differential [--regime dev|acceptance|smoke]\n"
      << "       [--seed N] [--case-target N] [--output-dir DIR]\n"
      << "       [--accel off|on|both] [--spec random_spec.json]\n"
      << "       [--max-ordinals N] [--quiet]\n";
}

bool parse_int(const std::string& text, int* out) {
  try {
    std::size_t used = 0;
    const int value = std::stoi(text, &used);
    if (used != text.size()) return false;
    *out = value;
    return true;
  } catch (...) {
    return false;
  }
}

bool parse_options(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto value = [&](const char* name) -> std::string {
      if (i + 1 >= argc) {
        std::cerr << "missing value for " << name << "\n";
        std::exit(2);
      }
      return argv[++i];
    };
    if (arg == "--regime") {
      options->regime = value("--regime");
    } else if (arg == "--seed") {
      if (!parse_int(value("--seed"), &options->seed)) return false;
    } else if (arg == "--case-target") {
      if (!parse_int(value("--case-target"), &options->case_target)) return false;
    } else if (arg == "--output-dir") {
      options->output_dir = value("--output-dir");
    } else if (arg == "--accel") {
      options->accel_mode = value("--accel");
    } else if (arg == "--spec") {
      options->spec_path = value("--spec");
    } else if (arg == "--max-ordinals") {
      if (!parse_int(value("--max-ordinals"), &options->max_ordinals)) return false;
    } else if (arg == "--quiet") {
      options->quiet = true;
    } else if (arg == "--help" || arg == "-h") {
      usage();
      std::exit(0);
    } else {
      std::cerr << "unknown argument: " << arg << "\n";
      return false;
    }
  }
  if (options->regime != "dev" && options->regime != "acceptance" &&
      options->regime != "smoke") {
    std::cerr << "unknown regime: " << options->regime << "\n";
    return false;
  }
  if (options->accel_mode != "off" && options->accel_mode != "on" &&
      options->accel_mode != "both") {
    std::cerr << "unknown --accel mode: " << options->accel_mode << "\n";
    return false;
  }
  return true;
}

RegimeSpec make_regime(const Options& options) {
  RegimeSpec spec;
  if (options.regime == "acceptance") {
    spec = planner::collision::differential::acceptance_regime_spec(options.seed);
  } else {
    spec = planner::collision::differential::dev_regime_spec();
  }
  if (options.case_target > 0) {
    spec.pose_checks = options.case_target / 2;
    spec.motion_checks = options.case_target - spec.pose_checks;
  }
  return spec;
}

std::string utc_stamp() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
  std::tm tm{};
  gmtime_r(&seconds, &tm);
  char buffer[32];
  std::strftime(buffer, sizeof(buffer), "%Y%m%dT%H%M%SZ", &tm);
  return std::string(buffer);
}

nlohmann::json pose_json(const planner::common::Pose2d& pose) {
  return nlohmann::json{{"x_m", pose.x_m}, {"y_m", pose.y_m},
                        {"yaw_rad", pose.yaw_rad}};
}

nlohmann::json obstacles_json(const DifferentialCase& test_case) {
  nlohmann::json obstacles = nlohmann::json::array();
  for (const auto& obstacle : test_case.scene.obstacles) {
    nlohmann::json vertices = nlohmann::json::array();
    for (const auto& vertex : obstacle.vertices) {
      vertices.push_back({{"x_m", vertex.x_m}, {"y_m", vertex.y_m}});
    }
    obstacles.push_back({{"id", obstacle.id},
                         {"type", "convex_polygon"},
                         {"vertices", vertices}});
  }
  return obstacles;
}

nlohmann::json case_json(const DifferentialCase& test_case,
                         const std::string& verdict_production,
                         const std::string& verdict_reference,
                         const std::string& notes) {
  nlohmann::json out;
  out["case_id"] = test_case.case_id;
  out["seed"] = test_case.seed;
  out["regime"] = test_case.regime;
  out["check_kind"] = test_case.check_kind;
  out["kind"] = test_case.check_kind;
  out["stratum"] =
      planner::collision::differential::to_string(test_case.stratum);
  out["generator_version"] =
      planner::collision::differential::kGeneratorVersion;
  out["vehicle"] =
      planner::common::encode_payload(test_case.vehicle);
  if (test_case.check_kind == "pose") {
    out["pose_or_segment"] = pose_json(test_case.start);
  } else {
    out["pose_or_segment"] = {
        {"start", pose_json(test_case.start)},
        {"length_m", test_case.length_m},
        {"direction", planner::common::to_string(test_case.direction)},
        {"curvature_inv_m", test_case.curvature_inv_m}};
  }
  out["obstacles"] = obstacles_json(test_case);
  out["grid"] = planner::common::encode_payload(test_case.scene.grid);
  out["verdict_production"] = verdict_production;
  out["verdict_reference"] = verdict_reference;
  out["design_gap_m"] = test_case.design_gap_m;
  out["measured_clearance_m"] = test_case.measured_clearance_m;
  out["replacement_of"] = test_case.replacement_of;
  out["notes"] = notes;
  return out;
}

std::string verdict_name(const CheckResult& result) {
  if (result.status == CheckStatus::kInvalidInput) return "invalid_input";
  return result.collided() ? "collision" : "clear";
}

std::string reference_verdict_name(ReferenceVerdict verdict) {
  switch (verdict) {
    case ReferenceVerdict::kCollision:
      return "collision";
    case ReferenceVerdict::kClear:
      return "clear";
    case ReferenceVerdict::kInconclusive:
      return "inconclusive";
    case ReferenceVerdict::kInvalidInput:
      return "invalid_input";
  }
  return "unknown";
}

// CSV 单元格不加引号时不能包含逗号；notes 中的这类字符需替换。
std::string csv_safe(const std::string& text) {
  std::string out = text;
  for (char& c : out) {
    if (c == ',' || c == '\n' || c == '\r') c = ';';
  }
  return out;
}

int check_spec_matches(const Options& options, const RegimeSpec& regime) {
  std::ifstream stream(options.spec_path);
  if (!stream) {
    std::cerr << "cannot read spec file: " << options.spec_path << "\n";
    return 3;
  }
  nlohmann::json spec;
  stream >> spec;
  const std::string key =
      options.regime == "acceptance" ? "acceptance" : "dev_regression";
  const nlohmann::json& entry = spec.at("regimes").at(key);
  const int expected_seed =
      options.regime == "acceptance" ? options.seed
                                     : entry.at("seed").get<int>();
  const std::string expected_prefix = entry.at("id_prefix").get<std::string>();
  bool ok = true;
  ok = ok && regime.seed == expected_seed;
  ok = ok && regime.id_prefix == expected_prefix;
  if (options.regime == "acceptance") {
    // random_spec.json 在 pose_checks / motion_checks 中存的是整个 regime 的
    // 汇总值；一次调用覆盖单个种子的 cases_per_seed 个用例（一半 pose、一半
    // motion）。
    const int cases_per_seed = entry.at("cases_per_seed").get<int>();
    const int seed_first = entry.at("seed_range").at("first").get<int>();
    const int seed_last = entry.at("seed_range").at("last").get<int>();
    const int per_seed_pose = cases_per_seed / 2;
    const int per_seed_motion = cases_per_seed - per_seed_pose;
    const int seed_count = seed_last - seed_first + 1;
    ok = ok && options.seed >= seed_first && options.seed <= seed_last;
    ok = ok && regime.pose_checks == per_seed_pose &&
         regime.motion_checks == per_seed_motion;
    ok = ok && entry.at("pose_checks").get<int>() == per_seed_pose * seed_count;
    ok = ok && entry.at("motion_checks").get<int>() ==
                   per_seed_motion * seed_count;
    ok = ok && entry.at("case_count").get<int>() == cases_per_seed * seed_count;
  } else {
    int expected_total = entry.at("pose_checks").get<int>() +
                         entry.at("motion_checks").get<int>();
    if (options.case_target > 0) {
      expected_total = options.case_target;
    }
    ok = ok && regime.pose_checks + regime.motion_checks == expected_total;
  }
  if (!ok) {
    std::cerr << "spec mismatch for regime " << options.regime << "\n";
    return 3;
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!parse_options(argc, argv, &options)) {
    usage();
    return 2;
  }
  const RegimeSpec regime = make_regime(options);
  if (!options.spec_path.empty()) {
    const int spec_status = check_spec_matches(options, regime);
    if (spec_status != 0) return spec_status;
  }

  std::string output_dir = options.output_dir;
  if (output_dir.empty()) {
    output_dir = "artifacts/collision_differential/" + utc_stamp() + "_" +
                 std::to_string(static_cast<long>(::getpid()));
  }
  std::error_code error;
  std::filesystem::create_directories(output_dir + "/failures", error);
  if (error) {
    std::cerr << "cannot create output directory " << output_dir << ": "
              << error.message() << "\n";
    return 4;
  }

  const std::vector<CaseAllocation> allocation =
      planner::collision::differential::build_regime_allocation(regime);
  const int target = regime.pose_checks + regime.motion_checks;
  const int max_ordinals =
      options.max_ordinals > 0 ? options.max_ordinals : target * 4;

  CheckOptions plain;
  plain.enable_acceleration = false;
  CheckOptions accelerated;
  accelerated.enable_acceleration = true;
  const bool run_accel = options.accel_mode != "off";

  std::ofstream csv(output_dir + "/cases.csv");
  csv << "case_id,seed,regime,check_kind,stratum,replacement_of,"
         "verdict_production,verdict_reference,verdict_accel,"
         "first_s_production,first_s_reference,first_s_accel,"
         "reason_production,design_gap_m,measured_clearance_m,elapsed_us,"
         "notes\n";

  std::ofstream inconclusive_jsonl(output_dir + "/inconclusive_samples.jsonl");

  int ordinal = 0;
  int next_slot = 0;
  int decidable = 0;
  int agree_collision = 0;
  int agree_clear = 0;
  int misses = 0;
  int false_positives = 0;
  int incontestable = 0;  // 参考 inconclusive 的样本
  int replacement_cases = 0;
  int accel_mismatches = 0;
  int window_mismatches = 0;
  int floor_collisions = 0;
  std::deque<CaseAllocation> replacements;
  std::vector<std::string> failure_ids;

  while (decidable < target && ordinal < max_ordinals) {
    CaseAllocation slot;
    std::string replacement_of;
    if (next_slot < static_cast<int>(allocation.size())) {
      slot = allocation[static_cast<std::size_t>(next_slot++)];
    } else if (!replacements.empty()) {
      slot = replacements.front();
      replacements.pop_front();
      replacement_of = "replacement";
      ++replacement_cases;
    } else {
      break;
    }
    const DifferentialCase test_case =
        planner::collision::differential::generate_case(regime, slot, ordinal,
                                                        0, replacement_of);
    ++ordinal;

    const ReferenceResult reference =
        planner::collision::differential::reference_check(test_case);
    if (reference.verdict == ReferenceVerdict::kInvalidInput) {
      std::cerr << "internal error: reference rejected generated case "
                << test_case.case_id << ": " << reference.detail << "\n";
      return 4;
    }
    const bool reference_inconclusive =
        reference.verdict == ReferenceVerdict::kInconclusive;
    const auto production_start = std::chrono::steady_clock::now();
    const CheckResult production =
        planner::collision::differential::production_check(test_case, plain);
    const auto production_end = std::chrono::steady_clock::now();
    const long elapsed_us = static_cast<long>(
        std::chrono::duration_cast<std::chrono::microseconds>(production_end -
                                                              production_start)
            .count());
    if (production.status == CheckStatus::kInvalidInput) {
      std::cerr << "internal error: production rejected generated case "
                << test_case.case_id << ": " << production.detail << "\n";
      return 4;
    }
    CheckResult accel_result;
    if (run_accel) {
      accel_result =
          planner::collision::differential::production_check(test_case,
                                                             accelerated);
      if (accel_result.status != production.status ||
          accel_result.reason_code != production.reason_code ||
          accel_result.first_s_m != production.first_s_m ||
          accel_result.first_pose.x_m != production.first_pose.x_m ||
          accel_result.first_pose.y_m != production.first_pose.y_m) {
        ++accel_mismatches;
        failure_ids.push_back(test_case.case_id);
        std::ofstream failure(output_dir + "/failures/" + test_case.case_id +
                              "_accel.json");
        failure << case_json(test_case, verdict_name(production),
                             reference_verdict_name(reference.verdict),
                             "acceleration path differs from the plain path")
                       .dump(2);
      }
    }
    if (production.reason_code ==
        planner::collision::CollisionReason::kPrecisionFloor) {
      ++floor_collisions;
    }
    if (std::isfinite(test_case.design_gap_m) &&
        std::isfinite(test_case.measured_clearance_m) &&
        test_case.design_gap_m >= 0.0) {
      if (std::abs(test_case.measured_clearance_m - test_case.design_gap_m) >
          1e-5) {
        ++window_mismatches;
      }
    }

    std::string production_verdict = verdict_name(production);
    std::string accel_verdict = run_accel ? verdict_name(accel_result) : "";
    std::string reference_verdict = reference_verdict_name(reference.verdict);
    std::string notes;

    if (reference_inconclusive) {
      ++incontestable;
      inconclusive_jsonl << case_json(test_case, production_verdict,
                                      reference_verdict,
                                      reference.detail)
                                .dump()
                         << "\n";
      replacements.push_back(slot);
      notes = "reference_inconclusive:" + reference.detail;
    } else {
      const bool reference_collision =
          reference.verdict == ReferenceVerdict::kCollision;
      if (reference_collision && production.collided()) {
        ++agree_collision;
      } else if (!reference_collision && !production.collided()) {
        ++agree_clear;
      } else if (reference_collision && !production.collided()) {
        ++misses;
        failure_ids.push_back(test_case.case_id);
        notes = "miss: production clear, reference collision";
        std::ofstream failure(output_dir + "/failures/" + test_case.case_id +
                              ".json");
        failure << case_json(test_case, production_verdict, reference_verdict,
                             notes)
                       .dump(2);
      } else {
        ++false_positives;
        failure_ids.push_back(test_case.case_id);
        notes = "false_positive: production collision, reference clear";
        std::ofstream failure(output_dir + "/failures/" + test_case.case_id +
                              ".json");
        failure << case_json(test_case, production_verdict, reference_verdict,
                             notes)
                       .dump(2);
      }
      ++decidable;
    }

    csv << test_case.case_id << "," << test_case.seed << ","
        << test_case.regime << "," << test_case.check_kind << ","
        << planner::collision::differential::to_string(test_case.stratum) << ","
        << replacement_of << "," << production_verdict << ","
        << reference_verdict << "," << accel_verdict << ","
        << production.first_s_m << "," << reference.first_s_m << ","
        << (run_accel ? accel_result.first_s_m : 0.0) << ","
        << planner::collision::to_string(production.reason_code) << ","
        << test_case.design_gap_m << "," << test_case.measured_clearance_m
        << "," << elapsed_us << "," << csv_safe(notes) << "\n";
  }
  csv.close();
  inconclusive_jsonl.close();

  const bool target_reached = decidable >= target;
  const bool passed =
      target_reached && misses == 0 && accel_mismatches == 0 &&
      window_mismatches == 0;

  const nlohmann::json summary = {
      {"schema_version", 1},
      {"type", "collision_differential_summary"},
      {"generator_version", planner::collision::differential::kGeneratorVersion},
      {"regime", regime.name},
      {"seed", regime.seed},
      {"run_utc", utc_stamp()},
      {"spec", options.spec_path},
      {"accel_mode", options.accel_mode},
      {"cases_target", target},
      {"cases_generated", ordinal},
      {"decidable_cases", decidable},
      {"agree_collision", agree_collision},
      {"agree_clear", agree_clear},
      {"true_collisions_reference", agree_collision + misses},
      {"misses", misses},
      {"false_positives", false_positives},
      {"inconclusive_samples", incontestable},
      {"replacement_cases", replacement_cases},
      {"acceleration_mismatches", accel_mismatches},
      {"window_mismatches", window_mismatches},
      {"precision_floor_collisions", floor_collisions},
      {"failure_ids", failure_ids},
      {"target_reached", target_reached},
      {"passed", passed},
      {"output_dir", output_dir}};
  std::ofstream summary_file(output_dir + "/summary.json");
  summary_file << summary.dump(2) << "\n";
  summary_file.close();

  if (!options.quiet) {
    std::cout << "collision differential: regime=" << regime.name
              << " seed=" << regime.seed << "\n"
              << "  cases_generated=" << ordinal
              << " decidable=" << decidable << "/" << target << "\n"
              << "  agree_collision=" << agree_collision
              << " agree_clear=" << agree_clear << "\n"
              << "  misses=" << misses << " false_positives=" << false_positives
              << " inconclusive=" << incontestable
              << " replacements=" << replacement_cases << "\n"
              << "  acceleration_mismatches=" << accel_mismatches
              << " precision_floor_collisions=" << floor_collisions << "\n"
              << "  output_dir=" << output_dir << "\n";
  }
  return passed ? 0 : 1;
}
