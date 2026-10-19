// planner_grid_scenario_gen --suite v1 --output-dir <dir> [options]
//
// grid_scenario_gen v1 确定性生成器（X-B1）：生成正式档/容量档栅格实例，
// 写出实例表（instances.json）、打包 uint8 实例文件（instances/<id>.u8）、
// SHA256 清单（instances.sha256）与生成摘要（generation_summary.json）。
//
// 用法示例：
//   planner_grid_scenario_gen --suite v1 --output-dir <dir>
//   planner_grid_scenario_gen --suite v1 --output-dir <dir> --capacity
//     --archive-dir <repo>/artifacts/data/sha256 --repo-root <repo>
//
// 规则与冻结解释见 Src/grid_search/benchmarks/README.md；>10 MiB 输入按
// benchmark_tiers.md §3.4 使用 SHA256 寻址归档（附 .sha256/.meta.json）。
//
// 退出码：0 成功；1 自检失败（实例保留，problems 写入摘要）；2 用法/IO 错误。

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "bench_util.hpp"
#include "scenario_gen.hpp"
#include "sha256.hpp"

namespace {

namespace fs = std::filesystem;
using planner::grid_search::bench::ScenarioInstance;
using planner::grid_search::bench::kArchiveThresholdBytes;
using planner::grid_search::bench::kBenchResolutionM;
using nlohmann::json;

struct Options {
  std::string suite{"v1"};
  std::string output_dir;
  std::string archive_dir;
  std::string repo_root;
  std::vector<std::string> classes;
  std::vector<int> scales;
  std::vector<int> seeds;
  bool capacity{false};
  std::int64_t archive_threshold{kArchiveThresholdBytes};
  bool classes_set{false};
  bool scales_set{false};
  bool seeds_set{false};
};

[[noreturn]] void usage_error(const std::string& message) {
  std::cerr << "planner_grid_scenario_gen: " << message << "\n";
  std::cerr << "usage: planner_grid_scenario_gen --suite v1 --output-dir <dir>"
               " [--classes a,b,c] [--scales 128,512,1024] [--seeds 0,1,2,3,4]"
               " [--capacity] [--archive-dir <dir>] [--repo-root <dir>]\n";
  std::exit(2);
}

std::vector<int> parse_int_list(const std::string& text) {
  std::vector<int> values;
  for (const std::string& item :
       planner::grid_search::bench::split_csv(text)) {
    try {
      values.push_back(std::stoi(item));
    } catch (const std::exception&) {
      usage_error("cannot parse integer list item: " + item);
    }
  }
  if (values.empty()) {
    usage_error("empty integer list");
  }
  return values;
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
    } else if (arg == "--archive-dir") {
      options.archive_dir = value("--archive-dir");
    } else if (arg == "--repo-root") {
      options.repo_root = value("--repo-root");
    } else if (arg == "--classes") {
      options.classes = planner::grid_search::bench::split_csv(value("--classes"));
      options.classes_set = true;
    } else if (arg == "--scales") {
      options.scales = parse_int_list(value("--scales"));
      options.scales_set = true;
    } else if (arg == "--seeds") {
      options.seeds = parse_int_list(value("--seeds"));
      options.seeds_set = true;
    } else if (arg == "--capacity") {
      options.capacity = true;
    } else if (arg == "--archive-threshold-bytes") {
      options.archive_threshold = std::stoll(value("--archive-threshold-bytes"));
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
  if (!options.classes_set) {
    options.classes = planner::grid_search::bench::all_scenario_classes();
  }
  if (!options.scales_set) {
    options.scales = options.capacity ? std::vector<int>{2048, 4096}
                                      : std::vector<int>{128, 512, 1024};
  }
  if (!options.seeds_set) {
    options.seeds = options.capacity ? std::vector<int>{0}
                                     : std::vector<int>{0, 1, 2, 3, 4};
  }
  if (options.capacity) {
    if (options.seeds.size() != 1 || options.seeds[0] != 0) {
      usage_error("--capacity requires seed 0 only (single-run protocol)");
    }
    for (int scale : options.scales) {
      if (scale != 2048 && scale != 4096) {
        usage_error("--capacity scales must be 2048 and/or 4096");
      }
    }
  }
  for (const std::string& name : options.classes) {
    if (!planner::grid_search::bench::is_valid_scenario_class(name)) {
      usage_error("unknown scenario class: " + name);
    }
  }
  for (int scale : options.scales) {
    if (!planner::grid_search::bench::is_valid_scale(scale)) {
      usage_error("unsupported scale: " + std::to_string(scale));
    }
  }
  for (int seed : options.seeds) {
    if (seed < 0 || seed > 4) {
      usage_error("seed out of range [0,4]: " + std::to_string(seed));
    }
  }
  if (options.repo_root.empty()) {
    options.repo_root = fs::current_path().string();
  }
  return options;
}

// 把 <repo>/artifacts/... 形式转成仓库相对路径；否则保留绝对路径。
std::string relative_to_repo(const std::string& path,
                             const std::string& repo_root) {
  std::error_code error;
  const fs::path canonical = fs::weakly_canonical(path, error);
  const fs::path root = fs::weakly_canonical(repo_root, error);
  const std::string canonical_text = canonical.string();
  const std::string root_text = root.string();
  if (canonical_text.rfind(root_text + "/", 0) == 0) {
    return canonical_text.substr(root_text.size() + 1);
  }
  return canonical_text;
}

struct ArchiveOutcome {
  std::string name;
  std::string sha256;
  std::int64_t size_bytes{0};
  std::string relative_path;
};

ArchiveOutcome archive_file(const fs::path& source, const std::string& sha256,
                            const std::string& archive_dir,
                            const std::string& repo_root,
                            const ScenarioInstance& instance) {
  ArchiveOutcome outcome;
  outcome.name = source.filename().string();
  outcome.sha256 = sha256;
  outcome.size_bytes =
      static_cast<std::int64_t>(fs::file_size(source));
  const fs::path target_dir = fs::path(archive_dir) / sha256;
  fs::create_directories(target_dir);
  const fs::path target = target_dir / outcome.name;

  bool exists = fs::exists(target);
  if (exists) {
    bool ok = false;
    const std::string existing_sha =
        planner::grid_search::bench::sha256_file_hex(target.string(), &ok);
    if (!ok || existing_sha != sha256) {
      throw std::runtime_error("archive collision at " + target.string() +
                               " (different content for same SHA256)");
    }
  } else {
    const fs::path temp = target_dir / (outcome.name + ".tmp");
    fs::copy_file(source, temp, fs::copy_options::overwrite_existing);
    fs::rename(temp, target);
  }

  planner::grid_search::bench::write_text_file(
      (target_dir / (outcome.name + ".sha256")).string(),
      sha256 + "  " + outcome.name + "\n");
  json meta;
  meta["type"] = "grid_bench_instance_archive";
  meta["schema_version"] = 1;
  meta["name"] = outcome.name;
  meta["sha256"] = sha256;
  meta["size_bytes"] = outcome.size_bytes;
  meta["generated_by_run_id"] =
      planner::grid_search::bench::env_string("PLANNER_RUN_ID", "unmonitored");
  meta["generated_at"] = planner::grid_search::bench::utc_now_iso8601();
  meta["generator"] = "grid_scenario_gen v1";
  meta["class"] = instance.scenario_class;
  meta["scale"] = instance.scale;
  meta["seed"] = instance.seed;
  meta["width"] = instance.width;
  meta["height"] = instance.height;
  meta["resolution_m"] = instance.resolution_m;
  meta["start"] = {instance.start.row, instance.start.col};
  meta["goal"] = {instance.goal.row, instance.goal.col};
  meta["distance_source"] = {instance.distance_source.row,
                             instance.distance_source.col};
  meta["expected"] = instance.expected;
  meta["note"] =
      ">10 MiB 输入按 benchmark_tiers.md §3.4 SHA256 寻址归档；"
      "生成规则与种子为权威，打包 uint8 行为内容寻址对象";
  planner::grid_search::bench::write_text_file(
      (target_dir / (outcome.name + ".meta.json")).string(),
      meta.dump(2) + "\n");

  outcome.relative_path =
      relative_to_repo(target.string(), repo_root);
  return outcome;
}

json rule_notes() {
  json rules;
  rules["open"] = "外围墙 1 格；起终点对角区域，边距 m=2+(seed%3)";
  rules["maze"] =
      "递归回溯；房间间距 pitch=max(8,规模/16)（16×16 房间）；"
      "墙厚 1–2（seed 奇偶）；房间内部 ≥4；门宽 3；左下房间→右上房间";
  rules["corridor"] =
      "两道平行墙形成宽 3 格通道，贯穿 75% 边长（列 [规模/8, 规模-规模/8-1]），"
      "通道中心行 = 规模/2+(seed-2)·max(1,规模/64)；两端开阔";
  rules["deadend"] =
      "U 形袋：入口宽 3，袋深 max(10,规模/16)+seed ≥10；"
      "袋底为终点，袋外开阔区为起点";
  rules["unreachable"] =
      "终点位于封闭腔（墙厚 1、无开口），腔半宽 max(4,规模/64)，"
      "中心行 = 规模/2+(seed-2)·max(1,规模/32)";
  rules["border_corner"] =
      "沿左下边界 ≥3 处 1 格宽对角障碍对（base=1+3k）；"
      "右侧贴边通道宽 3（墙列 规模-5，行 [规模/8, 规模-2]），底部留入口";
  rules["resolution_m"] = kBenchResolutionM;
  rules["start_goal"] =
      "每实例固定 1 对起终点 + 1 个距离场源点；期望 success/no_path 由生成器"
      "独立 BFS 自检（四/八邻域均验证）";
  return rules;
}

}  // namespace

int main(int argc, char** argv) {
  using namespace planner::grid_search::bench;
  const Options options = parse_args(argc, argv);
  const fs::path output_dir = options.output_dir;
  const fs::path instances_dir = output_dir / "instances";

  std::error_code error;
  fs::create_directories(instances_dir, error);
  if (error) {
    std::cerr << "cannot create output dir: " << error.message() << "\n";
    return 2;
  }

  const std::string run_id = env_string("PLANNER_RUN_ID", "unmonitored");
  const std::string created_at = utc_now_iso8601();

  json instances_json;
  instances_json["type"] = "grid_bench_instances";
  instances_json["schema_version"] = 1;
  instances_json["generator"] = "grid_scenario_gen v1";
  instances_json["generator_version"] = kGeneratorVersion;
  instances_json["suite"] = options.suite;
  instances_json["mode"] = options.capacity ? "capacity" : "formal";
  instances_json["created_at"] = created_at;
  instances_json["run_id"] = run_id;
  instances_json["resolution_m"] = kBenchResolutionM;
  instances_json["tolerance_note"] = "1e-8 + 1e-8*abs(reference)";
  instances_json["archive_threshold_bytes"] = options.archive_threshold;
  instances_json["rules"] = rule_notes();

  json entries = json::array();
  json archive_entries = json::array();
  json problems = json::array();
  std::map<std::string, int> by_class;
  std::map<std::string, int> by_scale;
  std::vector<std::pair<std::string, std::string>> hash_lines;  // (id, line)
  std::int64_t materialized_bytes = 0;
  std::int64_t archived_bytes = 0;
  int validation_failures = 0;

  try {
    for (const std::string& scenario_class : options.classes) {
      for (int scale : options.scales) {
        for (int seed : options.seeds) {
          const ScenarioInstance instance =
              generate_instance(scenario_class, scale, seed);
          const InstanceValidation validation = validate_instance(instance);
          if (!validation.ok) {
            ++validation_failures;
            for (const std::string& problem : validation.problems) {
              problems.push_back(instance.instance_id + ": " + problem);
            }
          }
          const std::string sha = instance_cells_sha256(instance);
          const std::string file_name = instance.instance_id + ".u8";
          const fs::path local_path = instances_dir / file_name;
          if (!write_binary_file(local_path.string(), instance.cells)) {
            std::cerr << "cannot write " << local_path << "\n";
            return 2;
          }

          json entry;
          entry["instance_id"] = instance.instance_id;
          entry["class"] = instance.scenario_class;
          entry["scale"] = instance.scale;
          entry["seed"] = instance.seed;
          entry["width"] = instance.width;
          entry["height"] = instance.height;
          entry["resolution_m"] = instance.resolution_m;
          entry["start"] = {instance.start.row, instance.start.col};
          entry["goal"] = {instance.goal.row, instance.goal.col};
          entry["distance_source"] = {instance.distance_source.row,
                                      instance.distance_source.col};
          entry["expected"] = instance.expected;
          json cells;
          cells["sha256"] = sha;
          cells["size_bytes"] =
              static_cast<std::int64_t>(instance.cells.size());

          if (static_cast<std::int64_t>(instance.cells.size()) >
              options.archive_threshold) {
            if (options.archive_dir.empty()) {
              std::cerr << "instance " << instance.instance_id << " is "
                        << instance.cells.size()
                        << " bytes (> threshold) but --archive-dir was not "
                           "given; refusing to keep a >10 MiB input in the "
                           "run outputs (benchmark_tiers.md §3.4)\n";
              return 2;
            }
            const ArchiveOutcome archived = archive_file(
                local_path, sha, options.archive_dir, options.repo_root,
                instance);
            cells["archived"] = true;
            cells["archive_relative_path"] = archived.relative_path;
            archive_entries.push_back(
                {{"instance_id", instance.instance_id},
                 {"name", archived.name},
                 {"sha256", archived.sha256},
                 {"size_bytes", archived.size_bytes},
                 {"relative_path", archived.relative_path}});
            archived_bytes += archived.size_bytes;
            // 归档后删除输出目录中的大文件，仅保留 sidecar 记录。
            fs::remove(local_path, error);
            write_text_file((instances_dir / (file_name + ".sha256")).string(),
                            sha + "  " + file_name + "\n");
            hash_lines.emplace_back(instance.instance_id,
                                    sha + "  " + archived.relative_path);
          } else {
            cells["archived"] = false;
            cells["local_path"] = "instances/" + file_name;
            materialized_bytes +=
                static_cast<std::int64_t>(instance.cells.size());
            hash_lines.emplace_back(instance.instance_id,
                                    sha + "  instances/" + file_name);
          }
          entry["cells"] = cells;
          entries.push_back(entry);
          by_class[instance.scenario_class] += 1;
          by_scale[std::to_string(instance.scale)] += 1;
        }
      }
    }
  } catch (const std::exception& exception) {
    std::cerr << "generation failed: " << exception.what() << "\n";
    return 2;
  }

  std::sort(hash_lines.begin(), hash_lines.end());
  std::string manifest;
  for (const auto& [id, line] : hash_lines) {
    (void)id;
    manifest += line + "\n";
  }
  write_text_file((output_dir / "instances.sha256").string(), manifest);

  instances_json["counts"] = {{"total", static_cast<int>(entries.size())},
                              {"by_class", by_class},
                              {"by_scale", by_scale}};
  instances_json["instances"] = entries;
  instances_json["archive"] = archive_entries;
  write_text_file((output_dir / "instances.json").string(),
                  instances_json.dump(2) + "\n");

  json summary;
  summary["type"] = "grid_bench_generation_summary";
  summary["schema_version"] = 1;
  summary["generator"] = "grid_scenario_gen v1";
  summary["generator_version"] = kGeneratorVersion;
  summary["suite"] = options.suite;
  summary["mode"] = options.capacity ? "capacity" : "formal";
  summary["run_id"] = run_id;
  summary["created_at"] = created_at;
  summary["output_dir"] = output_dir.string();
  summary["instance_count"] = static_cast<int>(entries.size());
  summary["counts_by_class"] = by_class;
  summary["counts_by_scale"] = by_scale;
  summary["materialized_bytes"] = materialized_bytes;
  summary["archived_bytes"] = archived_bytes;
  summary["archive"] = archive_entries;
  summary["validation"] = {{"passed", static_cast<int>(entries.size()) -
                                          validation_failures},
                           {"failed", validation_failures},
                           {"problems", problems}};
  summary["outputs"] = {{"instances_json", "instances.json"},
                        {"instances_manifest", "instances.sha256"},
                        {"instances_dir", "instances/"}};
  write_text_file((output_dir / "generation_summary.json").string(),
                  summary.dump(2) + "\n");

  std::cout << "grid_scenario_gen v1: " << entries.size() << " instances ("
            << options.classes.size() << " classes x " << options.scales.size()
            << " scales x " << options.seeds.size() << " seeds), "
            << "materialized " << materialized_bytes << " B, archived "
            << archived_bytes << " B, validation failures "
            << validation_failures << "\n";
  return validation_failures == 0 ? 0 : 1;
}
