/**
 * @file toml_config_loader.cpp
 * @brief 基于 toml++ 的车辆、规划器与场景配置加载及校验实现。
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <initializer_list>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <toml++/toml.hpp>
#include <utility>
#include <vector>

#include "planner/config/config_loader.hpp"

namespace planner::config {

namespace {

using common::Status;

constexpr std::int64_t kSchemaVersion = 1;
constexpr const char* kSchemaVersionKey = "schema_version";

struct FieldRef {
  std::string token;  ///< 校验失败原因中用于定位字段的关键词。
  std::string key_path;    ///< 对应的点分键路径。
  const toml::node* node;  ///< 对应 TOML 节点（用于行列定位）。
};

std::string location_prefix(const std::string& file, int line, int column) {
  std::string prefix = file;
  if (line > 0) {
    prefix += ":" + std::to_string(line);
    if (column > 0) {
      prefix += ":" + std::to_string(column);
    }
  }
  return prefix;
}

ConfigError make_error(Status status, const std::string& file,
                       const std::string& key_path, int line, int column,
                       const std::string& message) {
  ConfigError error;
  error.status = status;
  error.file = file;
  error.key_path = key_path;
  error.line = line;
  error.column = column;
  error.message = location_prefix(file, line, column) + ": ";
  if (!key_path.empty()) {
    error.message += key_path + ": ";
  }
  error.message += message;
  return error;
}

ConfigError make_parse_error(const std::string& file,
                             const toml::parse_error& error) {
  return make_error(
      Status::kInvalidInput, file, "",
      static_cast<int>(error.source().begin.line),
      static_cast<int>(error.source().begin.column),
      std::string("parse error: ") + std::string(error.description()));
}

template <typename T>
ConfigResult<T> make_error_result(const ConfigError& error) {
  ConfigResult<T> result;
  result.error = error;
  return result;
}

template <typename T>
ConfigResult<T> make_value_result(T value) {
  ConfigResult<T> result;
  result.value = std::move(value);
  return result;
}

class Loader {
 public:
  Loader(std::string file, const ConfigLoadOptions& options)
      : file_(std::move(file)), options_(options) {}

  template <typename T>
  ConfigResult<T> error_result() const {
    return make_error_result<T>(error_);
  }

  bool fail_value(const toml::node& node, const std::string& key_path,
                  const std::string& message,
                  Status status = Status::kInvalidInput) {
    const toml::source_position begin = node.source().begin;
    return fail(status, key_path, &node, nullptr, message,
                static_cast<int>(begin.line), static_cast<int>(begin.column));
  }

  bool check_keys(const toml::table& table, const std::string& prefix,
                  std::initializer_list<const char*> allowed) {
    for (const auto& entry : table) {
      const toml::key& key = entry.first;
      const std::string name(key.str());
      bool known = false;
      for (const char* candidate : allowed) {
        if (name == candidate) {
          known = true;
          break;
        }
      }
      if (known) {
        continue;
      }
      if (options_.allow_unknown_keys) {
        continue;
      }
      const std::string key_path = prefix.empty() ? name : prefix + "." + name;
      return fail(Status::kInvalidInput, key_path, &entry.second, &key,
                  "unknown key", 0, 0);
    }
    return true;
  }

  bool read_schema_version(const toml::table& root) {
    const toml::node* node = root.get(kSchemaVersionKey);
    const std::string key_path(kSchemaVersionKey);
    if (node == nullptr) {
      return fail_missing(root, key_path);
    }
    if (!node->is_integer()) {
      return fail_type(*node, key_path, "an integer");
    }
    const std::optional<std::int64_t> value = node->value_exact<std::int64_t>();
    if (!value) {
      return fail_type(*node, key_path, "an integer");
    }
    if (*value != kSchemaVersion) {
      return fail_value(
          *node, key_path,
          "schema_version must be 1 (got " + std::to_string(*value) + ")");
    }
    return true;
  }

  const toml::table* read_table(const toml::table& parent, const char* key,
                                const std::string& key_path,
                                bool required = true) {
    const toml::node* node = parent.get(key);
    if (node == nullptr) {
      if (required) {
        fail_missing(parent, key_path);
      }
      return nullptr;
    }
    if (!node->is_table()) {
      fail_type(*node, key_path, "a table");
      return nullptr;
    }
    return node->as_table();
  }

  const toml::array* read_array(const toml::table& parent, const char* key,
                                const std::string& key_path,
                                bool required = true) {
    const toml::node* node = parent.get(key);
    if (node == nullptr) {
      if (required) {
        fail_missing(parent, key_path);
      }
      return nullptr;
    }
    if (!node->is_array()) {
      fail_type(*node, key_path, "an array");
      return nullptr;
    }
    return node->as_array();
  }

  bool read_double(const toml::table& parent, const char* key,
                   const std::string& key_path, bool required, double* out) {
    const toml::node* node = parent.get(key);
    if (node == nullptr) {
      return required ? fail_missing(parent, key_path) : true;
    }
    double value = 0.0;
    if (node->is_floating_point()) {
      const std::optional<double> parsed = node->value_exact<double>();
      if (!parsed) {
        return fail_type(*node, key_path, "a float or integer");
      }
      value = *parsed;
    } else if (node->is_integer()) {
      const std::optional<std::int64_t> parsed =
          node->value_exact<std::int64_t>();
      if (!parsed) {
        return fail_type(*node, key_path, "a float or integer");
      }
      value = static_cast<double>(*parsed);
    } else {
      return fail_type(*node, key_path, "a float or integer");
    }
    if (!std::isfinite(value)) {
      return fail_value(*node, key_path, "value must be finite");
    }
    *out = value;
    return true;
  }

  bool read_int(const toml::table& parent, const char* key,
                const std::string& key_path, bool required, int* out) {
    if (parent.get(key) == nullptr) {
      return required ? fail_missing(parent, key_path) : true;
    }
    std::int64_t raw = 0;
    if (!read_int64(parent, key, key_path, true, &raw)) {
      return false;
    }
    if (raw < static_cast<std::int64_t>(std::numeric_limits<int>::min()) ||
        raw > static_cast<std::int64_t>(std::numeric_limits<int>::max())) {
      return fail_value(*parent.get(key), key_path, "integer is out of range");
    }
    *out = static_cast<int>(raw);
    return true;
  }

  bool read_int64(const toml::table& parent, const char* key,
                  const std::string& key_path, bool required,
                  std::int64_t* out) {
    const toml::node* node = parent.get(key);
    if (node == nullptr) {
      return required ? fail_missing(parent, key_path) : true;
    }
    if (!node->is_integer()) {
      return fail_type(*node, key_path, "an integer");
    }
    const std::optional<std::int64_t> value = node->value_exact<std::int64_t>();
    if (!value) {
      return fail_type(*node, key_path, "an integer");
    }
    *out = *value;
    return true;
  }

  bool read_bool(const toml::table& parent, const char* key,
                 const std::string& key_path, bool required, bool* out) {
    const toml::node* node = parent.get(key);
    if (node == nullptr) {
      return required ? fail_missing(parent, key_path) : true;
    }
    if (!node->is_boolean()) {
      return fail_type(*node, key_path, "a boolean");
    }
    const std::optional<bool> value = node->value_exact<bool>();
    if (!value) {
      return fail_type(*node, key_path, "a boolean");
    }
    *out = *value;
    return true;
  }

  bool read_string(const toml::table& parent, const char* key,
                   const std::string& key_path, bool required,
                   std::string* out) {
    const toml::node* node = parent.get(key);
    if (node == nullptr) {
      return required ? fail_missing(parent, key_path) : true;
    }
    if (!node->is_string()) {
      return fail_type(*node, key_path, "a string");
    }
    const std::optional<std::string> value = node->value_exact<std::string>();
    if (!value) {
      return fail_type(*node, key_path, "a string");
    }
    *out = *value;
    return true;
  }

  void fail_validation(const toml::node& fallback_node,
                       const std::string& fallback_path,
                       const std::string& reason,
                       const std::vector<FieldRef>& fields) {
    for (const FieldRef& field : fields) {
      if (field.node != nullptr &&
          reason.find(field.token) != std::string::npos) {
        fail(Status::kInvalidInput, field.key_path, field.node, nullptr, reason,
             0, 0);
        return;
      }
    }
    fail(Status::kInvalidInput, fallback_path, &fallback_node, nullptr, reason,
         0, 0);
  }

 private:
  bool fail_missing(const toml::node& parent, const std::string& key_path) {
    const toml::source_position begin = parent.source().begin;
    return fail(Status::kInvalidInput, key_path, &parent, nullptr,
                "missing required key", static_cast<int>(begin.line),
                static_cast<int>(begin.column));
  }

  bool fail_type(const toml::node& node, const std::string& key_path,
                 const char* expected) {
    return fail(Status::kInvalidInput, key_path, &node, nullptr,
                std::string("expected ") + expected, 0, 0);
  }

  bool fail(Status status, const std::string& key_path, const toml::node* node,
            const toml::key* key, const std::string& message, int line,
            int column) {
    if (failed_) {
      return false;
    }
    if (line <= 0 || column <= 0) {
      if (key != nullptr) {
        const toml::source_position begin = key->source().begin;
        line = static_cast<int>(begin.line);
        column = static_cast<int>(begin.column);
      }
    }
    if ((line <= 0 || column <= 0) && node != nullptr) {
      const toml::source_position begin = node->source().begin;
      line = static_cast<int>(begin.line);
      column = static_cast<int>(begin.column);
    }
    error_ = make_error(status, file_, key_path, line, column, message);
    failed_ = true;
    return false;
  }

  std::string file_;
  ConfigLoadOptions options_;
  ConfigError error_;
  bool failed_{false};
};

std::optional<toml::table> parse_document(const std::string& path,
                                          ConfigError* error) {
  try {
    return toml::parse_file(path);
  } catch (const toml::parse_error& parse_error) {
    *error = make_parse_error(path, parse_error);
  } catch (const std::exception& exception) {
    *error =
        make_error(Status::kInvalidInput, path, "", 0, 0,
                   std::string("failed to read config: ") + exception.what());
  } catch (...) {
    *error = make_error(Status::kInvalidInput, path, "", 0, 0,
                        "failed to read config: unknown error");
  }
  return std::nullopt;
}

}  // namespace

ConfigResult<common::VehicleModel> load_vehicle_config(
    std::string_view path_view, const ConfigLoadOptions& options) {
  const std::string path(path_view);
  ConfigError parse_error;
  std::optional<toml::table> document = parse_document(path, &parse_error);
  if (!document) {
    return make_error_result<common::VehicleModel>(parse_error);
  }

  try {
    const toml::table& root = *document;
    Loader loader(path, options);
    if (!loader.check_keys(root, "",
                           {"schema_version", "id", "wheelbase_m", "width_m",
                            "front_overhang_m", "rear_overhang_m",
                            "max_steer_rad", "safety_margin_m"})) {
      return loader.error_result<common::VehicleModel>();
    }
    if (!loader.read_schema_version(root)) {
      return loader.error_result<common::VehicleModel>();
    }

    common::VehicleModel vehicle;
    if (!loader.read_string(root, "id", "id", false, &vehicle.id) ||
        !loader.read_double(root, "wheelbase_m", "wheelbase_m", true,
                            &vehicle.wheelbase_m) ||
        !loader.read_double(root, "width_m", "width_m", true,
                            &vehicle.width_m) ||
        !loader.read_double(root, "front_overhang_m", "front_overhang_m", true,
                            &vehicle.front_overhang_m) ||
        !loader.read_double(root, "rear_overhang_m", "rear_overhang_m", true,
                            &vehicle.rear_overhang_m) ||
        !loader.read_double(root, "max_steer_rad", "max_steer_rad", true,
                            &vehicle.max_steer_rad) ||
        !loader.read_double(root, "safety_margin_m", "safety_margin_m", false,
                            &vehicle.safety_margin_m)) {
      return loader.error_result<common::VehicleModel>();
    }

    std::string reason;
    if (!common::vehicle_valid(vehicle, &reason)) {
      const std::vector<FieldRef> fields = {
          {"wheelbase", "wheelbase_m", root.get("wheelbase_m")},
          {"front overhang", "front_overhang_m", root.get("front_overhang_m")},
          {"rear overhang", "rear_overhang_m", root.get("rear_overhang_m")},
          {"safety margin", "safety_margin_m", root.get("safety_margin_m")},
          {"max steer", "max_steer_rad", root.get("max_steer_rad")},
          {"width", "width_m", root.get("width_m")},
      };
      loader.fail_validation(root, "", reason, fields);
      return loader.error_result<common::VehicleModel>();
    }
    return make_value_result(std::move(vehicle));
  } catch (const std::exception& exception) {
    return make_error_result<common::VehicleModel>(
        make_error(Status::kInvalidInput, path, "", 0, 0,
                   std::string("failed to load config: ") + exception.what()));
  } catch (...) {
    return make_error_result<common::VehicleModel>(
        make_error(Status::kInvalidInput, path, "", 0, 0,
                   "failed to load config: unknown error"));
  }
}

ConfigResult<common::UserAstarConfig> load_planner_config(
    std::string_view path_view, const ConfigLoadOptions& options) {
  const std::string path(path_view);
  ConfigError parse_error;
  std::optional<toml::table> document = parse_document(path, &parse_error);
  if (!document) {
    return make_error_result<common::UserAstarConfig>(parse_error);
  }

  try {
    const toml::table& root = *document;
    Loader loader(path, options);
    if (!loader.check_keys(root, "",
                           {"schema_version", "search", "cost",
                            "analytic_expansion", "budget"})) {
      return loader.error_result<common::UserAstarConfig>();
    }
    if (!loader.read_schema_version(root)) {
      return loader.error_result<common::UserAstarConfig>();
    }

    const toml::table* search = nullptr;
    if (root.get("search") != nullptr) {
      search = loader.read_table(root, "search", "search");
      if (search == nullptr ||
          !loader.check_keys(
              *search, "search",
              {"xy_resolution_m", "heading_bins", "motion_step_m",
               "steering_samples", "allow_reverse", "backward_search_range_m",
               "linear_preferred_distance_m2", "goal_capture_radius_m"})) {
        return loader.error_result<common::UserAstarConfig>();
      }
    }

    const toml::table* cost = nullptr;
    if (root.get("cost") != nullptr) {
      cost = loader.read_table(root, "cost", "cost");
      if (cost == nullptr ||
          !loader.check_keys(*cost, "cost",
                             {"forward_cost", "reverse_cost", "turn_penalty",
                              "switch_cost"})) {
        return loader.error_result<common::UserAstarConfig>();
      }
    }

    const toml::table* analytic = nullptr;
    if (root.get("analytic_expansion") != nullptr) {
      analytic =
          loader.read_table(root, "analytic_expansion", "analytic_expansion");
      if (analytic == nullptr ||
          !loader.check_keys(*analytic, "analytic_expansion",
                             {"ratio", "shot_distance_m", "max_length_m",
                              "min_connector_segment_m"})) {
        return loader.error_result<common::UserAstarConfig>();
      }
    }

    const toml::table* budget = nullptr;
    if (root.get("budget") != nullptr) {
      budget = loader.read_table(root, "budget", "budget");
      if (budget == nullptr || !loader.check_keys(*budget, "budget",
                                                  {"default_time_budget_s",
                                                   "default_node_budget"})) {
        return loader.error_result<common::UserAstarConfig>();
      }
    }

    common::UserAstarConfig config;
    if ((search != nullptr &&
         (!loader.read_double(*search, "xy_resolution_m",
                              "search.xy_resolution_m", false,
                              &config.xy_resolution_m) ||
          !loader.read_int(*search, "heading_bins", "search.heading_bins",
                           false, &config.heading_bins) ||
          !loader.read_double(*search, "motion_step_m", "search.motion_step_m",
                              false, &config.motion_step_m) ||
          !loader.read_int(*search, "steering_samples",
                           "search.steering_samples", false,
                           &config.steering_samples) ||
          !loader.read_bool(*search, "allow_reverse", "search.allow_reverse",
                            false, &config.allow_reverse) ||
          !loader.read_double(*search, "backward_search_range_m",
                              "search.backward_search_range_m", false,
                              &config.backward_search_range_m) ||
          !loader.read_double(*search, "linear_preferred_distance_m2",
                              "search.linear_preferred_distance_m2", false,
                              &config.linear_preferred_distance_square) ||
          !loader.read_double(*search, "goal_capture_radius_m",
                              "search.goal_capture_radius_m", false,
                              &config.goal_capture_radius_m))) ||
        (cost != nullptr &&
         (!loader.read_double(*cost, "forward_cost", "cost.forward_cost", false,
                              &config.forward_cost) ||
          !loader.read_double(*cost, "reverse_cost", "cost.reverse_cost", false,
                              &config.reverse_cost) ||
          !loader.read_double(*cost, "turn_penalty", "cost.turn_penalty", false,
                              &config.turn_penalty) ||
          !loader.read_double(*cost, "switch_cost", "cost.switch_cost", false,
                              &config.switch_cost))) ||
        (analytic != nullptr &&
         (!loader.read_int(*analytic, "ratio", "analytic_expansion.ratio",
                           false, &config.analytic_expansion_ratio) ||
          !loader.read_double(*analytic, "shot_distance_m",
                              "analytic_expansion.shot_distance_m", false,
                              &config.analytic_shot_distance_m) ||
          !loader.read_double(*analytic, "max_length_m",
                              "analytic_expansion.max_length_m", false,
                              &config.analytic_expansion_max_length_m) ||
          !loader.read_double(*analytic, "min_connector_segment_m",
                              "analytic_expansion.min_connector_segment_m",
                              false, &config.min_connector_segment_m))) ||
        (budget != nullptr &&
         (!loader.read_double(*budget, "default_time_budget_s",
                              "budget.default_time_budget_s", false,
                              &config.default_time_budget_s) ||
          !loader.read_int64(*budget, "default_node_budget",
                             "budget.default_node_budget", false,
                             &config.default_node_budget)))) {
      return loader.error_result<common::UserAstarConfig>();
    }

    std::string reason;
    if (!common::planner_config_valid(config, &reason)) {
      const std::vector<FieldRef> fields = {
          {"xy_resolution_m", "search.xy_resolution_m",
           search != nullptr ? search->get("xy_resolution_m") : nullptr},
          {"heading_bins", "search.heading_bins",
           search != nullptr ? search->get("heading_bins") : nullptr},
          {"motion_step_m", "search.motion_step_m",
           search != nullptr ? search->get("motion_step_m") : nullptr},
          {"steering_samples", "search.steering_samples",
           search != nullptr ? search->get("steering_samples") : nullptr},
          {"backward_search_range_m", "search.backward_search_range_m",
           search != nullptr ? search->get("backward_search_range_m")
                             : nullptr},
          {"linear_preferred_distance_square",
           "search.linear_preferred_distance_m2",
           search != nullptr ? search->get("linear_preferred_distance_m2")
                             : nullptr},
          {"goal_capture_radius_m", "search.goal_capture_radius_m",
           search != nullptr ? search->get("goal_capture_radius_m") : nullptr},
          {"forward_cost", "cost.forward_cost",
           cost != nullptr ? cost->get("forward_cost") : nullptr},
          {"reverse_cost", "cost.reverse_cost",
           cost != nullptr ? cost->get("reverse_cost") : nullptr},
          {"turn_penalty", "cost.turn_penalty",
           cost != nullptr ? cost->get("turn_penalty") : nullptr},
          {"switch_cost", "cost.switch_cost",
           cost != nullptr ? cost->get("switch_cost") : nullptr},
          {"analytic_expansion_ratio", "analytic_expansion.ratio",
           analytic != nullptr ? analytic->get("ratio") : nullptr},
          {"analytic_shot_distance_m", "analytic_expansion.shot_distance_m",
           analytic != nullptr ? analytic->get("shot_distance_m") : nullptr},
          {"analytic_expansion_max_length_m", "analytic_expansion.max_length_m",
           analytic != nullptr ? analytic->get("max_length_m") : nullptr},
          {"min_connector_segment_m",
           "analytic_expansion.min_connector_segment_m",
           analytic != nullptr ? analytic->get("min_connector_segment_m")
                               : nullptr},
          {"default_time_budget_s", "budget.default_time_budget_s",
           budget != nullptr ? budget->get("default_time_budget_s") : nullptr},
          {"default_node_budget", "budget.default_node_budget",
           budget != nullptr ? budget->get("default_node_budget") : nullptr},
      };
      loader.fail_validation(root, "", reason, fields);
      return loader.error_result<common::UserAstarConfig>();
    }
    return make_value_result(std::move(config));
  } catch (const std::exception& exception) {
    return make_error_result<common::UserAstarConfig>(
        make_error(Status::kInvalidInput, path, "", 0, 0,
                   std::string("failed to load config: ") + exception.what()));
  } catch (...) {
    return make_error_result<common::UserAstarConfig>(
        make_error(Status::kInvalidInput, path, "", 0, 0,
                   "failed to load config: unknown error"));
  }
}

ConfigResult<common::ScenarioConfig> load_scenario_config(
    std::string_view path_view, const ConfigLoadOptions& options) {
  const std::string path(path_view);
  ConfigError parse_error;
  std::optional<toml::table> document = parse_document(path, &parse_error);
  if (!document) {
    return make_error_result<common::ScenarioConfig>(parse_error);
  }

  try {
    const toml::table& root = *document;
    Loader loader(path, options);
    if (!loader.check_keys(
            root, "",
            {"schema_version", "map", "start", "goal", "obstacles"})) {
      return loader.error_result<common::ScenarioConfig>();
    }
    if (!loader.read_schema_version(root)) {
      return loader.error_result<common::ScenarioConfig>();
    }

    const toml::table* map = loader.read_table(root, "map", "map");
    if (map == nullptr ||
        !loader.check_keys(*map, "map",
                           {"width", "height", "resolution_m", "origin"})) {
      return loader.error_result<common::ScenarioConfig>();
    }

    common::ScenarioConfig scenario;
    if (!loader.read_int(*map, "width", "map.width", true, &scenario.width) ||
        !loader.read_int(*map, "height", "map.height", true,
                         &scenario.height) ||
        !loader.read_double(*map, "resolution_m", "map.resolution_m", true,
                            &scenario.resolution_m)) {
      return loader.error_result<common::ScenarioConfig>();
    }
    std::vector<FieldRef> validation_fields = {
        {"width", "map.width", map->get("width")},
        {"height", "map.height", map->get("height")},
        {"resolution", "map.resolution_m", map->get("resolution_m")},
        {"origin", "map.origin",
         map->get("origin") != nullptr ? map->get("origin") : map},
        {"start", "start", root.get("start")},
        {"goal", "goal", root.get("goal")},
    };

    if (map->get("origin") != nullptr) {
      const toml::table* origin =
          loader.read_table(*map, "origin", "map.origin");
      if (origin == nullptr || !loader.check_keys(*origin, "map.origin",
                                                  {"x_m", "y_m", "yaw_rad"})) {
        return loader.error_result<common::ScenarioConfig>();
      }
      if (!loader.read_double(*origin, "x_m", "map.origin.x_m", false,
                              &scenario.origin.x_m) ||
          !loader.read_double(*origin, "y_m", "map.origin.y_m", false,
                              &scenario.origin.y_m) ||
          !loader.read_double(*origin, "yaw_rad", "map.origin.yaw_rad", false,
                              &scenario.origin.yaw_rad)) {
        return loader.error_result<common::ScenarioConfig>();
      }
    }

    const auto read_pose = [&loader](const toml::table& parent, const char* key,
                                     const std::string& key_path,
                                     math::Pose2d* pose) {
      const toml::table* table = loader.read_table(parent, key, key_path);
      if (table == nullptr ||
          !loader.check_keys(*table, key_path, {"x_m", "y_m", "yaw_rad"})) {
        return false;
      }
      return loader.read_double(*table, "x_m", key_path + ".x_m", true,
                                &pose->x_m) &&
             loader.read_double(*table, "y_m", key_path + ".y_m", true,
                                &pose->y_m) &&
             loader.read_double(*table, "yaw_rad", key_path + ".yaw_rad", false,
                                &pose->yaw_rad);
    };

    if (!read_pose(root, "start", "start", &scenario.start) ||
        !read_pose(root, "goal", "goal", &scenario.goal)) {
      return loader.error_result<common::ScenarioConfig>();
    }

    const toml::node* obstacles_node = root.get("obstacles");
    if (obstacles_node != nullptr) {
      const toml::array* obstacles =
          loader.read_array(root, "obstacles", "obstacles");
      if (obstacles == nullptr) {
        return loader.error_result<common::ScenarioConfig>();
      }
      for (std::size_t i = 0; i < obstacles->size(); ++i) {
        const std::string item_path = "obstacles[" + std::to_string(i) + "]";
        const toml::node* item = obstacles->get(i);
        if (item == nullptr || !item->is_table()) {
          loader.fail_value(item != nullptr ? *item : *obstacles_node,
                            item_path, "expected a table");
          return loader.error_result<common::ScenarioConfig>();
        }
        const toml::table& obstacle_table = *item->as_table();
        if (!loader.check_keys(obstacle_table, item_path, {"id", "vertices"})) {
          return loader.error_result<common::ScenarioConfig>();
        }

        common::ObstacleConfig obstacle;
        if (!loader.read_string(obstacle_table, "id", item_path + ".id", true,
                                &obstacle.id)) {
          return loader.error_result<common::ScenarioConfig>();
        }
        const toml::node* vertices_node = obstacle_table.get("vertices");
        const toml::array* vertices = loader.read_array(
            obstacle_table, "vertices", item_path + ".vertices");
        if (vertices == nullptr) {
          return loader.error_result<common::ScenarioConfig>();
        }
        validation_fields.push_back({"obstacle " + std::to_string(i) + " id",
                                     item_path + ".id",
                                     obstacle_table.get("id")});
        validation_fields.push_back(
            {"obstacle " + std::to_string(i) + " vertices",
             item_path + ".vertices", vertices_node});
        obstacle.vertices.reserve(vertices->size());
        for (std::size_t j = 0; j < vertices->size(); ++j) {
          const std::string vertex_path =
              item_path + ".vertices[" + std::to_string(j) + "]";
          const toml::node* vertex_node = vertices->get(j);
          if (vertex_node == nullptr || !vertex_node->is_table()) {
            loader.fail_value(
                vertex_node != nullptr ? *vertex_node : *vertices_node,
                vertex_path, "expected a table");
            return loader.error_result<common::ScenarioConfig>();
          }
          const toml::table& vertex_table = *vertex_node->as_table();
          if (!loader.check_keys(vertex_table, vertex_path, {"x_m", "y_m"})) {
            return loader.error_result<common::ScenarioConfig>();
          }
          math::Point2d vertex;
          if (!loader.read_double(vertex_table, "x_m", vertex_path + ".x_m",
                                  true, &vertex.x_m) ||
              !loader.read_double(vertex_table, "y_m", vertex_path + ".y_m",
                                  true, &vertex.y_m)) {
            return loader.error_result<common::ScenarioConfig>();
          }
          obstacle.vertices.push_back(vertex);
        }
        scenario.obstacles.push_back(std::move(obstacle));
      }
    }

    std::string reason;
    if (!common::scenario_config_valid(scenario, &reason)) {
      loader.fail_validation(root, "", reason, validation_fields);
      return loader.error_result<common::ScenarioConfig>();
    }
    return make_value_result(std::move(scenario));
  } catch (const std::exception& exception) {
    return make_error_result<common::ScenarioConfig>(
        make_error(Status::kInvalidInput, path, "", 0, 0,
                   std::string("failed to load config: ") + exception.what()));
  } catch (...) {
    return make_error_result<common::ScenarioConfig>(
        make_error(Status::kInvalidInput, path, "", 0, 0,
                   "failed to load config: unknown error"));
  }
}

}  // namespace planner::config
