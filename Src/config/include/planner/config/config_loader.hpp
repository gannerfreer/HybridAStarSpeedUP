/**
 * @file config_loader.hpp
 * @brief TOML 配置加载公共接口（公开头文件不依赖 toml++）。
 */

#pragma once

#include <string_view>

#include "planner/common/planner_config.hpp"
#include "planner/common/scenario_config.hpp"
#include "planner/common/vehicle.hpp"
#include "planner/config/config_result.hpp"

namespace planner::config {

/**
 * @brief 配置加载选项。
 */
struct ConfigLoadOptions {
  bool allow_unknown_keys{false};  ///< 是否忽略未知键（默认严格拒绝）。
};

/**
 * @brief 加载并校验车辆模型配置。
 * @param[in] path TOML 文件路径。
 * @param[in] options 加载选项。
 * @return 成功时包含 VehicleModel，失败时包含 ConfigError。
 */
[[nodiscard]] ConfigResult<common::VehicleModel> load_vehicle_config(
    std::string_view path, const ConfigLoadOptions& options = {});

/**
 * @brief 加载并校验 Hybrid A* 规划器配置。
 * @param[in] path TOML 文件路径。
 * @param[in] options 加载选项。
 * @return 成功时包含 UserAstarConfig，失败时包含 ConfigError。
 */
[[nodiscard]] ConfigResult<common::UserAstarConfig> load_planner_config(
    std::string_view path, const ConfigLoadOptions& options = {});

/**
 * @brief 加载并校验场景配置。
 * @param[in] path TOML 文件路径。
 * @param[in] options 加载选项。
 * @return 成功时包含 ScenarioConfig，失败时包含 ConfigError。
 */
[[nodiscard]] ConfigResult<common::ScenarioConfig> load_scenario_config(
    std::string_view path, const ConfigLoadOptions& options = {});

}  // namespace planner::config
