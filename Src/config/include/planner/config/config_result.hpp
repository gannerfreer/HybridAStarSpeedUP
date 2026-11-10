/**
 * @file config_result.hpp
 * @brief 配置加载错误与泛型加载结果定义。
 */

#pragma once

#include <optional>
#include <string>

#include "planner/common/status.hpp"

namespace planner::config {

/**
 * @brief 配置加载或校验失败时的诊断信息。
 */
struct ConfigError {
  common::Status status{common::Status::kInvalidInput};  ///< 对应核心状态码。
  std::string message;   ///< 含文件、键路径与行列位置的完整诊断文本。
  std::string file;      ///< 配置文件的原始路径。
  std::string key_path;  ///< 出错的点分键路径（解析错误时为空）。
  int line{0};           ///< 出错位置行号（1 起，未知时为 0）。
  int column{0};         ///< 出错位置列号（1 起，未知时为 0）。
};

/**
 * @brief 配置加载结果：成功时携带值，失败时携带诊断信息。
 * @tparam T 加载得到的配置类型。
 */
template <typename T>
struct ConfigResult {
  std::optional<T> value;  ///< 成功时的配置值。
  ConfigError error;       ///< 失败时的诊断信息。

  /**
   * @brief 判定配置是否加载成功。
   * @return 成功返回 true。
   */
  operator bool() const noexcept { return value.has_value(); }
};

}  // namespace planner::config
