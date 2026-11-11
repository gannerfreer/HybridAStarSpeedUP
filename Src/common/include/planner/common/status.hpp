/**
 * @file status.hpp
 * @brief 规划器与几何求解模块的执行状态码定义
 */

#pragma once

#include <string_view>

namespace planner::common {

/**
 * @brief 规划器与几何求解模块的执行状态码
 */
enum class Status {
  kSuccess,             ///< 求解成功，已找到满足要求的轨迹
  kInvalidInput,        ///< 输入参数无效或超出定义域
  kNoPath,              ///< 搜索结束但未找到无碰撞的可行路径
  kTimeout,             ///< 超过指定计算耗时预算，规划终止
  kCancelled,           ///< 外部客户端请求或回调触发主动取消
  kOptimizationFailed,  ///< 数值求解器或轨迹优化迭代收敛失败
  kValidationFailed,    ///< 候选路径未通过搜索后的安全完整性校验
};

/**
 * @brief 将执行状态码转换为易读的字符串形式
 * @param[in] status 状态码枚举值
 * @return 对应的状态描述字符串视图；未知枚举值返回 "unknown"
 */
[[nodiscard]] std::string_view to_string(Status status) noexcept;

}  // namespace planner::common
