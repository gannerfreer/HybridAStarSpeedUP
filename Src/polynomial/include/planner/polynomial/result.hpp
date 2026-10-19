#pragma once

// planner_polynomial 的状态/结果词汇（Wave 3）。
//
// polynomial 组件对预期的输入问题从不抛异常：工厂函数和求值返回带稳定
// `Code` 的 `Result<T>`，该错误码可映射为规划结果使用的共享
// planner::common::Status。编程错误（向量索引越界）仍然抛异常。

#include <cstddef>
#include <utility>

#include "planner/common/types.hpp"

namespace planner::polynomial {

// 结果码。`to_string` 保持稳定，供测试和示例使用。
enum class Code {
  kOk = 0,
  kOutOfRange,             // 时间超出声明的局部轨迹定义域
  kUnsupportedDerivative,  // 求导阶数超出第一版范围 0..3
  kInvalidInput,           // 非有限值、非正时长、空分段列表、
                           // 边界不匹配、批处理缓冲区为空
};

// 简短稳定名称，仅用于诊断。
const char* to_string(Code code) noexcept;

// 将组件错误码映射到共享状态词汇。所有失败都映射为 kInvalidInput，
// 因为调用方提供的参数超出了声明的多项式契约；这里不对资源终止建模。
planner::common::Status to_common_status(Code code) noexcept;

// 值加状态。仅当 `ok()` 为 true 时 `value` 才有意义。
template <typename T>
struct Result {
  T value{};
  Code code{Code::kInvalidInput};

  bool ok() const noexcept { return code == Code::kOk; }

  static Result success(T result) { return Result{std::move(result), Code::kOk}; }
  static Result failure(Code failure_code) { return Result{T{}, failure_code}; }
};

// 写入调用方提供缓冲区的批处理调用的结果。
// 已写入 `processed` 个值；当 `code != Code::kOk` 时，
// `first_bad_index` 是第一个被拒绝输入的索引（其前面的值有效，
// 且已写入输出缓冲区）。
struct BatchResult {
  Code code{Code::kOk};
  std::size_t processed{0};
  std::size_t first_bad_index{0};

  bool ok() const noexcept { return code == Code::kOk; }
};

// 所有曲线接口共享的第一版求导阶数范围
// （Docs/component_development_plan.md 第 6.1 节）。
inline constexpr int kMaxDerivativeOrder = 3;

}  // namespace planner::polynomial
