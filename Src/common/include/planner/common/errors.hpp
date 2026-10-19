#pragma once

// planner_common JSON 编解码器抛出的错误类别。
//
// 错误码保持稳定，测试、工具和调用方无需解析消息文本即可区分拒绝原因。
// 这些错误所执行的冻结契约见
// tests/fixtures/v1/serialization/README.md。

#include <stdexcept>
#include <string>
#include <utility>

namespace planner::common {

enum class ErrorCode {
  kParseError,                // 输入不是合法的 JSON
  kInvalidDocument,           // 文档根节点不是 JSON 对象
  kMissingField,              // 缺少必需字段
  kTypeError,                 // 字段的 JSON 类型错误
  kUnsupportedSchemaVersion,  // schema_version 为整数但不是受支持的版本 1
  kUnknownType,               // "type" 与请求的 C++ 类型不匹配
  kInvalidValue,              // 取值违反范围或不变量
  kUnknownEnumValue,          // 字符串枚举值不属于契约
  kNonFiniteValue,            // NaN 或 Infinity（JSON 无法表示）
};

// 错误码的简短稳定名称（仅用于诊断）。
const char* to_string(ErrorCode code) noexcept;

// planner_common 每次解码/编码失败都会抛出该异常。
class SerializationError : public std::runtime_error {
 public:
  SerializationError(ErrorCode code, std::string message)
      : std::runtime_error(std::move(message)), code_(code) {}

  ErrorCode code() const noexcept { return code_; }

 private:
  ErrorCode code_;
};

}  // namespace planner::common
