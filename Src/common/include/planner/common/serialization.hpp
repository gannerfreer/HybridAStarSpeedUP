#pragma once

// 第一版公开类型的 UTF-8 JSON 编码/解码。这里是定义线上契约的唯一位置；
// 各组件不得重新实现编码（见 Docs/component_development_plan.md 第 6.1 节
// 和 tests/fixtures/v1/serialization/README.md）。
//
// 文档信封（独立文件）：
//   { "schema_version": 1, "type": "<type-name>", <payload fields...> }
// 嵌套的载荷对象只携带其载荷字段，不带信封。解码时容忍未知的可选字段；
// NaN/Infinity、未知的大版本模式、缺少必需字段以及 JSON 类型错误
// 都会以 SerializationError 拒绝。

#include <string>
#include <string_view>

#include <nlohmann/json.hpp>

#include "planner/common/errors.hpp"
#include "planner/common/types.hpp"

namespace planner::common {

// 编解码器使用的严格枚举解析（未知值抛出 SerializationError；
// 规范字符串来自 to_string()）。
Direction direction_from_string(std::string_view text);
Status status_from_string(std::string_view text);

// 按类型的 JSON 编解码器。完整特化定义在 serialization.cpp 中；
// 显式实例化覆盖所有第一版公开类型，因此这些模板只对所列类型
// （Pose2d、VehicleModel、GridView、PathPoint、PathSegment、TrajectoryPoint、
// PlanRequest、PlanResult、Status、Corridor）可编译。
template <typename T>
struct json_codec;

// 编码/解码单个载荷对象（嵌套形式，不带信封）。
template <typename T>
nlohmann::json encode_payload(const T& value);
template <typename T>
T decode_payload(const nlohmann::json& payload);

// 编码/解码带 schema_version/type 信封的独立文档。
// `indent` 是 nlohmann dump 的缩进（默认 2）。
// 文档被拒绝时抛出 SerializationError。
template <typename T>
std::string encode_document(const T& value, int indent = 2);
template <typename T>
T decode_document(std::string_view utf8_text);

}  // namespace planner::common
