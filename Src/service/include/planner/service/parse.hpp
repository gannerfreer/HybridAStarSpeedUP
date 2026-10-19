#pragma once

// POST /plan/submit 请求体的严格解析器（障碍物 schema 与 web_scene 对齐，
// algorithm_params 见 Src/service/README.md）。
//
// 解析规则：
//   * 请求体必须是 JSON 对象；必填字段必须存在，类型和取值符合文档，
//     且数值必须有限；
//   * 顶层请求中的未知字段被容忍（向前兼容），而 algorithm_params 及其
//     corridor/dftpav 子对象内的未知键会带明确原因被拒绝（拼错的算法
//     参数绝不能静默忽略）；
//   * 矩形会被归一化（交换 min/max）；退化矩形以及非凸或退化多边形会被拒绝。

#include <string>

#include <nlohmann/json.hpp>

#include "planner/service/types.hpp"

namespace planner::service {

struct ParseOutcome {
  bool ok{false};
  std::string reason;
  std::string detail;

  static ParseOutcome success() { return {true, {}, {}}; }
  static ParseOutcome failure(std::string reason_text) {
    return {false, std::move(reason_text), {}};
  }
};

// 解析一个请求。成功时 `out` 被完整填充；失败时 `outcome.reason` 是稳定的
// 错误码（"missing_field"、"invalid_field"、"unknown_algorithm_parameter"、
// "invalid_vehicle"、"invalid_obstacle"、"vehicle_ref_error"），
// `outcome.detail` 指出有问题的字段。
ParseOutcome parse_plan_request(const nlohmann::json& body,
                                ServicePlanRequest* out);

// 解析一个 web_scene 障碍物对象（{"id", "type": "rectangle" |
// "convex_polygon", ...}）。
ParseOutcome parse_obstacle(const nlohmann::json& value, ServiceObstacle* out);

// 解析一个 vehicle_model 文档（payload 字段；容忍额外字段）。
ParseOutcome parse_vehicle(const nlohmann::json& value, common::VehicleModel* out);

// 从 JSON 文件加载 vehicle_model 文档（支持 vehicle_ref）。
ParseOutcome load_vehicle_file(const std::string& path, common::VehicleModel* out);

}  // namespace planner::service
