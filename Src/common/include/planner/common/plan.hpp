/**
 * @file plan.hpp
 * @brief 路径规划任务输入请求与执行结果输出载荷定义。
 */

#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "planner/common/path.hpp"
#include "planner/common/status.hpp"
#include "planner/common/vehicle.hpp"

namespace planner::common {

/**
 * @brief 路径规划任务输入请求载荷。
 * @details 包含起终点位姿、车辆模型以及可选的计算资源配额。
 *          若未设置预算，则回退使用规划器配置的默认预算。
 */
struct PlanRequest {
  std::string request_id;                   ///< 本次规划任务的唯一追踪 ID。
  std::string map_id;                       ///< 目标地图 ID。
  std::string map_version;                  ///< 期望匹配的地图版本号。
  VehicleModel vehicle;                     ///< 车辆几何与运动学模型。
  Pose2d start;                             ///< 规划起始位姿（后轴中心）。
  Pose2d goal;                              ///< 规划目标位姿（后轴中心）。
  std::optional<double> time_budget_s;      ///< 可选的最大计算耗时预算（秒）。
  std::optional<std::int64_t> node_budget;  ///< 可选的最大搜索展开节点数预算。
  bool cancelled{false};  ///< 取消标志位（true 表示放弃执行本次规划）。
};

/**
 * @brief 路径规划执行结果输出载荷。
 * @details 仅当 `status == Status::kSuccess` 时，`path` 中的轨迹数据才有效且可供执行。
 */
struct PlanResult {
  std::string request_id;                ///< 对应的规划请求任务 ID。
  Status status{Status::kInvalidInput};  ///< 规划执行最终状态码。
  std::string reason;                    ///< 结果诊断描述或失败原因文本。
  std::vector<PathSegment> path;  ///< 规划生成的完整路径（按行驶方向分段）。
};

}  // namespace planner::common
