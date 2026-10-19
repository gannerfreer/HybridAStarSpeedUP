#pragma once

// W5 规划服务：由 HTTP 层、作业管理器和规划流水线共享的公开值类型。
//
// 线上契约（端点、响应信封、错误码）记录在 Src/service/README.md 中。
// 请求和结果对每种规划器类型（Pose2d、PathSegment、TrajectoryPoint、
// Corridor）使用已冻结的 planner::common JSON 编码；本头文件仅增加服务级
// 请求信封、算法参数和作业状态机。
//
// Dependency note (GPL isolation): the service is the only consumer of the
// isolated experimental planner::dftpav backend (GPL-3.0 port, see
// Src/dftpav/PROVENANCE.md). The Dftpav library stays an experimental backend
// behind the pipeline; no upstream type appears in this public surface.

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "planner/common/types.hpp"
#include "planner/corridor/corridor_generator.hpp"
#include "planner/dftpav/optimizer.hpp"

namespace planner::service {

// ---------------------------------------------------------------------------
// 公共响应信封（Docs/component_development_plan.md 第 6.1 节）。
// ---------------------------------------------------------------------------

inline nlohmann::json envelope(const std::string& type) {
  return nlohmann::json{{"schema_version", common::kSchemaVersion}, {"type", type}};
}

// ---------------------------------------------------------------------------
// 请求障碍物（web_scene schema，W3/W5 对齐）。
// ---------------------------------------------------------------------------

// 一个测试障碍物。矩形被归一化，使 x_min_m < x_max_m 且 y_min_m < y_max_m；
// 多边形保留请求中的顶点顺序，并在规划开始前由碰撞组件验证为凸。
struct ServiceObstacle {
  enum class Kind {
    kRectangle,
    kConvexPolygon,
  };

  Kind kind{Kind::kRectangle};
  std::string id;
  // kind == kRectangle 时适用。
  double x_min_m{0.0};
  double y_min_m{0.0};
  double x_max_m{0.0};
  double y_max_m{0.0};
  // kind == kConvexPolygon 时适用。
  std::vector<common::Point2d> vertices;

  // 转换为正式规划器组件的类型。调用这些函数前，障碍物必须已经过验证
  // （parse_plan_request）。
  collision::Obstacle to_collision() const;
  corridor::Obstacle to_corridor() const;

  // 轴对齐包围盒（矩形和多边形）。
  void bounds(double* min_x, double* min_y, double* max_x, double* max_y) const;
};

// ---------------------------------------------------------------------------
// 算法参数（POST /plan/submit 的 algorithm_params）。
// ---------------------------------------------------------------------------

// 流水线参数及其第 7.2 节默认值：Hybrid A* 5 s / 1,000,000 个节点，
// Dftpav 10 s。`dftpav_limits` 为空表示“从请求车辆推导曲率限制”（其余量值
// 由优化器自身默认值覆盖）；有值时原样使用（解析器用车辆推导的基准值
// 加上请求覆盖项填充它）。
struct PipelineParams {
  std::string hybrid_backend{"kth_normalized"};
  // 额外的后端 configure() 键，原样透传。
  nlohmann::json hybrid_config = nlohmann::json::object();
  double hybrid_time_budget_s{5.0};
  std::int64_t hybrid_node_budget{1000000};
  double dftpav_time_budget_s{10.0};
  corridor::CorridorOptions corridor;
  std::optional<dftpav::OptimizeLimits> dftpav_limits;
  dftpav::OptimizeOptions dftpav_options;
};

// 一个完全解析的 POST /plan/submit 请求。
struct ServicePlanRequest {
  std::string request_id;
  std::string map_id;
  std::string map_version;
  common::VehicleModel vehicle;
  common::Pose2d start;
  common::Pose2d goal;
  std::vector<ServiceObstacle> obstacles;
  PipelineParams params;
};

// HTTP 处理结果：状态码加信封体。`raw_body` 非空时按原样发送（用于
// GET /plan/scene/{scene_id} 的逐字节快照），否则发送 `body.dump()`。
struct HttpResponse {
  int status{200};
  nlohmann::json body;
  std::string raw_body;
};

// ---------------------------------------------------------------------------
// 作业状态机。
// ---------------------------------------------------------------------------

// 一个规划作业的生命周期。终态为 kSucceeded、kFailed 和 kCancelled。
enum class JobState {
  kQueued,
  kRunning,
  kSucceeded,
  kFailed,
  kCancelled,
};

// 规范的小写名称："queued"、"running"、"succeeded"、"failed"、"cancelled"。
const char* to_string(JobState state) noexcept;
bool is_terminal(JobState state) noexcept;

// GET /plan/status 报告的流水线阶段："map"、"hybrid"、"corridor"、
// "optimize"、"validate"、"done"。
namespace stage {
inline constexpr const char* kMap = "map";
inline constexpr const char* kHybrid = "hybrid";
inline constexpr const char* kCorridor = "corridor";
inline constexpr const char* kOptimize = "optimize";
inline constexpr const char* kValidate = "validate";
inline constexpr const char* kDone = "done";
}  // namespace stage

// Docs/component_development_plan.md 第 7.1 节的默认测试车辆
// （与 tests/fixtures/v3/vehicles/default.json 相同）。
common::VehicleModel default_vehicle();

}  // namespace planner::service
