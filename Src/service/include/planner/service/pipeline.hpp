#pragma once

// W5 规划流水线：Hybrid A* -> 安全走廊 -> Dftpav 优化 ->
// 独立的 planner::validation 门禁。
//
// 一次调用为一个已解析的请求和一个已构建的场景运行完整链路。流水线是同步
// 且单线程的；作业管理器在其工作线程上运行它。阶段切换通过 `on_stage`
// 报告，协作式取消在阶段之间、Hybrid A* 搜索内部（取消回调）以及 Dftpav
// 求解的 L-BFGS 迭代边界（`OptimizeOptions::cancel_check`，见 Src/dftpav）
// 轮询；Dftpav 阶段的时间预算仍在阶段返回后判定（不中断求解）。
//
// 仅当独立重跑的 planner::validation 门禁接受优化后的轨迹时，结果才报告为
// kSuccess；每次失败都带有稳定的原因码和人类可读的 detail。

#include <functional>
#include <string>

#include <nlohmann/json.hpp>

#include "planner/collision/scene.hpp"
#include "planner/common/types.hpp"
#include "planner/corridor/corridor_generator.hpp"
#include "planner/dftpav/optimizer.hpp"
#include "planner/service/types.hpp"
#include "planner/validation/validation.hpp"

namespace planner::service {

struct PipelineResult {
  common::Status status{common::Status::kInvalidInput};
  // 稳定的原因码，成功时为空。示例："invalid_input"、"no_path"、"timeout"、
  // "cancelled"、"corridor_blocked"、"corridor_narrow"、
  // "corridor_no_corridor"、Dftpav 优化器的错误码（"solver_failed"、
  // "validation_collision" 等）以及服务级验证错误码
  // （"validation_<constraint>"）。
  std::string reason;
  std::string detail;
  // 到达的最后一个流水线阶段："hybrid"、"corridor"、"optimize"、"validate"
  // （"map" 阶段由调用方负责）。
  std::string stage{stage::kHybrid};
  std::string hybrid_backend;
  // 阶段到达标志（从未进入的阶段保留其默认结果；统计信息和 HTTP 结果
  // 只报告已到达的阶段）。
  bool hybrid_ran{false};
  bool corridor_ran{false};
  bool optimize_ran{false};
  bool validation_ran{false};

  common::PlanResult hybrid;
  corridor::CorridorResult corridor;
  dftpav::OptimizeResult optimize;
  validation::ValidationResult validation;

  nlohmann::json statistics = nlohmann::json::object();
  double hybrid_time_s{0.0};
  double corridor_time_s{0.0};
  double optimize_time_s{0.0};
  double validation_time_s{0.0};

  bool ok() const noexcept { return status == common::Status::kSuccess; }
};

// 阶段观察者；在每个阶段开始时以阶段名调用。
using StageCallback = std::function<void(const std::string& stage)>;
// 协作式取消轮询；为空表示“永不取消”。
using CancelCallback = std::function<bool()>;

// 由请求车辆推导的 Dftpav 限制（与 Dftpav 端到端链路的规则相同：曲率上界
// tan(max_steer) / wheelbase，速度和加速度取优化器默认值）。
dftpav::OptimizeLimits limits_for_vehicle(const common::VehicleModel& vehicle);

// 运行流水线。`grid`/`scene` 必须由 build_scene 针对同一请求和地图生成。
PipelineResult run_pipeline(const ServicePlanRequest& request,
                            const common::GridView& grid,
                            const collision::Scene& scene,
                            const StageCallback& on_stage = {},
                            const CancelCallback& cancelled = {});

}  // namespace planner::service
