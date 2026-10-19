#pragma once

// 统一的 Hybrid A* 适配器契约（Docs/component_development_plan.md §6.3）。
//
// 所有后端（上游复现与 ROS-free 归一化后端）都实现同样的三个操作：
//   * configure(config)  - 冻结参数，报告不支持的键；
//   * plan(request, grid, collision_checker, rs_provider) - 执行一次搜索；
//   * reset_cache()      - 清除所有依赖地图/目标/车辆的缓存。
//
// 该接口承载的规则：
//   * 上游/第三方类型不会出现在公开头文件中；每个适配器将其保留在内部；
//   * `native` 后端（上游复现，仅供参考）与 `normalized` 后端（统一契约，
//     可参与选择）由 BackendKind 区分并写入统计信息；
//   * 后端无法映射的参数会以 `unsupported` 记录在 ConfigReport 中，绝不
//     静默忽略：只要设置了不支持的参数，plan() 就会以 invalid_input 失败
//     并指出该参数；
//   * plan() 内的扩展循环必须检查时间预算、节点预算与取消
//    （request.cancelled 加上可选的 set_cancel_callback 钩子）；
//   * 后端无法测量的统计项保持缺省，并在结果的 `unsupported_statistics`
//     中附原因列出（绝不用零填充）。

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "planner/collision/collision_checker.hpp"
#include "planner/common/types.hpp"
#include "planner/hybrid_astar/rs_provider.hpp"

namespace planner::hybrid_astar {

// 实验标签。kNative 结果使用上游的搜索/检查，仅供参考；只有 kNormalized
// 后端在通过契约测试后可参与默认后端选择。
enum class BackendKind {
  kNative,
  kNormalized,
};

const char* to_string(BackendKind kind) noexcept;

// 一个配置键以及后端是否能将其映射到冻结的参数集合。`note` 说明不支持
// 的键，或记录取值范围/参数含义。
struct ParameterSupport {
  std::string name;
  bool supported{false};
  std::string note;
};

// configure() 的结果。`accepted=false` 表示整个配置被拒绝（JSON 类型错误、
// 取值越界），plan() 不得运行。`unsupported` 列出该后端无法映射的显式
// 设置的键；configure 本身仍会为诊断报告它们，但该列表非空时 plan()
// 拒绝运行。
struct ConfigReport {
  bool accepted{false};
  std::string reason;
  nlohmann::json effective_config = nlohmann::json::object();
  std::vector<ParameterSupport> parameters;
  std::vector<std::string> unsupported;
};

// 一个后端的注册表描述。
struct BackendDescriptor {
  std::string id;
  BackendKind kind{BackendKind::kNormalized};
  std::string description;
};

// 完整的后端描述，包括对比工具使用的逐参数与逐统计项支持表。
struct BackendInfo {
  std::string id;
  BackendKind kind{BackendKind::kNormalized};
  std::string description;
  std::vector<ParameterSupport> parameters;
  std::vector<ParameterSupport> statistics;
};

class HybridAStarBackend {
 public:
  virtual ~HybridAStarBackend() = default;

  virtual std::string id() const = 0;
  virtual BackendKind kind() const noexcept = 0;
  virtual BackendInfo info() const = 0;

  // 冻结配置。接受/拒绝与不支持的语义见 ConfigReport。调用 configure()
  // 会清除所有缓存。
  virtual ConfigReport configure(const nlohmann::json& config) = 0;

  // 实际生效的配置（默认值 + 被接受的覆盖项）。
  virtual const nlohmann::json& effective_config() const = 0;

  // 一次规划调用。`grid` 是地图快照，`collision_checker` 必须由同一车辆和
  // 地图构建，`rs_provider` 提供 RS 距离/解析连接。三者均为借用，必须在
  // 调用期间保持存活；规划器运行期间不得修改输入快照。
  virtual common::PlanResult plan(const common::PlanRequest& request,
                                  const common::GridView& grid,
                                  const collision::CollisionChecker& collision_checker,
                                  RsProvider& rs_provider) = 0;

  // 清除所有以地图、目标或车辆为键的缓存。热启动复用仅在同一个重置周期
  // 内有效（所有后端规则相同）。
  virtual void reset_cache() = 0;

  // 可选的协作式取消钩子，除 PlanRequest::cancelled 外，每个扩展循环内部
  // 也会检查。空回调表示禁用。
  virtual void set_cancel_callback(std::function<bool()> callback) = 0;
};

// 本次构建已知的后端。只列出完整实现的后端；未来的 Smac / KTH / zm0612
// 适配器将在实现后加入这里。
std::vector<BackendDescriptor> list_backends();

// 按 id 创建后端；未知 id 返回 nullptr（绝不静默替换为其他后端）。
std::unique_ptr<HybridAStarBackend> create_backend(const std::string& id);

}  // namespace planner::hybrid_astar
