#pragma once

// 首版规划器的 Reeds-Shepp (RS) 距离、路径分段与采样
// （Docs/component_development_plan.md 3.7、6.1、6.2 和 7.1 节）。
//
// 公开 API 规则：
//   * 不依赖 ROS、Lanelet2 和浏览器；仅使用 planner::common 类型。
//   * 此处绝不暴露任何 OMPL 类型。OMPL 1.6.0 仅用作封装一致性测试的
//     参照（见 src/ompl_reference.* 和 CMakeLists 中的组件 README 说明）；
//     交付的求解器是原生解析式 48 曲线枚举。
//   * 角度在输入回显与输出时归一化到 [-pi, pi)。
//
// 几何约定（冻结契约，tests/fixtures/v1/serialization）：
//   * Pose2d 表示后轴中心；x 向前、y 向左、yaw 逆时针为正。
//   * 路径是按行进顺序排列的常曲率分段序列。方向变化（尖点）
//     总是开启一个新分段。
//   * curvature_inv_m = tan(steer) / wheelbase，因此
//     yaw_rate = signed_speed * curvature_inv_m。符号描述物理转向方向
//     （正 = 左转），因而与挡位无关；前进/倒车由 direction 表示。
//   * 每个 plan() 结果都满足 |curvature_inv_m| <= 1 / min_turn_radius_m。

#include <string>
#include <vector>

#include "planner/common/types.hpp"

namespace planner::rs {

// 冻结的验收容差（Docs/component_development_plan.md 7.1）。
inline constexpr double kEndpointPositionToleranceM = 1e-6;
inline constexpr double kEndpointYawToleranceRad = 1e-6;

// 一个常曲率 RS 分段，按行进顺序排列。
//
//   * `length_m` 为行进的弧长，总是 >= 0。
//   * `direction` 为该分段使用的挡位。
//   * `curvature_inv_m` 为带符号的 tan(steer)/wheelbase，单位 1/m；直线段
//     为 0，圆弧为 +/-1/R，其中 R = min_turn_radius_m(vehicle)。
struct RsSegment {
  common::Direction direction{common::Direction::kForward};
  double length_m{0.0};
  double curvature_inv_m{0.0};
};

// 两个后轴位姿之间的最短 RS 路径。`start`/`goal` 回显（归一化后的）查询；
// `length_m` 等于各分段长度之和。零距离规划（start == goal）得到空的
// `segments` 向量。
struct RsPath {
  common::Pose2d start;
  common::Pose2d goal;
  double length_m{0.0};
  std::vector<RsSegment> segments;
};

// 不抛异常的求解结果。`status` 取值：
//   * kSuccess      - `path` 已填充并已对照 `goal` 验证；
//   * kInvalidInput - 位姿非有限或 VehicleModel 参数无效；
//   * kOptimizationFailed - 内部终点验证超出 1e-6 m / 1e-6 rad 容差
//     （绝不静默接受）。
struct RsPlan {
  common::Status status{common::Status::kInvalidInput};
  std::string reason;
  RsPath path;
};

// 简单的查询校验：位姿有限、wheelbase_m > 0、
// 0 < max_steer_rad < pi/2、最小转弯半径有限且为正。
bool is_valid_query(const common::Pose2d& start, const common::Pose2d& goal,
                    const common::VehicleModel& vehicle) noexcept;

// wheelbase_m / tan(max_steer_rad)；VehicleModel 无效时
// 抛出 std::invalid_argument。
double min_turn_radius_m(const common::VehicleModel& vehicle);

// 解析式 RS 求解。从不抛异常；无效输入通过 RsPlan::status（kInvalidInput）
// 连同可读原因报告。
RsPlan plan(const common::Pose2d& start, const common::Pose2d& goal,
            const common::VehicleModel& vehicle);

// plan() 的抛异常便捷包装：无效输入抛 std::invalid_argument，
// 未生成经验证的路径时抛 std::runtime_error。
double distance(const common::Pose2d& start, const common::Pose2d& goal,
                const common::VehicleModel& vehicle);
RsPath path(const common::Pose2d& start, const common::Pose2d& goal,
            const common::VehicleModel& vehicle);

// 沿 `segment` 将 `start` 移动 `ds_m`（0 <= ds_m <= segment.length_m）。
// ds_m 超出分段末端时以 std::invalid_argument 拒绝。
common::Pose2d integrate_segment(const common::Pose2d& start,
                                 const RsSegment& segment, double ds_m);

// 以近似弧长间隔 `ds_m` 对 `path` 采样：
//   * 首个点恰为 `path.start`，s_m = 0；
//   * 末个点恰为 `path.goal`，s_m = path.length_m；
//   * 包含每个分段边界（尖点）；边界点携带从该处开始的分段的方向/曲率
//     （末点携带最后一个分段的方向）；
//   * 边界之间相邻点的间隔至多为 ds_m。
// ds_m <= 0、非有限，或路径采样数会超出内部安全限制时，
// 抛出 std::invalid_argument。
std::vector<common::PathPoint> sample(const RsPath& path, double ds_m);

// 在累计弧长 `s_m`（属于 [0, path.length_m]）处的精确求值。
// 在尖点处，返回点遵循与 sample() 相同的“下一分段拥有边界”规则。
// 超出定义域时抛 std::out_of_range；s_m 非有限或路径畸形时抛
// std::invalid_argument。
common::PathPoint sample_at_s(const RsPath& path, double s_m);

}  // namespace planner::rs
