/**
 * @file collision_checker.hpp
 * @brief 车辆单点位姿、圆弧运动分段及离散路径的连续与离散碰撞检测器
 * @details 核心碰撞判定不变式：
 *          - 闭集几何假设：几何边界接触或相切在判定中均作为碰撞处理
 *          - 超出地图边界的外部区域以及未知栅格单元 (kCellUnknown) 均保守视作障碍物占用
 *          - 连续运动段采用基于自适应二分与保守位移包络的扫掠运动检测细化至极限扫掠精度
 *            `sweep_precision_m`（默认 1e-3 m）仍无法证明安全的不确定区间严格记为碰撞
 *          - 多点路径检测通过根据曲率、间距与挡位在相邻采样点间重构常曲率圆弧分段进行连续扫掠检测
 */

#pragma once

#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "planner/common/path.hpp"
#include "scene.hpp"

namespace planner::collision {

using planner::common::Direction;
using planner::common::PathSegment;

/**
 * @brief 碰撞检测查询的顶层执行状态
 */
enum class CheckStatus {
  kClear,        ///< 安全，待测位姿或运动轨迹全域无碰撞
  kCollision,    ///< 发生碰撞或边界接触（具体原因见 reason_code 与 detail）
  kInvalidInput  ///< 输入参数非法，未执行几何检测
};

/**
 * @brief 触发碰撞判定或请求拒绝的具体诱因分类
 */
enum class CollisionReason {
  kNone,          ///< 无碰撞
  kObstacle,      ///< 与凸多边形障碍物相交或接触
  kGridOccupied,  ///< 与占用栅格单元 (kCellOccupied) 重叠
  kGridUnknown,   ///< 与未知栅格单元 (kCellUnknown) 重叠（保守视作障碍物）
  kOutsideMap,    ///< 车体足迹部分或全部超出栅格地图矩形边界
  kPrecisionFloor,  ///< 细化区间达到极限扫掠精度下限仍未收敛（保守记为碰撞）
  kInvalidInput  ///< 输入参数包含非有限数或结构异常
};

/**
 * @brief 将 CheckStatus 状态码转换为易读的字符串
 * @param[in] status 检测状态枚举值
 * @return 对应的字符串；未知枚举值返回 "unknown"
 */
[[nodiscard]] std::string_view to_string(CheckStatus status) noexcept;

/**
 * @brief 将 CollisionReason 原因枚举转换为易读的字符串
 * @param[in] reason 碰撞原因枚举值
 * @return 对应的字符串；未知枚举值返回 "unknown"
 */
[[nodiscard]] std::string_view to_string(CollisionReason reason) noexcept;

/**
 * @brief 单次碰撞检测查询的完整结果载荷
 */
struct CheckResult {
  CheckStatus status{
      CheckStatus::kClear};  ///< 综合检测状态（kClear, kCollision 或 kInvalidInput）
  CollisionReason reason_code{CollisionReason::kNone};  ///< 触发碰撞的具体诱因
  std::string detail;     ///< 人类可读的诊断原因与细节描述
  Pose2d first_pose;      ///< 检测到首次发生接触/碰撞处的后轴中心位姿
  double first_s_m{0.0};  ///< 沿运动段或路径到首次碰撞点处的累计弧长（米）
  int path_segment_index{-1};  ///< 发生碰撞的路径分段索引（仅在多段路径检测时有效）

  /**
   * @brief 判定是否发生碰撞
   * @return 若 status == CheckStatus::kCollision 返回 true
   */
  bool collided() const noexcept { return status == CheckStatus::kCollision; }

  /**
   * @brief 判定是否完全安全无碰撞
   * @return 若 status == CheckStatus::kClear 返回 true
   */
  bool is_clear() const noexcept { return status == CheckStatus::kClear; }

  /**
   * @brief 判定输入是否合法并实际完成了碰撞检测
   * @return 若 status != CheckStatus::kInvalidInput 返回 true
   */
  bool is_valid() const noexcept { return status != CheckStatus::kInvalidInput; }
};

/**
 * @brief 碰撞检测精度控制与空间索引加速选项配置
 */
struct CheckOptions {
  /**
   * @brief 自适应二分判定的极限扫掠精度下限（米，默认 1e-3 m）
   * @details 单个叶子扫掠区间内车体刚体上任意点的最大物理位移上限
   */
  double sweep_precision_m{1e-3};

  /**
   * @brief 是否启用空间包围盒预排序与网格分块加速索引
   * @details 禁用时直接执行纯几何暴力筛选
   */
  bool enable_acceleration{false};
};

/**
 * @brief 单段常曲率圆弧或直线运动基元分段
 * @details 由起点位姿、行进弧长、行驶挡位及有符号曲率定义
 *          航向角增量满足 \f$ \Delta\psi = \kappa \cdot s_{\text{signed}} \f$
 */
struct MotionSegment {
  Pose2d start;                              ///< 车辆后轴中心起始位姿
  double length_m{0.0};                      ///< 沿曲线行进的弧长（米，非负）
  Direction direction{Direction::kForward};  ///< 行驶挡位（前进或倒车）
  double curvature_per_m{0.0};  ///< 有符号路径曲率 \f$ \kappa = \tan(\delta)/L \f$（1/m）
};

/**
 * @brief 检测车辆在指定单点位姿下的静态碰撞状态
 * @param[in] pose 待测车辆后轴中心位姿
 * @param[in] vehicle 车辆几何模型
 * @param[in] scene 场景环境（栅格与多边形障碍物）
 * @param[in] options 碰撞检测选项
 * @return 包含无碰撞或首个接触原因的结果体
 */
[[nodiscard]] CheckResult check_pose(const Pose2d& pose, const VehicleModel& vehicle,
                                     const Scene& scene, const CheckOptions& options = {});

/**
 * @brief 检测沿单一常曲率运动分段的连续扫掠运动碰撞
 * @param[in] segment 常曲率运动分段
 * @param[in] vehicle 车辆几何模型
 * @param[in] scene 场景环境
 * @param[in] options 碰撞检测选项
 * @return 包含无碰撞或首次接触位姿与弧长的检测结果
 */
[[nodiscard]] CheckResult check_motion(const MotionSegment& segment, const VehicleModel& vehicle,
                                       const Scene& scene, const CheckOptions& options = {});

/**
 * @brief 检测沿单一连续同向路径分段的连续运动碰撞
 * @param[in] segment 包含若干采样点的路径分段
 * @param[in] vehicle 车辆几何模型
 * @param[in] scene 场景环境
 * @param[in] options 碰撞检测选项
 * @return 检测结果
 */
[[nodiscard]] CheckResult check_path(const PathSegment& segment, const VehicleModel& vehicle,
                                     const Scene& scene, const CheckOptions& options = {});

/**
 * @brief 检测包含多个分段的完整换向规划路径的连续运动碰撞
 * @param[in] path 包含多段路径的轨迹序列
 * @param[in] vehicle 车辆几何模型
 * @param[in] scene 场景环境
 * @param[in] options 碰撞检测选项
 * @return 检测结果，包含首次碰撞所在的分段索引
 */
[[nodiscard]] CheckResult check_path(const std::vector<PathSegment>& path,
                                     const VehicleModel& vehicle, const Scene& scene,
                                     const CheckOptions& options = {});

struct CollisionCheckerImpl;

/**
 * @brief 可复用的碰撞检测器对象，预先缓存车体足迹与可选的场景空间加速索引
 */
class CollisionChecker {
 public:
  /**
   * @brief 构造并完整校验可复用的碰撞检测器实例
   * @param[in] vehicle 车辆几何模型
   * @param[in] scene 目标场景快照
   * @param[in] options 碰撞检测配置选项
   */
  CollisionChecker(const VehicleModel& vehicle, const Scene& scene, CheckOptions options = {});

  /**
   * @brief 检查检测器初始化与输入参数是否合法有效
   * @return 若参数有效返回 true，若被拒绝返回 false
   */
  bool valid() const noexcept;

  /**
   * @brief 获取初始化失败时的诊断错误原因
   * @return 错误原因文本字符串
   */
  const std::string& invalid_reason() const noexcept;

  const VehicleModel& vehicle() const noexcept;
  const Scene& scene() const noexcept;
  const CheckOptions& options() const noexcept;

  [[nodiscard]] CheckResult check_pose(const Pose2d& pose) const;
  [[nodiscard]] CheckResult check_motion(const MotionSegment& segment) const;
  [[nodiscard]] CheckResult check_path(const PathSegment& segment) const;
  [[nodiscard]] CheckResult check_path(const std::vector<PathSegment>& path) const;

 private:
  std::shared_ptr<const CollisionCheckerImpl> state_;
};

}  // namespace planner::collision
