/**
 * @file reeds_shepp.hpp
 * @brief Reeds-Shepp (RS) 解析最短路径求解器、曲线分段及离散点采样工具
 * @details 实现了针对具有最小转弯半径约束、且具备前进与倒车双向行驶能力的汽车类车辆的经典 48 类
 *          Reeds-Shepp 最短路径解析求解模型
 *
 * 几何与运动学规范约定：
 * - 位姿 Pose2d 表示车辆后轴中心在局部右手坐标系下的状态（x 朝前，y 朝左，逆时针航向角为正）
 * - 路径曲率定义为 \f$ \kappa = \tan(\delta) / L \f$，满足运动学方程 \f$ \dot{\psi} = v_{\text{signed}} \cdot \kappa \f$
 *   曲率符号仅由前轮物理偏角决定（左转为正，右转为负），独立于前进或倒车挡位
 * - 生成的所有分段均严格满足曲率硬约束 \f$ |\kappa| \le 1 / R_{\min} \f$
 */

#pragma once

#include <string>
#include <vector>

#include "planner/common/path.hpp"
#include "planner/common/status.hpp"
#include "planner/common/vehicle.hpp"

namespace planner::rs {

/**
 * @name 终点数值闭合容差
 * @{
 */
inline constexpr double kEndpointPositionToleranceM = 1e-6;  ///< 终点位置欧氏距离容差（米）
inline constexpr double kEndpointYawToleranceRad = 1e-6;  ///< 终点航向角闭合容差（弧度）
/// @}

/**
 * @brief Reeds-Shepp 路径的单一常曲率运动分段（直线段或固定半径圆弧段）
 */
struct RsSegment {
  common::Direction direction{common::Direction::kForward};  ///< 行驶挡位方向（前进或倒车）
  double length_m{0.0};  ///< 本段沿曲线行进的弧长（米，非负）
  double curvature_per_m{
      0.0};  ///< 有符号曲率 \f$ \tan(\delta)/L \f$（1/m，直线为 0，圆弧为 \f$\pm 1/R_{\min}\f$）
};

/**
 * @brief 两车辆位姿之间的最短 Reeds-Shepp 连续路径
 * @details 起终点重合的零距离查询对应的 `segments` 数组为空
 */
struct RsPath {
  common::Pose2d start;             ///< 规划起始位姿回显（已归一化）
  common::Pose2d goal;              ///< 规划目标位姿回显（已归一化）
  double length_m{0.0};             ///< 路径总弧长（米，即所有分段弧长之和）
  std::vector<RsSegment> segments;  ///< 从起点到终点有序连接的常曲率运动分段序列
};

/**
 * @brief Reeds-Shepp 路径规划的非异常返回结果载荷
 */
struct RsPlan {
  common::Status status{
      common::Status::
          kInvalidInput};  ///< 求解状态码（kSuccess, kInvalidInput 或 kOptimizationFailed）
  std::string reason;      ///< 失败时的详细诊断原因说明
  RsPath path;  ///< 最优最短路径（仅在 status == Status::kSuccess 时有效）
};

/**
 * @brief 校验 Reeds-Shepp 规划查询输入的有效性
 * @param[in] start 起始位姿
 * @param[in] goal 目标位姿
 * @param[in] vehicle 车辆运动学模型参数
 * @return 若位姿数值有限、轴距大于零、前轮最大转角在 (0, pi/2) 范围内且最小转弯半径有效则返回 true
 */
bool is_valid_query(const common::Pose2d& start, const common::Pose2d& goal,
                    const common::VehicleModel& vehicle) noexcept;

/**
 * @brief 根据轴距和最大转向角计算车辆最小转弯半径
 * @param[in] vehicle 车辆运动学模型
 * @return 最小转弯半径（米）：wheelbase / tan(max_steer)
 * @throw std::invalid_argument 若车辆参数非法或无法计算转弯半径
 */
double min_turn_radius_m(const common::VehicleModel& vehicle);

/**
 * @brief 求解两车辆位姿之间的最短 Reeds-Shepp 路径（非抛异常安全接口）
 * @param[in] start 车辆后轴中心起始位姿
 * @param[in] goal 车辆后轴中心目标位姿
 * @param[in] vehicle 车辆运动学模型
 * @return 包含执行状态、诊断文本及最优路径的 RsPlan 结构
 */
[[nodiscard]] RsPlan plan(const common::Pose2d& start, const common::Pose2d& goal,
                          const common::VehicleModel& vehicle);

/**
 * @brief 求解两车辆位姿之间的最短 Reeds-Shepp 距离（抛出异常的包装接口）
 * @param[in] start 起始位姿
 * @param[in] goal 目标位姿
 * @param[in] vehicle 车辆运动学模型
 * @return 最短路径长度（米）
 * @throw std::invalid_argument 若输入参数非法
 * @throw std::runtime_error 若路径求解或几何校验失败
 */
[[nodiscard]] double distance(const common::Pose2d& start, const common::Pose2d& goal,
                              const common::VehicleModel& vehicle);

/**
 * @brief 求解两车辆位姿之间的最短 Reeds-Shepp 路径（抛出异常的包装接口）
 * @param[in] start 起始位姿
 * @param[in] goal 目标位姿
 * @param[in] vehicle 车辆运动学模型
 * @return 求解得到的最短 Reeds-Shepp 路径对象
 * @throw std::invalid_argument 若输入参数非法
 * @throw std::runtime_error 若路径求解或几何校验失败
 */
RsPath path(const common::Pose2d& start, const common::Pose2d& goal,
            const common::VehicleModel& vehicle);

/**
 * @brief 沿常曲率运动分段将位姿推进指定的弧长距离 ds_m
 * @param[in] start 分段起点位姿
 * @param[in] segment 描述该段曲率与行驶方向的运动分段
 * @param[in] ds_m 前进距离（米，满足 0 <= ds_m <= segment.length_m）
 * @return 推进后的位姿
 * @throw std::invalid_argument 若 ds_m 为负或超过本段总长
 */
common::Pose2d integrate_segment(const common::Pose2d& start, const RsSegment& segment,
                                 double ds_m);

/**
 * @brief 沿 Reeds-Shepp 路径以近似均匀的弧长步长进行离散采样
 * @details 保证满足以下几何性质：
 *          - 第一个采样点精确为 `path.start`，其累计弧长 s_m = 0；
 *          - 最后一个采样点精确为 `path.goal`，其累计弧长 s_m = path.length_m；
 *          - 所有分段分界点（包含换向尖点 cusp）均被精确保留为采样点；
 *          - 相邻采样点之间的弧长间隔不超过 `ds_m`
 * @param[in] path 待采样的 Reeds-Shepp 路径
 * @param[in] ds_m 期望的标称采样步长（米，> 0）
 * @return 采样生成的 PathPoint 点集序列
 * @throw std::invalid_argument 若 ds_m <= 0、非有限或产生点数超出安全上限
 */
std::vector<common::PathPoint> sample(const RsPath& path, double ds_m);

/**
 * @brief 计算路径上指定累计弧长 `s_m` 处的单点精确航迹位姿
 * @param[in] path 目标 Reeds-Shepp 路径
 * @param[in] s_m 期望查询的目标弧长（米，取值范围 [0, path.length_m]）
 * @return 对应弧长处的航迹点（包含位姿、曲率与方向）
 * @throw std::out_of_range 若 s_m 超出 [0, path.length_m] 范围
 * @throw std::invalid_argument 若 s_m 为非有限数或路径分段异常
 */
common::PathPoint sample_at_s(const RsPath& path, double s_m);

}  // namespace planner::rs
