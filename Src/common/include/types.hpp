/**
 * @file types.hpp
 * @brief 运动规划通用基础数据结构与几何基元类型定义。
 * @details 统一物理量单位规范：距离采用米 (m)，时间采用秒 (s)，角度采用弧度 (rad)。
 *          局部笛卡尔坐标系统一采用右手系（x 轴朝前，y 轴朝左，逆时针航向角为正）。
 */

#pragma once

#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "planner/math/angle.hpp"
#include "planner/math/kinematics/bicycle_model.hpp"
#include "planner/math/pose2d.hpp"
#include "planner/math/vec2d.hpp"

namespace planner::common {

using math::kPi;
using math::Point2d;
using math::Pose2d;

/**
 * @name 栅格单元占用状态
 * @brief GridView::cells 中以行主序存储的单元占用状态整数值。
 * @{
 */
inline constexpr int kCellFree = 0;      ///< 自由空闲栅格，无障碍物且可安全通行。
inline constexpr int kCellOccupied = 1;  ///< 被障碍物占用的栅格。
inline constexpr int kCellUnknown = 2;   ///< 未知栅格（碰撞检测与规划保守视作占用）。
/// @}

/**
 * @brief 路径点或运动轨迹分段的行驶方向（前进/倒车）。
 */
enum class Direction {
  kForward,  ///< 前进挡位。
  kReverse,  ///< 倒车挡位。
};

/**
 * @brief 规划器与几何求解模块的执行状态码。
 */
enum class Status {
  kSuccess,             ///< 求解成功，已找到满足要求的轨迹。
  kInvalidInput,        ///< 输入参数无效或超出定义域。
  kNoPath,              ///< 搜索结束但未找到无碰撞的可行路径。
  kTimeout,             ///< 超过指定计算耗时预算，规划终止。
  kCancelled,           ///< 外部客户端请求或回调触发主动取消。
  kOptimizationFailed,  ///< 数值求解器或轨迹优化迭代收敛失败。
  kValidationFailed,    ///< 候选路径未通过搜索后的安全完整性校验。
};

/**
 * @brief 将行驶方向枚举转换为易读的字符串形式。
 * @param[in] direction 行驶方向枚举值。
 * @return 对应的字符串表示（"forward" 或 "reverse"）。
 * @throw std::invalid_argument 若传入未知枚举值。
 */
std::string to_string(Direction direction);

/**
 * @brief 将执行状态码转换为易读的字符串形式。
 * @param[in] status 状态码枚举值。
 * @return 对应的状态描述字符串。
 * @throw std::invalid_argument 若传入未知枚举值。
 */
std::string to_string(Status status);

/**
 * @brief 矩形包络车辆的几何尺寸与运动学模型参数。
 * @details 最小转弯半径由轴距与最大前轮转角推导得出。
 */
struct VehicleModel {
  std::string id;               ///< 车辆模型唯一标识。
  double wheelbase_m{0.0};       ///< 轴距（前后轴中心距，米）。
  double width_m{0.0};           ///< 车身总宽度（米）。
  double front_overhang_m{0.0};  ///< 前悬长度（前轴至前保险杠外缘距离，米）。
  double rear_overhang_m{0.0};   ///< 后悬长度（后轴至后保险杠外缘距离，米）。
  double max_steer_rad{0.0};     ///< 最大前轮转向角（弧度）。
  double safety_margin_m{0.0};   ///< 车体外围额外扩充的安全裕量（米）。

  /**
   * @brief 计算车辆后轴中心处的最小转弯半径。
   * @return 后轴中心最小转弯半径（米）：wheelbase / tan(max_steer)。
   */
  double min_turn_radius_m() const {
    return math::kinematics::min_turn_radius_m(wheelbase_m, max_steer_rad);
  }
};

/**
 * @brief 二维栅格地图占用网格只读视图。
 * @details 栅格单元状态以行主序平铺存储：0 = Free，1 = Occupied，2 = Unknown。
 *          超出地图边界的区域在规划与碰撞检测中均保守视作障碍物。
 *          `origin` 为栅格原点（对应第 0 行第 0 列单元的左下角）在局部坐标系中的位姿。
 *          单元格 (r, c) 的中心坐标为：
 *          \f$ \text{origin} + ((c + 0.5) \cdot \text{res}, (r + 0.5) \cdot \text{res}) \f$。
 */
struct GridView {
  std::string map_id;       ///< 地图唯一标识符。
  std::string map_version;  ///< 地图版本字符串，用于校验快照一致性。
  int width{0};             ///< 栅格列数（沿局部 X 轴方向）。
  int height{0};            ///< 栅格行数（沿局部 Y 轴方向）。
  double resolution_m{0.0}; ///< 正方形栅格单元边长分辨率（米）。
  Pose2d origin;            ///< 栅格地图原点位姿（[0,0] 单元左下角顶点）。
  std::vector<int> cells;   ///< 行主序存储的栅格单元占用状态数组。

  /**
   * @brief 检查给定的行列索引是否位于网格有效边界内。
   * @param[in] row 目标栅格行索引。
   * @param[in] col 目标栅格列索引。
   * @return 若 0 <= row < height 且 0 <= col < width 则返回 true，否则返回 false。
   */
  bool in_bounds(int row, int col) const {
    return row >= 0 && row < height && col >= 0 && col < width;
  }

  /**
   * @brief 查询指定 (row, col) 处的单元格占用状态。
   * @param[in] row 目标栅格行索引。
   * @param[in] col 目标栅格列索引。
   * @return 对应的占用状态整数值。
   * @throw std::out_of_range 若索引超出栅格边界。
   */
  int cell(int row, int col) const;
};

/**
 * @brief 车辆轨迹路径采样点。
 * @details `s_m` 为从路径起点算起的非负累计弧长（米）。
 *          `curvature_inv_m` 为有符号曲率 \f$ \kappa = \tan(\delta) / L \f$，
 *          满足运动学关系 \f$ \dot{\psi} = v_{\text{signed}} \cdot \kappa \f$。
 */
struct PathPoint {
  Pose2d pose;                  ///< 采样点处的车辆后轴中心位姿。
  double s_m{0.0};              ///< 沿路径的累计弧长（米）。
  double curvature_inv_m{0.0};  ///< 有符号路径曲率（1/m，左转为正，右转为负）。
  Direction direction{Direction::kForward};  ///< 该点对应的行驶方向（前进/倒车）。
};

/**
 * @brief 具有统一行驶挡位的连续路径段。
 * @details 挡位切换（折返尖点 cusp）处必定分割并启动新的路径段。段内累计弧长 `s_m` 严格单调递增。
 */
struct PathSegment {
  std::vector<PathPoint> points;  ///< 本段内有序排列的路径采样点序列。
};

/**
 * @brief 路径规划任务输入请求载荷。
 * @details 包含起终点位姿、车辆模型以及可选的计算资源配额。
 *          若未设置预算，则回退使用规划器配置的默认预算。
 */
struct PlanRequest {
  std::string request_id;                 ///< 本次规划任务的唯一追踪 ID。
  std::string map_id;                     ///< 目标地图 ID。
  std::string map_version;                ///< 期望匹配的地图版本号。
  VehicleModel vehicle;                   ///< 车辆几何与运动学模型。
  Pose2d start;                           ///< 规划起始位姿（后轴中心）。
  Pose2d goal;                            ///< 规划目标位姿（后轴中心）。
  std::optional<double> time_budget_s;    ///< 可选的最大计算耗时预算（秒）。
  std::optional<std::int64_t> node_budget;///< 可选的最大搜索展开节点数预算。
  bool cancelled{false};                  ///< 取消标志位（true 表示放弃执行本次规划）。
};

/**
 * @brief 路径规划执行结果输出载荷。
 * @details 仅当 `status == Status::kSuccess` 时，`path` 中的轨迹数据才有效且可供执行。
 */
struct PlanResult {
  std::string request_id;         ///< 对应的规划请求任务 ID。
  Status status{Status::kInvalidInput};  ///< 规划执行最终状态码。
  std::string reason;             ///< 结果诊断描述或失败原因文本。
  std::vector<PathSegment> path;  ///< 规划生成的完整路径（按行驶方向分段）。
};

/**
 * @name 精确全等比较操作符
 * @brief 规划核心数据结构的值相等性判断（用于缓存校验、回放对比与结果校验）。
 * @{
 */
bool operator==(const VehicleModel& lhs, const VehicleModel& rhs);
bool operator==(const GridView& lhs, const GridView& rhs);
bool operator==(const PathPoint& lhs, const PathPoint& rhs);
bool operator==(const PathSegment& lhs, const PathSegment& rhs);
/// @}

}  // namespace planner::common
