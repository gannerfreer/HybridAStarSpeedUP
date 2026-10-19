#pragma once

// planner_bspline：B-spline 曲线库（首版基线为 TinySpline 0.6.0 封装；
// Eigen Splines 为内部对照候选，EGO-Planner UniformBspline 为 GPL 隔离的
// 对照后端，见 BackendKind 与 README 的选型章节）。
//
// 契约（Docs/component_development_plan.md 第 6.1、6.2、7.1、7.3 节）：
//   * 2D 曲线由显式控制点列表与调用方提供的显式节点向量构造；
//     参数域由该节点向量推导（绝不隐式）。
//   * 统一接口：evaluate(parameter, derivative_order) 与批量
//     sample(parameters, output_buffer)，求导阶数 0..3。
//   * 超出参数域的参数会被拒绝，不存在隐式外推。在内部节点处使用右侧
//     分段；在参数域最后一点处使用最后一个分段。
//   * 区分几何节点参数与时间参数（ParameterKind）；时间参数化曲线
//     返回对时间的导数。
//   * 近似等弧长重采样，区间误差可度量（第 7.1 节容差 1e-3 m；
//     基于区间的重采样的最后一个余数区间单独报告）。
//
// 公开 API 绝不暴露 TinySpline 或 Eigen 类型与头文件；后端只以
// BackendKind 枚举出现。

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "planner/bspline/status.hpp"
#include "planner/common/types.hpp"

namespace planner::bspline {

namespace detail {
struct SplineImpl;
// 对照后端注册入口（实现由 GPL-3.0 隔离库 planner_bspline_ego 提供）。
// 主库不链接上游 EGO 代码；在创建 BackendKind::kEgoUniform 曲线之前调用
// 本函数（通常在测试/基准的 main 或测试套件初始化中），否则 create()
// 返回 std::nullopt 并说明后端未链接。须在使用前完成，非线程安全。
void install_ego_backend();
}  // namespace detail

// evaluate()/sample() 接受的曲线参数的含义。
enum class ParameterKind {
  // 几何曲线的节点参数（无量纲；纯几何使用方常用的参数化方式）。
  kGeometric,
  // 时间，单位为秒。节点域 [knots[degree], knots[n_ctrlp]] 仿射映射到
  // [min_parameter, max_parameter]，evaluate()/sample() 返回的导数为
  // 对时间的导数。
  kTime,
};

struct ParameterDomain {
  double min_parameter{0.0};
  double max_parameter{1.0};
  ParameterKind kind{ParameterKind::kGeometric};
};

// 曲线几何后端。TinySpline 为首版基线；Eigen Splines 为内部对照候选；
// EGO-Planner UniformBspline 为对照后端（GPL-3.0 隔离库
// planner_bspline_ego，须先调用 detail::install_ego_backend()）。
// 三个后端提供相同的公开语义（右侧分段、显式节点、域内求值）。
enum class BackendKind {
  kTinyspline,
  kEigenSplines,
  kEgoUniform,
};

// 当前选定的默认后端。内部选型（正确性对照 + 微基准）已完成并保持
// TinySpline（见 Docs/reports/bspline/20261010T150507859812Z_bspline_da8e3be8.md）；
// 若未来选型切换默认后端，只改这一处。
inline constexpr BackendKind kDefaultBackend = BackendKind::kTinyspline;

struct EvaluationResult {
  Status status{Status::kSuccess};
  std::string message;
  // 请求的参数（原样回显）与求导阶数。
  double parameter{0.0};
  int derivative_order{0};
  // 曲线在参数处的坐标（阶数 0）或导向量（阶数 >= 1）。
  // 高于曲线次数的导数恒为零，并以 kSuccess 状态如实报告。
  common::Point2d value{};
};

// 弧长重采样结果中的一个点。
struct ArcLengthSample {
  // 采样点在曲线参数域中的参数。
  double parameter{0.0};
  common::Point2d position{};
  // 从参数域起点到该采样点的累积弧长。
  double arc_length_m{0.0};
};

struct ArcLengthResampleResult {
  Status status{Status::kSuccess};
  std::string message;
  double total_length_m{0.0};
  // 相邻采样点之间请求的/推导出的间距。
  double nominal_interval_m{0.0};
  // 除基于区间的重采样的最后一个余数区间外，所有区间的
  // |实测区间弧长 - nominal_interval_m| 最大值。
  // 通过数值积分测量，而非弦长。
  double max_interval_error_m{0.0};
  // 最后一个区间的误差。对于 has_remainder == true 的基于区间的重采样，
  // 它是 |remainder - nominal_interval_m|，按契约不计入
  // max_interval_error_m（末段余数单独处理，见第 7.1 节）。
  double last_interval_error_m{0.0};
  // 当基于区间的重采样产生的最后区间短于 nominal_interval_m 时为 true。
  bool has_remainder{false};
  // 仅由按值返回采样点的重载填充；基于缓冲区的重载改为写入调用方缓冲区。
  std::vector<ArcLengthSample> samples;
};

// 不可变的 2D B-spline 曲线。可复制、可移动；复制为深拷贝。
class Bspline {
 public:
  Bspline(const Bspline& other);
  Bspline& operator=(const Bspline& other);
  Bspline(Bspline&&) noexcept;
  Bspline& operator=(Bspline&&) noexcept;
  ~Bspline();

  // 由显式控制点和显式节点向量构造几何曲线。次数由两者的规模推导：
  //   degree = knots.size() - control_points.size() - 1.
  // 以下情况会被拒绝（返回 std::nullopt；提供 error_message 时写入其中）：
  //   * 控制点少于两个、degree < 1 或 degree >= 控制点数量；
  //   * knots.size() != control_points.size() + degree + 1；
  //   * 节点递减或节点重数 > degree + 1；
  //   * 控制点或节点值非有限；
  //   * 参数域为空（knots[degree] == knots[n_ctrlp]）。
  static std::optional<Bspline> create(std::vector<common::Point2d> control_points,
                                       std::vector<double> knots,
                                       std::string* error_message = nullptr);

  // 同上，但显式选择几何后端（内部对照/测试用；默认后端见
  // kDefaultBackend）。拒绝条件与默认重载完全一致。
  static std::optional<Bspline> create(std::vector<common::Point2d> control_points,
                                       std::vector<double> knots,
                                       BackendKind backend,
                                       std::string* error_message = nullptr);

  // 构造时间参数化曲线：相同的几何与显式节点向量，
  // 节点域仿射映射到 [start_time_s, end_time_s]。
  // 此后 evaluate()/sample() 以秒为参数并返回对时间的导数。
  // start_time_s 必须为有限值且 < end_time_s。
  static std::optional<Bspline> create_time_parameterized(
      std::vector<common::Point2d> control_points, std::vector<double> knots,
      double start_time_s, double end_time_s,
      std::string* error_message = nullptr);

  // 同上，但显式选择几何后端。
  static std::optional<Bspline> create_time_parameterized(
      std::vector<common::Point2d> control_points, std::vector<double> knots,
      double start_time_s, double end_time_s, BackendKind backend,
      std::string* error_message = nullptr);

  int degree() const noexcept;
  std::size_t control_point_count() const noexcept;
  const std::vector<common::Point2d>& control_points() const noexcept;
  // 完整的显式节点向量（control_point_count + degree + 1 个值）。
  const std::vector<double>& knots() const noexcept;
  // 公开参数域：几何曲线为节点域，时间参数化曲线为时间区间。
  ParameterDomain parameter_domain() const noexcept;
  bool is_geometric_parameter() const noexcept;

  // 当前几何后端（构造时选择）及其稳定名称（"tinyspline" /
  // "eigen_splines" / "ego_uniform"）。
  BackendKind backend() const noexcept;
  const char* backend_name() const noexcept;

  // 求曲线本身（derivative_order == 0）或其导数。
  //   * derivative_order 超出 [0, 3] -> kDerivativeOrderUnsupported；
  //   * 参数超出参数域（超过 1e-12 的相对舍入容差）-> kParameterOutOfDomain；
  //   * derivative_order > degree -> 零向量，kSuccess；
  //   * 在内部节点处返回右侧分段的取值；在参数域最后一点处返回最后一个
  //     分段的取值。导数间断处即为单侧右导数。
  EvaluationResult evaluate(double parameter, int derivative_order = 0) const;

  // 在预先算好的参数上批量求值，等价于对每个参数调用 evaluate()。
  // 输出缓冲区会被调整为 parameters.size()；仅在缓冲区增长时分配内存。
  // 首先校验所有参数：出现 kParameterOutOfDomain 或
  // kDerivativeOrderUnsupported 时缓冲区保持不变。
  Status sample(const std::vector<double>& parameters,
                std::vector<common::Point2d>& output_buffer,
                int derivative_order = 0) const;

  // 总几何弧长，通过数值积分高精度计算。
  double total_length_m() const;

  // 近似等弧长重采样，输出 point_count 个点（point_count >= 1），
  // 包含首尾点。标称间距为 total_length_m() / (point_count - 1)。
  // 曲线长度为零时返回 kDegenerateCurve。当实测最大区间误差超过
  // interval_tolerance_m 时状态为 kArcLengthToleranceExceeded
  //（采样点仍会返回用于诊断）。
  ArcLengthResampleResult sample_equal_arc_length(
      std::size_t point_count, double interval_tolerance_m = 1e-3) const;
  ArcLengthResampleResult sample_equal_arc_length(
      std::size_t point_count, std::vector<ArcLengthSample>& output_buffer,
      double interval_tolerance_m = 1e-3) const;

  // 按指定的弧长间隔重采样。采样点位于
  // 0, interval_m, 2*interval_m, ...；存在余数时在参数域末端再补一个点；
  // 余数区间通过 has_remainder / last_interval_error_m 报告，
  // 且不计入 max_interval_error_m。拒绝非正或非有限的 interval_m。
  ArcLengthResampleResult resample_uniform_interval(
      double interval_m, double interval_tolerance_m = 1e-3) const;
  ArcLengthResampleResult resample_uniform_interval(
      double interval_m, std::vector<ArcLengthSample>& output_buffer,
      double interval_tolerance_m = 1e-3) const;

 private:
  explicit Bspline(std::unique_ptr<detail::SplineImpl> impl);
  std::unique_ptr<detail::SplineImpl> impl_;
};

// 便捷辅助函数：为给定次数的 control_point_count 个控制点生成
// [domain_min, domain_max] 上的夹持均匀节点向量
//（两端重数为 degree + 1，内部节点均匀分布）。
std::vector<double> clamped_uniform_knots(std::size_t control_point_count,
                                          int degree, double domain_min = 0.0,
                                          double domain_max = 1.0);

}  // namespace planner::bspline
