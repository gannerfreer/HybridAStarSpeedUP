#pragma once

// planner_bspline 的内部实现元数据与节点辅助函数。
// 不随包安装；几何后端（TinySpline / Eigen Splines）通过 Backend 接口
// 隔离，公开头文件绝不包含任何上游类型。

#include <cstddef>
#include <memory>
#include <vector>

#include "planner/bspline/curve.hpp"
#include "planner/common/types.hpp"

namespace planner::bspline::detail {

class Backend;

// 选择的后端域宽度，使 TinySpline 的绝对节点 epsilon
//（TS_KNOT_EPSILON = 1e-4）对应公开域宽度的 1e-10。
inline constexpr double kScaledSpan = 1.0e6;

// 持有为公开 Bspline 类提供服务所需的全部内容。
//
// 几何后端保存为不可变共享句柄（Backend）：复制 Bspline 只复制引用，
// 句柄在构造后不再变化。
//
// TinySpline 使用 1e-4 的绝对节点 epsilon（TS_KNOT_EPSILON，针对单精度
// 调校）：与节点相差 1e-4 以内的参数会被吸附到该节点。因此封装层让后端
// 工作在内部缩放的域中（kScaledSpan，见 detail.cpp），使实际相对吸附
// 容差约为域宽度的 1e-10。所有内部 `u` 值都是后端（缩放）参数；
// knot_min/knot_max/knots 保持 API 所用的公开单位。
struct SplineImpl {
  std::shared_ptr<const Backend> backend;  // 后端几何句柄（不可变）
  std::vector<common::Point2d> control_points;
  std::vector<double> knots;         // 公开（显式）节点向量
  std::vector<double> scaled_knots;  // 后端节点向量，布局相同
  int degree{0};
  double knot_min{0.0};              // 公开参数域
  double knot_max{1.0};
  double scaled_min{0.0};            // 后端参数域
  double scaled_max{1.0};
  double knot_scale{1.0};            // scaled = (u - knot_min) * knot_scale
  double knot_scale_inverse{1.0};
  ParameterKind parameter_kind{ParameterKind::kGeometric};
  // 时间区间（仅在 ParameterKind::kTime 时有意义）。
  double time_min{0.0};
  double time_max{1.0};
  // 重数为 degree + 1 的内部节点值（位置间断点）。
  // 间断曲线只在各参数分段内连续。
  std::vector<double> gap_knots;
};

// 公开节点参数 -> 后端（缩放）参数及反向转换。
double to_scaled_parameter(const SplineImpl& impl, double u) noexcept;
double to_original_parameter(const SplineImpl& impl, double scaled) noexcept;

// `value` 在完整节点向量中的重数（不存在时为 0）。
int knot_multiplicity(const std::vector<double>& knots, double value) noexcept;

// 当后端参数 `scaled` 是内部节点值（严格位于后端域内）时为 true。
bool interior_knot_multiplicity(const SplineImpl& impl, double scaled,
                                int* multiplicity) noexcept;

// 严格小于后端域上界且 <= `scaled` 的最大后端节点值；
// 等于包含该参数的分段的左端（参数等于域上界时为最后一个分段的左端）。
double span_left(const SplineImpl& impl, double scaled) noexcept;

// 当求导 `order` 次会合并 TinySpline 句柄的内部间断时为 true
//（某一步求导中重数 >= DEG + 2），这会使合并点之后的值失真。
// 此类求值会先限制到右侧分段再处理。
bool merge_possible(const SplineImpl& impl, int order) noexcept;

}  // namespace planner::bspline::detail
