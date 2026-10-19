#pragma once

// planner_bspline 的后端接口（内部，不随包安装）。
//
// 一个 Backend 是不可变的几何句柄，在“后端（缩放）参数”域内提供：
//   * eval：右侧分段语义的求值（内部节点/域端点取右侧分段）；
//   * derive：解析导数句柄（供弧长积分等内部算法使用）；
//   * restrict：限制到子参数域的句柄；
//   * clone：深拷贝句柄。
//
// TinySpline 与 Eigen Splines 两个实现共同满足该接口；公开 API 只暴露
// BackendKind 枚举（不含任何上游类型或头文件）。

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "impl.hpp"

namespace planner::bspline::detail {

class Backend {
 public:
  virtual ~Backend() = default;

  virtual BackendKind kind() const noexcept = 0;
  virtual const char* name() const noexcept = 0;

  // 在缩放参数 `u` 处求值。`select_right == true` 且 `u` 位于间断参数
  //（位置间断或导数间断）时返回右侧分段的结果。`u` 超出句柄参数域时
  // 返回 false，不隐式外推。
  virtual bool eval(double u, bool select_right,
                    common::Point2d* out) const = 0;

  // 批量求值；语义与逐个 eval(u, /*select_right=*/true) 一致。默认实现
  // 逐点调用；后端可覆盖为批量快速路径（调用方负责在间断参数处做
  // 右侧修正，见 curve.cpp 的 sample()）。
  virtual bool eval_batch(const double* parameters, std::size_t count,
                          common::Point2d* out) const {
    for (std::size_t i = 0; i < count; ++i) {
      if (!eval(parameters[i], true, &out[i])) {
        return false;
      }
    }
    return true;
  }

  // `order`（>= 1）阶解析导数句柄；后端不支持时返回 nullptr。
  virtual std::unique_ptr<Backend> derive(int order) const = 0;

  // 限制到 [from, to] 子域（要求 from < to）；空域返回 nullptr。
  virtual std::unique_ptr<Backend> restrict(double from, double to) const = 0;

  // 深拷贝句柄（共享不可变状态时也可直接复制引用）。
  virtual std::unique_ptr<Backend> clone() const = 0;
};

// 构造 TinySpline 后端几何。失败时返回 nullptr 并写入 error_message。
std::shared_ptr<const Backend> make_tinyspline_backend(
    const std::vector<common::Point2d>& control_points,
    const std::vector<double>& scaled_knots, int degree,
    std::string* error_message);

// 构造 Eigen Splines 后端几何。输入已通过公开构造校验；失败时返回 nullptr。
std::shared_ptr<const Backend> make_eigen_backend(
    const std::vector<common::Point2d>& control_points,
    const std::vector<double>& scaled_knots, int degree);

// EGO-Planner UniformBspline 对照后端（GPL-3.0 隔离库 planner_bspline_ego
// 提供实现）。主库只保存运行时注册的工厂指针，不链接上游代码；未注册时
// 构造 kEgoUniform 曲线会失败并给出说明。注册须在使用前完成（非线程安全）。
using EgoBackendFactory = std::shared_ptr<const Backend> (*)(
    const std::vector<common::Point2d>& control_points,
    const std::vector<double>& scaled_knots, int degree,
    std::string* error_message);
void set_ego_backend_factory(EgoBackendFactory factory) noexcept;
EgoBackendFactory ego_backend_factory() noexcept;

}  // namespace planner::bspline::detail
