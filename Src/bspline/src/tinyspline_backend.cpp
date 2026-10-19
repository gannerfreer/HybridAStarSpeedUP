// planner_bspline 的 TinySpline 后端实现（内部）。
//
// 该后端封装 tsBSpline 句柄，并在此处保留所有 TinySpline 特有的补偿逻辑：
//   * 后端参数位于缩放域（impl.hpp 的 kScaledSpan），避免 1e-4 绝对
//     节点 epsilon 吸附；
//   * 求导可能合并内部间断，因此派生句柄保留原始句柄供上层选择
//     restrict-then-derive 路径；
//   * 批量求值仅由本文件使用（上层通过 Backend::eval 逐个调用）。

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <string>
#include <utility>

#include <tinyspline.h>

#include "backend.hpp"

namespace planner::bspline::detail {

namespace {

static_assert(sizeof(tsReal) == sizeof(double),
              "planner_bspline requires TinySpline built with double precision");

// 持有单个 tsBSpline 句柄并确保只释放一次。
class OwnedSpline {
 public:
  OwnedSpline() = default;
  ~OwnedSpline() { reset(); }
  OwnedSpline(const OwnedSpline&) = delete;
  OwnedSpline& operator=(const OwnedSpline&) = delete;
  OwnedSpline(OwnedSpline&& other) noexcept
      : spline_(other.spline_), valid_(other.valid_) {
    other.valid_ = false;
  }
  OwnedSpline& operator=(OwnedSpline&& other) noexcept {
    if (this != &other) {
      reset();
      spline_ = other.spline_;
      valid_ = other.valid_;
      other.valid_ = false;
    }
    return *this;
  }

  // 供 TinySpline 构造函数使用的原始输出槽。
  tsBSpline* out() { return &spline_; }
  const tsBSpline& get() const { return spline_; }
  const tsBSpline* ptr() const { return &spline_; }
  bool valid() const { return valid_; }
  void set_valid() { valid_ = true; }
  void reset() {
    if (valid_) {
      ts_bspline_free(&spline_);
      valid_ = false;
    }
  }

 private:
  tsBSpline spline_{};
  bool valid_{false};
};

bool raw_eval(const tsBSpline& spline, double u, bool select_right,
              common::Point2d* out) {
  if (out == nullptr) {
    return false;
  }
  tsDeBoorNet net;
  tsStatus status;
  const tsError error = ts_bspline_eval(&spline, u, &net, &status);
  if (error != TS_SUCCESS) {
    return false;
  }
  bool ok = false;
  if (ts_deboornet_dimension(&net) == 2) {
    const std::size_t result_count = ts_deboornet_num_result(&net);
    if (result_count > 0) {
      const tsReal* result = ts_deboornet_result_ptr(&net);
      const std::size_t index =
          (select_right && result_count > 1) ? result_count - 1 : 0;
      out->x_m = result[index * 2 + 0];
      out->y_m = result[index * 2 + 1];
      ok = true;
    }
  }
  ts_deboornet_free(&net);
  return ok;
}

bool raw_derive(const tsBSpline& spline, int order, OwnedSpline* out) {
  if (out == nullptr || out->valid() || order < 1) {
    return false;
  }
  tsStatus status;
  const tsError error = ts_bspline_derive(
      &spline, static_cast<std::size_t>(order), -1.0, out->out(), &status);
  if (error != TS_SUCCESS) {
    return false;
  }
  out->set_valid();
  return true;
}

bool raw_restrict(const tsBSpline& spline, double from, double to,
                  OwnedSpline* out) {
  if (out == nullptr || out->valid() || !(from < to)) {
    return false;
  }
  tsStatus status;
  const tsError error = ts_bspline_sub_spline(&spline, from, to, out->out(),
                                              &status);
  if (error != TS_SUCCESS) {
    return false;
  }
  out->set_valid();
  return true;
}

bool raw_copy(const tsBSpline& spline, OwnedSpline* out) {
  if (out == nullptr || out->valid()) {
    return false;
  }
  tsStatus status;
  const tsError error = ts_bspline_copy(&spline, out->out(), &status);
  if (error != TS_SUCCESS) {
    return false;
  }
  out->set_valid();
  return true;
}

// TinySpline 后端句柄。持有一个有效 tsBSpline；所有句柄操作都委托给
// 上面的 raw_* 辅助函数，保持与首版实现完全相同的语义。
class TinysplineBackend final : public Backend {
 public:
  explicit TinysplineBackend(OwnedSpline spline) : spline_(std::move(spline)) {}

  BackendKind kind() const noexcept override { return BackendKind::kTinyspline; }
  const char* name() const noexcept override { return "tinyspline"; }

  bool eval(double u, bool select_right,
            common::Point2d* out) const override {
    return raw_eval(spline_.get(), u, select_right, out);
  }

  // 批量快速路径：使用 TinySpline 的 eval_all；间断参数处保留首个
  // 结果，右侧修正由调用方（curve.cpp 的 sample()）完成。
  bool eval_batch(const double* parameters, std::size_t count,
                  common::Point2d* out) const override {
    if (count == 0) {
      return true;
    }
    tsReal* points = nullptr;
    tsStatus status;
    const tsError error =
        ts_bspline_eval_all(spline_.ptr(), parameters, count, &points, &status);
    if (error != TS_SUCCESS) {
      std::free(points);
      return false;
    }
    for (std::size_t i = 0; i < count; ++i) {
      out[i].x_m = points[2 * i];
      out[i].y_m = points[2 * i + 1];
    }
    std::free(points);
    return true;
  }

  std::unique_ptr<Backend> derive(int order) const override {
    OwnedSpline derived;
    if (!raw_derive(spline_.get(), order, &derived)) {
      return nullptr;
    }
    return std::make_unique<TinysplineBackend>(std::move(derived));
  }

  std::unique_ptr<Backend> restrict(double from, double to) const override {
    OwnedSpline restricted;
    if (!raw_restrict(spline_.get(), from, to, &restricted)) {
      return nullptr;
    }
    return std::make_unique<TinysplineBackend>(std::move(restricted));
  }

  std::unique_ptr<Backend> clone() const override {
    OwnedSpline copy;
    if (!raw_copy(spline_.get(), &copy)) {
      return nullptr;
    }
    return std::make_unique<TinysplineBackend>(std::move(copy));
  }

 private:
  OwnedSpline spline_;
};

}  // namespace

std::shared_ptr<const Backend> make_tinyspline_backend(
    const std::vector<common::Point2d>& control_points,
    const std::vector<double>& scaled_knots, int degree,
    std::string* error_message) {
  const auto fail = [error_message](const std::string& message)
      -> std::shared_ptr<const Backend> {
    if (error_message != nullptr) {
      *error_message = message;
    }
    return nullptr;
  };

  OwnedSpline spline;
  tsStatus status;
  tsError error = ts_bspline_new(control_points.size(), 2,
                                 static_cast<std::size_t>(degree), TS_OPENED,
                                 spline.out(), &status);
  if (error != TS_SUCCESS) {
    return fail("TinySpline could not create the spline");
  }
  spline.set_valid();

  std::vector<tsReal> flat;
  flat.reserve(control_points.size() * 2);
  for (const common::Point2d& point : control_points) {
    flat.push_back(point.x_m);
    flat.push_back(point.y_m);
  }
  error = ts_bspline_set_control_points(spline.out(), flat.data(), &status);
  if (error != TS_SUCCESS) {
    return fail("TinySpline rejected the control points");
  }
  error = ts_bspline_set_knots(spline.out(), scaled_knots.data(), &status);
  if (error != TS_SUCCESS) {
    return fail("TinySpline rejected the knot vector");
  }
  return std::make_shared<TinysplineBackend>(std::move(spline));
}

}  // namespace planner::bspline::detail
