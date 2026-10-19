#include "planner/rs/reeds_shepp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

// ---------------------------------------------------------------------------
// 原生解析式 Reeds-Shepp 求解器。
//
// 下面的 48 词枚举沿用 OMPL 1.6.0
// `src/ompl/base/spaces/src/ReedsSheppStateSpace.cpp`
// （BSD-3-Clause，Copyright (c) 2010 Rice University）的结构与公式，
// 后者实现了：
//   J.A. Reeds and L.A. Shepp, "Optimal paths for a car that goes both
//   forwards and backwards", Pacific Journal of Mathematics, 145(2):367-393,
//   1990.
//
// 本次重写去除了所有 OMPL 类型：planner::rs 从不暴露 OMPL 对象，
// 且该组件不依赖 OMPL 也能工作。可选的 OMPL 适配器
// （src/ompl_reference.cpp）仅作封装一致性参照；由于两侧实现
// 同一曲线族，其比较明确*不是*独立的双重验证
// （Docs/component_development_plan.md 3.7）。独立证据来自解析直线/圆弧
// 测试、刚体变换不变性、曲率界和终点重构。
// ---------------------------------------------------------------------------

namespace planner::rs {
namespace {

constexpr double kPi = common::kPi;
constexpr double kTwoPi = 2.0 * common::kPi;
constexpr double kZero = 10.0 * std::numeric_limits<double>::epsilon();

// ---------------------------------------------------------------------------
// 从 Reeds-Shepp 术语体系转写的小型辅助函数。
// ---------------------------------------------------------------------------

// 与参考实现一致地环绕到 (-pi, pi]；公开结果再用
// common::normalize_angle_rad 重新归一化到 (-pi, pi]。
double mod2pi(double x) {
  double v = std::fmod(x, kTwoPi);
  if (v < -kPi) {
    v += kTwoPi;
  } else if (v > kPi) {
    v -= kTwoPi;
  }
  return v;
}

void polar(double x, double y, double& r, double& theta) {
  r = std::sqrt(x * x + y * y);
  theta = std::atan2(y, x);
}

void tauOmega(double u, double v, double xi, double eta, double phi, double& tau,
              double& omega) {
  const double delta = mod2pi(u - v);
  const double a = std::sin(u) - std::sin(delta);
  const double b = std::cos(u) - std::cos(delta) - 1.0;
  const double t1 = std::atan2(eta * a - xi * b, xi * a + eta * b);
  const double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
  tau = (t2 < 0.0) ? mod2pi(t1 + kPi) : mod2pi(t1);
  omega = mod2pi(tau - u + v - phi);
}

// ---------------------------------------------------------------------------
// 原始路径表示：按行进顺序排列的带符号常曲率原始段，
// 转弯半径为单位 1。
// ---------------------------------------------------------------------------

enum class Turn { kLeft, kStraight, kRight, kNone };

struct RawSegment {
  Turn turn{Turn::kNone};
  double length{0.0};  // 带符号的单位半径弧长；< 0 == 倒车挡
};

struct RawPath {
  RawSegment segments[5];
  int count{0};
  double total{std::numeric_limits<double>::max()};
};

// 枚举中的 18 个路径词 (L/R/S)。索引与参考实现一致；
// 末尾的 kNone 项用于终止路径词。
constexpr Turn kPathType[18][5] = {
    {Turn::kLeft, Turn::kRight, Turn::kLeft, Turn::kNone, Turn::kNone},        // 0
    {Turn::kRight, Turn::kLeft, Turn::kRight, Turn::kNone, Turn::kNone},       // 1
    {Turn::kLeft, Turn::kRight, Turn::kLeft, Turn::kRight, Turn::kNone},       // 2
    {Turn::kRight, Turn::kLeft, Turn::kRight, Turn::kLeft, Turn::kNone},       // 3
    {Turn::kLeft, Turn::kRight, Turn::kStraight, Turn::kLeft, Turn::kNone},    // 4
    {Turn::kRight, Turn::kLeft, Turn::kStraight, Turn::kRight, Turn::kNone},   // 5
    {Turn::kLeft, Turn::kStraight, Turn::kRight, Turn::kLeft, Turn::kNone},    // 6
    {Turn::kRight, Turn::kStraight, Turn::kLeft, Turn::kRight, Turn::kNone},   // 7
    {Turn::kLeft, Turn::kRight, Turn::kStraight, Turn::kRight, Turn::kNone},   // 8
    {Turn::kRight, Turn::kLeft, Turn::kStraight, Turn::kLeft, Turn::kNone},    // 9
    {Turn::kRight, Turn::kStraight, Turn::kRight, Turn::kLeft, Turn::kNone},   // 10
    {Turn::kLeft, Turn::kStraight, Turn::kLeft, Turn::kRight, Turn::kNone},    // 11
    {Turn::kLeft, Turn::kStraight, Turn::kRight, Turn::kNone, Turn::kNone},    // 12
    {Turn::kRight, Turn::kStraight, Turn::kLeft, Turn::kNone, Turn::kNone},    // 13
    {Turn::kLeft, Turn::kStraight, Turn::kLeft, Turn::kNone, Turn::kNone},     // 14
    {Turn::kRight, Turn::kStraight, Turn::kRight, Turn::kNone, Turn::kNone},   // 15
    {Turn::kLeft, Turn::kRight, Turn::kStraight, Turn::kLeft, Turn::kRight},   // 16
    {Turn::kRight, Turn::kLeft, Turn::kStraight, Turn::kRight, Turn::kLeft},   // 17
};

RawPath makeRawPath(int type_index, double a, double b = 0.0, double c = 0.0,
                    double d = 0.0, double e = 0.0) {
  const double lengths[5] = {a, b, c, d, e};
  RawPath path;
  path.count = 0;
  for (int i = 0; i < 5; ++i) {
    if (kPathType[type_index][i] == Turn::kNone) {
      break;
    }
    path.segments[path.count].turn = kPathType[type_index][i];
    path.segments[path.count].length = lengths[i];
    ++path.count;
  }
  double total = 0.0;
  for (int i = 0; i < path.count; ++i) {
    total += std::fabs(path.segments[i].length);
  }
  path.total = total;
  return path;
}

// ---------------------------------------------------------------------------
// Reeds-Shepp 论文的九组公式。注释沿用参考实现使用的论文编号。
// ---------------------------------------------------------------------------

// 公式 8.1
bool LpSpLp(double x, double y, double phi, double& t, double& u, double& v) {
  polar(x - std::sin(phi), y - 1.0 + std::cos(phi), u, t);
  if (t >= -kZero) {
    v = mod2pi(phi - t);
    if (v >= -kZero) {
      return true;
    }
  }
  return false;
}

// 公式 8.2
bool LpSpRp(double x, double y, double phi, double& t, double& u, double& v) {
  double t1, u1;
  polar(x + std::sin(phi), y - 1.0 - std::cos(phi), u1, t1);
  u1 = u1 * u1;
  if (u1 >= 4.0) {
    u = std::sqrt(u1 - 4.0);
    const double theta = std::atan2(2.0, u);
    t = mod2pi(t1 + theta);
    v = mod2pi(t - phi);
    return t >= -kZero && v >= -kZero;
  }
  return false;
}

void CSC(double x, double y, double phi, RawPath& path) {
  double t, u, v, L;
  double Lmin = path.total;
  if (LpSpLp(x, y, phi, t, u, v) && Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(14, t, u, v);
    Lmin = L;
  }
  if (LpSpLp(-x, y, -phi, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(14, -t, -u, -v);
    Lmin = L;
  }
  if (LpSpLp(x, -y, -phi, t, u, v) &&  // 镜像
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(15, t, u, v);
    Lmin = L;
  }
  if (LpSpLp(-x, -y, phi, t, u, v) &&  // 时间反转 + 镜像
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(15, -t, -u, -v);
    Lmin = L;
  }
  if (LpSpRp(x, y, phi, t, u, v) && Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(12, t, u, v);
    Lmin = L;
  }
  if (LpSpRp(-x, y, -phi, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(12, -t, -u, -v);
    Lmin = L;
  }
  if (LpSpRp(x, -y, -phi, t, u, v) &&  // 镜像
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(13, t, u, v);
    Lmin = L;
  }
  if (LpSpRp(-x, -y, phi, t, u, v) &&  // 时间反转 + 镜像
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(13, -t, -u, -v);
    Lmin = L;
  }
}

// 公式 8.3 / 8.4 *** 论文原文有笔误 ***
bool LpRmL(double x, double y, double phi, double& t, double& u, double& v) {
  const double xi = x - std::sin(phi), eta = y - 1.0 + std::cos(phi);
  double u1, theta;
  polar(xi, eta, u1, theta);
  if (u1 <= 4.0) {
    u = -2.0 * std::asin(0.25 * u1);
    t = mod2pi(theta + 0.5 * u + kPi);
    v = mod2pi(phi - t + u);
    return t >= -kZero && u <= kZero;
  }
  return false;
}

void CCC(double x, double y, double phi, RawPath& path) {
  double t, u, v, L;
  double Lmin = path.total;
  if (LpRmL(x, y, phi, t, u, v) && Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(0, t, u, v);
    Lmin = L;
  }
  if (LpRmL(-x, y, -phi, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(0, -t, -u, -v);
    Lmin = L;
  }
  if (LpRmL(x, -y, -phi, t, u, v) &&  // 镜像
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(1, t, u, v);
    Lmin = L;
  }
  if (LpRmL(-x, -y, phi, t, u, v) &&  // 时间反转 + 镜像
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(1, -t, -u, -v);
    Lmin = L;
  }

  // 反向
  const double xb = x * std::cos(phi) + y * std::sin(phi);
  const double yb = x * std::sin(phi) - y * std::cos(phi);
  if (LpRmL(xb, yb, phi, t, u, v) &&
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(0, v, u, t);
    Lmin = L;
  }
  if (LpRmL(-xb, yb, -phi, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(0, -v, -u, -t);
    Lmin = L;
  }
  if (LpRmL(xb, -yb, -phi, t, u, v) &&  // 镜像
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(1, v, u, t);
    Lmin = L;
  }
  if (LpRmL(-xb, -yb, phi, t, u, v) &&  // 时间反转 + 镜像
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(1, -v, -u, -t);
    Lmin = L;
  }
}

// 公式 8.7
bool LpRupLumRm(double x, double y, double phi, double& t, double& u, double& v) {
  const double xi = x + std::sin(phi), eta = y - 1.0 - std::cos(phi);
  const double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));
  if (rho <= 1.0) {
    u = std::acos(rho);
    tauOmega(u, -u, xi, eta, phi, t, v);
    return t >= -kZero && v <= kZero;
  }
  return false;
}

// 公式 8.8
bool LpRumLumRp(double x, double y, double phi, double& t, double& u, double& v) {
  const double xi = x + std::sin(phi), eta = y - 1.0 - std::cos(phi);
  const double rho = (20.0 - xi * xi - eta * eta) / 16.0;
  if (rho >= 0.0 && rho <= 1.0) {
    u = -std::acos(rho);
    if (u >= -0.5 * kPi) {
      tauOmega(u, u, xi, eta, phi, t, v);
      return t >= -kZero && v >= -kZero;
    }
  }
  return false;
}

void CCCC(double x, double y, double phi, RawPath& path) {
  double t, u, v, L;
  double Lmin = path.total;
  if (LpRupLumRm(x, y, phi, t, u, v) &&
      Lmin > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(2, t, u, -u, v);
    Lmin = L;
  }
  if (LpRupLumRm(-x, y, -phi, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(2, -t, -u, u, -v);
    Lmin = L;
  }
  if (LpRupLumRm(x, -y, -phi, t, u, v) &&  // 镜像
      Lmin > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(3, t, u, -u, v);
    Lmin = L;
  }
  if (LpRupLumRm(-x, -y, phi, t, u, v) &&  // 时间反转 + 镜像
      Lmin > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(3, -t, -u, u, -v);
    Lmin = L;
  }

  if (LpRumLumRp(x, y, phi, t, u, v) &&
      Lmin > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(2, t, u, u, v);
    Lmin = L;
  }
  if (LpRumLumRp(-x, y, -phi, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(2, -t, -u, -u, -v);
    Lmin = L;
  }
  if (LpRumLumRp(x, -y, -phi, t, u, v) &&  // 镜像
      Lmin > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(3, t, u, u, v);
    Lmin = L;
  }
  if (LpRumLumRp(-x, -y, phi, t, u, v) &&  // 时间反转 + 镜像
      Lmin > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(3, -t, -u, -u, -v);
    Lmin = L;
  }
}

// 公式 8.9
bool LpRmSmLm(double x, double y, double phi, double& t, double& u, double& v) {
  const double xi = x - std::sin(phi), eta = y - 1.0 + std::cos(phi);
  double rho, theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.0) {
    const double r = std::sqrt(rho * rho - 4.0);
    u = 2.0 - r;
    t = mod2pi(theta + std::atan2(r, -2.0));
    v = mod2pi(phi - 0.5 * kPi - t);
    return t >= -kZero && u <= kZero && v <= kZero;
  }
  return false;
}

// 公式 8.10
bool LpRmSmRm(double x, double y, double phi, double& t, double& u, double& v) {
  const double xi = x + std::sin(phi), eta = y - 1.0 - std::cos(phi);
  double rho, theta;
  polar(-eta, xi, rho, theta);
  if (rho >= 2.0) {
    t = theta;
    u = 2.0 - rho;
    v = mod2pi(t + 0.5 * kPi - phi);
    return t >= -kZero && u <= kZero && v <= kZero;
  }
  return false;
}

void CCSC(double x, double y, double phi, RawPath& path) {
  double t, u, v, L;
  double Lmin = path.total - 0.5 * kPi;
  if (LpRmSmLm(x, y, phi, t, u, v) &&
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(4, t, -0.5 * kPi, u, v);
    Lmin = L;
  }
  if (LpRmSmLm(-x, y, -phi, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(4, -t, 0.5 * kPi, -u, -v);
    Lmin = L;
  }
  if (LpRmSmLm(x, -y, -phi, t, u, v) &&  // 镜像
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(5, t, -0.5 * kPi, u, v);
    Lmin = L;
  }
  if (LpRmSmLm(-x, -y, phi, t, u, v) &&  // 时间反转 + 镜像
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(5, -t, 0.5 * kPi, -u, -v);
    Lmin = L;
  }

  if (LpRmSmRm(x, y, phi, t, u, v) &&
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(8, t, -0.5 * kPi, u, v);
    Lmin = L;
  }
  if (LpRmSmRm(-x, y, -phi, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(8, -t, 0.5 * kPi, -u, -v);
    Lmin = L;
  }
  if (LpRmSmRm(x, -y, -phi, t, u, v) &&  // 镜像
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(9, t, -0.5 * kPi, u, v);
    Lmin = L;
  }
  if (LpRmSmRm(-x, -y, phi, t, u, v) &&  // 时间反转 + 镜像
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(9, -t, 0.5 * kPi, -u, -v);
    Lmin = L;
  }

  // 反向
  const double xb = x * std::cos(phi) + y * std::sin(phi);
  const double yb = x * std::sin(phi) - y * std::cos(phi);
  if (LpRmSmLm(xb, yb, phi, t, u, v) &&
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(6, v, u, -0.5 * kPi, t);
    Lmin = L;
  }
  if (LpRmSmLm(-xb, yb, -phi, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(6, -v, -u, 0.5 * kPi, -t);
    Lmin = L;
  }
  if (LpRmSmLm(xb, -yb, -phi, t, u, v) &&  // 镜像
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(7, v, u, -0.5 * kPi, t);
    Lmin = L;
  }
  if (LpRmSmLm(-xb, -yb, phi, t, u, v) &&  // 时间反转 + 镜像
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(7, -v, -u, 0.5 * kPi, -t);
    Lmin = L;
  }

  if (LpRmSmRm(xb, yb, phi, t, u, v) &&
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(10, v, u, -0.5 * kPi, t);
    Lmin = L;
  }
  if (LpRmSmRm(-xb, yb, -phi, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(10, -v, -u, 0.5 * kPi, -t);
    Lmin = L;
  }
  if (LpRmSmRm(xb, -yb, -phi, t, u, v) &&  // 镜像
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(11, v, u, -0.5 * kPi, t);
    Lmin = L;
  }
  if (LpRmSmRm(-xb, -yb, phi, t, u, v) &&  // 时间反转 + 镜像
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(11, -v, -u, 0.5 * kPi, -t);
    Lmin = L;
  }
}

// 公式 8.11 *** 论文原文有笔误 ***
bool LpRmSLmRp(double x, double y, double phi, double& t, double& u, double& v) {
  const double xi = x + std::sin(phi), eta = y - 1.0 - std::cos(phi);
  double rho, theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.0) {
    u = 4.0 - std::sqrt(rho * rho - 4.0);
    if (u <= kZero) {
      t = mod2pi(std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
      v = mod2pi(t - phi);
      return t >= -kZero && v >= -kZero;
    }
  }
  return false;
}

void CCSCC(double x, double y, double phi, RawPath& path) {
  double t, u, v, L;
  double Lmin = path.total - kPi;
  if (LpRmSLmRp(x, y, phi, t, u, v) &&
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(16, t, -0.5 * kPi, u, -0.5 * kPi, v);
    Lmin = L;
  }
  if (LpRmSLmRp(-x, y, -phi, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(16, -t, 0.5 * kPi, -u, 0.5 * kPi, -v);
    Lmin = L;
  }
  if (LpRmSLmRp(x, -y, -phi, t, u, v) &&  // 镜像
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(17, t, -0.5 * kPi, u, -0.5 * kPi, v);
    Lmin = L;
  }
  if (LpRmSLmRp(-x, -y, phi, t, u, v) &&  // 时间反转 + 镜像
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(17, -t, 0.5 * kPi, -u, 0.5 * kPi, -v);
    Lmin = L;
  }
}

// 完整的 48 词枚举；返回找到的最短有效词。
RawPath reedsSheppUnit(double x, double y, double phi) {
  RawPath path;
  CSC(x, y, phi, path);
  CCC(x, y, phi, path);
  CCCC(x, y, phi, path);
  CCSC(x, y, phi, path);
  CCSCC(x, y, phi, path);
  return path;
}

// ---------------------------------------------------------------------------
// 转换辅助函数。
// ---------------------------------------------------------------------------

common::Pose2d normalize_pose(const common::Pose2d& pose) {
  common::Pose2d out = pose;
  out.yaw_rad = common::normalize_angle_rad(pose.yaw_rad);
  return out;
}

bool finite(double value) { return std::isfinite(value); }

bool finite_pose(const common::Pose2d& pose) {
  return finite(pose.x_m) && finite(pose.y_m) && finite(pose.yaw_rad);
}

double curvature_for_turn(Turn turn, double radius_m) {
  switch (turn) {
    case Turn::kLeft:
      return 1.0 / radius_m;
    case Turn::kRight:
      return -1.0 / radius_m;
    case Turn::kStraight:
    case Turn::kNone:
      break;
  }
  return 0.0;
}

// 不做角度归一化的原始双精度推进（三角函数使归一化对位置更新无影响；
// 位姿对外暴露时 yaw 只归一化一次）。
common::Pose2d advance_unchecked(const common::Pose2d& pose,
                                 const RsSegment& segment, double ds_m) {
  const double gear = (segment.direction == common::Direction::kForward) ? 1.0 : -1.0;
  const double kappa = segment.curvature_inv_m;
  if (std::fabs(kappa) <= 1e-12) {
    return {pose.x_m + gear * ds_m * std::cos(pose.yaw_rad),
            pose.y_m + gear * ds_m * std::sin(pose.yaw_rad), pose.yaw_rad};
  }
  const double yaw_end = pose.yaw_rad + gear * kappa * ds_m;
  return {pose.x_m + (std::sin(yaw_end) - std::sin(pose.yaw_rad)) / kappa,
          pose.y_m - (std::cos(yaw_end) - std::cos(pose.yaw_rad)) / kappa,
          yaw_end};
}

// 将单位半径原始词转换为缩放后的分段路径。零长度原始段会被丢弃，
// 挡位与曲率相同的相邻原始段会被合并（之间无尖点），
// 从而保持尖点边界精确。
//
// *归一化*（单位半径）长度不超过该阈值的原始段是闭式公式的舍入幻影
// （例如 polar(0, cos(pi/2)) 得到约 6e-17）。丢弃它们至多使终点变化
// 5 * 1e-14 * R，远在 1e-6 m / 1e-6 rad 验收容差之内，并使分段结构
// 中不含亚纳米级的碎片。
constexpr double kMinNormalizedStroke = 1e-14;

RsPath build_path(const common::Pose2d& start, const common::Pose2d& goal,
                  double radius_m, const RawPath& raw) {
  RsPath out;
  out.start = start;
  out.goal = goal;
  common::Pose2d cursor = start;
  double total = 0.0;
  for (int i = 0; i < raw.count; ++i) {
    const Turn turn = raw.segments[i].turn;
    if (turn == Turn::kNone) {
      continue;
    }
    const double signed_length = raw.segments[i].length;
    if (!(std::fabs(signed_length) > kMinNormalizedStroke)) {
      continue;
    }
    const double length_m = std::fabs(signed_length) * radius_m;
    RsSegment segment;
    segment.direction = (signed_length >= 0.0) ? common::Direction::kForward
                                               : common::Direction::kReverse;
    segment.length_m = length_m;
    segment.curvature_inv_m = curvature_for_turn(turn, radius_m);
    total += segment.length_m;
    cursor = advance_unchecked(cursor, segment, segment.length_m);
    if (!out.segments.empty()) {
      RsSegment& last = out.segments.back();
      if (last.direction == segment.direction &&
          last.curvature_inv_m == segment.curvature_inv_m) {
        last.length_m += segment.length_m;
        continue;
      }
    }
    out.segments.push_back(segment);
  }
  out.length_m = total;
  return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// 公开 API。
// ---------------------------------------------------------------------------

bool is_valid_query(const common::Pose2d& start, const common::Pose2d& goal,
                    const common::VehicleModel& vehicle) noexcept {
  if (!finite_pose(start) || !finite_pose(goal)) {
    return false;
  }
  if (!finite(vehicle.wheelbase_m) || !(vehicle.wheelbase_m > 0.0)) {
    return false;
  }
  if (!finite(vehicle.max_steer_rad) || !(vehicle.max_steer_rad > 0.0) ||
      !(vehicle.max_steer_rad < 0.5 * kPi)) {
    return false;
  }
  const double radius = vehicle.wheelbase_m / std::tan(vehicle.max_steer_rad);
  return finite(radius) && radius > 0.0;
}

double min_turn_radius_m(const common::VehicleModel& vehicle) {
  if (!finite(vehicle.wheelbase_m) || !(vehicle.wheelbase_m > 0.0)) {
    throw std::invalid_argument(
        "min_turn_radius_m: wheelbase_m must be finite and > 0");
  }
  if (!finite(vehicle.max_steer_rad) || !(vehicle.max_steer_rad > 0.0) ||
      !(vehicle.max_steer_rad < 0.5 * kPi)) {
    throw std::invalid_argument(
        "min_turn_radius_m: max_steer_rad must be finite, > 0 and < pi/2");
  }
  const double radius = vehicle.wheelbase_m / std::tan(vehicle.max_steer_rad);
  if (!finite(radius) || !(radius > 0.0)) {
    throw std::invalid_argument("min_turn_radius_m: derived radius is not positive");
  }
  return radius;
}

RsPlan plan(const common::Pose2d& start, const common::Pose2d& goal,
            const common::VehicleModel& vehicle) {
  if (!is_valid_query(start, goal, vehicle)) {
    return {common::Status::kInvalidInput,
            "invalid input: poses must be finite and the vehicle needs "
            "wheelbase_m > 0 and 0 < max_steer_rad < pi/2",
            {}};
  }
  const double radius = min_turn_radius_m(vehicle);
  const common::Pose2d start_norm = normalize_pose(start);
  const common::Pose2d goal_norm = normalize_pose(goal);

  // 起点坐标系下的目标点，按转弯半径归一化（论文单位）。
  const double c = std::cos(start_norm.yaw_rad);
  const double s = std::sin(start_norm.yaw_rad);
  const double dx = goal_norm.x_m - start_norm.x_m;
  const double dy = goal_norm.y_m - start_norm.y_m;
  const double x = (c * dx + s * dy) / radius;
  const double y = (-s * dx + c * dy) / radius;
  const double phi = common::normalize_angle_rad(goal_norm.yaw_rad - start_norm.yaw_rad);

  const RawPath raw = reedsSheppUnit(x, y, phi);
  RsPath result = build_path(start_norm, goal_norm, radius, raw);

  // 硬性终点验证：路径未在冻结容差内到达目标时，绝不报告成功。
  common::Pose2d cursor = result.start;
  for (const RsSegment& segment : result.segments) {
    cursor = advance_unchecked(cursor, segment, segment.length_m);
  }
  cursor.yaw_rad = common::normalize_angle_rad(cursor.yaw_rad);
  const double position_error =
      std::hypot(cursor.x_m - goal_norm.x_m, cursor.y_m - goal_norm.y_m);
  const double yaw_error = std::fabs(
      common::normalize_angle_rad(cursor.yaw_rad - goal_norm.yaw_rad));
  if (!(position_error <= kEndpointPositionToleranceM) ||
      !(yaw_error <= kEndpointYawToleranceRad)) {
    return {common::Status::kOptimizationFailed,
            "endpoint verification failed: position_error_m=" +
                std::to_string(position_error) +
                " yaw_error_rad=" + std::to_string(yaw_error),
            {}};
  }
  return {common::Status::kSuccess, "", std::move(result)};
}

double distance(const common::Pose2d& start, const common::Pose2d& goal,
                const common::VehicleModel& vehicle) {
  const RsPlan result = plan(start, goal, vehicle);
  if (result.status == common::Status::kInvalidInput) {
    throw std::invalid_argument("rs::distance: " + result.reason);
  }
  if (result.status != common::Status::kSuccess) {
    throw std::runtime_error("rs::distance: " + result.reason);
  }
  return result.path.length_m;
}

RsPath path(const common::Pose2d& start, const common::Pose2d& goal,
            const common::VehicleModel& vehicle) {
  RsPlan result = plan(start, goal, vehicle);
  if (result.status == common::Status::kInvalidInput) {
    throw std::invalid_argument("rs::path: " + result.reason);
  }
  if (result.status != common::Status::kSuccess) {
    throw std::runtime_error("rs::path: " + result.reason);
  }
  return std::move(result.path);
}

common::Pose2d integrate_segment(const common::Pose2d& start,
                                 const RsSegment& segment, double ds_m) {
  if (!finite(ds_m) || ds_m < 0.0) {
    throw std::invalid_argument(
        "integrate_segment: ds_m must be finite and >= 0");
  }
  if (!finite(segment.length_m) || segment.length_m < 0.0 ||
      !finite(segment.curvature_inv_m)) {
    throw std::invalid_argument("integrate_segment: malformed segment");
  }
  if (!finite_pose(start)) {
    throw std::invalid_argument("integrate_segment: start pose must be finite");
  }
  const double limit = segment.length_m * (1.0 + 1e-12) + 1e-12;
  if (ds_m > limit) {
    throw std::invalid_argument(
        "integrate_segment: ds_m exceeds the segment length");
  }
  const double ds = std::min(ds_m, segment.length_m);
  common::Pose2d out = advance_unchecked(start, segment, ds);
  out.yaw_rad = common::normalize_angle_rad(out.yaw_rad);
  return out;
}

}  // namespace planner::rs
