/**
 * @file reeds_shepp.cpp
 * @brief 原生 48 类解析 Reeds-Shepp 最短路径求解器的具体实现
 */

#include "reeds_shepp.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include "planner/math/kinematics/arc.hpp"
#include "planner/math/kinematics/bicycle_model.hpp"

// ---------------------------------------------------------------------------
// Reeds-Shepp 求解器
//
// 曲线枚举类遵循以下论文推导的解析公式集：
//   J.A. Reeds and L.A. Shepp, "Optimal paths for a car that goes both
//   forwards and backwards", Pacific Journal of Mathematics, 145(2):367-393, 1990.
// （架构设计参考 OMPL ReedsSheppStateSpace ）
// ---------------------------------------------------------------------------

namespace planner::rs {
namespace {

using math::kPi;
constexpr double kZero = 10.0 * std::numeric_limits<double>::epsilon();

// 缓存航向角 phi 及其正负两套正弦/余弦值，避免各解析公式重复计算三角函数
struct PhiFrame {
  double phi;
  double sin_phi;
  double cos_phi;
};

struct PhiFrames {
  PhiFrame positive;
  PhiFrame negative;
};

PhiFrame make_phi_frame(double angle) { return {angle, std::sin(angle), std::cos(angle)}; }

// ---------------------------------------------------------------------------
// Reeds-Shepp 曲线专用数学辅助函数
// ---------------------------------------------------------------------------

void polar(double x, double y, double& r, double& theta) {
  r = std::sqrt(x * x + y * y);
  theta = std::atan2(y, x);
}

void tauOmega(double u, double v, double xi, double eta, const PhiFrame& frame, double& tau,
              double& omega) {
  const double delta = math::angle_diff_rad(u, v);
  const double a = std::sin(u) - std::sin(delta);
  const double b = std::cos(u) - std::cos(delta) - 1.0;
  const double t1 = std::atan2(eta * a - xi * b, xi * a + eta * b);
  const double t2 = 2.0 * (std::cos(delta) - std::cos(v) - std::cos(u)) + 3.0;
  tau = (t2 < 0.0) ? math::normalize_angle_rad(t1 + kPi) : math::normalize_angle_rad(t1);
  omega = math::angle_diff_rad(tau - u + v, frame.phi);
}

// ---------------------------------------------------------------------------
// 无量纲原始路径表示：单位转弯半径 (R=1) 下的有符号常曲率分段
// ---------------------------------------------------------------------------

enum class Turn { kLeft, kStraight, kRight, kNone };

struct RawSegment {
  Turn turn{Turn::kNone};
  double length{0.0};  // 单位半径下的有符号弧长（< 0 表示倒车行驶）
};

struct RawPath {
  RawSegment segments[5];
  int count{0};
  double total{std::numeric_limits<double>::max()};
};

// 18 类基准曲线字类型（L=左转圆弧, R=右转圆弧, S=直行线段，kNone 表示结束）
constexpr Turn kPathType[18][5] = {
    {Turn::kLeft, Turn::kRight, Turn::kLeft, Turn::kNone, Turn::kNone},       // 0
    {Turn::kRight, Turn::kLeft, Turn::kRight, Turn::kNone, Turn::kNone},      // 1
    {Turn::kLeft, Turn::kRight, Turn::kLeft, Turn::kRight, Turn::kNone},      // 2
    {Turn::kRight, Turn::kLeft, Turn::kRight, Turn::kLeft, Turn::kNone},      // 3
    {Turn::kLeft, Turn::kRight, Turn::kStraight, Turn::kLeft, Turn::kNone},   // 4
    {Turn::kRight, Turn::kLeft, Turn::kStraight, Turn::kRight, Turn::kNone},  // 5
    {Turn::kLeft, Turn::kStraight, Turn::kRight, Turn::kLeft, Turn::kNone},   // 6
    {Turn::kRight, Turn::kStraight, Turn::kLeft, Turn::kRight, Turn::kNone},  // 7
    {Turn::kLeft, Turn::kRight, Turn::kStraight, Turn::kRight, Turn::kNone},  // 8
    {Turn::kRight, Turn::kLeft, Turn::kStraight, Turn::kLeft, Turn::kNone},   // 9
    {Turn::kRight, Turn::kStraight, Turn::kRight, Turn::kLeft, Turn::kNone},  // 10
    {Turn::kLeft, Turn::kStraight, Turn::kLeft, Turn::kRight, Turn::kNone},   // 11
    {Turn::kLeft, Turn::kStraight, Turn::kRight, Turn::kNone, Turn::kNone},   // 12
    {Turn::kRight, Turn::kStraight, Turn::kLeft, Turn::kNone, Turn::kNone},   // 13
    {Turn::kLeft, Turn::kStraight, Turn::kLeft, Turn::kNone, Turn::kNone},    // 14
    {Turn::kRight, Turn::kStraight, Turn::kRight, Turn::kNone, Turn::kNone},  // 15
    {Turn::kLeft, Turn::kRight, Turn::kStraight, Turn::kLeft, Turn::kRight},  // 16
    {Turn::kRight, Turn::kLeft, Turn::kStraight, Turn::kRight, Turn::kLeft},  // 17
};

RawPath makeRawPath(int type_index, double a, double b = 0.0, double c = 0.0, double d = 0.0,
                    double e = 0.0) {
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
// 论文中的 9 组解析求解公式（编号对应 1990 年原论文章节）
// ---------------------------------------------------------------------------

// 公式 8.1
bool LpSpLp(double x, double y, const PhiFrame& frame, double& t, double& u, double& v) {
  polar(x - frame.sin_phi, y - 1.0 + frame.cos_phi, u, t);
  if (t >= -kZero) {
    v = math::angle_diff_rad(frame.phi, t);
    if (v >= -kZero) {
      return true;
    }
  }
  return false;
}

// 公式 8.2
bool LpSpRp(double x, double y, const PhiFrame& frame, double& t, double& u, double& v) {
  double t1, u1;
  polar(x + frame.sin_phi, y - 1.0 - frame.cos_phi, u1, t1);
  u1 = u1 * u1;
  if (u1 >= 4.0) {
    u = std::sqrt(u1 - 4.0);
    const double theta = std::atan2(2.0, u);
    t = math::normalize_angle_rad(t1 + theta);
    v = math::angle_diff_rad(t, frame.phi);
    return t >= -kZero && v >= -kZero;
  }
  return false;
}

void CSC(double x, double y, const PhiFrames& frames, RawPath& path) {
  double t, u, v, L;
  double Lmin = path.total;
  if (LpSpLp(x, y, frames.positive, t, u, v) &&
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(14, t, u, v);
    Lmin = L;
  }
  if (LpSpLp(-x, y, frames.negative, t, u, v) &&  // 时间反转（Time flip）
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(14, -t, -u, -v);
    Lmin = L;
  }
  if (LpSpLp(x, -y, frames.negative, t, u, v) &&  // 镜像反射（Reflection）
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(15, t, u, v);
    Lmin = L;
  }
  if (LpSpLp(-x, -y, frames.positive, t, u, v) &&  // 时间反转 + 镜像反射
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(15, -t, -u, -v);
    Lmin = L;
  }
  if (LpSpRp(x, y, frames.positive, t, u, v) &&
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(12, t, u, v);
    Lmin = L;
  }
  if (LpSpRp(-x, y, frames.negative, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(12, -t, -u, -v);
    Lmin = L;
  }
  if (LpSpRp(x, -y, frames.negative, t, u, v) &&  // 镜像反射
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(13, t, u, v);
    Lmin = L;
  }
  if (LpSpRp(-x, -y, frames.positive, t, u, v) &&  // 时间反转 + 镜像反射
      Lmin > std::fabs(t) + std::fabs(u) + std::fabs(v)) {
    path = makeRawPath(13, -t, -u, -v);
  }
}

// 公式 8.3 / 8.4（已修正原论文公式中的印刷笔误）
bool LpRmL(double x, double y, const PhiFrame& frame, double& t, double& u, double& v) {
  const double xi = x - frame.sin_phi, eta = y - 1.0 + frame.cos_phi;
  double u1, theta;
  polar(xi, eta, u1, theta);
  if (u1 <= 4.0) {
    u = -2.0 * std::asin(0.25 * u1);
    t = math::normalize_angle_rad(theta + 0.5 * u + kPi);
    v = math::normalize_angle_rad(frame.phi - t + u);
    return t >= -kZero && u <= kZero;
  }
  return false;
}

void CCC(double x, double y, const PhiFrames& frames, RawPath& path) {
  double t, u, v, L;
  double Lmin = path.total;
  if (LpRmL(x, y, frames.positive, t, u, v) &&
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(0, t, u, v);
    Lmin = L;
  }
  if (LpRmL(-x, y, frames.negative, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(0, -t, -u, -v);
    Lmin = L;
  }
  if (LpRmL(x, -y, frames.negative, t, u, v) &&  // 镜像反射
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(1, t, u, v);
    Lmin = L;
  }
  if (LpRmL(-x, -y, frames.positive, t, u, v) &&  // 时间反转 + 镜像反射
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(1, -t, -u, -v);
    Lmin = L;
  }

  // 终点逆向回溯变换（Reverse direction）
  const double xb = x * frames.positive.cos_phi + y * frames.positive.sin_phi;
  const double yb = x * frames.positive.sin_phi - y * frames.positive.cos_phi;
  if (LpRmL(xb, yb, frames.positive, t, u, v) &&
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(0, v, u, t);
    Lmin = L;
  }
  if (LpRmL(-xb, yb, frames.negative, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(0, -v, -u, -t);
    Lmin = L;
  }
  if (LpRmL(xb, -yb, frames.negative, t, u, v) &&  // 镜像反射
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(1, v, u, t);
    Lmin = L;
  }
  if (LpRmL(-xb, -yb, frames.positive, t, u, v) &&  // 时间反转 + 镜像反射
      Lmin > std::fabs(t) + std::fabs(u) + std::fabs(v)) {
    path = makeRawPath(1, -v, -u, -t);
  }
}

// 公式 8.7
bool LpRupLumRm(double x, double y, const PhiFrame& frame, double& t, double& u, double& v) {
  const double xi = x + frame.sin_phi, eta = y - 1.0 - frame.cos_phi;
  const double rho = 0.25 * (2.0 + std::sqrt(xi * xi + eta * eta));
  if (rho <= 1.0) {
    u = std::acos(rho);
    tauOmega(u, -u, xi, eta, frame, t, v);
    return t >= -kZero && v <= kZero;
  }
  return false;
}

// 公式 8.8
bool LpRumLumRp(double x, double y, const PhiFrame& frame, double& t, double& u, double& v) {
  const double xi = x + frame.sin_phi, eta = y - 1.0 - frame.cos_phi;
  const double rho = (20.0 - xi * xi - eta * eta) / 16.0;
  if (rho >= 0.0 && rho <= 1.0) {
    u = -std::acos(rho);
    if (u >= -0.5 * kPi) {
      tauOmega(u, u, xi, eta, frame, t, v);
      return t >= -kZero && v >= -kZero;
    }
  }
  return false;
}

void CCCC(double x, double y, const PhiFrames& frames, RawPath& path) {
  double t, u, v, L;
  double Lmin = path.total;
  if (LpRupLumRm(x, y, frames.positive, t, u, v) &&
      Lmin > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(2, t, u, -u, v);
    Lmin = L;
  }
  if (LpRupLumRm(-x, y, frames.negative, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(2, -t, -u, u, -v);
    Lmin = L;
  }
  if (LpRupLumRm(x, -y, frames.negative, t, u, v) &&  // 镜像反射
      Lmin > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(3, t, u, -u, v);
    Lmin = L;
  }
  if (LpRupLumRm(-x, -y, frames.positive, t, u, v) &&  // 时间反转 + 镜像反射
      Lmin > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(3, -t, -u, u, -v);
    Lmin = L;
  }

  if (LpRumLumRp(x, y, frames.positive, t, u, v) &&
      Lmin > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(2, t, u, u, v);
    Lmin = L;
  }
  if (LpRumLumRp(-x, y, frames.negative, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(2, -t, -u, -u, -v);
    Lmin = L;
  }
  if (LpRumLumRp(x, -y, frames.negative, t, u, v) &&  // 镜像反射
      Lmin > (L = std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(3, t, u, u, v);
    Lmin = L;
  }
  if (LpRumLumRp(-x, -y, frames.positive, t, u, v) &&  // 时间反转 + 镜像反射
      Lmin > std::fabs(t) + 2.0 * std::fabs(u) + std::fabs(v)) {
    path = makeRawPath(3, -t, -u, -u, -v);
  }
}

// 公式 8.9
bool LpRmSmLm(double x, double y, const PhiFrame& frame, double& t, double& u, double& v) {
  const double xi = x - frame.sin_phi, eta = y - 1.0 + frame.cos_phi;
  double rho, theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.0) {
    const double r = std::sqrt(rho * rho - 4.0);
    u = 2.0 - r;
    t = math::normalize_angle_rad(theta + std::atan2(r, -2.0));
    v = math::angle_diff_rad(frame.phi - 0.5 * kPi, t);
    return t >= -kZero && u <= kZero && v <= kZero;
  }
  return false;
}

// 公式 8.10
bool LpRmSmRm(double x, double y, const PhiFrame& frame, double& t, double& u, double& v) {
  const double xi = x + frame.sin_phi, eta = y - 1.0 - frame.cos_phi;
  double rho, theta;
  polar(-eta, xi, rho, theta);
  if (rho >= 2.0) {
    t = theta;
    u = 2.0 - rho;
    v = math::angle_diff_rad(t + 0.5 * kPi, frame.phi);
    return t >= -kZero && u <= kZero && v <= kZero;
  }
  return false;
}

void CCSC(double x, double y, const PhiFrames& frames, RawPath& path) {
  double t, u, v, L;
  double Lmin = path.total - 0.5 * kPi;
  if (LpRmSmLm(x, y, frames.positive, t, u, v) &&
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(4, t, -0.5 * kPi, u, v);
    Lmin = L;
  }
  if (LpRmSmLm(-x, y, frames.negative, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(4, -t, 0.5 * kPi, -u, -v);
    Lmin = L;
  }
  if (LpRmSmLm(x, -y, frames.negative, t, u, v) &&  // 镜像反射
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(5, t, -0.5 * kPi, u, v);
    Lmin = L;
  }
  if (LpRmSmLm(-x, -y, frames.positive, t, u, v) &&  // 时间反转 + 镜像反射
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(5, -t, 0.5 * kPi, -u, -v);
    Lmin = L;
  }

  if (LpRmSmRm(x, y, frames.positive, t, u, v) &&
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(8, t, -0.5 * kPi, u, v);
    Lmin = L;
  }
  if (LpRmSmRm(-x, y, frames.negative, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(8, -t, 0.5 * kPi, -u, -v);
    Lmin = L;
  }
  if (LpRmSmRm(x, -y, frames.negative, t, u, v) &&  // 镜像反射
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(9, t, -0.5 * kPi, u, v);
    Lmin = L;
  }
  if (LpRmSmRm(-x, -y, frames.positive, t, u, v) &&  // 时间反转 + 镜像反射
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(9, -t, 0.5 * kPi, -u, -v);
    Lmin = L;
  }

  // 终点逆向回溯变换
  const double xb = x * frames.positive.cos_phi + y * frames.positive.sin_phi;
  const double yb = x * frames.positive.sin_phi - y * frames.positive.cos_phi;
  if (LpRmSmLm(xb, yb, frames.positive, t, u, v) &&
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(6, v, u, -0.5 * kPi, t);
    Lmin = L;
  }
  if (LpRmSmLm(-xb, yb, frames.negative, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(6, -v, -u, 0.5 * kPi, -t);
    Lmin = L;
  }
  if (LpRmSmLm(xb, -yb, frames.negative, t, u, v) &&  // 镜像反射
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(7, v, u, -0.5 * kPi, t);
    Lmin = L;
  }
  if (LpRmSmLm(-xb, -yb, frames.positive, t, u, v) &&  // 时间反转 + 镜像反射
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(7, -v, -u, 0.5 * kPi, -t);
    Lmin = L;
  }

  if (LpRmSmRm(xb, yb, frames.positive, t, u, v) &&
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(10, v, u, -0.5 * kPi, t);
    Lmin = L;
  }
  if (LpRmSmRm(-xb, yb, frames.negative, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(10, -v, -u, 0.5 * kPi, -t);
    Lmin = L;
  }
  if (LpRmSmRm(xb, -yb, frames.negative, t, u, v) &&  // 镜像反射
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(11, v, u, -0.5 * kPi, t);
    Lmin = L;
  }
  if (LpRmSmRm(-xb, -yb, frames.positive, t, u, v) &&  // 时间反转 + 镜像反射
      Lmin > std::fabs(t) + std::fabs(u) + std::fabs(v)) {
    path = makeRawPath(11, -v, -u, 0.5 * kPi, -t);
  }
}

// 公式 8.11（已修正原论文公式中的印刷笔误）
bool LpRmSLmRp(double x, double y, const PhiFrame& frame, double& t, double& u, double& v) {
  const double xi = x + frame.sin_phi, eta = y - 1.0 - frame.cos_phi;
  double rho, theta;
  polar(xi, eta, rho, theta);
  if (rho >= 2.0) {
    u = 4.0 - std::sqrt(rho * rho - 4.0);
    if (u <= kZero) {
      t = math::normalize_angle_rad(
          std::atan2((4.0 - u) * xi - 2.0 * eta, -2.0 * xi + (u - 4.0) * eta));
      v = math::angle_diff_rad(t, frame.phi);
      return t >= -kZero && v >= -kZero;
    }
  }
  return false;
}

void CCSCC(double x, double y, const PhiFrames& frames, RawPath& path) {
  double t, u, v, L;
  double Lmin = path.total - kPi;
  if (LpRmSLmRp(x, y, frames.positive, t, u, v) &&
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(16, t, -0.5 * kPi, u, -0.5 * kPi, v);
    Lmin = L;
  }
  if (LpRmSLmRp(-x, y, frames.negative, t, u, v) &&  // 时间反转
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(16, -t, 0.5 * kPi, -u, 0.5 * kPi, -v);
    Lmin = L;
  }
  if (LpRmSLmRp(x, -y, frames.negative, t, u, v) &&  // 镜像反射
      Lmin > (L = std::fabs(t) + std::fabs(u) + std::fabs(v))) {
    path = makeRawPath(17, t, -0.5 * kPi, u, -0.5 * kPi, v);
    Lmin = L;
  }
  if (LpRmSLmRp(-x, -y, frames.positive, t, u, v) &&  // 时间反转 + 镜像反射
      Lmin > std::fabs(t) + std::fabs(u) + std::fabs(v)) {
    path = makeRawPath(17, -t, 0.5 * kPi, -u, 0.5 * kPi, -v);
  }
}

// 完整枚举 48 类 Reeds-Shepp 曲线形式，返回全局总长最短的有效路径
RawPath reedsSheppUnit(double x, double y, double phi) {
  const PhiFrames frames{make_phi_frame(phi), make_phi_frame(-phi)};
  RawPath path;
  CSC(x, y, frames, path);
  CCC(x, y, frames, path);
  CCCC(x, y, frames, path);
  CCSC(x, y, frames, path);
  CCSCC(x, y, frames, path);
  return path;
}

// ---------------------------------------------------------------------------
// 坐标转换与分段积分辅助函数
// ---------------------------------------------------------------------------

common::Pose2d normalize_pose(const common::Pose2d& pose) {
  common::Pose2d out = pose;
  out.yaw_rad = math::normalize_angle_rad(pose.yaw_rad);
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

// 沿运动分段快速推进位姿，省去中间步骤的航向角周期归一化开销
// （正余弦具备天然周期性，对外暴露接口前统一归一化一次即可）
common::Pose2d advance_unchecked(const common::Pose2d& pose, const RsSegment& segment,
                                 double ds_m) {
  const double gear = segment.direction == common::Direction::kForward ? 1.0 : -1.0;
  return math::internal::advance_arc_trig_unchecked(pose, gear * ds_m, segment.curvature_per_m,
                                                    std::cos(pose.yaw_rad), std::sin(pose.yaw_rad));
}

// 将无量纲单位半径路径还原并缩放为具有实际物理尺度的分段路径
// 剔除零长分段，并合并相邻同挡位、同曲率的分段，确保换向尖点边界清晰
// 小于 kMinNormalizedStroke（亚纳米级）的微小分段是三角闭式解析的浮点误差假象，
// 剔除后终点偏差远低于容差上限，同时避免产生虚假退化微段
constexpr double kMinNormalizedStroke = 1e-14;

RsPath build_path(const common::Pose2d& start, const common::Pose2d& goal, double radius_m,
                  const RawPath& raw, common::Pose2d& end_pose) {
  RsPath out;
  out.start = start;
  out.goal = goal;
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
    segment.direction =
        (signed_length >= 0.0) ? common::Direction::kForward : common::Direction::kReverse;
    segment.length_m = length_m;
    segment.curvature_per_m = curvature_for_turn(turn, radius_m);
    total += segment.length_m;
    if (!out.segments.empty()) {
      RsSegment& last = out.segments.back();
      if (last.direction == segment.direction && last.curvature_per_m == segment.curvature_per_m) {
        last.length_m += segment.length_m;
        continue;
      }
    }
    out.segments.push_back(segment);
  }
  out.length_m = total;

  // 沿合并后的最终分段序列精确推进一次，得到路径终点位姿
  common::Pose2d cursor = start;
  for (const RsSegment& segment : out.segments) {
    cursor = advance_unchecked(cursor, segment, segment.length_m);
  }
  cursor.yaw_rad = math::normalize_angle_rad(cursor.yaw_rad);
  end_pose = cursor;
  return out;
}

// 校验查询位姿与车辆参数，并按需返回最小转弯半径
bool validate_query(const common::Pose2d& start, const common::Pose2d& goal,
                    const common::VehicleModel& vehicle, double* radius_out) noexcept {
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
  const double radius =
      math::kinematics::min_turn_radius_m(vehicle.wheelbase_m, vehicle.max_steer_rad);
  if (!finite(radius) || !(radius > 0.0)) {
    return false;
  }
  if (radius_out != nullptr) {
    *radius_out = radius;
  }
  return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// 公共接口实现
// ---------------------------------------------------------------------------

bool is_valid_query(const common::Pose2d& start, const common::Pose2d& goal,
                    const common::VehicleModel& vehicle) noexcept {
  return validate_query(start, goal, vehicle, nullptr);
}

double min_turn_radius_m(const common::VehicleModel& vehicle) {
  if (!finite(vehicle.wheelbase_m) || !(vehicle.wheelbase_m > 0.0)) {
    throw std::invalid_argument("min_turn_radius_m: wheelbase_m must be finite and > 0");
  }
  if (!finite(vehicle.max_steer_rad) || !(vehicle.max_steer_rad > 0.0) ||
      !(vehicle.max_steer_rad < 0.5 * kPi)) {
    throw std::invalid_argument("min_turn_radius_m: max_steer_rad must be finite, > 0 and < pi/2");
  }
  const double radius =
      math::kinematics::min_turn_radius_m(vehicle.wheelbase_m, vehicle.max_steer_rad);
  if (!finite(radius) || !(radius > 0.0)) {
    throw std::invalid_argument("min_turn_radius_m: derived radius is not positive");
  }
  return radius;
}

RsPlan plan(const common::Pose2d& start, const common::Pose2d& goal,
            const common::VehicleModel& vehicle) {
  double radius = 0.0;
  if (!validate_query(start, goal, vehicle, &radius)) {
    return {common::Status::kInvalidInput,
            "invalid input: poses must be finite and the vehicle needs "
            "wheelbase_m > 0 and 0 < max_steer_rad < pi/2",
            {}};
  }
  const common::Pose2d start_norm = normalize_pose(start);
  const common::Pose2d goal_norm = normalize_pose(goal);

  // 将目标位姿变换至起点局部坐标系，并按转弯半径作无量纲化归一
  const double c = std::cos(start_norm.yaw_rad);
  const double s = std::sin(start_norm.yaw_rad);
  const double dx = goal_norm.x_m - start_norm.x_m;
  const double dy = goal_norm.y_m - start_norm.y_m;
  const double x = (c * dx + s * dy) / radius;
  const double y = (-s * dx + c * dy) / radius;
  const double phi = math::angle_diff_rad(goal_norm.yaw_rad, start_norm.yaw_rad);

  const RawPath raw = reedsSheppUnit(x, y, phi);
  common::Pose2d end_pose;
  RsPath result = build_path(start_norm, goal_norm, radius, raw, end_pose);

  // 终点闭合精确度严格校验：确保生成路径精确终止于目标位姿容差内
  const double position_error =
      std::hypot(end_pose.x_m - goal_norm.x_m, end_pose.y_m - goal_norm.y_m);
  const double yaw_error = std::fabs(math::angle_diff_rad(end_pose.yaw_rad, goal_norm.yaw_rad));
  if (!(position_error <= kEndpointPositionToleranceM) ||
      !(yaw_error <= kEndpointYawToleranceRad)) {
    return {common::Status::kOptimizationFailed,
            "endpoint verification failed: position_error_m=" + std::to_string(position_error) +
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

common::Pose2d integrate_segment(const common::Pose2d& start, const RsSegment& segment,
                                 double ds_m) {
  if (!finite(ds_m) || ds_m < 0.0) {
    throw std::invalid_argument("integrate_segment: ds_m must be finite and >= 0");
  }
  if (!finite(segment.length_m) || segment.length_m < 0.0 || !finite(segment.curvature_per_m)) {
    throw std::invalid_argument("integrate_segment: malformed segment");
  }
  if (!finite_pose(start)) {
    throw std::invalid_argument("integrate_segment: start pose must be finite");
  }
  const double limit = segment.length_m * (1.0 + 1e-12) + 1e-12;
  if (ds_m > limit) {
    throw std::invalid_argument("integrate_segment: ds_m exceeds the segment length");
  }
  const double ds = std::min(ds_m, segment.length_m);
  common::Pose2d out = advance_unchecked(start, segment, ds);
  out.yaw_rad = math::normalize_angle_rad(out.yaw_rad);
  return out;
}

}  // namespace planner::rs
