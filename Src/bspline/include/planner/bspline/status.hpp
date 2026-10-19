#pragma once

// planner_bspline 组件（B-spline 曲线）的状态码。
//
// 每个可能拒绝输入或无法满足数值契约的公开操作都会报告其中一个值；
// 调用方不得通过解析消息字符串来区分结果。

namespace planner::bspline {

enum class Status {
  kSuccess = 0,
  // 构造输入被拒绝：节点向量格式错误、存在非有限值、
  // 控制点过少、参数域为空。
  kInvalidInput,
  // 求值参数超出显式参数域；本库绝不隐式外推。
  kParameterOutOfDomain,
  // 求导阶数超出第一版支持的 0..3 范围。
  kDerivativeOrderUnsupported,
  // 对零长度曲线（退化几何）进行弧长重采样。
  kDegenerateCurve,
  // 弧长重采样已完成，但实测区间误差超出请求的容差。
  kArcLengthToleranceExceeded,
  // 被封装的内部后端出现意外失败；曲线仍然有效。
  kInternalError,
};

// 供测试、示例与诊断使用的稳定短名称。
const char* to_string(Status status) noexcept;

}  // namespace planner::bspline
