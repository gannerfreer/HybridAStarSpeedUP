#pragma once

// 场景内容标识（栅格快照协议 v1 §5，X-M1）：量化、规范序列化与 SHA-256。
//
// 协议 §5.1 冻结的规范输入（§5.2 字段集合）序列化后取 SHA-256，输出 64 位
// 小写十六进制。地图服务预览与规划服务场景构建必须对同一有效输入产生相同
// scene_id，因此计算统一在场景库执行；两个服务只消费本入口。
//
// 实现规则（与协议 §5 及 v6 夹具生成器 tools/grid_fixtures.py 的独立复算
// 一致）：
//   * 所有实数先四舍五入（round half away from zero）到 1e-9 的整数倍，
//     再以定点格式输出（恰好 9 位小数、无指数、-0 归一化为 0）；
//   * 对象键按 Unicode 码点升序递归排序；无多余空白；
//   * 字符串最小 JSON 转义，其余按 UTF-8 原样输出；
//   * 障碍按 id 升序、id 相同按规范字节序排列。
//
// 障碍类型说明：collision::Obstacle 只携带规范化的凸多边形顶点（矩形由
// `Obstacle::rectangle` 展开为 4 个轴对齐顶点，多边形归一化为逆时针）。
// 规范序列化因此按几何归一化：4 个顶点且边轴对齐（量化后精确）的障碍按
// "rectangle" 形式输出（x_min/x_max/y_min/y_max 取顶点包围盒），其余按
// "convex_polygon" 形式输出（顶点顺序沿用 collision 的规范化顺序）。两个
// 服务使用同一函数，跨服务一致性由本规则保证；矩形与等价 4 顶点多边形
// 共享同一标识属文档化边界（协议 §5 审阅点，见 X-M1 报告）。

#include <string>
#include <vector>

#include "planner/collision/scene.hpp"
#include "planner/common/types.hpp"
#include "planner/map/types.hpp"

namespace planner::scene {

// 场景组合规则版本（协议 §7；scene_id 的哈希输入）。
inline constexpr const char* kSceneVersion = "scene_v1";

// 四舍五入（round half away from zero）到 1e-9 的整数倍，返回 double。
// 非有限输入原样返回（调用方必须先校验有限性）。
double quantize_scene_value(double value) noexcept;

// 定点格式：恰好 9 位小数，无指数、无前导 '+'，-0.000000000 归一化为
// 0.000000000。非有限输入返回 "0.000000000"（调用方保证有限）。
std::string format_scene_fixed(double value);

// 协议 §5.2 规范输入对象的规范 JSON 文本（键排序、定点数字、障碍排序）。
// 调用方传入的数值应已经是有效（量化后）输入；本函数按规则再量化一次，
// 因此与先量化再调用等价。
std::string canonical_scene_input(const std::string& map_id,
                                  const std::string& map_version,
                                  const map::Bounds2d& window,
                                  double resolution_m, int width, int height,
                                  const common::Pose2d& origin,
                                  const std::vector<collision::Obstacle>& obstacles);

// scene_id = SHA-256(canonical_scene_input(...))，64 位小写十六进制。
std::string scene_content_id(const std::string& map_id,
                             const std::string& map_version,
                             const map::Bounds2d& window, double resolution_m,
                             int width, int height, const common::Pose2d& origin,
                             const std::vector<collision::Obstacle>& obstacles);

}  // namespace planner::scene
