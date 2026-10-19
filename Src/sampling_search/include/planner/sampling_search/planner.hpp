#pragma once

// 适配器工厂与可用性查询（X-S4）。
//
// 每个适配器由独立 CMake 开关控制（默认：OMPL / Ch0p1k3 / Klamp't /
// vss2sn 为 ON，ai-winter 为 OFF 的 GPL-3.0 隔离目标）。未编译进本目标的
// 适配器：available_*() 返回 false，make_*() 返回 nullptr，绝不假装可用。
//
// 公共接口只返回 Planner 抽象类型，不暴露任何上游类型。

#include <memory>
#include <string>
#include <vector>

#include <planner/sampling_search/types.hpp>

namespace planner::sampling_search {

// 稳定后端标识（写入 Statistics::backend）。
inline constexpr char kBackendOmpl[] = "ompl";
inline constexpr char kBackendCh0p1k3[] = "ch0p1k3";
inline constexpr char kBackendKlampt[] = "klampt";
inline constexpr char kBackendVss2sn[] = "vss2sn";
inline constexpr char kBackendAiwinter[] = "aiwinter";

// 变体标识（写入 Statistics::variant）。
inline constexpr char kVariantRrtStar[] = "rrtstar";
inline constexpr char kVariantInformedRrtStar[] = "informed_rrtstar";

bool ompl_rrtstar_available();
bool ompl_informed_rrtstar_available();
bool ch0p1k3_rrtstar_available();
bool klampt_rrtstar_available();
bool vss2sn_rrtstar_available();
bool aiwinter_rrtstar_available();

std::unique_ptr<Planner> make_ompl_rrtstar();
std::unique_ptr<Planner> make_ompl_informed_rrtstar();
std::unique_ptr<Planner> make_ch0p1k3_rrtstar();
std::unique_ptr<Planner> make_klampt_rrtstar();
std::unique_ptr<Planner> make_vss2sn_rrtstar();
std::unique_ptr<Planner> make_aiwinter_rrtstar();

// 本构建中已编译的所有后端（Informed 变体单列；顺序稳定，便于测试）。
std::vector<std::string> available_backends();

// 已编译后端的实例工厂；未编译的适配器不出现。
std::vector<std::unique_ptr<Planner>> make_available_planners();

}  // namespace planner::sampling_search
