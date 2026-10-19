#pragma once

// 基准自包含 SHA-256（FIPS 180-4）。仓库内已有实现（map/scene 内部），但均为
// 组件私有；基准侧自带一份以保持 benchmarks/ 自包含、不反向依赖其他组件内部头。

#include <string>

namespace planner::sampling_search::bench {

std::string bench_sha256_hex(const std::string& bytes);

}  // namespace planner::sampling_search::bench
