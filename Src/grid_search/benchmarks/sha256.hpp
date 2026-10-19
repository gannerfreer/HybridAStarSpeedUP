#pragma once

// 自包含 SHA256（X-B1 基准工具；不依赖任何外部库）。
//
// 用途：实例打包字节的内容寻址（grid_scenario_gen v1）、>10 MiB 输入的
// SHA256 寻址归档（benchmark_tiers.md §3.4）与运行证据校验。

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace planner::grid_search::bench {

// 对内存块求 SHA256，返回 64 位小写十六进制。
std::string sha256_hex(const void* data, std::size_t size);
std::string sha256_hex(const std::vector<std::uint8_t>& bytes);

// 对文件求 SHA256；读取失败时返回空字符串并（可选）置 *ok=false。
std::string sha256_file_hex(const std::string& path, bool* ok = nullptr);

}  // namespace planner::grid_search::bench
