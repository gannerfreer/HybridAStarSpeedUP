#pragma once

// 用于地图源版本哈希的内部 SHA-256 辅助实现。仅组件内部使用，
// 不属于公开 API。

#include <cstddef>
#include <string>

namespace planner::map::detail {

// 原始字节的小写十六进制 SHA-256。
std::string sha256_hex(const std::string& bytes);

// 文件原始字节的 SHA-256。文件无法读取时抛出 std::runtime_error。
// `out_size` 非空时写入字节数。
std::string sha256_file_hex(const std::string& path, std::size_t* out_size);

}  // namespace planner::map::detail
