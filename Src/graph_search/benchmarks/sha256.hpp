// planner_graph_search 基准内部 SHA-256 辅助（X-B1）。
//
// 仅用于基准工具：实例文件内容哈希、>10 MiB 输入的 SHA256 寻址归档校验
// （benchmark_tiers.md §3.4 / §9）。不是公开 API，也不参与算法测量。
// 实现为独立的小型标准 SHA-256（FIPS 180-4），不依赖上游库。

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace planner {
namespace graph_search {
namespace bench {

// 字节串的小写十六进制 SHA-256。
std::string sha256_hex(const std::string& bytes);

// 文件原始字节的 SHA-256。成功返回哈希并把字节数写入 *size_out；
// 文件不可读时返回空串并把原因写入 *error（两者可空）。
std::string sha256_file_hex(const std::string& path, std::size_t* size_out,
                            std::string* error);

// 流式 SHA-256（大文件避免整体载入内存）。
class Sha256 {
 public:
  Sha256();
  void update(const void* data, std::size_t size);
  std::string hex_digest();

 private:
  void process_block(const std::uint8_t* block);
  std::uint32_t state_[8];
  std::uint64_t bit_count_;
  std::uint8_t buffer_[64];
  std::size_t buffer_size_;
};

}  // namespace bench
}  // namespace graph_search
}  // namespace planner
