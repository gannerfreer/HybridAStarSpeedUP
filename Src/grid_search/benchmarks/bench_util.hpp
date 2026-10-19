#pragma once

// X-B1 基准工具公共设施：计时、RSS 采样、文件与 CSV 工具。
//
// 口径（benchmark_tiers.md §5、组件开发计划 §7.3）：
//   * wall_ms = 端到端墙钟（含后端内部转换；适配器无独立准备接口，偏差登记）；
//   * cpu_ms = 进程 CPU 时间（CLOCK_PROCESS_CPUTIME_ID）；
//   * peak_rss_bytes = 进程 RSS 采样峰值（1 ms 间隔，标注为采样估计）；
//   * 输入生成、解码与磁盘 I/O 在计时外，单列 conversion_ms。

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace planner::grid_search::bench {

// ---------------------------------------------------------------------------
// 计时
// ---------------------------------------------------------------------------
double wall_now_s();  // 单调墙钟（steady_clock）
double cpu_now_s();   // 进程 CPU 时间

// ---------------------------------------------------------------------------
// 文件
// ---------------------------------------------------------------------------
std::optional<std::string> read_text_file(const std::string& path);
bool write_text_file(const std::string& path, const std::string& content);
bool write_binary_file(const std::string& path,
                       const std::vector<std::uint8_t>& bytes);
std::optional<std::vector<std::uint8_t>> read_binary_file(
    const std::string& path);

// ---------------------------------------------------------------------------
// RSS（/proc/self/status）
// ---------------------------------------------------------------------------
std::int64_t proc_status_kb(const char* key);  // 缺失返回 -1
std::int64_t vm_rss_bytes();
std::int64_t vm_hwm_bytes();

// 后台采样线程：每 interval_ms 读取一次 VmRSS，记录峰值。
// 采样值为估计（可能错过短峰值）；短于采样间隔的搜索其峰值≈基线。
class RssSampler {
 public:
  explicit RssSampler(int interval_ms = 1);
  ~RssSampler();
  RssSampler(const RssSampler&) = delete;
  RssSampler& operator=(const RssSampler&) = delete;

  void start();
  std::int64_t stop();  // 返回采样峰值字节（不可用为 0）
  std::int64_t peak_bytes() const;

 private:
  int interval_ms_;
  std::int64_t peak_{0};
  bool running_{false};
  struct Impl;
  Impl* impl_{nullptr};
};

// ---------------------------------------------------------------------------
// 环境与字符串
// ---------------------------------------------------------------------------
std::string env_string(const char* name, const std::string& fallback);

std::vector<std::string> split_csv(const std::string& text);
std::string join_csv(const std::vector<std::string>& items);
std::string trim(const std::string& text);

std::string fmt_double(double value, int precision = 6);
std::string fmt_int(std::int64_t value);
std::string opt_double(const std::optional<double>& value, int precision = 6);
std::string opt_int(const std::optional<std::int64_t>& value);

std::string utc_now_iso8601();

// ---------------------------------------------------------------------------
// CSV
// ---------------------------------------------------------------------------
std::string csv_escape(const std::string& field);

class CsvWriter {
 public:
  explicit CsvWriter(const std::string& path);
  ~CsvWriter();
  CsvWriter(const CsvWriter&) = delete;
  CsvWriter& operator=(const CsvWriter&) = delete;

  bool open_ok() const;
  void write_header(const std::vector<std::string>& header);
  void write_row(const std::vector<std::string>& fields);
  // 直接写入一行已转义的原始文本（用于合并工作进程 CSV，跳过其表头）。
  void write_row_raw(const std::string& line);
  void flush();

 private:
  struct Impl;
  Impl* impl_{nullptr};
};

}  // namespace planner::grid_search::bench
