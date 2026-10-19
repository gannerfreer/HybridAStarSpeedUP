// X-B1 基准工具公共设施实现。

#include "bench_util.hpp"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <sstream>
#include <thread>

namespace planner::grid_search::bench {

double wall_now_s() {
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

double cpu_now_s() {
  struct timespec ts {};
  if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts) != 0) {
    return 0.0;
  }
  return static_cast<double>(ts.tv_sec) +
         static_cast<double>(ts.tv_nsec) * 1e-9;
}

std::optional<std::string> read_text_file(const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    return std::nullopt;
  }
  std::ostringstream buffer;
  buffer << stream.rdbuf();
  return buffer.str();
}

bool write_text_file(const std::string& path, const std::string& content) {
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    return false;
  }
  stream.write(content.data(), static_cast<std::streamsize>(content.size()));
  return static_cast<bool>(stream);
}

bool write_binary_file(const std::string& path,
                       const std::vector<std::uint8_t>& bytes) {
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    return false;
  }
  stream.write(reinterpret_cast<const char*>(bytes.data()),
               static_cast<std::streamsize>(bytes.size()));
  return static_cast<bool>(stream);
}

std::optional<std::vector<std::uint8_t>> read_binary_file(
    const std::string& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    return std::nullopt;
  }
  stream.seekg(0, std::ios::end);
  const std::streamoff size = stream.tellg();
  if (size < 0) {
    return std::nullopt;
  }
  stream.seekg(0, std::ios::beg);
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
  if (size > 0) {
    stream.read(reinterpret_cast<char*>(bytes.data()), size);
    if (!stream) {
      return std::nullopt;
    }
  }
  return bytes;
}

std::int64_t proc_status_kb(const char* key) {
  std::ifstream stream("/proc/self/status");
  if (!stream) {
    return -1;
  }
  const std::string needle = std::string(key) + ":";
  std::string line;
  while (std::getline(stream, line)) {
    if (line.rfind(needle, 0) != 0) {
      continue;
    }
    std::istringstream values(line.substr(needle.size()));
    std::int64_t number = -1;
    values >> number;
    return number;
  }
  return -1;
}

std::int64_t vm_rss_bytes() {
  const std::int64_t kb = proc_status_kb("VmRSS");
  return kb < 0 ? 0 : kb * 1024;
}

std::int64_t vm_hwm_bytes() {
  const std::int64_t kb = proc_status_kb("VmHWM");
  return kb < 0 ? 0 : kb * 1024;
}

struct RssSampler::Impl {
  std::atomic<bool> stop{false};
  std::thread worker;
};

RssSampler::RssSampler(int interval_ms)
    : interval_ms_(interval_ms < 1 ? 1 : interval_ms),
      impl_(new Impl()) {}

RssSampler::~RssSampler() {
  stop();
  delete impl_;
}

void RssSampler::start() {
  if (running_) {
    return;
  }
  running_ = true;
  peak_ = vm_rss_bytes();
  impl_->stop.store(false);
  impl_->worker = std::thread([this]() {
    while (!impl_->stop.load()) {
      const std::int64_t rss = vm_rss_bytes();
      if (rss > peak_) {
        peak_ = rss;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms_));
    }
  });
}

std::int64_t RssSampler::stop() {
  if (!running_) {
    return peak_;
  }
  impl_->stop.store(true);
  if (impl_->worker.joinable()) {
    impl_->worker.join();
  }
  running_ = false;
  const std::int64_t rss = vm_rss_bytes();
  if (rss > peak_) {
    peak_ = rss;
  }
  return peak_;
}

std::int64_t RssSampler::peak_bytes() const { return peak_; }

std::string env_string(const char* name, const std::string& fallback) {
  const char* value = std::getenv(name);
  if (value == nullptr || value[0] == '\0') {
    return fallback;
  }
  return std::string(value);
}

std::vector<std::string> split_csv(const std::string& text) {
  std::vector<std::string> items;
  std::size_t start = 0;
  while (start <= text.size()) {
    const std::size_t comma = text.find(',', start);
    const std::size_t end = comma == std::string::npos ? text.size() : comma;
    std::string item = trim(text.substr(start, end - start));
    if (!item.empty()) {
      items.push_back(item);
    }
    if (comma == std::string::npos) {
      break;
    }
    start = comma + 1;
  }
  return items;
}

std::string join_csv(const std::vector<std::string>& items) {
  std::string text;
  for (std::size_t index = 0; index < items.size(); ++index) {
    if (index > 0) {
      text += ",";
    }
    text += items[index];
  }
  return text;
}

std::string trim(const std::string& text) {
  const std::size_t begin = text.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return std::string();
  }
  const std::size_t end = text.find_last_not_of(" \t\r\n");
  return text.substr(begin, end - begin + 1);
}

std::string fmt_double(double value, int precision) {
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*f", precision, value);
  return std::string(buffer);
}

std::string fmt_int(std::int64_t value) { return std::to_string(value); }

std::string opt_double(const std::optional<double>& value, int precision) {
  return value.has_value() ? fmt_double(*value, precision) : std::string();
}

std::string opt_int(const std::optional<std::int64_t>& value) {
  return value.has_value() ? fmt_int(*value) : std::string();
}

std::string utc_now_iso8601() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
  std::tm utc {};
  gmtime_r(&seconds, &utc);
  const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(
                          now.time_since_epoch()) %
                      1000;
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
                utc.tm_year + 1900, utc.tm_mon + 1, utc.tm_mday, utc.tm_hour,
                utc.tm_min, utc.tm_sec, static_cast<int>(millis.count()));
  return std::string(buffer);
}

std::string csv_escape(const std::string& field) {
  const bool needs_quotes =
      field.find_first_of(",\"\n\r") != std::string::npos;
  if (!needs_quotes) {
    return field;
  }
  std::string escaped = "\"";
  for (char character : field) {
    if (character == '"') {
      escaped += "\"\"";
    } else {
      escaped += character;
    }
  }
  escaped += "\"";
  return escaped;
}

struct CsvWriter::Impl {
  std::ofstream stream;
};

CsvWriter::CsvWriter(const std::string& path) : impl_(new Impl()) {
  impl_->stream.open(path, std::ios::binary | std::ios::trunc);
}

CsvWriter::~CsvWriter() {
  if (impl_ != nullptr) {
    impl_->stream.flush();
    delete impl_;
  }
}

bool CsvWriter::open_ok() const { return impl_ != nullptr && impl_->stream.good(); }

void CsvWriter::write_header(const std::vector<std::string>& header) {
  write_row(header);
}

void CsvWriter::write_row(const std::vector<std::string>& fields) {
  if (impl_ == nullptr) {
    return;
  }
  for (std::size_t index = 0; index < fields.size(); ++index) {
    if (index > 0) {
      impl_->stream << ',';
    }
    impl_->stream << csv_escape(fields[index]);
  }
  impl_->stream << '\n';
}

void CsvWriter::write_row_raw(const std::string& line) {
  if (impl_ == nullptr) {
    return;
  }
  impl_->stream << line << '\n';
}

void CsvWriter::flush() {
  if (impl_ != nullptr) {
    impl_->stream.flush();
  }
}

}  // namespace planner::grid_search::bench
