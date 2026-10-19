// 场景内容标识实现（见 content_id.hpp）：量化、规范序列化与 SHA-256。
//
// 规范序列化必须与协议 §5.1/§5.2 及 v6 夹具生成器的独立复算逐字节一致：
// 键排序、定点 9 位小数、障碍按 id/规范字节序排序。SHA-256 为 FIPS 180-4
// 标准实现（与地图组件的内部实现同构，场景库不反向依赖地图组件内部头）。

#include "planner/scene/content_id.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

namespace planner::scene {
namespace detail {

// ---------------------------------------------------------------------------
// SHA-256（FIPS 180-4）。
// ---------------------------------------------------------------------------

inline std::uint32_t rotr(std::uint32_t x, unsigned n) noexcept {
  return (x >> n) | (x << (32U - n));
}

constexpr std::array<std::uint32_t, 64> kK = {
    0x428a2f98U, 0x71374491U, 0xb5c0fbcfU, 0xe9b5dba5U, 0x3956c25bU, 0x59f111f1U,
    0x923f82a4U, 0xab1c5ed5U, 0xd807aa98U, 0x12835b01U, 0x243185beU, 0x550c7dc3U,
    0x72be5d74U, 0x80deb1feU, 0x9bdc06a7U, 0xc19bf174U, 0xe49b69c1U, 0xefbe4786U,
    0x0fc19dc6U, 0x240ca1ccU, 0x2de92c6fU, 0x4a7484aaU, 0x5cb0a9dcU, 0x76f988daU,
    0x983e5152U, 0xa831c66dU, 0xb00327c8U, 0xbf597fc7U, 0xc6e00bf3U, 0xd5a79147U,
    0x06ca6351U, 0x14292967U, 0x27b70a85U, 0x2e1b2138U, 0x4d2c6dfcU, 0x53380d13U,
    0x650a7354U, 0x766a0abbU, 0x81c2c92eU, 0x92722c85U, 0xa2bfe8a1U, 0xa81a664bU,
    0xc24b8b70U, 0xc76c51a3U, 0xd192e819U, 0xd6990624U, 0xf40e3585U, 0x106aa070U,
    0x19a4c116U, 0x1e376c08U, 0x2748774cU, 0x34b0bcb5U, 0x391c0cb3U, 0x4ed8aa4aU,
    0x5b9cca4fU, 0x682e6ff3U, 0x748f82eeU, 0x78a5636fU, 0x84c87814U, 0x8cc70208U,
    0x90befffaU, 0xa4506cebU, 0xbef9a3f7U, 0xc67178f2U};

class Sha256 {
 public:
  void update(const std::uint8_t* data, std::size_t len) {
    bit_count_ += static_cast<std::uint64_t>(len) * 8U;
    while (len > 0) {
      const std::size_t take = std::min<std::size_t>(len, 64U - buffer_len_);
      std::memcpy(buffer_.data() + buffer_len_, data, take);
      buffer_len_ += take;
      data += take;
      len -= take;
      if (buffer_len_ == 64U) {
        process_block(buffer_.data());
        buffer_len_ = 0;
      }
    }
  }

  std::string finish_hex() {
    const std::size_t pad_len =
        (buffer_len_ < 56U) ? (56U - buffer_len_) : (120U - buffer_len_);
    std::array<std::uint8_t, 128> pad{};
    pad[0] = 0x80U;
    for (std::size_t i = 0; i < 8U; ++i) {
      pad[pad_len + i] = static_cast<std::uint8_t>(bit_count_ >> (56U - 8U * i));
    }
    update(pad.data(), pad_len + 8U);

    static constexpr char kHex[] = "0123456789abcdef";
    std::string out;
    out.reserve(64);
    for (const std::uint32_t word : state_) {
      for (int shift = 28; shift >= 0; shift -= 4) {
        out.push_back(kHex[(word >> shift) & 0xFU]);
      }
    }
    return out;
  }

 private:
  void process_block(const std::uint8_t* block) {
    std::uint32_t w[64];
    for (std::size_t i = 0; i < 16U; ++i) {
      w[i] = (static_cast<std::uint32_t>(block[4U * i]) << 24U) |
             (static_cast<std::uint32_t>(block[4U * i + 1U]) << 16U) |
             (static_cast<std::uint32_t>(block[4U * i + 2U]) << 8U) |
             static_cast<std::uint32_t>(block[4U * i + 3U]);
    }
    for (std::size_t i = 16U; i < 64U; ++i) {
      const std::uint32_t s0 =
          rotr(w[i - 15U], 7U) ^ rotr(w[i - 15U], 18U) ^ (w[i - 15U] >> 3U);
      const std::uint32_t s1 =
          rotr(w[i - 2U], 17U) ^ rotr(w[i - 2U], 19U) ^ (w[i - 2U] >> 10U);
      w[i] = w[i - 16U] + s0 + w[i - 7U] + s1;
    }

    std::uint32_t a = state_[0];
    std::uint32_t b = state_[1];
    std::uint32_t c = state_[2];
    std::uint32_t d = state_[3];
    std::uint32_t e = state_[4];
    std::uint32_t f = state_[5];
    std::uint32_t g = state_[6];
    std::uint32_t h = state_[7];

    for (std::size_t i = 0; i < 64U; ++i) {
      const std::uint32_t s1 = rotr(e, 6U) ^ rotr(e, 11U) ^ rotr(e, 25U);
      const std::uint32_t ch = (e & f) ^ (~e & g);
      const std::uint32_t temp1 = h + s1 + ch + kK[i] + w[i];
      const std::uint32_t s0 = rotr(a, 2U) ^ rotr(a, 13U) ^ rotr(a, 22U);
      const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t temp2 = s0 + maj;
      h = g;
      g = f;
      f = e;
      e = d + temp1;
      d = c;
      c = b;
      b = a;
      a = temp1 + temp2;
    }

    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
    state_[4] += e;
    state_[5] += f;
    state_[6] += g;
    state_[7] += h;
  }

  std::array<std::uint32_t, 8> state_ = {0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U,
                                         0xa54ff53aU, 0x510e527fU, 0x9b05688cU,
                                         0x1f83d9abU, 0x5be0cd19U};
  std::array<std::uint8_t, 64> buffer_{};
  std::size_t buffer_len_ = 0;
  std::uint64_t bit_count_ = 0;
};

std::string sha256_hex(const std::string& bytes) {
  Sha256 hash;
  hash.update(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  return hash.finish_hex();
}

// ---------------------------------------------------------------------------
// 规范序列化辅助。
// ---------------------------------------------------------------------------

// 量化后的纳单位整数（round half away from zero）；与生成器 quantize_fixed
// 的 double 路径一致。
double rounded_nanos(double value) {
  const double scaled = value * 1e9;
  if (scaled >= 0.0) {
    return std::floor(scaled + 0.5);
  }
  return -std::floor(-scaled + 0.5);
}

// 最小 JSON 字符串转义（协议 §5.1 规则 3）。
std::string escape_json_string(const std::string& text) {
  std::string out;
  out.reserve(text.size() + 2);
  out.push_back('"');
  for (const char raw : text) {
    const unsigned char ch = static_cast<unsigned char>(raw);
    switch (ch) {
      case '"':
        out += "\\\"";
        break;
      case '\\':
        out += "\\\\";
        break;
      case '\b':
        out += "\\b";
        break;
      case '\f':
        out += "\\f";
        break;
      case '\n':
        out += "\\n";
        break;
      case '\r':
        out += "\\r";
        break;
      case '\t':
        out += "\\t";
        break;
      default:
        if (ch < 0x20U) {
          static constexpr char kHex[] = "0123456789abcdef";
          out += "\\u00";
          out.push_back(kHex[(ch >> 4) & 0xFU]);
          out.push_back(kHex[ch & 0xFU]);
        } else {
          out.push_back(raw);
        }
        break;
    }
  }
  out.push_back('"');
  return out;
}

// 障碍几何是否为量化后的轴对齐矩形（4 顶点、每条边轴对齐、四个角互异）。
bool axis_aligned_rectangle(const std::vector<common::Point2d>& vertices,
                            double* x_min, double* y_min, double* x_max,
                            double* y_max) {
  if (vertices.size() != 4U) {
    return false;
  }
  double lo_x = vertices.front().x_m;
  double hi_x = vertices.front().x_m;
  double lo_y = vertices.front().y_m;
  double hi_y = vertices.front().y_m;
  for (const common::Point2d& vertex : vertices) {
    lo_x = std::min(lo_x, vertex.x_m);
    hi_x = std::max(hi_x, vertex.x_m);
    lo_y = std::min(lo_y, vertex.y_m);
    hi_y = std::max(hi_y, vertex.y_m);
  }
  if (!(lo_x < hi_x) || !(lo_y < hi_y)) {
    return false;
  }
  bool corners[4] = {false, false, false, false};
  for (std::size_t i = 0; i < 4U; ++i) {
    const common::Point2d& a = vertices[i];
    const common::Point2d& b = vertices[(i + 1U) % 4U];
    const bool same_x = a.x_m == b.x_m;
    const bool same_y = a.y_m == b.y_m;
    if (same_x == same_y) {
      return false;  // 对角边或退化边
    }
    const bool x_at_min = a.x_m == lo_x;
    const bool x_at_max = a.x_m == hi_x;
    const bool y_at_min = a.y_m == lo_y;
    const bool y_at_max = a.y_m == hi_y;
    if ((!x_at_min && !x_at_max) || (!y_at_min && !y_at_max)) {
      return false;
    }
    const int index = (x_at_max ? 1 : 0) + (y_at_max ? 2 : 0);
    corners[index] = true;
  }
  if (!corners[0] || !corners[1] || !corners[2] || !corners[3]) {
    return false;
  }
  *x_min = lo_x;
  *y_min = lo_y;
  *x_max = hi_x;
  *y_max = hi_y;
  return true;
}

// 单个障碍的规范 JSON 文本（键排序；矩形按 rectangle 形式）。
std::string canonical_obstacle_json(const collision::Obstacle& obstacle) {
  double x_min = 0.0;
  double y_min = 0.0;
  double x_max = 0.0;
  double y_max = 0.0;
  if (axis_aligned_rectangle(obstacle.vertices, &x_min, &y_min, &x_max, &y_max)) {
    return "{\"id\":" + escape_json_string(obstacle.id) +
           ",\"type\":\"rectangle\",\"x_max_m\":" + format_scene_fixed(x_max) +
           ",\"x_min_m\":" + format_scene_fixed(x_min) +
           ",\"y_max_m\":" + format_scene_fixed(y_max) +
           ",\"y_min_m\":" + format_scene_fixed(y_min) + "}";
  }
  std::string out = "{\"id\":" + escape_json_string(obstacle.id) +
                    ",\"type\":\"convex_polygon\",\"vertices\":[";
  for (std::size_t i = 0; i < obstacle.vertices.size(); ++i) {
    if (i > 0U) {
      out.push_back(',');
    }
    out += "{\"x_m\":" + format_scene_fixed(obstacle.vertices[i].x_m) +
           ",\"y_m\":" + format_scene_fixed(obstacle.vertices[i].y_m) + "}";
  }
  out += "]}";
  return out;
}

}  // namespace detail

double quantize_scene_value(double value) noexcept {
  if (!std::isfinite(value)) {
    return value;
  }
  const double nanos = detail::rounded_nanos(value);
  if (nanos == 0.0) {
    return 0.0;  // -0 归一化
  }
  return nanos / 1e9;
}

std::string format_scene_fixed(double value) {
  if (!std::isfinite(value)) {
    return "0.000000000";
  }
  const double rounded = detail::rounded_nanos(value);
  if (rounded == 0.0) {
    return "0.000000000";
  }
  const bool negative = rounded < 0.0;
  const double magnitude = negative ? -rounded : rounded;
#if defined(__SIZEOF_INT128__)
  // 定点整数（纳单位）精确打印；__int128 覆盖远超 double 可分辨精度的范围。
  // __extension__ 抑制 -Wpedantic 对 128 位扩展的提示（GCC/Clang）。
  __extension__ using int128_t = __int128;
  const int128_t nanos = static_cast<int128_t>(magnitude);
  const int128_t whole = nanos / 1000000000;
  const int128_t frac = nanos % 1000000000;
  std::string out;
  if (negative) {
    out.push_back('-');
  }
  out += std::to_string(static_cast<long long>(whole));
  out.push_back('.');
  std::string frac_text = std::to_string(static_cast<long long>(frac));
  out.append(static_cast<std::size_t>(9) - frac_text.size(), '0');
  out += frac_text;
  return out;
#else
  // 回退：平台无 128 位整数时使用 printf 定点（|值| < 1e6 时与整数路径
  // 一致；更大值受 double 精度限制，属协议 §5.1 规则 4 的文档化边界）。
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.9f", rounded);
  std::string out(buffer);
  if (out == "-0.000000000") {
    out = "0.000000000";
  }
  return out;
#endif
}

std::string canonical_scene_input(const std::string& map_id,
                                  const std::string& map_version,
                                  const map::Bounds2d& window, double resolution_m,
                                  int width, int height,
                                  const common::Pose2d& origin,
                                  const std::vector<collision::Obstacle>& obstacles) {
  using detail::escape_json_string;
  // 障碍按 id 升序、id 相同按规范字节序（协议 §5.1 规则 5）。
  std::vector<std::pair<std::string, std::string>> entries;
  entries.reserve(obstacles.size());
  for (const collision::Obstacle& obstacle : obstacles) {
    entries.emplace_back(obstacle.id, detail::canonical_obstacle_json(obstacle));
  }
  std::sort(entries.begin(), entries.end(),
            [](const auto& lhs, const auto& rhs) {
              if (lhs.first != rhs.first) {
                return lhs.first < rhs.first;
              }
              return lhs.second < rhs.second;
            });

  std::string out = "{\"domain\":\"planner.scene.content.v1\",\"height\":";
  out += std::to_string(height);
  out += ",\"map_id\":" + escape_json_string(map_id);
  out += ",\"map_version\":" + escape_json_string(map_version);
  out += ",\"obstacles\":[";
  for (std::size_t i = 0; i < entries.size(); ++i) {
    if (i > 0U) {
      out.push_back(',');
    }
    out += entries[i].second;
  }
  out += "],\"origin\":{\"x_m\":" + format_scene_fixed(origin.x_m) +
         ",\"y_m\":" + format_scene_fixed(origin.y_m) +
         ",\"yaw_rad\":" + format_scene_fixed(origin.yaw_rad) + "}";
  out += ",\"resolution_m\":" + format_scene_fixed(resolution_m);
  out += ",\"scene_version\":\"";
  out += kSceneVersion;
  out += "\",\"width\":";
  out += std::to_string(width);
  out += ",\"window\":{\"max_x_m\":" + format_scene_fixed(window.max_x_m) +
         ",\"max_y_m\":" + format_scene_fixed(window.max_y_m) +
         ",\"min_x_m\":" + format_scene_fixed(window.min_x_m) +
         ",\"min_y_m\":" + format_scene_fixed(window.min_y_m) + "}}";
  return out;
}

std::string scene_content_id(const std::string& map_id,
                             const std::string& map_version,
                             const map::Bounds2d& window, double resolution_m,
                             int width, int height, const common::Pose2d& origin,
                             const std::vector<collision::Obstacle>& obstacles) {
  const std::string canonical = canonical_scene_input(
      map_id, map_version, window, resolution_m, width, height, origin, obstacles);
  return detail::sha256_hex(canonical);
}

}  // namespace planner::scene
