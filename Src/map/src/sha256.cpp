// 精简的 SHA-256（FIPS 180-4），仅用于地图源版本哈希。
// 标准实现，无外部依赖。

#include "sha256.hpp"

#include <array>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace planner::map::detail {
namespace {

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
    // 填充：0x80、若干 0 字节，以及 64 位大端序的比特数。
    const std::size_t pad_len = (buffer_len_ < 56U) ? (56U - buffer_len_) : (120U - buffer_len_);
    std::array<std::uint8_t, 128> pad{};
    pad[0] = 0x80U;
    for (std::size_t i = 0; i < 8U; ++i) {
      pad[pad_len + i] = static_cast<std::uint8_t>(bit_count_ >> (56U - 8U * i));
    }
    update(pad.data(), pad_len + 8U);
    // update() 增加了 bit_count_；此时 state_ 中保存的值已是最终结果。

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
      const std::uint32_t s0 = rotr(w[i - 15U], 7U) ^ rotr(w[i - 15U], 18U) ^ (w[i - 15U] >> 3U);
      const std::uint32_t s1 = rotr(w[i - 2U], 17U) ^ rotr(w[i - 2U], 19U) ^ (w[i - 2U] >> 10U);
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

  std::array<std::uint32_t, 8> state_ = {0x6a09e667U, 0xbb67ae85U, 0x3c6ef372U, 0xa54ff53aU,
                                         0x510e527fU, 0x9b05688cU, 0x1f83d9abU, 0x5be0cd19U};
  std::array<std::uint8_t, 64> buffer_{};
  std::size_t buffer_len_ = 0;
  std::uint64_t bit_count_ = 0;
};

}  // namespace

std::string sha256_hex(const std::string& bytes) {
  Sha256 h;
  h.update(reinterpret_cast<const std::uint8_t*>(bytes.data()), bytes.size());
  return h.finish_hex();
}

std::string sha256_file_hex(const std::string& path, std::size_t* out_size) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("cannot open file for hashing: " + path);
  }
  Sha256 h;
  std::vector<char> chunk(1U << 16);
  std::size_t total = 0;
  while (in) {
    in.read(chunk.data(), static_cast<std::streamsize>(chunk.size()));
    const std::streamsize got = in.gcount();
    if (got > 0) {
      h.update(reinterpret_cast<const std::uint8_t*>(chunk.data()),
               static_cast<std::size_t>(got));
      total += static_cast<std::size_t>(got);
    }
  }
  if (!in.eof()) {
    throw std::runtime_error("read error while hashing: " + path);
  }
  if (out_size != nullptr) {
    *out_size = total;
  }
  return h.finish_hex();
}

}  // namespace planner::map::detail
