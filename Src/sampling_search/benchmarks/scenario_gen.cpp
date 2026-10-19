// X-B1 采样搜索基准场景生成器实现（规则见 scenario_gen.hpp 文件头）。
//
// 确定性：SplitMix64 伪随机流由固定几何种子派生；所有几何逐位可复现。
// 规范 JSON 使用 nlohmann::json 的对象（std::map，键有序）与 dump()。

#include "scenario_gen.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <queue>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include <planner/sampling_search/validity.hpp>

#include "sha256.hpp"

namespace planner::sampling_search::bench {
namespace {

// ---------------------------------------------------------------------------
// 确定性伪随机（SplitMix64）与工具
// ---------------------------------------------------------------------------

class SplitMix64 {
 public:
  explicit SplitMix64(std::uint64_t seed) : state_(seed) {}

  std::uint64_t next() {
    state_ += 0x9E3779B97F4A7C15ULL;
    std::uint64_t z = state_;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    return z ^ (z >> 31);
  }

  // [a, b] 均匀（端点理论可达）。
  double uniform(double a, double b) {
    const double u = static_cast<double>(next() >> 11) *
                     (1.0 / 9007199254740992.0);  // 2^53
    return a + (b - a) * u;
  }

  int uniform_int(int a, int b) {  // [a, b]
    const std::uint64_t span = static_cast<std::uint64_t>(b - a + 1);
    return a + static_cast<int>(next() % span);
  }

  // Fisher–Yates 洗牌。
  template <typename T>
  void shuffle(std::vector<T>* items) {
    for (std::size_t i = items->size(); i > 1; --i) {
      const std::size_t j = static_cast<std::size_t>(next() % i);
      std::swap((*items)[i - 1], (*items)[j]);
    }
  }

 private:
  std::uint64_t state_;
};

const char kBase64Alphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

std::string base64_encode(const std::vector<std::uint8_t>& bytes) {
  std::string out;
  out.reserve((bytes.size() + 2) / 3 * 4);
  std::size_t i = 0;
  while (i + 3 <= bytes.size()) {
    const std::uint32_t triple = (static_cast<std::uint32_t>(bytes[i]) << 16) |
                                 (static_cast<std::uint32_t>(bytes[i + 1]) << 8) |
                                 static_cast<std::uint32_t>(bytes[i + 2]);
    out.push_back(kBase64Alphabet[(triple >> 18) & 0x3F]);
    out.push_back(kBase64Alphabet[(triple >> 12) & 0x3F]);
    out.push_back(kBase64Alphabet[(triple >> 6) & 0x3F]);
    out.push_back(kBase64Alphabet[triple & 0x3F]);
    i += 3;
  }
  const std::size_t remaining = bytes.size() - i;
  if (remaining == 1) {
    const std::uint32_t triple = static_cast<std::uint32_t>(bytes[i]) << 16;
    out.push_back(kBase64Alphabet[(triple >> 18) & 0x3F]);
    out.push_back(kBase64Alphabet[(triple >> 12) & 0x3F]);
    out.push_back('=');
    out.push_back('=');
  } else if (remaining == 2) {
    const std::uint32_t triple = (static_cast<std::uint32_t>(bytes[i]) << 16) |
                                 (static_cast<std::uint32_t>(bytes[i + 1]) << 8);
    out.push_back(kBase64Alphabet[(triple >> 18) & 0x3F]);
    out.push_back(kBase64Alphabet[(triple >> 12) & 0x3F]);
    out.push_back(kBase64Alphabet[(triple >> 6) & 0x3F]);
    out.push_back('=');
  }
  return out;
}

bool segment_intersects_rect(const Point2d& a, const Point2d& b,
                             const RectObstacle& r) {
  // Liang–Barsky 裁剪；闭区间（相切视为相交，保守）。
  double t0 = 0.0;
  double t1 = 1.0;
  const double dx = b.x_m - a.x_m;
  const double dy = b.y_m - a.y_m;
  const auto clip = [&](double p, double q) -> bool {
    if (std::fabs(p) < 1e-15) {
      return q >= 0.0;
    }
    const double t = q / p;
    if (p < 0.0) {
      if (t > t1) {
        return false;
      }
      if (t > t0) {
        t0 = t;
      }
    } else {
      if (t < t0) {
        return false;
      }
      if (t < t1) {
        t1 = t;
      }
    }
    return true;
  };
  if (!clip(-dx, a.x_m - r.x0)) return false;
  if (!clip(dx, r.x1 - a.x_m)) return false;
  if (!clip(-dy, a.y_m - r.y0)) return false;
  if (!clip(dy, r.y1 - a.y_m)) return false;
  return t0 <= t1;
}

bool rect_near_point(const RectObstacle& r, const Point2d& p, double margin) {
  return p.x_m >= r.x0 - margin && p.x_m <= r.x1 + margin &&
         p.y_m >= r.y0 - margin && p.y_m <= r.y1 + margin;
}

bool point_in_any_rect(const std::vector<RectObstacle>& rects,
                       const Point2d& p) {
  for (const RectObstacle& r : rects) {
    if (p.x_m >= r.x0 && p.x_m <= r.x1 && p.y_m >= r.y0 && p.y_m <= r.y1) {
      return true;
    }
  }
  return false;
}

// 保守粗网格连通检查：cell 网格上，保守模式要求单元 9 个采样点全部有效
// （全部有效才视为自由，保证“粗网格可达 ⇒ 连续空间可达”）；非保守模式仅
// 检查单元中心。4 邻域 BFS。
bool coarse_connected(double size, const std::vector<RectObstacle>& rects,
                      const Point2d& start, const Point2d& goal, double cell,
                      bool conservative) {
  const int n = static_cast<int>(std::llround(size / cell));
  if (n <= 0 || std::fabs(size / cell - n) > 1e-9) {
    return false;
  }
  const auto free_cell = [&](int row, int col) {
    const double x0 = col * cell;
    const double y0 = row * cell;
    if (conservative) {
      // 9 点（角/边中点/中心）：全部有效才视为自由（自由单元 ⊆ 无障区域）。
      for (int di = 0; di <= 2; ++di) {
        for (int dj = 0; dj <= 2; ++dj) {
          const Point2d p{x0 + 0.5 * cell * dj, y0 + 0.5 * cell * di};
          if (point_in_any_rect(rects, p)) {
            return false;
          }
        }
      }
      return true;
    }
    const Point2d center{x0 + 0.5 * cell, y0 + 0.5 * cell};
    return !point_in_any_rect(rects, center);
  };
  const auto to_cell = [&](const Point2d& p) {
    return std::pair<int, int>{static_cast<int>(std::floor(p.y_m / cell)),
                               static_cast<int>(std::floor(p.x_m / cell))};
  };
  const auto start_cell = to_cell(start);
  const auto goal_cell = to_cell(goal);
  for (const auto& c : {start_cell, goal_cell}) {
    if (c.first < 0 || c.first >= n || c.second < 0 || c.second >= n ||
        !free_cell(c.first, c.second)) {
      return false;
    }
  }
  std::vector<std::uint8_t> visited(static_cast<std::size_t>(n) * n, 0);
  std::queue<std::pair<int, int>> frontier;
  frontier.push(start_cell);
  visited[static_cast<std::size_t>(start_cell.first) * n + start_cell.second] = 1;
  const int dr[4] = {-1, 1, 0, 0};
  const int dc[4] = {0, 0, -1, 1};
  while (!frontier.empty()) {
    const auto current = frontier.front();
    frontier.pop();
    if (current == goal_cell) {
      return true;
    }
    for (int k = 0; k < 4; ++k) {
      const int nr = current.first + dr[k];
      const int nc = current.second + dc[k];
      if (nr < 0 || nr >= n || nc < 0 || nc >= n) {
        continue;
      }
      const std::size_t index = static_cast<std::size_t>(nr) * n + nc;
      if (visited[index] != 0 || !free_cell(nr, nc)) {
        continue;
      }
      visited[index] = 1;
      frontier.push({nr, nc});
    }
  }
  return false;
}

// ---------------------------------------------------------------------------
// 连续组实例构造
// ---------------------------------------------------------------------------

ContinuousInstance make_open(double size, bool capacity,
                             std::uint64_t geometry_seed) {
  ContinuousInstance inst;
  inst.class_name = "open";
  inst.capacity = capacity;
  inst.size_m = size;
  inst.geometry_seed = geometry_seed;
  inst.bounds = Bounds2d{0.0, 0.0, size, size};
  inst.start = Point2d{0.1 * size, 0.1 * size};
  inst.goal = Point2d{0.9 * size, 0.9 * size};
  inst.edge_check_step = 0.1;
  return inst;
}

// 稀疏障碍：count 个矩形，边长 U[5,12] m，总面积占边界面积 8%–12%，
// 起终点 1.0 m 净空；保守 1 m 粗网格 BFS 必须可达（否则换派生重试）。
ContinuousInstance make_sparse(double size, int count, bool capacity,
                               std::uint64_t geometry_seed) {
  ContinuousInstance inst = make_open(size, capacity, geometry_seed);
  inst.class_name = "sparse";
  const Point2d start = inst.start;
  const Point2d goal = inst.goal;
  const double target_lo = 0.08 * size * size;
  const double target_hi = 0.12 * size * size;
  for (int attempt = 0; attempt < 128; ++attempt) {
    SplitMix64 rng(geometry_seed * 1000003ULL + 101ULL * attempt + 7ULL);
    std::vector<RectObstacle> rects;
    rects.reserve(static_cast<std::size_t>(count));
    double area = 0.0;
    int draws = 0;
    while (static_cast<int>(rects.size()) < count && draws < count * 40) {
      ++draws;
      const double w = rng.uniform(5.0, 12.0);
      const double h = rng.uniform(5.0, 12.0);
      const double x0 = rng.uniform(0.04 * size, 0.96 * size - w);
      const double y0 = rng.uniform(0.04 * size, 0.96 * size - h);
      const RectObstacle r{x0, y0, x0 + w, y0 + h};
      if (rect_near_point(r, start, 1.0) || rect_near_point(r, goal, 1.0)) {
        continue;
      }
      rects.push_back(r);
      area += w * h;
    }
    if (static_cast<int>(rects.size()) != count) {
      continue;
    }
    if (area < target_lo || area > target_hi) {
      continue;
    }
    if (!coarse_connected(size, rects, start, goal, 1.0, true)) {
      continue;
    }
    inst.obstacles = std::move(rects);
    return inst;
  }
  throw std::runtime_error(
      "sparse scenario generation failed after 128 deterministic attempts "
      "(size=" +
      std::to_string(size) + " m, count=" + std::to_string(count) +
      ", geometry_seed=" + std::to_string(geometry_seed) + ")");
}

// 窄道：两道平行墙（厚 3 m）形成 2.0 m 宽通道，通道长 0.6×size；
// 两墙分别贴左/右边界（经边界墙），起终点在通道内部两端。
ContinuousInstance make_narrow(double size, bool capacity,
                               std::uint64_t geometry_seed) {
  ContinuousInstance inst = make_open(size, capacity, geometry_seed);
  inst.class_name = "narrow";
  const double corridor_lo = 0.2 * size;
  const double corridor_hi = 0.8 * size;
  const double center_y = 0.5 * size;
  const double gap = 2.0;
  const double thickness = 3.0;
  const double bottom_hi = center_y - 0.5 * gap;
  const double top_lo = center_y + 0.5 * gap;
  inst.obstacles = {
      RectObstacle{corridor_lo, bottom_hi - thickness, corridor_hi, bottom_hi},
      RectObstacle{corridor_lo, top_lo, corridor_hi, top_lo + thickness}};
  inst.start = Point2d{corridor_lo + 1.0, center_y};
  inst.goal = Point2d{corridor_hi - 1.0, center_y};
  return inst;
}

// 陷阱：U 形/带 2.0 m 开口的腔体（壁厚 3 m），外部 → 内部。
ContinuousInstance make_trap(double size, bool capacity,
                             std::uint64_t geometry_seed) {
  ContinuousInstance inst = make_open(size, capacity, geometry_seed);
  inst.class_name = "trap";
  const double lo = 0.4 * size;
  const double hi = 0.6 * size;
  const double thickness = 3.0;
  const double mouth = 2.0;
  const double center = 0.5 * size;
  const double mouth_lo = center - 0.5 * mouth;
  const double mouth_hi = center + 0.5 * mouth;
  inst.obstacles = {
      RectObstacle{lo, lo, hi, lo + thickness},                  // 底
      RectObstacle{lo, lo + thickness, lo + thickness, hi},      // 左
      RectObstacle{hi - thickness, lo + thickness, hi, hi},      // 右
      RectObstacle{lo, hi - thickness, mouth_lo, hi},            // 顶左残段
      RectObstacle{mouth_hi, hi - thickness, hi, hi}};           // 顶右残段
  inst.start = Point2d{0.15 * size, 0.15 * size};
  inst.goal = Point2d{center, center};
  return inst;
}

// 贴边：起点/终点距边界 0.5 m；沿底边一道通长墙形成 1.5 m 贴边通道。
ContinuousInstance make_edge(double size, bool capacity,
                             std::uint64_t geometry_seed) {
  ContinuousInstance inst = make_open(size, capacity, geometry_seed);
  inst.class_name = "edge";
  const double wall_lo = 1.5;
  const double wall_hi = 4.0;
  inst.obstacles = {RectObstacle{0.0, wall_lo, size, wall_hi}};
  inst.start = Point2d{0.1 * size, 0.5};
  inst.goal = Point2d{0.9 * size, 0.5};
  return inst;
}

// 不可达：终点在封闭矩形内（墙厚 0.2 m），外部 → 内部。
ContinuousInstance make_unreachable(double size, bool capacity,
                                    std::uint64_t geometry_seed) {
  ContinuousInstance inst = make_open(size, capacity, geometry_seed);
  inst.class_name = "unreachable";
  const double lo = 0.8 * size;
  const double hi = 0.92 * size;
  const double t = 0.2;
  inst.obstacles = {
      RectObstacle{lo, lo, hi, lo + t},              // 下壁
      RectObstacle{lo, hi - t, hi, hi},              // 上壁
      RectObstacle{lo, lo + t, lo + t, hi - t},      // 左壁
      RectObstacle{hi - t, lo + t, hi, hi - t}};     // 右壁
  inst.start = Point2d{0.1 * size, 0.1 * size};
  inst.goal = Point2d{0.86 * size, 0.86 * size};
  return inst;
}

ContinuousInstance make_continuous_by_class(const std::string& class_name,
                                            double size, bool capacity,
                                            std::uint64_t geometry_seed) {
  if (class_name == "open") {
    return make_open(size, capacity, geometry_seed);
  }
  if (class_name == "sparse") {
    return make_sparse(size, capacity ? 60 : 15, capacity, geometry_seed);
  }
  if (class_name == "narrow") {
    return make_narrow(size, capacity, geometry_seed);
  }
  if (class_name == "trap") {
    return make_trap(size, capacity, geometry_seed);
  }
  if (class_name == "edge") {
    return make_edge(size, capacity, geometry_seed);
  }
  if (class_name == "unreachable") {
    return make_unreachable(size, capacity, geometry_seed);
  }
  throw std::runtime_error("unknown continuous class: " + class_name);
}

// ---------------------------------------------------------------------------
// 栅格组实例构造（sampling_search_grid_gen v1，暂定；规则按 §2.2）
// ---------------------------------------------------------------------------

void add_border(GridInstance* inst) {
  const int n = inst->size;
  for (int i = 0; i < n; ++i) {
    inst->cells[static_cast<std::size_t>(0) * n + i] = 1;
    inst->cells[static_cast<std::size_t>(n - 1) * n + i] = 1;
    inst->cells[static_cast<std::size_t>(i) * n + 0] = 1;
    inst->cells[static_cast<std::size_t>(i) * n + (n - 1)] = 1;
  }
}

void fill_rect(GridInstance* inst, int row0, int col0, int row1, int col1) {
  for (int row = row0; row <= row1; ++row) {
    for (int col = col0; col <= col1; ++col) {
      inst->cells[static_cast<std::size_t>(row) * inst->size + col] = 1;
    }
  }
}

GridInstance make_grid_open(int n, bool capacity, std::uint64_t geometry_seed) {
  GridInstance inst;
  inst.class_name = "open";
  inst.capacity = capacity;
  inst.size = n;
  inst.geometry_seed = geometry_seed;
  inst.cells.assign(static_cast<std::size_t>(n) * n, 0);
  add_border(&inst);
  inst.start_row = n / 8;
  inst.start_col = n / 8;
  inst.goal_row = n - 1 - n / 8;
  inst.goal_col = n - 1 - n / 8;
  return inst;
}

// 迷宫：房间间距 P = max(8, N/16)，墙厚 2，门洞宽 3；房间生成树（随机 DFS）
// 保证连通；起点/终点为左下/右上房间中心。
GridInstance make_grid_maze(int n, bool capacity, std::uint64_t geometry_seed) {
  GridInstance inst;
  inst.class_name = "maze";
  inst.capacity = capacity;
  inst.size = n;
  inst.geometry_seed = geometry_seed;
  inst.cells.assign(static_cast<std::size_t>(n) * n, 0);
  add_border(&inst);
  const int pitch = std::max(8, n / 16);
  if (n % pitch != 0) {
    throw std::runtime_error("maze: size not divisible by room pitch");
  }
  const int rooms = n / pitch;
  // 墙带：房间 r 内部行 [1+r*P, r*P+P-2]；带 [r*P+P-1, r*P+P]。
  for (int r = 0; r + 1 < rooms; ++r) {
    fill_rect(&inst, r * pitch + pitch - 1, 1, r * pitch + pitch, n - 2);
  }
  for (int c = 0; c + 1 < rooms; ++c) {
    fill_rect(&inst, 1, c * pitch + pitch - 1, n - 2, c * pitch + pitch);
  }
  const auto room_center = [&](int r, int c) {
    const int row = 1 + r * pitch + (pitch - 2) / 2;
    const int col = 1 + c * pitch + (pitch - 2) / 2;
    return std::pair<int, int>{row, col};
  };
  const auto carve_door = [&](int r, int c, int nr, int nc) {
    const auto center = room_center(r, c);
    if (nr == r) {  // 水平相邻：竖直墙带
      const int w0 = std::min(c, nc) * pitch + pitch - 1;
      for (int row = center.first - 1; row <= center.first + 1; ++row) {
        for (int col = w0; col <= w0 + 1; ++col) {
          inst.cells[static_cast<std::size_t>(row) * n + col] = 0;
        }
      }
    } else {  // 垂直相邻：水平墙带
      const int w0 = std::min(r, nr) * pitch + pitch - 1;
      for (int row = w0; row <= w0 + 1; ++row) {
        for (int col = center.second - 1; col <= center.second + 1; ++col) {
          inst.cells[static_cast<std::size_t>(row) * n + col] = 0;
        }
      }
    }
  };
  SplitMix64 rng(geometry_seed * 7919ULL + 13ULL);
  std::vector<std::uint8_t> visited(static_cast<std::size_t>(rooms) * rooms, 0);
  std::vector<std::pair<int, int>> stack;
  stack.push_back({0, 0});
  visited[0] = 1;
  const int dr[4] = {-1, 1, 0, 0};
  const int dc[4] = {0, 0, -1, 1};
  while (!stack.empty()) {
    const auto current = stack.back();
    std::vector<int> directions = {0, 1, 2, 3};
    rng.shuffle(&directions);
    bool advanced = false;
    for (const int k : directions) {
      const int nr = current.first + dr[k];
      const int nc = current.second + dc[k];
      if (nr < 0 || nr >= rooms || nc < 0 || nc >= rooms) {
        continue;
      }
      if (visited[static_cast<std::size_t>(nr) * rooms + nc] != 0) {
        continue;
      }
      carve_door(current.first, current.second, nr, nc);
      visited[static_cast<std::size_t>(nr) * rooms + nc] = 1;
      stack.push_back({nr, nc});
      advanced = true;
      break;
    }
    if (!advanced) {
      stack.pop_back();
    }
  }
  const auto start = room_center(0, 0);
  const auto goal = room_center(rooms - 1, rooms - 1);
  inst.start_row = start.first;
  inst.start_col = start.second;
  inst.goal_row = goal.first;
  inst.goal_col = goal.second;
  return inst;
}

// 窄道：3 格宽通道贯穿约 75% 边长；两墙分别贴左/右边界（经边界墙）。
GridInstance make_grid_narrow(int n, bool capacity,
                              std::uint64_t geometry_seed) {
  GridInstance inst;
  inst.class_name = "narrow";
  inst.capacity = capacity;
  inst.size = n;
  inst.geometry_seed = geometry_seed;
  inst.cells.assign(static_cast<std::size_t>(n) * n, 0);
  add_border(&inst);
  const int wall_row = n / 2 - 2;
  const int span = 3 * n / 4;  // ≈75% 边长
  const int upper_hi = span;              // 上墙 cols 1..span（贴左边界）
  const int lower_lo = n - 1 - span;      // 下墙 cols (n-1-span)..n-2（贴右边界）
  fill_rect(&inst, wall_row, 1, wall_row, upper_hi);
  fill_rect(&inst, wall_row + 4, lower_lo, wall_row + 4, n - 2);
  inst.start_row = wall_row + 2;
  inst.start_col = 2;
  inst.goal_row = wall_row + 2;
  inst.goal_col = n - 3;
  return inst;
}

// 死胡同：入口宽 3、袋深 max(10, N/8)；袋外开阔区 → 袋底。
GridInstance make_grid_deadend(int n, bool capacity,
                               std::uint64_t geometry_seed) {
  GridInstance inst;
  inst.class_name = "deadend";
  inst.capacity = capacity;
  inst.size = n;
  inst.geometry_seed = geometry_seed;
  inst.cells.assign(static_cast<std::size_t>(n) * n, 0);
  add_border(&inst);
  const int left = n / 2 - 2;
  const int right = left + 4;
  const int bottom = std::max(2, n / 4);
  const int depth = std::max(10, n / 8);
  const int top = bottom + depth;
  fill_rect(&inst, bottom, left, bottom, right);
  fill_rect(&inst, bottom, left, top, left);
  fill_rect(&inst, bottom, right, top, right);
  inst.start_row = n / 4;
  inst.start_col = n / 4;
  inst.goal_row = bottom + 2;
  inst.goal_col = left + 2;
  return inst;
}

// 不可达：终点在封闭腔（墙厚 1 格、无开口）。
GridInstance make_grid_unreachable(int n, bool capacity,
                                   std::uint64_t geometry_seed) {
  GridInstance inst;
  inst.class_name = "unreachable";
  inst.capacity = capacity;
  inst.size = n;
  inst.geometry_seed = geometry_seed;
  inst.cells.assign(static_cast<std::size_t>(n) * n, 0);
  add_border(&inst);
  const int side = std::max(6, n / 16);
  const int center = n / 2;
  const int half = side / 2;
  const int lo = center - half - 1;
  const int hi = center + half + 1;
  for (int row = lo; row <= hi; ++row) {
    for (int col = lo; col <= hi; ++col) {
      if (row == lo || row == hi || col == lo || col == hi) {
        inst.cells[static_cast<std::size_t>(row) * n + col] = 1;
      }
    }
  }
  inst.start_row = n / 4;
  inst.start_col = n / 4;
  inst.goal_row = center;
  inst.goal_col = center;
  return inst;
}

// 边界斜角：沿底/左边界的 ≥3 处 1 格宽对角障碍对 + 贴右边界的 1 格通道。
GridInstance make_grid_border_corner(int n, bool capacity,
                                     std::uint64_t geometry_seed) {
  GridInstance inst;
  inst.class_name = "border_corner";
  inst.capacity = capacity;
  inst.size = n;
  inst.geometry_seed = geometry_seed;
  inst.cells.assign(static_cast<std::size_t>(n) * n, 0);
  add_border(&inst);
  const int pairs = std::max(3, n / 32);
  for (int i = 0; i < pairs; ++i) {
    const int base = 3 + 5 * i;
    inst.cells[static_cast<std::size_t>(base) * n + (base + 1)] = 1;
    inst.cells[static_cast<std::size_t>(base + 1) * n + base] = 1;
  }
  // 贴边通道：col n-3 的墙（rows 1..n-6），通道为 col n-2。
  fill_rect(&inst, 1, n - 3, n - 6, n - 3);
  inst.start_row = 2;
  inst.start_col = 2;
  inst.goal_row = n - 4;
  inst.goal_col = n - 2;
  return inst;
}

GridInstance make_grid_by_class(const std::string& class_name, int n,
                                bool capacity, std::uint64_t geometry_seed) {
  if (class_name == "open") {
    return make_grid_open(n, capacity, geometry_seed);
  }
  if (class_name == "maze") {
    return make_grid_maze(n, capacity, geometry_seed);
  }
  if (class_name == "narrow") {
    return make_grid_narrow(n, capacity, geometry_seed);
  }
  if (class_name == "deadend") {
    return make_grid_deadend(n, capacity, geometry_seed);
  }
  if (class_name == "unreachable") {
    return make_grid_unreachable(n, capacity, geometry_seed);
  }
  if (class_name == "border_corner") {
    return make_grid_border_corner(n, capacity, geometry_seed);
  }
  throw std::runtime_error("unknown grid class: " + class_name);
}

// 栅格单元级连通检查（8 邻域；用于生成器自检）。
bool grid_cells_connected(const GridInstance& inst, bool* reachable) {
  const int n = inst.size;
  const auto index = [&](int row, int col) {
    return static_cast<std::size_t>(row) * n + col;
  };
  if (inst.at(inst.start_row, inst.start_col) != 0 ||
      inst.at(inst.goal_row, inst.goal_col) != 0) {
    return false;
  }
  std::vector<std::uint8_t> visited(static_cast<std::size_t>(n) * n, 0);
  std::queue<std::pair<int, int>> frontier;
  frontier.push({inst.start_row, inst.start_col});
  visited[index(inst.start_row, inst.start_col)] = 1;
  while (!frontier.empty()) {
    const auto current = frontier.front();
    frontier.pop();
    if (current.first == inst.goal_row && current.second == inst.goal_col) {
      *reachable = true;
      return true;
    }
    for (int dr = -1; dr <= 1; ++dr) {
      for (int dc = -1; dc <= 1; ++dc) {
        if (dr == 0 && dc == 0) {
          continue;
        }
        const int nr = current.first + dr;
        const int nc = current.second + dc;
        if (nr < 0 || nr >= n || nc < 0 || nc >= n) {
          continue;
        }
        if (visited[index(nr, nc)] != 0 || inst.at(nr, nc) != 0) {
          continue;
        }
        visited[index(nr, nc)] = 1;
        frontier.push({nr, nc});
      }
    }
  }
  *reachable = false;
  return true;
}

}  // namespace

// ---------------------------------------------------------------------------
// 公共接口
// ---------------------------------------------------------------------------

std::vector<std::string> continuous_instance_ids(bool include_capacity) {
  std::vector<std::string> ids = {"cont-open-100-s0",      "cont-sparse-100-s0",
                                  "cont-narrow-100-s0",    "cont-trap-100-s0",
                                  "cont-edge-100-s0",      "cont-unreachable-100-s0"};
  if (include_capacity) {
    ids.push_back("cont-cap-open-200-s0");
    ids.push_back("cont-cap-sparse-200-s0");
  }
  return ids;
}

std::vector<std::string> grid_instance_ids(bool include_capacity) {
  static const char* kClasses[] = {"open",        "maze",       "narrow",
                                   "deadend",     "unreachable", "border_corner"};
  std::vector<std::string> ids;
  for (const int scale : {128, 512, 1024}) {
    for (const char* class_name : kClasses) {
      ids.push_back("grid-" + std::string(class_name) + "-" +
                    std::to_string(scale) + "-s0");
    }
  }
  if (include_capacity) {
    for (const int scale : {2048, 4096}) {
      for (const char* class_name : kClasses) {
        ids.push_back("grid-" + std::string(class_name) + "-" +
                      std::to_string(scale) + "-s0");
      }
    }
  }
  return ids;
}

std::string group_of(const std::string& instance_id) {
  if (instance_id.rfind("cont-cap-", 0) == 0) {
    return "capacity_continuous";
  }
  if (instance_id.rfind("cont-", 0) == 0) {
    return "continuous";
  }
  if (instance_id.rfind("grid-", 0) == 0) {
    if (instance_id.find("-2048-") != std::string::npos ||
        instance_id.find("-4096-") != std::string::npos) {
      return "capacity_grid";
    }
    return "grid";
  }
  throw std::runtime_error("unknown instance id: " + instance_id);
}

ContinuousInstance make_continuous_instance(const std::string& id) {
  // cont[-cap]-<class>-<size>-s<seed>
  std::vector<std::string> parts;
  std::string token;
  for (const char ch : id) {
    if (ch == '-') {
      parts.push_back(token);
      token.clear();
    } else {
      token.push_back(ch);
    }
  }
  parts.push_back(token);
  if (parts.size() < 4 || parts[0] != "cont") {
    throw std::runtime_error("bad continuous instance id: " + id);
  }
  bool capacity = false;
  std::size_t index = 1;
  if (parts[1] == "cap") {
    capacity = true;
    index = 2;
  }
  if (parts.size() != index + 3) {
    throw std::runtime_error("bad continuous instance id: " + id);
  }
  const std::string class_name = parts[index];
  const double size = std::stod(parts[index + 1]);
  const std::string seed_token = parts[index + 2];
  if (seed_token.rfind("s", 0) != 0) {
    throw std::runtime_error("bad continuous instance id: " + id);
  }
  const std::uint64_t geometry_seed =
      static_cast<std::uint64_t>(std::stoull(seed_token.substr(1)));
  ContinuousInstance inst =
      make_continuous_by_class(class_name, size, capacity, geometry_seed);
  inst.id = id;
  return inst;
}

GridInstance make_grid_instance(const std::string& id) {
  // grid-<class>-<size>-s<seed>
  std::vector<std::string> parts;
  std::string token;
  for (const char ch : id) {
    if (ch == '-') {
      parts.push_back(token);
      token.clear();
    } else {
      token.push_back(ch);
    }
  }
  parts.push_back(token);
  if (parts.size() != 4 || parts[0] != "grid") {
    throw std::runtime_error("bad grid instance id: " + id);
  }
  const std::string class_name = parts[1];
  const int size = std::stoi(parts[2]);
  const std::string seed_token = parts[3];
  if (seed_token.rfind("s", 0) != 0) {
    throw std::runtime_error("bad grid instance id: " + id);
  }
  const std::uint64_t geometry_seed =
      static_cast<std::uint64_t>(std::stoull(seed_token.substr(1)));
  const bool capacity = size == 2048 || size == 4096;
  GridInstance inst =
      make_grid_by_class(class_name, size, capacity, geometry_seed);
  inst.id = id;
  return inst;
}

bool continuous_point_valid(const ContinuousInstance& instance,
                            const Point2d& point) {
  if (point.x_m < instance.bounds.min_x || point.x_m > instance.bounds.max_x ||
      point.y_m < instance.bounds.min_y || point.y_m > instance.bounds.max_y) {
    return false;
  }
  return !point_in_any_rect(instance.obstacles, point);
}

bool continuous_edge_valid(const ContinuousInstance& instance,
                           const Point2d& a, const Point2d& b) {
  if (!continuous_point_valid(instance, a) ||
      !continuous_point_valid(instance, b)) {
    return false;
  }
  for (const RectObstacle& rect : instance.obstacles) {
    if (segment_intersects_rect(a, b, rect)) {
      return false;
    }
  }
  return true;
}

bool grid_point_valid(const GridInstance& instance, const Point2d& point) {
  const double res = instance.resolution_m;
  const double extent = static_cast<double>(instance.size) * res;
  if (point.x_m < 0.0 || point.x_m > extent || point.y_m < 0.0 ||
      point.y_m > extent) {
    return false;
  }
  int col = static_cast<int>(std::floor(point.x_m / res));
  int row = static_cast<int>(std::floor(point.y_m / res));
  col = std::clamp(col, 0, instance.size - 1);
  row = std::clamp(row, 0, instance.size - 1);
  return instance.at(row, col) == 0;
}

Problem make_problem(const ContinuousInstance& instance, double budget_s,
                     std::uint64_t seed) {
  Problem problem;
  problem.bounds = instance.bounds;
  problem.start = instance.start;
  problem.goal = instance.goal;
  problem.edge_check_step = instance.edge_check_step;
  problem.budget_s = budget_s;
  problem.seed = seed;
  auto geometry = std::make_shared<ContinuousInstance>(instance);
  problem.point_valid = [geometry](const Point2d& p) {
    return continuous_point_valid(*geometry, p);
  };
  problem.edge_valid = [geometry](const Point2d& a, const Point2d& b) {
    return continuous_edge_valid(*geometry, a, b);
  };
  return problem;
}

Problem make_problem(const GridInstance& instance, double budget_s,
                     std::uint64_t seed) {
  Problem problem;
  const double res = instance.resolution_m;
  const double extent = static_cast<double>(instance.size) * res;
  problem.bounds = Bounds2d{0.0, 0.0, extent, extent};
  problem.start = Point2d{(instance.start_col + 0.5) * res,
                          (instance.start_row + 0.5) * res};
  problem.goal = Point2d{(instance.goal_col + 0.5) * res,
                         (instance.goal_row + 0.5) * res};
  problem.edge_check_step = res;
  problem.budget_s = budget_s;
  problem.seed = seed;
  auto geometry = std::make_shared<GridInstance>(instance);
  problem.point_valid = [geometry](const Point2d& p) {
    return grid_point_valid(*geometry, p);
  };
  auto validity = std::make_shared<SampledEdgeValidity>(
      [geometry](const Point2d& p) { return grid_point_valid(*geometry, p); },
      res);
  problem.edge_valid = [validity](const Point2d& a, const Point2d& b) {
    return validity->is_edge_valid(a, b);
  };
  return problem;
}

std::string canonical_instance_json(const ContinuousInstance& instance) {
  nlohmann::json j;
  j["format"] = "planner.sampling_search.continuous_instance";
  j["format_version"] = 1;
  j["generator"] = kContinuousGenerator;
  j["id"] = instance.id;
  j["class"] = instance.class_name;
  j["capacity"] = instance.capacity;
  j["size_m"] = instance.size_m;
  j["geometry_seed"] = instance.geometry_seed;
  j["bounds"] = {instance.bounds.min_x, instance.bounds.min_y,
                 instance.bounds.max_x, instance.bounds.max_y};
  j["start"] = {instance.start.x_m, instance.start.y_m};
  j["goal"] = {instance.goal.x_m, instance.goal.y_m};
  j["edge_check_step_m"] = instance.edge_check_step;
  j["edge_check_mode"] = "exact_segment_rectangle";
  nlohmann::json obstacles = nlohmann::json::array();
  for (const RectObstacle& r : instance.obstacles) {
    obstacles.push_back({r.x0, r.y0, r.x1, r.y1});
  }
  j["obstacles"] = std::move(obstacles);
  return j.dump();
}

std::string canonical_instance_json(const GridInstance& instance) {
  nlohmann::json j;
  j["format"] = "planner.sampling_search.grid_instance";
  j["format_version"] = 1;
  j["generator"] = kGridGenerator;
  j["generator_note"] =
      "provisional; cross-check with grid_scenario_gen v1 pending";
  j["id"] = instance.id;
  j["class"] = instance.class_name;
  j["capacity"] = instance.capacity;
  j["size"] = instance.size;
  j["resolution_m"] = instance.resolution_m;
  j["geometry_seed"] = instance.geometry_seed;
  j["start_rc"] = {instance.start_row, instance.start_col};
  j["goal_rc"] = {instance.goal_row, instance.goal_col};
  j["edge_check_mode"] = "sampled_at_resolution";
  j["cell_order"] = "row-major; row 0 = bottom (+y); 1 = occupied";
  std::vector<std::uint8_t> packed(
      (static_cast<std::size_t>(instance.size) * instance.size + 7) / 8, 0);
  for (std::size_t i = 0; i < instance.cells.size(); ++i) {
    if (instance.cells[i] != 0) {
      packed[i / 8] |= static_cast<std::uint8_t>(1U << (7U - (i % 8U)));
    }
  }
  j["cells_bits_b64"] = base64_encode(packed);
  return j.dump();
}

std::string instance_sha256(const ContinuousInstance& instance) {
  return bench_sha256_hex(canonical_instance_json(instance));
}

std::string instance_sha256(const GridInstance& instance) {
  return bench_sha256_hex(canonical_instance_json(instance));
}

std::string generator_self_check(bool include_capacity) {
  for (const std::string& id : continuous_instance_ids(include_capacity)) {
    const ContinuousInstance inst = make_continuous_instance(id);
    if (!continuous_point_valid(inst, inst.start)) {
      return id + ": start invalid";
    }
    if (!continuous_point_valid(inst, inst.goal)) {
      return id + ": goal invalid";
    }
    if (inst.class_name == "sparse") {
      if (!coarse_connected(inst.size_m, inst.obstacles, inst.start, inst.goal,
                            1.0, true)) {
        return id + ": conservative coarse grid is disconnected";
      }
    }
    if (inst.class_name == "unreachable") {
      if (coarse_connected(inst.size_m, inst.obstacles, inst.start, inst.goal,
                           1.0, true)) {
        return id + ": closed rectangle leaks through conservative grid";
      }
    }
  }
  for (const std::string& id : grid_instance_ids(include_capacity)) {
    const GridInstance inst = make_grid_instance(id);
    if (inst.at(inst.start_row, inst.start_col) != 0) {
      return id + ": start cell occupied";
    }
    if (inst.at(inst.goal_row, inst.goal_col) != 0) {
      return id + ": goal cell occupied";
    }
    bool reachable = false;
    if (!grid_cells_connected(inst, &reachable)) {
      return id + ": cell BFS setup failed";
    }
    if (inst.class_name == "unreachable") {
      if (reachable) {
        return id + ": closed cavity is reachable at cell level";
      }
    } else if (!reachable) {
      return id + ": cells disconnected";
    }
  }
  return {};
}

}  // namespace planner::sampling_search::bench
