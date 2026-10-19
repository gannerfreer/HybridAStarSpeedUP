// grid_snapshot 文档组装与 base64_u8 编码实现（见 snapshot.hpp）。

#include "planner/scene/snapshot.hpp"

#include <stdexcept>
#include <string>

namespace planner::scene {
namespace {

constexpr char kBase64Alphabet[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

}  // namespace

std::string encode_cells_base64(const std::vector<int>& cells) {
  std::string out;
  out.reserve(((cells.size() + 2U) / 3U) * 4U);
  std::size_t index = 0;
  while (index + 3U <= cells.size()) {
    const int a = cells[index];
    const int b = cells[index + 1U];
    const int c = cells[index + 2U];
    if (a < 0 || a > 2 || b < 0 || b > 2 || c < 0 || c > 2) {
      throw std::invalid_argument(
          "grid cell value outside {0,1,2} while encoding base64_u8");
    }
    const unsigned int block =
        (static_cast<unsigned int>(a) << 16U) | (static_cast<unsigned int>(b) << 8U) |
        static_cast<unsigned int>(c);
    out.push_back(kBase64Alphabet[(block >> 18U) & 0x3FU]);
    out.push_back(kBase64Alphabet[(block >> 12U) & 0x3FU]);
    out.push_back(kBase64Alphabet[(block >> 6U) & 0x3FU]);
    out.push_back(kBase64Alphabet[block & 0x3FU]);
    index += 3U;
  }
  const std::size_t remaining = cells.size() - index;
  if (remaining == 1U) {
    const int a = cells[index];
    if (a < 0 || a > 2) {
      throw std::invalid_argument(
          "grid cell value outside {0,1,2} while encoding base64_u8");
    }
    const unsigned int block = static_cast<unsigned int>(a) << 16U;
    out.push_back(kBase64Alphabet[(block >> 18U) & 0x3FU]);
    out.push_back(kBase64Alphabet[(block >> 12U) & 0x3FU]);
    out += "==";
  } else if (remaining == 2U) {
    const int a = cells[index];
    const int b = cells[index + 1U];
    if (a < 0 || a > 2 || b < 0 || b > 2) {
      throw std::invalid_argument(
          "grid cell value outside {0,1,2} while encoding base64_u8");
    }
    const unsigned int block =
        (static_cast<unsigned int>(a) << 16U) | (static_cast<unsigned int>(b) << 8U);
    out.push_back(kBase64Alphabet[(block >> 18U) & 0x3FU]);
    out.push_back(kBase64Alphabet[(block >> 12U) & 0x3FU]);
    out.push_back(kBase64Alphabet[(block >> 6U) & 0x3FU]);
    out.push_back('=');
  }
  return out;
}

std::string encode_cells_base64(const common::GridView& grid) {
  return encode_cells_base64(grid.cells);
}

nlohmann::json make_grid_snapshot_json(const common::GridView& grid,
                                       const GridSnapshotOptions& options) {
  nlohmann::json doc;
  doc["schema_version"] = common::kSchemaVersion;
  doc["type"] = "grid_snapshot";
  doc["source"] = options.source;
  if (!options.scene_id.empty()) {
    doc["scene_id"] = options.scene_id;
  }
  if (!options.scene_version.empty()) {
    doc["scene_version"] = options.scene_version;
  }
  doc["map_id"] = grid.map_id;
  doc["map_version"] = grid.map_version;
  doc["window"] = nlohmann::json{{"min_x_m", options.window.min_x_m},
                                 {"min_y_m", options.window.min_y_m},
                                 {"max_x_m", options.window.max_x_m},
                                 {"max_y_m", options.window.max_y_m}};
  doc["requested_resolution_m"] = options.requested_resolution_m;
  doc["effective_resolution_m"] = grid.resolution_m;
  doc["resolution_m"] = grid.resolution_m;
  doc["degraded"] = grid.resolution_m > options.requested_resolution_m;
  doc["width"] = grid.width;
  doc["height"] = grid.height;
  doc["origin"] = nlohmann::json{{"x_m", grid.origin.x_m},
                                 {"y_m", grid.origin.y_m},
                                 {"yaw_rad", grid.origin.yaw_rad}};
  doc["encoding"] = "base64_u8";
  doc["cells"] = encode_cells_base64(grid);
  if (options.request_seq.has_value()) {
    doc["request_seq"] = *options.request_seq;
  }
  doc["budget_cells"] = options.budget_cells;
  return doc;
}

}  // namespace planner::scene
