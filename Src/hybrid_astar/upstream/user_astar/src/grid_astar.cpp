#include "astar/grid_astar.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>

namespace astar {

namespace {

bool ParseDouble(const std::string& token, double* value) {
    if (value == nullptr) {
        return false;
    }
    try {
        std::size_t parsed = 0;
        const double result = std::stod(token, &parsed);
        if (parsed != token.size()) {
            return false;
        }
        *value = result;
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace

GridAStar::GridAStar(double grid_resolution)
    : grid_resolution_(grid_resolution > 0.0 ? grid_resolution : 1.0) {}

GridAStar::GridAStar(const PlannerConfig& config)
    : config_(config),
      grid_resolution_(config.hybrid_astar.grid_dist > 0.0 ? config.hybrid_astar.grid_dist : 1.0) {
    SetConfig(config);
}

void GridAStar::SetConfig(const PlannerConfig& config) {
    config_ = config;
    if (config_.hybrid_astar.grid_dist > 0.0) {
        grid_resolution_ = config_.hybrid_astar.grid_dist;
    }
    double inflation_meter = config_.vehicle.boundary_inflation_meter;
    if (inflation_meter < 0.0) {
        // Keep behavior consistent with HybridAStar and README:
        // negative value means no extra boundary inflation.
        inflation_meter = 0.0;
    }
    inflation_cells_ = static_cast<int32_t>(std::ceil(inflation_meter / grid_resolution_));
}

const PlannerConfig& GridAStar::GetConfig() const {
    return config_;
}

void GridAStar::ClearBoundaries() {
    boundary_set_.clear();
}

std::size_t GridAStar::BoundaryCount() const {
    return boundary_set_.size();
}

bool GridAStar::LoadBoundaryFromCsv(const std::string& csv_path, bool close_loop) {
    (void)close_loop; // Standalone mode uses discrete boundary points only.
    std::ifstream ifs(csv_path);
    if (!ifs.is_open()) {
        return false;
    }

    std::vector<IntCoordinate> points;
    std::string line;
    while (std::getline(ifs, line)) {
        double x = 0.0;
        double y = 0.0;
        if (!ParseCsvPoint(line, &x, &y)) {
            continue;
        }

        IntCoordinate point;
        point.x = static_cast<int16_t>(std::floor(x / grid_resolution_));
        point.y = static_cast<int16_t>(std::floor(y / grid_resolution_));
        point.z = 0;

        if (!points.empty() && points.back().x == point.x && points.back().y == point.y) {
            continue;
        }
        points.push_back(point);
    }

    if (points.empty()) {
        return false;
    }

    for (const auto& p : points) {
        AddBoundaryPoint(p);
    }
    return true;
}

bool GridAStar::Search(const IntCoordinate& start, const IntCoordinate& goal, std::vector<IntCoordinate>* path, float* cost) {
    if (path != nullptr) {
        path->clear();
    }
    if (cost != nullptr) {
        *cost = std::numeric_limits<float>::infinity();
    }

    if (IsBoundGrid(start) || IsBoundGrid(goal)) {
        return false;
    }

    nodes2d_.clear();
    open2d_.clear();
    parent_.clear();

    Node2D start_node(start.x, start.y, 0.0f, 0.0f);
    Node2D goal_node(goal.x, goal.y, 0.0f, 0.0f);

    start_node.UpdateH(goal_node);
    start_node.Open();
    start_node.Discover();

    nodes2d_[start_node.GetIdx()] = start_node;
    open2d_.insert(OpenHash(start_node));
    parent_[start_node.GetIdx()] = start_node.GetIdx();

    while (!open2d_.empty()) {
        const uint64_t open_key = *open2d_.begin();
        open2d_.erase(open2d_.begin());

        const uint32_t pred_idx = static_cast<uint32_t>(open_key & 0xFFFFFFFFULL);
        auto pred_iter = nodes2d_.find(pred_idx);
        if (pred_iter == nodes2d_.end()) {
            continue;
        }
        if (pred_iter->second.IsClosed()) {
            continue;
        }

        pred_iter->second.Close();
        pred_iter->second.Discover();
        const Node2D pred = pred_iter->second;

        if (pred == goal_node) {
            if (cost != nullptr) {
                *cost = pred.GetG() * static_cast<float>(grid_resolution_);
            }
            if (path != nullptr) {
                std::vector<IntCoordinate> reversed_path;
                uint32_t current = pred_idx;
                while (true) {
                    reversed_path.push_back(Hash2Coordinate(current));
                    const auto parent_iter = parent_.find(current);
                    if (parent_iter == parent_.end() || parent_iter->second == current) {
                        break;
                    }
                    current = parent_iter->second;
                }
                std::reverse(reversed_path.begin(), reversed_path.end());
                *path = reversed_path;
            }
            return true;
        }

        for (int32_t i = 0; i < Node2D::dir; ++i) {
            Node2D successor = pred.CreateSuccessor(static_cast<int16_t>(i));
            const uint32_t succ_idx = successor.GetIdx();
            const IntCoordinate succ_point{successor.GetX(), successor.GetY(), 0};

            auto succ_iter = nodes2d_.find(succ_idx);
            const bool exists = (succ_iter != nodes2d_.end());

            if (IsBoundGrid(succ_point) || (exists && succ_iter->second.IsClosed())) {
                continue;
            }

            const float new_g = pred.GetG() + ComputeStepCost(i);
            successor = Node2D(successor.GetX(), successor.GetY(), new_g, 0.0f);
            if (exists && succ_iter->second.IsOpen()) {
                if (new_g >= succ_iter->second.GetG()) {
                    continue;
                }
                open2d_.erase(OpenHash(succ_iter->second));
            }

            successor.UpdateH(goal_node);
            successor.Open();
            successor.Discover();
            nodes2d_[succ_idx] = successor;
            parent_[succ_idx] = pred_idx;
            open2d_.insert(OpenHash(successor));
        }
    }

    return false;
}

uint32_t GridAStar::Coordinate2Hash(const IntCoordinate& point) const {
    uint32_t hash = 0U;
    hash |= static_cast<uint32_t>(static_cast<uint16_t>(point.x)) << 16U;
    hash |= static_cast<uint32_t>(static_cast<uint16_t>(point.y));
    return hash;
}

IntCoordinate GridAStar::Hash2Coordinate(uint32_t hash) const {
    IntCoordinate point;
    point.x = static_cast<int16_t>((hash >> 16U) & 0xFFFFU);
    point.y = static_cast<int16_t>(hash & 0xFFFFU);
    point.z = 0;
    return point;
}

bool GridAStar::IsBoundGrid(const IntCoordinate& point) const {
    const uint32_t hash = Coordinate2Hash(point);

    const IntCoordinate west{static_cast<int16_t>(point.x - 1), point.y, point.z};
    const IntCoordinate east{static_cast<int16_t>(point.x + 1), point.y, point.z};
    const IntCoordinate north{point.x, static_cast<int16_t>(point.y + 1), point.z};
    const IntCoordinate south{point.x, static_cast<int16_t>(point.y - 1), point.z};

    const IntCoordinate west_north{static_cast<int16_t>(point.x - 1), static_cast<int16_t>(point.y + 1), point.z};
    const IntCoordinate east_north{static_cast<int16_t>(point.x + 1), static_cast<int16_t>(point.y + 1), point.z};
    const IntCoordinate west_south{static_cast<int16_t>(point.x - 1), static_cast<int16_t>(point.y - 1), point.z};
    const IntCoordinate east_south{static_cast<int16_t>(point.x + 1), static_cast<int16_t>(point.y - 1), point.z};

    if (boundary_set_.count(hash) == 1U) {
        return true;
    }
    if ((boundary_set_.count(Coordinate2Hash(west)) == 1U) && (boundary_set_.count(Coordinate2Hash(north)) == 1U) &&
        (boundary_set_.count(Coordinate2Hash(west_north)) == 0U)) {
        return true;
    }
    if ((boundary_set_.count(Coordinate2Hash(west)) == 1U) && (boundary_set_.count(Coordinate2Hash(south)) == 1U) &&
        (boundary_set_.count(Coordinate2Hash(west_south)) == 0U)) {
        return true;
    }
    if ((boundary_set_.count(Coordinate2Hash(east)) == 1U) && (boundary_set_.count(Coordinate2Hash(south)) == 1U) &&
        (boundary_set_.count(Coordinate2Hash(east_south)) == 0U)) {
        return true;
    }
    if ((boundary_set_.count(Coordinate2Hash(east)) == 1U) && (boundary_set_.count(Coordinate2Hash(north)) == 1U) &&
        (boundary_set_.count(Coordinate2Hash(east_north)) == 0U)) {
        return true;
    }

    return false;
}

bool GridAStar::ParseCsvPoint(const std::string& line, double* x, double* y) const {
    const std::string trimmed = Trim(line);
    if (trimmed.empty() || trimmed[0] == '#') {
        return false;
    }

    std::stringstream ss(trimmed);
    std::string token_x;
    std::string token_y;
    std::string token_z;
    if (!std::getline(ss, token_x, ',')) {
        return false;
    }
    if (!std::getline(ss, token_y, ',')) {
        return false;
    }
    // Optional z column, ignored by planner. Default is 0 when absent.
    if (!std::getline(ss, token_z, ',')) {
        token_z = "0";
    }

    token_x = Trim(token_x);
    token_y = Trim(token_y);
    token_z = Trim(token_z);

    double z_unused = 0.0;
    if (token_z.empty()) {
        token_z = "0";
    }

    return ParseDouble(token_x, x) && ParseDouble(token_y, y) && ParseDouble(token_z, &z_unused);
}

void GridAStar::RasterizeLine(const IntCoordinate& start, const IntCoordinate& end) {
    int x0 = start.x;
    int y0 = start.y;
    const int x1 = end.x;
    const int y1 = end.y;

    const int dx = std::abs(x1 - x0);
    const int sx = (x0 < x1) ? 1 : -1;
    const int dy = -std::abs(y1 - y0);
    const int sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    while (true) {
        IntCoordinate point{static_cast<int16_t>(x0), static_cast<int16_t>(y0), 0};
        AddBoundaryPoint(point);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int err2 = 2 * err;
        if (err2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (err2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void GridAStar::AddBoundaryPoint(const IntCoordinate& point) {
    for (int32_t dx = -inflation_cells_; dx <= inflation_cells_; ++dx) {
        for (int32_t dy = -inflation_cells_; dy <= inflation_cells_; ++dy) {
            if (dx * dx + dy * dy > inflation_cells_ * inflation_cells_) {
                continue;
            }
            const IntCoordinate p{
                static_cast<int16_t>(point.x + dx),
                static_cast<int16_t>(point.y + dy),
                point.z
            };
            boundary_set_.insert(Coordinate2Hash(p));
        }
    }
}

float GridAStar::ComputeStepCost(int32_t move_index) const {
    const float base = Node2D::dg[move_index] * static_cast<float>(grid_resolution_);
    const bool is_diagonal = (std::abs(Node2D::dx[move_index]) + std::abs(Node2D::dy[move_index])) == 2;
    if (is_diagonal) {
        return base * static_cast<float>(config_.hybrid_astar.turnning_penalty);
    }
    return base * static_cast<float>(config_.hybrid_astar.forward_penalty);
}

uint64_t GridAStar::OpenHash(const Node2D& node) {
    const uint64_t f = static_cast<uint64_t>(std::llround(static_cast<double>(node.GetF()) * 1000.0));
    return node.GetIdx() | (f << 32U);
}

std::string GridAStar::Trim(const std::string& text) {
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

} // namespace astar
