#include "astar/hybrid_astar.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <sstream>
#include <utility>

namespace astar {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxBoundaryCollisionMarginMeter = 1.0;
constexpr double kCollisionEpsilon = 1e-9;

bool ParseDoubleStrict(const std::string& token, double* out) {
    if (out == nullptr) {
        return false;
    }
    try {
        std::size_t parsed = 0;
        const double value = std::stod(token, &parsed);
        if (parsed != token.size()) {
            return false;
        }
        *out = value;
        return true;
    } catch (...) {
        return false;
    }
}

} // namespace

HybridAStar::HybridAStar(const PlannerConfig& config) : config_(config) {
    SetConfig(config);
}

void HybridAStar::SetConfig(const PlannerConfig& config) {
    config_ = config;
    grid_dist_ = (config_.hybrid_astar.grid_dist > 0.0) ? config_.hybrid_astar.grid_dist : 1.0;

    const double grid_angle_deg = (config_.hybrid_astar.grid_angle > 0.0) ? config_.hybrid_astar.grid_angle : 1.0;
    grid_angle_rad_ = grid_angle_deg * kPi / 180.0;
    angle_bin_count_ = std::max(8, static_cast<int32_t>(std::round(2.0 * kPi / grid_angle_rad_)));

    collision_front_meter_ = std::max(0.0, config_.vehicle.center2front);
    collision_rear_meter_ = std::max(0.0, config_.vehicle.center2rear);
    collision_half_width_meter_ = std::max(0.0, config_.vehicle.center2side);
    collision_safe_margin_meter_ = std::clamp(config_.vehicle.safe_margin, 0.0, kMaxBoundaryCollisionMarginMeter);

    // boundary_inflation_meter is kept as optional extra map inflation in standalone mode.
    // The vehicle footprint collision itself is handled by exact rectangle SAT check in IsCollision().
    double inflation_meter = config_.vehicle.boundary_inflation_meter;
    if (inflation_meter < 0.0) {
        inflation_meter = 0.0;
    }
    inflation_cells_ = std::max(0, static_cast<int32_t>(std::ceil(inflation_meter / grid_dist_)));
}

const PlannerConfig& HybridAStar::GetConfig() const {
    return config_;
}

void HybridAStar::ClearBoundaries() {
    boundaries_.clear();
}

std::size_t HybridAStar::BoundaryCount() const {
    return boundaries_.size();
}

// [planner-adapter 扩展] 钩子与栅格化入口实现（不改变默认搜索语义）。
void HybridAStar::SetAbortCallback(std::function<bool()> callback) {
    abort_callback_ = std::move(callback);
}

void HybridAStar::SetPoseCollisionHook(std::function<bool(const Pose2D&)> hook) {
    pose_collision_hook_ = std::move(hook);
}

void HybridAStar::SetMotionCollisionHook(
    std::function<bool(const Pose2D&, const Pose2D&)> hook) {
    motion_collision_hook_ = std::move(hook);
}

void HybridAStar::SetGoalHook(std::function<bool(const Pose2D&)> hook) {
    goal_hook_ = std::move(hook);
}

void HybridAStar::AddBoundaryWorldPoint(double x, double y) {
    GridPoint p;
    p.x = static_cast<int32_t>(std::floor(x / grid_dist_));
    p.y = static_cast<int32_t>(std::floor(y / grid_dist_));
    AddBoundaryPoint(p);
}

bool HybridAStar::LoadBoundaryFromCsv(const std::string& csv_path, bool close_loop) {
    (void)close_loop; // Standalone mode uses discrete boundary points only.
    std::ifstream ifs(csv_path);
    if (!ifs.is_open()) {
        return false;
    }

    std::vector<GridPoint> points;
    std::string line;
    while (std::getline(ifs, line)) {
        double x = 0.0;
        double y = 0.0;
        if (!ParseCsvPoint(line, &x, &y)) {
            continue;
        }
        GridPoint p;
        p.x = static_cast<int32_t>(std::floor(x / grid_dist_));
        p.y = static_cast<int32_t>(std::floor(y / grid_dist_));
        if (!points.empty() && points.back().x == p.x && points.back().y == p.y) {
            continue;
        }
        points.push_back(p);
    }

    if (points.empty()) {
        return false;
    }
    for (const auto& p : points) {
        AddBoundaryPoint(p);
    }
    return true;
}

bool HybridAStar::Search(const Pose2D& start, const Pose2D& goal, HybridSearchResult* result) {
    if (result == nullptr) {
        return false;
    }
    result->success = false;
    result->cost = 0.0;
    result->iterations = 0;
    result->path.clear();
    result->message.clear();

    while (!open_queue_.empty()) {
        open_queue_.pop();
    }
    open_set_.clear();
    closed_set_.clear();
    best_nodes_.clear();
    search_start_pose_ = start;

    Vertex3D start_forward;
    start_forward.x = start.x;
    start_forward.y = start.y;
    start_forward.yaw = NormalizeAngle(start.yaw);
    start_forward.direction = MotionDirection::Forward;
    start_forward.parent_id = 0;
    start_forward.id = VertexHash(start_forward);
    start_forward.g = 0.0;
    start_forward.h = Heuristic(start_forward, goal);
    start_forward.f = start_forward.g + start_forward.h;

    Vertex3D start_backward = start_forward;
    start_backward.direction = MotionDirection::Backward;
    start_backward.id = VertexHash(start_backward);
    start_backward.f = start_backward.h;

    Vertex3D goal_check;
    goal_check.x = goal.x;
    goal_check.y = goal.y;
    goal_check.yaw = NormalizeAngle(goal.yaw);
    goal_check.direction = goal.direction;
    if (IsCollision(start_forward) || IsCollision(goal_check)) {
        result->message = "start or goal in collision";
        return false;
    }

    open_set_[start_forward.id] = start_forward;
    best_nodes_[start_forward.id] = start_forward;
    open_queue_.push(OpenEntry{start_forward.f, start_forward.id});

    open_set_[start_backward.id] = start_backward;
    best_nodes_[start_backward.id] = start_backward;
    open_queue_.push(OpenEntry{start_backward.f, start_backward.id});

    const int32_t max_iterations = std::max(1, static_cast<int32_t>(config_.hybrid_astar.max_iterations_astar));
    uint64_t      final_id = 0;
    bool          found = false;

    for (int32_t iter = 0; iter < max_iterations; ++iter) {
        if (abort_callback_ && abort_callback_()) {
            result->message = "aborted";
            return false;
        }
        result->iterations = iter + 1;
        if (open_queue_.empty()) {
            break;
        }

        const OpenEntry top = open_queue_.top();
        open_queue_.pop();

        auto open_it = open_set_.find(top.id);
        if (open_it == open_set_.end()) {
            continue;
        }
        if (std::fabs(open_it->second.f - top.f) > 1e-9) {
            continue;
        }

        const Vertex3D current = open_it->second;
        open_set_.erase(open_it);
        closed_set_[current.id] = current;

        if (IsGoal(current, goal)) {
            found = true;
            final_id = current.id;
            result->cost = current.g;
            break;
        }

        ExpandNode(current, start, goal);
    }

    if (!found) {
        result->message = "no feasible path";
        return false;
    }

    if (!ReconstructPath(final_id, result)) {
        result->message = "failed to reconstruct path";
        return false;
    }

    result->success = true;
    return true;
}

double HybridAStar::CalculateTransitionCostForTest(const Pose2D& from, const Pose2D& to) const {
    Vertex3D vf;
    vf.x = from.x;
    vf.y = from.y;
    vf.yaw = NormalizeAngle(from.yaw);
    vf.direction = from.direction;
    vf.g = 0.0;

    Vertex3D vt;
    vt.x = to.x;
    vt.y = to.y;
    vt.yaw = NormalizeAngle(to.yaw);
    vt.direction = to.direction;
    vt.g = 0.0;
    return TransitionCost(vf, vt);
}

bool HybridAStar::ParseCsvPoint(const std::string& line, double* x, double* y) const {
    const std::string t = Trim(line);
    if (t.empty() || t[0] == '#' || t[0] == ';') {
        return false;
    }
    std::stringstream ss(t);
    std::string sx;
    std::string sy;
    std::string sz;
    if (!std::getline(ss, sx, ',')) {
        return false;
    }
    if (!std::getline(ss, sy, ',')) {
        return false;
    }
    // Optional z column, ignored by planner. Default is 0 when absent.
    if (!std::getline(ss, sz, ',')) {
        sz = "0";
    }
    sx = Trim(sx);
    sy = Trim(sy);
    sz = Trim(sz);
    if (sz.empty()) {
        sz = "0";
    }
    double z_unused = 0.0;
    return ParseDoubleStrict(sx, x) && ParseDoubleStrict(sy, y) && ParseDoubleStrict(sz, &z_unused);
}

void HybridAStar::RasterizeLine(const GridPoint& start, const GridPoint& end) {
    int32_t x0 = start.x;
    int32_t y0 = start.y;
    const int32_t x1 = end.x;
    const int32_t y1 = end.y;

    const int32_t dx = std::abs(x1 - x0);
    const int32_t sx = (x0 < x1) ? 1 : -1;
    const int32_t dy = -std::abs(y1 - y0);
    const int32_t sy = (y0 < y1) ? 1 : -1;
    int32_t err = dx + dy;

    while (true) {
        AddBoundaryPoint(GridPoint{x0, y0});
        if (x0 == x1 && y0 == y1) {
            break;
        }
        const int32_t e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void HybridAStar::AddBoundaryPoint(const GridPoint& p) {
    for (int32_t dx = -inflation_cells_; dx <= inflation_cells_; ++dx) {
        for (int32_t dy = -inflation_cells_; dy <= inflation_cells_; ++dy) {
            boundaries_.insert(CoordinateHash(GridPoint{p.x + dx, p.y + dy}));
        }
    }
}

bool HybridAStar::IsBoundGrid(const GridPoint& p) const {
    const uint64_t hash = CoordinateHash(p);
    const GridPoint west{p.x - 1, p.y};
    const GridPoint east{p.x + 1, p.y};
    const GridPoint north{p.x, p.y + 1};
    const GridPoint south{p.x, p.y - 1};
    const GridPoint west_north{p.x - 1, p.y + 1};
    const GridPoint east_north{p.x + 1, p.y + 1};
    const GridPoint west_south{p.x - 1, p.y - 1};
    const GridPoint east_south{p.x + 1, p.y - 1};

    if (boundaries_.count(hash) == 1U) {
        return true;
    }
    if ((boundaries_.count(CoordinateHash(west)) == 1U) && (boundaries_.count(CoordinateHash(north)) == 1U) &&
        (boundaries_.count(CoordinateHash(west_north)) == 0U)) {
        return true;
    }
    if ((boundaries_.count(CoordinateHash(west)) == 1U) && (boundaries_.count(CoordinateHash(south)) == 1U) &&
        (boundaries_.count(CoordinateHash(west_south)) == 0U)) {
        return true;
    }
    if ((boundaries_.count(CoordinateHash(east)) == 1U) && (boundaries_.count(CoordinateHash(south)) == 1U) &&
        (boundaries_.count(CoordinateHash(east_south)) == 0U)) {
        return true;
    }
    if ((boundaries_.count(CoordinateHash(east)) == 1U) && (boundaries_.count(CoordinateHash(north)) == 1U) &&
        (boundaries_.count(CoordinateHash(east_north)) == 0U)) {
        return true;
    }
    return false;
}

bool HybridAStar::IsCollision(const Vertex3D& node) const {
    if (pose_collision_hook_) {
        Pose2D pose;
        pose.x = node.x;
        pose.y = node.y;
        pose.yaw = node.yaw;
        pose.direction = node.direction;
        return pose_collision_hook_(pose);
    }
    if (boundaries_.empty()) {
        return false;
    }

    const double front = collision_front_meter_ + collision_safe_margin_meter_;
    const double rear = collision_rear_meter_ + collision_safe_margin_meter_;
    const double half_width = collision_half_width_meter_ + collision_safe_margin_meter_;
    if (front <= 0.0 || rear <= 0.0 || half_width <= 0.0) {
        return false;
    }

    const double half_length = 0.5 * (front + rear);
    const double center_offset = 0.5 * (front - rear);
    const double cos_yaw = std::cos(node.yaw);
    const double sin_yaw = std::sin(node.yaw);
    const double center_x = node.x + center_offset * cos_yaw;
    const double center_y = node.y + center_offset * sin_yaw;

    const double ux_x = cos_yaw;
    const double ux_y = sin_yaw;
    const double uy_x = -sin_yaw;
    const double uy_y = cos_yaw;

    const double abs_ux_x = std::fabs(ux_x);
    const double abs_ux_y = std::fabs(ux_y);
    const double abs_uy_x = std::fabs(uy_x);
    const double abs_uy_y = std::fabs(uy_y);

    // Vehicle OBB AABB for broad-phase culling.
    const double extent_x = half_length * abs_ux_x + half_width * abs_uy_x;
    const double extent_y = half_length * abs_ux_y + half_width * abs_uy_y;
    const double min_x = center_x - extent_x;
    const double max_x = center_x + extent_x;
    const double min_y = center_y - extent_y;
    const double max_y = center_y + extent_y;

    const int32_t min_ix = static_cast<int32_t>(std::floor(min_x / grid_dist_));
    const int32_t max_ix = static_cast<int32_t>(std::floor(max_x / grid_dist_));
    const int32_t min_iy = static_cast<int32_t>(std::floor(min_y / grid_dist_));
    const int32_t max_iy = static_cast<int32_t>(std::floor(max_y / grid_dist_));

    for (int32_t gx = min_ix; gx <= max_ix; ++gx) {
        for (int32_t gy = min_iy; gy <= max_iy; ++gy) {
            const GridPoint cell{gx, gy};
            if (!IsBoundGrid(cell)) {
                continue;
            }

            // Treat each occupied grid as a square obstacle cell instead of a single point.
            const double obstacle_x = (static_cast<double>(gx) + 0.5) * grid_dist_;
            const double obstacle_y = (static_cast<double>(gy) + 0.5) * grid_dist_;
            const double tx = obstacle_x - center_x;
            const double ty = obstacle_y - center_y;

            const double proj_on_ux = std::fabs(tx * ux_x + ty * ux_y);
            const double proj_on_uy = std::fabs(tx * uy_x + ty * uy_y);
            const double half_cell = 0.5 * grid_dist_;
            const double cell_proj_on_ux = half_cell * (abs_ux_x + abs_ux_y);
            const double cell_proj_on_uy = half_cell * (abs_uy_x + abs_uy_y);
            if (proj_on_ux <= half_length + cell_proj_on_ux + kCollisionEpsilon &&
                proj_on_uy <= half_width + cell_proj_on_uy + kCollisionEpsilon) {
                return true;
            }
        }
    }
    return false;
}

bool HybridAStar::IsGoal(const Vertex3D& node, const Pose2D& goal) const {
    if (goal_hook_) {
        Pose2D pose;
        pose.x = node.x;
        pose.y = node.y;
        pose.yaw = node.yaw;
        pose.direction = node.direction;
        return goal_hook_(pose);
    }
    const double dist = std::hypot(node.x - goal.x, node.y - goal.y);
    const double angle_diff = std::fabs(NormalizeAngle(node.yaw - goal.yaw));
    const double pos_tol = std::max(config_.hybrid_astar.delta_dist, grid_dist_ * 1.5);
    const double configured_yaw_tol_deg =
        (config_.hybrid_astar.goal_yaw_tolerance_deg > 0.0) ? config_.hybrid_astar.goal_yaw_tolerance_deg : 1.0;
    const double yaw_tol = std::max(grid_angle_rad_, configured_yaw_tol_deg * kPi / 180.0);
    return dist <= pos_tol && angle_diff <= yaw_tol;
}

uint64_t HybridAStar::CoordinateHash(const GridPoint& p) const {
    return (static_cast<uint64_t>(static_cast<uint32_t>(p.x)) << 32U) |
           static_cast<uint64_t>(static_cast<uint32_t>(p.y));
}

uint64_t HybridAStar::VertexHash(const Vertex3D& node) const {
    int32_t x_bin = static_cast<int32_t>(std::floor(node.x / grid_dist_));
    int32_t y_bin = static_cast<int32_t>(std::floor(node.y / grid_dist_));
    int32_t angle_bin = static_cast<int32_t>(std::floor((NormalizeAngle(node.yaw) + kPi) / grid_angle_rad_));
    if (angle_bin >= angle_bin_count_) {
        angle_bin %= angle_bin_count_;
    }
    if (angle_bin < 0) {
        angle_bin = (angle_bin % angle_bin_count_ + angle_bin_count_) % angle_bin_count_;
    }

    uint64_t hash = 0U;
    if (node.direction == MotionDirection::Backward) {
        hash |= 0x8000000000000000ULL;
    }
    if (x_bin < 0) {
        hash |= 0x2000000000000000ULL;
        x_bin = -x_bin;
    }
    if (y_bin < 0) {
        hash |= 0x1000000000000000ULL;
        y_bin = -y_bin;
    }
    hash |= (static_cast<uint64_t>(angle_bin) & 0xFFFULL) << 48U;
    hash |= (static_cast<uint64_t>(x_bin) & 0xFFFFFFULL) << 24U;
    hash |= static_cast<uint64_t>(y_bin) & 0xFFFFFFULL;
    return hash;
}

double HybridAStar::NormalizeAngle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

std::string HybridAStar::Trim(const std::string& text) {
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

void HybridAStar::ExpandNode(const Vertex3D& current, const Pose2D& start, const Pose2D& goal) {
    const int32_t steering_num = std::max(2, static_cast<int32_t>(std::round(config_.hybrid_astar.angle_discrete_num)));
    const double min_steering_rad = config_.hybrid_astar.min_steering * kPi / 180.0;
    const double max_steering_rad = config_.hybrid_astar.max_steering * kPi / 180.0;
    const double delta_steering = (max_steering_rad - min_steering_rad) / static_cast<double>(steering_num - 1);

    bool allow_backward = config_.hybrid_astar.allow_reverse;
    if (allow_backward && config_.hybrid_astar.backward_search_range > 0.0) {
        const double dist2_start_sq = (current.x - start.x) * (current.x - start.x) + (current.y - start.y) * (current.y - start.y);
        const double range_sq = config_.hybrid_astar.backward_search_range * config_.hybrid_astar.backward_search_range;
        if (dist2_start_sq > range_sq) {
            allow_backward = false;
        }
    }

    for (int32_t i = 0; i < steering_num; ++i) {
        const double steering = min_steering_rad + static_cast<double>(i) * delta_steering;
        const MotionDirection begin_dir = MotionDirection::Forward;
        const MotionDirection end_dir = allow_backward ? MotionDirection::Backward : MotionDirection::Forward;

        for (int32_t d = static_cast<int32_t>(begin_dir); d <= static_cast<int32_t>(end_dir); ++d) {
            const MotionDirection direction = static_cast<MotionDirection>(d);
            Vertex3D succ = Propagate(current, direction, steering);
            succ.id = VertexHash(succ);
            if (closed_set_.find(succ.id) != closed_set_.end()) {
                continue;
            }
            if (motion_collision_hook_) {
                Pose2D from;
                from.x = current.x;
                from.y = current.y;
                from.yaw = current.yaw;
                from.direction = current.direction;
                Pose2D to;
                to.x = succ.x;
                to.y = succ.y;
                to.yaw = succ.yaw;
                to.direction = succ.direction;
                if (motion_collision_hook_(from, to)) {
                    continue;
                }
            } else if (IsCollision(succ)) {
                continue;
            }

            succ.g = TransitionCost(current, succ);
            succ.h = Heuristic(succ, goal);
            succ.f = succ.g + succ.h;

            auto open_it = open_set_.find(succ.id);
            if (open_it == open_set_.end() || succ.g < open_it->second.g) {
                open_set_[succ.id] = succ;
                best_nodes_[succ.id] = succ;
                open_queue_.push(OpenEntry{succ.f, succ.id});
            }
        }
    }
}

HybridAStar::Vertex3D HybridAStar::Propagate(const Vertex3D& start, MotionDirection direction, double steering_rad) const {
    Vertex3D end;
    end.parent_id = start.id;
    end.direction = direction;

    const double dir_sign = (direction == MotionDirection::Forward) ? 1.0 : -1.0;
    if (std::fabs(steering_rad) < 1e-6) {
        end.x = start.x + dir_sign * config_.hybrid_astar.delta_dist * std::cos(start.yaw);
        end.y = start.y + dir_sign * config_.hybrid_astar.delta_dist * std::sin(start.yaw);
        end.yaw = NormalizeAngle(start.yaw);
    } else {
        const double wheel_base = std::max(1e-3, config_.vehicle.wheel_base);
        const double beta = config_.hybrid_astar.delta_dist / wheel_base * std::tan(steering_rad);
        const double radius = config_.hybrid_astar.delta_dist / beta;
        // [planner-adapter 扩展] 默认保持上游原行为；standard_reverse_kinematics
        // 打开时倒车使用标准自行车模型（yaw 变化 -beta、精确圆弧位移）。
        double yaw_delta = beta;
        double position_sign = dir_sign;
        if (config_.hybrid_astar.standard_reverse_kinematics) {
            yaw_delta = dir_sign * beta;
            position_sign = 1.0;
        }
        end.x = start.x + position_sign * radius * (std::sin(start.yaw + yaw_delta) - std::sin(start.yaw));
        end.y = start.y + position_sign * radius * (std::cos(start.yaw) - std::cos(start.yaw + yaw_delta));
        end.yaw = NormalizeAngle(start.yaw + yaw_delta);
    }
    return end;
}

double HybridAStar::TransitionCost(const Vertex3D& from, const Vertex3D& to) const {
    const double dx_start = from.x - search_start_pose_.x;
    const double dy_start = from.y - search_start_pose_.y;
    const double dist_to_start_sq = dx_start * dx_start + dy_start * dy_start;
    const double coeff = (dist_to_start_sq > config_.hybrid_astar.linear_preferred_distance_square) ? 1.0 : 5.0;
    const double heading_change = std::fabs(NormalizeAngle(to.yaw - from.yaw));
    const double direction_cost =
        (to.direction == MotionDirection::Backward) ? config_.hybrid_astar.backward_penalty : config_.hybrid_astar.forward_penalty;
    const double switch_cost =
        (to.direction == from.direction) ? 0.0 : config_.hybrid_astar.switch_penalty;

    return from.g + config_.hybrid_astar.delta_dist * direction_cost +
           coeff * config_.hybrid_astar.turnning_penalty * heading_change + switch_cost;
}

double HybridAStar::Heuristic(const Vertex3D& node, const Pose2D& goal) const {
    const double euclid = std::hypot(node.x - goal.x, node.y - goal.y);
    const double yaw_diff = std::fabs(NormalizeAngle(node.yaw - goal.yaw));
    return euclid * std::max(1.0, config_.hybrid_astar.forward_penalty) +
           yaw_diff * config_.hybrid_astar.turnning_penalty;
}

bool HybridAStar::ReconstructPath(uint64_t final_id, HybridSearchResult* result) const {
    std::vector<Pose2D> reversed;
    uint64_t current_id = final_id;
    while (true) {
        const auto it = best_nodes_.find(current_id);
        if (it == best_nodes_.end()) {
            return false;
        }
        Pose2D pose;
        pose.x = it->second.x;
        pose.y = it->second.y;
        pose.yaw = it->second.yaw;
        pose.direction = it->second.direction;
        reversed.push_back(pose);

        if (it->second.parent_id == 0 || it->second.parent_id == current_id) {
            break;
        }
        current_id = it->second.parent_id;
    }
    std::reverse(reversed.begin(), reversed.end());
    result->path = reversed;
    return !result->path.empty();
}

} // namespace astar
