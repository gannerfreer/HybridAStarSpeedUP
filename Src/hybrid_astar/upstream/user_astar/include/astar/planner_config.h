#ifndef astar_PLANNER_CONFIG_H
#define astar_PLANNER_CONFIG_H

#include <string>

namespace astar {

struct VehicleConfig {
    double center2front = 1.0;
    double center2rear = 1.0;
    double center2side = 0.5;
    double safe_margin = 0.0;
    double wheel_base = 1.0;
    // Optional extra map boundary inflation. >= 0 means use this inflation meter directly; < 0 means no extra inflation.
    double boundary_inflation_meter = -1.0;
};

struct HybridAStarConfig {
    double min_steering = -30.0;
    double max_steering = 30.0;
    double max_iterations_astar = 100000.0;
    double angle_discrete_num = 8.0;
    double delta_dist = 1.0;
    double grid_dist = 1.0;
    double grid_angle = 1.0;
    double goal_yaw_tolerance_deg = 1.0;
    double forward_penalty = 1.0;
    double backward_penalty = 1.0;
    double turnning_penalty = 1.0;
    double switch_penalty = 1.0;
    double backward_search_range = 0.0;
    double linear_preferred_distance_square = 0.0;
    // [planner-adapter 扩展，默认 true = 原行为] false 时不再生成倒车原语。
    bool allow_reverse = true;
    // [planner-adapter 扩展，默认 false = 原行为] true 时倒车运动学使用
    // 标准自行车模型（yaw 变化取 -beta，位移取精确圆弧），供归一化后端使用。
    bool standard_reverse_kinematics = false;
};

struct PlannerConfig {
    VehicleConfig     vehicle;
    HybridAStarConfig hybrid_astar;
    struct StartGoalConfig {
        double start_x = 4.0;
        double start_y = 4.0;
        double start_yaw_deg = 0.0;
        int    start_direction = 0;
        double goal_x = 6.0;
        double goal_y = 6.0;
        double goal_yaw_deg = 90.0;
        int    goal_direction = 0;
    } start_goal;
};

bool LoadPlannerConfigFromIni(const std::string& ini_path, PlannerConfig* config, std::string* error_message);

} // namespace astar

#endif // astar_PLANNER_CONFIG_H
