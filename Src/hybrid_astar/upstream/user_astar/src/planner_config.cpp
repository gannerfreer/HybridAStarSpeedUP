#include "astar/planner_config.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>

namespace astar {

namespace {

std::string Trim(const std::string& text) {
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = text.find_last_not_of(" \t\r\n");
    return text.substr(first, last - first + 1);
}

std::string ToLower(std::string text) {
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return text;
}

const std::string* FindValue(const std::unordered_map<std::string, std::string>& kv, const std::string& key) {
    auto it = kv.find(key);
    if (it != kv.end()) {
        return &it->second;
    }
    it = kv.find(ToLower(key));
    if (it != kv.end()) {
        return &it->second;
    }
    return nullptr;
}

bool ParseDouble(const std::unordered_map<std::string, std::string>& kv, const std::string& key, double* out, std::string* err) {
    const std::string* value_text = FindValue(kv, key);
    if (value_text == nullptr) {
        if (err != nullptr) {
            *err = "missing key: " + key;
        }
        return false;
    }
    try {
        std::size_t parsed = 0;
        const double value = std::stod(*value_text, &parsed);
        if (parsed != value_text->size()) {
            if (err != nullptr) {
                *err = "invalid number for key: " + key;
            }
            return false;
        }
        *out = value;
        return true;
    } catch (...) {
        if (err != nullptr) {
            *err = "invalid number for key: " + key;
        }
        return false;
    }
}

bool ParseBool(const std::unordered_map<std::string, std::string>& kv, const std::string& key, bool* out, std::string* err) {
    const std::string* value_text = FindValue(kv, key);
    if (value_text == nullptr) {
        if (err != nullptr) {
            *err = "missing key: " + key;
        }
        return false;
    }
    const std::string value = ToLower(Trim(*value_text));
    if (value == "true" || value == "1") {
        *out = true;
        return true;
    }
    if (value == "false" || value == "0") {
        *out = false;
        return true;
    }
    if (err != nullptr) {
        *err = "invalid bool for key: " + key;
    }
    return false;
}

bool ParseInt(const std::unordered_map<std::string, std::string>& kv, const std::string& key, int* out, std::string* err) {
    const std::string* value_text = FindValue(kv, key);
    if (value_text == nullptr) {
        if (err != nullptr) {
            *err = "missing key: " + key;
        }
        return false;
    }
    try {
        std::size_t parsed = 0;
        const int value = std::stoi(*value_text, &parsed);
        if (parsed != value_text->size()) {
            if (err != nullptr) {
                *err = "invalid int for key: " + key;
            }
            return false;
        }
        *out = value;
        return true;
    } catch (...) {
        if (err != nullptr) {
            *err = "invalid int for key: " + key;
        }
        return false;
    }
}

bool ParseOptionalDouble(
    const std::unordered_map<std::string, std::string>& kv, const std::string& key, double* out, std::string* err) {
    const std::string* value_text = FindValue(kv, key);
    if (value_text == nullptr) {
        return true;
    }
    try {
        std::size_t parsed = 0;
        const double value = std::stod(*value_text, &parsed);
        if (parsed != value_text->size()) {
            if (err != nullptr) {
                *err = "invalid number for key: " + key;
            }
            return false;
        }
        *out = value;
        return true;
    } catch (...) {
        if (err != nullptr) {
            *err = "invalid number for key: " + key;
        }
        return false;
    }
}

} // namespace

bool LoadPlannerConfigFromIni(const std::string& ini_path, PlannerConfig* config, std::string* error_message) {
    if (config == nullptr) {
        if (error_message != nullptr) {
            *error_message = "config is null";
        }
        return false;
    }

    std::ifstream ifs(ini_path);
    if (!ifs.is_open()) {
        if (error_message != nullptr) {
            *error_message = "cannot open ini: " + ini_path;
        }
        return false;
    }

    std::unordered_map<std::string, std::string> kv;
    std::string section;
    std::string line;
    while (std::getline(ifs, line)) {
        std::string t = Trim(line);
        if (t.empty() || t[0] == '#' || t[0] == ';') {
            continue;
        }
        if (t.front() == '[' && t.back() == ']') {
            section = Trim(t.substr(1, t.size() - 2));
            continue;
        }
        const auto pos = t.find('=');
        if (pos == std::string::npos) {
            continue;
        }
        const std::string key = Trim(t.substr(0, pos));
        const std::string value = Trim(t.substr(pos + 1));
        if (section.empty() || key.empty()) {
            continue;
        }
        const std::string scoped_key = section + "." + key;
        kv[scoped_key] = value;
        kv[ToLower(scoped_key)] = value;
    }

    std::string err;
    auto* v = &config->vehicle;
    auto* h = &config->hybrid_astar;
    auto* s = &config->start_goal;
    const bool ok =
        ParseDouble(kv, "Vehicle.center2front", &v->center2front, &err) &&
        ParseDouble(kv, "Vehicle.center2rear", &v->center2rear, &err) &&
        ParseDouble(kv, "Vehicle.center2side", &v->center2side, &err) &&
        ParseDouble(kv, "Vehicle.wheel_base", &v->wheel_base, &err) &&
        ParseDouble(kv, "Vehicle.safe_margin", &v->safe_margin, &err) &&
        ParseOptionalDouble(kv, "Vehicle.boundary_inflation_meter", &v->boundary_inflation_meter, &err) &&
        ParseDouble(kv, "HybridAStar.min_steering", &h->min_steering, &err) &&
        ParseDouble(kv, "HybridAStar.max_steering", &h->max_steering, &err) &&
        ParseDouble(kv, "HybridAStar.max_iterations_astar", &h->max_iterations_astar, &err) &&
        ParseDouble(kv, "HybridAStar.angle_discrete_num", &h->angle_discrete_num, &err) &&
        ParseDouble(kv, "HybridAStar.delta_dist", &h->delta_dist, &err) &&
        ParseDouble(kv, "HybridAStar.grid_dist", &h->grid_dist, &err) &&
        ParseDouble(kv, "HybridAStar.grid_angle", &h->grid_angle, &err) &&
        ParseOptionalDouble(kv, "HybridAStar.goal_yaw_tolerance_deg", &h->goal_yaw_tolerance_deg, &err) &&
        ParseDouble(kv, "HybridAStar.forward_penalty", &h->forward_penalty, &err) &&
        ParseDouble(kv, "HybridAStar.backward_penalty", &h->backward_penalty, &err) &&
        ParseDouble(kv, "HybridAStar.turnning_penalty", &h->turnning_penalty, &err) &&
        ParseDouble(kv, "HybridAStar.switch_penalty", &h->switch_penalty, &err) &&
        ParseDouble(kv, "HybridAStar.backward_search_range", &h->backward_search_range, &err) &&
        ParseDouble(kv, "HybridAStar.linear_preferred_distance_square", &h->linear_preferred_distance_square, &err) &&
        ParseDouble(kv, "StartGoal.start_x", &s->start_x, &err) &&
        ParseDouble(kv, "StartGoal.start_y", &s->start_y, &err) &&
        ParseDouble(kv, "StartGoal.start_yaw_deg", &s->start_yaw_deg, &err) &&
        ParseInt(kv, "StartGoal.start_direction", &s->start_direction, &err) &&
        ParseDouble(kv, "StartGoal.goal_x", &s->goal_x, &err) &&
        ParseDouble(kv, "StartGoal.goal_y", &s->goal_y, &err) &&
        ParseDouble(kv, "StartGoal.goal_yaw_deg", &s->goal_yaw_deg, &err) &&
        ParseInt(kv, "StartGoal.goal_direction", &s->goal_direction, &err);

    if (!ok && error_message != nullptr) {
        *error_message = err;
    }
    return ok;
}

} // namespace astar
