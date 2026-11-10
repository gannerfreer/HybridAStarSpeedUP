# TOML configuration

All configuration files use TOML, SI units, and a mandatory top-level
`schema_version = 1`. Unknown keys are rejected by default (including keys in
nested tables). `ConfigLoadOptions::allow_unknown_keys` can relax this for
forward-compatible deployments.

Loaders live in `planner::config`:

```cpp
auto vehicle = planner::config::load_vehicle_config("config/vehicles/standard_sedan.toml");
auto planner = planner::config::load_planner_config("config/planner/default.toml");
auto scenario = planner::config::load_scenario_config("config/scenarios/narrow_parking.toml");
if (!vehicle) {
  std::cerr << vehicle.error.message << "\n";
}
```

`ConfigError` contains `status`, `message`, `file`, `key_path`, `line`, and
`column`. Lines and columns are 1-based. No TOML exception escapes a loader.

## Vehicle: `config/vehicles/*.toml`

| Key | Type | Required | Default | Unit |
|---|---|---:|---|---|
| `schema_version` | integer | yes | — | — (must be `1`) |
| `id` | string | no | `""` | — |
| `wheelbase_m` | float | yes | — | m |
| `width_m` | float | yes | — | m |
| `front_overhang_m` | float | yes | — | m |
| `rear_overhang_m` | float | yes | — | m |
| `max_steer_rad` | float | yes | — | rad, open interval `(0, pi/2)` |
| `safety_margin_m` | float | no | `0.0` | m |

Validation is delegated to `common::vehicle_valid`: positive wheelbase and
width, non-negative overhangs/margin, and a maximum steering angle satisfying
`0 < max_steer_rad < pi/2`.

```toml
schema_version = 1
id = "standard_sedan"
wheelbase_m = 2.7
width_m = 1.8
front_overhang_m = 0.9
rear_overhang_m = 0.9
max_steer_rad = 0.55
safety_margin_m = 0.1
```

## Planner: `config/planner/*.toml`

All planner fields are optional and default to `common::UserAstarConfig`.
Nested TOML tables are mapped to the flat core structure.

| Key | Type | Default | Unit |
|---|---|---:|---|
| `schema_version` | integer | — (must be `1`) | — |
| `search.xy_resolution_m` | float | `0.5` | m |
| `search.heading_bins` | integer | `72` | — (>= 8) |
| `search.motion_step_m` | float | `1.0` | m |
| `search.steering_samples` | integer | `5` | — (>= 2) |
| `search.allow_reverse` | boolean | `true` | — |
| `search.backward_search_range_m` | float | `0.0` | m (`0` = unlimited) |
| `search.linear_preferred_distance_m2` | float | `0.0` | m² |
| `search.goal_capture_radius_m` | float | `8.0` | m |
| `cost.forward_cost` | float | `1.0` | — |
| `cost.reverse_cost` | float | `2.0` | — |
| `cost.turn_penalty` | float | `1.0` | — |
| `cost.switch_cost` | float | `2.0` | — |
| `analytic_expansion.ratio` | integer | `3` | — (>= 1) |
| `analytic_expansion.shot_distance_m` | float | `0.0` | m (`0` = unlimited) |
| `analytic_expansion.max_length_m` | float | `12.0` | m |
| `analytic_expansion.min_connector_segment_m` | float | `0.15` | m |
| `budget.default_time_budget_s` | float | `5.0` | s |
| `budget.default_node_budget` | integer | `1_000_000` | nodes |

`search.linear_preferred_distance_m2` maps directly to the core field
`UserAstarConfig::linear_preferred_distance_m2`. Validation is delegated to
`common::planner_config_valid`.

```toml
schema_version = 1

[search]
xy_resolution_m = 0.5
heading_bins = 72
motion_step_m = 1.0
steering_samples = 5
allow_reverse = true
goal_capture_radius_m = 8.0

[budget]
default_time_budget_s = 5.0
default_node_budget = 1_000_000
```

## Scenario: `config/scenarios/*.toml`

| Key | Type | Required | Default | Unit |
|---|---|---:|---|---|
| `schema_version` | integer | yes | — | — (must be `1`) |
| `map.width` | integer | yes | — | cells (> 0) |
| `map.height` | integer | yes | — | cells (> 0) |
| `map.resolution_m` | float | yes | — | m (> 0) |
| `map.origin.x_m` | float | no | `0.0` | m |
| `map.origin.y_m` | float | no | `0.0` | m |
| `map.origin.yaw_rad` | float | no | `0.0` | rad |
| `start.x_m` | float | yes | — | m |
| `start.y_m` | float | yes | — | m |
| `start.yaw_rad` | float | no | `0.0` | rad |
| `goal.x_m` | float | yes | — | m |
| `goal.y_m` | float | yes | — | m |
| `goal.yaw_rad` | float | no | `0.0` | rad |
| `obstacles[].id` | string | yes | — | — |
| `obstacles[].vertices[].x_m` | float | yes | — | m |
| `obstacles[].vertices[].y_m` | float | yes | — | m |

Each obstacle must have at least three distinct finite vertices forming a
strictly convex polygon. `collision::make_scene` converts the scenario into a
`collision::Scene` whose grid is all-free and whose obstacles are convex
polygon obstacles.

```toml
schema_version = 1

[map]
width = 80
height = 80
resolution_m = 0.5

[start]
x_m = 8.0
y_m = 8.0
yaw_rad = 0.0

[goal]
x_m = 32.0
y_m = 8.0
yaw_rad = 0.0

[[obstacles]]
id = "wall"
vertices = [
  { x_m = 19.5, y_m = 0.0 },
  { x_m = 20.5, y_m = 0.0 },
  { x_m = 20.5, y_m = 12.0 },
  { x_m = 19.5, y_m = 12.0 },
]
```

## Examples

- `config/vehicles/standard_sedan.toml`
- `config/vehicles/mini_agv.toml`
- `config/planner/default.toml`
- `config/planner/aggressive_parking.toml`
- `config/scenarios/narrow_parking.toml`

Run the end-to-end demo with:

```sh
./build/tools/plan_demo \
  --vehicle config/vehicles/standard_sedan.toml \
  --planner config/planner/default.toml \
  --scenario config/scenarios/narrow_parking.toml
```
