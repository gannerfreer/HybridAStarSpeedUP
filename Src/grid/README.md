# planner_grid（通用栅格：坐标换算、窗口单元数与预算降级数学、三态校验）

与显示和地图实现无关的通用栅格数学与校验库（`planner::grid`，X-A1-17 提取）。
边界（冻结计划 §2.1）：**只依赖 `planner::common`**（`GridView` 按冻结补充暂留
common），不暴露 Lanelet2 或任何上游类型，不运行任何搜索；全部能力以自由函数
提供。坐标约定沿用 `common::GridView` 的冻结 v1 契约（与 ROS OccupancyGrid
一致，见 [common README](../common/README.md)）。

当前状态：测试 15/15（标签 `grid`，期望值全部手工推导）；被 map（窗口尺寸数学、
服务单元预算）、scene（预算降级与坐标助手）复用；无基准套件。

## 目录

```text
Src/grid/
  CMakeLists.txt                       组件构建（planner_grid，PLANNER_BUILD_GRID 默认 ON）
  include/planner/grid/grid.hpp        公开接口：CellIndex、坐标换算、窗口/预算数学、校验
  src/grid.cpp                         实现（int 边界防护、32 次降级 guard、校验原因串）
  tests/
    test_grid.cpp                      GoogleTest（标签 grid，15 例，手工期望值）
```

## 公开接口

- `CellIndex{row, col}`（行沿 +y、列沿 +x，`(0,0)` 为左下角）与 `==`/`!=`。
- `world_to_cell(grid, x_m, y_m)` → `std::optional<CellIndex>`：世界坐标到包含该
  点的单元（floor 语义，**不夹取**，边界点属于上侧单元）；非有限输入、空栅格、
  非有限或非正分辨率、非有限原点、索引超出 int 可表示范围时返回 `nullopt`。
- `world_to_cell_clamped(grid, x_m, y_m)` → `CellIndex`：同 `world_to_cell` 后夹取
  到 `[0, width-1] × [0, height-1]`；空栅格或输入不可映射时返回 `{0, 0}`
  （调用方负责先校验位姿在窗口内）。
- `cell_center(grid, row, col)` → `common::Point2d`：单元中心世界坐标（含
  `origin.yaw_rad` 旋转；不做范围检查）。
- `ceil_cell_count(span_m, resolution_m)` → `int`：覆盖长度所需单元数
  `ceil(span/resolution)`；参数非有限或非正时返回 0，结果超出 int 表示范围时也
  返回 0（由调用方按预算拒绝）。
- `WindowGridSpec{width, height, cells}` 与 `window_grid_spec(width_m, height_m,
  resolution_m)`：轴对齐窗口的栅格尺寸（ceil 语义，`cells = width * height`）；
  参数无效或尺寸无法用 int/size_t 表示时返回 `nullopt`。
- `CellBudgetFit{resolution_m, cells, within_budget}` 与
  `fit_resolution_to_cell_budget(width_m, height_m, resolution_m, max_cells)`：
  单元预算降级——分辨率按 2 的幂增大直至单元数不超过预算（最多翻倍 32 次）；
  `max_cells=0` 按 1 处理；参数无效时 `within_budget=false` 且 `resolution_m`
  为请求值；`cells` 仅在预算内且可安全转换时给出，极端参数下保持 0。
- `cell_state_valid(value)`：三态取值校验（0=空闲、1=占据、2=未知）。
- `valid_grid(grid, reason*)`：栅格数据校验——`width`/`height` 必须都为零（无
  栅格）或都为正；`resolution_m` 有限且为正；`origin` 三个分量有限；`cells`
  长度等于 `width × height`；每个单元取值 ∈ {0,1,2}。空栅格（0×0，`cells` 为空）
  视为有效；`reason` 可选接收首个错误说明（英文短句）。

## 坐标与三态约定（与 GridView 冻结契约一致）

- `origin` 是单元 `(0,0)` 左下角的位姿；`world_to_cell` 先做逆旋转
  （世界 → 原点坐标系）再按 `floor(local/resolution)` 取行列，因此带
  `yaw_rad` 的栅格同样成立（`GridWorldToCellTest.RespectsOriginYaw` 固定该语义）。
- `cell_center` 是逆变换：局部 `((col+0.5)·res, (row+0.5)·res)` 经原点旋转平移回
  世界系；单元中心往返可回到同一单元（测试固定）。
- 单元取值 0/1/2 与消费方规则（未知与地图外视为占据）不在本库实现，只做校验；
  三态常量来自 `planner::common`。

## 窗口单元数与预算降级

- 单元数按 `ceil(width/res) × ceil(height/res)` 计算（与 map/scene 的窗口语义
  一致）；`fit_resolution_to_cell_budget` 只在超预算时**增大**分辨率
  （`effective >= requested`），最多 32 次翻倍，超限返回 `within_budget=false`
  由调用方映射为稳定错误码（map 服务 `grid_too_large`、scene
  `window_too_large`）。
- 32 次 guard、`max_cells=0→1`、无效参数与 int 溢出行为都有显式用例
  （`GridBudgetFitTest.*`、`GridCeilCellCountTest.*`）。
- 本库不分配栅格数据、不做窗口导出；`GridView` 的分配与内容由 map/scene 负责。

## 复用关系

- **map 库**（`Src/map/src/map.cpp`）：`Map::export_driveable_grid` 的窗口尺寸
  与单元预算复用 `grid::window_grid_spec`（X-A1-17 起）。
- **地图服务**（`Src/map/src/service_main.cpp`）：`/map/grid/driveable` 端点的
  降级复用 `grid::fit_resolution_to_cell_budget`（X-M1）；`/map/scene/preview`
  经 `planner::scene` 内部使用同一助手。
- **scene 库**（`Src/scene`）：预算降级与坐标助手以 PRIVATE 链接消费本库；
  场景组合的窗口/分辨率语义与本库一致。
- 依赖方通过 `planner::grid` 目标链接；本库不反向依赖任何组件。

## 复现命令

```sh
python3 tools/task_runner.py build --component grid --profile release --parallel 4
python3 tools/task_runner.py test --component grid --workers 2
```

`grid` 无注册基准套件（`tools/components.json` 的 `benchmark_suites` 为空）；
测试经 `ctest -L grid` 选中，仅依赖 `planner::common`，不读地图或显示夹具。

## 已知限制

1. `world_to_cell` 不夹取：栅格矩形之外返回负索引或超界索引，调用方自行裁剪
   （或使用 `world_to_cell_clamped`）；边界点固定归上侧单元。
2. `world_to_cell_clamped` 对不可映射输入返回 `{0, 0}` 而非错误信号，调用方必须
   先做有限性/范围校验，否则可能把非法输入静默当成角单元。
3. 预算降级的 32 次上限与 `cells` 的 0 值语义是文档化边界：极端参数下
   `within_budget` 仍有效，但单元数需由调用方以最终分辨率与导出结果为准。
4. `valid_grid` 只校验几何/形状/三态；`map_id`/`map_version` 等标识字符串的
   非空约束属于序列化契约，不在本库重复检查。
5. 无基准套件；本库为纯数学助手，性能随消费方负载一起测量。

## 报告

- 提取与验收（X-A1-17，15/15）：[`Docs/reports/expansion/X-A1.md`](../../Docs/reports/expansion/X-A1.md)
  （§3.2 独立构建/测试，build `9d304e44`、test `42a40650`；§3.4 全组件回归）。
  首次构建失败 `e676749e`（`cell_count_double` 参数不匹配）已按 §5.4 保留证据并
  定向修复后重试成功。
- 复用验收：map 68/68（复用 `grid::window_grid_spec`）与 X-M1 服务预算降级见
  [`Docs/reports/expansion/X-M1.md`](../../Docs/reports/expansion/X-M1.md)。
- 问题跟踪：[`Docs/reports/ISSUES.md`](../../Docs/reports/ISSUES.md)（无 `grid-*` 条目）。
