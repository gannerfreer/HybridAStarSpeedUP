# tests/fixtures/v6：E0 固定夹具（v6 修订版：栅格协议夹具与显示档）

本目录保存跨组件复用的版本化测试输入。v6 在 v5 基础上新增/修订 `grids/` 集合（栅格协议
夹具、手算三态夹具与显示档快照）；其余文件与 v5 逐字节一致（不含 v5 清单）。本目录由
`tools/grid_fixtures.py` 确定性生成并自检；`fixtures freeze/verify`（哈希清单校验）经统一
入口执行。手算数值与显示档 sha256/spot_checks 均由生成器独立复算核对，未抄录文档数值。

## v6 修订说明（相对 v5，2026-10-12）

v6 = v5 全量复制（不含 v5 的 `fixture_manifest.sha256.json`，由 v6 freeze 重新生成）
+ 本表修订；v1–v5 保持冻结不动。

| 文件 | v6 变更 | 原因 |
| --- | --- | --- |
| `grids/minimal.json` | 与 v5 逐字节一致（复制） | 保持既有公共契约测试与消费方路径 |
| `grids/rotated_origin_ccw.json`、`rotated_origin_cw.json`、`edge_cells.json`、`unknown_edges.json`、`obstacle_overlay.json`、`coord_conversion.json`、`pattern_64.json` | 新增（手算 `grid_view` 夹具 G1–G8 中的 7 个） | 旋转原点、边缘单元、未知、障碍叠加、行列↔世界坐标 |
| `grids/snapshot_small.json` | 新增（小型 `grid_snapshot`，G7） | 协议解码黄金样例（base64 三态 + 长度自检） |
| `grids/protocol_cases.json` | 新增（P1–P13） | 空窗口、超限/降级、预览 vs 规划逐格一致、请求乱序与陈旧拒绝 |
| `grids/display_spec.json`、`display_256.json`、`display_512.json`、`display_1024.json` | 新增（显示档 + 生成规范） | 显示性能档位 256²/512²/1024² |
| `index.json`、`README.md` | 修订（v6 说明、`grids` 集合描述、revision_history/notes） | 版本化修订记录（v5 体例） |
| `fixture_manifest.sha256.json` | 由 freeze 重新生成 | 清单不含自身；verify 拒绝清单外文件 |
| 其余文件 | 与 v5 逐字节一致 | 避免无关变更 |

### SHA256 寻址归档约定（§9，v5 起）

- 单文件 >10 MiB 的生成输入存放于 `artifacts/data/sha256/<sha256>/`（不入 Git）：
  `<scale>.osm`、`<scale>.sha256`（sha256sum 格式）、`<scale>.meta.json`（生成规则、
  spec archive 记录与校验结果）；`specs.json` 的 `archive.relative_path` 以仓库根为基准，
  `archive.sha256` 为文件字节哈希。
- 离线归档必须包含被引用文件；生成器每次生成后校验生成文件 SHA256 与 `archive.sha256`
  一致（不一致时报错退出，不静默继续）。
- v6 显示档最大约 1.4 MiB（1024²），直接入库，无归档需求。

消费方迁移：

| 消费方 | v6 使用 | 回退/说明 |
| --- | --- | --- |
| X-W1 固定夹具渲染 | `grids/display_*`、`snapshot_small.json`、手算夹具 | 回退 v5 `grids/minimal.json`；不依赖任何服务 |
| X-M1/X-W2 协议联调 | `grids/protocol_cases.json`、`snapshot_small.json` + v3 地图夹具 | 地图仍用 `tests/fixtures/v3/map/mapping_example.osm` |
| 公共契约测试 | 现有版本路径 | v6 `minimal.json` 与 v5 逐字节一致，不要求迁移 |
| `planner_map_bench` | 维持 v5 `benchmarks/map/specs.json` | v6 benchmarks 与 v5 逐字节一致，优先级不变 |
| Web W4 回放 | 维持 v5 `playback/` | v6 逐字节复制 |

## 目录结构

| 路径 | 内容 | 状态 |
| --- | --- | --- |
| `index.json` | 夹具索引：集合清单与说明；不内嵌哈希 | v6 修订 |
| `grids/minimal.json` | GridView 最小样例（20×20、0.1 m，三态 + 边界） | v5 复制 |
| `grids/rotated_origin_ccw.json` | G1 旋转原点（CCW，π/2，4×4、0.5 m） | v6 新增 |
| `grids/rotated_origin_cw.json` | G2 旋转原点（CW，-π/2，4×4、0.5 m） | v6 新增 |
| `grids/edge_cells.json` | G3 边缘单元与越界（4×4、1.0 m） | v6 新增 |
| `grids/unknown_edges.json` | G4 unknown 与三态（5×5、0.5 m） | v6 新增 |
| `grids/obstacle_overlay.json` | G5 障碍叠加（6×4、1.0 m，矩形 + 凸多边形） | v6 新增 |
| `grids/coord_conversion.json` | G6 行列↔世界坐标（6×5、0.25 m） | v6 新增 |
| `grids/snapshot_small.json` | G7 小型 `grid_snapshot`（3×2、0.1 m，base64 三态） | v6 新增 |
| `grids/pattern_64.json` | G8 64² 正确性小图（0.1 m，三态 + 边界 + %5 规则） | v6 新增 |
| `grids/protocol_cases.json` | P1–P13 协议用例（错误/降级/一致性/陈旧） | v6 新增 |
| `grids/display_spec.json` | 显示档生成规范（splitmix64_thresholds_v1 + sha256 + spot_checks） | v6 新增 |
| `grids/display_256.json` | 显示档 256²、0.1 m、seed 20261012 | v6 新增 |
| `grids/display_512.json` | 显示档 512²、0.1 m、seed 20261013 | v6 新增 |
| `grids/display_1024.json` | 显示档 1024²、0.1 m、seed 20261014 | v6 新增 |
| `vehicles/ paths/ collision/ serialization/ benchmarks/ playback/ map/` | 与 v5 逐字节一致 | v5 复制 |

## JSON 顶层约定

- 每个 JSON 文件为 UTF-8；顶层含整数 `schema_version: 1` 与字符串 `type`。
- 对象 ID 一律使用字符串；枚举使用字符串；拒绝 NaN/Infinity；数值保存足以往返 double 的精度。
- 单位后缀约定：`_m`（米）、`_rad`（弧度）、`_s`（秒）、`_mps`、`_mps2`、`_inv_m`（1/米，曲率）。
- 必填字段缺失或主版本未知应报错；未知可选字段允许读取，但不得改变已有字段含义。
- 引用其他夹具文件时，除非文件内另有说明，使用以 `tests/fixtures/v6/` 为根的相对路径；
  `playback/index.json` 中的文件名相对 `playback/` 目录。
- `playback/` 内文件哈希对保存后的原始文件字节计算（sha256sum 格式小写十六进制），
  不要求不同语言生成相同文本布局。
- 栅格：`grid_view` 用整数数组（行主序，0=free、1=occupied、2=unknown）；`grid_snapshot`
  用 `base64_u8`（协议 §2.3）。`origin` 为 cell (0,0) 左下角位姿（ROS OccupancyGrid 约定），
  cell 中心 = `origin + R(yaw)·((c+0.5)·res, (r+0.5)·res)`；unknown 与地图外视为占据。

## 状态与待补

- `grids/`：G1–G8 与 P1–P13 由生成器输出后通过冻结字段校验与独立复算；显示档
  sha256/spot_checks 强制复验；X-W1 渲染验收抽查 `snapshot_small.json` 与手算夹具
  （图例/坐标/点击查询），X-M1 复核 P1–P13 服务侧语义。
- 计划文本偏差（已在生成器与夹具 notes 记录）：G5 `obs-d` 三角形计划底边 y=2.5 与
  第 2 行单元中心共线，按闭合边界语义会多占 (2,0)，与计划期望 8 格不符；生成器以
  计划期望 cells 为准，底边上移 0.1 m 为 y=2.6。
- `paths/`、`collision/`、`benchmarks/`、`playback/`、`map/`：与 v5 逐字节一致，状态
  沿用 v5 README。
- 本 README 描述的“生成”“自检”均为夹具生成状态；不表示任何服务已加载或算法已验证。

## 合成数据声明

`grids/` 中显示档与手算夹具为合成显示数据（`source: "fixture"` 或 `grid_view`），仅作
显示/解码/协议用例测试，不能作为算法正确性证据，也不代表任何上游实现的实际输出。
`playback/` 下全部文件标记 `synthetic: true`；`paths/`、`benchmarks/`、`collision/` 中的
数据为确定性测试输入或生成规范，同样不构成算法正确性结论。
