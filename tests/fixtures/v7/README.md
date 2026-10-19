# tests/fixtures/v7：E0 固定夹具（v7 修订版：hybrid A* 基准套件 v2 场景集）

本目录保存跨组件复用的版本化测试输入。v7 在 v6 基础上新增 `paths/cases_bench_v2.json`
（36 场景基准集、统一画布），其余文件与 v6 逐字节一致（不含 v6 的清单）。场景集由
夹具设计代理编写，并以独立 Python 脚本校验（结论见 `paths/cases_bench_v2.json` 的
`self_check.validation`）；`fixtures freeze/verify`（哈希清单校验）经统一入口执行。

## v7 修订说明（相对 v6，2026-10-16）

v7 = v6 全量复制（不含 v6 的 `fixture_manifest.sha256.json`，由 v7 freeze 重新生成）
+ 本表修订；v1–v6 保持冻结不动。

| 文件 | v7 变更 | 原因 |
| --- | --- | --- |
| `paths/cases_bench_v2.json` | 新增（36 场景 = 6 类 × 6 变体，统一画布 300×160 @0.1 m） | Hybrid A* 基准套件 v2（统一栅格档实验）场景集，供 `planner_hybrid_astar_bench --suite v2` 消费 |
| `index.json`、`README.md` | 修订（v7 说明、`paths` 集合描述、revision_history/notes） | 版本化修订记录（v5/v6 体例） |
| `fixture_manifest.sha256.json` | 由 freeze 重新生成（73 个文件） | 清单不含自身；verify 拒绝清单外文件 |
| 其余文件 | 与 v6 逐字节一致 | 避免无关变更 |

### 基准场景集 `paths/cases_bench_v2.json`（v7 新增）

- 画布：全部 36 例同一画布 300×160 格 @0.1 m（30×16 m），origin (0,0)、row0 底行、
  yaw 0；`grid.map_id = hybrid_bench_v2_<id>` 唯一、`grid.map_version = "v7"`。
- 车辆：默认测试车辆（包络 4.9×2.0 m、最小转弯半径 R = 2.8/tan(0.5) ≈ 5.125 m）。
- 场景（6 类 × 6 变体，id `b2-<class>-01..06`，seed 701–736）：

| 类 | 变体要点 | 期望 |
| --- | --- | --- |
| `open` | 6 种不同起终点位置/朝向（东西直线、南北直线、对角 45°、对头、跨区斜向、朝南停靠） | 6× success |
| `obstacle` | 居中块、偏左/偏右侧方障碍、两段交错侧墙、凸四边形、两墙 3.5 m 门洞 | 6× success |
| `corridor` | 直通道、错位（+0.5 m）、S 形（上 0.5/下 0.5）、侧向进出口（7 m 侧门）、入口横移、最窄 2.6 m 通道 | 6× success |
| `park` | 上向车位（含镜像与纯倒车变体）、下向车位（含旋转 180° 镜像）、平行车位；起点朝向覆盖东/西/南，均倒车入位 | 6× success |
| `deadend` | 口部朝下/朝上/朝西/朝东的袋形位，另含漏斗形与 L 形支路，均倒车退出 | 6× success |
| `unreachable` | 封闭房间、全高隔断、1.5 m 缺口、实体环、堵死门洞、1.6 m 缺口隔断 | 6× no_path |

- 独立校验（`self_check.validation`，脚本仅标准库）：①起终点包络足迹无碰撞且在界内
  36/36；②障碍几何合法 36/36；③通道净宽（半径 1.0 m 圆盘最大最小间隙路径）30/30
  success 连通、最窄瓶颈净宽 2.30 m ≥ 2.2 m；④unreachable 封闭性 6/6（最大瓶颈净宽
  1.50 m）；⑤36 例画布全等。结论 `passed`。侧方类场景共 8 例（obstacle-02/03/04、
  corridor-04/05、park-06、deadend-03/04）。
- 各例可行构造机动（直线段 + 四分之一圆弧，R≈5.125 m）与逐段包络间隙见各例 `notes`；
  为手写几何论证，未运行规划器，实测可解性由基准套件 v2 运行记录。
- 消费方：`planner_hybrid_astar_bench --suite v2 --fixtures-dir {repo_root}/tests/fixtures/v7`
  （见 `Src/hybrid_astar/README.md`「基准套件 v2」）；schema 与 `paths/cases.json` 一致。

消费方迁移：

| 消费方 | v7 使用 | 回退/说明 |
| --- | --- | --- |
| Hybrid A* 基准套件 v2（实验） | `paths/cases_bench_v2.json` + `vehicles/default.json` | 与 v1 正式选型并存；无回退需求 |
| Hybrid A* 既有测试/基准 v1 | 维持 v2–v6 的 `paths/cases.json`（逐字节一致） | v7 `paths/cases.json` 与 v6 逐字节一致 |
| X-W1/X-M1/X-W2、公共契约、planner_map_bench、Web W4 | 维持既有版本路径 | v7 对应文件与 v6 逐字节一致 |

## 目录结构

| 路径 | 内容 | 状态 |
| --- | --- | --- |
| `index.json` | 夹具索引：集合清单与说明；不内嵌哈希 | v7 修订 |
| `paths/cases_bench_v2.json` | Hybrid A* 基准套件 v2 场景集（36 例、统一画布、30 success + 6 no_path） | v7 新增 |
| `paths/cases.json` | 路径规划固定场景（六类×5 + 4 边界例，v2 修订） | v6 复制 |
| `grids/` | 栅格夹具（v6 新增/修订的 14 个文件：手算 G1–G8、协议 P1–P13、snapshot、显示档 256²/512²/1024²） | v6 复制 |
| `vehicles/ collision/ serialization/ benchmarks/ playback/ map/` | 与 v6 逐字节一致 | v6 复制 |
| `fixture_manifest.sha256.json` | freeze 生成的哈希清单（不含自身） | v7 重新生成 |

## JSON 顶层约定

- 每个 JSON 文件为 UTF-8；顶层含整数 `schema_version: 1` 与字符串 `type`。
- 对象 ID 一律使用字符串；枚举使用字符串；拒绝 NaN/Infinity；数值保存足以往返 double 的精度。
- 单位后缀约定：`_m`（米）、`_rad`（弧度）、`_s`（秒）、`_mps`、`_mps2`、`_inv_m`（1/米，曲率）。
- 必填字段缺失或主版本未知应报错；未知可选字段允许读取，但不得改变已有字段含义。
- 引用其他夹具文件时，除非文件内另有说明，使用以 `tests/fixtures/v7/` 为根的相对路径；
  `playback/index.json` 中的文件名相对 `playback/` 目录。
- `playback/` 内文件哈希对保存后的原始文件字节计算（sha256sum 格式小写十六进制），
  不要求不同语言生成相同文本布局。
- 栅格：`grid_view` 用整数数组（行主序，0=free、1=occupied、2=unknown）；`grid_snapshot`
  用 `base64_u8`（协议 §2.3）。`origin` 为 cell (0,0) 左下角位姿（ROS OccupancyGrid 约定），
  cell 中心 = `origin + R(yaw)·((c+0.5)·res, (r+0.5)·res)`；unknown 与地图外视为占据。
- `paths/cases_bench_v2.json` 的 `grid` 只给几何；占据格由 `obstacles` 按“接触即占据”
  栅格化得到，无 unknown 格，地图外视为占据（与 `paths/cases.json` 约定一致）。

## 状态与待补

- `paths/cases_bench_v2.json`：36 例由独立 Python 脚本校验通过（见 `self_check.validation`
  与 `self_check.checks`）；可行构造为手写几何论证，未运行规划器；实测分类与计时由
  Hybrid A* 基准套件 v2 运行记录。
- `grids/`：G1–G8 与 P1–P13 由生成器输出后通过冻结字段校验与独立复算；显示档
  sha256/spot_checks 强制复验；状态沿用 v6 README。
- `paths/`、`collision/`、`benchmarks/`、`playback/`、`map/`：与 v6 逐字节一致，状态
  沿用 v6 README。
- 本 README 描述的“生成”“自检”均为夹具状态；不表示任何服务已加载或算法已验证。

## 合成数据声明

`grids/` 中显示档与手算夹具为合成显示数据（`source: "fixture"` 或 `grid_view`），仅作
显示/解码/协议用例测试，不能作为算法正确性证据，也不代表任何上游实现的实际输出。
`paths/cases_bench_v2.json` 为确定性合成测试输入（统一画布基准场景），不构成算法正确性
结论。`playback/` 下全部文件标记 `synthetic: true`；`paths/`、`benchmarks/`、`collision/`
中的数据为确定性测试输入或生成规范，同样不构成算法正确性结论。
