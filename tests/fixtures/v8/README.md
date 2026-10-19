# tests/fixtures/v8：E0 固定夹具（v8 修订版：停车基准 115 场景集）

本目录保存跨组件复用的版本化测试输入。v8 = v7 全量复制（不含 v7 清单）+ 新增论文停车基准 115 场景集与论文车辆模型；v1–v7 保持冻结不动。

## v8 修订说明（相对 v7）

| 文件 | v8 变更 | 原因 |
| --- | --- | --- |
| `paths/cases_bench_parking.json` | 新增（115 场景） | 停车基准套件 v3 场景集，供 `planner_hybrid_astar_bench --suite v3` 消费 |
| `vehicles/parking_bench.json` | 新增 | 论文车辆精确参数（安全边距 0） |
| `README.md`、`index.json` | 修订（v8 说明） | 版本化修订记录 |
| `fixture_manifest.sha256.json` | 由 `fixtures freeze` 生成 | 清单不含自身 |
| 其余文件 | 与 v7 逐字节一致 | 避免无关变更 |

### 停车场景集 `paths/cases_bench_parking.json`

- 来源：ParkingMotionPlanningTITS21 `ParkingBenchmarks/CaseNo_1..115.mat`（IEEE TITS 2022 论文基准）；`.mat` → 中立 JSON → 本夹具的两段转换分别由 `cpp/tools/export_cases.py`（scipy，一次性）与 `tools/parking_fixtures.py`（标准库）完成。
- 画布：论文画布 40×40 m（[-20,20]²），本夹具每边外扩 0.1 m（一格）为 402×402 格 @0.1 m，origin (-20.1,-20.1)、row0 底行、yaw 0；`map_version=v8`、`grid.map_id=parking_case_<i>` 唯一。外扩原因：源数据 2/115 例（case_22 x=20.021、case_28 x=-20.052）起点矩形包络擦出论文边界 2.1/5.2 cm，冻结碰撞契约（地图矩形之外=碰撞）需外扩一格才能让 115 例全部可评估；越界障碍照常按画布外=占据裁剪。
- 车辆：`vehicles/parking_bench.json`（论文参数：轴距 2.8 m、宽 1.942 m、前悬 0.96 m、后悬 0.929 m、满舵 0.75 rad、安全边距 0；包络 4.689×1.942 m）。
- 场景：`id=case_1..case_115`、`class=parking`、`seed=901..1015`；每例 5 个凸多边形障碍（源数据随机布设）；start/goal 取自 `.mat` 边界位姿（yaw 保持源值，可能超出 (-π,π]，消费方按 `normalize_angle_rad` 语义处理）。
- 期望：统一 `expected.status=success`（论文基线）；本套件为探索性对比，失败（no_path/timeout）只记录，不参与 v1 正式选型门槛。

### 自检（生成时）

- 索引连续：case_1..case_115 连续。
- 画布：全部 115 例同一画布：402×402 格 @0.1 m（40.2×40.2 m），origin (-20.1,-20.1)、row0 底行、yaw 0；map_version=v8、map_id 唯一。论文画布为 40×40 m（[-20,20]²）；此处每边外扩 0.1 m（一格）——源数据中 2/115 例（case_22 x=20.021、case_28 x=-20.052）起点矩形包络擦出论文边界 2.1/5.2 cm，冻结碰撞契约把“地图矩形之外”判为碰撞，外扩一格保证 115 例全部可被评估；越界障碍照常按画布外=占据裁剪。
- 包络画布检查：起终点包络足迹角点全部在画布内 115/115。
- 障碍越界统计：源数据障碍越出画布为常态：186/575 个多边形（最大出界 9.542 m），越界部分按“地图矩形之外视为占据”语义裁剪，与 MATLAB 侧 costmap 裁剪一致。
- 凸性：多边形凸性由导出器（cpp/tools/export_cases.py）逐一校验通过。
- 正式碰撞检查器起终点预检（planner_hybrid_astar_bench --suite v3 --precheck）：115/115 例起终点足迹无碰撞。

### 生成与冻结

```sh
python3 tools/parking_fixtures.py generate \
    --input <repo>/ParkingMotionPlanningTITS21/parking_cases.json --force
python3 tools/task_runner.py fixtures freeze --version v8
python3 tools/task_runner.py fixtures verify --version v8
```

其余集合（grids/serialization/collision/benchmarks/playback/map 与 v1–v6 的 paths/cases.json）与 v7 逐字节一致，说明见 v7 目录及其 index。
