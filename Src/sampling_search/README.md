# planner_sampling_search（X-S4：二维 RRT* 多实现适配）

统一采样搜索契约与多来源适配（`planner::sampling_search`，依赖 `planner::common`；
公共 API 不暴露 OMPL / Ch0p1k3 / Klamp't / vss2sn / ai-winter 类型）。
本轮交付**正确性**（合成夹具、统一路径复检、多进程复现验证）；性能与正式基准属
X-B1，基准工具已就绪（见 [benchmarks/README.md](benchmarks/README.md)）。
组件不依赖地图服务与显示、不依赖 ROS。

- 来源与适配记录（URL/commit/许可证/补丁/原生偏差/验证 run）：[provenance.md](provenance.md)
- 上游最小抽取与补丁说明：`third_party/`（Ch0p1k3、nanoflann、vss2sn；
  `PATCHES.md` 与许可证随目录保留）

## 1. 契约与分组

- 输入：二维边界、起终点、统一点/边有效性回调、显式沿边步长、种子、长度目标、
  预算与取消；状态区分 `kSuccess`/`kNoPath`/`kTimeout`/`kCancelled`/`kInvalidInput`。
- 预算耗尽无解报 `kTimeout`（**不作为不可达证明**）；取消丢弃部分结果；返回路径
  统一经 `path_edges_valid` 复检，不满足原生沿边有效性/端点容差的解按无解处理。
- 起终点重合（≤1e-9）经公共助手返回单点成功路径（cost 0），不调用后端。
- 分组（候选登记 §4.1）：连续空间组 = OMPL（RRT*/Informed 单列）、Ch0p1k3、
  Klamp't；栅格采样组 = vss2sn（与连续组分开评分）；ai-winter（GPL-3.0）为
  独立原生组、适配延期（开关默认 OFF，显式置 ON 报错，见 provenance §6）。

## 2. 适配器

| 适配器 | 来源 | 许可证 | 关键口径 |
| --- | --- | --- | --- |
| `ompl_rrtstar` / `ompl_informed_rrtstar` | OMPL 1.6.0（锁条目 `ompl`） | BSD-3-Clause | `RNG::setSeed` 进程级种子；Informed 单列 |
| `ch0p1k3_rrtstar` | Ch0p1k3 抽取副本 + nanoflann | MIT | 保守栅格化、整数边界；种子钩子 4 行 |
| `klampt_rrtstar` | KrisLibrary（FetchContent 48 文件） | BSD-3-Clause | 原生 `rrt*` = RRG* 语义（披露）；bidirectional 固定 false |
| `vss2sn_rrtstar` | vss2sn/path_planning 抽取副本 | BSD-3-Clause | 栅格组；仅方阵栅格；每 trial 性能临界（见 §4） |
| `aiwinter_*`（延期） | ai-winter/ros_motion_planning | GPL-3.0 | 适配延期；默认 OFF，显式 ON 报错 |

## 3. 构建与测试

经统一入口（推荐；编译持全局编译锁、`-j4` 上限）：

```sh
python3 tools/task_runner.py build --component sampling_search --parallel 4
python3 tools/task_runner.py test  --component sampling_search --workers 1
```

独立配置（standalone；自动获取锁定依赖）：

```sh
cmake -S Src/sampling_search -B build/sampling_search -DCMAKE_BUILD_TYPE=Release
cmake --build build/sampling_search --parallel 4
ctest --test-dir build/sampling_search -L sampling_search --output-on-failure
```

用例构成：34 个 CTest 条目（核心契约 10、统一契约 9、适配器专项 15）；
适配器独立开关（默认 ON 的 OMPL/Ch0p1k3/Klamp't/vss2sn；OFF 不下载、不链接）。

## 4. 基准工具（X-B1 就绪）与烟测

- `benchmarks/`：连续/栅格场景生成器 v1（SHA256 实例校验）、**每 trial 独立
  子进程**运行器（OMPL `RNG::setSeed` 为进程全局单例，正式矩阵必须每 trial
  独立进程）、回退硬超时与容量档列；套件 `v1` 已并入 `tools/components.json`，
  正式入口 `benchmark --component sampling_search --suite v1`（独占窗口）。
- 烟测 run `20261012T135320409529Z_sampling_search_f8de5fdc`：46/46 子进程、
  46/46 记录、0 SHA 失配、0 跳过，`harness.ok=true`（含容量档路径）。
- 观察（如实记录，不作结论）：vss2sn 栅格组在 128² open/maze、0.1/0.5 s 全部
  超时（性能临界，provenance §8.9）；Ch0p1k3 在 2.0 m 窄道场景超时（其 1 m
  保守栅格化把通道两侧单元判为受阻，属原生障碍映射语义，provenance §3）。
- 正式矩阵与容量档属 X-B1 独占窗口；结果须如实记录成功率/超时，不得调参隐藏。
