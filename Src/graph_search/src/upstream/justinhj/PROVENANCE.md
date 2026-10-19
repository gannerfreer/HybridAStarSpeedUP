# S-03 justinhj/astar-algorithm-cpp 来源与许可证说明（vendored + 1 行适配补丁）

> 本目录是 `planner_graph_search` S-03 适配器（图 A*，内存池开/关两组配置）所复制
> 的上游核心头文件。示例、测试、基准（`8puzzle.cpp`、`mazebench.cpp`、`bench/`、
> doctest FetchContent）不接入。

## 1. 上游固定信息

| 项 | 值 |
| --- | --- |
| 上游仓库 | <https://github.com/justinhj/astar-algorithm-cpp> |
| 固定 tag | `v1.4.2` |
| 固定 commit | `875bbd8ba4cdf75463cb30424d0ab4b2a8db9653`（2026-09-30） |
| master 关系 | master HEAD `25b05a33720cd1009f00e1791c2a305b89839747`（2026-10-06）相对 tag 仅新增基准（`bench/`、`mazebench.cpp`、CMake 目标）；`stlastar.h`/`fsa.h`/`license.txt` 完全一致（family_a §4.1） |
| 许可证 | MIT（`license.txt`，SHA256 `417efafe07c1723ee6322cf11604cbce87eed6c4eb66fe15b838dfb0a76ebe91`，Copyright 2001-2026） |
| 文件级说明 | `stlastar.h`/`fsa.h` 文件头另有自定义宽松声明（保留署名即可再分发 + AS IS 免责）；与根 `license.txt` 并列记录 |
| 获取方式 | **vendored**（逐文件复制，随组件入 Git；不下载） |

## 2. 复制文件清单与改动

| 本仓库文件 | 上游路径 | 上游 SHA256 | 本仓库 SHA256 | 改动 |
| --- | --- | --- | --- | --- |
| `stlastar.h` | `stlastar.h` | `285249ded4ea586f2809ec583a13b8be8cadd41a5f0cd05d5ccc49552c53a591` | `9d89a19f8e2b4d6d4a7e9e39f427a6418084685b0cd4f02dfa5519215309bb2a` | **1 行级适配补丁**（见 §3） |
| `fsa.h` | `fsa.h` | `606539463810f240062fb4b2ac1edc07548ab5f9aa6b5a39f5d4f3edf73b0fde` | 同上游（逐字节一致） | 无 |
| `license.txt` | `license.txt` | `417efafe07c1723ee6322cf11604cbce87eed6c4eb66fe15b838dfb0a76ebe91` | 同上游 | 无 |
| `stlastar_pool_guard.patch` | — | — | — | 上述补丁的 diff 记录（复现用） |

## 3. 内存池开关适配补丁（Wave 3 条件项）

上游将 `#define USE_FSA_MEMORY 1` 硬编码，无编译期关闭路径。补丁以
`#ifndef` 守卫包装该定义（默认行为不变，仅允许 `-DUSE_FSA_MEMORY=0` 关闭内存池）：

```diff
--- a/stlastar.h
+++ b/stlastar.h
@@
 // Fixed size memory allocator can be disabled to compare performance
 // Uses std new and delete instead if you turn it off
+// ... (planner_graph_search adaptation note, see file)
+#ifndef USE_FSA_MEMORY
 #define USE_FSA_MEMORY 1
+#endif
```

完整 diff 见 `stlastar_pool_guard.patch`（18 行，含注释）。适配层据此编译
`planner_graph_search_justinhj`（池开，默认）与 `planner_graph_search_justinhj_nopool`
（池关）两个目标；池容量按图规模设置（`max(1000, num_vertices + 16)`，记录于
`EffectiveConfig.options.memory_pool_capacity`）。

## 4. 适配补丁与原生偏差（S-03，X-S1）

`stlastar.h` 除上述守卫外**未修改**；其余为适配层行为：

1. **float32 精度（登记条件项）**：上游 g/h/f 与 `GetCost` 均为 `float`。适配层在
   结果元数据标注 `cost_precision=float32`；正确性档对原生代价使用已登记的
   float32 门槛，另按返回路径以 double 重算代价并核对 §6.2 容差（不改核心）。
   探针偏差 Δ≈2.8e-6（42.183766 参考值），超 §6.2 容差约 6.5×（family_a §4.3）。
2. **取消/无路径区分（登记条件项）**：上游 `CancelSearch()` 后状态为
   `SEARCH_STATE_FAILED`，与“无路径”不可区分；`m_CancelRequest` 为普通 bool
   （非原子）。适配层自行记录取消/超时标志并在步间检查时钟（`SearchStep()` 分步
   可中断），据此区分 `kCancel`/`kTimeout`/`kNoPath`；跨线程取消由适配层同步
   （不依赖裸 bool 的线程安全）。
3. **内存池耗尽**：池容量不足时上游返回 `SEARCH_STATE_OUT_OF_MEMORY`；适配层映射为
   `kInvalidInput` 并在 message 中说明（正确性测试容量充足，不会触发）。
4. **去重语义**：上游以 `Hash`/`IsSameState` 经 `unordered_set` 去重；默认
   `ConsistentHeuristic=true` 不重开 closed 节点，正确性绑定一致启发式前提。
5. **-Wreorder 警告**：上游头在 `-Wall` 下产生成员初始化顺序警告（无害）；仅在
   S-03 适配源文件上抑制 `-Wno-reorder`，不修改上游代码。

## 5. 复核方式

```sh
git clone https://github.com/justinhj/astar-algorithm-cpp /tmp/justinhj
cd /tmp/justinhj && git checkout 875bbd8ba4cdf75463cb30424d0ab4b2a8db9653
sha256sum stlastar.h fsa.h license.txt
# stlastar.h 上游 SHA256 应为 285249de...；fsa.h / license.txt 应与本文件 §2 一致
# 补丁后 stlastar.h 应为 9d89a19f...
```
