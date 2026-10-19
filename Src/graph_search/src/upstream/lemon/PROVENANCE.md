# S-04 LEMON（OpenROAD 副本）来源与许可证说明（vendored include 集）

> 本目录是 `planner_graph_search` S-04 适配器（Dijkstra 点到点 + 完整距离场）所复制的
> LEMON 头文件最小集。上游为 **OpenROAD 组织维护的 LEMON 源码副本**
> （`https://github.com/The-OpenROAD-Project/lemon-graph`），不是独立上游。

## 1. 上游固定信息

| 项 | 值 |
| --- | --- |
| 上游仓库（副本） | <https://github.com/The-OpenROAD-Project/lemon-graph>（默认分支 `master`） |
| 副本 commit | `31d79d686f776a62ace1925cb2e40e84a80d1394`（family_b §2.1，2026-10-12 核实） |
| 上游基线 | 官方 LEMON **1.3.1** 发布版（`lemon-1.3.1.tar.gz`，SHA256 `71b7c725f4c0b4a8ccb92eb87b208701586cf7a96156ebd821ca3ed855bad3c8`）；副本基线 commit `62ac75337e5a`（“Add lemon 1.3.1”），与官方 tarball **全部 2465 文件逐字节相同** |
| 副本相对基线补丁 | HEAD 相对基线 9 文件、49+/34−，均为 C++20 兼容/告警级修正（`allocator_traits`、`std::iterator`、`register`、CMake 最低版本）；`lemon/dijkstra.h` 及其依赖无算法性修改（family_b §2.1） |
| 许可证 | 整体 **BSL-1.0**：`LICENSE`（SHA256 `0057760df95992e06a0bc78f5632a389a03744f6725e497cedd85bc21e39b28a`）为 EGRES 2003–2012 版权头 + Boost Software License 1.0 全文 |
| 文件级例外 | ① `lemon/bits/enable_if.h`：Boost enable_if 衍生（Indiana University），明确 BSL-1.0，**在 Dijkstra 包含集内**；② `lemon/random.h`：Matsumoto & Nishimura BSD-3 式声明，**不在包含集内**（未复制）。未发现 GPL/著佐权污染（family_b §2.2） |
| 获取方式 | **vendored**（逐文件复制，随组件入 Git；不下载） |
| 版本宏 | `LEMON_VERSION "1.3.1"`（见 §3 `config.h`） |

## 2. 复制文件清单（逐字节一致，未修改）

Dijkstra 传递包含集（`dijkstra.h` 闭包）：22 个头文件。除 `config.h`（生成文件，见 §3）
外全部与副本 commit 逐字节一致。

| 本仓库文件（`include/lemon/`） | SHA256 |
| --- | --- |
| `assert.h` | `4834da7ac2bca235bf1e752c8bb982f4fb4d5670d207902ea6831beb75dab013` |
| `bin_heap.h` | `52451e49afa225be3619d8aa8cc189690f4052bd6d2e665588cc0a26ca2f6c2b` |
| `bits/alteration_notifier.h` | `6a2b546751ad0de1ec8ee16e86c64978f9e25e209a3e872356e1eea45b930e2a` |
| `bits/array_map.h` | `93cc28d0d1b1b3f618203274ba57d9dbcb967dd373d8daa1aa633514125ea76a` |
| `bits/default_map.h` | `7b1b2a1d223b14e93ba7fad98a01a45c7d3abaecd848104b03a43df0b94cf914` |
| `bits/enable_if.h` | `6356649233e7383ffb1c95a6e18b2aabbe313eb38525a70d6bb933c92b070cff` |
| `bits/graph_extender.h` | `646399eb34ba87499d78046edd0a689bfb0022cd9d5adc2496539ec38c05bb03` |
| `bits/lock.h` | `6c188cdf83299408be6677a2615b116dd3dcc565c06b5c43bfb266437495af10` |
| `bits/map_extender.h` | `5129c5f1a13c7909b0ae1a62dfcbeaaaf84e817d4ed2535ae52b27076472e669` |
| `bits/path_dump.h` | `9a96d901c59790346dd1f5d11fe1d7b093c11440ab2b4e2240b9b0d092e79d4a` |
| `bits/traits.h` | `e1c02ba5376edb7bdde009ed746ed721150f0b735a854679fdfec3b22cd43da7` |
| `bits/vector_map.h` | `7ab1d75cd6739049367c7511b7bfc29cc04b3cba01660ec97b89e00c4e860fb5` |
| `bits/windows.h` | `4cd0ff49187646a9a49f595cbd536617712b63efb04923d27a768a616cd80052` |
| `concept_check.h` | `aa6cf641fc628064482b7154f13869e3aecd84b96d19a48a8503db99224747cb` |
| `concepts/maps.h` | `15a81ac5dcb72fe9873a14262e3fd505fb46ab61dd9f6b1353e6cf66a34bee9c` |
| `concepts/path.h` | `4ad261deb09ef6b74141ca294b7538342b5932060556f74fe14d15bbe7fd2a09` |
| `config.h` | `f31dcfbc78cc786807d7ce39da87cf0de429e6e6b6e7d8e2488565c8bee32a54`（生成，见 §3） |
| `core.h` | `b7e8d81c076cfdd48eb019169df2855273603b69cb5ca7e5718b86ce375ebb33` |
| `dijkstra.h` | `6aab770e44c7ef12267105b37f81c7b43cfdcf1721ea2a31efdf44ca2bffe1d1` |
| `error.h` | `6619fdabe22e94086948bb317fee4aeb98176ed8faef7288e122645dfde71355` |
| `list_graph.h` | `2d56dcbfce356cf736f846f6d5cc4539ee4eeb803090dc4f32ac20c7aad1bbb4` |
| `maps.h` | `b75bd0473aa7f1945a5860b83bd3a6f2213fb09755a37e0e25b6c34dc06f9706` |
| `path.h` | `acbaa611b721ed09c1c857bfa6014f3b87c50954cfe8c9711ad6075eb2a4093d` |
| `LICENSE` | `0057760df95992e06a0bc78f5632a389a03744f6725e497cedd85bc21e39b28a` |

未复制的上游内容：其余 LEMON 算法头、`tools/`、`demo/`、`test/`、LP 后端与
CMake 工程（本组件只以头文件方式使用 Dijkstra，不构建 `libemon`）。

## 3. `config.h` 生成说明

上游 `config.h` 由 CMake 生成（本组件不运行上游构建）。副本生成的版本记录于
family_b 探针（`LEMON_VERSION "1.3.1"`、LP/MIP 后端关闭）。本目录的 vendored
副本复现该配置，并有一处**登记偏差**：`LEMON_USE_PTHREAD` 保持未定义，使
`lemon/bits/lock.h` 选择空实现（单线程）而非 pthread 互斥锁；Dijkstra 包含集
不使用跨线程状态。若后续需要 pthread 语义，应重新生成并在来源记录登记。

## 4. 适配补丁与原生偏差（S-04，X-S2）

上游头文件**未修改**；以下为适配层行为：

1. **点到点 vs 完整距离场分离**：`dij.run(s)` 完整场；点到点用
   `init()+addSource()+processNextNode()` 手动循环，目标节点处理即早停
   （等价 `run(s,t)` 语义，可插入预算/取消检查）。两种模式结果不混用
   （早停后未处理节点距离不完整）。
2. **取消/超时**：LEMON 无 visitor 钩子，但 `processNextNode()`/`nextNode()`/
   `emptyQueue()` 为公开 API；适配层逐节点检查取消标志与时间预算（O(1)/节点），
   超限中止并区分 `kCancel`/`kTimeout`。
3. **不可达语义**：`reached(node)==false` 判定不可达，统一输出为
   `std::numeric_limits<double>::infinity()`；不使用数值哨兵。
4. **图结构选择**：`ListDigraph` + `ArcMap<double>`；节点 id ↔ `Node` 映射由适配层
   维护，建图成本计入 `Stats.preprocess_ms`。

## 5. 复核方式

```sh
git clone https://github.com/The-OpenROAD-Project/lemon-graph /tmp/lemon-graph
cd /tmp/lemon-graph && git checkout 31d79d686f776a62ace1925cb2e40e84a80d1394
sha256sum lemon/dijkstra.h lemon/list_graph.h lemon/core.h LICENSE
# 应与本文件 §2 一致；副本与官方 1.3.1 的 2465 文件逐字节比对证据见
# artifacts/probes/family_b_20261012T111444Z/lemon/diff_upstream_1.3.1_vs_baseline.txt
```
