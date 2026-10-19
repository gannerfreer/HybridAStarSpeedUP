# Ch0p1k3 最小抽取与适配补丁记录（X-S4）

上游：`https://github.com/Ch0p1k3/path-planning-algorithms-rrt-rrtstar`
commit `4ae8c935f985097e50976e75220c27dac57ffa61`（2023-04-22，MIT）。
仓库含约 2.9 GB `tests/`（MovingAI 场景数据），无法整仓下载；按探针
`supplement_ch0p1k3_probe.md` 的已验证抽取集复制到本目录。

## 1. 抽取清单与补丁状态

`src/` 为上游 `src/` 的最小复制（23 个文件，全部保留原始 MIT 版权头）。

- 除 `rrt.hpp` 外全部与上游 `4ae8c935` 逐字节一致（见 provenance.md 的
  逐文件 SHA256 与比对方式）。
- `rrt.hpp` 仅含下述 4 行种子钩子（探针 `seed_hook.patch` 正式化）；
  采样、最近邻、steer、选父、重连等算法代码未改。
- `sfml_stub/SFML/Graphics.hpp` 为探针侧最小桩（非上游代码）：仅为让
  `tree.cpp`/`rrt.cpp` 的 `drawTree` 可视化函数通过编译；可视化路径永不调用。
- 上游 CMake 未使用（其强制 Clang ≥ 14 并 FetchContent SFML）；本组件以
  CMake 显式文件列表编译 7 个核心 .cpp + pugixml（锁条目 1.16；上游
  FetchContent pin 为 1.13，XML 解析 API 不变，差异记录于 provenance.md）。
- `third_party/nanoflann/nanoflann.hpp`：nanoflann v1.4.3
  （commit `2f54b8eb07c719bc605445e4ec44d61240f5a2ce`，BSD-2-Clause 头保留），
  与上游 FetchContent pin 一致。

## 2. 种子钩子补丁（rrt.hpp，唯一上游修改）

```diff
--- a/rrt.hpp
+++ b/rrt.hpp
@@ -25,6 +25,10 @@
   // void printTree(std::ostream& = std::cout);
   void drawTree(sf::RenderWindow& window);
 
+  void setSeed(const unsigned seed) {
+    gen.seed(seed);
+  }
+
   inline Geometry::Point getStart() const {
     return start;
   }
```

说明：`RRT::gen` 为私有成员（`std::mt19937`），上游无种子 API；4 行钩子
只重播种，不改变采样分布。适配器其余缺口（预算/取消驱动、XML 生成、
输出转换）全部在适配层，不触碰核心。

## 3. 已知上游风险（不修补，登记于 provenance.md）

1. `Tree::used` 去重表为死代码（`tree.hpp:109` / `tree.cpp:89` 从未写入）；
2. `Obstacle::obstacleFree` 半径搜索 `len²+1` 的理论漏检窗口；
3. `Map`/`Algorithm`/`Obstacle` 构造在输入错误或起终点落在障碍时调用
   `exit()`；适配器先做完整预检并强制解除起点/终点单元以规避；
4. 目标域语义为 `eps` 距离内到达（非精确终点），适配器按统一边有效补接
   精确终点并记录原生末点间距。
