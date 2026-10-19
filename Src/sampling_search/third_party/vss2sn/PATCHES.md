# vss2sn 最小抽取与适配补丁记录（X-S4）

上游：`https://github.com/vss2sn/path_planning`
commit `d8f9edaeae064ab4665350bf820d1d5ebbbfcc45`（BSD-3-Clause，LICENSE.md
已复制到本目录）。

## 1. 抽取清单

- `include/path_planning/rrt_star.hpp`、`include/path_planning/planner.hpp`
  （上游 `include/path_planning/`）；
- `include/utils/utils.hpp`、`src/utils.cpp`（上游 `lib/utils/` 子库）；
- `src/rrt_star.cpp`（上游 `src/`）。

未抽取其他规划器、示例、测试与可视化代码。头文件保留原始版权头。

## 2. 适配补丁（种子 / 预算-取消 / 首次解 / 迭代计数）

上游 RRT* 的缺口：`GenerateRandomNode` 每次调用新建 `std::random_device`
（不可复现）；`Plan` 为固定迭代数的单次调用（无预算/取消钩子，且调用方
无法观测首次解时间）。适配层补齐方式为下述最小补丁；最近邻、选父、重连、
障碍判定等算法原语未改。

`include/path_planning/rrt_star.hpp`：

```diff
 #include <functional>
 #include <limits>
 #include <random>
 ...
   void SetParams(const int threshold = 2, const int max_iter_x_factor = 20);
+
+  // X-S4 adapter hooks (patched; NOT upstream API):
+  void SetSeed(const unsigned int seed) { eng_.seed(seed); }
+  void SetStopPredicate(std::function<bool()> stop) { stop_pred_ = std::move(stop); }
+  void SetFirstSolutionCallback(std::function<void()> cb) { first_solution_cb_ = std::move(cb); }
+  std::size_t GetIterations() const { return iterations_done_; }
+  bool GetStoppedEarly() const { return stopped_; }
 ...
   double threshold_ = 1.5;
   int max_iter_x_factor_ = 500;
+
+  // X-S4 adapter hooks (patched).
+  mutable std::mt19937 eng_{std::random_device{}()};
+  std::function<bool()> stop_pred_;
+  std::function<void()> first_solution_cb_;
+  std::size_t iterations_done_{0};
+  bool stopped_{false};
 };
```

`src/rrt_star.cpp`：

```diff
 Node RRTStar::GenerateRandomNode() const {
-  std::random_device rd;
-  std::mt19937 eng(rd());
   std::uniform_int_distribution<int> distr(0, n_ - 1);
-  const int x = distr(eng);
-  const int y = distr(eng);
+  const int x = distr(eng_);
+  const int y = distr(eng_);
   return Node(x, y, 0, 0, n_ * x + y, 0);
 }
@@
   while (iteration < max_iterations) {
+    // X-S4 adapter hook (patched): stop predicate (wall-clock budget /
+    // cancellation) checked once per attempt; the algorithm body is unchanged.
+    if (stop_pred_ && stop_pred_()) {
+      stopped_ = true;
+      break;
+    }
     iteration++;
@@
     if (CheckGoalVisible(new_node)) {
       found_goal = true;
+      if (!first_solution_seen) {
+        first_solution_seen = true;
+        if (first_solution_cb_) {
+          first_solution_cb_();
+        }
+      }
     }
   }
+  iterations_done_ = static_cast<std::size_t>(iteration);
```

（完整补丁即本目录文件相对上游的差异；上游 commit 下四个文件的 SHA256
记录于 provenance.md，接入复核时逐文件比对。）

## 3. 原生限制（登记于 provenance.md）

- 原生栅格为方阵 `n×n`（`max_iter_x_factor_ * n_ * n_` 与随机采样均假设
  方阵）；非方阵边界返回 `kInvalidInput`；
- 状态离散为格点，属栅格采样组，与连续空间组分开评分；
- 起终点按最近格点吸附；对齐输入端点精确，非对齐时按统一边有效补接精确
  端点；
- 预算耗尽或迭代上限耗尽无解均报 `kTimeout`（不证明不可达）。
