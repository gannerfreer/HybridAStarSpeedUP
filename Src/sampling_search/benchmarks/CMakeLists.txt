# planner_sampling_search_bench（X-B1）：RRT* 族基准运行器。
#
# 由 Src/sampling_search/CMakeLists.txt 在 PLANNER_BUILD_BENCHMARKS=ON 时加入。
# 注册片段（供 wiring 合并进 tools/components.json）见 benchmark_suite.json；
# 正式矩阵与容量档在独占窗口经 tools/task_runner.py benchmark 运行。
# 组件绝不写入 --output-dir 以外的位置（tools/README.md §7.2）。

add_executable(planner_sampling_search_bench
  sampling_search_bench.cpp
  scenario_gen.cpp
  sha256.cpp)

target_compile_features(planner_sampling_search_bench PRIVATE cxx_std_17)
target_link_libraries(planner_sampling_search_bench PRIVATE
  planner_sampling_search
  nlohmann_json::nlohmann_json)

# GCC < 10 中 std::filesystem 位于单独的库（本机为 GCC 9.4）。
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 10)
  target_link_libraries(planner_sampling_search_bench PRIVATE stdc++fs)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_sampling_search_bench PRIVATE
    -Wall -Wextra -Wpedantic)
endif()
