# X-B1 栅格基准工具（benchmarks/）：确定性场景生成器 + 正式/容量基准运行器。
#
# 由 Src/grid_search/CMakeLists.txt 在 PLANNER_BUILD_BENCHMARKS=ON 时加入
# （默认 ON，与根选项一致）。不注册 CTest：开发期通过 tools/task_runner.py
# exec 运行，独占窗口阶段通过注册后的 benchmark 子命令运行（见
# benchmark_suite.json 注册片段）。

add_library(planner_grid_bench_support STATIC
  bench_util.cpp
  scenario_gen.cpp
  sha256.cpp)
target_include_directories(planner_grid_bench_support PUBLIC
  "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(planner_grid_bench_support PUBLIC planner_grid_search)
target_compile_features(planner_grid_bench_support PUBLIC cxx_std_17)

find_package(Threads REQUIRED)
target_link_libraries(planner_grid_bench_support PUBLIC Threads::Threads)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_grid_bench_support PRIVATE
    -Wall -Wextra -Wpedantic)
endif()

add_executable(planner_grid_scenario_gen grid_scenario_gen.cpp)
target_link_libraries(planner_grid_scenario_gen
  PRIVATE planner_grid_bench_support)

add_executable(planner_grid_search_bench grid_search_bench.cpp)
target_link_libraries(planner_grid_search_bench
  PRIVATE planner_grid_bench_support)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_grid_scenario_gen PRIVATE
    -Wall -Wextra -Wpedantic)
  target_compile_options(planner_grid_search_bench PRIVATE
    -Wall -Wextra -Wpedantic)
endif()
