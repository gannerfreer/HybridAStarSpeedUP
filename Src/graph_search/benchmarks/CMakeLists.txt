# planner_graph_search_bench（X-B1）：graph_gen v1 确定性生成器 + 正式矩阵运行器。
#
# 由上级 Src/graph_search/CMakeLists.txt 在 PLANNER_BUILD_BENCHMARKS=ON 时
# add_subdirectory；独立配置与根接线两种路径行为一致。
#
# 链接全部已构建的适配器目标：非隔离目标 + planner_graph_search_igraph
# （GPL-2.0-or-later 隔离组只进入本专用离线基准可执行文件，不进入默认库、
# 示例或在线服务；构建隔离不代替许可证核查，见 provenance.md）。
# 未构建的适配器对应 PLANNER_GRAPH_SEARCH_HAS_* 宏不定义，基准注册表将其
# 标记为未编译并跳过。

add_executable(planner_graph_search_bench
  graph_search_bench.cpp
  graph_gen.cpp
  sha256.cpp)
target_compile_features(planner_graph_search_bench PRIVATE cxx_std_17)
target_include_directories(planner_graph_search_bench PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}")

# nlohmann/json 用于实例表与 summary（planner_graph_search 以 PRIVATE 链接，
# 这里显式复用同一目标；standalone 与根项目两种模式下该目标均已存在）。
target_link_libraries(planner_graph_search_bench PRIVATE
  nlohmann_json::nlohmann_json)

set(_gs_bench_links planner_graph_search)
foreach(_gs_bench_adapter
    planner_graph_search_boost
    planner_graph_search_lmrp
    planner_graph_search_justinhj
    planner_graph_search_lemon
    planner_graph_search_igraph)
  if(TARGET ${_gs_bench_adapter})
    list(APPEND _gs_bench_links ${_gs_bench_adapter})
  endif()
endforeach()
target_link_libraries(planner_graph_search_bench PRIVATE ${_gs_bench_links})

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_graph_search_bench PRIVATE
    -Wall -Wextra -Wpedantic)
endif()

message(STATUS
  "graph_search X-B1 bench: planner_graph_search_bench "
  "(graph_gen v1 + suite v1; adapters linked: ${_gs_bench_links})")
