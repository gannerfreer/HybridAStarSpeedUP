# planner_graph_search：图搜索统一接口与多实现适配（X-S1 图 A* / X-S2 图 Dijkstra）。
#
# 通常由根 CMakeLists.txt 添加（PLANNER_BUILD_GRAPH_SEARCH=ON）。本文件同时支持
# 独立配置（standalone）：
#   cmake -S Src/graph_search -B build/graph_search-standalone
# 独立模式下自动获取 planner::common 的锁定依赖（nlohmann_json）与 GoogleTest，
# 并以内嵌固定值获取 igraph（wiring 代理写入 dependencies.lock.json 后优先用锁）。
#
# 目标：
#   planner_graph_search          公共统一接口库（依赖 planner::common；不含上游算法）
#   planner_graph_search_boost    S-01 Boost Graph A*/Dijkstra/距离场（系统 Boost 1.71）
#   planner_graph_search_lmrp     S-02 libMultiRobotPlanning 图 A*（vendored）
#   planner_graph_search_justinhj S-03 justinhj 图 A*（vendored；池开/关两组配置）
#   planner_graph_search_lemon    S-04 LEMON（OpenROAD 副本）Dijkstra/距离场（vendored）
#   planner_graph_search_igraph   S-05 igraph Dijkstra/距离场（GPL-2.0-or-later 隔离组）
#
# 隔离声明：igraph 属 GPL-2.0-or-later 隔离组，只在独立目标
# planner_graph_search_igraph 中编译并链接静态 igraph；planner_graph_search
# 与其他适配器目标均不链接 igraph。构建隔离不代替许可证核查（见 provenance.md）。
#
# 每个适配器独立开关（默认 ON）；关闭时该适配器不编译、不下载、不链接：
#   PLANNER_GRAPH_SEARCH_WITH_BOOST / _LMRP / _JUSTINHJ / _LEMON / _IGRAPH

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  cmake_minimum_required(VERSION 3.19)
  project(NewStructPlannerGraphSearch LANGUAGES C CXX)
  set(PLANNER_GRAPH_SEARCH_STANDALONE ON)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

if(NOT DEFINED PLANNER_ROOT_DIR)
  get_filename_component(PLANNER_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
endif()
if(NOT DEFINED PLANNER_BUILD_TESTS)
  option(PLANNER_BUILD_TESTS "Build tests and register them with CTest" ON)
endif()
if(NOT DEFINED PLANNER_BUILD_EXAMPLES)
  option(PLANNER_BUILD_EXAMPLES "Build component examples" ON)
endif()
if(NOT DEFINED PLANNER_BUILD_BENCHMARKS)
  option(PLANNER_BUILD_BENCHMARKS "Build component benchmarks" ON)
endif()

include(FetchContent)
if(NOT DEFINED FETCHCONTENT_TRY_FIND_PACKAGE_MODE)
  set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE NEVER)
endif()
if(PLANNER_GRAPH_SEARCH_STANDALONE AND PLANNER_BUILD_TESTS)
  enable_testing()
endif()
if(NOT DEFINED PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS)
  if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
    set(PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  else()
    set(PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS "")
  endif()
endif()

# ---------------------------------------------------------------------------
# 依赖锁读取：wiring 代理通过根项目提供 planner_lock_get()；独立模式下本文件
# 直接读取 dependencies.lock.json。缺失条目返回空串（非致命），由内嵌固定值兜底。
# ---------------------------------------------------------------------------
if(NOT DEFINED PLANNER_GRAPH_SEARCH_LOCK_JSON)
  if(DEFINED PLANNER_LOCK_JSON)
    set(PLANNER_GRAPH_SEARCH_LOCK_JSON "${PLANNER_LOCK_JSON}")
  elseif(EXISTS "${PLANNER_ROOT_DIR}/dependencies.lock.json")
    file(READ "${PLANNER_ROOT_DIR}/dependencies.lock.json"
         PLANNER_GRAPH_SEARCH_LOCK_JSON)
  else()
    set(PLANNER_GRAPH_SEARCH_LOCK_JSON "")
  endif()
endif()

# graph_search_lock_get(<dep> <out> <json-path...>)：条目缺失时输出空串。
function(graph_search_lock_get DEPENDENCY OUTVAR)
  set(${OUTVAR} "" PARENT_SCOPE)
  if(PLANNER_GRAPH_SEARCH_LOCK_JSON STREQUAL "")
    return()
  endif()
  string(JSON _gs_type ERROR_VARIABLE _gs_err TYPE
         "${PLANNER_GRAPH_SEARCH_LOCK_JSON}" dependencies "${DEPENDENCY}" ${ARGN})
  if(_gs_err OR _gs_type STREQUAL "NULL")
    return()
  endif()
  string(JSON _gs_value ERROR_VARIABLE _gs_err GET
         "${PLANNER_GRAPH_SEARCH_LOCK_JSON}" dependencies "${DEPENDENCY}" ${ARGN})
  if(_gs_err)
    return()
  endif()
  set(${OUTVAR} "${_gs_value}" PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------
# 独立模式依赖引导：GoogleTest（测试）、nlohmann_json + planner::common。
# 根项目已提供时全部跳过。
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_TESTS AND NOT TARGET GTest::gtest_main)
  graph_search_lock_get(googletest _gs_gtest_url source url)
  graph_search_lock_get(googletest _gs_gtest_sha256 source sha256)
  if(_gs_gtest_url STREQUAL "")
    set(_gs_gtest_url
        "https://github.com/google/googletest/releases/download/v1.15.2/googletest-1.15.2.tar.gz")
    set(_gs_gtest_sha256
        "7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926")
    message(STATUS
      "graph_search: googletest lock entry absent; using pinned v1.15.2 asset")
  endif()
  message(STATUS "graph_search dependency googletest (standalone bootstrap)")
  FetchContent_Declare(googletest
    URL "${_gs_gtest_url}"
    URL_HASH "SHA256=${_gs_gtest_sha256}"
    ${PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS})
  set(BUILD_GMOCK OFF CACHE BOOL "graph_search: gmock not needed" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "graph_search: not installed" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

if(NOT TARGET planner::common)
  if(NOT TARGET nlohmann_json::nlohmann_json)
    graph_search_lock_get(nlohmann_json _gs_json_url source url)
    graph_search_lock_get(nlohmann_json _gs_json_sha256 source sha256)
    if(_gs_json_url STREQUAL "")
      set(_gs_json_url
          "https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz")
      set(_gs_json_sha256
          "d6c65aca6b1ed68e7a182f4757257b107ae403032760ed6ef121c9d55e81757d")
      message(STATUS
        "graph_search: nlohmann_json lock entry absent; using pinned v3.11.3 asset")
    endif()
    FetchContent_Declare(nlohmann_json
      URL "${_gs_json_url}"
      URL_HASH "SHA256=${_gs_json_sha256}"
      ${PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS})
    set(JSON_BuildTests OFF CACHE INTERNAL "graph_search: upstream tests off")
    set(JSON_Install OFF CACHE INTERNAL "graph_search: not installed")
    FetchContent_MakeAvailable(nlohmann_json)
  endif()
  # 加入 common，但临时关闭其测试（本组件自带测试；根项目模式不会走到这里）。
  set(_gs_saved_build_tests "${PLANNER_BUILD_TESTS}")
  set(PLANNER_BUILD_TESTS OFF)
  add_subdirectory("${PLANNER_ROOT_DIR}/Src/common"
                   "${CMAKE_BINARY_DIR}/Src/common")
  set(PLANNER_BUILD_TESTS "${_gs_saved_build_tests}")
  if(NOT TARGET planner::common)
    message(FATAL_ERROR
      "graph_search: Src/common did not define planner::common; standalone "
      "configuration cannot continue.")
  endif()
endif()

# ---------------------------------------------------------------------------
# 公共统一接口库（不含任何上游算法；JSON 导出使用 nlohmann/json）。
# ---------------------------------------------------------------------------
add_library(planner_graph_search src/types.cpp)
add_library(planner::graph_search ALIAS planner_graph_search)
target_compile_features(planner_graph_search PUBLIC cxx_std_17)
target_include_directories(planner_graph_search PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(planner_graph_search
  PUBLIC planner::common
  PRIVATE nlohmann_json::nlohmann_json)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_graph_search PRIVATE -Wall -Wextra -Wpedantic)
endif()

# 统一的适配器编译选项辅助。
function(graph_search_adapter_options TARGET_NAME)
  target_compile_features(${TARGET_NAME} PRIVATE cxx_std_17)
  target_link_libraries(${TARGET_NAME} PUBLIC planner_graph_search)
  # 适配器实现共享 src/detail/control.hpp（内部头，不对外安装）。
  target_include_directories(${TARGET_NAME} PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src")
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${TARGET_NAME} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()

# ---------------------------------------------------------------------------
# S-01 Boost Graph（系统 Boost 1.71；header-only）
# ---------------------------------------------------------------------------
option(PLANNER_GRAPH_SEARCH_WITH_BOOST
  "Build the S-01 Boost Graph adapter (system Boost headers)" ON)
if(PLANNER_GRAPH_SEARCH_WITH_BOOST)
  find_package(Boost 1.71 REQUIRED)
  add_library(planner_graph_search_boost STATIC
    src/adapters/boost_adapter.cpp)
  add_library(planner::graph_search_boost ALIAS planner_graph_search_boost)
  graph_search_adapter_options(planner_graph_search_boost)
  target_compile_definitions(planner_graph_search_boost
    PUBLIC PLANNER_GRAPH_SEARCH_HAS_BOOST=1)
  if(TARGET Boost::headers)
    target_link_libraries(planner_graph_search_boost PRIVATE Boost::headers)
  elseif(TARGET Boost::boost)
    target_link_libraries(planner_graph_search_boost PRIVATE Boost::boost)
  endif()
  message(STATUS
    "graph_search S-01 Boost adapter: Boost ${Boost_VERSION} (system headers)")
endif()

# ---------------------------------------------------------------------------
# S-02 libMultiRobotPlanning（vendored；单机器人 A*）
# ---------------------------------------------------------------------------
option(PLANNER_GRAPH_SEARCH_WITH_LMRP
  "Build the S-02 libMultiRobotPlanning graph A* adapter (vendored)" ON)
if(PLANNER_GRAPH_SEARCH_WITH_LMRP)
  add_library(planner_graph_search_lmrp STATIC
    src/adapters/lmrp_adapter.cpp)
  add_library(planner::graph_search_lmrp ALIAS planner_graph_search_lmrp)
  graph_search_adapter_options(planner_graph_search_lmrp)
  target_include_directories(planner_graph_search_lmrp PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/upstream/lmrp/include")
  target_compile_definitions(planner_graph_search_lmrp
    PUBLIC PLANNER_GRAPH_SEARCH_HAS_LMRP=1)
  # 上游默认使用 Boost heap（d_ary_heap）头文件。
  if(TARGET Boost::headers)
    target_link_libraries(planner_graph_search_lmrp PRIVATE Boost::headers)
  elseif(TARGET Boost::boost)
    target_link_libraries(planner_graph_search_lmrp PRIVATE Boost::boost)
  endif()
endif()

# ---------------------------------------------------------------------------
# S-03 justinhj（vendored + 1 行内存池守卫补丁；池开/关两 TU 不同命名空间）
# ---------------------------------------------------------------------------
option(PLANNER_GRAPH_SEARCH_WITH_JUSTINHJ
  "Build the S-03 justinhj graph A* adapter (vendored; pool on/off)" ON)
if(PLANNER_GRAPH_SEARCH_WITH_JUSTINHJ)
  add_library(planner_graph_search_justinhj STATIC
    src/adapters/justinhj_adapter_pool.cpp
    src/adapters/justinhj_adapter_nopool.cpp)
  add_library(planner::graph_search_justinhj ALIAS planner_graph_search_justinhj)
  graph_search_adapter_options(planner_graph_search_justinhj)
  target_include_directories(planner_graph_search_justinhj PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/upstream/justinhj")
  target_compile_definitions(planner_graph_search_justinhj
    PUBLIC PLANNER_GRAPH_SEARCH_HAS_JUSTINHJ_POOL=1
    PUBLIC PLANNER_GRAPH_SEARCH_HAS_JUSTINHJ_NOPOOL=1)
  # 上游头在 -Wall/-Wextra 下有 -Wreorder（成员初始化顺序）与池关闭时
  # MaxNodes 未使用警告（无害）；只对本目标抑制，不修改上游代码。
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_graph_search_justinhj PRIVATE
      -Wno-reorder -Wno-unused-parameter)
  endif()
endif()

# ---------------------------------------------------------------------------
# S-04 LEMON（OpenROAD 副本；vendored include 集，header-only 使用）
# ---------------------------------------------------------------------------
option(PLANNER_GRAPH_SEARCH_WITH_LEMON
  "Build the S-04 LEMON (OpenROAD copy) Dijkstra adapter (vendored)" ON)
if(PLANNER_GRAPH_SEARCH_WITH_LEMON)
  add_library(planner_graph_search_lemon STATIC
    src/adapters/lemon_adapter.cpp)
  add_library(planner::graph_search_lemon ALIAS planner_graph_search_lemon)
  graph_search_adapter_options(planner_graph_search_lemon)
  target_include_directories(planner_graph_search_lemon PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/upstream/lemon/include")
  target_compile_definitions(planner_graph_search_lemon
    PUBLIC PLANNER_GRAPH_SEARCH_HAS_LEMON=1)
endif()

# ---------------------------------------------------------------------------
# S-05 igraph（GPL-2.0-or-later 隔离组；官方 1.0.1 发布 tar 包，静态链接）
# ---------------------------------------------------------------------------
option(PLANNER_GRAPH_SEARCH_WITH_IGRAPH
  "Build the S-05 igraph Dijkstra adapter (GPL-2.0-or-later isolation group)" ON)
if(PLANNER_GRAPH_SEARCH_WITH_IGRAPH)
  graph_search_lock_get(igraph _gs_igraph_url source url)
  graph_search_lock_get(igraph _gs_igraph_sha256 source sha256)
  if(_gs_igraph_url STREQUAL "")
    set(_gs_igraph_url
        "https://github.com/igraph/igraph/releases/download/1.0.1/igraph-1.0.1.tar.gz")
    message(STATUS
      "graph_search: igraph lock entry absent (wiring pending); using the "
      "pinned 1.0.1 release tarball recorded in provenance.md")
  endif()
  if(_gs_igraph_sha256 STREQUAL "")
    set(_gs_igraph_sha256
        "969f2d7d22f67e788d8638c9a8c96615f50d7819c08978b3ef4a787bb6daa96c")
  endif()

  # 上游构建最小化：静态、无 LP/GraphML/Infomap/OpenMP、无测试/示例/文档。
  # 这些是普通变量（非缓存），只作用于 igraph 子目录，不影响其他组件。
  set(BUILD_SHARED_LIBS OFF)
  set(BUILD_TESTING OFF)
  set(IGRAPH_GLPK_SUPPORT OFF)
  set(IGRAPH_GRAPHML_SUPPORT OFF)
  set(IGRAPH_INFOMAP_SUPPORT OFF)
  set(IGRAPH_OPENMP_SUPPORT OFF)
  FetchContent_Declare(igraph
    URL "${_gs_igraph_url}"
    URL_HASH "SHA256=${_gs_igraph_sha256}"
    ${PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS})
  FetchContent_MakeAvailable(igraph)

  if(NOT TARGET igraph)
    message(FATAL_ERROR
      "graph_search: igraph was populated from '${_gs_igraph_url}' but did "
      "not define the 'igraph' target; the pinned source package is broken.")
  endif()

  add_library(planner_graph_search_igraph STATIC
    src/adapters/igraph_adapter.cpp)
  add_library(planner::graph_search_igraph ALIAS planner_graph_search_igraph)
  graph_search_adapter_options(planner_graph_search_igraph)
  target_compile_definitions(planner_graph_search_igraph
    PUBLIC PLANNER_GRAPH_SEARCH_HAS_IGRAPH=1)
  # GPL 隔离：igraph 只作为 PRIVATE 依赖链接进本隔离目标。
  target_link_libraries(planner_graph_search_igraph PRIVATE igraph)
  message(STATUS
    "graph_search S-05 igraph adapter: isolation group (GPL-2.0-or-later), "
    "static link into planner_graph_search_igraph only")
endif()

# ---------------------------------------------------------------------------
# 独立示例（随组件构建；--self-check 注册为 CTest）
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_EXAMPLES)
  # 示例只链接非隔离适配器；GPL-2.0-or-later 的 igraph 隔离目标仅由专用测试
  # 可执行文件链接（与 planner_bspline_ego 仅测试/基准链接的隔离模式一致）。
  set(_gs_demo_links planner_graph_search)
  if(TARGET planner_graph_search_boost)
    list(APPEND _gs_demo_links planner_graph_search_boost)
  endif()
  if(TARGET planner_graph_search_lmrp)
    list(APPEND _gs_demo_links planner_graph_search_lmrp)
  endif()
  if(TARGET planner_graph_search_justinhj)
    list(APPEND _gs_demo_links planner_graph_search_justinhj)
  endif()
  if(TARGET planner_graph_search_lemon)
    list(APPEND _gs_demo_links planner_graph_search_lemon)
  endif()

  add_executable(planner_graph_search_demo examples/graph_search_demo.cpp)
  target_link_libraries(planner_graph_search_demo PRIVATE ${_gs_demo_links})
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_graph_search_demo PRIVATE
      -Wall -Wextra -Wpedantic)
  endif()

  if(PLANNER_BUILD_TESTS)
    add_test(NAME planner_graph_search_demo_self_check
             COMMAND planner_graph_search_demo --self-check)
    set_tests_properties(planner_graph_search_demo_self_check
      PROPERTIES LABELS "graph_search")
  endif()
endif()

# ---------------------------------------------------------------------------
# 测试（GoogleTest + CTest，标签 "graph_search"）。
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_TESTS)
  if(NOT TARGET GTest::gtest_main)
    message(FATAL_ERROR
      "PLANNER_BUILD_TESTS=ON but GTest::gtest_main is not available. "
      "Configure through the root CMakeLists.txt or enable the standalone "
      "bootstrap (this file fetches the locked googletest when missing).")
  endif()
  add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------
# 基准（X-B1；PLANNER_BUILD_BENCHMARKS，默认 ON）。
# planner_graph_search_bench：graph_gen v1 确定性生成器（--generate-only/--check）
# 与正式矩阵运行器（--suite v1 / --smoke）。链接全部已构建适配器（含 igraph
# 隔离目标；GPL-2.0-or-later 只进入该专用离线基准可执行文件）。规则与指标
# 契约见 benchmarks/README.md；components.json 注册片段见
# benchmarks/benchmark_suite.json（由单一写者并入）。
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()
