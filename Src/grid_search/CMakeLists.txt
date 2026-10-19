# planner_grid_search：统一栅格搜索契约与多来源适配（X-S3 栅格 A* + X-S2 栅格
# Dijkstra 部分）。
#
# 依赖边界（计划 §2.1）：只依赖 planner::common（GridView 类型按冻结补充暂留
# common）；不暴露 Lanelet2 或上游类型；不依赖地图服务与显示。上游源码按来源
# 抽取进 thirdparty/ 并保留许可证头，适配补丁与原生偏差见 provenance.md。
#
# 通常由根 CMakeLists.txt 添加（PLANNER_BUILD_GRID_SEARCH=ON，默认 OFF，由
# X-S3 接线代理统一开启）。本文件也可作为顶层项目独立配置：
#   cmake -S Src/grid_search -B <build-dir> \
#     -DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=<json-src> \
#     -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=<gtest-src>   # 离线/受限网络时
# 独立模式从 dependencies.lock.json 读取 nlohmann_json / googletest 的锁定
# 版本（与根项目同一格式），并只构建 planner_common 的库目标（不构建其测试）。

cmake_minimum_required(VERSION 3.19)

if(NOT DEFINED PLANNER_ROOT_DIR)
  get_filename_component(PLANNER_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
endif()
if(NOT DEFINED PLANNER_BUILD_TESTS)
  if(DEFINED BUILD_TESTING)
    set(PLANNER_BUILD_TESTS_DEFAULT "${BUILD_TESTING}")
  else()
    set(PLANNER_BUILD_TESTS_DEFAULT ON)
  endif()
  option(PLANNER_BUILD_TESTS "Build tests and register them with CTest"
    ${PLANNER_BUILD_TESTS_DEFAULT})
endif()
if(NOT DEFINED PLANNER_BUILD_EXAMPLES)
  option(PLANNER_BUILD_EXAMPLES "Build component examples" ON)
endif()

# ---------------------------------------------------------------------------
# 独立配置模式（非根项目添加时）：获取 common 的锁定依赖并加入 common 目标。
# ---------------------------------------------------------------------------
set(PLANNER_GRID_SEARCH_STANDALONE OFF)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(PLANNER_GRID_SEARCH_STANDALONE ON)
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()

  include(FetchContent)
  set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE NEVER)
  if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
    set(_gs_download_args DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  else()
    set(_gs_download_args "")
  endif()

  set(_gs_lock "${PLANNER_ROOT_DIR}/dependencies.lock.json")
  if(NOT EXISTS "${_gs_lock}")
    message(FATAL_ERROR
      "dependencies.lock.json is missing at ${_gs_lock}; the standalone "
      "grid_search configure needs it (or configure through the root project)")
  endif()
  file(READ "${_gs_lock}" _gs_lock_json)

  function(gs_lock_get DEPENDENCY OUTVAR)
    string(JSON _value ERROR_VARIABLE _error GET "${_gs_lock_json}"
      dependencies "${DEPENDENCY}" ${ARGN})
    if(_error)
      message(FATAL_ERROR
        "dependencies.lock.json: cannot read dependencies.${DEPENDENCY}.${ARGN}: ${_error}")
    endif()
    set(${OUTVAR} "${_value}" PARENT_SCOPE)
  endfunction()
  function(gs_lock_get_nullable DEPENDENCY OUTVAR)
    string(JSON _type ERROR_VARIABLE _error TYPE "${_gs_lock_json}"
      dependencies "${DEPENDENCY}" ${ARGN})
    if(_error)
      message(FATAL_ERROR
        "dependencies.lock.json: cannot read dependencies.${DEPENDENCY}.${ARGN}: ${_error}")
    endif()
    if(_type STREQUAL "NULL")
      set(${OUTVAR} "" PARENT_SCOPE)
    else()
      string(JSON _value GET "${_gs_lock_json}" dependencies "${DEPENDENCY}" ${ARGN})
      set(${OUTVAR} "${_value}" PARENT_SCOPE)
    endif()
  endfunction()

  gs_lock_get(nlohmann_json _gs_json_url source url)
  gs_lock_get_nullable(nlohmann_json _gs_json_sha256 source sha256)
  if(_gs_json_sha256)
    set(_gs_json_hash_args URL_HASH "SHA256=${_gs_json_sha256}")
  else()
    set(_gs_json_hash_args "")
  endif()
  FetchContent_Declare(nlohmann_json URL "${_gs_json_url}"
    ${_gs_json_hash_args} ${_gs_download_args})
  set(JSON_BuildTests OFF CACHE INTERNAL "Locked dependency: upstream tests are not built")
  set(JSON_Install OFF CACHE INTERNAL "Locked dependency: not installed")
  FetchContent_MakeAvailable(nlohmann_json)
  if(NOT TARGET nlohmann_json::nlohmann_json)
    message(FATAL_ERROR "nlohmann_json did not define nlohmann_json::nlohmann_json")
  endif()

  if(PLANNER_BUILD_TESTS)
    gs_lock_get(googletest _gs_gtest_url source url)
    gs_lock_get_nullable(googletest _gs_gtest_sha256 source sha256)
    if(_gs_gtest_sha256)
      set(_gs_gtest_hash_args URL_HASH "SHA256=${_gs_gtest_sha256}")
    else()
      set(_gs_gtest_hash_args "")
    endif()
    FetchContent_Declare(googletest URL "${_gs_gtest_url}"
      ${_gs_gtest_hash_args} ${_gs_download_args})
    set(BUILD_GMOCK OFF CACHE BOOL "Locked dependency: gmock is not needed" FORCE)
    set(INSTALL_GTEST OFF CACHE BOOL "Locked dependency: not installed" FORCE)
    FetchContent_MakeAvailable(googletest)
    if(NOT TARGET GTest::gtest_main)
      message(FATAL_ERROR "googletest did not define GTest::gtest_main")
    endif()
  endif()

  if(NOT TARGET planner::common)
    if(NOT EXISTS "${PLANNER_ROOT_DIR}/Src/common/CMakeLists.txt")
      message(FATAL_ERROR "Src/common/CMakeLists.txt is missing")
    endif()
    set(_gs_saved_tests "${PLANNER_BUILD_TESTS}")
    set(_gs_saved_examples "${PLANNER_BUILD_EXAMPLES}")
    set(PLANNER_BUILD_TESTS OFF)
    set(PLANNER_BUILD_EXAMPLES OFF)
    add_subdirectory("${PLANNER_ROOT_DIR}/Src/common"
      "${CMAKE_BINARY_DIR}/Src/common")
    set(PLANNER_BUILD_TESTS "${_gs_saved_tests}")
    set(PLANNER_BUILD_EXAMPLES "${_gs_saved_examples}")
  endif()
endif()

if(NOT TARGET planner::common)
  message(FATAL_ERROR
    "planner_grid_search requires the planner::common target. Configure "
    "through the root CMakeLists.txt (PLANNER_BUILD_COMMON=ON) or use the "
    "standalone mode (cmake -S Src/grid_search -B <dir>).")
endif()

# ---------------------------------------------------------------------------
# 每适配器独立开关（默认 ON）。开关 OFF 时不编译对应源码、不要求其第三方
# 头文件，available_backends() 也不返回该后端。
# ---------------------------------------------------------------------------
option(PLANNER_GRID_SEARCH_WITH_VSS2SN
  "Build the vss2sn/path_planning grid Dijkstra/A* adapters (BSD-3-Clause)" ON)
option(PLANNER_GRID_SEARCH_WITH_SMAC2D
  "Build the Nav2 SmacPlanner2D grid A* adapter (Apache-2.0, decoupled subset)" ON)
option(PLANNER_GRID_SEARCH_WITH_JPS3D
  "Build the jps3d grid A* / JPS adapters (BSD-3-Clause; needs Eigen3 headers)" ON)
option(PLANNER_GRID_SEARCH_WITH_LIBMULTIROBOT
  "Build the libMultiRobotPlanning grid A* adapter (MIT; needs Boost headers)" ON)
option(PLANNER_GRID_SEARCH_WITH_JUSTINHJ
  "Build the justinhj A* grid adapters (MIT; memory pool on/off)" ON)

# jps3d 需要 Eigen3 头文件（上游 data_type.h）。
set(PLANNER_GRID_SEARCH_JPS3D_ENABLED OFF)
set(PLANNER_GRID_SEARCH_EIGEN_TARGET "")
if(PLANNER_GRID_SEARCH_WITH_JPS3D)
  find_package(Eigen3 QUIET)
  if(TARGET Eigen3::Eigen)
    set(PLANNER_GRID_SEARCH_JPS3D_ENABLED ON)
    set(PLANNER_GRID_SEARCH_EIGEN_TARGET Eigen3::Eigen)
  elseif(EIGEN3_INCLUDE_DIRS)
    set(PLANNER_GRID_SEARCH_JPS3D_ENABLED ON)
  elseif(TARGET planner_eigen_headers)
    # 系统 Eigen 不可用时回退到锁定 Eigen 头目标（与 rs 组件的回退一致）。
    set(PLANNER_GRID_SEARCH_JPS3D_ENABLED ON)
    set(PLANNER_GRID_SEARCH_EIGEN_TARGET planner_eigen_headers)
  else()
    message(WARNING
      "PLANNER_GRID_SEARCH_WITH_JPS3D=ON but Eigen3 headers were not found; "
      "the jps3d adapters are skipped (record this deviation in the report).")
  endif()
endif()

# ---------------------------------------------------------------------------
# planner_grid_search：统一契约核心 + 已启用适配器。
# ---------------------------------------------------------------------------
set(_gs_sources
  src/neighborhood.cpp
  src/types.cpp
  src/distance_field.cpp
  src/backend.cpp
  src/registry.cpp)

set(_gs_compile_definitions "")

if(PLANNER_GRID_SEARCH_WITH_VSS2SN)
  list(APPEND _gs_sources
    src/adapters/vss2sn_adapter.cpp
    thirdparty/vss2sn/src/a_star.cpp
    thirdparty/vss2sn/src/dijkstra.cpp
    thirdparty/vss2sn/src/utils.cpp)
  list(APPEND _gs_compile_definitions PLANNER_GRID_SEARCH_WITH_VSS2SN=1)
  set_source_files_properties(
    src/adapters/vss2sn_adapter.cpp
    thirdparty/vss2sn/src/a_star.cpp
    thirdparty/vss2sn/src/dijkstra.cpp
    thirdparty/vss2sn/src/utils.cpp
    PROPERTIES INCLUDE_DIRECTORIES
      "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/vss2sn/include")
  set_source_files_properties(
    thirdparty/vss2sn/src/a_star.cpp
    thirdparty/vss2sn/src/dijkstra.cpp
    thirdparty/vss2sn/src/utils.cpp
    PROPERTIES COMPILE_OPTIONS "-w")
endif()

if(PLANNER_GRID_SEARCH_WITH_SMAC2D)
  list(APPEND _gs_sources
    src/adapters/smac_2d_adapter.cpp
    thirdparty/nav2_smac_planner/src/node_2d.cpp)
  list(APPEND _gs_compile_definitions PLANNER_GRID_SEARCH_WITH_SMAC2D=1)
  set_source_files_properties(
    src/adapters/smac_2d_adapter.cpp
    thirdparty/nav2_smac_planner/src/node_2d.cpp
    PROPERTIES INCLUDE_DIRECTORIES
      "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/nav2_smac_planner/include;${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/nav2_smac_planner/stubs/include")
  set_source_files_properties(
    thirdparty/nav2_smac_planner/src/node_2d.cpp
    PROPERTIES COMPILE_OPTIONS "-w")
endif()

if(PLANNER_GRID_SEARCH_JPS3D_ENABLED)
  list(APPEND _gs_sources
    src/adapters/jps3d_adapter.cpp
    thirdparty/jps3d/src/jps_planner/graph_search.cpp
    thirdparty/jps3d/src/jps_planner/jps_planner.cpp)
  list(APPEND _gs_compile_definitions PLANNER_GRID_SEARCH_WITH_JPS3D=1)
  if(TARGET Eigen3::Eigen)
    get_target_property(_gs_eigen_includes Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
  elseif(TARGET planner_eigen_headers)
    get_target_property(_gs_eigen_includes planner_eigen_headers INTERFACE_INCLUDE_DIRECTORIES)
  else()
    set(_gs_eigen_includes "${EIGEN3_INCLUDE_DIRS}")
  endif()
  set_source_files_properties(
    src/adapters/jps3d_adapter.cpp
    thirdparty/jps3d/src/jps_planner/graph_search.cpp
    thirdparty/jps3d/src/jps_planner/jps_planner.cpp
    PROPERTIES INCLUDE_DIRECTORIES
      "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/jps3d/include;${_gs_eigen_includes}")
  set_source_files_properties(
    thirdparty/jps3d/src/jps_planner/graph_search.cpp
    thirdparty/jps3d/src/jps_planner/jps_planner.cpp
    PROPERTIES COMPILE_OPTIONS "-w")
endif()

if(PLANNER_GRID_SEARCH_WITH_LIBMULTIROBOT)
  list(APPEND _gs_sources src/adapters/libmultirobot_adapter.cpp)
  list(APPEND _gs_compile_definitions PLANNER_GRID_SEARCH_WITH_LIBMULTIROBOT=1)
  set_source_files_properties(
    src/adapters/libmultirobot_adapter.cpp
    PROPERTIES INCLUDE_DIRECTORIES
      "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/libMultiRobotPlanning/include")
endif()

if(PLANNER_GRID_SEARCH_WITH_JUSTINHJ)
  list(APPEND _gs_sources
    src/adapters/justinhj_adapter.cpp
    src/adapters/justinhj_nopool_adapter.cpp)
  list(APPEND _gs_compile_definitions PLANNER_GRID_SEARCH_WITH_JUSTINHJ=1)
  set_source_files_properties(
    src/adapters/justinhj_adapter.cpp
    src/adapters/justinhj_nopool_adapter.cpp
    PROPERTIES INCLUDE_DIRECTORIES
      "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/justinhj/include")
  # 内存池关闭配置：上游把 USE_FSA_MEMORY 硬编码为 1（1 行级补丁，默认行为
  # 不变）；本翻译单元以 USE_FSA_MEMORY=0 编译。
  set_source_files_properties(
    src/adapters/justinhj_nopool_adapter.cpp
    PROPERTIES COMPILE_DEFINITIONS "USE_FSA_MEMORY=0")
endif()

add_library(planner_grid_search ${_gs_sources})
add_library(planner::grid_search ALIAS planner_grid_search)

target_compile_features(planner_grid_search PUBLIC cxx_std_17)
target_include_directories(planner_grid_search PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_compile_definitions(planner_grid_search PRIVATE ${_gs_compile_definitions})
target_link_libraries(planner_grid_search PUBLIC planner::common)
if(PLANNER_GRID_SEARCH_EIGEN_TARGET)
  target_link_libraries(planner_grid_search PRIVATE ${PLANNER_GRID_SEARCH_EIGEN_TARGET})
elseif(TARGET Eigen3::Eigen)
  target_link_libraries(planner_grid_search PRIVATE Eigen3::Eigen)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_grid_search PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# 示例（PLANNER_BUILD_EXAMPLES，默认 ON）：独立示例 + CTest 自检。
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_EXAMPLES)
  add_executable(planner_grid_search_demo examples/grid_search_demo.cpp)
  target_link_libraries(planner_grid_search_demo PRIVATE planner_grid_search)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_grid_search_demo PRIVATE -Wall -Wextra -Wpedantic)
  endif()
  if(PLANNER_BUILD_TESTS)
    add_test(NAME planner_grid_search_demo_self_check
      COMMAND planner_grid_search_demo --self-check)
    set_tests_properties(planner_grid_search_demo_self_check PROPERTIES
      LABELS "grid_search")
  endif()
endif()

# ---------------------------------------------------------------------------
# 测试（GoogleTest + CTest，标签 "grid_search"；正确性档，性能属 X-B1）。
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_TESTS)
  if(NOT TARGET GTest::gtest_main)
    message(FATAL_ERROR
      "PLANNER_BUILD_TESTS=ON but GTest::gtest_main is not available. "
      "Configure through the root CMakeLists.txt (which fetches googletest "
      "from dependencies.lock.json) or use the standalone mode.")
  endif()
  if(PLANNER_GRID_SEARCH_STANDALONE)
    enable_testing()
  endif()
  add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------
# 基准（X-B1）：PLANNER_BUILD_BENCHMARKS 默认 ON（与根选项一致）。生成器与
# 运行器位于 benchmarks/；不注册 CTest，开发期经 tools/task_runner.py exec、
# 独占窗口经注册后的 benchmark 子命令运行（见 benchmarks/README.md 与
# benchmarks/benchmark_suite.json）。
# ---------------------------------------------------------------------------
if(NOT DEFINED PLANNER_BUILD_BENCHMARKS)
  option(PLANNER_BUILD_BENCHMARKS "Build component benchmarks" ON)
endif()
if(PLANNER_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()
