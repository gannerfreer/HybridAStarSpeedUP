# planner_sampling_search：二维 RRT* 多实现适配（X-S4）。
#
# 职责与依赖边界（冻结计划 §2.1）：依赖 planner::common 与夹具侧轻量二维
# 有效性回调；不依赖显示、地图服务或 ROS。公共目标 planner_sampling_search
# 不暴露任何上游类型（OMPL / Ch0p1k3 / Klamp't / vss2sn / ai-winter）。
#
# 独立可配置：本文件既可由根 CMakeLists.txt 通过
#   planner_add_optional_component(PLANNER_BUILD_SAMPLING_SEARCH Src/sampling_search)
# 添加，也可独立配置（standalone 分支会自行获取 common/GoogleTest/依赖锁条目）：
#   cmake -S Src/sampling_search -B build/sampling_search \
#         -DCMAKE_BUILD_TYPE=Release
#
# 适配器独立开关（未启用不下载、不链接；ai-winter 为 GPL-3.0 隔离组，默认 OFF）：
#   PLANNER_SAMPLING_SEARCH_WITH_OMPL      （默认 ON，复用锁 1.6.0 条目）
#   PLANNER_SAMPLING_SEARCH_WITH_CH0P1K3   （默认 ON，vendored 最小抽取 + pugixml）
#   PLANNER_SAMPLING_SEARCH_WITH_KLAMPT    （默认 ON，固定 URL+SHA256 拉取 48 文件）
#   PLANNER_SAMPLING_SEARCH_WITH_VSS2SN    （默认 ON，vendored 最小抽取）
#   PLANNER_SAMPLING_SEARCH_WITH_AIWINTER  （默认 OFF，GPL-3.0 隔离；适配延期）
#
# 依赖锁定：OMPL / pugixml / nlohmann_json / googletest 读取
# dependencies.lock.json（根项目提供 planner_lock_get 时直接复用；standalone
# 时本文件自带只读解析）。Klamp't 暂无锁条目，使用组件内固定 URL + SHA256
# （两次独立下载复算一致），待 wiring 阶段写入依赖锁；可通过
# -DFETCHCONTENT_SOURCE_DIR_SAMPLING_SEARCH_KLAMPT=<dir> 使用离线源码树。

cmake_minimum_required(VERSION 3.19)

# ---------------------------------------------------------------------------
# standalone 引导：本目录即顶层工程时自建设置。
# ---------------------------------------------------------------------------
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(planner_sampling_search_standalone LANGUAGES CXX)
  set(PLANNER_SAMPLING_SEARCH_STANDALONE ON)
  enable_testing()
  if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
  endif()
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
  if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  endif()
endif()

if(NOT DEFINED PLANNER_ROOT_DIR)
  get_filename_component(PLANNER_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
endif()
if(NOT DEFINED PLANNER_BUILD_TESTS)
  if(DEFINED BUILD_TESTING)
    set(PLANNER_BUILD_TESTS "${BUILD_TESTING}")
  else()
    set(PLANNER_BUILD_TESTS ON)
  endif()
endif()
if(NOT DEFINED PLANNER_BUILD_EXAMPLES)
  option(PLANNER_BUILD_EXAMPLES "Build component examples" ON)
endif()
if(NOT DEFINED FETCHCONTENT_TRY_FIND_PACKAGE_MODE)
  set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE NEVER)
endif()
include(FetchContent)

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
  set(PLANNER_SS_TIMESTAMP_ARGS DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
else()
  set(PLANNER_SS_TIMESTAMP_ARGS "")
endif()

option(PLANNER_SAMPLING_SEARCH_WITH_OMPL
  "OMPL 1.6.0 RRT*/Informed RRT* adapter (locked dependency)" ON)
option(PLANNER_SAMPLING_SEARCH_WITH_CH0P1K3
  "Ch0p1k3 RRT* adapter (vendored minimal extraction, MIT)" ON)
option(PLANNER_SAMPLING_SEARCH_WITH_KLAMPT
  "Klamp't/KrisLibrary native rrt* adapter (pinned URL + SHA256, BSD-3-Clause)" ON)
option(PLANNER_SAMPLING_SEARCH_WITH_VSS2SN
  "vss2sn grid RRT* adapter (vendored minimal extraction, BSD-3-Clause)" ON)
option(PLANNER_SAMPLING_SEARCH_WITH_AIWINTER
  "ai-winter RRT* adapter (GPL-3.0 isolation group; adaptation deferred, default OFF)" OFF)

# ---------------------------------------------------------------------------
# 依赖锁读取（standalone 自带解析；根项目场景下变量已就绪时同样可用）。
# ---------------------------------------------------------------------------
set(PLANNER_SS_LOCK "${PLANNER_ROOT_DIR}/dependencies.lock.json")
if(NOT EXISTS "${PLANNER_SS_LOCK}")
  message(FATAL_ERROR "dependencies.lock.json is missing at ${PLANNER_SS_LOCK}")
endif()
file(READ "${PLANNER_SS_LOCK}" PLANNER_SS_LOCK_JSON)

function(planner_ss_lock_has DEPENDENCY OUTVAR)
  string(JSON _ss_type ERROR_VARIABLE _ss_err TYPE
    "${PLANNER_SS_LOCK_JSON}" dependencies "${DEPENDENCY}")
  if(_ss_err OR _ss_type STREQUAL "NULL")
    set(${OUTVAR} FALSE PARENT_SCOPE)
  else()
    set(${OUTVAR} TRUE PARENT_SCOPE)
  endif()
endfunction()

function(planner_ss_lock_get DEPENDENCY OUTVAR)
  string(JSON _ss_value ERROR_VARIABLE _ss_err GET
    "${PLANNER_SS_LOCK_JSON}" dependencies "${DEPENDENCY}" ${ARGN})
  if(_ss_err)
    message(FATAL_ERROR
      "dependencies.lock.json: cannot read dependencies.${DEPENDENCY}.${ARGN}: ${_ss_err}")
  endif()
  set(${OUTVAR} "${_ss_value}" PARENT_SCOPE)
endfunction()

# ---------------------------------------------------------------------------
# 依赖组件与测试框架（standalone 时自行获取；根项目场景复用既有目标）。
# ---------------------------------------------------------------------------
if(NOT TARGET nlohmann_json::nlohmann_json)
  planner_ss_lock_get(nlohmann_json _ss_json_tag tag)
  planner_ss_lock_get(nlohmann_json _ss_json_url source url)
  planner_ss_lock_get(nlohmann_json _ss_json_sha source sha256)
  message(STATUS "sampling_search standalone: fetching nlohmann/json ${_ss_json_tag}")
  FetchContent_Declare(nlohmann_json
    URL "${_ss_json_url}"
    URL_HASH "SHA256=${_ss_json_sha}"
    ${PLANNER_SS_TIMESTAMP_ARGS})
  set(JSON_BuildTests OFF CACHE INTERNAL "sampling_search: upstream tests off")
  set(JSON_Install OFF CACHE INTERNAL "sampling_search: no install")
  FetchContent_MakeAvailable(nlohmann_json)
endif()

if(PLANNER_BUILD_TESTS AND NOT TARGET GTest::gtest_main)
  planner_ss_lock_get(googletest _ss_gtest_tag tag)
  planner_ss_lock_get(googletest _ss_gtest_url source url)
  planner_ss_lock_get(googletest _ss_gtest_sha source sha256)
  message(STATUS "sampling_search standalone: fetching googletest ${_ss_gtest_tag}")
  FetchContent_Declare(googletest
    URL "${_ss_gtest_url}"
    URL_HASH "SHA256=${_ss_gtest_sha}"
    ${PLANNER_SS_TIMESTAMP_ARGS})
  set(BUILD_GMOCK OFF CACHE BOOL "sampling_search: gmock off" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "sampling_search: no install" FORCE)
  FetchContent_MakeAvailable(googletest)
endif()

if(NOT TARGET planner::common)
  if(NOT EXISTS "${PLANNER_ROOT_DIR}/Src/common/CMakeLists.txt")
    message(FATAL_ERROR
      "planner_sampling_search requires Src/common (planner::common); "
      "configure through the root CMakeLists.txt with PLANNER_BUILD_COMMON=ON.")
  endif()
  add_subdirectory("${PLANNER_ROOT_DIR}/Src/common" "${CMAKE_BINARY_DIR}/Src/common")
endif()

# ---------------------------------------------------------------------------
# OMPL 1.6.0（锁条目；复用 RS 阶段的精简构建标志）。
# ---------------------------------------------------------------------------
set(PLANNER_SS_OMPL_AVAILABLE OFF)
if(PLANNER_SAMPLING_SEARCH_WITH_OMPL)
  if(TARGET ompl)
    set(PLANNER_SS_OMPL_AVAILABLE ON)
  else()
    planner_ss_lock_has(ompl _ss_has_ompl)
    if(_ss_has_ompl)
      planner_ss_lock_get(ompl _ss_ompl_tag tag)
      planner_ss_lock_get(ompl _ss_ompl_url source url)
      planner_ss_lock_get(ompl _ss_ompl_sha source sha256)
      message(STATUS "Dependency ompl (sampling_search): ${_ss_ompl_tag}")
      FetchContent_Declare(ompl
        URL "${_ss_ompl_url}"
        URL_HASH "SHA256=${_ss_ompl_sha}"
        ${PLANNER_SS_TIMESTAMP_ARGS})
      set(OMPL_BUILD_TESTS OFF CACHE BOOL "sampling_search: upstream tests off" FORCE)
      set(OMPL_BUILD_DEMOS OFF CACHE BOOL "sampling_search: upstream demos off" FORCE)
      set(OMPL_BUILD_PYBINDINGS OFF CACHE BOOL "sampling_search: pybindings off" FORCE)
      set(OMPL_BUILD_PYTESTS OFF CACHE BOOL "sampling_search: pytest off" FORCE)
      set(OMPL_REGISTRATION OFF CACHE BOOL "sampling_search: no registration" FORCE)
      FetchContent_MakeAvailable(ompl)
      if(TARGET ompl)
        set(PLANNER_SS_OMPL_AVAILABLE ON)
      else()
        message(WARNING
          "OMPL was populated but did not define the 'ompl' target; the OMPL "
          "adapter is skipped.")
      endif()
    else()
      message(WARNING
        "PLANNER_SAMPLING_SEARCH_WITH_OMPL=ON but dependencies.lock.json has "
        "no ompl entry; the OMPL adapter is skipped.")
    endif()
  endif()
endif()

# ---------------------------------------------------------------------------
# pugixml（Ch0p1k3 的 XML 输入解析；锁条目 1.16，与 map 组件一致）。
# ---------------------------------------------------------------------------
set(PLANNER_SS_PUGIXML_AVAILABLE OFF)
if(PLANNER_SAMPLING_SEARCH_WITH_CH0P1K3)
  if(TARGET pugixml::pugixml)
    set(PLANNER_SS_PUGIXML_AVAILABLE ON)
  else()
    planner_ss_lock_has(pugixml _ss_has_pugixml)
    if(_ss_has_pugixml)
      planner_ss_lock_get(pugixml _ss_pugi_tag tag)
      planner_ss_lock_get(pugixml _ss_pugi_url source url)
      planner_ss_lock_get(pugixml _ss_pugi_sha source sha256)
      message(STATUS "Dependency pugixml (sampling_search): ${_ss_pugi_tag}")
      FetchContent_Declare(pugixml
        URL "${_ss_pugi_url}"
        URL_HASH "SHA256=${_ss_pugi_sha}"
        ${PLANNER_SS_TIMESTAMP_ARGS})
      set(PUGIXML_BUILD_TESTS OFF CACHE BOOL "sampling_search: upstream tests off" FORCE)
      FetchContent_MakeAvailable(pugixml)
      if(TARGET pugixml::pugixml)
        set(PLANNER_SS_PUGIXML_AVAILABLE ON)
      endif()
    else()
      message(WARNING
        "PLANNER_SAMPLING_SEARCH_WITH_CH0P1K3=ON but dependencies.lock.json "
        "has no pugixml entry; the Ch0p1k3 adapter is skipped.")
    endif()
  endif()
endif()

# ---------------------------------------------------------------------------
# Klamp't / KrisLibrary（固定 commit 归档；仅拉取源码，不构建上游工程）。
# ---------------------------------------------------------------------------
set(PLANNER_SS_KLAMPT_AVAILABLE OFF)
if(PLANNER_SAMPLING_SEARCH_WITH_KLAMPT)
  planner_ss_lock_has(klampt _ss_has_klampt)
  if(_ss_has_klampt)
    planner_ss_lock_get(klampt _ss_klampt_url source url)
    planner_ss_lock_get(klampt _ss_klampt_sha source sha256)
  else()
    # 组件内固定来源（2026-10-12 两次独立下载复算一致；待 wiring 写入依赖锁）。
    set(_ss_klampt_url
      "https://github.com/krishauser/KrisLibrary/archive/f1c9ba76e287d9a3966b1ab3f870fa96e159ef7f.tar.gz")
    set(_ss_klampt_sha
      "e1b36dac98ff5df1fda5aa597559b6ff42373d8551bac99d22c358e7c04960e2")
  endif()
  FetchContent_Declare(sampling_search_klampt
    URL "${_ss_klampt_url}"
    URL_HASH "SHA256=${_ss_klampt_sha}"
    SOURCE_SUBDIR cmake/planner-sampling-search-no-upstream-build
    ${PLANNER_SS_TIMESTAMP_ARGS})
  FetchContent_MakeAvailable(sampling_search_klampt)
  if(EXISTS "${sampling_search_klampt_SOURCE_DIR}/planning/OptimalMotionPlanner.cpp")
    set(PLANNER_SS_KLAMPT_AVAILABLE ON)
  else()
    message(WARNING "Klamp't source tree incomplete; adapter skipped.")
  endif()
endif()

# ---------------------------------------------------------------------------
# ai-winter（GPL-3.0 隔离组，默认 OFF；适配延期，见 provenance.md）。
# ---------------------------------------------------------------------------
if(PLANNER_SAMPLING_SEARCH_WITH_AIWINTER)
  message(FATAL_ERROR
    "PLANNER_SAMPLING_SEARCH_WITH_AIWINTER=ON: the ai-winter adapter is "
    "deferred (probe evidence covers feasibility; see "
    "Src/sampling_search/provenance.md). This target must stay OFF and "
    "GPL-3.0-isolated until the adaptation lands.")
endif()

# ---------------------------------------------------------------------------
# 上游静态库（仅编译所需最小文件集；上游代码不参与公共头文件）。
# ---------------------------------------------------------------------------
if(PLANNER_SS_PUGIXML_AVAILABLE)
  add_library(planner_sampling_search_ch0p1k3_upstream STATIC
    third_party/ch0p1k3/src/geometry.cpp
    third_party/ch0p1k3/src/map.cpp
    third_party/ch0p1k3/src/obstacle.cpp
    third_party/ch0p1k3/src/algorithm.cpp
    third_party/ch0p1k3/src/tree.cpp
    third_party/ch0p1k3/src/rrt.cpp
    third_party/ch0p1k3/src/rrtstar.cpp)
  target_compile_features(planner_sampling_search_ch0p1k3_upstream PRIVATE cxx_std_17)
  target_include_directories(planner_sampling_search_ch0p1k3_upstream PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ch0p1k3/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nanoflann"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ch0p1k3/sfml_stub")
  target_link_libraries(planner_sampling_search_ch0p1k3_upstream PRIVATE pugixml::pugixml)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_sampling_search_ch0p1k3_upstream PRIVATE
      -Wall -Wno-error -Wno-unused-parameter -Wno-unused-variable)
  endif()
endif()

if(PLANNER_SS_KLAMPT_AVAILABLE)
  set(PLANNER_SS_KLAMPT_SRCS
    errors.cpp Logger.cpp Timer.cpp File.cpp
    planning/CSpace.cpp planning/CSpaceHelpers.cpp planning/EdgePlanner.cpp
    planning/Interpolator.cpp planning/MotionPlanner.cpp planning/Objective.cpp
    planning/OptimalMotionPlanner.cpp planning/Path.cpp planning/PointLocation.cpp
    planning/AnyMotionPlanner.cpp planning/SBL.cpp planning/SBLTree.cpp
    planning/FMM.cpp planning/FMMMotionPlanner.cpp planning/DensityEstimator.cpp
    planning/CSetHelpers.cpp
    optimization/NonlinearProgram.cpp
    geometry/KDTree.cpp geometry/BallTree.cpp geometry/GridSubdivision.cpp
    spline/PiecewisePolynomial.cpp
    utils/unionfind.cpp utils/socketutils.cpp utils/PropertyMap.cpp
    utils/PrimitiveValue.cpp utils/AnyCollection.cpp utils/indexing.cpp
    utils/ioutils.cpp utils/stringutils.cpp
    math/complex.cpp math/MatrixTemplate.cpp math/VectorTemplate.cpp
    math3d/primitives.cpp
    math/metric.cpp math/random.cpp math/sample.cpp math/misc.cpp
    math/function.cpp math/infnan.cpp math/vectorfunction.cpp
    math/LinearPath.cpp math/angle.cpp math/AABB.cpp
    graph/IO.cpp)
  set(_ss_klampt_full_srcs "")
  foreach(_ss_src IN LISTS PLANNER_SS_KLAMPT_SRCS)
    list(APPEND _ss_klampt_full_srcs
      "${sampling_search_klampt_SOURCE_DIR}/${_ss_src}")
  endforeach()
  add_library(planner_sampling_search_klampt_upstream STATIC ${_ss_klampt_full_srcs})
  target_compile_features(planner_sampling_search_klampt_upstream PRIVATE cxx_std_17)
  # 上游头文件使用 <KrisLibrary/...> 路径；建立 include 根符号链接。
  file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/klampt_include")
  if(NOT EXISTS "${CMAKE_CURRENT_BINARY_DIR}/klampt_include/KrisLibrary")
    file(CREATE_LINK "${sampling_search_klampt_SOURCE_DIR}"
      "${CMAKE_CURRENT_BINARY_DIR}/klampt_include/KrisLibrary" SYMBOLIC)
  endif()
  target_include_directories(planner_sampling_search_klampt_upstream PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/klampt_include"
    "${sampling_search_klampt_SOURCE_DIR}")
  target_compile_definitions(planner_sampling_search_klampt_upstream PRIVATE
    HAS_STD_CMATH=1)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_sampling_search_klampt_upstream PRIVATE
      -Wall -Wno-error -Wno-unused-parameter -Wno-unused-variable
      -Wno-unused-but-set-variable -Wno-maybe-uninitialized)
  endif()
endif()

if(PLANNER_SAMPLING_SEARCH_WITH_VSS2SN)
  add_library(planner_sampling_search_vss2sn_upstream STATIC
    third_party/vss2sn/src/rrt_star.cpp
    third_party/vss2sn/src/utils.cpp)
  target_compile_features(planner_sampling_search_vss2sn_upstream PRIVATE cxx_std_17)
  target_include_directories(planner_sampling_search_vss2sn_upstream PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/vss2sn/include")
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_sampling_search_vss2sn_upstream PRIVATE
      -Wall -Wno-error -Wno-unused-parameter)
  endif()
endif()

# ---------------------------------------------------------------------------
# 公共目标。
# ---------------------------------------------------------------------------
add_library(planner_sampling_search
  src/core.cpp
  src/adapter_ompl.cpp
  src/adapter_ch0p1k3.cpp
  src/adapter_klampt.cpp
  src/adapter_vss2sn.cpp)

add_library(planner::sampling_search ALIAS planner_sampling_search)

target_compile_features(planner_sampling_search PUBLIC cxx_std_17)
target_include_directories(planner_sampling_search PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(planner_sampling_search PUBLIC planner::common)

if(PLANNER_SS_OMPL_AVAILABLE)
  FetchContent_GetProperties(ompl
    SOURCE_DIR _ss_ompl_src_dir BINARY_DIR _ss_ompl_bin_dir)
  if(NOT _ss_ompl_src_dir)
    set(_ss_ompl_src_dir "${ompl_SOURCE_DIR}")
    set(_ss_ompl_bin_dir "${ompl_BINARY_DIR}")
  endif()
  target_compile_definitions(planner_sampling_search PRIVATE
    PLANNER_SAMPLING_SEARCH_HAS_OMPL=1)
  target_link_libraries(planner_sampling_search PRIVATE ompl)
  target_include_directories(planner_sampling_search PRIVATE
    "${_ss_ompl_bin_dir}/src"
    "${_ss_ompl_src_dir}/src")
  find_package(Eigen3 QUIET)
  if(TARGET Eigen3::Eigen)
    target_link_libraries(planner_sampling_search PRIVATE Eigen3::Eigen)
  elseif(EIGEN3_INCLUDE_DIRS)
    target_include_directories(planner_sampling_search PRIVATE ${EIGEN3_INCLUDE_DIRS})
  endif()
endif()

if(PLANNER_SS_PUGIXML_AVAILABLE)
  target_compile_definitions(planner_sampling_search PRIVATE
    PLANNER_SAMPLING_SEARCH_HAS_CH0P1K3=1)
  target_link_libraries(planner_sampling_search PRIVATE
    planner_sampling_search_ch0p1k3_upstream
    pugixml::pugixml)
  target_include_directories(planner_sampling_search PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ch0p1k3/src"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/nanoflann"
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/ch0p1k3/sfml_stub")
endif()

if(PLANNER_SS_KLAMPT_AVAILABLE)
  target_compile_definitions(planner_sampling_search PRIVATE
    PLANNER_SAMPLING_SEARCH_HAS_KLAMPT=1)
  target_link_libraries(planner_sampling_search PRIVATE
    planner_sampling_search_klampt_upstream)
  target_include_directories(planner_sampling_search PRIVATE
    "${CMAKE_CURRENT_BINARY_DIR}/klampt_include"
    "${sampling_search_klampt_SOURCE_DIR}")
  target_compile_definitions(planner_sampling_search PRIVATE HAS_STD_CMATH=1)
endif()

if(PLANNER_SAMPLING_SEARCH_WITH_VSS2SN)
  target_compile_definitions(planner_sampling_search PRIVATE
    PLANNER_SAMPLING_SEARCH_HAS_VSS2SN=1)
  target_link_libraries(planner_sampling_search PRIVATE
    planner_sampling_search_vss2sn_upstream)
  target_include_directories(planner_sampling_search PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/vss2sn/include")
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_sampling_search PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# 示例（独立自检；PLANNER_BUILD_EXAMPLES，默认 ON）。
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_EXAMPLES)
  add_executable(planner_sampling_search_demo examples/sampling_search_demo.cpp)
  target_link_libraries(planner_sampling_search_demo PRIVATE planner_sampling_search)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_sampling_search_demo PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

# ---------------------------------------------------------------------------
# 测试（GoogleTest + CTest，标签 "sampling_search"）。
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_TESTS)
  if(NOT TARGET GTest::gtest_main)
    message(FATAL_ERROR
      "PLANNER_BUILD_TESTS=ON but GTest::gtest_main is not available. "
      "Configure through the root CMakeLists.txt (which fetches googletest) "
      "or let the standalone bootstrap fetch it.")
  endif()
  add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------
# 基准（X-B1；默认随全局 PLANNER_BUILD_BENCHMARKS；注册片段见
# benchmarks/benchmark_suite.json，正式矩阵在独占窗口经入口运行）。
# ---------------------------------------------------------------------------
if(NOT DEFINED PLANNER_BUILD_BENCHMARKS)
  option(PLANNER_BUILD_BENCHMARKS "Build component benchmarks" ON)
endif()
if(PLANNER_BUILD_BENCHMARKS)
  add_subdirectory(benchmarks)
endif()
