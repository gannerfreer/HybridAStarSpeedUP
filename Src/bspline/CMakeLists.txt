# planner_bspline：第一版 B-spline 曲线（TinySpline 0.6.0 封装；Eigen Splines
# 与 EGO-Planner UniformBspline 为内部对照后端）。
#
# 由根 CMakeLists.txt 添加（PLANNER_BUILD_BSPLINE=ON）。锁定的 TinySpline
# 源码在此通过 dependencies.lock.json 中的 URL + URL_HASH 获取
# （planner_lock_get 由根项目提供）。TinySpline 类型对库为 PRIVATE；
# 公开 API 不暴露后端。EGO 对照代码为 GPL-3.0，编译进独立的
# planner_bspline_ego 库（只被测试/基准链接），主库不包含（见
# src/upstream_ego/PROVENANCE.md）。

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

# X-A1-03：GPL-3.0 EGO 对照后端独立开关（默认 ON，保持既有行为）。
# OFF 时不编译 planner_bspline_ego，测试与基准也不链接它。
option(PLANNER_BSPLINE_WITH_EGO
  "Build planner_bspline_ego (GPL-3.0 EGO UniformBspline reference; tests/benchmarks only)" ON)

if(NOT TARGET planner::common)
  message(FATAL_ERROR
    "planner_bspline requires the planner::common target. Configure through "
    "the root CMakeLists.txt (PLANNER_BUILD_COMMON=ON).")
endif()

# ---------------------------------------------------------------------------
# 锁定的 TinySpline 0.6.0（依赖锁条目 `tinyspline`）。
# ---------------------------------------------------------------------------
if(NOT COMMAND planner_lock_get)
  message(FATAL_ERROR
    "planner_lock_get() is not defined; the B-spline component must be added "
    "from the root CMakeLists.txt, which reads dependencies.lock.json.")
endif()

planner_lock_get(tinyspline _tinyspline_tag tag)
planner_lock_get(tinyspline _tinyspline_commit commit)
planner_lock_get(tinyspline _tinyspline_url source url)
planner_lock_get_nullable(tinyspline _tinyspline_sha256 source sha256)

message(STATUS "Dependency TinySpline: ${_tinyspline_tag} (${_tinyspline_commit})")
if(_tinyspline_sha256)
  set(_tinyspline_hash_args URL_HASH "SHA256=${_tinyspline_sha256}")
else()
  set(_tinyspline_hash_args "")
  message(WARNING
    "dependencies.lock.json has no sha256 for tinyspline (status pending); "
    "the download will not be hash-verified")
endif()

# 上游默认会构建测试、示例、文档和 C++ 接口；此处只需要静态 C 库。
# TinySpline 自身的 cmake_minimum_required(VERSION 3.12) 会在其目录作用域内
# 将 CMP0077 降级，因此 option() 会忽略普通变量：改为强制设置缓存值。
set(TINYSPLINE_BUILD_TESTS OFF CACHE BOOL
    "planner_bspline: TinySpline tests are not built" FORCE)
set(TINYSPLINE_BUILD_EXAMPLES OFF CACHE BOOL
    "planner_bspline: TinySpline examples are not built" FORCE)
set(TINYSPLINE_BUILD_DOCS OFF CACHE BOOL
    "planner_bspline: TinySpline documentation is not built" FORCE)
set(TINYSPLINE_ENABLE_CXX OFF CACHE BOOL
    "planner_bspline: only the TinySpline C interface is used" FORCE)
set(TINYSPLINE_WARNINGS_AS_ERRORS OFF CACHE BOOL
    "planner_bspline: upstream warnings are not part of the quality gate" FORCE)

FetchContent_Declare(tinyspline
  URL "${_tinyspline_url}"
  ${_tinyspline_hash_args}
  ${PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS})
FetchContent_MakeAvailable(tinyspline)

if(NOT TARGET tinyspline)
  message(FATAL_ERROR
    "TinySpline was populated from '${_tinyspline_url}' but did not define the "
    "'tinyspline' target; the locked source package is broken.")
endif()

# ---------------------------------------------------------------------------
# 锁定的 Eigen 3.4.0（依赖锁条目 `eigen`）：Eigen Splines 对照后端。
# 仅头文件库：以 FetchContent_Populate + INTERFACE 目标消费，不运行上游
# CMake —— 避免其全局 `uninstall` 自定义目标与 OMPL 等同树依赖冲突，
# 也避免与 OMPL 经 find_package 引入的系统 Eigen3::Eigen 混淆。
#
# X-A1-04 决策：Eigen 后端（BackendKind::kEigenSplines）是主库公开后端契约
# 的一部分（curve.hpp 的 BackendKind 与 backend() 名称都包含它），因此
# planner_eigen_headers 记录为 planner_bspline 的**必要**依赖，不加开关；
# 可选化的只是 EGO 对照库（X-A1-03）。若未来把 Eigen 后端改为可选，需要
# 同步修改公开枚举与文档，属独立决定。
# ---------------------------------------------------------------------------
if(NOT TARGET planner_eigen_headers)
  planner_lock_get(eigen _eigen_tag tag)
  planner_lock_get(eigen _eigen_commit commit)
  planner_lock_get(eigen _eigen_url source url)
  planner_lock_get_nullable(eigen _eigen_sha256 source sha256)

  message(STATUS "Dependency Eigen: ${_eigen_tag} (${_eigen_commit})")
  if(_eigen_sha256)
    set(_eigen_hash_args URL_HASH "SHA256=${_eigen_sha256}")
  else()
    set(_eigen_hash_args "")
    message(WARNING
      "dependencies.lock.json has no sha256 for eigen (status pending); "
      "the download will not be hash-verified")
  endif()

  FetchContent_Declare(eigen
    URL "${_eigen_url}"
    ${_eigen_hash_args}
    ${PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS})
  FetchContent_Populate(eigen)
  if(NOT EXISTS "${eigen_SOURCE_DIR}/unsupported/Eigen/Splines")
    message(FATAL_ERROR
      "Eigen source tree at '${eigen_SOURCE_DIR}' is missing "
      "unsupported/Eigen/Splines; the locked source package is broken.")
  endif()
  add_library(planner_eigen_headers INTERFACE)
  target_include_directories(planner_eigen_headers INTERFACE
    "${eigen_SOURCE_DIR}")
endif()

# ---------------------------------------------------------------------------
# 库
# ---------------------------------------------------------------------------
add_library(planner_bspline
  src/curve.cpp
  src/detail.cpp
  src/resample.cpp
  src/tinyspline_backend.cpp
  src/eigen_backend.cpp)

add_library(planner::bspline ALIAS planner_bspline)

target_compile_features(planner_bspline PUBLIC cxx_std_17)
target_include_directories(planner_bspline PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
# planner::common 为 PUBLIC（公开头文件使用 planner::common::Point2d）；
# TinySpline 与 Eigen 保持 PRIVATE（公开 API 中不含上游类型）。
target_link_libraries(planner_bspline
  PUBLIC planner::common
  PRIVATE tinyspline planner_eigen_headers)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_bspline PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# 对照库 planner_bspline_ego：EGO-Planner UniformBspline（GPL-3.0，仅对照）。
# 独立静态库，只被测试与基准可执行文件链接；planner_bspline 主库不编译、
# 不链接 GPL-3.0 代码。来源、复制范围与隔离说明见
# src/upstream_ego/PROVENANCE.md。X-A1-03：由 PLANNER_BSPLINE_WITH_EGO
# （默认 ON）独立控制；OFF 时本目标不存在。
# ---------------------------------------------------------------------------
if(PLANNER_BSPLINE_WITH_EGO)
  add_library(planner_bspline_ego STATIC
    src/upstream_ego/src/uniform_bspline.cpp
    src/ego_backend.cpp)

  target_compile_features(planner_bspline_ego PRIVATE cxx_std_17)
  # 上游布局：include/bspline_opt/...，cpp 的原始包含路径因此保持不变。
  target_include_directories(planner_bspline_ego PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/src/upstream_ego/include")
  target_link_libraries(planner_bspline_ego
    PUBLIC planner_bspline
    PRIVATE planner_eigen_headers)

  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_bspline_ego PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  # 上游 getMeanAndMaxVel/Acc 在 getTimeSpan 失败路径上有 maybe-uninitialized
  # 警告（tm/tmp 未初始化）；上游代码逐字保留，只对该文件抑制该警告
  # （GCC 专属，见 PROVENANCE.md §2）。
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set_source_files_properties(src/upstream_ego/src/uniform_bspline.cpp
      PROPERTIES COMPILE_OPTIONS "-Wno-maybe-uninitialized")
  endif()
endif()

# ---------------------------------------------------------------------------
# 命令行示例（随组件一起构建；PLANNER_BUILD_EXAMPLES 默认 ON）。
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_EXAMPLES)
  add_executable(planner_bspline_example examples/bspline_demo.cpp)
  target_link_libraries(planner_bspline_example PRIVATE planner_bspline)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_bspline_example PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

# ---------------------------------------------------------------------------
# 微基准测试二进制：{build_dir}/Src/bspline/planner_bspline_bench。
# 默认读取 tests/fixtures/v1/benchmarks/bspline/inputs.json；按 tools/README.md
# 第 7.2 节接受 --suite v1 --output-dir <dir>。
# 基准包含 ego_uniform 对照（--backend ego），因此需要 EGO 对照库；
# PLANNER_BSPLINE_WITH_EGO=OFF 时不构建基准（X-A1-03）。
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_BENCHMARKS AND PLANNER_BSPLINE_WITH_EGO)
  add_executable(planner_bspline_bench benchmarks/bspline_bench.cpp)
  target_link_libraries(planner_bspline_bench PRIVATE
    planner_bspline
    planner_bspline_ego)
  target_compile_definitions(planner_bspline_bench PRIVATE
    "PLANNER_BSPLINE_BENCH_INPUTS=\"${PLANNER_ROOT_DIR}/tests/fixtures/v1/benchmarks/bspline/inputs.json\"")
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_bspline_bench PRIVATE -Wall -Wextra -Wpedantic)
  endif()
elseif(PLANNER_BUILD_BENCHMARKS)
  message(STATUS
    "planner_bspline_bench skipped: PLANNER_BSPLINE_WITH_EGO=OFF and the "
    "benchmark includes the ego_uniform comparison backend.")
endif()

# ---------------------------------------------------------------------------
# 测试（GoogleTest + CTest，标签 "bspline"）。
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_TESTS)
  if(NOT TARGET GTest::gtest_main)
    message(FATAL_ERROR
      "PLANNER_BUILD_TESTS=ON but GTest::gtest_main is not available. "
      "Configure through the root CMakeLists.txt, which fetches googletest "
      "from dependencies.lock.json.")
  endif()
  add_subdirectory(tests)
endif()
