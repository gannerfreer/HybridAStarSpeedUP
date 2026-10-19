# planner_polynomial：分段三次/五次多项式轨迹
# （Wave 3，Docs/component_development_plan.md 第 6.2 节“多项式”行）。
#
# 通常由根 CMakeLists.txt 添加（PLANNER_BUILD_POLYNOMIAL=ON）。
# 下面的回退只是让该文件更易于被其他父项目使用；根项目总是提供
# PLANNER_ROOT_DIR，并从 dependencies.lock.json 获取
# nlohmann_json / GoogleTest。

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

# X-A1-05：mav_trajectory_generation 对照后端独立开关（默认 ON，保持既有
# 行为）。OFF 时不下载 Eigen/glog/mtg，也不创建对照库；主库与测试保持完整。
option(PLANNER_POLYNOMIAL_WITH_MAV_COMPARE
  "Build planner_polynomial_mav_compare (Eigen + glog + mav_trajectory_generation)" ON)

# v1 夹具是权威的基准测试输入（第 7.3 节）；绝对路径被固化到基准测试二进制中，
# 因此正式运行不依赖工作目录。
set(PLANNER_POLYNOMIAL_BENCH_FIXTURE
  "${PLANNER_ROOT_DIR}/tests/fixtures/v1/benchmarks/polynomial/inputs.json")
if(NOT EXISTS "${PLANNER_POLYNOMIAL_BENCH_FIXTURE}")
  message(FATAL_ERROR
    "Polynomial benchmark fixture is missing at "
    "${PLANNER_POLYNOMIAL_BENCH_FIXTURE}; it is part of the v1 fixture set.")
endif()

# ---------------------------------------------------------------------------
# 内部对照候选（计划 §3.4）：Eigen 3.4.0 + glog 0.6.0 +
# mav_trajectory_generation 求值核心。只服务于对照测试与基准，
# 不进入 planner_polynomial 的公开 API。
# X-A1-05：由 PLANNER_POLYNOMIAL_WITH_MAV_COMPARE（默认 ON）控制；OFF 时
# 不下载 Eigen/glog/mtg，也不创建对照库与依赖它们的基准。
# ---------------------------------------------------------------------------
if(PLANNER_POLYNOMIAL_WITH_MAV_COMPARE)
  if(NOT COMMAND planner_lock_get)
    message(FATAL_ERROR
      "planner_lock_get() is not defined; the polynomial component must be "
      "added from the root CMakeLists.txt, which reads dependencies.lock.json.")
  endif()

  # Eigen（依赖锁条目 `eigen`）：头文件库，以 FetchContent_Populate +
  # INTERFACE 目标消费（不运行上游 CMake，避免与 OMPL 等同树依赖的全局
  # 目标冲突，也避免误用系统 Eigen3::Eigen）。
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

  # glog（mav_trajectory_generation 的 CHECK/LOG 依赖；关闭 gflags/unwind/测试）。
  planner_lock_get(glog _glog_tag tag)
  planner_lock_get(glog _glog_commit commit)
  planner_lock_get(glog _glog_url source url)
  planner_lock_get_nullable(glog _glog_sha256 source sha256)
  message(STATUS "Dependency glog: ${_glog_tag} (${_glog_commit})")
  if(_glog_sha256)
    set(_glog_hash_args URL_HASH "SHA256=${_glog_sha256}")
  else()
    set(_glog_hash_args "")
    message(WARNING
      "dependencies.lock.json has no sha256 for glog (status pending); "
      "the download will not be hash-verified")
  endif()
  set(WITH_GFLAGS OFF CACHE BOOL
      "planner_polynomial: glog does not use gflags" FORCE)
  set(WITH_UNWIND OFF CACHE BOOL
      "planner_polynomial: glog does not use libunwind" FORCE)
  set(WITH_GTEST OFF CACHE BOOL
      "planner_polynomial: glog upstream tests are not built" FORCE)
  set(BUILD_TESTING OFF CACHE BOOL
      "planner_polynomial: upstream dependency tests are not built" FORCE)
  FetchContent_Declare(glog
    URL "${_glog_url}"
    ${_glog_hash_args}
    ${PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS})
  FetchContent_MakeAvailable(glog)
  if(NOT TARGET glog::glog)
    message(FATAL_ERROR
      "glog was populated from '${_glog_url}' but did not define the "
      "'glog::glog' target; the locked source package is broken.")
  endif()

  # mav_trajectory_generation（上游为 catkin_simple 包、无顶层 CMakeLists；
  # 只取锁定源码，由组件以自定义目标编译求值核心）。
  planner_lock_get(mav_trajectory_generation _mtg_commit commit)
  planner_lock_get(mav_trajectory_generation _mtg_url source url)
  planner_lock_get_nullable(mav_trajectory_generation _mtg_sha256 source sha256)
  message(STATUS "Dependency mav_trajectory_generation: ${_mtg_commit}")
  if(_mtg_sha256)
    set(_mtg_hash_args URL_HASH "SHA256=${_mtg_sha256}")
  else()
    set(_mtg_hash_args "")
    message(WARNING
      "dependencies.lock.json has no sha256 for mav_trajectory_generation "
      "(status pending); the download will not be hash-verified")
  endif()
  FetchContent_Declare(mav_trajectory_generation
    URL "${_mtg_url}"
    ${_mtg_hash_args}
    ${PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS})
  FetchContent_Populate(mav_trajectory_generation)
  set(PLANNER_MTG_ROOT
    "${mav_trajectory_generation_SOURCE_DIR}/mav_trajectory_generation")
  if(NOT EXISTS "${PLANNER_MTG_ROOT}/src/polynomial.cpp")
    message(FATAL_ERROR
      "mav_trajectory_generation source tree is missing "
      "${PLANNER_MTG_ROOT}/src/polynomial.cpp; the locked package is broken.")
  endif()
endif()

add_library(planner_polynomial
  src/polynomial.cpp
  src/trajectory_segment.cpp
  src/piecewise_trajectory.cpp)

add_library(planner::polynomial ALIAS planner_polynomial)

target_compile_features(planner_polynomial PUBLIC cxx_std_17)
target_include_directories(planner_polynomial PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(planner_polynomial PUBLIC planner::common)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_polynomial PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# 对照求值库：planner_polynomial_mav_compare（X-A1-05 独立开关）。
# 包含本组件的适配层与 mav_trajectory_generation 的求值核心
# （polynomial.cpp + Jenkins-Traub 求根）。公开头文件不暴露 Eigen/glog 类型。
# ---------------------------------------------------------------------------
if(PLANNER_POLYNOMIAL_WITH_MAV_COMPARE)
  add_library(planner_polynomial_mav_compare STATIC
    compare/mav_segment_evaluator.cpp
    "${PLANNER_MTG_ROOT}/src/polynomial.cpp"
    "${PLANNER_MTG_ROOT}/src/rpoly/rpoly_ak1.cpp")

  add_library(planner::polynomial::mav_compare ALIAS planner_polynomial_mav_compare)

  target_compile_features(planner_polynomial_mav_compare PUBLIC cxx_std_17)
  target_include_directories(planner_polynomial_mav_compare PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/compare>")
  target_include_directories(planner_polynomial_mav_compare PRIVATE
    "${PLANNER_MTG_ROOT}/include")
  # 上游求值核心只在 PRIVATE 链接中出现；GTest 不需要 glog 头文件。
  target_link_libraries(planner_polynomial_mav_compare
    PRIVATE planner::common planner_eigen_headers glog::glog)
  # 上游源码不参与本组件的告警清洁度门槛（只对自己的适配层开启告警）。
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set_source_files_properties(compare/mav_segment_evaluator.cpp
      PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra;-Wpedantic")
  endif()
endif()

# 独立命令行示例（自检 + CSV 采样）。随组件一起构建
# （PLANNER_BUILD_EXAMPLES 默认 ON）；不注册为 CTest 测试。
if(PLANNER_BUILD_EXAMPLES)
  add_executable(planner_polynomial_example examples/polynomial_demo.cpp)
  target_link_libraries(planner_polynomial_example PRIVATE planner_polynomial)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_polynomial_example PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

# 微基准测试二进制。契约：tools/README.md 第 7.2 节；输出路径约定
# {build_dir}/Src/polynomial/planner_polynomial_bench 与 components.json 条目
# 一致。基准比较 mav 对照求值，因此需要对照库（X-A1-05）；OFF 时不构建。
if(PLANNER_BUILD_BENCHMARKS AND PLANNER_POLYNOMIAL_WITH_MAV_COMPARE)
  add_executable(planner_polynomial_bench src/bench_main.cpp)
  target_link_libraries(planner_polynomial_bench PRIVATE
    planner_polynomial
    planner_polynomial_mav_compare)
  target_compile_definitions(planner_polynomial_bench PRIVATE
    "PLANNER_POLYNOMIAL_BENCH_FIXTURE=\"${PLANNER_POLYNOMIAL_BENCH_FIXTURE}\"")
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_polynomial_bench PRIVATE -Wall -Wextra -Wpedantic)
  endif()
elseif(PLANNER_BUILD_BENCHMARKS)
  message(STATUS
    "planner_polynomial_bench skipped: PLANNER_POLYNOMIAL_WITH_MAV_COMPARE=OFF "
    "and the benchmark compares against the mav evaluation core.")
endif()

if(PLANNER_BUILD_TESTS)
  if(NOT TARGET GTest::gtest_main)
    message(FATAL_ERROR
      "PLANNER_BUILD_TESTS=ON but GTest::gtest_main is not available. "
      "Configure through the root CMakeLists.txt, which fetches googletest "
      "from dependencies.lock.json.")
  endif()
  add_subdirectory(tests)
endif()
