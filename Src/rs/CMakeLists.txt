# planner_rs：Reeds-Shepp 距离、路径分段与采样（Wave 3）。
#
# 通常由根 CMakeLists.txt 添加（PLANNER_BUILD_RS=ON）。下面的回退只是让该文件
# 更易于独立使用；根项目提供 PLANNER_ROOT_DIR、planner_lock_get() 和
# GTest::gtest_main。
#
# OMPL 1.6.0（锁定于 dependencies.lock.json）通过 FetchContent 的
# URL + URL_HASH 获取，并在构建时禁用其测试/示例/python 绑定。它仅支撑可选的
# `planner_rs_ompl_reference` 静态库：公开的 planner::rs API 从不暴露 OMPL
# 类型，随附的求解器是 src/reeds_shepp.cpp 中的原生解析枚举。由于两侧实现
# 同一曲线族，OMPL 对比是封装一致性测试，而非独立的双重验证。
#
# 离线/受限构建的应急开关：使用以下方式配置
#   -DPLANNER_RS_WITH_OMPL=OFF
# （或像其他锁定依赖那样提供 FETCHCONTENT_SOURCE_DIR_OMPL）；该组件随后只
# 构建原生求解器。

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
if(NOT DEFINED FETCHCONTENT_TRY_FIND_PACKAGE_MODE)
  set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE NEVER)
endif()

option(PLANNER_RS_WITH_OMPL
  "Fetch and build the locked OMPL 1.6.0 as the RS reference adapter" ON)

# ---------------------------------------------------------------------------
# 可选的 OMPL 1.6.0 参考实现（对比/一致性基线）。
# ---------------------------------------------------------------------------
set(PLANNER_RS_OMPL_AVAILABLE OFF)

if(PLANNER_RS_WITH_OMPL)
  if(NOT COMMAND planner_lock_get)
    message(WARNING
      "PLANNER_RS_WITH_OMPL=ON but planner_lock_get() is unavailable (the "
      "component was not added by the root CMakeLists.txt); building the "
      "native RS solver without the OMPL reference.")
  else()
    include(FetchContent)
    planner_lock_get(ompl _rs_ompl_tag tag)
    planner_lock_get(ompl _rs_ompl_commit commit)
    planner_lock_get(ompl _rs_ompl_url source url)
    planner_lock_get_nullable(ompl _rs_ompl_sha256 source sha256)
    message(STATUS "Dependency ompl (RS reference): ${_rs_ompl_tag} (${_rs_ompl_commit})")
    if(_rs_ompl_sha256)
      set(_rs_ompl_hash_args URL_HASH "SHA256=${_rs_ompl_sha256}")
    else()
      set(_rs_ompl_hash_args "")
      message(WARNING
        "dependencies.lock.json has no sha256 for ompl (status pending); the "
        "download will not be hash-verified")
    endif()
    if(DEFINED PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS)
      set(_rs_ompl_timestamp_args ${PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS})
    elseif(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
      set(_rs_ompl_timestamp_args DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
    else()
      set(_rs_ompl_timestamp_args "")
    endif()

    FetchContent_Declare(ompl
      URL "${_rs_ompl_url}"
      ${_rs_ompl_hash_args}
      ${_rs_ompl_timestamp_args})

    # 精简上游构建：不含上游测试、示例、python 绑定，
    # 也不做一次性注册（那会尝试打开网页浏览器）。
    set(OMPL_BUILD_TESTS OFF CACHE BOOL "RS reference: upstream tests disabled" FORCE)
    set(OMPL_BUILD_DEMOS OFF CACHE BOOL "RS reference: upstream demos disabled" FORCE)
    set(OMPL_BUILD_PYBINDINGS OFF CACHE BOOL "RS reference: python bindings disabled" FORCE)
    set(OMPL_BUILD_PYTESTS OFF CACHE BOOL "RS reference: python tests disabled" FORCE)
    set(OMPL_REGISTRATION OFF CACHE BOOL "RS reference: no upstream registration" FORCE)

    FetchContent_MakeAvailable(ompl)

    if(TARGET ompl)
      set(PLANNER_RS_OMPL_AVAILABLE ON)
    else()
      message(WARNING
        "OMPL was populated but did not define the 'ompl' target; the OMPL "
        "reference adapter is skipped and the native solver is used alone.")
    endif()
  endif()
endif()

# ---------------------------------------------------------------------------
# planner_rs：原生解析求解器（随附实现）。
# ---------------------------------------------------------------------------
add_library(planner_rs
  src/reeds_shepp.cpp
  src/path_sampling.cpp)

add_library(planner::rs ALIAS planner_rs)

target_compile_features(planner_rs PUBLIC cxx_std_17)
target_include_directories(planner_rs PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_link_libraries(planner_rs PUBLIC planner::common)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(planner_rs PRIVATE -Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# 可选的 OMPL 参考适配器（私有目标；不属于公开 API）。
# X-A1-01：开关 PLANNER_RS_WITH_OMPL（默认 ON）同时控制 FetchContent 与本
# 目标的创建；OFF 时不下载 OMPL，测试/基准也不链接对照库。
# ---------------------------------------------------------------------------
if(PLANNER_RS_OMPL_AVAILABLE)
  add_library(planner_rs_ompl_reference STATIC src/ompl_reference.cpp)
  target_link_libraries(planner_rs_ompl_reference
    PUBLIC planner_rs
    PRIVATE ompl)
  # 上游目标不会在源码树内导出其包含目录，因此显式添加 OMPL 源码树和生成的
  # config.h 目录。OMPL 的公开头文件还会引入 Eigen/Core。
  #
  # X-A1-02 依赖登记：OMPL 1.6.0 自身的构建经 find_package 消费系统
  # Eigen3（≥3.3），因此适配器优先链接同一个系统 Eigen3::Eigen，保证与
  # OMPL 编译时使用的头文件一致；锁条目 `eigen`（3.4.0）只服务于
  # bspline/polynomial 的对照后端，不在此处混用。仅在系统 Eigen 不可用而
  # 锁定的头文件目标已经存在（bspline/polynomial 已启用）时回退到后者。
  find_package(Eigen3 QUIET)
  target_include_directories(planner_rs_ompl_reference PRIVATE
    "${ompl_BINARY_DIR}/src"
    "${ompl_SOURCE_DIR}/src")
  if(TARGET Eigen3::Eigen)
    target_link_libraries(planner_rs_ompl_reference PRIVATE Eigen3::Eigen)
  elseif(TARGET planner_eigen_headers)
    target_link_libraries(planner_rs_ompl_reference PRIVATE planner_eigen_headers)
  elseif(EIGEN3_INCLUDE_DIRS)
    target_include_directories(planner_rs_ompl_reference PRIVATE ${EIGEN3_INCLUDE_DIRS})
  endif()
  target_compile_definitions(planner_rs_ompl_reference PUBLIC PLANNER_RS_HAS_OMPL=1)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_rs_ompl_reference PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

# ---------------------------------------------------------------------------
# 示例与基准测试（根选项 PLANNER_BUILD_EXAMPLES / PLANNER_BUILD_BENCHMARKS，
# 默认 ON）。
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_EXAMPLES)
  add_executable(planner_rs_demo examples/rs_demo.cpp)
  target_link_libraries(planner_rs_demo PRIVATE planner_rs)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_rs_demo PRIVATE -Wall -Wextra -Wpedantic)
  endif()

  # 将独立示例自检注册到 CTest（标签 "rs"），
  # 使该示例也能通过官方测试入口运行。
  if(PLANNER_BUILD_TESTS)
    add_test(NAME planner_rs_demo_self_check COMMAND planner_rs_demo --self-check)
    set_tests_properties(planner_rs_demo_self_check PROPERTIES LABELS "rs")
  endif()
endif()

# planner_rs_bench --suite v1 --output-dir <dir>；由任务运行器在独占的基准测试
# 窗口中测量（tools/components.json 的 benchmark_suites）。
if(PLANNER_BUILD_BENCHMARKS)
  add_executable(planner_rs_bench benchmarks/rs_bench.cpp)
  target_link_libraries(planner_rs_bench PRIVATE planner_rs nlohmann_json::nlohmann_json)
  if(PLANNER_RS_OMPL_AVAILABLE)
    target_link_libraries(planner_rs_bench PRIVATE planner_rs_ompl_reference)
    target_include_directories(planner_rs_bench PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/src")
  endif()
  # 在 GCC < 10 中 std::filesystem 位于单独的库（本机为 GCC 9.4）。
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 10)
    target_link_libraries(planner_rs_bench PRIVATE stdc++fs)
  endif()
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(planner_rs_bench PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endif()

# ---------------------------------------------------------------------------
# 测试（CTest 标签 "rs"）。
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
