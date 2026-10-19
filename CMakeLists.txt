# NewStructPlanner root build (E0).
#
# Scope: C++17, ROS-free core. Every component is independently enableable;
# `common` (public types + JSON codecs) and `grid` (generic grid math) build by
# default, and `scene` builds whenever the map component is enabled (it
# hard-depends on map). All other optional components stay OFF by default, so
# a configure works even before their directories exist.
#
# Optional target switches (X-A1-01..16): examples and benchmarks have global
# switches (PLANNER_BUILD_EXAMPLES / PLANNER_BUILD_BENCHMARKS, default ON to
# preserve the previous behavior); reference backends and HTTP services have
# per-component switches (PLANNER_RS_WITH_OMPL, PLANNER_BSPLINE_WITH_EGO,
# PLANNER_POLYNOMIAL_WITH_MAV_COMPARE, PLANNER_DFTPAV_WITH_E2E,
# PLANNER_MAP_WITH_SERVICE, PLANNER_SERVICE_WITH_HTTP). Disabling a switch
# must not download or link the corresponding dependency.
#
# The browser workbench (`Src/web`) is an npm component: it has no
# CMakeLists.txt and is built/served through tools/components.json
# (`task_runner.py build --component web`); there is deliberately no
# PLANNER_BUILD_WEB option (X-A1-15).
#
# Dependencies are locked in dependencies.lock.json and fetched with
# FetchContent pinned by URL + URL_HASH (SHA256). Floating branches are never
# followed. The lock file also stores the exact tag/commit and license path
# for every dependency; see that file for the format description.
#
# Offline / restricted network:
#   * Pre-populate the source trees and point CMake at them (hash checks are
#     replaced by trusting the operator-provided tree):
#       cmake -S . -B build/release \
#         -DFETCHCONTENT_SOURCE_DIR_NLOHMANN_JSON=/path/to/json-3.11.3 \
#         -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/path/to/googletest-1.15.2
#   * Or, after one successful configure, work fully disconnected using the
#     already populated _deps directory:
#       cmake -S . -B build/release -DFETCHCONTENT_FULLY_DISCONNECTED=ON
#
# Do not relax URL_HASH verification to make a build pass; instead record the
# problem and fix the lock file through the integration owner.

cmake_minimum_required(VERSION 3.19)

project(NewStructPlanner VERSION 0.1.0 LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# The first documented profile is release; build/<profile>/ is the expected
# directory layout used by tools/task_runner.py.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type (release profile is the default)" FORCE)
endif()

set(PLANNER_ROOT_DIR "${CMAKE_CURRENT_SOURCE_DIR}")

# ---------------------------------------------------------------------------
# Options. Optional components and the experimental Dftpav backend are OFF,
# except common/grid (always useful foundations) and scene (enabled by default
# but only added when the map component is enabled; see below). Enabling a
# component whose directory does not exist yet prints a warning and skips it
# (see planner_add_optional_component) instead of failing configure.
# ---------------------------------------------------------------------------
option(PLANNER_BUILD_COMMON "Build planner_common (public types and JSON codecs)" ON)
option(PLANNER_BUILD_GRID "Build planner_grid (generic grid math and validation)" ON)
option(PLANNER_BUILD_SCENE "Build planner_scene (map window + obstacles composition; requires PLANNER_BUILD_MAP=ON)" ON)

if(DEFINED BUILD_TESTING)
  set(PLANNER_BUILD_TESTS_DEFAULT "${BUILD_TESTING}")
else()
  set(PLANNER_BUILD_TESTS_DEFAULT ON)
endif()
option(PLANNER_BUILD_TESTS "Build tests and register them with CTest" ${PLANNER_BUILD_TESTS_DEFAULT})

# Global example/benchmark switches (X-A1-16). Default ON so the existing
# component entries and local workflows keep building the same targets.
option(PLANNER_BUILD_EXAMPLES "Build component examples" ON)
option(PLANNER_BUILD_BENCHMARKS "Build component benchmarks" ON)

option(PLANNER_BUILD_RS "Build the Reeds-Shepp component (OMPL 1.6.0 reference)" OFF)
option(PLANNER_BUILD_COLLISION "Build the collision checking component" OFF)
option(PLANNER_BUILD_HYBRID_ASTAR "Build the Hybrid A* component" OFF)
option(PLANNER_BUILD_CORRIDOR "Build the safety corridor component" OFF)
option(PLANNER_BUILD_VALIDATION "Build the trajectory validation component" OFF)
option(PLANNER_BUILD_BSPLINE "Build the B-spline component" OFF)
option(PLANNER_BUILD_POLYNOMIAL "Build the polynomial component" OFF)
option(PLANNER_BUILD_MAP "Build the Lanelet2 map component" OFF)
option(PLANNER_BUILD_DFTPAV "Build the isolated experimental Dftpav backend" OFF)
option(PLANNER_BUILD_SERVICE "Build the W5 planning service (integration front end)" OFF)
option(PLANNER_BUILD_GRAPH_SEARCH "Build the graph search component (X-S1/X-S2 multi-implementation adapters)" OFF)
option(PLANNER_BUILD_GRID_SEARCH "Build the grid search component (X-S3/X-S2 grid A*/Dijkstra + distance fields)" OFF)
option(PLANNER_BUILD_SAMPLING_SEARCH "Build the sampling search component (X-S4 2D RRT* adapters)" OFF)
# No PLANNER_BUILD_WEB: `Src/web` is an npm component without CMakeLists.txt
# (see the header comment); it is registered in tools/components.json instead.

# Test registration: standard CTest entry point.
if(PLANNER_BUILD_TESTS)
  enable_testing()
endif()

# ---------------------------------------------------------------------------
# Dependency lock: read dependencies.lock.json with string(JSON) (CMake >=3.19).
# ---------------------------------------------------------------------------
include(FetchContent)

# Always build the locked source tree; never silently substitute a system
# package for a locked dependency.
set(FETCHCONTENT_TRY_FIND_PACKAGE_MODE NEVER)

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
  set(PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
else()
  set(PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS "")
endif()

set(PLANNER_DEPENDENCIES_LOCK "${PLANNER_ROOT_DIR}/dependencies.lock.json")
if(NOT EXISTS "${PLANNER_DEPENDENCIES_LOCK}")
  message(FATAL_ERROR "dependencies.lock.json is missing at ${PLANNER_DEPENDENCIES_LOCK}")
endif()
file(READ "${PLANNER_DEPENDENCIES_LOCK}" PLANNER_LOCK_JSON)

# The lock file is self-describing; only format_version 1 is understood.
string(JSON PLANNER_LOCK_FORMAT_VERSION ERROR_VARIABLE _lock_format_error GET "${PLANNER_LOCK_JSON}" format_version)
if(_lock_format_error)
  message(FATAL_ERROR "dependencies.lock.json: cannot read format_version: ${_lock_format_error}")
endif()
if(NOT PLANNER_LOCK_FORMAT_VERSION EQUAL 1)
  message(FATAL_ERROR
    "dependencies.lock.json: unsupported format_version '${PLANNER_LOCK_FORMAT_VERSION}' (expected 1)")
endif()

# planner_lock_get(<dependency-key> <out-var> <json-path...>)
function(planner_lock_get DEPENDENCY OUTVAR)
  string(JSON _value ERROR_VARIABLE _error GET "${PLANNER_LOCK_JSON}" dependencies "${DEPENDENCY}" ${ARGN})
  if(_error)
    message(FATAL_ERROR "dependencies.lock.json: cannot read dependencies.${DEPENDENCY}.${ARGN}: ${_error}")
  endif()
  set(${OUTVAR} "${_value}" PARENT_SCOPE)
endfunction()

# Same as planner_lock_get but returns an empty string for a JSON null value
# (a null sha256 means "pending: network was unavailable when locking").
function(planner_lock_get_nullable DEPENDENCY OUTVAR)
  string(JSON _type ERROR_VARIABLE _error TYPE "${PLANNER_LOCK_JSON}" dependencies "${DEPENDENCY}" ${ARGN})
  if(_error)
    message(FATAL_ERROR "dependencies.lock.json: cannot read dependencies.${DEPENDENCY}.${ARGN}: ${_error}")
  endif()
  if(_type STREQUAL "NULL")
    set(${OUTVAR} "" PARENT_SCOPE)
  else()
    string(JSON _value GET "${PLANNER_LOCK_JSON}" dependencies "${DEPENDENCY}" ${ARGN})
    set(${OUTVAR} "${_value}" PARENT_SCOPE)
  endif()
endfunction()

# nlohmann/json is required by planner_common serialization.
if(PLANNER_BUILD_COMMON)
  planner_lock_get(nlohmann_json _nlohmann_tag tag)
  planner_lock_get(nlohmann_json _nlohmann_commit commit)
  planner_lock_get(nlohmann_json _nlohmann_url source url)
  planner_lock_get_nullable(nlohmann_json _nlohmann_sha256 source sha256)

  message(STATUS "Dependency nlohmann/json: ${_nlohmann_tag} (${_nlohmann_commit})")
  if(_nlohmann_sha256)
    set(_nlohmann_hash_args URL_HASH "SHA256=${_nlohmann_sha256}")
  else()
    set(_nlohmann_hash_args "")
    message(WARNING "dependencies.lock.json has no sha256 for nlohmann_json (status pending); the download will not be hash-verified")
  endif()

  FetchContent_Declare(nlohmann_json
    URL "${_nlohmann_url}"
    ${_nlohmann_hash_args}
    ${PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS})

  set(JSON_BuildTests OFF CACHE INTERNAL "Locked dependency: upstream tests are not built")
  set(JSON_Install OFF CACHE INTERNAL "Locked dependency: not installed")
  FetchContent_MakeAvailable(nlohmann_json)

  if(NOT TARGET nlohmann_json::nlohmann_json)
    message(FATAL_ERROR
      "nlohmann_json was populated from '${_nlohmann_url}' but did not define "
      "nlohmann_json::nlohmann_json; the locked source package is broken.")
  endif()
endif()

# GoogleTest is required by every component test. It is fetched once here
# (when tests are enabled) so that component CMakeLists can rely on the
# GTest::gtest_main target; future components only need PLANNER_BUILD_TESTS.
if(PLANNER_BUILD_TESTS)
  planner_lock_get(googletest _gtest_tag tag)
  planner_lock_get(googletest _gtest_commit commit)
  planner_lock_get(googletest _gtest_url source url)
  planner_lock_get_nullable(googletest _gtest_sha256 source sha256)

  message(STATUS "Dependency googletest: ${_gtest_tag} (${_gtest_commit})")
  if(_gtest_sha256)
    set(_gtest_hash_args URL_HASH "SHA256=${_gtest_sha256}")
  else()
    set(_gtest_hash_args "")
    message(WARNING "dependencies.lock.json has no sha256 for googletest (status pending); the download will not be hash-verified")
  endif()

  FetchContent_Declare(googletest
    URL "${_gtest_url}"
    ${_gtest_hash_args}
    ${PLANNER_DOWNLOAD_EXTRACT_TIMESTAMP_ARGS})

  set(BUILD_GMOCK OFF CACHE BOOL "Locked dependency: gmock is not needed" FORCE)
  set(INSTALL_GTEST OFF CACHE BOOL "Locked dependency: not installed" FORCE)
  set(gtest_force_shared_crt ON CACHE BOOL "Locked dependency: default Windows CRT handling" FORCE)
  FetchContent_MakeAvailable(googletest)

  if(NOT TARGET GTest::gtest_main)
    message(FATAL_ERROR
      "googletest was populated from '${_gtest_url}' but did not define "
      "GTest::gtest_main; the locked release asset is broken.")
  endif()
endif()

# OMPL 1.6.0 is locked in dependencies.lock.json for the RS component. It is
# deliberately not fetched here: RS is OFF at E0 and its integration method
# (FetchContent source dir layout, ROS-free build flags) is decided by the RS
# owner in the RS phase. Read it with planner_lock_get(ompl ...) then.
if(PLANNER_BUILD_RS AND NOT EXISTS "${PLANNER_ROOT_DIR}/Src/rs/CMakeLists.txt")
  message(WARNING "PLANNER_BUILD_RS=ON but Src/rs/CMakeLists.txt does not exist yet; OMPL is not fetched at E0")
endif()

# ---------------------------------------------------------------------------
# Components.
# ---------------------------------------------------------------------------
if(PLANNER_BUILD_COMMON)
  if(EXISTS "${PLANNER_ROOT_DIR}/Src/common/CMakeLists.txt")
    add_subdirectory("${PLANNER_ROOT_DIR}/Src/common" "${CMAKE_BINARY_DIR}/Src/common")
  else()
    message(FATAL_ERROR "PLANNER_BUILD_COMMON=ON but Src/common/CMakeLists.txt is missing")
  endif()
endif()

# Adds an optional component when its option is ON and its `planner::<id>`
# target does not exist yet (a dependency fallback inside another component
# may have added the same directory already). A missing directory is a
# warning, not a configure error: optional work packages may not exist yet.
function(planner_add_optional_component OPTION_NAME COMPONENT_DIR)
  if(NOT ${OPTION_NAME})
    return()
  endif()
  get_filename_component(_component_id "${COMPONENT_DIR}" NAME)
  if(TARGET "planner::${_component_id}")
    message(STATUS "Component already added as a dependency: ${COMPONENT_DIR}")
    return()
  endif()
  set(_component_path "${PLANNER_ROOT_DIR}/${COMPONENT_DIR}")
  if(EXISTS "${_component_path}/CMakeLists.txt")
    message(STATUS "Enabling optional component: ${COMPONENT_DIR}")
    add_subdirectory("${_component_path}" "${CMAKE_BINARY_DIR}/${COMPONENT_DIR}")
  else()
    message(WARNING "Optional component '${COMPONENT_DIR}' is enabled by ${OPTION_NAME}=ON but '${_component_path}/CMakeLists.txt' does not exist yet; skipping it without failing configuration.")
  endif()
endfunction()

# grid is a foundation for map/scene and builds whenever common is available
# (default ON; skipped, not failed, when common is disabled).
if(PLANNER_BUILD_GRID AND (PLANNER_BUILD_COMMON OR TARGET planner::common))
  planner_add_optional_component(PLANNER_BUILD_GRID Src/grid)
elseif(PLANNER_BUILD_GRID)
  message(STATUS
    "planner_grid is enabled by default but planner_common is OFF; skipping "
    "planner_grid (it depends on planner::common).")
endif()

planner_add_optional_component(PLANNER_BUILD_RS Src/rs)
planner_add_optional_component(PLANNER_BUILD_COLLISION Src/collision)
planner_add_optional_component(PLANNER_BUILD_HYBRID_ASTAR Src/hybrid_astar)
planner_add_optional_component(PLANNER_BUILD_CORRIDOR Src/corridor)
planner_add_optional_component(PLANNER_BUILD_VALIDATION Src/validation)
planner_add_optional_component(PLANNER_BUILD_BSPLINE Src/bspline)
planner_add_optional_component(PLANNER_BUILD_POLYNOMIAL Src/polynomial)
planner_add_optional_component(PLANNER_BUILD_MAP Src/map)
planner_add_optional_component(PLANNER_BUILD_DFTPAV Src/dftpav)
planner_add_optional_component(PLANNER_BUILD_SERVICE Src/service)

# Wave 3 search components (X-S1..X-S4): optional, OFF by default, each with its
# own per-adapter switches (default ON inside the component; disabling a switch
# must not download or link the corresponding upstream). The lock entries for
# igraph/Klamp't (FetchContent) and the vendored/extracted sources are consumed
# by the components through dependencies.lock.json.
planner_add_optional_component(PLANNER_BUILD_GRAPH_SEARCH Src/graph_search)
planner_add_optional_component(PLANNER_BUILD_GRID_SEARCH Src/grid_search)
planner_add_optional_component(PLANNER_BUILD_SAMPLING_SEARCH Src/sampling_search)

# scene composes map windows, the driveable grid and user obstacles; it is
# enabled by default but hard-depends on the map component. It is added after
# the other components so that a map pulled in by any dependency fallback
# (e.g. the service component) is already visible. Without map it is skipped
# with a clear message instead of failing the default configure.
if(PLANNER_BUILD_SCENE AND (PLANNER_BUILD_MAP OR TARGET planner::map))
  planner_add_optional_component(PLANNER_BUILD_SCENE Src/scene)
elseif(PLANNER_BUILD_SCENE)
  message(STATUS
    "planner_scene is enabled by default but planner_map is OFF; skipping "
    "planner_scene (enable PLANNER_BUILD_MAP, as the scene component entry "
    "in tools/components.json does).")
endif()

# X-A1-09: the W2 map service will reuse planner::scene for grid/scene preview
# endpoints (X-M1). Wiring the link here keeps the relation explicit and makes
# collision transitively available without depending on Hybrid A* or Dftpav.
if(TARGET planner_map_service AND TARGET planner::scene)
  target_link_libraries(planner_map_service PRIVATE planner::scene)
endif()

message(STATUS "---------------- NewStructPlanner configuration ----------------")
message(STATUS "  build type          : ${CMAKE_BUILD_TYPE}")
message(STATUS "  planner_common      : ${PLANNER_BUILD_COMMON}")
message(STATUS "  planner_grid        : ${PLANNER_BUILD_GRID}")
message(STATUS "  planner_scene       : ${PLANNER_BUILD_SCENE}")
message(STATUS "  planner_service     : ${PLANNER_BUILD_SERVICE}")
message(STATUS "  planner_graph_search: ${PLANNER_BUILD_GRAPH_SEARCH}")
message(STATUS "  planner_grid_search : ${PLANNER_BUILD_GRID_SEARCH}")
message(STATUS "  planner_sampling_search: ${PLANNER_BUILD_SAMPLING_SEARCH}")
message(STATUS "  examples            : ${PLANNER_BUILD_EXAMPLES}")
message(STATUS "  benchmarks          : ${PLANNER_BUILD_BENCHMARKS}")
message(STATUS "  tests + CTest       : ${PLANNER_BUILD_TESTS}")
message(STATUS "  dependency lock     : ${PLANNER_DEPENDENCIES_LOCK}")
message(STATUS "  C++ standard        : ${CMAKE_CXX_STANDARD}")
message(STATUS "  web (npm)           : not a CMake target; use tools/components.json")
message(STATUS "---------------------------------------------------------------")
