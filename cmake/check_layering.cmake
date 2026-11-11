# Layering check for the planner source tree.
#
# Usage:
#   cmake -DPLANNER_SOURCE_DIR=/path/to/repo -P cmake/check_layering.cmake
#
# The check scans first-party headers and sources for forbidden project
# includes. It intentionally works on raw text and reports every offending
# file before failing.

if(NOT DEFINED PLANNER_SOURCE_DIR)
  message(FATAL_ERROR "PLANNER_SOURCE_DIR must be defined")
endif()

set(_layers math common collision rs hybrid_astar config)

# Collect violations as "layer|pattern|file|include" entries so that a single
# run reports every problem instead of stopping at the first one.
set(_violations "")

function(_planner_record_violations layer files patterns)
  foreach(file IN LISTS files)
    file(READ "${file}" _contents)
    foreach(pattern IN LISTS patterns)
      string(REGEX MATCHALL
        "#[ \t]*include[ \t]*[<\"][^>\"]*${pattern}[^>\"]*[>\"]"
        _matches "${_contents}")
      foreach(match IN LISTS _matches)
        list(APPEND _violations "${layer}|${pattern}|${file}|${match}")
      endforeach()
    endforeach()
  endforeach()
  set(_violations "${_violations}" PARENT_SCOPE)
endfunction()

foreach(layer IN LISTS _layers)
  file(GLOB_RECURSE _files
    "${PLANNER_SOURCE_DIR}/Src/${layer}/*.hpp"
    "${PLANNER_SOURCE_DIR}/Src/${layer}/*.cpp")

  # Tests are intentionally excluded: integration tests may exercise several
  # layers at once (for example the end-to-end config loader test).
  list(FILTER _files EXCLUDE REGEX "/tests/")

  if(layer STREQUAL "math")
    set(_forbidden
      "planner/common"
      "types\\.hpp"
      "toml\\+\\+"
      "planner/config")
  elseif(layer STREQUAL "common")
    set(_forbidden
      "planner/collision"
      "planner/hybrid_astar"
      "planner/rs"
      "planner/config"
      "planner/math/detail/"
      "toml\\+\\+")
  elseif(layer STREQUAL "config")
    # config may include common and toml++ only; project includes from any
    # other layer are rejected below.
    set(_forbidden
      "planner/math"
      "planner/collision"
      "planner/hybrid_astar"
      "planner/rs"
      "planner/math/detail/")
  else()
    # Math implementation details are private to the math layer: other modules
    # may rely on public planner/math/*.hpp headers only.
    set(_forbidden
      "planner/math/detail/"
      "toml\\+\\+"
      "planner/config")
  endif()

  _planner_record_violations("${layer}" "${_files}" "${_forbidden}")
endforeach()

# Enforce that config's project includes are limited to planner/common and
# planner/config (its own headers).
file(GLOB_RECURSE _config_files
  "${PLANNER_SOURCE_DIR}/Src/config/*.hpp"
  "${PLANNER_SOURCE_DIR}/Src/config/*.cpp")
list(FILTER _config_files EXCLUDE REGEX "/tests/")
foreach(file IN LISTS _config_files)
  file(READ "${file}" _contents)
  string(REGEX MATCHALL "#[ \t]*include[ \t]*[<\"]planner/[^>\"]*[>\"]"
    _project_includes "${_contents}")
  foreach(match IN LISTS _project_includes)
    if(NOT match MATCHES "planner/(common|config)/")
      list(APPEND _violations "config|allowed-project-include|${file}|${match}")
    endif()
  endforeach()
endforeach()

if(_violations)
  set(_report "")
  foreach(violation IN LISTS _violations)
    string(REPLACE "|" "  " _line "${violation}")
    string(APPEND _report "  ${_line}\n")
  endforeach()
  message(FATAL_ERROR "Layering check failed:\n${_report}")
endif()

message(STATUS "Layering check passed")
