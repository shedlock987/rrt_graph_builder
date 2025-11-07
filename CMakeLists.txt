cmake_minimum_required(VERSION 3.22)
project(rrt_graph_builder)
set(BUILD_SHARED_LIBS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Place prebuilt libs under a platform-specific folder. Use macOS name when building on Apple.
if(APPLE)
  set(PREBUILT_DIR "${CMAKE_SOURCE_DIR}/prebuilt/macOS")
else()
  set(PREBUILT_DIR "${CMAKE_SOURCE_DIR}/prebuilt/${CMAKE_SYSTEM_NAME}")
endif()
file(MAKE_DIRECTORY "${PREBUILT_DIR}")

# Single-config generators (Make, Ninja)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PREBUILT_DIR}")   # .a, .lib (static archives)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PREBUILT_DIR}")   # .so, .dylib (shared libs)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PREBUILT_DIR}")   # executables (optional)

# Multi-config generators (Xcode, Visual Studio) — set per-configuration output as well
if(CMAKE_CONFIGURATION_TYPES)
  foreach(_cfg ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPER "${_cfg}" _ucfg)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${_ucfg} "${PREBUILT_DIR}")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${_ucfg} "${PREBUILT_DIR}")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${_ucfg} "${PREBUILT_DIR}")
  endforeach()
endif()

include_directories(${CMAKE_SOURCE_DIR}/graphLib/include)
add_subdirectory(graphLib)
add_subdirectory(test)

