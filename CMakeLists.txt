cmake_minimum_required(VERSION 3.22)
project(rrt_graph_builder)
set(BUILD_SHARED_LIBS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Platform-specific prebuilt dirs
if(APPLE)
  set(PREBUILT_DIR "${CMAKE_SOURCE_DIR}/prebuilt/macOS")
  set(PREBUILT_LINUX_DIR "${CMAKE_SOURCE_DIR}/prebuilt/linux")
elseif(UNIX)
  # assume linux-like
  set(PREBUILT_DIR "${CMAKE_SOURCE_DIR}/prebuilt/linux")
else()
  set(PREBUILT_DIR "${CMAKE_SOURCE_DIR}/prebuilt/${CMAKE_SYSTEM_NAME}")
endif()
file(MAKE_DIRECTORY "${PREBUILT_DIR}")
if(DEFINED PREBUILT_LINUX_DIR)
  file(MAKE_DIRECTORY "${PREBUILT_LINUX_DIR}")
endif()

# Route outputs to the platform-specific prebuilt directory
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${PREBUILT_DIR}")   # .a, .lib
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${PREBUILT_DIR}")   # .so, .dylib
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${PREBUILT_DIR}")   # executables (optional)

# Multi-config generators (Xcode, Visual Studio) — set per-configuration output as well
if(CMAKE_CONFIGURATION_TYPES)
  foreach(_cfg ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER "${_cfg}" _ucfg)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${_ucfg} "${PREBUILT_DIR}")
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${_ucfg} "${PREBUILT_DIR}")
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${_ucfg} "${PREBUILT_DIR}")
  endforeach()
endif()

include_directories(${CMAKE_SOURCE_DIR}/graphLib/include)
add_subdirectory(graphLib)
add_subdirectory(test)

# On macOS also create .so-named copies in prebuilt/linux (useful to ship both names).
# Note: this only renames/copies the artifact; it does not change binary format.
if(APPLE)
  if(TARGET libgraphLib)
    add_custom_command(TARGET libgraphLib POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E make_directory "${PREBUILT_LINUX_DIR}"
      COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:libgraphLib> "${PREBUILT_LINUX_DIR}/graphLib.so"
      COMMENT "Copying macOS libgraphLib output to ${PREBUILT_LINUX_DIR}/graphLib.so"
    )
  endif()
  if(TARGET librrtLib)
    add_custom_command(TARGET librrtLib POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E make_directory "${PREBUILT_LINUX_DIR}"
      COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:librrtLib> "${PREBUILT_LINUX_DIR}/rrtLib.so"
      COMMENT "Copying macOS librrtLib output to ${PREBUILT_LINUX_DIR}/rrtLib.so"
    )
  endif()
endif()

