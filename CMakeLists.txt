cmake_minimum_required(VERSION 3.22)
include_directories(${CMAKE_SOURCE_DIR}/graphLib/include)
project(rrt_graph_builder)            # this sets the project name
add_subdirectory(graphLib)
add_subdirectory(rrtDemo)
add_subdirectory(test)

