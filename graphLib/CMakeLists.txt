cmake_minimum_required(VERSION 3.22)

add_library(graphLib src/graph.cpp)
message("graphLib current source dir = ${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories( graphLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)