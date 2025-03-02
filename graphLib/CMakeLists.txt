cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_library(graphLib src/graph.cpp)
add_library(rrt src/rrt.cpp)
message("graphLib current source dir = ${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories( graphLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories( rrt PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)