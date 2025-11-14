cmake_minimum_required(VERSION 3.10)
project(graphLib)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON) 

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(graphLib
    src/graph.cpp
)

add_library(rrtLib
    src/rrt.cpp
)

target_link_libraries(rrtLib PRIVATE graphLib)