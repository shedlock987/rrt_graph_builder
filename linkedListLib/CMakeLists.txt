cmake_minimum_required(VERSION 3.30)

add_library(linkedListLib src/linked_list.cpp)
message("linkedListLib current source dir = ${CMAKE_CURRENT_SOURCE_DIR}")
target_include_directories( linkedListLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)