cmake_minimum_required(VERSION 3.22)

add_executable(rrtBuilderApp src/main.cpp)
target_link_libraries(rrtBuilderApp graphLib)