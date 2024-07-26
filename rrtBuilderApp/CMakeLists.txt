cmake_minimum_required(VERSION 3.30)

add_executable(rrtBuilderApp src/main.cpp)
target_link_libraries(rrtBuilderApp linkedListLib)