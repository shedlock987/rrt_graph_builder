cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

add_executable(rrtBuilderApp src/main.cpp)
target_link_libraries(rrtBuilderApp graphLib)
target_link_libraries(rrtBuilderApp rrtLib)