cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(GTest REQUIRED)

add_executable(graphTest src/graph_test.cpp)
target_link_libraries(graphTest GTest::gtest GTest::gtest_main graphLib rrtLib)