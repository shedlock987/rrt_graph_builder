cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#include(GoogleTest)
#include(FetchContent)
#FetchContent_Declare(
#  googletest
#  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
#)

add_executable(graphTest src/graph_test.cpp)
target_link_libraries(graphTest graphLib)
#GTest::gtest_main



#gtest_discover_tests(graphTest)