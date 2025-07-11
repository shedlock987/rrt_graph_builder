cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Find Python and Boost.Python
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Boost COMPONENTS python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR} REQUIRED)
message(STATUS "Found Boost Python: ${Boost_LIBRARIES}")

add_executable(rrtDemo src/rrtDemo.cpp)
target_include_directories(rrtDemo PRIVATE ${Boost_INCLUDE_DIRS} ${Python3_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(rrtDemo graphLib)
target_link_libraries(rrtDemo rrtLib)
target_link_libraries(rrtDemo ${Boost_LIBRARIES} ${Python3_LIBRARIES})