cmake_minimum_required(VERSION 3.22)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Find Python and Boost.Python
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
find_package(Boost COMPONENTS python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR} REQUIRED)

add_executable(rrtBuilderApp src/main.cpp)
target_link_libraries(rrtBuilderApp graphLib)
target_link_libraries(rrtBuilderApp rrtLib)
target_include_directories(rrtBuilderApp PRIVATE ${Boost_INCLUDE_DIRS} ${Python3_INCLUDE_DIRS})
target_link_libraries(rrtBuilderApp ${Boost_LIBRARIES} ${Python3_LIBRARIES})