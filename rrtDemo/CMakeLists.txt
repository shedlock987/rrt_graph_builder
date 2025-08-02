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

# Add the shared library for Python module
add_library(rrtDemoModule SHARED src/rrtDemo.cpp)
set_target_properties(rrtDemoModule PROPERTIES OUTPUT_NAME "rrtDemo" PREFIX "" SUFFIX ".so")
target_include_directories(rrtDemoModule PRIVATE ${Boost_INCLUDE_DIRS} ${Python3_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(rrtDemoModule graphLib rrtLib ${Boost_LIBRARIES} ${Python3_LIBRARIES})