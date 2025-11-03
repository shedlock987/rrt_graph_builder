cmake_minimum_required(VERSION 3.10)
project(rrtDemo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

find_package(PythonLibs REQUIRED)
find_package(Boost REQUIRED COMPONENTS python)

include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_SOURCE_DIR}/graphLib/include)

# Shared Python module
add_library(rrtDemo SHARED src/rrtDemo.cpp)
target_link_libraries(rrtDemo
    rrtLib
    graphLib
    ${Boost_PYTHON_LIBRARY}
    ${PYTHON_LIBRARIES}
)
set_target_properties(rrtDemo PROPERTIES PREFIX "" SUFFIX ".so")

# Executable target: build from a separate main.cpp that provides int main()
#add_executable(rrtDemoMain src/main.cpp)
## Ensure executable links the shared module (rrtDemo) which contains VisRRT implementation
#target_link_libraries(rrtDemoMain
#    rrtDemo
#    rrtLib
#    graphLib
#    ${Boost_PYTHON_LIBRARY}
#    ${PYTHON_LIBRARIES}
#)
#add_dependencies(rrtDemoMain rrtDemo)
