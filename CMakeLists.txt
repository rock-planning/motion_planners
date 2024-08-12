# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
find_package(Rock)

add_subdirectory(src/planners)

rock_init(motion_planners 0.2)
rock_standard_layout()
add_subdirectory(test)
