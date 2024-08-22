cmake_minimum_required(VERSION 3.9)
project(motion_planners
        VERSION 0.2
        DESCRIPTION "A framework for robot motion planning")

find_package(Rock)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

rock_init()
rock_standard_layout()
