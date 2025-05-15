cmake_minimum_required(VERSION 3.10)
project(test_motion_planners VERSION 0.2)
set(TARGET_NAME ${PROJECT_NAME})

# CMake configuration
set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# External dependencies
INCLUDE(FindPkgConfig)
pkg_check_modules(MOTION_PLANNERS REQUIRED motion_planners)

# Add link directories BEFORE add_executable
link_directories(${MOTION_PLANNERS_LIBRARY_DIRS})

# Create the executable (not a library)
add_executable(${TARGET_NAME} test_motion_planners.cpp)

# Target-based dependency management
target_include_directories(${TARGET_NAME} PRIVATE
    ${MOTION_PLANNERS_INCLUDE_DIRS}
)
target_link_libraries(${TARGET_NAME} PRIVATE
    ${MOTION_PLANNERS_LIBRARIES}
)

message("Library dirs: ${MOTION_PLANNERS_LIBRARY_DIRS}")