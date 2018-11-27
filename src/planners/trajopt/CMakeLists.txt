cmake_minimum_required(VERSION 3.0)

project(trajopt)
set(TRAJOPT_VERSION 0.1)

set(TARGET_NAME ${PROJECT_NAME})
CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

find_package(Boost COMPONENTS system filesystem program_options REQUIRED)

find_package(Eigen3 REQUIRED)


INCLUDE(FindPkgConfig)
pkg_check_modules(YAML REQUIRED yaml-cpp)

set (TRAJOPT_PATH ${CMAKE_CURRENT_LIST_DIR})

add_subdirectory(trajopt_utils)
add_subdirectory(trajopt_sco)
add_subdirectory(trajopt_ext)

add_subdirectory(trajopt)

get_directory_property(TRAJOPT_UTILS_DIR_PATH DIRECTORY trajopt_utils DEFINITION TRAJOPT_UTILS_DIR_PATH)
get_directory_property(TRAJOPT_SCO_DIR_PATH DIRECTORY trajopt_sco DEFINITION TRAJOPT_SCO_DIR_PATH)
get_directory_property(JSON_DIR_PATH DIRECTORY trajopt_ext DEFINITION JSON_DIR_PATH)
get_directory_property(TRAJOPT_PLANNER_DIR_PATH DIRECTORY trajopt DEFINITION TRAJOPT_PLANNER_DIR_PATH)

get_directory_property(TRAJOPT_INCLUDE_HEADER DIRECTORY trajopt DEFINITION TRAJOPT_INCLUDE_HEADER)

include_directories(
${EIGEN3_INCLUDE_DIR}
${TRAJOPT_INCLUDE_HEADER}
${BULLET_INCLUDE_DIRS}
${YAML_INCLUDE_DIRS}
)

add_library(trajopt SHARED
        ${TRAJOPT_PLANNER_DIR_PATH}/src/trajectory_costs.cpp
        ${TRAJOPT_PLANNER_DIR_PATH}/src/kinematic_terms.cpp
        ${TRAJOPT_PLANNER_DIR_PATH}/src/collision_terms.cpp
        ${TRAJOPT_PLANNER_DIR_PATH}/src/json_marshal.cpp
        ${TRAJOPT_PLANNER_DIR_PATH}/src/problem_description.cpp
        ${TRAJOPT_PLANNER_DIR_PATH}/src/utils.cpp
)
target_link_libraries(trajopt ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${YAML_LIBRARIES} trajopt_sco trajopt_utils json)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)


