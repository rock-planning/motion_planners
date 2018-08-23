cmake_minimum_required(VERSION 3.0)

project(trajopt)
set(TRAJOPT_VERSION 0.1)

set(TARGET_NAME ${PROJECT_NAME})
CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

#find_package(Eigen REQUIRED)
find_package(Boost COMPONENTS system filesystem program_options REQUIRED)

include("${CMAKE_CURRENT_LIST_DIR}/cmake/FindEigen2.cmake")

#find_package(Eigen3 REQUIRED)


INCLUDE(FindPkgConfig)
pkg_check_modules(YAML REQUIRED yaml-cpp)

set (TRAJOPT_PATH ${CMAKE_CURRENT_LIST_DIR})
set(BULLET_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/ext/bullet/src)
set(BULLET_LIBRARIES BulletCollision LinearMath)

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
${TRAJOPT_INCLUDE_HEADER}
 ${BULLET_INCLUDE_DIRS}
 ${YAML_INCLUDE_DIRS}
)

add_library(trajopt SHARED
        # traj_plotter.cpp
        ${TRAJOPT_PLANNER_DIR_PATH}/src/trajectory_costs.cpp
        ${TRAJOPT_PLANNER_DIR_PATH}/src/kinematic_terms.cpp
        ${TRAJOPT_PLANNER_DIR_PATH}/src/collision_terms.cpp
        ${TRAJOPT_PLANNER_DIR_PATH}/src/json_marshal.cpp
        ${TRAJOPT_PLANNER_DIR_PATH}/src/problem_description.cpp
#        ${TRAJOPT_PLANNER_DIR_PATH}/src/bullet_collision_checker.cpp
        # configuration_space.cpp
        ${TRAJOPT_PLANNER_DIR_PATH}/src/utils.cpp
        # rave_utils.cpp
        ${TRAJOPT_PLANNER_DIR_PATH}/src/collision_checker.cpp
        # plot_callback.cpp
        # bullet_unity.cpp

        ${TRAJOPT_PLANNER_DIR_PATH}/src/TrajoptConfig.cpp
)
 target_link_libraries(trajopt ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${YAML_LIBRARIES} sco utils)

 install(DIRECTORY include/ DESTINATION include/trajectory_optimization/planner)
 install(TARGETS ${PROJECT_NAME} DESTINATION lib)


