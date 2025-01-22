cmake_minimum_required(VERSION 3.10)
project(motion_planners VERSION 0.2)

set(TARGET_NAME ${PROJECT_NAME})

# CMake configuration
set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Boost configuration
find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)

# External dependencies
find_package(Eigen3 REQUIRED)
INCLUDE(FindPkgConfig)
pkg_check_modules(BASE_TYPES REQUIRED base_types)
pkg_check_modules(BASE_LOGGING REQUIRED base_logging)
pkg_check_modules(OROCOS_KDL REQUIRED orocos_kdl)
pkg_check_modules(KDL_PARSER REQUIRED kdl_parser)
pkg_check_modules(URDF REQUIRED urdfdom)
pkg_check_modules(URDFHEADER REQUIRED urdfdom_headers)
pkg_check_modules(SRDF REQUIRED srdfdom)
pkg_check_modules(OCTOMAP REQUIRED octomap)
pkg_check_modules(KINEMATICS REQUIRED kinematics_library)
pkg_check_modules(COLLISION_DETECTION REQUIRED collision_detection)
pkg_check_modules(ROBOT_MODEL REQUIRED robot_model)

# Include subdirectory for planners
add_subdirectory(src/planners)

get_directory_property(PLANNERS_PATH DIRECTORY src/planners DEFINITION PLANNERS_DIR_PATH)
get_directory_property(STOMP_PATH DIRECTORY src/planners DEFINITION STOMP_PATH)
get_directory_property(TRAJOPT_INCLUDE_HEADER DIRECTORY src/planners DEFINITION TRAJOPT_INCLUDE_HEADER)

#If OMPL is installed through ROS, we need to specify the ompl install folder for linking.
get_directory_property(WRAPPERS_LINK_DIR DIRECTORY src/planners DEFINITION WRAPPERS_LINK_DIR)

# Collect include directories and libraries
set(MOTIONPLANNER_LIB_INCLUDES
    ${CMAKE_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
    ${BASE_TYPES_INCLUDE_DIRS}
    ${BASE_LOGGING_INCLUDE_DIRS}
    ${OROCOS_KDL_INCLUDE_DIRS}
    ${KDL_PARSER_INCLUDE_DIRS}
    ${URDF_INCLUDE_DIRS}
    ${URDFHEADER_INCLUDE_DIRS}
    ${SRDF_INCLUDE_DIRS}
    ${OCTOMAP_INCLUDE_DIRS}
    ${KINEMATICS_INCLUDE_DIRS}
    ${COLLISION_DETECTION_INCLUDE_DIRS}
    ${ROBOT_MODEL_INCLUDE_DIRS}
    ${PLANNERS_PATH}/include
    ${STOMP_PATH}/include
    ${TRAJOPT_INCLUDE_HEADER}
)

set(MOTIONPLANNER_DEP_LIBS
    Boost::system
    Boost::filesystem
    Boost::program_options
    ${BASE_TYPES_LIBRARIES}
    ${BASE_LOGGING_LIBRARIES}
    ${OROCOS_KDL_LIBRARIES}
    ${KDL_PARSER_LIBRARIES}
    ${URDF_LIBRARIES}
    ${URDFHEADER_LIBRARIES}
    ${SRDF_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    ${KINEMATICS_LIBRARIES}
    ${COLLISION_DETECTION_LIBRARIES}
    ${ROBOT_MODEL_LIBRARIES}
)

# Create the main library
add_library(${TARGET_NAME} SHARED src/MotionPlanners.cpp)

# Target-based dependency management
target_include_directories(${TARGET_NAME} PUBLIC ${MOTIONPLANNER_LIB_INCLUDES})
target_link_libraries(${TARGET_NAME} PUBLIC ${MOTIONPLANNER_DEP_LIBS} planner_factory)

# Generate pkg-config file
set(PKGCONFIG_EXTRA_CFLAGS "-I${PLANNERS_PATH}/include -I${STOMP_PATH}/include")
foreach(include ${TRAJOPT_INCLUDE_HEADER})
    string(REPLACE " " "\\ " include "${include}")
    set(PKGCONFIG_EXTRA_CFLAGS "${PKGCONFIG_EXTRA_CFLAGS} -I${include}")
endforeach()
list(APPEND PKGCONFIG_REQUIRES base_types base_logging orocos_kdl kdl_parser urdfdom urdfdom_headers srdfdom octomap kinematics_library collision_detection robot_model)
list(JOIN PKGCONFIG_REQUIRES " " PKGCONFIG_REQUIRES_STR)

configure_file("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

# Install rules
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME} DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)

# Testing configuration
option(ENABLE_TESTS "Build tests" ON)

if(ENABLE_TESTS)
    enable_testing()

    add_executable(test_motion_planners test/test_motion_planners.cpp)

    target_include_directories(test_motion_planners PRIVATE ${MOTIONPLANNER_LIB_INCLUDES})

    target_link_libraries(test_motion_planners PRIVATE
        ${TARGET_NAME}
        ${MOTIONPLANNER_DEP_LIBS}
    )

    add_test(NAME MotionPlannersTest COMMAND test_motion_planners)
endif()
