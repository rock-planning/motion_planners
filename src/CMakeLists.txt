project(motion_planners)

set(MOTION_PLANNERS 0.1)
set(TARGET_NAME ${PROJECT_NAME})

CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

INCLUDE(FindPkgConfig)

find_package(Eigen3 REQUIRED)
#find_package(orocos_kdl REQUIRED)
find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
pkg_check_modules(BASE_TYPES REQUIRED base-types)
pkg_search_module(BASE_LOGGING REQUIRED base-logging)
pkg_check_modules(OROCOS_KDL REQUIRED orocos_kdl)
pkg_check_modules(KDL_PARSER REQUIRED kdl_parser)
pkg_check_modules(URDF REQUIRED urdfdom)
pkg_check_modules(URDFHEADER REQUIRED urdfdom_headers)
pkg_check_modules(OCTOMAP REQUIRED octomap)
pkg_check_modules(KINEMATICS REQUIRED kinematics_library)
pkg_check_modules(COLLISION_DETECTION REQUIRED collision_detection)
pkg_check_modules(ROBOT_MODEL REQUIRED robot_model)


get_directory_property(PLANNERS_PATH DIRECTORY planners DEFINITION PLANNERS_DIR_PATH)
get_directory_property(STOMP_PATH DIRECTORY planners DEFINITION STOMP_PATH)
get_directory_property(TRAJOPT_INCLUDE_HEADER DIRECTORY planners DEFINITION TRAJOPT_INCLUDE_HEADER)

#If OMPL is installed through ROS, we need to specify the ompl install folder for linking.
get_directory_property(WRAPPERS_LINK_DIR DIRECTORY planners DEFINITION WRAPPERS_LINK_DIR)

set(DEPS_INCLUDE_HEADER ${EIGEN3_INCLUDE_DIR} ${KDL_PARSER_INCLUDE_DIRS}
                         ${OCTOMAP_INCLUDE_DIRS} ${COLLISION_DETECTION_INCLUDE_DIRS} ${KINEMATICS_INCLUDE_DIRS} ${PLANNERS_PATH}/include
                         ${ROBOT_MODEL_INCLUDE_DIRS} ${STOMP_PATH}/include ${TRAJOPT_INCLUDE_HEADER})

set(MOTIONPLANNER_LIB_INCLUDES  ${CMAKE_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR} ${BASE_TYPES_INCLUDE_DIRS} 
                                ${BASE_LOGGING_INCLUDE_DIRS} ${OROCOS_PARSER_INCLUDE_DIRS} ${KDL_PARSER_INCLUDE_DIRS} 
                                ${URDF_INCLUDE_DIRS} ${URDFHEADER_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${KINEMATICS_INCLUDE_DIRS} 
                                ${COLLISION_DETECTION_INCLUDE_DIRS} ${ROBOT_MODEL_INCLUDE_DIRS} ${PLANNERS_PATH}/include 
                                ${STOMP_PATH}/include ${TRAJOPT_INCLUDE_HEADER})

set(MOTIONPLANNER_LINK_DIRS ${BASE_TYPES_LIBRARY_DIRS} ${BASE_LOGGING_LIBRARY_DIRS} ${KDL_PARSER_LIBRARY_DIRS} 
                            ${URDF_LIBRARY_DIRS} ${URDFHEADER_LIBRARY_DIRS} ${OCTOMAP_LIBRARY_DIRS} 
                            ${KINEMATICS_LIBRARY_DIRS} ${COLLISION_DETECTION_LIBRARY_DIRS} ${ROBOT_MODEL_LIBARAY_DIRS}}
)

set(MOTIONPLANNER_DEP_LIBS  ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY}
                            ${BASE_TYPES_LIBRARIES} ${BASE_LOGGING_LIBRARIES} ${OROCOS_KDL_LIBRARIES} ${KDL_PARSER_LIBRARIES}
                            ${URDF_LIBRARIES} ${URDFHEADER_LIBRARIES} ${OCTOMAP_LIBRARIES} ${KINEMATICS_LIBRARIES} 
                            ${COLLISION_DETECTION_LIBRARIES})


include_directories(${CMAKE_SOURCE_DIR}/include ${MOTIONPLANNER_LIB_INCLUDES})
link_directories(${MOTIONPLANNER_LINK_DIRS} ${WRAPPERS_LINK_DIR})


add_library(motion_planners SHARED
            ${CMAKE_CURRENT_LIST_DIR}/MotionPlanners.cpp
)

target_link_libraries(motion_planners planner_factory ${MOTIONPLANNER_DEPS_LIB})

set(PKGCONFIG_CFLAGS "${PKGCONFIG_CFLAGS} -I${PLANNERS_PATH}/include;${STOMP_PATH}/include;${TRAJOPT_INCLUDE_HEADER}")
set(PKGCONFIG_LIBS "${PKGCONFIG_LIBS} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY}")
set(PROJECT_DESCRIPTION "A libray which holds information on robot states")
# now we add all the required libraries in PKGCONFIG_REQUIRES macro
list(APPEND PKGCONFIG_REQUIRES base-types)
list(APPEND PKGCONFIG_REQUIRES base-logging)
list(APPEND PKGCONFIG_REQUIRES orocos_kdl)
list(APPEND PKGCONFIG_REQUIRES kdl_parser)
list(APPEND PKGCONFIG_REQUIRES urdfdom)
list(APPEND PKGCONFIG_REQUIRES urdfdom_headers)
list(APPEND PKGCONFIG_REQUIRES octomap)
list(APPEND PKGCONFIG_REQUIRES kinematics_library)
list(APPEND PKGCONFIG_REQUIRES collision_detection)
list(APPEND PKGCONFIG_REQUIRES robot_model)
string (REPLACE ";" " " PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES}")


CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

install(DIRECTORY ../include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)
