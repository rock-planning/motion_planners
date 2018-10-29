project(motion_planners)

set(MOTION_PLANNERS 0.1)
set(TARGET_NAME ${PROJECT_NAME})

CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

INCLUDE(FindPkgConfig)

find_package(Eigen3 REQUIRED)
pkg_check_modules(kdl_parser REQUIRED kdl_parser)

find_package(PCL 1.8 REQUIRED COMPONENTS features filters io segmentation surface)
set(PCL_VERSION "${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}")

pkg_check_modules(KINEMATICS REQUIRED kinematics_library)
pkg_check_modules(COLLISION_DETECTION REQUIRED collision_detection)
message(${PCL_VERSION})
get_directory_property(ROBOT_MODEL_PATH DIRECTORY robot_model DEFINITION ROBOT_MODEL_DIR_PATH)
get_directory_property(PLANNERS_PATH DIRECTORY planners DEFINITION PLANNERS_DIR_PATH)
get_directory_property(STOMP_PATH DIRECTORY planners DEFINITION STOMP_PATH)
get_directory_property(TRAJOPT_INCLUDE_HEADER DIRECTORY planners DEFINITION TRAJOPT_INCLUDE_HEADER)

#If OMPL is installed through ROS, we need to specify the ompl install folder for linking.
get_directory_property(WRAPPERS_LINK_DIR DIRECTORY planners DEFINITION WRAPPERS_LINK_DIR)

set(DEPS_INCLUDE_HEADER ${EIGEN3_INCLUDE_DIR} ${kdl_parser_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS}
					 ${COLLISION_DETECTION_INCLUDE_DIRS} ${KINEMATICS_INCLUDE_DIRS} ${PLANNERS_PATH}/include
				 ${ROBOT_MODEL_PATH}/include ${STOMP_PATH}/include ${TRAJOPT_INCLUDE_HEADER})
include_directories(${CMAKE_SOURCE_DIR}/include ${DEPS_INCLUDE_HEADER})
link_directories(${kdl_parser_LIBRARY_DIRS} ${WRAPPERS_LINK_DIR})


add_library(motion_planners SHARED
            ${CMAKE_CURRENT_LIST_DIR}/MotionPlanners.cpp
)

target_link_libraries(motion_planners collision_detection kinematics_library robot_model planner_factory ${PCL_LIBRARIES})

set(PKGCONFIG_CFLAGS "${PKGCONFIG_CFLAGS} -I${CMAKE_SOURCE_DIR}/include;${DEPS_INCLUDE_HEADER}")
CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

install(DIRECTORY ../include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)
