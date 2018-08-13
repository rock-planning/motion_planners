project(robot_model)

set(ROBOT_MODEL_VERSION 0.1)
set(TARGET_NAME ${PROJECT_NAME})

INCLUDE(FindPkgConfig)

find_package(Eigen3 REQUIRED)
#find_package(orocos_kdl REQUIRED)
pkg_check_modules(kdl_parser REQUIRED kdl_parser)

find_package(PCL 1.7 REQUIRED COMPONENTS core features filters io segmentation surface)
set(PCL_VERSION "${PCL_VERSION_MAJOR}.${PCL_VERSION_MINOR}")

pkg_check_modules(KINEMATICS REQUIRED kinematics_library)
pkg_check_modules(COLLISION_DETECTION REQUIRED collision_detection)
pkg_check_modules(SRDF REQUIRED srdfdom)


set (ROBOT_MODEL_DIR_PATH ${CMAKE_CURRENT_LIST_DIR})

#include_directories(${ROBOT_MODEL_DIR_PATH}/include ${EIGEN3_INCLUDE_DIR} ${kdl_parser_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(${ROBOT_MODEL_DIR_PATH}/include ${EIGEN3_INCLUDE_DIR} ${kdl_parser_INCLUDE_DIRS} ${SRDF_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} 
		    ${COLLISION_DETECTION_INCLUDE_DIRS} ${KINEMATICS_INCLUDE_DIRS} )
link_directories(${kdl_parser_LIBRARY_DIRS})


add_library(robot_model SHARED
            ${CMAKE_CURRENT_LIST_DIR}/src/RobotModel.cpp
            ${CMAKE_CURRENT_LIST_DIR}/src/RobotLink.cpp
            ${CMAKE_CURRENT_LIST_DIR}/src/HelperFunctions.cpp
)

target_link_libraries(robot_model   ${kdl_parser_LIBRARIES} ${SRDF_LIBRARIES} ${orocos_kdl_LIBRARIES} 
				    ${PCL_LIBRARIES} abstract_collision_detection ${COLLISION_DETECTION_LIBRARIES} ${KINEMATICS_LIBRARIES})

set(PKGCONFIG_CFLAGS "${PKFCONFIG_CFLAGS} -I${COLLISION_DETECTION_INCLUDE_DIRS};${KINEMATICS_INCLUDE_DIRS}")
CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

install(DIRECTORY include/ DESTINATION include/motion_planners/robot_model)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig) 




