add_subdirectory(stomp)
add_subdirectory(trajopt)

project(planner_factory)

set(PLANNER_FACTORY_VERSION 0.1)
set(TARGET_NAME ${PROJECT_NAME})
set (PLANNERS_DIR_PATH ${CMAKE_CURRENT_LIST_DIR})

INCLUDE(FindPkgConfig)

# get basic library
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(PCL 1.7 REQUIRED COMPONENTS core features filters io segmentation surface)
pkg_check_modules(kdl_parser REQUIRED kdl_parser)
pkg_check_modules(COLLISION_DETECTION REQUIRED collision_detection)
pkg_check_modules(KINEMATICS REQUIRED kinematics_library)
pkg_check_modules(YAML REQUIRED yaml-cpp)

# get the planners
get_directory_property(STOMP_PATH DIRECTORY stomp DEFINITION STOMP_DIR_PATH)
get_directory_property(ROBOT_MODEL_PATH DIRECTORY ../robot_model DEFINITION ROBOT_MODEL_DIR_PATH)
get_directory_property(TRAJOPT_PATH DIRECTORY trajopt DEFINITION TRAJOPT_PATH)

get_directory_property(TRAJOPT_INCLUDE_HEADER DIRECTORY trajopt DEFINITION TRAJOPT_INCLUDE_HEADER)

# assign the available planner's path
set (PLANNERS_PATH ${STOMP_PATH}/include ${TRAJOPT_INCLUDE_HEADER})

#set the header and library path
include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_CURRENT_LIST_DIR}/include ${EIGEN3_INCLUDE_DIR} ${PCL_INCLUDE_DIRS} ${kdl_parser_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS}
		    ${YAML_INCLUDE_DIRS} ${COLLISION_DETECTION_INCLUDE_DIRS} ${ROBOT_MODEL_PATH}/include ${PLANNERS_PATH} ${KINEMATICS_INCLUDE_DIRS})
link_directories(${kdl_parser_LIBRARY_DIRS})

#store wrapper's library name
set (WRAPPERS_LIB "")
#store link directory
set (WRAPPERS_LINK_DIR "")

#stomp wrappers
add_library(stomp_wrapper SHARED
	    ${CMAKE_CURRENT_LIST_DIR}/src/wrappers/stomp/OptimizationTask.cpp
	    ${CMAKE_CURRENT_LIST_DIR}/src/wrappers/stomp/StompPlanner.cpp
	    ${CMAKE_CURRENT_LIST_DIR}/src/wrappers/stomp/HandleStompConfig.cpp
)
target_link_libraries(stomp_wrapper ${orocos_kdl_LIBRARIES} ${YAML_LIBRARIES} stomp collision_detection robot_model)
LIST(APPEND WRAPPERS_LIB "stomp_wrapper")

add_library(trajopt_wrapper SHARED
            ${CMAKE_CURRENT_LIST_DIR}/src/wrappers/trajopt/TrajoptPlanner.cpp
            ${CMAKE_CURRENT_LIST_DIR}/src/wrappers/trajopt/RobotModelWrapper.cpp
            ${CMAKE_CURRENT_LIST_DIR}/src/wrappers/trajopt/FCLCollisionChecker.cpp
            ${CMAKE_CURRENT_LIST_DIR}/src/wrappers/trajopt/RobotLinkWrapper.cpp
)

target_link_libraries(trajopt_wrapper ${orocos_kdl_LIBRARIES} ${YAML_LIBRARIES} trajopt collision_detection robot_model)

LIST(APPEND WRAPPERS_LIB "trajopt_wrapper")

# if OMPL is found, add it to the planner factory
pkg_check_modules(OMPL ompl)
set(OMPL_FOUND false)
if(${OMPL_FOUND})
    message ("OMPL found !")
    message("OMPL dir ${OMPL_INCLUDE_DIRS}")
    message(${OMPL_LIBRARY_DIRS})
    message(${OMPL_LIBRARIES})
    include_directories(${PLANNERS_PATH} ${OMPL_INCLUDE_DIRS})
    link_directories(${OMPL_LIBRARY_DIRS})

    add_library(ompl_wrapper SHARED
	    ${CMAKE_CURRENT_LIST_DIR}/src/wrappers/ompl/OmplPlanner.cpp
	    ${CMAKE_CURRENT_LIST_DIR}/src/wrappers/ompl/HandleOmplConfig.cpp)

    target_link_libraries(ompl_wrapper ${OMPL_LIBRARIES} ${orocos_kdl_LIBRARIES} ${YAML_LIBRARIES} collision_detection robot_model)

    LIST(APPEND WRAPPERS_LIB "ompl_wrapper")
    #If OMPL is installed through ROS, we need to specify the ompl install folder for linking.
    LIST(APPEND WRAPPERS_LINK_DIR ${OMPL_LIBRARY_DIRS})

    install(TARGETS ompl_wrapper DESTINATION lib)

else(${OMPL_FOUND})
    message ("OMPL not found !")
endif()

add_definitions(-DOMPL_FOUND=${OMPL_FOUND})

#planner factory
add_library(planner_factory SHARED
            ${CMAKE_CURRENT_LIST_DIR}/src/PlannerFactory.cpp
)

target_link_libraries(planner_factory ${orocos_kdl_LIBRARIES} ${YAML_LIBRARIES} ${WRAPPERS_LIB})

set(PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES} stomp")
set(PKGCONFIG_CFLAGS "${PKGCONFIG_CFLAGS};${STOMP_PATH}/include")

set(PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES} trajopt")
set(PKGCONFIG_CFLAGS "${PKGCONFIG_CFLAGS};${TRAJOPT_PATH}/trajopt/include")

CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

install(DIRECTORY include/ DESTINATION include/motion_planners/planner_factory)
install(TARGETS stomp_wrapper DESTINATION lib)
install(TARGETS trajopt_wrapper DESTINATION lib)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
#install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig)
