project(planner_factory)

# Add subdirectories for STOMP and TrajOpt
add_subdirectory(stomp)
add_subdirectory(trajopt)

set(PLANNER_FACTORY_VERSION 0.1)
set(TARGET_NAME ${PROJECT_NAME})
set(PLANNERS_DIR_PATH ${CMAKE_CURRENT_LIST_DIR}) 

INCLUDE(FindPkgConfig)

# Find basic libraries
find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(yaml-cpp REQUIRED)

# Find the packages using pkg-config
pkg_check_modules(OROCOS_KDL REQUIRED orocos_kdl)
pkg_check_modules(KDL_PARSER REQUIRED kdl_parser)
pkg_check_modules(COLLISION_DETECTION REQUIRED collision_detection)
pkg_check_modules(KINEMATICS REQUIRED kinematics_library)
pkg_check_modules(ROBOT_MODEL REQUIRED robot_model)

# Libraries required for linking
set(DEPS_LIB ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} 
              ${OROCOS_KDL_LIBRARIES} ${KDL_PARSER_LIBRARIES} ${ROBOT_MODEL_LIBRARIES}
              ${COLLISION_DETECTION_LIBRARIES} ${KINEMATICS_LIBRARIES})

# Get the planner directories
get_directory_property(STOMP_PATH DIRECTORY stomp DEFINITION STOMP_DIR_PATH)
get_directory_property(TRAJOPT_PATH DIRECTORY trajopt DEFINITION TRAJOPT_PATH)
get_directory_property(TRAJOPT_INCLUDE_HEADER DIRECTORY trajopt DEFINITION TRAJOPT_INCLUDE_HEADER)

# Set the available planner's path
set(PLANNERS_PATH ${STOMP_PATH}/include ${TRAJOPT_INCLUDE_HEADER})

# Set the header and library path
include_directories(${CMAKE_SOURCE_DIR}/include ${CMAKE_CURRENT_LIST_DIR}/include ${EIGEN3_INCLUDE_DIR} 
                    ${OROCOS_KDL_INCLUDE_DIRS} ${KDL_PARSER_INCLUDE_DIRS}
                    ${COLLISION_DETECTION_INCLUDE_DIRS} ${ROBOT_MODEL_INCLUDE_DIRS} ${PLANNERS_PATH} 
                    ${KINEMATICS_INCLUDE_DIRS})

link_directories(${KDL_PARSER_LIBRARY_DIRS} 
                 ${ROBOT_MODEL_LIBRARY_DIRS} 
                 ${OROCOS_KDL_LIBRARY_DIRS} 
                 ${COLLISION_DETECTION_LIBRARY_DIRS}  
                 ${KINEMATICS_LIBRARY_DIRS})

# Abstract Planner Library
add_library(abstract_planner SHARED
            ${CMAKE_CURRENT_LIST_DIR}/src/abstract/AbstractPlanner.cpp
)
target_link_libraries(abstract_planner ${ROBOT_MODEL_LIBRARIES})

# Store wrapper's library name
set(WRAPPERS_LIB "")
# Store link directory
set(WRAPPERS_LINK_DIR "")

# STOMP Wrappers
add_library(stomp_wrapper SHARED
            ${CMAKE_CURRENT_LIST_DIR}/src/wrappers/stomp/OptimizationTask.cpp
            ${CMAKE_CURRENT_LIST_DIR}/src/wrappers/stomp/StompPlanner.cpp	 
            ${CMAKE_CURRENT_LIST_DIR}/src/wrappers/stomp/HandleStompConfig.cpp
)
target_link_libraries(stomp_wrapper abstract_planner stomp ${DEPS_LIB})
LIST(APPEND WRAPPERS_LIB "stomp_wrapper")

# TrajOpt Wrappers
add_library(trajopt_wrapper SHARED
            ${CMAKE_CURRENT_LIST_DIR}/src/wrappers/trajopt/TrajoptPlanner.cpp
            ${CMAKE_CURRENT_LIST_DIR}/src/wrappers/trajopt/RobotModelWrapper.cpp
            ${CMAKE_CURRENT_LIST_DIR}/src/wrappers/trajopt/FCLCollisionChecker.cpp
)

target_link_libraries(trajopt_wrapper abstract_planner trajopt ${DEPS_LIB})
LIST(APPEND WRAPPERS_LIB "trajopt_wrapper")

# OMPL Configuration
find_package(ompl REQUIRED)

if(ompl_FOUND)
    message(STATUS "OMPL found!")
    # include_directories(/usr/include/ompl-1.5)  # Add OMPL includes
    set(OMPL_INCLUDE_DIRS "/usr/include/ompl-1.5")
    include_directories(${OMPL_INCLUDE_DIRS})
    link_directories(${OMPL_LIBRARY_DIRS})

    # OMPL Wrappers
    add_library(ompl_wrapper SHARED
        ${CMAKE_CURRENT_LIST_DIR}/src/wrappers/ompl/OmplPlanner.cpp
        ${CMAKE_CURRENT_LIST_DIR}/src/wrappers/ompl/HandleOmplConfig.cpp
    )

    target_include_directories(ompl_wrapper PUBLIC ${OMPL_INCLUDE_DIRS})

    target_link_libraries(ompl_wrapper
        ${OMPL_LIBRARIES}
        abstract_planner
        collision_detection
        robot_model
        ${DEPS_LIB}
    )

    # target_link_directories(ompl_wrapper PUBLIC ${OMPL_LIBRARY_DIRS})

    LIST(APPEND WRAPPERS_LIB "ompl_wrapper")
    install(TARGETS ompl_wrapper DESTINATION lib)
else()
    message(STATUS "OMPL not found!")
endif()

# Planner Factory Library
add_library(planner_factory SHARED
            ${CMAKE_CURRENT_LIST_DIR}/src/PlannerFactory.cpp
)
target_link_libraries(planner_factory ${DEPS_LIB} ${WRAPPERS_LIB})

# Configuring PKG-Config
set(PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES} stomp")
set(PKGCONFIG_CFLAGS "${PKGCONFIG_CFLAGS};${STOMP_PATH}/include")
set(PKGCONFIG_REQUIRES "${PKGCONFIG_REQUIRES} trajopt")
set(PKGCONFIG_CFLAGS "${PKGCONFIG_CFLAGS};${TRAJOPT_PATH}/trajopt/include")
CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

# Installation Directories
install(DIRECTORY include/ DESTINATION include/motion_planners/planner_factory)
install(TARGETS abstract_planner DESTINATION lib)
install(TARGETS stomp_wrapper DESTINATION lib)
install(TARGETS trajopt_wrapper DESTINATION lib)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)