project(stomp)

set(STOMP_VERSION 0.1)

set(TARGET_NAME ${PROJECT_NAME})

CONFIGURE_FILE("${PROJECT_NAME}.pc.in" "${PROJECT_NAME}.pc" @ONLY)

#set(CMAKE_BUILD_TYPE RelWithDebInfo)
find_package(Boost COMPONENTS system filesystem program_options REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
INCLUDE(FindPkgConfig)
#pkg_check_modules(YAML REQUIRED yaml-cpp)

set (STOMP_DIR_PATH ${CMAKE_CURRENT_LIST_DIR})

#include_directories(${STOMP_DIR_PATH}/include ${EIGEN3_INCLUDE_DIR} ${orocos_kdl_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS})
include_directories(${STOMP_DIR_PATH}/include ${EIGEN3_INCLUDE_DIR} ${orocos_kdl_INCLUDE_DIRS})

#message(${orocos_kdl_INCLUDE_DIRS})
#message(${orocos_kdl_LIBRARIES})
#message(${STOMP_DIR_PATH})

add_library(stomp SHARED
            ${CMAKE_CURRENT_LIST_DIR}/src/CovariantMovementPrimitive.cpp
            ${CMAKE_CURRENT_LIST_DIR}/src/PolicyImprovement.cpp
            ${CMAKE_CURRENT_LIST_DIR}/src/Stomp.cpp
            ${CMAKE_CURRENT_LIST_DIR}/src/StompUtils.cpp            
)

#target_link_libraries(stomp ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${OpenMP_CXX_FLAGS} ${orocos_kdl_LIBRARIES} ${YAML_LIBRARIES})
target_link_libraries(stomp ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_PROGRAM_OPTIONS_LIBRARY} ${OpenMP_CXX_FLAGS} ${orocos_kdl_LIBRARIES})

#add_executable(test_stomp2d
#  test/stomp_2d_test.cpp
#)
#
#add_executable(test_cmp
#  test/test_cmp.cpp
#)
#
#add_executable(stomp_test_distributions
#  test/test_distributions.cpp
#  src/stomp_constrained.cpp
#)

install(DIRECTORY include/ DESTINATION include/motion_planners/planner)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
#install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig) 

#add_subdirectory(test)
