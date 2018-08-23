
INCLUDE(FindPkgConfig)

pkg_check_modules(YAML REQUIRED yaml-cpp)

include_directories(
  SYSTEM
  # ${BULLET_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen2_INCLUDE_DIRS}
  # ${JSON_INCLUDE_DIRS}
)
include("${CMAKE_CURRENT_LIST_DIR}/../cmake/FindEigen2.cmake")

set (TRAJOPT_PLANNER_DIR_PATH ${CMAKE_CURRENT_LIST_DIR})

get_directory_property(TRAJOPT_UTILS_DIR_PATH DIRECTORY ../trajopt_utils DEFINITION TRAJOPT_UTILS_DIR_PATH)
get_directory_property(TRAJOPT_SCO_DIR_PATH DIRECTORY ../trajopt_sco DEFINITION TRAJOPT_SCO_DIR_PATH)
get_directory_property(JSON_DIR_PATH DIRECTORY ../trajopt_ext DEFINITION JSON_DIR_PATH)


set(TRAJOPT_INCLUDE_HEADER ${TRAJOPT_DIR_PATH}/include ${EIGEN_INCLUDE_DIR} ${BOOST_INCLUDE_DIR}
    ${TRAJOPT_DIR_PATH}/ext
    ${TRAJOPT_SCO_DIR_PATH}/include
    ${TRAJOPT_PLANNER_DIR_PATH}/include
    ${TRAJOPT_UTILS_DIR_PATH}/include
    ${JSON_DIR_PATH}/include
                                         )

include_directories(${TRAJOPT_INCLUDE_HEADER}  ${YAML_INCLUDE_DIRS}
)

if (NOT APPLE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden")
endif()


# add_subdirectory(test)
