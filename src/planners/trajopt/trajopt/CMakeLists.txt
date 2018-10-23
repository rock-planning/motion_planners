INCLUDE(FindPkgConfig)

pkg_check_modules(YAML REQUIRED yaml-cpp)

include_directories(
  SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

find_package(Eigen3 REQUIRED)

set (TRAJOPT_PLANNER_DIR_PATH ${CMAKE_CURRENT_LIST_DIR})

get_directory_property(TRAJOPT_UTILS_DIR_PATH DIRECTORY ../trajopt_utils DEFINITION TRAJOPT_UTILS_DIR_PATH)
get_directory_property(TRAJOPT_SCO_DIR_PATH DIRECTORY ../trajopt_sco DEFINITION TRAJOPT_SCO_DIR_PATH)
get_directory_property(JSON_DIR_PATH DIRECTORY ../trajopt_ext DEFINITION JSON_DIR_PATH)


set(TRAJOPT_INCLUDE_HEADER ${TRAJOPT_DIR_PATH}/include $
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
