project(utils)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

set (TRAJOPT_UTILS_DIR_PATH ${CMAKE_CURRENT_LIST_DIR})

add_library(utils SHARED
    ${CMAKE_CURRENT_LIST_DIR}/src/stl_to_string.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/clock.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/config.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/logging.cpp)
target_link_libraries(utils ${Boost_PROGRAM_OPTIONS_LIBRARY})


install(DIRECTORY include/ DESTINATION include/trajectory_optimization/planner)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
