project(trajopt_utils)

include_directories(${CMAKE_CURRENT_LIST_DIR}/include)

set (TRAJOPT_UTILS_DIR_PATH ${CMAKE_CURRENT_LIST_DIR})

add_library(${PROJECT_NAME} SHARED
    ${CMAKE_CURRENT_LIST_DIR}/src/stl_to_string.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/clock.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/config.cpp
    ${CMAKE_CURRENT_LIST_DIR}/src/logging.cpp)
target_link_libraries(${PROJECT_NAME} ${Boost_PROGRAM_OPTIONS_LIBRARY})


install(DIRECTORY include/ DESTINATION include/motion_planners/planner/trajopt/)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
