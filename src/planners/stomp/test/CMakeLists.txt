
INCLUDE(FindPkgConfig)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

#pkg_check_modules(STOMP REQUIRED stomp)

pkg_check_modules(YAML REQUIRED yaml-cpp)

message(${STOMP_DIR_PATH}/include)

include_directories(${EIGEN3_INCLUDE_DIR} ${STOMP_DIR_PATH}/include ${orocos_kdl_INCLUDE_DIRS} ${YAML_INCLUDE_DIRS})
link_directories(${YAML_LIBRARY_DIRS})



add_executable(test_cmp test_cmp.cpp )
add_executable(stomp_2d stomp_2d_test.cpp )

target_link_libraries(test_cmp stomp ${YAML_LIBRARIES})
target_link_libraries(stomp_2d stomp ${YAML_LIBRARIES})

#target_include_directories(test_cmp PUBLIC ${STOMP_INCLUDE_DIRS})
#add_definitions(${EIGEN_DEFINITIONS} ${OpenMP_CXX_FLAGS})

#install(DIRECTORY include/ DESTINATION include/trajectory_optimisation/planner)
#install(TARGETS ${PROJECT_NAME} DESTINATION lib)
#install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.pc DESTINATION lib/pkgconfig) 



