project(trajopt_sco)

find_package(Eigen3 REQUIRED)

set (TRAJOPT_SCO_DIR_PATH ${CMAKE_CURRENT_LIST_DIR})

get_directory_property(TRAJOPT_UTILS_DIR_PATH DIRECTORY ../trajopt_utils DEFINITION TRAJOPT_UTILS_DIR_PATH)

include_directories(${EIGEN3_INCLUDE_DIR} ${TRAJOPT_SCO_DIR_PATH}/include ${TRAJOPT_UTILS_DIR_PATH}/include)


set(SCO_SOURCE_FILES
        ${TRAJOPT_SCO_DIR_PATH}/src/solver_interface.cpp
        ${TRAJOPT_SCO_DIR_PATH}/src/modeling.cpp
        ${TRAJOPT_SCO_DIR_PATH}/src/expr_ops.cpp
        ${TRAJOPT_SCO_DIR_PATH}/src/expr_vec_ops.cpp
        ${TRAJOPT_SCO_DIR_PATH}/src/optimizers.cpp
        ${TRAJOPT_SCO_DIR_PATH}/src/modeling_utils.cpp
        ${TRAJOPT_SCO_DIR_PATH}/src/num_diff.cpp
)

if (NOT APPLE)
  set (HAVE_BPMPD TRUE)
endif()


if (HAVE_BPMPD)
  add_executable(bpmpd_caller ${TRAJOPT_SCO_DIR_PATH}/src/bpmpd_caller.cpp)

  if( CMAKE_SIZEOF_VOID_P EQUAL 8 ) # 64 bits
        set(BPMPD_LIBRARY "${TRAJOPT_SCO_DIR_PATH}/3rdpartylib/bpmpd_linux64.a")
  else()
        set(BPMPD_LIBRARY "${TRAJOPT_SCO_DIR_PATH}/3rdpartylib/bpmpd_linux32.a")
  endif()

target_link_libraries(bpmpd_caller ${BPMPD_LIBRARY} -static)
list(APPEND SCO_SOURCE_FILES  ${TRAJOPT_SCO_DIR_PATH}/src/bpmpd_interface.cpp)
set_property(SOURCE  ${TRAJOPT_SCO_DIR_PATH}/src/bpmpd_interface.cpp APPEND PROPERTY COMPILE_DEFINITIONS BPMPD_CALLER="\\\"${CMAKE_CURRENT_BINARY_DIR}/bpmpd_caller\\\"")


set(BPMPD_WORKING_DIR ${CMAKE_CURRENT_BINARY_DIR})
set_property(SOURCE ${TRAJOPT_SCO_DIR_PATH}/src/bpmpd_caller.cpp APPEND PROPERTY COMPILE_DEFINITIONS BPMPD_WORKING_DIR="${BPMPD_WORKING_DIR}")
file(COPY ${TRAJOPT_SCO_DIR_PATH}/src/bpmpd.par DESTINATION ${CMAKE_CURRENT_BINARY_DIR})


  set_property(SOURCE ${TRAJOPT_SCO_DIR_PATH}/src/solver_interface.cpp APPEND PROPERTY COMPILE_DEFINITIONS HAVE_BPMPD )
endif()

if (GUROBI_FOUND)
  set_property(SOURCE ${TRAJOPT_SCO_DIR_PATH}/src/solver_interface.cpp APPEND PROPERTY COMPILE_DEFINITIONS HAVE_GUROBI )
  list(APPEND SCO_SOURCE_FILES ${TRAJOPT_SCO_DIR_PATH}/src/gurobi_interface.cpp)
endif(GUROBI_FOUND)

add_library(${PROJECT_NAME} SHARED
  ${SCO_SOURCE_FILES}
)


set (SCO_LINK_LIBS trajopt_utils)
if (GUROBI_FOUND)
  list(APPEND SCO_LINK_LIBS ${GUROBI_LIBRARIES})
endif()
if (HAVE_BPMPD)
  list(APPEND SCO_LINK_LIBS ${BPMPD_LIBRARY})
endif()

target_link_libraries(${PROJECT_NAME} ${SCO_LINK_LIBS})

install(DIRECTORY include/ DESTINATION include/motion_planners/planner/trajopt/)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)

# add_subdirectory(test)
