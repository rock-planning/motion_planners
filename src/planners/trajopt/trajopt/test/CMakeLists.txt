cmake_minimum_required(VERSION 3.0)
find_package(Boost COMPONENTS system filesystem unit_test_framework REQUIRED)
add_executable(trajoptTest trajoptTest.cpp)

target_link_libraries(trajoptTest
                      ${Boost_FILESYSTEM_LIBRARY}
                      ${Boost_SYSTEM_LIBRARY}
                      ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY}
                      )


#add_test(tester tester)
