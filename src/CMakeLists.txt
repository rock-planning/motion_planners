rock_library(motion_planners
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(motion_planners_bin Main.cpp
    DEPS motion_planners)

