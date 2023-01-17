cmake_minimum_required(VERSION 3.0.0)
project(config_utilities)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

include_directories(code)

###############
# Executables #
###############

cs_add_executable(demo_config_checker
        demos/demo_config_checker.cpp
        )

cs_add_executable(demo_config
        demos/demo_config.cpp
        )

cs_add_executable(demo_ros_param
        demos/demo_ros_param.cpp
        )

cs_add_executable(demo_inheritance
        demos/demo_inheritance.cpp
        )

cs_add_executable(demo_factory
        demos/demo_factory.cpp
        )

cs_add_executable(demo_ros_factory
        demos/demo_ros_factory.cpp
        )

cs_add_executable(demo_variable_config
        demos/demo_variable_config.cpp
        )

cs_add_executable(demo_global_settings
demos/demo_global_settings.cpp
)

########
# Test #
########

if (CATKIN_ENABLE_TESTING)
    find_package(gtest)
    catkin_add_gtest(demos-test test/demos.cpp)
    target_link_libraries(demos-test gtest ${catkin_LIBRARIES})
endif()

##########
# Export #
##########

cs_install()
cs_export()
