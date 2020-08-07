cmake_minimum_required(VERSION 2.8.3)
project(config_utilities_demos)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

###############
# Executables #
###############

cs_add_executable(demo_config_checker
        demo_config_checker.cpp
        )

cs_add_executable(demo_config
        demo_config.cpp
        )

#cs_add_executable(demo_ros_param
#        demo_ros_param.cpp
#        )
#
cs_add_executable(demo_inheritance
        demo_inheritance.cpp
        )

##########
# Export #
##########

cs_install()
cs_export()