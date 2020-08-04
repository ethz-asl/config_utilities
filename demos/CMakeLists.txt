cmake_minimum_required(VERSION 2.8.3)
project(config_utilities_demos)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

###############
# Executables #
###############

cs_add_executable(config_demo.cpp
        config_demo.cpp
        )
target_link_libraries(config_demo ${PROJECT_NAME})

##########
# Export #
##########

cs_install()
cs_export()