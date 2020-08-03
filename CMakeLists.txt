cmake_minimum_required(VERSION 2.8.3)
project(config_utilities)

set(CMAKE_CXX_STANDARD 17)
find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_package()

#############
# Libraries #
#############

cs_add_library(${PROJECT_NAME}
        src/config_utility_lib.cpp
        )

###############
# Executables #
###############

cs_add_executable(demo
        src/demo.cpp
        )
target_link_libraries(demo ${PROJECT_NAME})

##########
# Export #
##########

cs_install()
cs_export()