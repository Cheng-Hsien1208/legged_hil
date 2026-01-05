cmake_minimum_required(VERSION 3.0.2)
project(legged_hil_mirle_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED
    COMPONENTS
)

###################################
## catkin specific configuration ##
###################################
catkin_package(CATKIN_DEPENDS)


# Mark resource files for installation
install(
    DIRECTORY meshes urdf launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

