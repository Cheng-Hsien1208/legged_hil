cmake_minimum_required(VERSION 3.10)
project(legged_hil_interface)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp hardware_interface
)

# Header-only package: no need for global include_directories(),
# but it's harmless. Prefer target-based.
add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(legged_hil_interface INTERFACE
  include
)

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling for target ${PROJECT_NAME}")
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CF_WERROR
  )
endif()

#############
## Install ##
#############

# Install headers
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

# Install scripts
install(PROGRAMS scripts/generate_urdf.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/scripts
)
