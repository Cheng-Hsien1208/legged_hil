cmake_minimum_required(VERSION 3.10)
project(legged_hil_lcm_hw)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  legged_hil_hw
  legged_hil_lcm
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS roscpp legged_hil_hw legged_hil_lcm
)

include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${LCM_INCLUDE_DIRS}
)

link_directories(
  ${LCM_LIBRARY_DIRS}
)

# ---- library ----
add_library(${PROJECT_NAME}_lib
  src/LcmHW.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${LCM_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_lib
  ${catkin_EXPORTED_TARGETS}
)

# ---- (optional) node ----
add_executable(${PROJECT_NAME}_node
  src/legged_hil_lcm_hw_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${LCM_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_node
  ${catkin_EXPORTED_TARGETS}
)

# ---- install ----
install(TARGETS ${PROJECT_NAME}_lib ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
