cmake_minimum_required(VERSION 3.10)
project(legged_hil_lcm)

find_package(catkin REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED lcm)

find_program(LCM_GEN_EXECUTABLE lcm-gen REQUIRED)
message(STATUS "Found lcm-gen: ${LCM_GEN_EXECUTABLE}")

# (Optional) Make C++ standard consistent across your package
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

###################################
## Paths
###################################
set(LCM_MSG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lcm)

# Put generated files in the build tree (recommended)
set(LCM_CPP_GEN_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(LCM_JAVA_GEN_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lcm_java)

file(GLOB LCM_FILES CONFIGURE_DEPENDS ${LCM_MSG_DIR}/*.lcm)

###################################
## LCM C++ message generation
###################################
set(LCM_HEADERS "")
foreach(lcm_file IN LISTS LCM_FILES)
  get_filename_component(lcm_name ${lcm_file} NAME_WE)
  set(out_hpp ${LCM_CPP_GEN_DIR}/${lcm_name}.hpp)

  add_custom_command(
    OUTPUT ${out_hpp}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${LCM_CPP_GEN_DIR}
    COMMAND ${LCM_GEN_EXECUTABLE} --cpp --cpp-std=c++11
            --cpp-hpath=${LCM_CPP_GEN_DIR}
            ${lcm_file}
    DEPENDS ${lcm_file}
    COMMENT "Generating LCM C++ header: ${lcm_name}.hpp"
    VERBATIM
  )

  list(APPEND LCM_HEADERS ${out_hpp})
endforeach()

add_custom_target(generate_lcm_headers ALL
  DEPENDS ${LCM_HEADERS}
)

###################################
## LCM Java message generation
###################################
find_package(Java REQUIRED)
include(UseJava)

# Allow override of LCM jar path if needed
set(LCM_JAR "/usr/share/java/lcm.jar" CACHE FILEPATH "Path to lcm.jar")

set(LCM_JAVA_STAMP ${LCM_JAVA_GEN_DIR}/lcm_java.stamp)
set(LCM_JAVA_SOURCES_LIST ${LCM_JAVA_GEN_DIR}/java_sources.txt)

add_custom_command(
  OUTPUT ${LCM_JAVA_STAMP}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${LCM_JAVA_GEN_DIR}
  COMMAND ${LCM_GEN_EXECUTABLE} -j --jpath=${LCM_JAVA_GEN_DIR} ${LCM_FILES}
  COMMAND find ${LCM_JAVA_GEN_DIR} -name "*.java" > ${LCM_JAVA_SOURCES_LIST}
  COMMAND ${Java_JAVAC_EXECUTABLE} -cp ${LCM_JAR} -d ${LCM_JAVA_GEN_DIR} @${LCM_JAVA_SOURCES_LIST}
  COMMAND ${CMAKE_COMMAND} -E touch ${LCM_JAVA_STAMP}
  DEPENDS ${LCM_FILES}
  COMMENT "Generating and compiling LCM Java classes"
  VERBATIM
)

add_custom_target(lcm_java ALL
  DEPENDS ${LCM_JAVA_STAMP}
)

###################################
## Catkin package
###################################
catkin_package(
  INCLUDE_DIRS include ${LCM_CPP_GEN_DIR}
)

include_directories(
  include
  ${LCM_CPP_GEN_DIR}
  ${catkin_INCLUDE_DIRS}
  ${LCM_INCLUDE_DIRS}
)
