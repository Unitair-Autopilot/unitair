# This file is parsed by the tool CMake to automatically generate a MakeFile
# which is parsed by the tool Make to compile and link the entire project.
##################################################

cmake_minimum_required(VERSION 3.0.2)
project(unitair)  # just names the project

option(FAST_MATH "Enable the -ffast-math compiler flag." ON)

#########################
# Dependencies
#########################

# Catkin macros and libraries for ROS
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    rospy
    message_generation
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
    tf
)

# YAML-CPP for configuration
find_package(yaml-cpp REQUIRED)

# Eigen for math (headers-only library)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Doxygen for documentation (optional)
find_package(Doxygen)

# Stokesim for simulation (optional)
set(STOKESIM_INCLUDE_DIR "/usr/local/include/stokesim/")
if(EXISTS ${STOKESIM_INCLUDE_DIR})
  set(STOKESIM_LIBRARIES ${STOKESIM_INCLUDE_DIR}/../../lib/libstokesim.so)
  set(STOKESIM_FOUND TRUE)
else()
  message(WARNING "Stokesim installation not found.\nSkipping build of the Unitair-Stokesim plugin.")
endif()

#########################
# Documentation
#########################

# Configure Doxygen target (if Doxygen is installed)
if(${Doxygen_FOUND})
  set(DOXYGEN_IN ${PROJECT_SOURCE_DIR}/doc/Doxyfile.in)
  set(DOXYGEN_OUT ${PROJECT_BINARY_DIR}/Doxyfile)
  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT})
  add_custom_target(${PROJECT_NAME}_doxygen ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    COMMENT "Generating ${PROJECT_NAME} source/API documentation with Doxygen"
    VERBATIM
  )
else()
  message(WARNING "Doxygen installation not found.\nSkipping build of the documentation.")
endif()

#########################
# ROS
#########################

# Add the messages in the 'msg' folder
file(GLOB_RECURSE MSG_FILES RELATIVE ${PROJECT_SOURCE_DIR}/msg CONFIGURE_DEPENDS *.msg)
add_message_files(FILES ${MSG_FILES})

# Add the services in the 'srv' folder
file(GLOB_RECURSE SRV_FILES RELATIVE ${PROJECT_SOURCE_DIR}/srv CONFIGURE_DEPENDS *.srv)
add_service_files(FILES ${SRV_FILES})

# Generate source for the added messages and services
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
)

# Establish catkin package
catkin_package(
  INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/src  # our headers and source files are kept together
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    visualization_msgs
    tf
)

#########################
# Compiler
#########################

# Header file locations (order matters)
include_directories(
  ${PROJECT_SOURCE_DIR}/src  # our headers and source files are kept together
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# Libraries that all targets will link to
link_libraries(
  ${YAML_CPP_LIBRARIES}
)

# Compile as C++14, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)
# Compile with highest optimizations
add_compile_options(-O3)
if(FAST_MATH)
  add_compile_options(-ffast-math)
endif()
# Compile with all common warnings enabled
add_compile_options(-Wall)

# Create directories for local build and logging outputs
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/bld)
file(MAKE_DIRECTORY ${PROJECT_SOURCE_DIR}/log)

#########################
# Helpers
#########################

# Convenience function for targets that interface with ROS
function(enable_ros TARGET)
  target_compile_definitions(${TARGET} PUBLIC UNITAIR_USE_ROS)
  target_link_libraries(${TARGET} ${catkin_LIBRARIES})
  add_dependencies(${TARGET} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endfunction()

# Convenience function for targets that interface with hardware
function(enable_hardware TARGET)
  target_compile_definitions(${TARGET} PUBLIC UNITAIR_USE_HARDWARE)
  target_sources(${TARGET} PRIVATE src/hardware/common/i2c.cpp)
  target_link_libraries(${TARGET} i2c)
endfunction()

#########################
# Libraries
#########################

# Stokesim-plugin shared-object
#####
if(${STOKESIM_FOUND})
  add_library(stokesim_node SHARED
    src/simulation/stokesim_node.cpp
  )
  target_include_directories(stokesim_node PRIVATE ${STOKESIM_INCLUDE_DIR})
  enable_ros(stokesim_node)
  # Place plugin on a consistent path
  set_target_properties(stokesim_node PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bld
  )
  # Guarantee rebuild if Stokesim is modified
  add_custom_command(
    OUTPUT  ${PROJECT_SOURCE_DIR}/src/simulation/stokesim_node.cpp
    COMMAND ${CMAKE_COMMAND} -E touch ${PROJECT_SOURCE_DIR}/src/simulation/stokesim_node.cpp
    DEPENDS ${STOKESIM_LIBRARIES}
  )
endif()
#####

# ...

#########################
# Executables
#########################

# Tools
#####
add_executable(tools_test
  src/tools/tools_test.cpp
)
#####

# Mission
#####
# (all Python)
#####

# Guidance
#####
add_executable(planner_test
  src/guidance/planner_test.cpp
  src/guidance/planner.cpp
)

add_executable(planner_node
  src/guidance/planner_node.cpp
  src/guidance/planner.cpp
)
enable_ros(planner_node)
#####

# Navigation
#####
file(GLOB NAVIGATION_SUBCLASSES
  src/navigation/processes/*.cpp
  src/navigation/informants/*.cpp
)

add_executable(estimator_test
  src/navigation/estimator_test.cpp
  src/navigation/estimator.cpp
  ${NAVIGATION_SUBCLASSES}
)

add_executable(estimator_node
  src/navigation/estimator_node.cpp
  src/navigation/estimator.cpp
  ${NAVIGATION_SUBCLASSES}
)
enable_ros(estimator_node)
#####

# Control
#####
add_executable(regulator_test
  src/control/regulator_test.cpp
  src/control/regulator.cpp
)

add_executable(regulator_node
  src/control/regulator_node.cpp
  src/control/regulator.cpp
)
enable_ros(regulator_node)

add_executable(allocator_test
  src/control/allocator_test.cpp
  src/control/allocator.cpp
)

add_executable(allocator_node
  src/control/allocator_node.cpp
  src/control/allocator.cpp
)
enable_ros(allocator_node)
#####

# Hardware
#####
add_executable(pwm_driver_node
  src/hardware/pwm_driver/PCA9685.cpp
  src/hardware/pwm_driver/pwm_driver_node.cpp
)
enable_hardware(pwm_driver_node)
enable_ros(pwm_driver_node)

add_executable(accelerometer_magnetometer_node
  src/hardware/accelerometer_magnetometer/FXOS8700.cpp
  src/hardware/accelerometer_magnetometer/accelerometer_magnetometer_node.cpp
)
enable_hardware(accelerometer_magnetometer_node)
enable_ros(accelerometer_magnetometer_node)

add_executable(gyroscope_node
  src/hardware/gyroscope/FXAS21002.cpp
  src/hardware/gyroscope/gyroscope_node.cpp
)
enable_hardware(gyroscope_node)
enable_ros(gyroscope_node)

add_executable(barometer_node
  src/hardware/barometer/DPS310.cpp
  src/hardware/barometer/barometer_node.cpp
)
enable_hardware(barometer_node)
enable_ros(barometer_node)

add_executable(pitot_node
  src/hardware/pitot/MS4525DO.cpp
  src/hardware/pitot/pitot_node.cpp
)
enable_hardware(pitot_node)
enable_ros(pitot_node)
#####

# ...

##################################################
