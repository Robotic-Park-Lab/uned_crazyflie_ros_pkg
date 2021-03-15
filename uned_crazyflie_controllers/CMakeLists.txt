cmake_minimum_required(VERSION 3.0.2)
project(uned_crazyflie_controllers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mav_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(
  Eigen3 REQUIRED
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME} ${Eigen3_INCLUDE_DIRS}
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs rospy std_msgs
  DEPENDS Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${${PROJECT_NAME}_SOURCE_DIR}/include
  ${Eigen3_INCLUDE_DIRS}
)

add_library(uned_crazyflie_position_controller src/uned_crazyflie_controller/uned_crazyflie_position_controller.cpp)
target_link_libraries(uned_crazyflie_position_controller ${catkin_LIBRARIES})
add_dependencies(uned_crazyflie_position_controller ${catkin_EXPORTED_TARGETS})

add_library(uned_crazyflie_attitude_controller src/uned_crazyflie_controller/uned_crazyflie_attitude_controller.cpp)
target_link_libraries(uned_crazyflie_attitude_controller ${catkin_LIBRARIES})
add_dependencies(uned_crazyflie_attitude_controller ${catkin_EXPORTED_TARGETS})

# Declare a C++ executable
add_executable(uned_crazyflie_position_controller_node src/uned_crazyflie_position_controller_node.cpp)
target_link_libraries(uned_crazyflie_position_controller_node uned_crazyflie_position_controller
   ${catkin_LIBRARIES}
)

add_executable(uned_crazyflie_attitude_controller_node src/uned_crazyflie_attitude_controller_node.cpp)
target_link_libraries(uned_crazyflie_attitude_controller_node uned_crazyflie_attitude_controller
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS uned_crazyflie_position_controller uned_crazyflie_attitude_controller
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS uned_crazyflie_position_controller_node uned_crazyflie_attitude_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
