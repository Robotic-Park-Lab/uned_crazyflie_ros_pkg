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

################################################
## Declare ROS messages, services and actions ##
################################################
add_message_files(
  DIRECTORY msg
  FILES
    AttitudeRefs.msg
    RateMixerRefs.msg
)

generate_messages(DEPENDENCIES std_msgs)

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

########################################################################################################################
add_library(uned_crazyflie_trajectory_controller src/uned_crazyflie_controller/uned_crazyflie_trajectory_controller.cpp)
target_link_libraries(uned_crazyflie_trajectory_controller ${catkin_LIBRARIES})
add_dependencies(uned_crazyflie_trajectory_controller ${catkin_EXPORTED_TARGETS})

add_executable(uned_crazyflie_trajectory_controller_node src/uned_crazyflie_trajectory_controller_node.cpp)
target_link_libraries(uned_crazyflie_trajectory_controller_node uned_crazyflie_trajectory_controller
   ${catkin_LIBRARIES}
)
#################################
##     POSITION CONTROLLER     ##
#################################
add_library(periodic_pid_position_controller src/uned_crazyflie_controller/periodic_pid_position_controller.cpp)
target_link_libraries(periodic_pid_position_controller ${catkin_LIBRARIES})
add_dependencies(periodic_pid_position_controller ${catkin_EXPORTED_TARGETS})

add_executable(periodic_pid_position_controller_node src/periodic_pid_position_controller_node.cpp)
target_link_libraries(periodic_pid_position_controller_node periodic_pid_position_controller
   ${catkin_LIBRARIES}
)
#################################
##     ATTITUDE CONTROLLER     ##
#################################
add_library(periodic_pid_attitude_controller src/uned_crazyflie_controller/periodic_pid_attitude_controller.cpp)
target_link_libraries(periodic_pid_attitude_controller ${catkin_LIBRARIES})
add_dependencies(periodic_pid_attitude_controller ${catkin_EXPORTED_TARGETS})

add_executable(periodic_pid_attitude_controller_node src/periodic_pid_attitude_controller_node.cpp)
target_link_libraries(periodic_pid_attitude_controller_node periodic_pid_attitude_controller
   ${catkin_LIBRARIES}
)

add_library(discrete_pid_attitude_controller src/uned_crazyflie_controller/discrete_pid_attitude_controller.cpp)
target_link_libraries(discrete_pid_attitude_controller ${catkin_LIBRARIES})
add_dependencies(discrete_pid_attitude_controller ${catkin_EXPORTED_TARGETS})

add_executable(discrete_pid_attitude_controller_node src/discrete_pid_attitude_controller_node.cpp)
target_link_libraries(discrete_pid_attitude_controller_node discrete_pid_attitude_controller
   ${catkin_LIBRARIES}
)

##################################
##     RATEMIXER CONTROLLER     ##
##################################
add_library(periodic_pid_ratemixer_controller src/uned_crazyflie_controller/periodic_pid_ratemixer_controller.cpp)
target_link_libraries(periodic_pid_ratemixer_controller ${catkin_LIBRARIES})
add_dependencies(periodic_pid_ratemixer_controller ${catkin_EXPORTED_TARGETS})

add_executable(periodic_pid_ratemixer_controller_node src/periodic_pid_ratemixer_controller_node.cpp)
target_link_libraries(periodic_pid_ratemixer_controller_node periodic_pid_ratemixer_controller
   ${catkin_LIBRARIES}
)

add_library(discrete_pid_ratemixer_controller src/uned_crazyflie_controller/discrete_pid_ratemixer_controller.cpp)
target_link_libraries(discrete_pid_ratemixer_controller ${catkin_LIBRARIES})
add_dependencies(discrete_pid_ratemixer_controller ${catkin_EXPORTED_TARGETS})

add_executable(discrete_pid_ratemixer_controller_node src/discrete_pid_ratemixer_controller_node.cpp)
target_link_libraries(discrete_pid_ratemixer_controller_node discrete_pid_ratemixer_controller
   ${catkin_LIBRARIES}
)
########################################################################################################################
#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS uned_crazyflie_trajectory_controller periodic_pid_position_controller discrete_pid_attitude_controller periodic_pid_attitude_controller discrete_pid_ratemixer_controller periodic_pid_ratemixer_controller
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS uned_crazyflie_trajectory_controller periodic_pid_position_controller_node discrete_pid_attitude_controller_node periodic_pid_attitude_controller_node discrete_pid_ratemixer_controller_node periodic_pid_ratemixer_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
