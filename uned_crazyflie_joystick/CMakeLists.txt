cmake_minimum_required(VERSION 3.0.2)
project(uned_crazyflie_joystick)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs genmsg sensor_msgs geometry_msgs)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(FILES Num.msg)


## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES uned_crazyflie_joystick
   CATKIN_DEPENDS roscpp std_msgs
   DEPENDS sensor_msgs geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(joystick_node src/joystick_node.cpp)
target_link_libraries(joystick_node ${catkin_LIBRARIES})
add_dependencies(joystick_node uned_crazyflie_joystick_generate_messages_cpp)
