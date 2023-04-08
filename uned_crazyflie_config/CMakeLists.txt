cmake_minimum_required(VERSION 3.8)
project(uned_crazyflie_config)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclpy REQUIRED)
find_package(urdf REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Triggering.msg"
  "msg/StateEstimate.msg"
  "msg/Cmdsignal.msg"
  "msg/Pidcontroller.msg"
  "msg/Actuators.msg"
  "srv/AddTwoInts.srv"
)
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

#############
## Install ##
#############

install(
  DIRECTORY
    launch
    model
    resources
    rviz
    rqt
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
