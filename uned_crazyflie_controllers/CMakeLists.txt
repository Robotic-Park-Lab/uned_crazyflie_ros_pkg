cmake_minimum_required(VERSION 3.8)
project(uned_crazyflie_controllers)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3)
find_package(rosidl_default_generators REQUIRED)
# find_package(mav_msgs REQUIRED)
# find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/AttitudeRefs.msg")

ament_export_include_directories(include/${PROJECT_NAME} ${Eigen3_INCLUDE_DIRS})
ament_export_dependencies(rclcpp std_msgs rclpy sensor_msgs geometry_msgs Eigen3)
include_directories(
  include/${PROJECT_NAME}
  ${${PROJECT_NAME}_SOURCE_DIR}/include
)

add_executable(uned_crazyflie_position_controller src/uned_crazyflie_position_controller.cpp)
ament_target_dependencies(uned_crazyflie_position_controller
  rclcpp std_msgs rclpy sensor_msgs geometry_msgs Eigen3)

install(TARGETS uned_crazyflie_position_controller
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include)

ament_package()
