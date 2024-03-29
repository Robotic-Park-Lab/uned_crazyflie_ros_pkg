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
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(uned_crazyflie_config REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3)

ament_export_include_directories(include/${PROJECT_NAME} ${Eigen3_INCLUDE_DIRS})
ament_export_dependencies(rclcpp std_msgs rclpy sensor_msgs geometry_msgs Eigen3 uned_crazyflie_config)
include_directories(
    include/${PROJECT_NAME}
    ${${PROJECT_NAME}_SOURCE_DIR}/include
)

add_executable(eventbased_pid_position_controller src/eventbased_pid_position_controller.cpp)
ament_target_dependencies(eventbased_pid_position_controller
    rclcpp std_msgs sensor_msgs geometry_msgs Eigen3 uned_crazyflie_config)

add_executable(generalized_predictive_controller src/generalized_predictive_controller.cpp)
ament_target_dependencies(generalized_predictive_controller
    rclcpp std_msgs sensor_msgs geometry_msgs Eigen3 uned_crazyflie_config)

add_executable(periodic_pid_position_controller src/periodic_pid_position_controller.cpp)
ament_target_dependencies(periodic_pid_position_controller
    rclcpp std_msgs sensor_msgs geometry_msgs Eigen3 uned_crazyflie_config)

add_executable(periodic_pid_attituderate_controller src/periodic_pid_attituderate_controller.cpp)
ament_target_dependencies(periodic_pid_attituderate_controller
    rclcpp std_msgs sensor_msgs geometry_msgs Eigen3 uned_crazyflie_config)

add_executable(trajectory_controller src/trajectory_controller.cpp)
ament_target_dependencies(trajectory_controller
    rclcpp std_msgs sensor_msgs geometry_msgs Eigen3 uned_crazyflie_config)

install(TARGETS eventbased_pid_position_controller periodic_pid_position_controller trajectory_controller periodic_pid_attituderate_controller generalized_predictive_controller
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include)

ament_package()
