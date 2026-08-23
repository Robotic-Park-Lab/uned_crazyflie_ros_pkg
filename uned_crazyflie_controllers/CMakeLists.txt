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

install(TARGETS eventbased_pid_position_controller periodic_pid_position_controller periodic_pid_attituderate_controller generalized_predictive_controller
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include)

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    # Each test compiles the real controller .cpp (not a copy of its PID
    # logic) with UNED_CRAZYFLIE_CONTROLLERS_TEST_BUILD defined, which
    # #ifndef-guards that file's own main() out of the build so it can be
    # linked into a gtest binary instead. periodic/eventbased position
    # controllers share the same PositionController class (same header),
    # so they must stay in separate test executables -- linking both .cpp
    # files together would collide on duplicate symbol definitions.

    ament_add_gtest(test_periodic_pid_position_controller
        test/test_periodic_pid_position_controller.cpp
        src/periodic_pid_position_controller.cpp)
    target_compile_definitions(test_periodic_pid_position_controller
        PRIVATE UNED_CRAZYFLIE_CONTROLLERS_TEST_BUILD)
    ament_target_dependencies(test_periodic_pid_position_controller
        rclcpp std_msgs sensor_msgs geometry_msgs Eigen3 uned_crazyflie_config)

    ament_add_gtest(test_eventbased_pid_position_controller
        test/test_eventbased_pid_position_controller.cpp
        src/eventbased_pid_position_controller.cpp)
    target_compile_definitions(test_eventbased_pid_position_controller
        PRIVATE UNED_CRAZYFLIE_CONTROLLERS_TEST_BUILD)
    ament_target_dependencies(test_eventbased_pid_position_controller
        rclcpp std_msgs sensor_msgs geometry_msgs Eigen3 uned_crazyflie_config)

    ament_add_gtest(test_attituderate_controller
        test/test_attituderate_controller.cpp
        src/periodic_pid_attituderate_controller.cpp)
    target_compile_definitions(test_attituderate_controller
        PRIVATE UNED_CRAZYFLIE_CONTROLLERS_TEST_BUILD)
    ament_target_dependencies(test_attituderate_controller
        rclcpp std_msgs sensor_msgs geometry_msgs Eigen3 uned_crazyflie_config)
endif()

ament_package()
