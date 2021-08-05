#include <cstdio>
#include <chrono>
#include <cmath>
#include <array>
#include <cstring>
#include <iostream>
#include <time.h>
#include <chrono>
#include <functional>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <uned_crazyflie_config/msg/triggering.hpp>

using namespace std::chrono_literals;

class EventTriggering : public rclcpp::Node
{
public:
  EventTriggering() : Node("event_triggering") {}

  bool initialize();
  bool iterate();

private:
  rclcpp::Publisher<uned_crazyflie_config::msg::Triggering>::SharedPtr events_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr GT_pose_;
  void gtposeCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
    GT_pose.position = msg->position;
  	GT_pose.orientation = msg->orientation;
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ref_pose_;
  void positionreferenceCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
    ref_pose.position = msg->position;
  	ref_pose.orientation = msg->orientation;
  }

  geometry_msgs::msg::Pose GT_pose, ref_pose;
  // Umbral
  double fthreshold[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  // Cota
  double threshold[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  // Error tolerable en estado estacionario
  double c0[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  // Factor proporcional al error de la señal
  double a[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  // Error
  double error[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  // Ajuste al nivel de ruido
  double cn[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
  // double noiseEstimation(){}
  bool events[5] = {false, false, false, false, false};

  double x_error, y_error;
};
