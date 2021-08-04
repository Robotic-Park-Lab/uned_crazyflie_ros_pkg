#include <cstdio>
#include <chrono>
#include <array>
#include <cstring>
#include <iostream>
#include <time.h>
#include <chrono>
#include <functional>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>

using namespace std::chrono_literals;

class EventTriggering : public rclcpp::Node
{
public:
  EventTriggering() : Node("event_triggering") {}

  bool initialize();
  bool iterate();

private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr event_;


  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr GT_pose_;
  void gtposeCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
    GT_pose.position = msg->position;
  	GT_pose.orientation = msg->orientation;
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ref_pose_;
  void positionreferenceCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(),"New Pose: x: %f \ty: %f \tz: %f", ref_pose.position.x, ref_pose.position.y, ref_pose.position.z);
    ref_pose.position = msg->position;
  	ref_pose.orientation = msg->orientation;
  }

  geometry_msgs::msg::Pose GT_pose, ref_pose;
};
