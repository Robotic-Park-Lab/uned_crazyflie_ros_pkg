#include <cstdio>
#include <chrono>
#include <array>
#include <cstring>
#include <iostream>
#include <fstream>
#include <time.h>
#include <chrono>
#include <functional>
#include <vector>
#include <typeinfo>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class TrajectoryController : public rclcpp::Node
{
  public:
    TrajectoryController() : Node("trajectory_controller"), count_(0)
    {}
    bool initialize();
    bool iterate();

  private:
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr ref_pose_;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr GT_pose_;

    std::string robotid;
    bool fail = false;
    bool debug_flag = false;
    geometry_msgs::msg::Pose last_pose, GT_pose, ref_pose;
    bool first_pose_received = false;
    bool new_ref = true;
    size_t count_;
    double start = 0.0;
    double end = 0.0;
    double t = 0.0;
    std::vector<std::vector<double>> trayectory;
    std::vector<double> file_pose;

    bool readFile(std::string name);
    void gtposeCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
};
