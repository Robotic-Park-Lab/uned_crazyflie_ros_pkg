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
#include <geometry_msgs/msg/quaternion.hpp>
#include <uned_crazyflie_config/msg/cmdsignal.hpp>
#include <uned_crazyflie_config/msg/pidcontroller.hpp>
#include <uned_crazyflie_config/msg/actuators.hpp>

struct pid_s{
    double kp, ki, kd, td;
    int nd;
    double error[2], integral, derivative, upperlimit, lowerlimit;
};
struct euler_angles {
    double roll, pitch, yaw;
};
struct threshold {
  double co, ai, cn;
  double dt;
  double noise[20] = {};
  double last_signal = 0.0;
  std::chrono::steady_clock::time_point last_time;
};

using namespace std::chrono_literals;

class AttitudeRateController : public rclcpp::Node
{
public:
    AttitudeRateController() : Node("attituderate_controller") {
      this->declare_parameter("ROBOT_ID", "dron01");
      this->declare_parameter("DEBUG", false);
    }

  bool initialize();
  bool iterate();

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_act_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr GT_pose_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_cmd_;

  std::string  m_controller_type, robotid, m_controller_mode;
  geometry_msgs::msg::Pose GT_pose;
  uned_crazyflie_config::msg::Cmdsignal ref_cmd;
  bool first_pose_received = false;
  bool first_ref_received = false;
  bool debug_flag = false;
  double dt = 0.002;
  double kp, ki, kd, td;
  // Controllers
  struct pid_s pitch_controller, roll_controller, yaw_controller, dpitch_controller, droll_controller, dyaw_controller;
  // Control Signals
  double dpitch_ref, droll_ref, dyaw_ref, delta_pitch, delta_roll, delta_yaw;
  double dpitch_signal, droll_signal, dyaw_signal;
  double dpitch_feedback[2], droll_feedback[2], dyaw_feedback[2];
  double motors[4];
  // Angles
  struct euler_angles rpy_ref, rpy_state;

  // Function
  void gtposeCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
  void refcmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  euler_angles quaternion2euler(geometry_msgs::msg::Quaternion quat);
  double pid_controller(struct pid_s &controller, double dt);
  struct pid_s init_controller(const char id[], double kp, double ki, double kd, double td, int nd, double upperlimit, double lowerlimit);
};
