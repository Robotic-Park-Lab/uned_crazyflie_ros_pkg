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
#include <uned_crazyflie_config/msg/cmdsignal.hpp>

using namespace std::chrono_literals;

class PositionController : public rclcpp::Node
{
public:
  PositionController() : Node("position_controller") {}

  bool initialize();
  bool iterate();

private:
  rclcpp::Publisher<uned_crazyflie_config::msg::Cmdsignal>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_omega_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr GT_pose_;
  void gtposeCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
    GT_pose.position = msg->position;
    GT_pose.orientation = msg->orientation;
    if(!first_pose_received){
        ref_pose.position = msg->position;
        ref_pose.orientation = msg->orientation;
        first_pose_received = true;
        RCLCPP_INFO(this->get_logger(),"Init Pose: x: %f \ty: %f \tz: %f", ref_pose.position.x, ref_pose.position.y, ref_pose.position.z);
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ref_pose_;
  void positionreferenceCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(),"New Pose: x: %f \ty: %f \tz: %f", ref_pose.position.x, ref_pose.position.y, ref_pose.position.z);
    ref_pose.position = msg->position;
    ref_pose.orientation = msg->orientation;
  }

  void rotorvelocitiesCallback(const Eigen::Vector4d rotor_velocities);

  double m_x_init, m_y_init, m_z_init;
  std::string  m_controller_type, m_robot_id, m_controller_mode;
  geometry_msgs::msg::Pose GT_pose, ref_pose;
  bool first_pose_received = false;

  // Controllers
  double Z_q[3], X_q[3], Y_q[3], U_q[3], V_q[3], Yaw_q[3];
  // Altitude paremeters
  double z_error_signal[3], delta_omega[2];
  double omega = 0.0;
  double we = 14480.0;
  // X-Y paremeters
  double x_error_signal[3], y_error_signal[3], uc[2], vc[2], u_feedback[2], v_feedback[2];
  double u_error_signal[3], v_error_signal[3], pitch_ref[2], roll_ref[2];
  // Yaw Controller
  double yaw_error_signal[3], dyaw_ref[2];
};
