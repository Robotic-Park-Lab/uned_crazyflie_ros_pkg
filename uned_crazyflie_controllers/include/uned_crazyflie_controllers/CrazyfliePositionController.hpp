#include <cstdio>
#include <chrono>
#include <array>
#include <cstring>
#include <iostream>
#include <time.h>
#include <chrono>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose.hpp>
// #include <uned_crazyflie_controllers/msg/attitude_refs.hpp>

using namespace std::chrono_literals;

class PositionController : public rclcpp::Node
{
public:
  PositionController() : Node("position_controller") {}

  // ros::Publisher m_pub_control_signal, m_pub_omega, m_pub_dyaw;

  // ros::Subscriber m_sub_GT_pose, m_sub_pos_ref;

  bool initialize();
  bool iterate();

private:
  void gtposeCallback(const geometry_msgs::msg::Pose::SharedPtr& msg);
  void rotorvelocitiesCallback(const Eigen::Vector4d rotor_velocities);
  void attitudeRateMixerRefsCallback(const double omega, const double pitch, const double roll, const double dyaw);
  void positionreferenceCallback(const geometry_msgs::msg::Pose::SharedPtr& msg);

  double m_x_init, m_y_init, m_z_init;
  std_msgs::msg::String m_controller_type, m_robot_id, m_controller_mode;
  geometry_msgs::msg::Pose m_GT_pose, m_ref_pose, m_ref_position, m_ref_test;

  // Controllers
  double Z_q[3], X_q[3], Y_q[3], U_q[3], V_q[3], Yaw_q[3];
  // Altitude paremeters
  double z_error_signal[3], delta_omega[2];
  double omega = 0.0;
  double we = 16073.0; //14480.0;
  // X-Y paremeters
  double x_error_signal[3], y_error_signal[3], uc[2], vc[2], u_feedback[2], v_feedback[2];
  double u_error_signal[3], v_error_signal[3], pitch_ref[2], roll_ref[2];
  // Yaw Controller
  double yaw_error_signal[3], dyaw_ref[2];
};
