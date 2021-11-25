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

using namespace std::chrono_literals;

class AttitudeRateController : public rclcpp::Node
{
public:
    AttitudeRateController() : Node("attituderate_controller") {}

  bool initialize();
  bool iterate();

private:
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_act_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr GT_pose_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_cmd_;

  struct pid_s{
      double kp, ki, kd, td;
      int nd;
      double error[2], integral, derivative[2], upperlimit, lowerlimit;
  };
  struct euler_angles{
      double roll, pitch, yaw;
  };

  std::string  m_controller_type, m_robot_id, m_controller_mode, str_id;
  geometry_msgs::msg::Pose GT_pose;
  uned_crazyflie_config::msg::Cmdsignal ref_cmd;
  bool first_pose_received = false;
  bool first_ref_received = false;
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
  void gtposeCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
      GT_pose.position = msg->position;
      GT_pose.orientation = msg->orientation;
      if (!first_pose_received)
          first_pose_received = true;
  }
  void refcmdCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
      ref_cmd.thrust = msg->data[0];
      ref_cmd.roll = msg->data[1];
      ref_cmd.pitch = msg->data[2];
      ref_cmd.yaw = msg->data[3];
      if (!first_ref_received)
          auto msg_cmd = std_msgs::msg::Bool();
          //msg_cmd.data = true;
          // pub_act_->publish(msg_cmd);
          first_ref_received = true;
  }
  euler_angles quaternion2euler(geometry_msgs::msg::Quaternion quat){
      euler_angles rpy;

      // roll (x-axis rotation)
      double sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z);
      double cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y);
      rpy.roll = std::atan2(sinr_cosp, cosr_cosp) * (180 / 3.14159265);

      // pitch (y-axis rotation)
      double sinp = 2 * (quat.w * quat.y - quat.z * quat.x);
      if (std::abs(sinp) >= 1)
          rpy.pitch = std::copysign(3.14159265 / 2, sinp) * (180 / 3.14159265);
      else
          rpy.pitch = std::asin(sinp) * (180 / 3.14159265);

      // yaw (z-axis rotation)
      double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
      double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
      rpy.yaw = std::atan2(siny_cosp, cosy_cosp) * (180 / 3.14159265);

      return rpy;
  }
  double pid_controller(struct pid_s controller, double dt){
      double outP = controller.kp * controller.error[0];
      controller.integral = controller.integral + controller.ki * controller.error[1] * dt;
      controller.derivative[0] = (controller.td/(controller.td+controller.nd+dt))*controller.derivative[1]+(controller.kd*controller.nd/(controller.td+controller.nd*dt))*(controller.error[0]-controller.error[1]);
      double out = outP + controller.integral + controller.derivative[0];

      double out_i = out;

      if (out > controller.upperlimit)
          out = controller.upperlimit;
      if (out < controller.lowerlimit)
          out = controller.lowerlimit;

      controller.integral = controller.integral - (out - out_i) * sqrt(controller.kp / controller.ki);

      controller.error[1] = controller.error[0];
      controller.derivative[1] = controller.derivative[0];

      return out;
  }
  struct pid_s init_controller(const char id[], double kp, double ki, double kd, double td, int nd, double upperlimit, double lowerlimit) {
      struct pid_s controller;

      controller.kp = kp;
      controller.ki = ki;
      controller.kd = kd;
      controller.td = td;
      controller.nd = nd;
      controller.error[0] = 0.0;
      controller.error[1] = 0.0;
      controller.integral = 0.0;
      controller.derivative[0] = 0.0;
      controller.derivative[1] = 0.0;
      controller.upperlimit = upperlimit;
      controller.lowerlimit = lowerlimit;

      RCLCPP_INFO(this->get_logger(), "%s Controller: kp: %0.2f \tki: %0.2f \tkd: %0.2f", id, controller.kp, controller.ki, controller.kd);
      return controller;
  }
};
