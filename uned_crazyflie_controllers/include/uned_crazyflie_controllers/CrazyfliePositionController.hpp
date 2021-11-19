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
#include <uned_crazyflie_config/msg/pidcontroller.hpp>

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
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ref_pose_;

  struct pid_s{
      double kp;
      double ki;
      double kd;
      double td;
      int nd;
      double error[2];
      double integral;
      double derivative[2];
      double upperlimit;
      double lowerlimit;
  };

  double m_x_init, m_y_init, m_z_init;
  std::string  m_controller_type, m_robot_id, m_controller_mode;
  geometry_msgs::msg::Pose GT_pose, ref_pose;
  bool first_pose_received = false;
  bool first_ref_received = false;
  // Controllers
  double Z_q[3], X_q[3], Y_q[3], U_q[3], V_q[3], Yaw_q[3];
  struct pid_s z_controller, w_controller, x_controller, u_controller, y_controller, v_controller, yaw_controller;
  // Altitude paremeters
  double z_error_signal[2], delta_omega[2], z_iterm[2], z_derterm[2];
  double omega = 0.0;
  double we = 14480.0;
  // X-Y paremeters
  double uc[2], vc[2], u_feedback[2], v_feedback[2];
  double pitch_ref[2], roll_ref[2];
  // Yaw Controller
  double dyaw_ref[2];

  // Function
  void gtposeCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
    GT_pose.position = msg->position;
    GT_pose.orientation = msg->orientation;
    if(!first_pose_received){
        ref_pose.position = msg->position;
        ref_pose.orientation = msg->orientation;
        first_pose_received = true;
        u_feedback[0] = ref_pose.position.x;
        v_feedback[0] = ref_pose.position.y;
        RCLCPP_INFO(this->get_logger(),"Init Pose: x: %f \ty: %f \tz: %f", ref_pose.position.x, ref_pose.position.y, ref_pose.position.z);
    }
  }
  void positionreferenceCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
    RCLCPP_INFO(this->get_logger(),"New Pose: x: %f \ty: %f \tz: %f", ref_pose.position.x, ref_pose.position.y, ref_pose.position.z);
    ref_pose.position = msg->position;
    ref_pose.orientation = msg->orientation;
    if (!first_ref_received) {
        first_ref_received = true;
    }
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

      double es = out - out_i;

      controller.integral = controller.integral - es * controller.kp / controller.ki;

      controller.error[1] = controller.error[0];
      controller.derivative[1] = controller.derivative[0];

      return out;
  }
  void init_controller(char id[], struct pid_s controller, double kp, double ki, double kd, double td, int nd, double upperlimit, double lowerlimit){
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

      RCLCPP_INFO(this->get_logger(),"%s Controller: kp: %0.2f \tki: %0.2f \tkd: %0.2f", id, controller.kp, controller.ki, controller.kd);
  }
};
