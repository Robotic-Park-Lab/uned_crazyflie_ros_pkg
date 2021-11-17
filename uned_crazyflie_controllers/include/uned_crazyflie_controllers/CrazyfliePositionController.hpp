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
      double integral[2];
      double derivative[2];
  };

  double m_x_init, m_y_init, m_z_init;
  std::string  m_controller_type, m_robot_id, m_controller_mode;
  geometry_msgs::msg::Pose GT_pose, ref_pose;
  bool first_pose_received = false;
  bool first_ref_received = false;
  // Controllers
  double Z_q[3], X_q[3], Y_q[3], U_q[3], V_q[3], Yaw_q[3];
  struct pid_s z_controller;
  struct pid_s w_controller;
  struct pid_s x_controller;
  struct pid_s u_controller;
  struct pid_s y_controller;
  struct pid_s v_controller;
  struct pid_s yaw_controller;
  // Altitude paremeters
  double z_error_signal[2], delta_omega[2], z_iterm[2], z_derterm[2];
  double omega = 0.0;
  double we = 14480.0;
  // X-Y paremeters
  double x_error_signal[3], y_error_signal[3], uc[2], vc[2], u_feedback[2], v_feedback[2];
  double u_error_signal[3], v_error_signal[3], pitch_ref[2], roll_ref[2];
  // Yaw Controller
  double yaw_error_signal[3], dyaw_ref[2];

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
    first_ref_received = true;
  }
  double pid_controller(double dt){
      RCLCPP_INFO(this->get_logger(),"PositionController::pid_controller() ok.");
      /*
      double td = controller[2]/controller[0];
      double outP = controller[0] * error[0];
      integral_term[0] = integral_term[0] + controller[1] * error[1] * dt;
      derivative_term[0] = (td/(td+Nd+dt))*derivative_term[1]+(controller[2]*Nd/(td+Nd*dt))*(error[0]-error[1]);
      double out = outP + integral_term[0] + derivative_term[0];
      */
      return dt;
  }
  void init_controller(char id[], struct pid_s controller, double kp, double ki, double kd, double td, int nd){
      controller.kp = kp;
      controller.ki = ki;
      controller.kd = kd;
      controller.td = td;
      controller.nd = nd;
      controller.error[0] = 0.0;
      controller.error[1] = 0.0;
      controller.integral[0] = 0.0;
      controller.integral[1] = 0.0;
      controller.derivative[0] = 0.0;
      controller.derivative[1] = 0.0;

      RCLCPP_INFO(this->get_logger(),"%s Controller: kp: %0.2f \tki: %0.2f \tkd: %0.2f", id, controller.kp, controller.ki, controller.kd);
  }
};
