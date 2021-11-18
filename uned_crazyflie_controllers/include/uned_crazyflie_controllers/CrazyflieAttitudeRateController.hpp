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

class AttitudeRateController : public rclcpp::Node
{
public:
    AttitudeRateController() : Node("attituderate_controller") {}

  bool initialize();
  bool iterate();

private:
  // rclcpp::Publisher<uned_crazyflie_config::msg::Cmdsignal>::SharedPtr pub_cmd_;

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr GT_pose_;

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

  std::string  m_controller_type, m_robot_id, m_controller_mode;
  // Controllers
  struct pid_s pitch_controller;

  // Function
  double pid_controller(struct pid_s controller, double dt){
      double outP = controller.kp * controller.error[0];
      controller.integral = controller.integral + controller.ki * controller.error[1] * dt;
      controller.derivative[0] = (controller.td/(controller.td+controller.nd+dt))*controller.derivative[1]+(controller.kd*controller.nd/(controller.td+controller.nd*dt))*(controller.error[0]-controller.error[1]);
      double out = outP + controller.integral + controller.derivative[0];

      if (out > controller.upperlimit)
          out = controller.upperlimit;
      if (out < controller.lowerlimit)
          out = controller.lowerlimit;

      // TO-DO: Antiwindup!!

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
