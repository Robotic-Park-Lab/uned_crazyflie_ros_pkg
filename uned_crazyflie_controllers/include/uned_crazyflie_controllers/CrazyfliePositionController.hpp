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

struct pid_s{
    double kp, ki, kd, td;
    int nd;
    double error[2], integral, derivative, upperlimit, lowerlimit;
};
struct euler_angles {
    double roll, pitch, yaw;
};

using namespace std::chrono_literals;

class PositionController : public rclcpp::Node
{
public:
    PositionController() : Node("position_controller") {
      // this->declare_parameter("ZKp");
      this->declare_parameter("ZKp", 0.);
      this->declare_parameter("ZKi", 0.);
      this->declare_parameter("ZKd", 0.);
      this->declare_parameter("ZTd", 0.);
      this->declare_parameter("WKp", 0.);
      this->declare_parameter("WKi", 0.);
      this->declare_parameter("WKd", 0.);
      this->declare_parameter("WTd", 0.);
      this->declare_parameter("XKp", 0.);
      this->declare_parameter("XKi", 0.);
      this->declare_parameter("XKd", 0.);
      this->declare_parameter("XTd", 0.);
      this->declare_parameter("UKp", 0.);
      this->declare_parameter("UKi", 0.);
      this->declare_parameter("UKd", 0.);
      this->declare_parameter("UTd", 0.);
      this->declare_parameter("YKp", 0.);
      this->declare_parameter("YKi", 0.);
      this->declare_parameter("YKd", 0.);
      this->declare_parameter("YTd", 0.);
      this->declare_parameter("VKp", 0.);
      this->declare_parameter("VKi", 0.);
      this->declare_parameter("VKd", 0.);
      this->declare_parameter("VTd", 0.);
      this->declare_parameter("ROBOT_ID", "dron01");
      this->declare_parameter("Feedback_topic", "cf_pose");
    }

    bool initialize();
    bool iterate();

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_omega_;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr GT_pose_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ref_pose_;

    std::string robotid, feedback_topic, m_controller_type, m_controller_mode;
    double Kp, Ki, Kd, Td;
    double dt = 0.01;
    geometry_msgs::msg::Pose GT_pose, ref_pose;
    bool first_pose_received = false;
    bool first_ref_received = false;
    bool fail = false;
    // Controllers
    struct pid_s z_controller, w_controller, x_controller, u_controller, y_controller, v_controller;
    // Altitude Controller
    double w_feedback[2], w_signal, w_ref, thrust;
    // X-Y paremeters
    double x_global_error, y_global_error, u_ref, v_ref, u_feedback[2], v_feedback[2];
    double u_signal, v_signal, pitch, roll;
    // Angles
    struct euler_angles rpy_ref, rpy_state;

    // Function
    void gtposeCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void positionreferenceCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    euler_angles quaternion2euler(geometry_msgs::msg::Quaternion quat);
    double pid_controller(struct pid_s controller, double dt);
    struct pid_s init_controller(const char id[], double kp, double ki, double kd, double td, int nd, double upperlimit, double lowerlimit);
};
