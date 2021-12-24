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
struct threshold {
  double co, ai, cn;
  double dt;
  double noise[20] = {};
  double last_signal = 0.0;
  std::chrono::steady_clock::time_point last_time;
};

using namespace std::chrono_literals;

class PositionController : public rclcpp::Node
{
public:
    PositionController() : Node("position_controller") {
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
      this->declare_parameter("DEBUG", false);
      this->declare_parameter("Zco", 0.);
      this->declare_parameter("Zai", 0.);
      this->declare_parameter("Wco", 0.);
      this->declare_parameter("Wai", 0.);
      this->declare_parameter("Xco", 0.);
      this->declare_parameter("Xai", 0.);
      this->declare_parameter("Uco", 0.);
      this->declare_parameter("Uai", 0.);
      this->declare_parameter("Yco", 0.);
      this->declare_parameter("Yai", 0.);
      this->declare_parameter("Vco", 0.);
      this->declare_parameter("Vai", 0.);
    }

    bool initialize();
    bool iterate();

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_z_event_, pub_w_event_;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr GT_pose_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ref_pose_;

    std::string robotid, feedback_topic, m_controller_type, m_controller_mode;
    // Params
    double Kp, Ki, Kd, Td, Co, Ai, Cn;
    double dt = 0.01;
    geometry_msgs::msg::Pose GT_pose, ref_pose;
    bool first_pose_received = false;
    bool first_ref_received = false;
    bool fail = false;
    bool debug_flag = false;

    // Controllers
    struct pid_s z_controller, w_controller, x_controller, u_controller, y_controller, v_controller;
    // Altitude Controller
    double w_feedback[2] = {};
    double w_signal, w_ref, thrust;
    // X-Y paremeters
    double x_global_error, y_global_error, u_ref, v_ref, u_feedback[2], v_feedback[2];
    double u_signal, v_signal, pitch, roll;
    // Angles
    struct euler_angles rpy_ref, rpy_state;

    // Event Based Control
    bool events = false;
    bool z_event, w_event;
    struct threshold z_threshold, w_threshold, x_threshold, u_threshold, y_threshold, v_threshold;
    // Function
    void gtposeCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void positionreferenceCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    euler_angles quaternion2euler(geometry_msgs::msg::Quaternion quat);
    double pid_controller(struct pid_s &controller, double dt);
    struct pid_s init_controller(const char id[], double kp, double ki, double kd, double td, int nd, double upperlimit, double lowerlimit);
    struct threshold init_triggering(const char id[], double co, double a);
    bool eval_threshold(struct threshold &trigger, double signal, double ref);
};
