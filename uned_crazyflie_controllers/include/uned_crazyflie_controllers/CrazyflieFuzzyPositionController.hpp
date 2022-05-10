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
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <uned_crazyflie_config/msg/cmdsignal.hpp>
#include <uned_crazyflie_config/msg/pidcontroller.hpp>

#include "flc/fuzzyController.h"
// #include <fstream>
// #include <vector>
// #include <typeinfo>

#define NUMBER_OF_INPUTS    2
#define NUMBER_OF_RULES     9
#define NUMBER_OF_INPUT_MF  3
#define NUMBER_OF_OUTPUT_MF 6

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

struct fpid_s{
    pFuzzyController_t pFuzzyController;
    double ke, kd, k0, k1;
    int nd;
    double error[2], integral, derivative, upperlimit, lowerlimit;
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
      this->declare_parameter("XKe", 0.);
      this->declare_parameter("XKd", 0.);
      this->declare_parameter("XK0", 0.);
      this->declare_parameter("XK1", 0.);
      this->declare_parameter("YKe", 0.);
      this->declare_parameter("YKd", 0.);
      this->declare_parameter("YK0", 0.);
      this->declare_parameter("YK1", 0.);
      this->declare_parameter("ROBOT_ID", "dron01");
      this->declare_parameter("Feedback_pose_topic", "cf_pose");
      this->declare_parameter("Feedback_twist_topic", "cf_twist");
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
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_, pub_zcon_, pub_xcon_, pub_ycon_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_z_event_, pub_w_event_;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr GT_pose_, ref_pose_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr GT_twist_;

    std::string robotid, feedback_pose_topic, feedback_twist_topic, m_controller_type, m_controller_mode;
    // Params
    double Kp, Ki, Kd, Td, Ke, K0, K1, Co, Ai, Cn;
    double dt = 0.01;
    geometry_msgs::msg::Pose GT_pose, ref_pose;
    geometry_msgs::msg::Twist GT_twist;
    bool first_pose_received = false;
    bool first_ref_received = false;
    bool fail = false;
    bool debug_flag = false;

    // Controllers
    struct pid_s z_controller, w_controller;
    struct fpid_s x_controller, y_controller;
    
    // Altitude Controller
    double w_feedback[2] = {};
    double w_signal, w_ref, thrust;
    // X-Y paremeters
    double x_global_error, y_global_error, u_ref, v_ref, u_feedback[2], v_feedback[2];
    double u_global_error, v_global_error, u_signal, v_signal, pitch, roll;
    // Angles
    struct euler_angles rpy_ref, rpy_state;

    // x Fuzzy Logic Controller
    FUZZY_CONTROLLER_HADLER_INIT(fuzzyControllerX, NUMBER_OF_INPUTS, NUMBER_OF_RULES);
    FUZZY_CONTROLLER_DEFINE_INPUT(fuzzyControllerX, errX, NUMBER_OF_INPUT_MF);
    FUZZY_CONTROLLER_DEFINE_INPUT(fuzzyControllerX, difErrX, NUMBER_OF_INPUT_MF);
    FUZZY_CONTROLLER_DEFINE_OUTPUT(fuzzyControllerX, udX, NUMBER_OF_OUTPUT_MF);

    // y Fuzzy Logic Controller
    FUZZY_CONTROLLER_HADLER_INIT(fuzzyControllerY, NUMBER_OF_INPUTS, NUMBER_OF_RULES);
    FUZZY_CONTROLLER_DEFINE_INPUT(fuzzyControllerY, errY, NUMBER_OF_INPUT_MF);
    FUZZY_CONTROLLER_DEFINE_INPUT(fuzzyControllerY, difErrY, NUMBER_OF_INPUT_MF);
    FUZZY_CONTROLLER_DEFINE_OUTPUT(fuzzyControllerY, udY, NUMBER_OF_OUTPUT_MF);

    
    // GPC
    // std::vector<std::vector<double>> gpc_trayectory, w1;
    // std::vector<double> ref_gpc_pose;
    // int N = 100; //Horizonte de predicción
    // double du = 0.0;
    // double u = 0.0;
    // double Gp[10] = {0.0285, 0.0594, 0.0904, 0.1215, 0.1525, 0.1836, 0.2146, 0.2457, 0.2767, 0.3078};
    // double f[10] = {0.0};
    // double Fp[10][3] = {{2.0821, -1.1642, 0.0821},{3.1709, -2.3418, 0.1709},{4.2603, -3.5206, 0.2603},{5.3497, -4.6994, 0.3497},{6.4391, -5.8783, 0.4391},{7.5286, -7.0571, 0.5286},{8.6180, -8.2360, 0.6180},{9.7074, -9.4148, 0.7074},{10.7968, -10.5937, 0.7968},{11.8863, -11.7725, 0.8863}};
    // double yaux[3] = {0.0};
    // double K[10] = {1.5892, 2.0468, 1.2086, 0.3940, -0.0124, -0.1114, -0.0821, -0.0340, -0.0026, 0.0151};

    // Function
    void gtposeCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    void gtTwistCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void positionreferenceCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    euler_angles quaternion2euler(geometry_msgs::msg::Quaternion quat);
    double pid_controller(struct pid_s &controller, double dt);
    struct pid_s init_controller(const char id[], double kp, double ki, double kd, double td, int nd, double upperlimit, double lowerlimit);
    struct threshold init_triggering(const char id[], double co, double a);
    bool eval_threshold(struct threshold &trigger, double signal, double ref);
    bool readFile(std::string name);
    struct fpid_s init_fpid(const char id[], pFuzzyController_t controller, double ke, double kd, double k0, double k1, int nd, double upperlimit, double lowerlimit);
    double fpid_controller(struct fpid_s &controller, double dt);
};
