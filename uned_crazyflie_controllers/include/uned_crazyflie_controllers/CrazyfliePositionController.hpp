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
    double error[2], integral, derivative[2], upperlimit, lowerlimit;
};
struct euler_angles {
    double roll, pitch, yaw;
};

using namespace std::chrono_literals;

class PositionController : public rclcpp::Node
{
public:
    PositionController() : Node("position_controller") {
      this->declare_parameter("ZKp");
      this->declare_parameter("ZKi");
      this->declare_parameter("ZKd");
      this->declare_parameter("ZTd");
      this->declare_parameter("WKp");
      this->declare_parameter("WKi");
      this->declare_parameter("WKd");
      this->declare_parameter("WTd");
      this->declare_parameter("XKp");
      this->declare_parameter("XKi");
      this->declare_parameter("XKd");
      this->declare_parameter("XTd");
      this->declare_parameter("UKp");
      this->declare_parameter("UKi");
      this->declare_parameter("UKd");
      this->declare_parameter("UTd");
      this->declare_parameter("YKp");
      this->declare_parameter("YKi");
      this->declare_parameter("YKd");
      this->declare_parameter("YTd");
      this->declare_parameter("VKp");
      this->declare_parameter("VKi");
      this->declare_parameter("VKd");
      this->declare_parameter("VTd");
      this->declare_parameter("ROBOT_ID");
    }

    bool initialize();
    bool iterate();

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_cmd_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_omega_;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr GT_pose_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr ref_pose_;

    std::string robotid;
    double Kp, Ki, Kd, Td;
    double dt = 0.01;
    double m_x_init, m_y_init, m_z_init;
    std::string  m_controller_type, m_robot_id, m_controller_mode, str_id;
    geometry_msgs::msg::Pose GT_pose, ref_pose;
    bool first_pose_received = false;
    bool first_ref_received = false;
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
    void gtposeCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
        GT_pose.position = msg->position;
        GT_pose.orientation = msg->orientation;
        if(!first_pose_received){
            ref_pose.position = msg->position;
            ref_pose.orientation = msg->orientation;
            first_pose_received = true;
            u_feedback[0] = ref_pose.position.x;
            v_feedback[0] = ref_pose.position.y;
            w_feedback[0] = ref_pose.position.z;
            RCLCPP_INFO(this->get_logger(),"Init Pose: x: %f \ty: %f \tz: %f", ref_pose.position.x, ref_pose.position.y, ref_pose.position.z);
        }
    }
    void positionreferenceCallback(const geometry_msgs::msg::Pose::SharedPtr msg){
        // RCLCPP_INFO(this->get_logger(),"New Pose: x: %f \ty: %f \tz: %f", ref_pose.position.x, ref_pose.position.y, ref_pose.position.z);
        ref_pose.position = msg->position;
        ref_pose.orientation = msg->orientation;
        if (!first_ref_received) {
            first_ref_received = true;
        }
    }
    euler_angles quaternion2euler(geometry_msgs::msg::Quaternion quat);
    double pid_controller(struct pid_s controller, double dt);
    struct pid_s init_controller(const char id[], double kp, double ki, double kd, double td, int nd, double upperlimit, double lowerlimit);
};
