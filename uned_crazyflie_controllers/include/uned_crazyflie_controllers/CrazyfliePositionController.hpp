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
        double kp, ki, kd, td;
        int nd;
        double error[2], integral, derivative[2], upperlimit, lowerlimit;
    };
    struct euler_angles {
        double roll, pitch, yaw;
    };

    double dt = 0.01;
    double m_x_init, m_y_init, m_z_init;
    std::string  m_controller_type, m_robot_id, m_controller_mode;
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
        RCLCPP_INFO(this->get_logger(),"New Pose: x: %f \ty: %f \tz: %f", ref_pose.position.x, ref_pose.position.y, ref_pose.position.z);
        ref_pose.position = msg->position;
        ref_pose.orientation = msg->orientation;
        if (!first_ref_received) {
            first_ref_received = true;
        }
    }
    euler_angles quaternion2euler(geometry_msgs::msg::Quaternion quat) {
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
