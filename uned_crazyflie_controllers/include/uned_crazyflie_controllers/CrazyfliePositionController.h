#pragma once

#include <ros/ros.h>
#include <array>
#include <cstring>
#include <iostream>
#include <Eigen/Eigen>
#include <ros/console.h>
#include <mav_msgs/default_topics.h>
#include <time.h>
#include <chrono>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <mav_msgs/Actuators.h>
#include <uned_crazyflie_controllers/AttitudeRefs.h>

class CrazyfliePositionController
{
   public:
	CrazyfliePositionController() {}

	/**
	 * NodeHandle is the main access point to communications with the ROS
	 * system. The first NodeHandle constructed will fully initialize this node,
	 * and the last NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle m_nh{};
	ros::NodeHandle m_nh_params{"~"};

	ros::Publisher m_pub_control_signal, m_pub_omega, m_pub_dyaw;

	ros::Subscriber m_sub_GT_pose, m_sub_pos_ref;

	bool initialize();

	bool iterate();

   protected:
    void gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void rotorvelocitiesCallback(const Eigen::Vector4d rotor_velocities);
    void attitudeRateMixerRefsCallback(const double omega, const double pitch, const double roll, const double dyaw);
    void positionreferenceCallback(const geometry_msgs::Pose::ConstPtr& msg);
    struct pid_s{
        double kp, ki, kd, td;
        int nd;
        double error[2], integral, derivative[2], upperlimit, lowerlimit;
    };
    struct euler_angles {
        double roll, pitch, yaw;
    };
    euler_angles quaternion2euler(geometry_msgs::Quaternion quat) {
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
    struct pid_s init_controller(const char id[], double kp, double ki, double kd, double td, int nd, double upperlimit, double lowerlimit){
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

        ROS_INFO("%s Controller: kp: %0.2f \tki: %0.2f \tkd: %0.2f", id, controller.kp, controller.ki, controller.kd);
        return controller;
  }

    double dt = 0.01;
    double m_x_init, m_y_init, m_z_init;
    std::string m_controller_type, m_robot_id, m_controller_mode, str_id;
    geometry_msgs::Pose GT_pose, ref_pose;
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
};
