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

struct pid_s{
    double kp, ki, kd, td;
    int nd;
    double error[2], integral, derivative[2], upperlimit, lowerlimit;
};
struct euler_angles {
    double roll, pitch, yaw;
};

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
    euler_angles quaternion2euler(geometry_msgs::Quaternion quat);
    double pid_controller(struct pid_s controller, double dt);
    struct pid_s init_controller(const char id[], double kp, double ki, double kd, double td, int nd, double upperlimit, double lowerlimit);

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

    // Controllers
    double Z_q[3], X_q[3], Y_q[3], U_q[3], V_q[3];
    // Altitude paremeters
    double z_error_signal[3], delta_omega[2];
    double omega = 0.0;
    double we = 16073.0; //14480.0;
    // X-Y paremeters
    double x_error_signal[3], y_error_signal[3], uc[2], vc[2];
    double u_error_signal[3], v_error_signal[3], pitch_ref[2], roll_ref[2];
    double yaw;
};
