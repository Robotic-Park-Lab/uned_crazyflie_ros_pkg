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
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
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

	ros::Publisher m_pub_control_signal;

	ros::Subscriber m_sub_GT_pose, m_sub_pos_ref;

	bool initialize();

	bool iterate();

   protected:
    void gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void rotorvelocitiesCallback(const Eigen::Vector4d rotor_velocities);
    void attitudeRateMixerRefsCallback(const double omega, const double pitch, const double roll, const double dyaw);
    void positionreferenceCallback(const geometry_msgs::Pose::ConstPtr& msg);

    double m_x_init, m_y_init, m_z_init;
    std::string m_controller_type, m_robot_id, m_controller_mode;
    geometry_msgs::Pose m_GT_pose, m_ref_pose, m_ref_position;

    // Altitude paremeters
    double Z_T = 1/100;
    double z_error_signal[3] = {0.0, 0.0, 0.0};
    double Z_s[3] = {15000.0, 3500.0, 9000.0};
    //double Z_q[3] = {Z_s[0]+Z_s[1]*Z_T/2.0+Z_s[2]/Z_T, -Z_s[0]+Z_s[1]*Z_T/2.0-2.0*Z_s[2]/Z_T, Z_s[2]/Z_T};
    double Z_q[3] = {915017.5, -1814982.5, 900000};
    double delta_omega[2] = {0.0, 0.0};
    double omega = 0.0;
    double we = 14480.0;
    // X-Y paremeters
    double XY_T = 1/100;
    double x_error_signal[3] = {0.0, 0.0, 0.0};
    double y_error_signal[3] = {0.0, 0.0, 0.0};
    double X_q[3] = {1.0, -1.0, 0.0};
    double Y_q[3] = {1.0, -1.0, 0.0};
    double uc[2] = {0.0, 0.0};
    double vc[2] = {0.0, 0.0};
    double u_feedback[2] = {0.0, 0.0};
    double v_feedback[2] = {0.0, 0.0};
    double u_error_signal[3] = {0.0, 0.0, 0.0};
    double v_error_signal[3] = {0.0, 0.0, 0.0};
    double U_q[3] = {30.0, -29.99, 0.0};
    double V_q[3] = {-30.0, 29.99, 0.0};
    double pitch_ref[2] = {0.0, 0.0};
    double roll_ref[2] = {0.0, 0.0};
    // Yaw Controller
    double yaw_T = 1/100;
    double yaw_error_signal[3] = {0.0, 0.0, 0.0};
    double Yaw_q[3] = {3.0, -3.0, 0.0};
    double dyaw_ref[2] = {0.0, 0.0};
};
