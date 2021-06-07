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
    geometry_msgs::Pose m_GT_pose, m_ref_pose, m_ref_position, m_ref_test;

    // Controllers
    double Z_q[3], X_q[3], Y_q[3], U_q[3], V_q[3], Yaw_q[3];
    // Altitude paremeters
    double z_error_signal[3], delta_omega[2];
    double omega = 0.0;
    double we = 16073.0; //14480.0;
    // X-Y paremeters
    double x_error_signal[3], y_error_signal[3], uc[2], vc[2], u_feedback[2], v_feedback[2];
    double u_error_signal[3], v_error_signal[3], pitch_ref[2], roll_ref[2];
    // Yaw Controller
    double yaw_error_signal[3], dyaw_ref[2];
};
