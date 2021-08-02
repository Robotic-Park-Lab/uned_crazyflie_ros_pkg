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
#include <uned_crazyflie_controllers/RateMixerRefs.h>

class CrazyflieAttitudeController
{
   public:
	CrazyflieAttitudeController() {}

	/**
	 * NodeHandle is the main access point to communications with the ROS
	 * system. The first NodeHandle constructed will fully initialize this node,
	 * and the last NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle m_nh{};
	ros::NodeHandle m_nh_params{"~"};

	ros::Publisher m_pub_control_signal;

	ros::Subscriber m_sub_GT_pose, m_sub_attitude_ref;

	bool initialize();

	bool iterate();

   protected:
    void gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void rotorvelocitiesCallback(const Eigen::Vector4d rotor_velocities);
    void attitudeRefsCallback(const uned_crazyflie_controllers::AttitudeRefs::ConstPtr& msg);
    void rateMixerRefsCallback(const double omega, const double dpitch, const double droll, const double dyaw);

    std::string m_controller_type, m_robot_id, m_controller_mode;
    geometry_msgs::Pose m_GT_pose;

    double omega, pitch_ref, roll_ref, dyaw, pitch_dron, roll_dron;

    double pitch_error[3], dpitch[2], Phi_q[3];

    double roll_error[3], droll[2], Theta_q[3];
};
