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
#include <uned_crazyflie_controllers/RateMixerRefs.h>

class CrazyflieRateMixerController
{
   public:
	CrazyflieRateMixerController() {}

	/**
	 * NodeHandle is the main access point to communications with the ROS
	 * system. The first NodeHandle constructed will fully initialize this node,
	 * and the last NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle m_nh{};
	ros::NodeHandle m_nh_params{"~"};

	ros::Publisher m_pub_motor_velocity_reference;

	ros::Subscriber m_sub_GT_pose, m_sub_ratemixer_ref;

	bool initialize();

	bool iterate();

   protected:
    void gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void rotorvelocitiesCallback(const Eigen::Vector4d rotor_velocities);
    void rateMixerRefsCallback(const uned_crazyflie_controllers::RateMixerRefs::ConstPtr& msg);

    std::string m_controller_type, m_robot_id, m_controller_mode;
    geometry_msgs::Pose m_GT_pose;

    double omega, dpitch_ref, droll_ref, dyaw_ref, dpitch_dron, droll_dron, dyaw_dron;
    double pitch_dron[2] = {0.0,0.0};
    double roll_dron[2] = {0.0,0.0};
    double yaw_dron[2] = {0.0,0.0};

    double dpitch_error[3] = {0.0,0.0,0.0};
    double dpitch[2] = {0.0,0.0};
    double Dphi_q[3] = {70.0,-70.0,0.0};

    double droll_error[3] = {0.0,0.0,0.0};
    double droll[2] = {0.0,0.0};
    double Dtheta_q[3] = {70.0,-70.0,0.0};

    double dyaw_error[3] = {0.0,0.0,0.0};
    double dyaw[2] = {0.0,0.0};
    double Dpsi_q[3] = {70.0167,-69.9833,0.0};
};
