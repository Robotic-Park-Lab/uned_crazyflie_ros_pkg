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
    Eigen::Vector4d ref_rotor_velocities;

    double omega, dpitch_ref, droll_ref, dyaw_ref, dpitch_dron, droll_dron, dyaw_dron;
    const double PI = 3.14159265;
    double fm = 0.2685;
    double C = (4070.3*fm)*PI/30;
    double pitch_dron[2], roll_dron[2], yaw_dron[2];

    double dpitch_error[3],dpitch[2], Dphi_q[3];

    double droll_error[3], droll[2], Dtheta_q[3];

    double dyaw_error[3], dyaw[2], Dpsi_q[3];
};
