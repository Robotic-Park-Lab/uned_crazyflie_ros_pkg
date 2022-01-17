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
#include <geometry_msgs/Twist.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <uned_crazyflie_controllers/AttitudeRefs.h>

class CrazyflieDriverSim
{
   public:
	CrazyflieDriverSim() {}

	/**
	 * NodeHandle is the main access point to communications with the ROS
	 * system. The first NodeHandle constructed will fully initialize this node,
	 * and the last NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle m_nh{};
	ros::NodeHandle m_nh_params{"~"};

	ros::Publisher m_pub_cmdcontrol, m_pub_control_signal, m_pub_omega, m_pub_dyaw;

	ros::Subscriber m_sub_onboard, m_sub_cmd_motors;

	bool initialize();

	bool iterate();

   protected:

     void onboardCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
     void cmdcontrolCallback(const std_msgs::Float64MultiArray::ConstPtr& msg);
     void rotorvelocitiesCallback(const Eigen::Vector4d rotor_velocities);
     void attitudeRateMixerRefsCallback(const double omega, const double pitch, const double roll, const double yaw);


     std_msgs::String m_controller_type, m_robot_id, m_controller_mode;
     Eigen::Vector4d ref_rotor_velocities;
     std_msgs::Float64MultiArray m_cmd_motors;

     bool onboard = false;
     bool motors = false;
     double thrust, roll, pitch, yaw;

};
