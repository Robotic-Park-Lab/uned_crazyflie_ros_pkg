#pragma once

#include <ros/ros.h>
#include <array>
#include <cstring>
#include <iostream>
#include <Eigen/Eigen>
#include <ros/console.h>
#include <mav_msgs/default_topics.h>
#include <sensor_msgs/Imu.h>
#include <time.h>
#include <chrono>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <mav_msgs/Actuators.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

class CrazyflieController
{
   public:
	CrazyflieController() {}

	/**
	 * NodeHandle is the main access point to communications with the ROS
	 * system. The first NodeHandle constructed will fully initialize this node,
	 * and the last NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle m_nh{};
	ros::NodeHandle m_nh_params{"~"};

	ros::Publisher m_pub_control_signal, m_pub_motor_velocity_reference;

	ros::Subscriber m_sub_eje_x, m_sub_eje_y, m_sub_GT_pose;

	bool initialize();

	/** called when work is to be done */
	bool iterate();

   protected:
	void ejexCallback(const std_msgs::Float64::ConstPtr& msg);
	void ejeyCallback(const std_msgs::Float64::ConstPtr& msg);
    void gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void rotorvelocitiesCallback(const Eigen::Vector4d rotor_velocities);
    void readTrajectory(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);

    double m_joy_x{.0}, m_joy_y{.0}, speed{.0};
    std::string m_controller_type, m_robot_id, m_controller_mode;
    geometry_msgs::Pose m_GT_pose, m_ref_pose, m_error_pose;

    int step{0};


};
