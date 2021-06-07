#pragma once

#include <ros/ros.h>
#include <array>
#include <cstring>
#include <iostream>
#include <Eigen/Eigen>
#include <ros/console.h>
#include <mav_msgs/conversions.h>
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
#include <geometry_msgs/Pose.h>

class CrazyflieTrajectoryController
{
   public:
	CrazyflieTrajectoryController() {}

	/**
	 * NodeHandle is the main access point to communications with the ROS
	 * system. The first NodeHandle constructed will fully initialize this node,
	 * and the last NodeHandle destructed will close down the node.
	 */
	ros::NodeHandle m_nh{};
	ros::NodeHandle m_nh_params{"~"};

	ros::Publisher m_pub_trayectory;

	ros::Subscriber m_sub_GT_pose;

	bool initialize();

	/** called when work is to be done */
	bool iterate();

   protected:
    void gtposeCallback(const geometry_msgs::Pose::ConstPtr& msg);
    void readTrajectory(const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);
    void trajectoryCallback(const Eigen::Vector3d position, double yaw);

    std::string m_controller_type, m_robot_id, m_controller_mode;
    geometry_msgs::Pose m_GT_pose;



};
